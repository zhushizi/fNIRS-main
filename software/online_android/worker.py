"""在线分析后台线程。"""

from __future__ import annotations

import threading
from typing import TYPE_CHECKING, Any

from .buffer import OnlineSampleBuffer
from .channel_dispatcher import ChannelDispatcher
from .config import OnlineSettings
from .reporter import AndroidReporter

if TYPE_CHECKING:
    from .tcp_bridge import HostTcpSerialBridge


class OnlineAnalysisWorker:
    """定时从缓冲取全会话 → 按通道构建批次 → 分别回传安卓。"""

    def __init__(
        self,
        buffer: OnlineSampleBuffer,
        bridge: HostTcpSerialBridge,
        dispatcher: ChannelDispatcher,
        settings: OnlineSettings | None = None,
        reporter: AndroidReporter | None = None,
    ) -> None:
        self.buffer = buffer
        self.settings = settings or OnlineSettings()
        self.dispatcher = dispatcher
        self.reporter = reporter or AndroidReporter(bridge, self.settings)
        self.stop_event = threading.Event()
        self._baseline_lock = threading.Lock()
        self._last_sent_channels: set[int] = set()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread.is_alive():
            try:
                self._analyze_and_send_once(force_recompute=True)
            except Exception as exc:
                print(f"Online analysis final batch skipped: {exc}")
            self.thread.join(
                timeout=max(1.0, self.settings.update_interval_seconds * 2.0),
            )

    def _run(self) -> None:
        print(
            "Online analysis enabled: "
            "per-channel full-session interleaved, "
            f"interval={self.settings.update_interval_seconds:.1f}s."
        )
        while not self.stop_event.wait(self.settings.update_interval_seconds):
            if getattr(self.reporter.bridge, "closed", False):
                break
            try:
                self._analyze_and_send_once()
            except Exception as exc:
                print(f"Online analysis skipped one batch: {exc}")

    def _analyze_and_send_once(self, *, force_recompute: bool = False) -> None:
        raw_df = self.buffer.snapshot_all()
        batches = self.dispatcher.build_all(raw_df, force_recompute=force_recompute)
        sent: set[int] = set()
        for channel_code, batch in batches:
            try:
                self.reporter.send_live_batch(channel_code, batch)
                sent.add(channel_code)
            except Exception as exc:
                # 单个通道回传失败不影响其它通道。
                print(f"[online][channel {channel_code}] send failed: {exc}")
        if sent != self._last_sent_channels:
            print(f"[online] live batch channels now sending -> {sorted(sent)}")
            self._last_sent_channels = sent

    def handle_set_baseline(self, body: dict[str, Any]) -> dict[str, Any]:
        """安卓 set_baseline：更新 BL 并对每个通道全会话重算后分别回传 live_analysis_batch。"""
        rso2_raw = body.get("baseline_rso2_pct")
        if rso2_raw is None:
            return {"ok": False, "message": "baseline_rso2_pct is required"}

        try:
            rso2_pct = float(rso2_raw)
        except (TypeError, ValueError):
            return {"ok": False, "message": "baseline_rso2_pct must be a number"}

        if not (1.0 <= rso2_pct <= 99.0):
            return {
                "ok": False,
                "message": "baseline_rso2_pct must be between 1 and 99",
            }

        hbt_uM: float | None = None
        if "baseline_hbt_uM" in body and body["baseline_hbt_uM"] is not None:
            try:
                hbt_uM = float(body["baseline_hbt_uM"])
            except (TypeError, ValueError):
                return {"ok": False, "message": "baseline_hbt_uM must be a number"}
            if not (10.0 <= hbt_uM <= 200.0):
                return {
                    "ok": False,
                    "message": "baseline_hbt_uM must be between 10 and 200",
                }

        with self._baseline_lock:
            bl_pct, bl_hbt = self.dispatcher.update_baseline(rso2_pct, hbt_uM)
            raw_df = self.buffer.snapshot_all()
            results = self.dispatcher.apply_baseline_and_rebuild_all(raw_df)

        for channel_code, batch in results:
            self.reporter.send_live_batch(channel_code, batch)

        recomputed = len(results) > 0
        baseline_ready = any(batch.baseline_ready for _, batch in results)
        total_samples = sum(len(batch.times) for _, batch in results)
        result: dict[str, Any] = {
            "ok": True,
            "baseline_rso2_pct": bl_pct,
            "baseline_hbt_uM": bl_hbt,
            "recomputed": recomputed,
            "baseline_ready": baseline_ready,
            "sample_count": total_samples,
            "channels": [int(code) for code, _ in results],
        }
        if not recomputed:
            result["message"] = "insufficient samples for recompute"
        return result
