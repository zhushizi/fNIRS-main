"""在线分析后台线程。"""

from __future__ import annotations

import threading
from typing import TYPE_CHECKING, Any

from .buffer import OnlineSampleBuffer
from .channel_dispatcher import ChannelDispatcher
from .config import OnlineSettings
from .logging_utils import get_logger
from .reporter import AndroidReporter

if TYPE_CHECKING:
    from .tcp_bridge import HostTcpSerialBridge

log = get_logger("worker")


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
                log.warning("Online analysis final batch skipped: %s", exc)
            self.thread.join(
                timeout=max(1.0, self.settings.update_interval_seconds * 2.0),
            )

    def _run(self) -> None:
        log.info(
            "Online analysis enabled: mode=%s interval=%.1fs warmup=%.0fs "
            "baseline_window=%.0f-%.0fs.",
            self.settings.online_mode,
            self.settings.update_interval_seconds,
            self.settings.causal_warmup_seconds,
            self.settings.rso2_baseline_start_s,
            self.settings.rso2_baseline_end_s,
        )
        while not self.stop_event.wait(self.settings.update_interval_seconds):
            if getattr(self.reporter.bridge, "closed", False):
                break
            try:
                self._analyze_and_send_once()
            except Exception as exc:
                log.warning("Online analysis skipped one batch: %s", exc)

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
                log.warning("[channel %s] send failed: %s", channel_code, exc)
        if sent != self._last_sent_channels:
            log.info("live batch channels now sending -> %s", sorted(sent))
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

        log.info(
            "set_baseline received: baseline_rso2_pct=%s baseline_hbt_uM=%s "
            "(将对所有通道整段回填 replace_full_series=True)",
            rso2_pct,
            hbt_uM,
        )
        with self._baseline_lock:
            bl_pct, bl_hbt = self.dispatcher.update_baseline(rso2_pct, hbt_uM)
            raw_df = self.buffer.snapshot_all()
            results = self.dispatcher.apply_baseline_and_rebuild_all(raw_df)

        rebuilt = sorted(code for code, _ in results)
        log.info("set_baseline applied -> rebuilt+resent channels %s (each replace_full_series=True)", rebuilt)
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
