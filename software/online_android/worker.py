"""在线分析后台线程。"""

from __future__ import annotations

import threading
from typing import TYPE_CHECKING, Any

from .batch_builder import IncrementalBatchBuilder
from .buffer import OnlineSampleBuffer
from .causal_processor import IncrementalCausalProcessor
from .config import OnlineSettings
from .reporter import AndroidReporter

if TYPE_CHECKING:
    from .tcp_bridge import HostTcpSerialBridge


class OnlineAnalysisWorker:
    """定时从缓冲取窗口 → 构建批次 → 回传安卓。"""

    def __init__(
        self,
        buffer: OnlineSampleBuffer,
        bridge: HostTcpSerialBridge,
        batch_builder: IncrementalBatchBuilder | IncrementalCausalProcessor,
        settings: OnlineSettings | None = None,
        reporter: AndroidReporter | None = None,
    ) -> None:
        self.buffer = buffer
        self.settings = settings or OnlineSettings()
        self.batch_builder = batch_builder
        self.reporter = reporter or AndroidReporter(bridge, self.settings)
        self.stop_event = threading.Event()
        self._baseline_lock = threading.Lock()
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
            "full-session interleaved, full-series replace, "
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
        batch = self.batch_builder.try_build(raw_df, force_recompute=force_recompute)
        if batch is None:
            return
        self.reporter.send_live_batch(batch)

    def handle_set_baseline(self, body: dict[str, Any]) -> dict[str, Any]:
        """安卓 set_baseline：更新 BL 并重算当前全会话后回传 live_analysis_batch。"""
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
            bl_pct, bl_hbt = self.batch_builder.update_baseline(rso2_pct, hbt_uM)
            raw_df = self.buffer.snapshot_all()
            batch = self.batch_builder.apply_baseline_and_rebuild(raw_df)

        if batch is not None:
            self.reporter.send_live_batch(batch)

        result: dict[str, Any] = {
            "ok": True,
            "baseline_rso2_pct": bl_pct,
            "baseline_hbt_uM": bl_hbt,
            "recomputed": batch is not None,
            "baseline_ready": batch.baseline_ready if batch is not None else False,
            "sample_count": len(batch.times) if batch is not None else 0,
        }
        if batch is None:
            result["message"] = "insufficient samples for recompute"
        return result

