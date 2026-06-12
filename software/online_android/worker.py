"""在线分析后台线程。"""

from __future__ import annotations

import threading
from typing import TYPE_CHECKING

from .batch_builder import IncrementalBatchBuilder
from .buffer import OnlineSampleBuffer
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
        batch_builder: IncrementalBatchBuilder,
        settings: OnlineSettings | None = None,
        reporter: AndroidReporter | None = None,
    ) -> None:
        self.buffer = buffer
        self.settings = settings or OnlineSettings()
        self.batch_builder = batch_builder
        self.reporter = reporter or AndroidReporter(bridge, self.settings)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(
                timeout=max(1.0, self.settings.update_interval_seconds * 2.0),
            )

    def _run(self) -> None:
        print(
            "Online analysis enabled: "
            f"window={self.settings.window_seconds:.1f}s "
            f"interval={self.settings.update_interval_seconds:.1f}s."
        )
        while not self.stop_event.wait(self.settings.update_interval_seconds):
            if getattr(self.reporter.bridge, "closed", False):
                break
            try:
                self._analyze_and_send_once()
            except Exception as exc:
                print(f"Online analysis skipped one batch: {exc}")

    def _analyze_and_send_once(self) -> None:
        raw_df = self.buffer.snapshot_recent(self.settings.window_seconds)
        batch = self.batch_builder.try_build(raw_df)
        if batch is None:
            return
        self.reporter.send_live_batch(batch)
