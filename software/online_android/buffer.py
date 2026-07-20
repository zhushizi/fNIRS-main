"""线程安全的在线原始采样缓冲。"""

from __future__ import annotations

import threading
from collections import deque

import pandas as pd


from .online_config import OnlineSettings


class OnlineSampleBuffer:
    """采集线程写入、在线分析线程按窗口读取。"""

    _COLUMNS = ["Time (s)", "ChannelId", "DetectorId", "Channel", "Wavelength", "Value"]

    def __init__(self, settings: OnlineSettings | None = None) -> None:
        cfg = settings or OnlineSettings()
        self.retention_seconds = cfg.buffer_retention_seconds
        self._rows: deque[tuple[float, int, int, str, int, float]] = deque()
        self._lock = threading.Lock()

    def append(
        self,
        elapsed_time: float,
        detector_id: int,
        value: float,
        wavelength_code: int,
        channel_name: str = "",
        acq_channel_code: int = 0x01,
    ) -> None:
        with self._lock:
            self._rows.append(
                (elapsed_time, acq_channel_code, detector_id, channel_name, wavelength_code, value)
            )
            if self.retention_seconds is not None:
                cutoff = elapsed_time - self.retention_seconds
                while self._rows and self._rows[0][0] < cutoff:
                    self._rows.popleft()

    def snapshot_all(self) -> pd.DataFrame:
        """返回本会话至今的全部原始采样（与离线 all_groups.csv 等价）。"""
        with self._lock:
            rows = list(self._rows)
        return pd.DataFrame(rows, columns=self._COLUMNS)

    def snapshot_recent(self, window_seconds: float) -> pd.DataFrame:
        with self._lock:
            if not self._rows:
                rows: list[tuple[float, int, int, str, int, float]] = []
            else:
                latest_time = self._rows[-1][0]
                cutoff = latest_time - window_seconds
                rows = [row for row in self._rows if row[0] >= cutoff]

        return pd.DataFrame(rows, columns=self._COLUMNS)
