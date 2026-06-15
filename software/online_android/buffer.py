"""线程安全的在线原始采样缓冲。"""

from __future__ import annotations

import threading
from collections import deque

import pandas as pd

from config import CHANNEL_NAME

from .config import OnlineSettings


class OnlineSampleBuffer:
    """采集线程写入、在线分析线程按窗口读取。"""

    def __init__(self, settings: OnlineSettings | None = None) -> None:
        cfg = settings or OnlineSettings()
        self.retention_seconds = cfg.buffer_retention_seconds
        self._rows: deque[tuple[float, int, float, int]] = deque()
        self._lock = threading.Lock()

    def append(
        self,
        elapsed_time: float,
        sensor_id: int,
        value: float,
        wavelength_code: int,
    ) -> None:
        with self._lock:
            self._rows.append((elapsed_time, sensor_id, value, wavelength_code))
            if self.retention_seconds is not None:
                cutoff = elapsed_time - self.retention_seconds
                while self._rows and self._rows[0][0] < cutoff:
                    self._rows.popleft()

    def snapshot_all(self) -> pd.DataFrame:
        """返回本会话至今的全部原始采样（与离线 all_groups.csv 等价）。"""
        with self._lock:
            rows = list(self._rows)
        return pd.DataFrame(
            rows,
            columns=["Time (s)", "SensorId", CHANNEL_NAME, "Wavelength"],
        )

    def snapshot_recent(self, window_seconds: float) -> pd.DataFrame:
        with self._lock:
            if not self._rows:
                rows: list[tuple[float, int, float, int]] = []
            else:
                latest_time = self._rows[-1][0]
                cutoff = latest_time - window_seconds
                rows = [row for row in self._rows if row[0] >= cutoff]

        return pd.DataFrame(
            rows,
            columns=["Time (s)", "SensorId", CHANNEL_NAME, "Wavelength"],
        )
