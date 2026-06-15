"""将发往安卓的 live_analysis_batch 保存为 processed_output 同款 CSV。"""

from __future__ import annotations

import csv
from collections.abc import Callable
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd

from config import CHANNEL_NAME

from .types import LiveAnalysisBatch

HBO_COL = f"{CHANNEL_NAME}_hbo"
HBR_COL = f"{CHANNEL_NAME}_hbr"
PROCESSED_HEADER = ["Time", HBO_COL, HBR_COL]

PrepareInterleavedFn = Callable[[pd.DataFrame, bool], pd.DataFrame]
CalculateSeriesFn = Callable[
    [pd.DataFrame], Optional[tuple[np.ndarray, np.ndarray, np.ndarray]]
]


class AndroidLiveOutputRecorder:
    """保存在线全量曲线；采集结束时可用全会话 raw 终算覆盖。"""

    def __init__(self, output_path: str | Path | None) -> None:
        self.output_path = Path(output_path) if output_path else None
        self._times: list[float] = []
        self._hbo: list[float] = []
        self._hbr: list[float] = []

    @property
    def sample_count(self) -> int:
        return len(self._times)

    def set_series(
        self,
        times: list[float] | np.ndarray,
        hbo: list[float] | np.ndarray,
        hbr: list[float] | np.ndarray,
    ) -> None:
        self._times = [float(x) for x in times]
        self._hbo = [float(x) for x in hbo]
        self._hbr = [float(x) for x in hbr]

    def append_batch(self, batch: LiveAnalysisBatch) -> None:
        if batch.replace_full_series:
            self.set_series(batch.times, batch.hbo, batch.hbr)
            return
        if self.output_path is None:
            return
        for time_s, hbo, hbr in zip(batch.times, batch.hbo, batch.hbr):
            self._times.append(float(time_s))
            self._hbo.append(float(hbo))
            self._hbr.append(float(hbr))

    def flush_from_raw(
        self,
        raw_df: pd.DataFrame,
        *,
        prepare_interleaved: PrepareInterleavedFn,
        calculate_series: CalculateSeriesFn,
    ) -> bool:
        """采集结束时用全会话 raw 终算，保证与 processed_output.csv 一致。"""
        if self.output_path is None or raw_df.empty:
            return False

        interleaved_df = prepare_interleaved(raw_df, start_at_zero=True)
        if interleaved_df.empty:
            return False
        series = calculate_series(interleaved_df)
        if series is None:
            return False
        times, hbo, hbr = series
        n = min(len(times), len(hbo), len(hbr))
        if n == 0:
            return False
        self.set_series(times[:n], hbo[:n], hbr[:n])
        return True

    def flush(self) -> str | None:
        if self.output_path is None or not self._times:
            return None

        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.output_path, "w", newline="", encoding="utf-8") as f_out:
            writer = csv.writer(f_out)
            writer.writerow(PROCESSED_HEADER)
            for time_s, hbo, hbr in zip(self._times, self._hbo, self._hbr):
                writer.writerow([time_s, hbo, hbr])

        path_str = str(self.output_path)
        print(
            f"Android live HbO/HbR saved to '{path_str}' "
            f"({len(self._times)} samples, aligned with processed_output.csv)."
        )
        return path_str
