"""将发往安卓的 live_analysis_batch 保存为 processed_output 同款 CSV。"""

from __future__ import annotations

import csv
from collections.abc import Callable
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd

from config import OUTPUT_CHANNEL, processed_column_names

from .types import LiveAnalysisBatch

PrepareInterleavedFn = Callable[[pd.DataFrame, bool], pd.DataFrame]
CalculateSeriesFn = Callable[
    [pd.DataFrame], Optional[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]]
]

HBO_COL, HBR_COL, CYT_COL = processed_column_names(OUTPUT_CHANNEL)
PROCESSED_HEADER = ["Time", HBO_COL, HBR_COL, CYT_COL]


class AndroidLiveOutputRecorder:
    """保存在线全量曲线；采集结束时可用全会话 raw 终算覆盖。"""

    def __init__(self, output_path: str | Path | None) -> None:
        self.output_path = Path(output_path) if output_path else None
        self._times: list[float] = []
        self._hbo: list[float] = []
        self._hbr: list[float] = []
        self._cyt: list[float] = []

    @property
    def sample_count(self) -> int:
        return len(self._times)

    def set_series(
        self,
        times: list[float] | np.ndarray,
        hbo: list[float] | np.ndarray,
        hbr: list[float] | np.ndarray,
        cyt: list[float] | np.ndarray | None = None,
    ) -> None:
        self._times = [float(x) for x in times]
        self._hbo = [float(x) for x in hbo]
        self._hbr = [float(x) for x in hbr]
        if cyt is not None:
            self._cyt = [float(x) for x in cyt]

    def append_batch(self, batch: LiveAnalysisBatch) -> None:
        if batch.replace_full_series:
            self.set_series(batch.times, batch.hbo, batch.hbr, batch.cyt)
            return
        if self.output_path is None:
            return
        for time_s, hbo, hbr, cyt in zip(batch.times, batch.hbo, batch.hbr, batch.cyt):
            self._times.append(float(time_s))
            self._hbo.append(float(hbo))
            self._hbr.append(float(hbr))
            self._cyt.append(float(cyt))

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

        from fnirs_pipeline.mbll import compute_all_channel_concentrations, select_output_series

        computed = compute_all_channel_concentrations(interleaved_df)
        if computed is None:
            series = calculate_series(interleaved_df)
            if series is None:
                return False
            times, hbo, hbr, cyt = series
            n = min(len(times), len(hbo), len(hbr), len(cyt))
            if n == 0:
                return False
            self.set_series(times[:n], hbo[:n], hbr[:n], cyt[:n])
            return True

        times, all_results = computed
        selected = select_output_series(times, all_results, OUTPUT_CHANNEL)
        if selected is None:
            return False
        times, hbo, hbr, cyt = selected
        self.set_series(times, hbo, hbr, cyt)
        return True

    def flush(self) -> str | None:
        if self.output_path is None or not self._times:
            return None

        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.output_path, "w", newline="", encoding="utf-8") as f_out:
            writer = csv.writer(f_out)
            writer.writerow(PROCESSED_HEADER)
            n = len(self._times)
            for idx in range(n):
                cyt_val = self._cyt[idx] if idx < len(self._cyt) else ""
                writer.writerow([self._times[idx], self._hbo[idx], self._hbr[idx], cyt_val])

        path_str = str(self.output_path)
        print(
            f"Android live output ({OUTPUT_CHANNEL}) saved to '{path_str}' "
            f"({len(self._times)} samples, aligned with processed_output.csv)."
        )
        return path_str
