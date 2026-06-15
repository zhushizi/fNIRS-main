"""从全会话原始数据构建 live_analysis_batch（全量重发）。"""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional

import numpy as np
import pandas as pd

from .rso2 import compute_rso2_series

from .config import OnlineSettings
from .types import LiveAnalysisBatch

PrepareInterleavedFn = Callable[[pd.DataFrame, bool], pd.DataFrame]
CalculateSeriesFn = Callable[
    [pd.DataFrame], Optional[tuple[np.ndarray, np.ndarray, np.ndarray]]
]


class IncrementalBatchBuilder:
    """
    全会话 interleaved + MBLL；仅当配对行数增加（或 force）时重算，
    每次向安卓发送【整段】曲线并置 replace_full_series=true。
    """

    def __init__(
        self,
        settings: OnlineSettings,
        *,
        prepare_interleaved: PrepareInterleavedFn,
        calculate_series: CalculateSeriesFn,
    ) -> None:
        self.settings = settings
        self._prepare_interleaved = prepare_interleaved
        self._calculate_series = calculate_series
        self._last_interleaved_count = 0

    def try_build(
        self,
        raw_df: pd.DataFrame,
        *,
        force_recompute: bool = False,
    ) -> LiveAnalysisBatch | None:
        if raw_df.empty:
            return None

        interleaved_df = self._prepare_interleaved(raw_df, start_at_zero=True)
        n_interleaved = len(interleaved_df)
        if interleaved_df.empty or n_interleaved < self.settings.min_interleaved_points:
            return None
        if not force_recompute and n_interleaved <= self._last_interleaved_count:
            return None

        series = self._calculate_series(interleaved_df)
        self._last_interleaved_count = n_interleaved
        if series is None:
            return None
        times, hbo, hbr = series
        n = min(len(times), len(hbo), len(hbr))
        if n == 0:
            return None
        times = times[:n]
        hbo = hbo[:n]
        hbr = hbr[:n]

        rso2_full, baseline_ready, latest_rso2_pct = self._build_rso2_full(times, hbo, hbr)
        session_times = interleaved_df["Time (s)"].to_numpy(dtype=float)
        return LiveAnalysisBatch(
            times=[float(x) for x in times],
            hbo=[float(x) for x in hbo],
            hbr=[float(x) for x in hbr],
            window_start_s=float(session_times[0]),
            window_end_s=float(session_times[-1]),
            baseline_ready=baseline_ready,
            rso2=rso2_full,
            latest_rso2_pct=latest_rso2_pct,
            baseline_rso2_pct=(
                self.settings.rso2_baseline_rso2_pct if baseline_ready else None
            ),
            replace_full_series=True,
        )

    def build_from_raw(self, raw_df: pd.DataFrame) -> LiveAnalysisBatch | None:
        """终算落盘：忽略配对计数门闩，对当前原始数据做一次全会话分析。"""
        return self.try_build(raw_df, force_recompute=True)

    def _build_rso2_full(
        self,
        times: np.ndarray,
        hbo: np.ndarray,
        hbr: np.ndarray,
    ) -> tuple[list[float | None] | None, bool, float | None]:
        rso2_series = compute_rso2_series(
            times,
            hbo,
            hbr,
            baseline_start_s=self.settings.rso2_baseline_start_s,
            baseline_end_s=self.settings.rso2_baseline_end_s,
            baseline_hbt_uM=self.settings.rso2_baseline_hbt_uM,
            baseline_rso2_pct=self.settings.rso2_baseline_rso2_pct,
        )
        if rso2_series is None:
            return None, False, None

        rso2_full = [float(v) if np.isfinite(v) else None for v in rso2_series]
        finite_rso2 = rso2_series[np.isfinite(rso2_series)]
        latest = float(finite_rso2[-1]) if finite_rso2.size > 0 else None
        return rso2_full, True, latest
