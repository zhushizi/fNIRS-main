"""从原始窗口数据构建增量 live_analysis_batch。"""

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
    维护已发送时间点与累积 HbO/HbR，在滑动窗口上重算但只输出新增点。

    核心分析函数由外部注入（通常来自 fNIRS_processing），保证与离线流程一致。
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
        self.last_sent_time = -float("inf")
        self._cum_times: list[float] = []
        self._cum_hbo: list[float] = []
        self._cum_hbr: list[float] = []

    def try_build(self, raw_df: pd.DataFrame) -> LiveAnalysisBatch | None:
        if raw_df.empty:
            return None

        window_span = float(raw_df["Time (s)"].iloc[-1] - raw_df["Time (s)"].iloc[0])
        min_span = max(
            0.0,
            self.settings.window_seconds - self.settings.update_interval_seconds * 0.5,
        )
        if window_span < min_span:
            return None

        interleaved_df = self._prepare_interleaved(raw_df, start_at_zero=False)
        if interleaved_df.empty or len(interleaved_df) < self.settings.min_interleaved_points:
            return None

        series = self._calculate_series(interleaved_df)
        if series is None:
            return None
        times, hbo, hbr = series

        new_mask = times > self.last_sent_time + 1e-9
        if not np.any(new_mask):
            return None

        times_new = times[new_mask]
        hbo_new = hbo[new_mask]
        hbr_new = hbr[new_mask]
        self.last_sent_time = float(times_new[-1])
        self._cum_times.extend(float(x) for x in times_new)
        self._cum_hbo.extend(float(x) for x in hbo_new)
        self._cum_hbr.extend(float(x) for x in hbr_new)

        rso2_new, baseline_ready, latest_rso2_pct = self._build_rso2_slice(len(times_new))
        return LiveAnalysisBatch(
            times=[float(x) for x in times_new],
            hbo=[float(x) for x in hbo_new],
            hbr=[float(x) for x in hbr_new],
            window_start_s=float(interleaved_df["Time (s)"].iloc[0]),
            window_end_s=float(interleaved_df["Time (s)"].iloc[-1]),
            baseline_ready=baseline_ready,
            rso2=rso2_new,
            latest_rso2_pct=latest_rso2_pct,
            baseline_rso2_pct=(
                self.settings.rso2_baseline_rso2_pct if baseline_ready else None
            ),
        )

    def _build_rso2_slice(
        self,
        n_new: int,
    ) -> tuple[list[float | None] | None, bool, float | None]:
        rso2_series = compute_rso2_series(
            np.asarray(self._cum_times, dtype=float),
            np.asarray(self._cum_hbo, dtype=float),
            np.asarray(self._cum_hbr, dtype=float),
            baseline_start_s=self.settings.rso2_baseline_start_s,
            baseline_end_s=self.settings.rso2_baseline_end_s,
            baseline_hbt_uM=self.settings.rso2_baseline_hbt_uM,
            baseline_rso2_pct=self.settings.rso2_baseline_rso2_pct,
        )
        if rso2_series is None:
            return None, False, None

        rso2_slice = rso2_series[-n_new:]
        rso2_new = [float(v) if np.isfinite(v) else None for v in rso2_slice]
        finite_rso2 = rso2_series[np.isfinite(rso2_series)]
        latest = float(finite_rso2[-1]) if finite_rso2.size > 0 else None
        return rso2_new, True, latest
