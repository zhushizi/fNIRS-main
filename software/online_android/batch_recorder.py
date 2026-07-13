"""将发往安卓的 live_analysis_batch 保存为 processed_output 同款 CSV。"""

from __future__ import annotations

import csv
from collections.abc import Callable
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd

from config import OUTPUT_CHANNEL, processed_column_names

from .logging_utils import get_logger
from .types import LiveAnalysisBatch, compute_delta_thb

log = get_logger("recorder")

PrepareInterleavedFn = Callable[[pd.DataFrame, bool], pd.DataFrame]
CalculateSeriesFn = Callable[
    [pd.DataFrame], Optional[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]]
]

HBO_COL, HBR_COL, CYT_COL = processed_column_names(OUTPUT_CHANNEL)
DTHB_COL = f"{OUTPUT_CHANNEL}_dthb"
THI_COL = f"{OUTPUT_CHANNEL}_thi"
DTHI_COL = f"{OUTPUT_CHANNEL}_dthi"
PROCESSED_HEADER = ["Time", HBO_COL, HBR_COL, CYT_COL, DTHB_COL, THI_COL, DTHI_COL]


class AndroidLiveOutputRecorder:
    """
    保存实际回传安卓的在线曲线（replace 整段重置、append 追加），
    采集结束直接 flush，落盘内容与安卓所见一致。

    `flush_from_raw`（用离线全会话终算覆盖）保留备用，但默认不再调用，
    以免覆盖掉真实回传的数据。
    """

    def __init__(self, output_path: str | Path | None) -> None:
        self.output_path = Path(output_path) if output_path else None
        self._times: list[float] = []
        self._hbo: list[float] = []
        self._hbr: list[float] = []
        self._cyt: list[float] = []
        # tHi / ΔtHi 无法从 hbo/hbr 重算（依赖冻结基线），故直接存回传值；
        # 热身期批次不带 tHi，对应位置存 None，落盘写空。
        self._thi: list[float | None] = []
        self._delta_thi: list[float | None] = []

    @property
    def sample_count(self) -> int:
        return len(self._times)

    def set_series(
        self,
        times: list[float] | np.ndarray,
        hbo: list[float] | np.ndarray,
        hbr: list[float] | np.ndarray,
        cyt: list[float] | np.ndarray | None = None,
        thi: list[float] | np.ndarray | None = None,
        delta_thi: list[float] | np.ndarray | None = None,
    ) -> None:
        self._times = [float(x) for x in times]
        self._hbo = [float(x) for x in hbo]
        self._hbr = [float(x) for x in hbr]
        if cyt is not None:
            self._cyt = [float(x) for x in cyt]
        n = len(self._times)
        self._thi = [float(x) for x in thi] if thi is not None else [None] * n
        self._delta_thi = [float(x) for x in delta_thi] if delta_thi is not None else [None] * n

    def append_batch(self, batch: LiveAnalysisBatch) -> None:
        if batch.replace_full_series:
            self.set_series(batch.times, batch.hbo, batch.hbr, batch.cyt, batch.thi, batch.delta_thi)
            return
        if self.output_path is None:
            return
        count = len(batch.times)
        thi_vals = list(batch.thi) if batch.thi is not None else [None] * count
        dthi_vals = list(batch.delta_thi) if batch.delta_thi is not None else [None] * count
        for idx, (time_s, hbo, hbr, cyt) in enumerate(
            zip(batch.times, batch.hbo, batch.hbr, batch.cyt)
        ):
            self._times.append(float(time_s))
            self._hbo.append(float(hbo))
            self._hbr.append(float(hbr))
            self._cyt.append(float(cyt))
            thi_v = thi_vals[idx] if idx < len(thi_vals) else None
            dthi_v = dthi_vals[idx] if idx < len(dthi_vals) else None
            self._thi.append(float(thi_v) if thi_v is not None else None)
            self._delta_thi.append(float(dthi_v) if dthi_v is not None else None)

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
            dthb_series = compute_delta_thb(self._hbo, self._hbr)
            for idx in range(n):
                cyt_val = self._cyt[idx] if idx < len(self._cyt) else ""
                dthb_val = dthb_series[idx] if idx < len(dthb_series) else ""
                thi_val = self._thi[idx] if idx < len(self._thi) and self._thi[idx] is not None else ""
                dthi_val = (
                    self._delta_thi[idx]
                    if idx < len(self._delta_thi) and self._delta_thi[idx] is not None
                    else ""
                )
                writer.writerow(
                    [self._times[idx], self._hbo[idx], self._hbr[idx], cyt_val, dthb_val, thi_val, dthi_val]
                )

        path_str = str(self.output_path)
        log.info(
            "Android live output (%s) saved to '%s' (%d samples, exactly as streamed to Android).",
            OUTPUT_CHANNEL,
            path_str,
            len(self._times),
        )
        return path_str
