"""
路线 B：因果增量在线分析处理器。

与 IncrementalBatchBuilder（每 tick 对全会话非因果重算、整段重发）不同，
本处理器做到真正的增量与因果：

1. **增量配对**：自己维护流式分段 RMS + 五波长×双接收源凑组，
   每个原始样本只折叠一次，不再每 tick 对全会话重跑 prepare_interleaved；
2. **固定基准 I0**：用基线窗均值算 ΔOD，历史不随新数据漂移；
3. **因果 IIR 带通**：sosfilt + 持久 zi 状态替代零相位 sosfiltfilt，
   过去的滤波值不再被改写；
4. 短距回归 β、显示锚定基线、rSO₂ 基线在热身末端一次性冻结；
5. 热身完成后 **只追加新样本**（replace_full_series=False），单 tick 成本 O(新样本)。

热身阶段（累计跨度 < causal_warmup_seconds）用全量 calculate_series 提供
临时（未锚定）曲线（replace + baseline_ready=False）；到达热身末端做一次
整段回填（replace + ready），随后切到追加流。

注意：为保证因果与增量，本路径用「分段 RMS」直接代表每个波长时隙，
不再做离线那一步非因果 1Hz 低通，因此在线结果与离线 processed_output.csv
不再逐点一致——离线终算仍是权威结果。
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional

import numpy as np
import pandas as pd
from scipy.signal import sosfilt, sosfilt_zi

from config import (
    DETECTOR_CHANNELS,
    INTENSITY_COLUMNS,
    MBLL_DEFAULT_AGE,
    OUTPUT_CHANNEL,
    WAVELENGTH_CHANNELS,
    WAVELENGTH_OFF_CODE,
    detector_channel_by_code,
    intensity_column_for,
    mbll_wavelengths_nm,
    wavelength_channel_by_code,
)

from fnirs_pipeline.mbll_core import get_dpf, intensities_to_od_changes

from .baseline_state import MutableBaseline
from .online_config import OnlineSettings
from .types import LiveAnalysisBatch

PrepareInterleavedFn = Callable[[pd.DataFrame, bool], pd.DataFrame]
CalculateSeriesFn = Callable[
    [pd.DataFrame], Optional[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]]
]

_TIME_COL = "Time (s)"


def _detector_by_name(name: str):
    for det in DETECTOR_CHANNELS:
        if det.name == name:
            return det
    return None


class IncrementalCausalProcessor:
    """因果增量处理器，接口与 IncrementalBatchBuilder.try_build 保持一致。"""

    def __init__(
        self,
        settings: OnlineSettings,
        *,
        prepare_interleaved: PrepareInterleavedFn,  # 接口对齐，本路径不逐 tick 调用
        calculate_series: CalculateSeriesFn,
        output_channel: str = OUTPUT_CHANNEL,
        baseline: MutableBaseline | None = None,
    ) -> None:
        self.settings = settings
        self._calculate_series = calculate_series
        self._output_channel = output_channel
        self._baseline = baseline or MutableBaseline(
            rso2_pct=settings.rso2_baseline_rso2_pct,
            hbt_uM=settings.rso2_baseline_hbt_uM,
        )

        # 输出通道几何解析
        if output_channel.endswith("_ssr"):
            self._ssr = True
            long_name = output_channel[: -len("_ssr")]
        else:
            self._ssr = False
            long_name = output_channel
        self._long_det = _detector_by_name(long_name)
        self._short_det = (
            min(DETECTOR_CHANNELS, key=lambda d: d.distance_cm)
            if self._ssr and len(DETECTOR_CHANNELS) >= 2
            else None
        )
        if self._short_det is not None and self._short_det.name == long_name:
            self._ssr = False
            self._short_det = None
        self._distance_cm = float(self._long_det.distance_cm) if self._long_det else 3.0

        # --- 增量配对状态 ---
        self._raw_consumed = 0
        self._cur_key: tuple[int, int] | None = None
        self._cur_vals: list[float] = []
        self._cur_times: list[float] = []
        self._pending: dict[tuple[int, int], tuple[float, float]] = {}
        self._required_keys = {
            (det.code, wl.code) for det in DETECTOR_CHANNELS for wl in WAVELENGTH_CHANNELS
        }
        self._inter_rows: list[dict[str, float]] = []

        # --- 冻结状态（热身完成时填充）---
        self._ready = False
        self._processed_count = 0
        self._wavelengths: list[float] = []
        self._dpfs: list[float] = []
        self._ref_long: np.ndarray | None = None
        self._ref_short: np.ndarray | None = None
        self._beta: np.ndarray | None = None
        self._short_mean: np.ndarray | None = None
        self._sos: np.ndarray | None = None
        self._zi: np.ndarray | None = None
        self._baseline_conc: np.ndarray | None = None  # (3,) 摩尔，显示锚定
        self._rso2_hbo_abs = 0.0
        self._rso2_hbr_abs = 0.0
        self._rso2_dhbo_mean = 0.0
        self._rso2_dhbr_mean = 0.0

    # ------------------------------------------------------------------ public
    def try_build(
        self,
        raw_df: pd.DataFrame,
        *,
        force_recompute: bool = False,
    ) -> LiveAnalysisBatch | None:
        if self._long_det is None or raw_df.empty:
            return None

        self._ingest_raw(raw_df)
        n = len(self._inter_rows)
        if n < self.settings.min_interleaved_points:
            return None

        times = np.asarray([r[_TIME_COL] for r in self._inter_rows], dtype=float)
        span = float(times[-1] - times[0]) if n >= 2 else 0.0

        if not self._ready:
            if span < self.settings.causal_warmup_seconds:
                return self._provisional()
            self._freeze_baseline(times)
            return self._build_backfill(times)

        if n <= self._processed_count:
            return None
        return self._build_append(times)

    def update_baseline(
        self,
        rso2_pct: float,
        hbt_uM: float | None = None,
    ) -> tuple[float, float]:
        return self._baseline.update(rso2_pct, hbt_uM)

    def apply_baseline_and_rebuild(self, raw_df: pd.DataFrame) -> LiveAnalysisBatch | None:
        """安卓 set_baseline 后：更新 BL 假设并对全会话重算，整段回填安卓。"""
        if self._long_det is None:
            return None

        self._ingest_raw(raw_df)
        n = len(self._inter_rows)
        if n < self.settings.min_interleaved_points:
            return None

        times = np.asarray([r[_TIME_COL] for r in self._inter_rows], dtype=float)
        bl_pct, bl_hbt = self._baseline.snapshot()
        hbt_M = bl_hbt * 1e-6
        self._rso2_hbo_abs = hbt_M * (bl_pct / 100.0)
        self._rso2_hbr_abs = hbt_M - self._rso2_hbo_abs

        if not self._ready:
            self._freeze_baseline(times)
        return self._build_backfill(times)

    # --------------------------------------------------------- 增量配对
    def _ingest_raw(self, raw_df: pd.DataFrame) -> None:
        """把新到的原始样本折叠进流式分段 RMS / 凑组，必要时产出 interleaved 行。"""
        total = len(raw_df)
        if total <= self._raw_consumed:
            return
        new = raw_df.iloc[self._raw_consumed:]
        t = new[_TIME_COL].to_numpy(dtype=float)
        det = new["DetectorId"].to_numpy(dtype=int)
        wl = new["Wavelength"].to_numpy(dtype=int)
        val = new["Value"].to_numpy(dtype=float)
        for i in range(len(new)):
            self._fold_sample(t[i], int(det[i]), int(wl[i]), val[i])
        self._raw_consumed = total

    def _fold_sample(self, t: float, det: int, wl: int, val: float) -> None:
        if wl == WAVELENGTH_OFF_CODE:
            return  # 离线在分段前丢弃 OFF；这里等价为忽略，不打断当前段
        if wavelength_channel_by_code(wl) is None or detector_channel_by_code(det) is None:
            return
        key = (det, wl)
        if self._cur_key is None:
            self._cur_key, self._cur_vals, self._cur_times = key, [val], [t]
            return
        if key == self._cur_key:
            self._cur_vals.append(val)
            self._cur_times.append(t)
            return
        self._finalize_segment()
        self._cur_key, self._cur_vals, self._cur_times = key, [val], [t]

    def _finalize_segment(self) -> None:
        if self._cur_key is None or not self._cur_vals:
            return
        rms = float(np.sqrt(np.mean(np.square(self._cur_vals))))
        mean_t = float(np.mean(self._cur_times))
        if self._cur_key in self._pending:
            self._pending.clear()  # 同键重现且未凑齐：重同步
        self._pending[self._cur_key] = (rms, mean_t)
        if len(self._pending) == len(self._required_keys):
            self._emit_cycle_row()

    def _emit_cycle_row(self) -> None:
        mean_time = float(np.mean([v[1] for v in self._pending.values()]))
        row: dict[str, float] = {_TIME_COL: mean_time}
        for det in DETECTOR_CHANNELS:
            for wl in WAVELENGTH_CHANNELS:
                row[intensity_column_for(det.name, wl.emitter_nm)] = self._pending[
                    (det.code, wl.code)
                ][0]
        self._inter_rows.append(row)
        self._pending.clear()

    def _interleaved_df(self, start: int = 0, end: int | None = None) -> pd.DataFrame:
        rows = self._inter_rows[start:end]
        return pd.DataFrame(rows, columns=[_TIME_COL, *INTENSITY_COLUMNS])

    # --------------------------------------------------------------- warmup
    def _provisional(self) -> LiveAnalysisBatch | None:
        """热身期：用全量 calculate_series 出临时（未锚定）曲线，标记 not-ready。"""
        inter_df = self._interleaved_df()
        series = self._calculate_series(inter_df)
        if series is None:
            return None
        times, hbo, hbr, cyt = series
        n = min(len(times), len(hbo), len(hbr), len(cyt))
        if n == 0:
            return None
        scale = self.settings.conc_scale
        times = [float(x) for x in times[:n]]
        return LiveAnalysisBatch(
            times=times,
            hbo=[float(x) * scale for x in hbo[:n]],
            hbr=[float(x) * scale for x in hbr[:n]],
            cyt=[float(x) * scale for x in cyt[:n]],
            window_start_s=times[0],
            window_end_s=times[-1],
            baseline_ready=False,
            replace_full_series=True,
            unit=self.settings.conc_unit,
        )

    def _freeze_baseline(self, times: np.ndarray) -> None:
        """热身末端：一次性冻结 I0 / β / 滤波状态 / 锚定基线 / rSO₂ 基线。"""
        from fnirs_pipeline.mbll import butter_bandpass_sos, short_separation_regression
        from fnirs_pipeline.preprocessing import stack_intensities_for_detector

        diffs = np.diff(times)
        valid = diffs[np.isfinite(diffs) & (diffs > 0)]
        fs = 1.0 / float(np.mean(valid)) if valid.size else 1.0

        self._wavelengths = mbll_wavelengths_nm()
        self._dpfs = [get_dpf(wl, MBLL_DEFAULT_AGE) for wl in self._wavelengths]

        inter_df = self._interleaved_df()
        baseline_mask = (
            (times >= self.settings.rso2_baseline_start_s)
            & (times <= self.settings.rso2_baseline_end_s)
        )
        if not np.any(baseline_mask):
            baseline_mask = np.ones_like(times, dtype=bool)

        s_long = stack_intensities_for_detector(inter_df, self._long_det)
        self._ref_long = np.mean(np.abs(s_long[:, baseline_mask]), axis=1)
        long_od = intensities_to_od_changes(s_long, refs=self._ref_long)

        if self._ssr and self._short_det is not None:
            s_short = stack_intensities_for_detector(inter_df, self._short_det)
            self._ref_short = np.mean(np.abs(s_short[:, baseline_mask]), axis=1)
            short_od = intensities_to_od_changes(s_short, refs=self._ref_short)
            self._short_mean = np.mean(short_od, axis=1)
            _corrected, self._beta = short_separation_regression(long_od, short_od)
            corrected_all = long_od - self._beta[:, np.newaxis] * (
                short_od - self._short_mean[:, np.newaxis]
            )
        else:
            corrected_all = long_od

        self._sos = butter_bandpass_sos(
            self._bp_low_hz, self._bp_high_hz, fs, order=self._bp_order
        )
        if self._sos is not None:
            zi0 = (
                sosfilt_zi(self._sos)[:, np.newaxis, :]
                * corrected_all[:, 0][np.newaxis, :, np.newaxis]
            )
            filtered_all, self._zi = sosfilt(self._sos, corrected_all, axis=1, zi=zi0)
        else:
            filtered_all = corrected_all
            self._zi = None

        conc_all = self._mbll(filtered_all)  # (3, N) 摩尔
        self._baseline_conc = np.mean(conc_all[:, baseline_mask], axis=1)

        bl_pct, bl_hbt = self._baseline.snapshot()
        hbt_M = bl_hbt * 1e-6
        self._rso2_hbo_abs = hbt_M * (bl_pct / 100.0)
        self._rso2_hbr_abs = hbt_M - self._rso2_hbo_abs
        self._rso2_dhbo_mean = float(np.mean(conc_all[0, baseline_mask]))
        self._rso2_dhbr_mean = float(np.mean(conc_all[1, baseline_mask]))

        self._ready = True
        self._processed_count = 0

    # --------------------------------------------------------------- batches
    def _build_backfill(self, times: np.ndarray) -> LiveAnalysisBatch:
        """热身完成：对全部历史行做因果终值，整段回填（replace）。"""
        inter_df = self._interleaved_df()
        conc = self._concentrations(inter_df, reuse_zi=False)
        n = conc.shape[1]
        self._processed_count = n
        return self._make_batch([float(t) for t in times[:n]], conc, replace_full_series=True)

    def _build_append(self, times: np.ndarray) -> LiveAnalysisBatch:
        """流式阶段：只对新行做因果增量，仅追加。"""
        start = self._processed_count
        inter_df = self._interleaved_df(start=start)
        conc_new = self._concentrations(inter_df, reuse_zi=True)
        k = conc_new.shape[1]
        new_times = [float(t) for t in times[start:start + k]]
        self._processed_count = start + k
        return self._make_batch(new_times, conc_new, replace_full_series=False)

    def _concentrations(self, df: pd.DataFrame, *, reuse_zi: bool) -> np.ndarray:
        """对给定（整段或增量）interleaved 行计算 (3, N) 摩尔浓度。"""
        from fnirs_pipeline.preprocessing import stack_intensities_for_detector

        s_long = stack_intensities_for_detector(df, self._long_det)
        long_od = intensities_to_od_changes(s_long, refs=self._ref_long)
        if self._ssr and self._short_det is not None:
            s_short = stack_intensities_for_detector(df, self._short_det)
            short_od = intensities_to_od_changes(s_short, refs=self._ref_short)
            corrected = long_od - self._beta[:, np.newaxis] * (
                short_od - self._short_mean[:, np.newaxis]
            )
        else:
            corrected = long_od

        if self._sos is None:
            filtered = corrected
        elif reuse_zi and self._zi is not None:
            filtered, self._zi = sosfilt(self._sos, corrected, axis=1, zi=self._zi)
        else:
            zi0 = (
                sosfilt_zi(self._sos)[:, np.newaxis, :]
                * corrected[:, 0][np.newaxis, :, np.newaxis]
            )
            filtered, self._zi = sosfilt(self._sos, corrected, axis=1, zi=zi0)
        return self._mbll(filtered)

    def _mbll(self, delta_od: np.ndarray) -> np.ndarray:
        from fnirs_pipeline.mbll import generalized_mbll

        return generalized_mbll(delta_od, self._wavelengths, self._dpfs, self._distance_cm)

    def _make_batch(
        self,
        times: list[float],
        conc: np.ndarray,
        *,
        replace_full_series: bool,
    ) -> LiveAnalysisBatch:
        scale = self.settings.conc_scale
        anchor = self._baseline_conc if self.settings.baseline_anchor else np.zeros(3)
        hbo = (conc[0] - anchor[0]) * scale
        hbr = (conc[1] - anchor[1]) * scale
        cyt = (conc[2] - anchor[2]) * scale

        # tHi / ΔtHi 与 rSO₂ 共用同一组冻结基线的绝对 HbO/HbR，保证三者自洽。
        hbo_abs, hbr_abs = self._abs_hb(conc)
        hbt_M = hbo_abs + hbr_abs
        _, bl_hbt = self._baseline.snapshot()
        baseline_hbt_M = bl_hbt * 1e-6
        thi = hbt_M * scale                            # 绝对量，≈ 基线 HbT
        delta_thi = (hbt_M - baseline_hbt_M) * scale   # 相对基线，≈0 起算

        rso2 = self._rso2_from_abs(hbo_abs, hbr_abs)
        finite = [v for v in rso2 if v is not None and np.isfinite(v)]
        latest = float(finite[-1]) if finite else None
        bl_pct, _ = self._baseline.snapshot()

        return LiveAnalysisBatch(
            times=[float(t) for t in times],
            hbo=[float(v) for v in hbo],
            hbr=[float(v) for v in hbr],
            cyt=[float(v) for v in cyt],
            window_start_s=float(times[0]) if times else 0.0,
            window_end_s=float(times[-1]) if times else 0.0,
            baseline_ready=True,
            rso2=rso2,
            latest_rso2_pct=latest,
            baseline_rso2_pct=bl_pct,
            replace_full_series=replace_full_series,
            unit=self.settings.conc_unit,
            thi=[float(v) for v in thi],
            delta_thi=[float(v) for v in delta_thi],
        )

    def _abs_hb(self, conc: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """用冻结基线把 ΔHbO/ΔHbR（摩尔）转绝对 HbO/HbR（摩尔，下限截 0）。"""
        hbo_abs = np.maximum(self._rso2_hbo_abs + (conc[0] - self._rso2_dhbo_mean), 0.0)
        hbr_abs = np.maximum(self._rso2_hbr_abs + (conc[1] - self._rso2_dhbr_mean), 0.0)
        return hbo_abs, hbr_abs

    @staticmethod
    def _rso2_from_abs(hbo_abs: np.ndarray, hbr_abs: np.ndarray) -> list[float | None]:
        """由绝对 HbO/HbR（摩尔）算 rSO₂（%）。"""
        hbt = hbo_abs + hbr_abs
        with np.errstate(divide="ignore", invalid="ignore"):
            rso2 = np.where(hbt > 0, 100.0 * hbo_abs / hbt, np.nan)
        return [float(v) if np.isfinite(v) else None for v in rso2]

    # band-pass 参数集中一处，便于在线单独调（默认沿用 config 的 OD 带通）。
    @property
    def _bp_low_hz(self) -> float:
        from config import BP_LOW_HZ

        return BP_LOW_HZ

    @property
    def _bp_high_hz(self) -> float:
        from config import BP_HIGH_HZ

        return BP_HIGH_HZ

    @property
    def _bp_order(self) -> int:
        from config import BP_ORDER

        return BP_ORDER
