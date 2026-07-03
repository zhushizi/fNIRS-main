"""
rSO2 估算（与 data_analysis.plot_blood_oxygen_content / rso2_from_processed 一致）。

由 ΔHbO/ΔHbR 时间序列，在固定基线 HbT 与基线 rSO2 假设下，
用基线时段浓度变化均值做校正，再计算 rSO2 = 100 * HbO_abs / (HbO_abs + HbR_abs)。
"""

from __future__ import annotations

import numpy as np

DEFAULT_BASELINE_START_S = 30.0
DEFAULT_BASELINE_END_S = 60.0
DEFAULT_BASELINE_HBT_uM = 80.0
DEFAULT_BASELINE_RSO2_PCT = 65.0


def _absolute_hb_series(
    times: np.ndarray,
    delta_hbo: np.ndarray,
    delta_hbr: np.ndarray,
    *,
    baseline_start_s: float,
    baseline_end_s: float,
    baseline_hbt_uM: float,
    baseline_rso2_pct: float,
) -> tuple[np.ndarray, np.ndarray] | None:
    """
    在固定基线假设下把 ΔHbO/ΔHbR 转为绝对 HbO/HbR（摩尔，下限截 0）。

    基线时段 [baseline_start_s, baseline_end_s] 内无样本时返回 None。
    rSO2 与 tHi 共用此核心，保证二者自洽（rSO2 = 100·HbO_abs / (HbO_abs+HbR_abs)）。
    """
    times = np.asarray(times, dtype=float)
    delta_hbo = np.asarray(delta_hbo, dtype=float)
    delta_hbr = np.asarray(delta_hbr, dtype=float)
    n = min(times.size, delta_hbo.size, delta_hbr.size)
    if n == 0:
        return None
    times = times[:n]
    delta_hbo = delta_hbo[:n]
    delta_hbr = delta_hbr[:n]

    baseline_mask = (times >= baseline_start_s) & (times <= baseline_end_s)
    if not np.any(baseline_mask):
        return None

    baseline_hbt_M = baseline_hbt_uM * 1e-6
    baseline_hbo_abs_M = baseline_hbt_M * (baseline_rso2_pct / 100.0)
    baseline_hbr_abs_M = baseline_hbt_M - baseline_hbo_abs_M

    baseline_delta_hbo_mean = float(np.mean(delta_hbo[baseline_mask]))
    baseline_delta_hbr_mean = float(np.mean(delta_hbr[baseline_mask]))

    hbo_abs_M = np.maximum(baseline_hbo_abs_M + (delta_hbo - baseline_delta_hbo_mean), 0.0)
    hbr_abs_M = np.maximum(baseline_hbr_abs_M + (delta_hbr - baseline_delta_hbr_mean), 0.0)
    return hbo_abs_M, hbr_abs_M


def compute_rso2_series(
    times: np.ndarray,
    delta_hbo: np.ndarray,
    delta_hbr: np.ndarray,
    *,
    baseline_start_s: float = DEFAULT_BASELINE_START_S,
    baseline_end_s: float = DEFAULT_BASELINE_END_S,
    baseline_hbt_uM: float = DEFAULT_BASELINE_HBT_uM,
    baseline_rso2_pct: float = DEFAULT_BASELINE_RSO2_PCT,
) -> np.ndarray | None:
    """
    由 ΔHbO/ΔHbR 时间序列估算 rSO2（%）。

    基线时段 [baseline_start_s, baseline_end_s] 内无样本时返回 None。
    """
    abs_hb = _absolute_hb_series(
        times,
        delta_hbo,
        delta_hbr,
        baseline_start_s=baseline_start_s,
        baseline_end_s=baseline_end_s,
        baseline_hbt_uM=baseline_hbt_uM,
        baseline_rso2_pct=baseline_rso2_pct,
    )
    if abs_hb is None:
        return None
    hbo_abs_M, hbr_abs_M = abs_hb
    hbt_abs_M = hbo_abs_M + hbr_abs_M

    with np.errstate(divide="ignore", invalid="ignore"):
        return np.where(hbt_abs_M > 0, 100.0 * hbo_abs_M / hbt_abs_M, np.nan)


def compute_thi_series(
    times: np.ndarray,
    delta_hbo: np.ndarray,
    delta_hbr: np.ndarray,
    *,
    baseline_start_s: float = DEFAULT_BASELINE_START_S,
    baseline_end_s: float = DEFAULT_BASELINE_END_S,
    baseline_hbt_uM: float = DEFAULT_BASELINE_HBT_uM,
    baseline_rso2_pct: float = DEFAULT_BASELINE_RSO2_PCT,
) -> tuple[np.ndarray, np.ndarray] | None:
    """
    由 ΔHbO/ΔHbR 时间序列估算 tHi 与 ΔtHi（摩尔）。

    - tHi = HbO_abs + HbR_abs（固定基线伪绝对总血红蛋白，≈ 基线 HbT）
    - ΔtHi = tHi − 基线 HbT（相对基线的变化，≈0 起算）

    与 rSO2 共用绝对 HbO/HbR，二者自洽。基线窗内无样本时返回 None。
    """
    abs_hb = _absolute_hb_series(
        times,
        delta_hbo,
        delta_hbr,
        baseline_start_s=baseline_start_s,
        baseline_end_s=baseline_end_s,
        baseline_hbt_uM=baseline_hbt_uM,
        baseline_rso2_pct=baseline_rso2_pct,
    )
    if abs_hb is None:
        return None
    hbo_abs_M, hbr_abs_M = abs_hb
    thi_M = hbo_abs_M + hbr_abs_M
    baseline_hbt_M = baseline_hbt_uM * 1e-6
    return thi_M, thi_M - baseline_hbt_M
