"""光强 → OD → 带通 → 广义 MBLL → SSR → 三色团浓度。"""

from __future__ import annotations

import csv
import os

import nirsimple.preprocessing as nsp
import numpy as np
import pandas as pd
from scipy.signal import butter, resample_poly, sosfiltfilt
from tabulate import tabulate

from config import (
    BP_HIGH_HZ,
    BP_LOW_HZ,
    BP_ORDER,
    BP_TARGET_FS_HZ,
    CHROMOPHORE_TYPES,
    CYT_DIFFERENCE_EXTINCTION,
    DETECTOR_CHANNELS,
    INTENSITY_COLUMNS,
    MBLL_DEFAULT_AGE,
    OUTPUT_CHANNEL,
    WAVELENGTH_CHANNELS,
    mbll_wavelengths_nm,
    processed_column_names,
)
from online_android import AnalysisResult, enrich_result_with_rso2

from .preprocessing import stack_intensities_for_detector


def butter_bandpass_sos(lowcut: float, highcut: float, fs: float, order: int = 4):
    """构造带通滤波器的 SOS 形式。"""
    nyq = 0.5 * fs
    if nyq <= 0 or lowcut >= highcut or highcut >= nyq:
        return None
    return butter(order, [lowcut / nyq, highcut / nyq], btype="band", output="sos")


def smart_bandpass(
    data: np.ndarray,
    fs: float,
    lowcut: float = BP_LOW_HZ,
    highcut: float = BP_HIGH_HZ,
    order: int = BP_ORDER,
    target_fs: float = BP_TARGET_FS_HZ,
) -> np.ndarray:
    """对 OD 数据做稳健带通。"""
    if data.shape[1] < max(16, 3 * (order + 1) + 1):
        return data

    if fs > target_fs + 1:
        decim = int(round(fs / target_fs))
        fs_ds = fs / decim
        data_ds = resample_poly(data, up=1, down=decim, axis=1)
    else:
        decim, fs_ds, data_ds = 1, fs, data

    sos = butter_bandpass_sos(lowcut, highcut, fs_ds, order)
    if sos is None or data_ds.shape[1] < max(16, 3 * (order + 1) + 1):
        return data

    padlen = min(data_ds.shape[1] - 1, 3 * (order + 1))
    if padlen <= 0:
        return data

    data_bp = sosfiltfilt(sos, data_ds, axis=1, padtype="odd", padlen=padlen)
    if decim > 1:
        data_bp = resample_poly(data_bp, up=decim, down=1, axis=1)
    return data_bp


def short_separation_regression(
    long_od: np.ndarray,
    short_od: np.ndarray,
    eps: float = 1e-12,
) -> tuple[np.ndarray, np.ndarray]:
    """用短距 OD 回归校正长距 OD，返回 (corrected_long_od, beta)。"""
    if long_od.shape != short_od.shape:
        raise ValueError("long_od and short_od must have the same shape.")
    if long_od.ndim != 2:
        raise ValueError("OD matrices must be shaped as (n_wavelengths, n_timepoints).")

    short_centered = short_od - np.mean(short_od, axis=1, keepdims=True)
    long_centered = long_od - np.mean(long_od, axis=1, keepdims=True)
    denominator = np.sum(short_centered * short_centered, axis=1)
    numerator = np.sum(long_centered * short_centered, axis=1)
    beta = np.divide(
        numerator,
        denominator,
        out=np.zeros_like(numerator, dtype=float),
        where=denominator > eps,
    )
    corrected = long_od - beta[:, np.newaxis] * short_centered
    return corrected, beta


def _hemoglobin_extinctions(wavelengths: list[float], table: str = "wray") -> np.ndarray:
    """读取 nirsimple 内置 HbO/HbR 消光系数表，并插值到目标波长。"""
    ex_path = os.path.join(os.path.dirname(nsp.__file__), "tables", f"{table}.csv")
    ex_df = pd.read_csv(ex_path)
    table_wls = ex_df["lambda"].to_numpy(dtype=float)
    hbo = ex_df["hbo"].to_numpy(dtype=float)
    hbr = ex_df["hbr"].to_numpy(dtype=float)
    return np.column_stack(
        [
            np.interp(wavelengths, table_wls, hbo),
            np.interp(wavelengths, table_wls, hbr),
        ]
    )


def _cyt_extinction_vector(wavelengths: list[float]) -> np.ndarray:
    """返回 Cyt 差分消光系数向量，单位与 HbO/HbR 一致（OD / cm / M）。"""
    return np.asarray([CYT_DIFFERENCE_EXTINCTION[float(wl)] for wl in wavelengths], dtype=float) * 1000.0


def generalized_mbll(
    delta_od: np.ndarray,
    wavelengths: list[float],
    dpfs: list[float],
    distance_cm: float,
    table: str = "wray",
) -> np.ndarray:
    """
    分步广义 MBLL：先反演 HbO/HbR，再从 Hb 残差估计 Cyt。

    返回 (3, n_timepoints)，行顺序为 HbO / HbR / Cyt。
    """
    hb_ex = _hemoglobin_extinctions(wavelengths, table)
    cyt_ex = _cyt_extinction_vector(wavelengths)
    pathlength = np.asarray(dpfs, dtype=float) * float(distance_cm)
    a_hb = hb_ex * pathlength[:, np.newaxis]
    a_cyt = (cyt_ex * pathlength)[:, np.newaxis]

    hb_delta_c = np.linalg.pinv(a_hb) @ delta_od
    hb_fit_od = a_hb @ hb_delta_c
    residual_od = delta_od - hb_fit_od

    hb_projection = a_hb @ np.linalg.pinv(a_hb)
    cyt_residual_basis = a_cyt - hb_projection @ a_cyt
    cyt_delta_c = np.linalg.pinv(cyt_residual_basis) @ residual_od
    return np.vstack([hb_delta_c, cyt_delta_c])


def _estimate_fs(times: np.ndarray) -> float:
    dt = np.mean(np.diff(times))
    return 1.0 / dt if dt > 0 else 1.0


def compute_all_channel_concentrations(
    interleaved_df: pd.DataFrame,
    *,
    age: int = MBLL_DEFAULT_AGE,
    molar_ext_coeff_table: str = "wray",
    bp_low: float = BP_LOW_HZ,
    bp_high: float = BP_HIGH_HZ,
    bp_order: int = BP_ORDER,
) -> tuple[np.ndarray, dict[str, np.ndarray]] | None:
    """
    从配对光强表计算全部通道浓度。

    返回 (times, {channel_name: (3, n_timepoints)})。
    channel_name 包含各 DETECTOR_CHANNELS.name 及长距 SSR 通道（如 S1_D1_ssr）。
    """
    required_cols = ["Time (s)", *INTENSITY_COLUMNS]
    if interleaved_df.empty or any(col not in interleaved_df.columns for col in required_cols):
        return None

    times = interleaved_df["Time (s)"].to_numpy(dtype=float)
    if len(times) < 2:
        return None

    fs = _estimate_fs(times)
    wavelengths = mbll_wavelengths_nm()
    dpfs = [nsp.get_dpf(wl, age) for wl in wavelengths]

    detector_delta_od: dict[str, np.ndarray] = {}
    results: dict[str, np.ndarray] = {}

    for detector in DETECTOR_CHANNELS:
        samples = stack_intensities_for_detector(interleaved_df, detector)
        delta_od = nsp.intensities_to_od_changes(samples)
        detector_delta_od[detector.name] = delta_od
        delta_od_filt = smart_bandpass(delta_od, fs, lowcut=bp_low, highcut=bp_high, order=bp_order)
        results[detector.name] = generalized_mbll(
            delta_od_filt,
            wavelengths,
            dpfs,
            detector.distance_cm,
            table=molar_ext_coeff_table,
        )

    if len(DETECTOR_CHANNELS) >= 2:
        short_detector = min(DETECTOR_CHANNELS, key=lambda det: det.distance_cm)
        long_detector = max(DETECTOR_CHANNELS, key=lambda det: det.distance_cm)
        if short_detector.name != long_detector.name:
            corrected_od, _beta = short_separation_regression(
                detector_delta_od[long_detector.name],
                detector_delta_od[short_detector.name],
            )
            corrected_od_filt = smart_bandpass(
                corrected_od,
                fs,
                lowcut=bp_low,
                highcut=bp_high,
                order=bp_order,
            )
            ssr_name = f"{long_detector.name}_ssr"
            results[ssr_name] = generalized_mbll(
                corrected_od_filt,
                wavelengths,
                dpfs,
                long_detector.distance_cm,
                table=molar_ext_coeff_table,
            )

    return times, results


def select_output_series(
    times: np.ndarray,
    all_results: dict[str, np.ndarray],
    output_channel: str = OUTPUT_CHANNEL,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None:
    """从全部通道结果中提取配置输出通道的 HbO/HbR/Cyt 序列。"""
    delta_c = all_results.get(output_channel)
    if delta_c is None or delta_c.shape[0] < len(CHROMOPHORE_TYPES):
        return None

    hbo = delta_c[0]
    hbr = delta_c[1]
    cyt = delta_c[2]
    n_cols = min(len(times), len(hbo), len(hbr), len(cyt))
    if n_cols == 0:
        return None

    times = times[:n_cols]
    hbo = hbo[:n_cols]
    hbr = hbr[:n_cols]
    cyt = cyt[:n_cols]
    finite_mask = np.isfinite(times) & np.isfinite(hbo) & np.isfinite(hbr) & np.isfinite(cyt)
    if not np.any(finite_mask):
        return None
    return times[finite_mask], hbo[finite_mask], hbr[finite_mask], cyt[finite_mask]


def calculate_concentration_series(
    interleaved_df: pd.DataFrame,
    age: int = MBLL_DEFAULT_AGE,
    molar_ext_coeff_table: str = "wray",
    bp_low: float = BP_LOW_HZ,
    bp_high: float = BP_HIGH_HZ,
    bp_order: int = BP_ORDER,
    output_channel: str = OUTPUT_CHANNEL,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None:
    """
    从配对光强表计算配置输出通道的 HbO/HbR/Cyt 时间序列（在线分析入口）。
    """
    computed = compute_all_channel_concentrations(
        interleaved_df,
        age=age,
        molar_ext_coeff_table=molar_ext_coeff_table,
        bp_low=bp_low,
        bp_high=bp_high,
        bp_order=bp_order,
    )
    if computed is None:
        return None
    times, all_results = computed
    selected = select_output_series(times, all_results, output_channel)
    if selected is None:
        return None
    return selected


def summarize_processed_concentrations(
    delta_c: np.ndarray,
    times: np.ndarray | None = None,
    output_channel: str = OUTPUT_CHANNEL,
) -> AnalysisResult:
    """为安卓端提取配置输出通道的 HbO/HbR 均值摘要。"""
    if delta_c.shape[0] < 2:
        return {
            "ok": False,
            "message": f"Cannot extract HbO/HbR from channel {output_channel}.",
        }

    hbo = delta_c[0]
    hbr = delta_c[1]
    hbo = hbo[np.isfinite(hbo)]
    hbr = hbr[np.isfinite(hbr)]
    if hbo.size == 0 or hbr.size == 0:
        return {"ok": False, "message": "HbO/HbR result contains no finite values."}

    result: AnalysisResult = {
        "ok": True,
        "output_channel": output_channel,
        "mean_hbo": float(np.mean(hbo)),
        "mean_hbr": float(np.mean(hbr)),
        "sample_count": int(min(hbo.size, hbr.size)),
    }
    if delta_c.shape[0] >= 3:
        cyt = delta_c[2]
        cyt_finite = cyt[np.isfinite(cyt)]
        if cyt_finite.size > 0:
            result["mean_cyt"] = float(np.mean(cyt_finite))

    if times is not None and times.size >= 2:
        n = min(times.size, delta_c.shape[1])
        return enrich_result_with_rso2(
            result,
            times=times[:n],
            hbo_series=delta_c[0, :n],
            hbr_series=delta_c[1, :n],
        )
    return result


def process_csv_dataset(
    input_csv: str,
    output_csv: str,
    age: int = MBLL_DEFAULT_AGE,
    molar_ext_coeff_table: str = "wray",
    bp_low: float = BP_LOW_HZ,
    bp_high: float = BP_HIGH_HZ,
    bp_order: int = BP_ORDER,
    output_channel: str = OUTPUT_CHANNEL,
) -> AnalysisResult:
    """离线终算：写入配置输出通道的 HbO/HbR/Cyt 到 processed_output.csv。"""
    df = pd.read_csv(input_csv)
    computed = compute_all_channel_concentrations(
        df,
        age=age,
        molar_ext_coeff_table=molar_ext_coeff_table,
        bp_low=bp_low,
        bp_high=bp_high,
        bp_order=bp_order,
    )
    if computed is None:
        message = "Insufficient or invalid interleaved data for processing."
        print(message)
        return {"ok": False, "message": message}

    times, all_results = computed
    selected = select_output_series(times, all_results, output_channel)
    if selected is None:
        message = f"Output channel {output_channel!r} not available in MBLL results."
        print(message)
        return {"ok": False, "message": message}

    times, hbo, hbr, cyt = selected
    delta_c = np.vstack([hbo, hbr, cyt])
    col_names = processed_column_names(output_channel)
    header_out = ["Time"] + list(col_names)

    with open(output_csv, "w", newline="", encoding="utf-8") as f_out:
        writer = csv.writer(f_out)
        writer.writerow(header_out)
        n_cols = min(len(times), delta_c.shape[1])
        for col_idx in range(n_cols):
            writer.writerow([times[col_idx]] + list(delta_c[:, col_idx]))

    last_sample = delta_c[:, -1]
    table_data = [
        [output_channel, chrom, f"{last_sample[idx]:.4e}"]
        for idx, chrom in enumerate(CHROMOPHORE_TYPES)
    ]
    print(f"Post-processing complete. Output channel: {output_channel}")
    print(f"Output saved to '{output_csv}'.")
    print("\nExample: Processed concentrations at the final time sample (Cyt uses UCL extinction):")
    print(tabulate(table_data, headers=["Channel", "Type", "Concentration"]))
    return summarize_processed_concentrations(delta_c, times=times, output_channel=output_channel)
