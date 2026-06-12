"""光强 → OD → 带通 → MBLL → CBSI 及结果汇总。"""

from __future__ import annotations

import csv

import nirsimple.preprocessing as nsp
import nirsimple.processing as nproc
import numpy as np
import pandas as pd
from scipy.signal import butter, resample_poly, sosfiltfilt
from tabulate import tabulate

from config import (
    BP_HIGH_HZ,
    BP_LOW_HZ,
    BP_ORDER,
    BP_TARGET_FS_HZ,
    CHANNEL_NAME,
    INTENSITY_COLUMNS,
    MBLL_DEFAULT_AGE,
    SOURCE_DETECTOR_DISTANCE_CM,
    mbll_wavelengths_nm,
)
from online_android import AnalysisResult, enrich_result_with_rso2

from .preprocessing import stack_intensities_for_mbll


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
    """
    对 OD 数据做稳健带通。

    当采样率过高时，先降采样再滤波，最后升采样回来，
    可以减少数值不稳定和不必要的计算量。
    """
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


def build_channel_info(
    age: int = MBLL_DEFAULT_AGE,
    source_detector_distance_cm: float = SOURCE_DETECTOR_DISTANCE_CM,
):
    """构造单通道 MBLL 所需的通道名、波长、DPF 与源探距离。"""
    ch_wls = mbll_wavelengths_nm()
    channel_names = [CHANNEL_NAME] * len(ch_wls)
    ch_dpfs = [nsp.get_dpf(wl, age) for wl in ch_wls]
    ch_distances = [source_detector_distance_cm] * len(ch_wls)
    return channel_names, ch_wls, ch_dpfs, ch_distances


def extract_hbo_hbr_series(
    delta_c_corr: np.ndarray,
    channel_types: list[str],
) -> tuple[np.ndarray, np.ndarray] | None:
    """从 CBSI 输出中按通道类型提取 HbO/HbR 序列；多通道时取均值。"""
    hbo_indices: list[int] = []
    hbr_indices: list[int] = []
    for idx, channel_type in enumerate(channel_types):
        normalized = str(channel_type).lower()
        if "hbo" in normalized:
            hbo_indices.append(idx)
        elif "hbr" in normalized:
            hbr_indices.append(idx)

    if not hbo_indices or not hbr_indices:
        return None
    hbo = np.mean(delta_c_corr[hbo_indices, :], axis=0)
    hbr = np.mean(delta_c_corr[hbr_indices, :], axis=0)
    return hbo, hbr


def run_mbll_cbsi(
    samples: np.ndarray,
    times: np.ndarray,
    *,
    age: int = MBLL_DEFAULT_AGE,
    source_detector_distance_cm: float = SOURCE_DETECTOR_DISTANCE_CM,
    molar_ext_coeff_table: str = "wray",
    bp_low: float = BP_LOW_HZ,
    bp_high: float = BP_HIGH_HZ,
    bp_order: int = BP_ORDER,
) -> tuple[np.ndarray, list[str], list[str]]:
    """共享核心：光强矩阵 → OD 变化 → 带通 → MBLL → CBSI。"""
    channel_names, ch_wls, ch_dpfs, ch_distances = build_channel_info(
        age,
        source_detector_distance_cm,
    )
    delta_od = nsp.intensities_to_od_changes(samples)

    dt = np.mean(np.diff(times))
    fs = 1.0 / dt if dt > 0 else 1.0
    delta_od_filt = smart_bandpass(delta_od, fs, lowcut=bp_low, highcut=bp_high, order=bp_order)

    delta_c, new_ch_names, new_ch_types = nsp.mbll(
        delta_od_filt,
        channel_names,
        ch_wls,
        ch_dpfs,
        ch_distances,
        unit="cm",
        table=molar_ext_coeff_table,
    )
    delta_c_corr, corr_ch_names, corr_ch_types = nproc.cbsi(delta_c, new_ch_names, new_ch_types)
    return delta_c_corr, corr_ch_names, corr_ch_types


def calculate_concentration_series(
    interleaved_df: pd.DataFrame,
    age: int = MBLL_DEFAULT_AGE,
    source_detector_distance_cm: float = SOURCE_DETECTOR_DISTANCE_CM,
    molar_ext_coeff_table: str = "wray",
    bp_low: float = BP_LOW_HZ,
    bp_high: float = BP_HIGH_HZ,
    bp_order: int = BP_ORDER,
) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    """
    从配对后的光强表计算 HbO/HbR 时间序列（在线分析入口）。

    流程：光强 → OD 变化 → 带通 → MBLL → CBSI → 提取 HbO/HbR。
    离线终算使用 process_csv_dataset，算法步骤相同但未调用本函数。
    """
    required_cols = ["Time (s)", *INTENSITY_COLUMNS]
    if interleaved_df.empty or any(col not in interleaved_df.columns for col in required_cols):
        return None

    times = interleaved_df["Time (s)"].to_numpy(dtype=float)
    if len(times) < 2:
        return None

    samples = stack_intensities_for_mbll(interleaved_df)
    delta_c_corr, _corr_ch_names, corr_ch_types = run_mbll_cbsi(
        samples,
        times,
        age=age,
        source_detector_distance_cm=source_detector_distance_cm,
        molar_ext_coeff_table=molar_ext_coeff_table,
        bp_low=bp_low,
        bp_high=bp_high,
        bp_order=bp_order,
    )
    series = extract_hbo_hbr_series(delta_c_corr, corr_ch_types)
    if series is None:
        return None

    hbo, hbr = series
    n_cols = min(len(times), len(hbo), len(hbr))
    if n_cols == 0:
        return None
    times = times[:n_cols]
    hbo = hbo[:n_cols]
    hbr = hbr[:n_cols]
    finite_mask = np.isfinite(times) & np.isfinite(hbo) & np.isfinite(hbr)
    if not np.any(finite_mask):
        return None
    return times[finite_mask], hbo[finite_mask], hbr[finite_mask]


def summarize_processed_concentrations(
    delta_c_corr: np.ndarray,
    channel_types: list[str],
    times: np.ndarray | None = None,
) -> AnalysisResult:
    """为安卓端测试显示提取本次 HbO/HbR 平均值，并在可能时附带 rSO2 摘要。"""
    hbo_values = []
    hbr_values = []
    for idx, channel_type in enumerate(channel_types):
        normalized = str(channel_type).lower()
        if "hbo" in normalized:
            hbo_values.append(delta_c_corr[idx])
        elif "hbr" in normalized:
            hbr_values.append(delta_c_corr[idx])

    if not hbo_values or not hbr_values:
        return {
            "ok": False,
            "message": f"Cannot identify HbO/HbR columns from channel types: {channel_types}",
        }

    hbo = np.concatenate(hbo_values)
    hbr = np.concatenate(hbr_values)
    hbo = hbo[np.isfinite(hbo)]
    hbr = hbr[np.isfinite(hbr)]
    if hbo.size == 0 or hbr.size == 0:
        return {"ok": False, "message": "HbO/HbR result contains no finite values."}

    result: AnalysisResult = {
        "ok": True,
        "mean_hbo": float(np.mean(hbo)),
        "mean_hbr": float(np.mean(hbr)),
        "sample_count": int(min(hbo.size, hbr.size)),
    }
    if times is not None and times.size >= 2:
        hbo_series = np.mean(np.vstack(hbo_values), axis=0)
        hbr_series = np.mean(np.vstack(hbr_values), axis=0)
        n = min(times.size, hbo_series.size, hbr_series.size)
        return enrich_result_with_rso2(
            result,
            times=times[:n],
            hbo_series=hbo_series[:n],
            hbr_series=hbr_series[:n],
        )
    return result


def process_csv_dataset(
    input_csv: str,
    output_csv: str,
    age: int = MBLL_DEFAULT_AGE,
    source_detector_distance_cm: float = SOURCE_DETECTOR_DISTANCE_CM,
    molar_ext_coeff_table: str = "wray",
    bp_low: float = BP_LOW_HZ,
    bp_high: float = BP_HIGH_HZ,
    bp_order: int = BP_ORDER,
) -> AnalysisResult:
    """
    离线终算入口：从 interleaved CSV 计算 HbO/HbR 并写入 processed_output.csv。

    算法与 calculate_concentration_series 相同（OD → 带通 → MBLL → CBSI），
    但面向全段文件；成功时返回 {"ok": True} 供 analysis_result 回传安卓。
    """
    df = pd.read_csv(input_csv)
    required_cols = ["Time (s)", *INTENSITY_COLUMNS]
    if df.empty or any(col not in df.columns for col in required_cols):
        message = "Insufficient or invalid interleaved data for processing."
        print(message)
        return {"ok": False, "message": message}

    times = df["Time (s)"].to_numpy(dtype=float)
    if len(times) < 2:
        message = "Need at least two time samples to run MBLL."
        print(message)
        return {"ok": False, "message": message}

    samples = stack_intensities_for_mbll(df)
    delta_c_corr, corr_ch_names, corr_ch_types = run_mbll_cbsi(
        samples,
        times,
        age=age,
        source_detector_distance_cm=source_detector_distance_cm,
        molar_ext_coeff_table=molar_ext_coeff_table,
        bp_low=bp_low,
        bp_high=bp_high,
        bp_order=bp_order,
    )

    processed_headers = [f"{name}_{ctype}" for name, ctype in zip(corr_ch_names, corr_ch_types)]
    header_out = ["Time"] + processed_headers

    with open(output_csv, "w", newline="", encoding="utf-8") as f_out:
        writer = csv.writer(f_out)
        writer.writerow(header_out)
        n_cols = min(len(times), delta_c_corr.shape[1])
        for col_idx in range(n_cols):
            writer.writerow([times[col_idx]] + list(delta_c_corr[:, col_idx]))

    last_sample = delta_c_corr[:, -1]
    table_data = []
    for idx, ch in enumerate(corr_ch_names):
        table_data.append([ch, corr_ch_types[idx], f"{last_sample[idx]:.4e}"])
    print(f"Post-processing complete. Output saved to '{output_csv}'.")
    print("\nExample: Processed concentrations at the final time sample:")
    print(tabulate(table_data, headers=["Channel", "Type", "Concentration"]))
    return {"ok": True}
