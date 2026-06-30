"""双接收源五波长信号预处理：滤波、RMS 切段、多波长配对。"""

from __future__ import annotations

import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt

from config import (
    DETECTOR_CHANNELS,
    INTENSITY_COLUMNS,
    WAVELENGTH_CHANNELS,
    WAVELENGTH_OFF_CODE,
    detector_channel_by_code,
    intensity_column_for,
    wavelength_channel_by_code,
)


def _empty_interleaved_frame() -> pd.DataFrame:
    return pd.DataFrame(columns=["Time (s)", *INTENSITY_COLUMNS])


def threshold_filter(
    df: pd.DataFrame,
    signal_col: str = "Value",
    lower_threshold: int = 50000,
    upper_threshold: int = 300000,
    zero_level: int = 170000,
) -> pd.DataFrame:
    """对采样值做阈值抑制，超限值直接替换为 zero_level。"""
    filtered = df.copy()
    filtered[signal_col] = np.where(
        (df[signal_col] < lower_threshold) | (df[signal_col] > upper_threshold),
        zero_level,
        df[signal_col],
    )
    return filtered


def butter_lowpass_filter(
    df: pd.DataFrame,
    cutoff_hz: float,
    fs: float,
    order: int = 4,
    signal_col: str = "Value",
) -> pd.DataFrame:
    """按接收源/波长组合分别低通，避免不同 LED/PD 的电平互相污染。"""
    if len(df) < max(12, order * 3):
        return df.copy()

    filtered = df.copy()
    if signal_col in filtered.columns:
        filtered[signal_col] = filtered[signal_col].astype(float)
    nyquist = 0.5 * fs
    if nyquist <= 0 or cutoff_hz >= nyquist:
        return filtered

    b, a = butter(order, cutoff_hz / nyquist, btype="low", analog=False)
    group_cols = [col for col in ("DetectorId", "Wavelength") if col in filtered.columns]
    grouped = filtered.groupby(group_cols, sort=False) if group_cols else [(None, filtered)]
    for _, group in grouped:
        if len(group) < max(12, order * 3):
            continue
        padlen = min(len(group) - 1, 3 * (max(len(a), len(b)) - 1))
        if padlen <= 0:
            continue
        filtered.loc[group.index, signal_col] = filtfilt(
            b,
            a,
            group[signal_col].astype(float),
            padlen=padlen,
        )
    return filtered


def sliding_window_rms(
    df: pd.DataFrame,
    signal_col: str = "Value",
    wavelength_col: str = "Wavelength",
    detector_col: str = "DetectorId",
    remove_dc: bool = False,
) -> pd.DataFrame:
    """
    按 (DetectorId, Wavelength) 连续段切段，并把每一段压成一行 RMS 代表值。
    """
    if df.empty:
        return df.copy()

    keys = df[[detector_col, wavelength_col]].astype(int).to_numpy()
    change_points = np.where(np.any(np.diff(keys, axis=0) != 0, axis=1))[0] + 1
    segments = np.split(np.arange(len(df)), change_points)
    rows: list[pd.Series] = []

    for segment in segments:
        if len(segment) == 0:
            continue
        segment_idx = segment.tolist()
        segment_data = df.loc[segment_idx, signal_col].astype(float)
        if remove_dc:
            segment_data = segment_data - segment_data.mean()
        rms_val = float(np.sqrt(np.mean(np.square(segment_data))))
        rep_row = df.loc[segment_idx[0]].copy()
        rep_row[signal_col] = rms_val
        if "Time (s)" in df.columns:
            rep_row["Time (s)"] = float(df.loc[segment_idx, "Time (s)"].mean())
        rows.append(rep_row)

    return pd.DataFrame(rows).reset_index(drop=True)


def aggregate_wavelength_cycles(
    df: pd.DataFrame,
    signal_col: str = "Value",
    mode_col: str = "Wavelength",
) -> pd.DataFrame:
    """
    将 RMS 后的 2 接收源 x 5 波长段合并为一行。

    在时序上每凑齐 DETECTOR_CHANNELS x WAVELENGTH_CHANNELS 即输出一行。
    """
    required_keys = {(det.code, wl.code) for det in DETECTOR_CHANNELS for wl in WAVELENGTH_CHANNELS}
    pending: dict[tuple[int, int], pd.Series] = {}
    rows: list[dict[str, float]] = []

    def flush_pending() -> None:
        if set(pending.keys()) != required_keys:
            return
        times = [float(row["Time (s)"]) for row in pending.values()]
        out: dict[str, float] = {"Time (s)": float(np.mean(times))}
        for det in DETECTOR_CHANNELS:
            for wl in WAVELENGTH_CHANNELS:
                out[intensity_column_for(det.name, wl.emitter_nm)] = float(
                    pending[(det.code, wl.code)][signal_col]
                )
        rows.append(out)
        pending.clear()

    for _, row in df.reset_index(drop=True).iterrows():
        code = int(row[mode_col])
        detector_code = int(row["DetectorId"])
        if wavelength_channel_by_code(code) is None or detector_channel_by_code(detector_code) is None:
            continue
        key = (detector_code, code)
        if key in pending:
            pending.clear()
        pending[key] = row
        if len(pending) == len(required_keys):
            flush_pending()

    return pd.DataFrame(rows)


def stack_intensities_for_detector(df: pd.DataFrame, detector) -> np.ndarray:
    """按 WAVELENGTH_CHANNELS 顺序堆叠为 (n_wavelengths, n_timepoints)。"""
    from config import intensity_columns_for_detector

    return np.vstack(
        [df[col].to_numpy(dtype=float) for col in intensity_columns_for_detector(detector)]
    )


def prepare_interleaved_dataframe(
    raw_df: pd.DataFrame,
    *,
    start_at_zero: bool,
) -> pd.DataFrame:
    """
    原始双接收 CSV 格式 → 多波长配对后的 interleaved 表。

    期望列：Time (s), DetectorId, Channel, Wavelength, Value
    （兼容旧格式：Time (s), SensorId, <任意>, Wavelength，值在第三列）
    """
    if raw_df.empty or len(raw_df) < 2:
        return _empty_interleaved_frame()

    df = raw_df.copy()
    if "Value" not in df.columns:
        value_col = [c for c in df.columns if c not in ("Time (s)", "SensorId", "DetectorId", "Channel", "Wavelength")]
        if not value_col:
            return _empty_interleaved_frame()
        df = df.rename(columns={value_col[0]: "Value"})
    if "DetectorId" not in df.columns:
        if "SensorId" in df.columns:
            df = df.rename(columns={"SensorId": "DetectorId"})
        else:
            return _empty_interleaved_frame()

    df = df[df["Wavelength"] != WAVELENGTH_OFF_CODE].reset_index(drop=True)
    if df.empty or len(df) < 2:
        return _empty_interleaved_frame()

    dt = df["Time (s)"].diff().mean()
    fs = 1.0 / dt if pd.notna(dt) and dt > 0 else 1.0

    df = butter_lowpass_filter(df=df, cutoff_hz=1.0, fs=fs, order=4)
    df = sliding_window_rms(df=df)
    final_df = aggregate_wavelength_cycles(df, mode_col="Wavelength")
    if final_df.empty:
        return final_df

    pair_times = final_df["Time (s)"].to_numpy(dtype=float)
    pair_dt = np.diff(pair_times)
    valid_pair_dt = pair_dt[np.isfinite(pair_dt) & (pair_dt > 0)]
    increment = float(np.mean(valid_pair_dt)) if valid_pair_dt.size else 0.001
    first_time = 0.0 if start_at_zero else float(pair_times[0])
    final_df = final_df.drop(columns=["Time (s)"])
    final_df.insert(0, "Time (s)", [first_time + i * increment for i in range(len(final_df))])
    final_df["Time (s)"] = final_df["Time (s)"].round(6)
    return final_df
