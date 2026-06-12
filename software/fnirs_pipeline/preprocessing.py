"""原始单通道信号预处理：滤波、RMS 切段、多波长配对。"""

from __future__ import annotations

import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt

from config import (
    CHANNEL_NAME,
    INTENSITY_COLUMNS,
    WAVELENGTH_CHANNELS,
    WAVELENGTH_OFF_CODE,
    wavelength_channel_by_code,
)


def _empty_interleaved_frame() -> pd.DataFrame:
    return pd.DataFrame(columns=["Time (s)", *INTENSITY_COLUMNS])


def threshold_filter(
    df: pd.DataFrame,
    signal_col: str = CHANNEL_NAME,
    lower_threshold: int = 50000,
    upper_threshold: int = 300000,
    zero_level: int = 170000,
) -> pd.DataFrame:
    """对单通道数值做阈值抑制，超限值直接替换为 zero_level。"""
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
    signal_col: str = CHANNEL_NAME,
) -> pd.DataFrame:
    """对单通道沿时间轴做低通滤波。"""
    if len(df) < max(12, order * 3):
        return df.copy()

    filtered = df.copy()
    nyquist = 0.5 * fs
    if nyquist <= 0 or cutoff_hz >= nyquist:
        return filtered

    b, a = butter(order, cutoff_hz / nyquist, btype="low", analog=False)
    padlen = min(len(df) - 1, 3 * (max(len(a), len(b)) - 1))
    if padlen <= 0:
        return filtered
    filtered[signal_col] = filtfilt(b, a, df[signal_col], padlen=padlen)
    return filtered


def sliding_window_rms(
    df: pd.DataFrame,
    signal_col: str = CHANNEL_NAME,
    wavelength_col: str = "Wavelength",
    remove_dc: bool = False,
) -> pd.DataFrame:
    """
    按波长切段，并把每一段压成一行 RMS 代表值。

    这样后续在做双波长配对时，每个波长块只保留一个物理有效样本，
    避免把同一块里的重复 RMS 再展开成多行。
    """
    if df.empty:
        return df.copy()

    wavelength_reference = df[wavelength_col].values
    change_points = np.where(np.diff(wavelength_reference) != 0)[0] + 1
    segments = np.split(np.arange(len(df)), change_points)
    rows = []

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
    signal_col: str = CHANNEL_NAME,
    mode_col: str = "Wavelength",
) -> pd.DataFrame:
    """
    将 RMS 后的多波长段合并为一行：每种活跃波长一列（列名见 config）。

    在时序上每凑齐 WAVELENGTH_CHANNELS 中的全部协议码即输出一行；
    与具体交错顺序无关，便于扩展到 3 路及以上波长。
    """
    n_wl = len(WAVELENGTH_CHANNELS)
    required_codes = {ch.code for ch in WAVELENGTH_CHANNELS}
    pending: dict[int, pd.Series] = {}
    rows: list[dict[str, float]] = []

    def flush_pending() -> None:
        if set(pending.keys()) != required_codes:
            return
        times = [float(pending[ch.code]["Time (s)"]) for ch in WAVELENGTH_CHANNELS]
        out: dict[str, float] = {"Time (s)": float(np.mean(times))}
        for ch in WAVELENGTH_CHANNELS:
            out[ch.intensity_column] = float(pending[ch.code][signal_col])
        rows.append(out)
        pending.clear()

    for _, row in df.reset_index(drop=True).iterrows():
        code = int(row[mode_col])
        if wavelength_channel_by_code(code) is None:
            continue
        pending[code] = row
        if len(pending) == n_wl:
            flush_pending()

    return pd.DataFrame(rows)


def stack_intensities_for_mbll(df: pd.DataFrame) -> np.ndarray:
    """按 WAVELENGTH_CHANNELS 顺序堆叠为 (n_wavelengths, n_timepoints)。"""
    return np.vstack(
        [df[ch.intensity_column].to_numpy(dtype=float) for ch in WAVELENGTH_CHANNELS]
    )


def prepare_interleaved_dataframe(
    raw_df: pd.DataFrame,
    *,
    start_at_zero: bool,
) -> pd.DataFrame:
    """
    原始单通道 CSV 格式 → 多波长配对后的 interleaved 表。

    在线分析与离线 run_pipeline 共用同一套预处理函数；
    start_at_zero=False 时保留窗口内真实起始时间（在线用），
    True 时时间轴从 0 起编（离线 run_pipeline 等价行为）。
    """
    if raw_df.empty or len(raw_df) < 2:
        return _empty_interleaved_frame()

    df = raw_df.copy()
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
    # 用平均配对间隔重建等间隔时间轴，减轻采集抖动对 fs 估计的影响
    final_df = final_df.drop(columns=["Time (s)"])
    final_df.insert(0, "Time (s)", [first_time + i * increment for i in range(len(final_df))])
    final_df["Time (s)"] = final_df["Time (s)"].round(6)
    return final_df
