import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import nirsimple.preprocessing as nsp
import nirsimple.processing as nproc
from scipy.signal import butter, filtfilt, resample_poly, sosfiltfilt

from config import (
    CHANNEL_NAME,
    MBLL_DEFAULT_AGE,
    MBLL_WAVELENGTH_WL1_NM,
    MBLL_WAVELENGTH_WL2_NM,
    WAVELENGTH_OFF_CODE,
)

def plot_curves(df: pd.DataFrame, title_suffix: str = ""):
    # 按 Wavelength 分组：1 和 2 分别组成数组。
    df_w1 = df[df["Wavelength"] == 1].reset_index(drop=True)
    df_w2 = df[df["Wavelength"] == 2].reset_index(drop=True)
    wavelength_1_array = df_w1["S1_D1"].to_numpy()
    wavelength_2_array = df_w2["S1_D1"].to_numpy()
    print(f"Wavelength=1 样本数: {len(wavelength_1_array)}")
    print(f"Wavelength=2 样本数: {len(wavelength_2_array)}")
    n_samples = min(len(wavelength_1_array), len(wavelength_2_array))
    wavelength_1_array = wavelength_1_array[:n_samples]
    wavelength_2_array = wavelength_2_array[:n_samples]

    # 计算各波长组的采样率
    def _calc_fs(sub_df):
        dt = sub_df["Time (s)"].diff().mean()
        return 1.0 / dt if pd.notna(dt) and dt > 0 else 1.0

    fs_w1 = _calc_fs(df_w1)
    fs_w2 = _calc_fs(df_w2)

    suffix = f" ({title_suffix})" if title_suffix else ""

    if len(df_w1) > 0 or len(df_w2) > 0:
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))

        # --- 子图1：时域波形 ---
        ax1 = axes[0]
        ax1.plot(
            wavelength_1_array,
            label=f"Wavelength=1 ({int(MBLL_WAVELENGTH_WL1_NM)}nm MBLL, fs={fs_w1:.1f}Hz)",
            linewidth=1.0,
        )
        ax1.plot(
            wavelength_2_array,
            label=f"Wavelength=2 ({int(MBLL_WAVELENGTH_WL2_NM)}nm, fs={fs_w2:.1f}Hz)",
            linewidth=1.0,
        )
        ax1.set_xlabel("Sample Index")
        ax1.set_ylabel("S1_D1")
        # ax1.set_ylim(0, 20e6)
        ax1.set_title(f"S1_D1 Time-Domain{suffix}")
        ax1.legend()
        ax1.grid(alpha=0.3)

        # --- 子图2：FFT 频谱 ---
        ax2 = axes[1]
        if n_samples >= 16:
            # 对信号去均值后做 FFT，突出动态分量
            w1_detrended = wavelength_1_array - np.mean(wavelength_1_array)
            w2_detrended = wavelength_2_array - np.mean(wavelength_2_array)

            f1 = np.fft.rfftfreq(n_samples, d=1.0 / fs_w1)
            f2 = np.fft.rfftfreq(n_samples, d=1.0 / fs_w2)
            fft1 = np.fft.rfft(w1_detrended)
            fft2 = np.fft.rfft(w2_detrended)
            amp1 = np.abs(fft1) / n_samples
            amp2 = np.abs(fft2) / n_samples
            if n_samples > 1:
                amp1[1:-1] *= 2
                amp2[1:-1] *= 2

            ax2.plot(f1, amp1, label=f"Wavelength=1 ({int(MBLL_WAVELENGTH_WL1_NM)}nm)", linewidth=1.0)
            ax2.plot(f2, amp2, label=f"Wavelength=2 ({int(MBLL_WAVELENGTH_WL2_NM)}nm)", linewidth=1.0)
            ax2.set_xlabel("Frequency (Hz)")
            ax2.set_ylabel("Amplitude")
            ax2.set_title(f"S1_D1 FFT Spectrum{suffix}")
            ax2.legend()
            ax2.set_xlim(0, max(fs_w1, fs_w2) / 2)
        else:
            ax2.text(0.5, 0.5, "数据不足，无法计算频谱", ha="center", va="center", transform=ax2.transAxes)

        ax2.grid(alpha=0.3)
        fig.tight_layout()
        plt.show()
        plt.close(fig)

def butter_lowpass_filter(df, cutoff_hz, fs, order=4, exclude_columns=None):
    """Apply a low-pass Butterworth filter to selected columns of a data frame."""
    if exclude_columns is None:
        exclude_columns = []

    filtered_df = df.copy()
    nyquist = 0.5 * fs
    normal_cutoff = cutoff_hz / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)

    for col in ['S1_D1']:
        filtered_df[col] = filtfilt(b, a, df[col])

    return filtered_df


def sliding_window_rms(df: pd.DataFrame) -> pd.DataFrame:
    """
    按 Wavelength 的连续段切段，并在每段结束时用 RMS 代表该段。

    返回字段：
    - Time (s): 该连续段时间均值
    - SensorId: 该段 SensorId 众数（若无则取首值）
    - S1_D1: 该连续段 RMS
    - Wavelength: 该连续段波长码
    """
    required_cols = {"Time (s)", "S1_D1", "Wavelength"}
    missing = required_cols - set(df.columns)
    if missing:
        raise ValueError(f"Missing required columns for sliding_window_rms: {sorted(missing)}")

    if df.empty:
        return df.copy()

    result_rows = []
    current_wl = None
    phase_vals: list[float] = []
    phase_times: list[float] = []
    phase_sensor_ids: list[int] = []

    def finalize_phase():
        if current_wl is None or not phase_vals:
            return
        phase_vals_np = np.asarray(phase_vals, dtype=float)
        phase_rms = float(np.sqrt(np.mean(np.square(phase_vals_np))))
        phase_time = float(np.mean(np.asarray(phase_times, dtype=float)))
        sensor_id = int(pd.Series(phase_sensor_ids).mode().iloc[0]) if phase_sensor_ids else 0
        result_rows.append(
            {
                "Time (s)": phase_time,
                "SensorId": sensor_id,
                "S1_D1": phase_rms,
                "Wavelength": int(current_wl),
            }
        )

    col_idx_time = df.columns.get_loc("Time (s)")
    col_idx_s1d1 = df.columns.get_loc("S1_D1")
    col_idx_wl = df.columns.get_loc("Wavelength")
    col_idx_sensor = df.columns.get_loc("SensorId") if "SensorId" in df.columns else None

    for row in df.itertuples(index=False, name=None):
        wl = int(row[col_idx_wl])
        if current_wl is None:
            current_wl = wl
        elif wl != current_wl:
            finalize_phase()
            current_wl = wl
            phase_vals.clear()
            phase_times.clear()
            phase_sensor_ids.clear()

        phase_vals.append(float(row[col_idx_s1d1]))
        phase_times.append(float(row[col_idx_time]))
        if col_idx_sensor is not None:
            phase_sensor_ids.append(int(row[col_idx_sensor]))

    # 收尾：最后一个连续段
    finalize_phase()

    if not result_rows:
        return pd.DataFrame(columns=["Time (s)", "SensorId", "S1_D1", "Wavelength"])

    return pd.DataFrame(result_rows, columns=["Time (s)", "SensorId", "S1_D1", "Wavelength"])

def _channel_info():
    """单通道双波长的通道名、波长、DPF、源探距离（见 config.MBLL_WAVELENGTH_*）。"""
    ch_names = [CHANNEL_NAME, CHANNEL_NAME]
    ch_wls = [MBLL_WAVELENGTH_WL1_NM, MBLL_WAVELENGTH_WL2_NM]
    ch_dpfs = [
        nsp.get_dpf(MBLL_WAVELENGTH_WL1_NM, MBLL_DEFAULT_AGE),
        nsp.get_dpf(MBLL_WAVELENGTH_WL2_NM, MBLL_DEFAULT_AGE),
    ]
    ch_distances = [3.0, 3.0]
    return ch_names, ch_wls, ch_dpfs, ch_distances

def butter_bandpass_sos(lowcut, highcut, fs, order=4):
    """Return an SOS band-pass filter."""
    nyq = 0.5 * fs
    sos = butter(order, [lowcut / nyq, highcut / nyq],
                 btype="band", output="sos")
    return sos

def smart_bandpass(data, fs,
                   lowcut=0.05, highcut=0.1, order=4,
                   target_fs=20.0):
    """
    Zero-phase band-pass along *time* axis.
    `data` shape: (n_channels, n_timepoints)
    """
    # Down-sample
    if fs > target_fs + 1: # leave a little margin
        decim = int(round(fs / target_fs))
        fs_ds = fs / decim
        data_ds = resample_poly(data, up=1, down=decim, axis=1)
    else:
        decim, fs_ds, data_ds = 1, fs, data
    # Design stable filter
    sos = butter_bandpass_sos(lowcut, highcut, fs_ds, order)
    # Zero-phase filtering
    data_bp = sosfiltfilt(sos, data_ds, axis=1, padtype="odd",
                          padlen=3 * (order + 1))
    # Up-sample back if we had decimated
    if decim > 1:
        data_bp = resample_poly(data_bp, up=decim, down=1, axis=1)
    return data_bp

def threshold_filter(df, lower_threshold=200, upper_threshold=10000000, zero_level=1000000, exclude_columns=None):
    """Suppress outliers of a data frame."""
    if exclude_columns is None:
        exclude_columns = []

    suppressed_df = df.copy()

    for col in ['S1_D1']:
        suppressed_df[col] = np.where(
            (df[col] < lower_threshold) | (df[col] > upper_threshold),
            zero_level,
            df[col]
        )

    return suppressed_df

def plot_array(delta_c_corr,corr_ch_types):
    plt.figure(figsize=(12, 5))
    plt.plot(delta_c_corr[0], label=corr_ch_types[0], linewidth=1.0)
    plt.plot(delta_c_corr[1], label=corr_ch_types[1], linewidth=1.0)
    plt.xlabel("Time (s)")
    plt.ylabel("S1_D1")
    # plt.ylim(0, 20e6)
    plt.title("S1_D1 Curves Grouped by Hbo/Hbr")
    plt.legend()
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.show()
    plt.close()


def plot_blood_oxygen_content(
    delta_c: np.ndarray,
    ch_types: list[str],
    times: np.ndarray,
    baseline_start_s: float = 30.0,
    baseline_end_s: float = 60.0,
    baseline_hbt_uM: float = 80.0,
    baseline_rso2_pct: float = 65.0,
):
    """参考 rso2_from_processed.py，用固定基线估算并绘制 rSO2。"""
    lowered = [str(t).lower() for t in ch_types]
    hbo_idx = next((i for i, t in enumerate(lowered) if "hbo" in t), None)
    hbr_idx = next((i for i, t in enumerate(lowered) if "hbr" in t), None)
    if hbo_idx is None or hbr_idx is None:
        print(f"未找到 HbO/HbR 通道，跳过 rSO2 绘图。当前通道: {ch_types}")
        return

    delta_hbo = np.asarray(delta_c[hbo_idx], dtype=float)
    delta_hbr = np.asarray(delta_c[hbr_idx], dtype=float)
    times = np.asarray(times, dtype=float)
    n = min(len(times), len(delta_hbo), len(delta_hbr))
    if n == 0:
        print("rSO2 绘图数据为空，跳过。")
        return
    times = times[:n]
    delta_hbo = delta_hbo[:n]
    delta_hbr = delta_hbr[:n]

    baseline_mask = (times >= baseline_start_s) & (times <= baseline_end_s)
    if not np.any(baseline_mask):
        print(
            f"在 {baseline_start_s:.1f}s~{baseline_end_s:.1f}s 内没有基线样本，"
            "跳过 rSO2 绘图。"
        )
        return

    baseline_hbt_M = baseline_hbt_uM * 1e-6
    baseline_hbo_abs_M = baseline_hbt_M * (baseline_rso2_pct / 100.0)
    baseline_hbr_abs_M = baseline_hbt_M - baseline_hbo_abs_M

    baseline_delta_hbo_mean = float(np.mean(delta_hbo[baseline_mask]))
    baseline_delta_hbr_mean = float(np.mean(delta_hbr[baseline_mask]))
    hbo_abs_M = baseline_hbo_abs_M + (delta_hbo - baseline_delta_hbo_mean)
    hbr_abs_M = baseline_hbr_abs_M + (delta_hbr - baseline_delta_hbr_mean)
    hbo_abs_M = np.maximum(hbo_abs_M, 0.0)
    hbr_abs_M = np.maximum(hbr_abs_M, 0.0)
    hbt_abs_M = hbo_abs_M + hbr_abs_M

    with np.errstate(divide="ignore", invalid="ignore"):
        rso2_pct = np.where(hbt_abs_M > 0, 100.0 * hbo_abs_M / hbt_abs_M, np.nan)

    plt.figure(figsize=(12, 4))
    plt.plot(times, rso2_pct, linewidth=1.2, label="rSO2")
    plt.xlabel("Time (s)")
    plt.ylabel("Blood Oxygen Content (%)")
    plt.title("rSO2 (Estimated from HbO/HbR)")
    plt.grid(alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.show()
    plt.close()

# 兼容“直接运行脚本”场景，确保能导入项目根目录下的 software 包。
PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

SCRIPT_DIR = Path(__file__).resolve().parent
raw_csv_path = SCRIPT_DIR / 'result_table' / '2026-04-23_19-57-29' / 'all_groups.csv'

df = pd.read_csv(raw_csv_path)

# 过滤数据
df = df[df["Wavelength"] != WAVELENGTH_OFF_CODE].reset_index(drop=True) # 去掉未点亮的数据

dt = df["Time (s)"].diff().mean() # 计算时间戳的差值的平均值
fs = 1.0 / dt if pd.notna(dt) and dt > 0 else 1.0 # 计算采样率
print(f"采样率: {fs} Hz")

plot_curves(df, title_suffix="原始数据")



# # 阈值截断
df = threshold_filter(df)

# 低通滤波
# filtered_df = butter_lowpass_filter(df=df, cutoff_hz=1.0, fs=fs, order=4)
filtered_df = df
# plot_curves(filtered_df)

# 分段 RMS（按波长连续段）
rms_df = sliding_window_rms(df=filtered_df)
print(f"分段 RMS 后样本数: {len(rms_df)}")
rms_dt = rms_df["Time (s)"].diff().mean() # 计算时间戳的差值的平均值
rms_fs = 1.0 / rms_dt if pd.notna(rms_dt) and rms_dt > 0 else 1.0 # 计算采样率
print(f"采样率: {rms_fs} Hz")
plot_curves(rms_df)

df_w1 = rms_df[rms_df["Wavelength"] == 1].reset_index(drop=True)
df_w2 = rms_df[rms_df["Wavelength"] == 2].reset_index(drop=True)
wavelength_1_array = df_w1["S1_D1"].to_numpy()
wavelength_2_array = df_w2["S1_D1"].to_numpy()
split_dt = df_w1["Time (s)"].diff().mean()
split_fs = 1.0 / split_dt if pd.notna(split_dt) and split_dt > 0 else 1.0
print(f"采样率: {split_fs} Hz")
n_samples = min(len(wavelength_1_array), len(wavelength_2_array))
wavelength_1_array = wavelength_1_array[:n_samples]
wavelength_2_array = wavelength_2_array[:n_samples]
samples = np.vstack(
    [
        wavelength_1_array,
        wavelength_2_array,
    ]
)
print('samples.shape: ', samples.shape)
b, a = butter(4, 1.0/(0.5*split_fs), btype='low', analog=False)
samples = filtfilt(b, a, samples, axis=1)
plot_array(samples,['Wavelength=1','Wavelength=2'])
delta_od = nsp.intensities_to_od_changes(samples)
delta_od_filt = smart_bandpass(delta_od, split_fs, lowcut=0.01, highcut=0.1, order=4)
ch_names, ch_wls, ch_dpfs, ch_distances = _channel_info()
delta_c, new_ch_names, new_ch_types = nsp.mbll(
        delta_od_filt,
        ch_names,
        ch_wls,
        ch_dpfs,
        ch_distances,
        unit='cm',
        table="wray"
    )
# CBSI 用于进一步抑制生理伪差，改善 HbO/HbR 的相关性。
# delta_c_corr, corr_ch_names, corr_ch_types = nproc.cbsi(delta_c, new_ch_names, new_ch_types)
plot_array(delta_c,new_ch_types)
plot_blood_oxygen_content(
    delta_c=delta_c,
    ch_types=new_ch_types,
    times=df_w1["Time (s)"].to_numpy(dtype=float),
)

