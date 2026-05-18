"""
单通道 fNIRS 采集与处理主脚本。

完整链路如下：
1. 通过 26B 协议采集原始单通道数据
2. 写入 all_groups.csv
3. 对单通道信号做阈值/低通/RMS 预处理
4. 按 config.WAVELENGTH_CHANNELS 聚合成多列光强
5. 计算 OD -> MBLL -> CBSI
6. 输出 processed_output.csv
"""

from __future__ import annotations

import csv
import os
import threading
import time
from datetime import datetime

import nirsimple.preprocessing as nsp
import nirsimple.processing as nproc
import numpy as np
import pandas as pd
import serial
from scipy.signal import butter, filtfilt, resample_poly, sosfiltfilt
from tabulate import tabulate

from config import (
    ACK_TIMEOUT_SECONDS,
    BAUD_RATE,
    BP_HIGH_HZ,
    BP_LOW_HZ,
    BP_ORDER,
    BP_TARGET_FS_HZ,
    CHANNEL_NAME,
    DEFAULT_INTENSITY_MA,
    DEFAULT_STREAM_ENABLED,
    INTENSITY_COLUMNS,
    MBLL_DEFAULT_AGE,
    RAW_OUTPUT_CSV,
    SERIAL_PORT,
    SOURCE_DETECTOR_DISTANCE_CM,
    TIMEOUT,
    WAVELENGTH_CHANNELS,
    WAVELENGTH_OFF_CODE,
    mbll_wavelengths_nm,
    wavelength_channel_by_code,
)
from protocol import (
    FrameReader,
    build_command_frame,
    parse_data_frame,
    send_frame_with_ack,
)


def open_serial() -> serial.Serial:
    """按配置打开串口。"""
    return serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=TIMEOUT)


def _start_enter_listener(stop_event: threading.Event) -> threading.Thread:
    """起一个后台线程监听回车，用于结束采集。"""
    def _wait_for_enter() -> None:
        try:
            input("Press Enter to stop capture and continue processing.\n")
            stop_event.set()
        except EOFError:
            return

    thread = threading.Thread(target=_wait_for_enter, daemon=True)
    thread.start()
    return thread


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
    padlen = min(len(df) - 1, 3 * (max(len(a), len(b)) - 1)) # 计算滤波器长度
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

    wavelength_reference = df[wavelength_col].values # 波长编号序列
    change_points = np.where(np.diff(wavelength_reference) != 0)[0] + 1 # 找到波长变化点
    segments = np.split(np.arange(len(df)), change_points) # 按波长变化点分割数据
    rows = [] # 存储每一段数据的 RMS 代表值

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


def process_csv_dataset(
    input_csv: str,
    output_csv: str,
    age: int = MBLL_DEFAULT_AGE,
    source_detector_distance_cm: float = SOURCE_DETECTOR_DISTANCE_CM,
    molar_ext_coeff_table: str = "wray",
    bp_low: float = BP_LOW_HZ,
    bp_high: float = BP_HIGH_HZ,
    bp_order: int = BP_ORDER,
) -> None:
    """从配对后的 CSV 计算最终的 HbO/HbR。"""
    df = pd.read_csv(input_csv)
    required_cols = ["Time (s)", *INTENSITY_COLUMNS]
    if df.empty or any(col not in df.columns for col in required_cols):
        print("Insufficient or invalid interleaved data for processing.")
        return

    times = df["Time (s)"].to_numpy(dtype=float)
    if len(times) < 2:
        print("Need at least two time samples to run MBLL.")
        return

    samples = stack_intensities_for_mbll(df)

    channel_names, ch_wls, ch_dpfs, ch_distances = build_channel_info(age, source_detector_distance_cm)
    # 先把光强转成 OD，再做带通和 MBLL。
    delta_od = nsp.intensities_to_od_changes(samples)

    dt = np.mean(np.diff(times))
    fs = 1.0 / dt if dt > 0 else 1.0
    # 带通滤波
    delta_od_filt = smart_bandpass(delta_od, fs, lowcut=bp_low, highcut=bp_high, order=bp_order)

    # MBLL 计算
    delta_c, new_ch_names, new_ch_types = nsp.mbll(
        delta_od_filt,
        channel_names,
        ch_wls,
        ch_dpfs,
        ch_distances,
        unit="cm",
        table=molar_ext_coeff_table,
    )
    # CBSI 用于进一步抑制生理伪差，改善 HbO/HbR 的相关性。
    delta_c_corr, corr_ch_names, corr_ch_types = nproc.cbsi(delta_c, new_ch_names, new_ch_types)

    # 写入文件
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


def capture_data(
    csv_filename: str = RAW_OUTPUT_CSV,
    stop_on_enter: bool = True,
    duration_seconds: float | None = None,
    intensity_ma: int = DEFAULT_INTENSITY_MA,
) -> None:
    """
    采集单通道原始数据并写 CSV。

    每收到一帧 0x02 数据帧就：
    1. 回 ACK
    2. 解析波长/传感器/采样值
    3. 记录到 all_groups.csv
    """
    ser = open_serial() # 打开串口
    reader = FrameReader(ser) # 创建帧读取器
    stop_event = threading.Event() 
    listener = _start_enter_listener(stop_event) if stop_on_enter else None # 创建监听器

    try:
        ser.reset_input_buffer() # 清空串口接收缓冲区里的残留数据
        started = send_frame_with_ack( # 发送启动命令
            ser,
            reader,
            build_command_frame(DEFAULT_STREAM_ENABLED, intensity_ma),
        )
        if not started:
            raise RuntimeError(
                f"Failed to start stream: no ACK within {ACK_TIMEOUT_SECONDS * 1000:.0f} ms."
            )

        with open(csv_filename, mode="w", newline="", encoding="utf-8") as csvfile:
            writer = csv.writer(csvfile)
            # 单通道原始采样表：时间 + 传感器号 + 数值 + 波长编号
            writer.writerow(["Time (s)", "SensorId", CHANNEL_NAME, "Wavelength"])

            print("Starting single-channel raw ADC logging (seconds elapsed)...")
            start_time = time.time()

            while True:
                if stop_event.is_set():
                    print("Stop requested by user.")
                    break
                if duration_seconds is not None and (time.time() - start_time) >= duration_seconds:
                    print("Capture duration reached.")
                    break

                frame = reader.read_frame(timeout_seconds=TIMEOUT)
                if frame is None:
                    continue
                if frame.frame_type != 0x02:
                    continue

                sample = parse_data_frame(frame)
                elapsed_time = round(time.time() - start_time, 6)
                writer.writerow(
                    [
                        elapsed_time,
                        sample.sensor_id,
                        sample.value,
                        sample.wavelength_code,
                    ]
                )
                csvfile.flush()
                print(
                    f"{elapsed_time:.3f}s - value={sample.value} "
                    f"wl={int(sample.wavelength_code)} sensor={int(sample.sensor_id)}"
                )
    finally:
        try:
            send_frame_with_ack(ser, reader, build_command_frame(False, intensity_ma))
        except Exception:
            pass
        ser.close()
        if listener is not None and listener.is_alive():
            stop_event.set()


# 三个结果表统一放在此目录下，每次运行占一个以时间命名的子文件夹
RESULT_TABLE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "result_table")


def run_pipeline() -> None:
    """一键跑完整链路：采集 -> 预处理 -> 配对 -> MBLL -> CSV 输出。三个表写入 result_table/<时间>/。"""
    os.makedirs(RESULT_TABLE_DIR, exist_ok=True)
    run_id = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output_dir = os.path.join(RESULT_TABLE_DIR, run_id)
    os.makedirs(output_dir, exist_ok=True)
    raw_path = os.path.join(output_dir, "all_groups.csv")
    interleaved_path = os.path.join(output_dir, "interleaved_output.csv")
    processed_path = os.path.join(output_dir, "processed_output.csv")
    print(f"本次结果将保存到: {output_dir}")

    # 采集数据
    capture_data(csv_filename=raw_path, stop_on_enter=True)

    # 读取数据
    df = pd.read_csv(raw_path)
    if df.empty or len(df) < 2:
        print("No enough raw rows captured; skipping processing.")
        return

    # 过滤数据
    n_raw = len(df)
    df = df[df["Wavelength"] != WAVELENGTH_OFF_CODE].reset_index(drop=True) # 去掉未点亮的数据
    dropped = n_raw - len(df) # 计算去掉的数据量
    if dropped:
        print(f"Dropped {dropped} raw row(s) with Wavelength=OFF (0x00); not used for dual-wavelength pairing.")

    if df.empty or len(df) < 2:
        print("No enough non-OFF samples after filtering; skipping processing.")
        return

    dt = df["Time (s)"].diff().mean() # 计算时间戳的差值的平均值
    fs = 1.0 / dt if pd.notna(dt) and dt > 0 else 1.0 # 计算采样率
    print(f"采样率: {fs} Hz")

    # # 阈值截断
    # df = threshold_filter(df)

    # 低通滤波
    df = butter_lowpass_filter(df=df, cutoff_hz=1.0, fs=fs, order=4)

    # 分段 RMS
    df = sliding_window_rms(df=df)

    # 多波长周期聚合（列名由 config.WAVELENGTH_CHANNELS 决定）
    final_df = aggregate_wavelength_cycles(df, mode_col="Wavelength")
    if final_df.empty:
        print("No complete wavelength cycles were formed; skipping MBLL.")
        return

    # 按配对后的真实平均间隔重建等间隔时间戳，避免采集抖动影响后续基于 fs 的滤波。
    pair_times = final_df["Time (s)"].to_numpy(dtype=float)
    pair_dt = np.diff(pair_times)
    valid_pair_dt = pair_dt[np.isfinite(pair_dt) & (pair_dt > 0)]
    increment = float(np.mean(valid_pair_dt)) if valid_pair_dt.size else 0.001
    if "Time (s)" in final_df.columns:
        final_df = final_df.drop(columns=["Time (s)"])
    final_df.insert(0, "Time (s)", [i * increment for i in range(len(final_df))])

    # 统一保留6位小数，减少浮点表示伪差。
    final_df["Time (s)"] = final_df["Time (s)"].round(6)

    # 写入文件
    final_df.to_csv(interleaved_path, index=False)
    print(final_df.head(20))

    # 计算最终的 HbO/HbR
    process_csv_dataset(interleaved_path, processed_path)


if __name__ == "__main__":
    run_pipeline()
