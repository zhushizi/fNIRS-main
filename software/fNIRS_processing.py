"""
fNIRS_processing.py
==================
该脚本从串口采集原始 ADC 数据并进行处理，
最终将结果输出为 CSV 文件。包含数据包解析、数据采集、
模式块交错，以及对数据集应用改进的 Beer-Lambert 定律（MBLL）
来计算血红蛋白浓度变化等功能。
"""

import csv
import time
import signal
import struct
import numpy as np
from tabulate import tabulate
import nirsimple.preprocessing as nsp
import nirsimple.processing as nproc
import pandas as pd
import serial
from config import SERIAL_PORT, BAUD_RATE, TIMEOUT
from scipy.signal import butter, sosfiltfilt, resample_poly, filtfilt

ser = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=TIMEOUT)

STOP_FLAG = False  # 用于停止采集循环的全局标志

def handle_stop_signal():
    """
    SIGUSR1 信号处理函数：将 STOP_FLAG 置为 True。
    用于让采集循环平滑停止。
    """
    global STOP_FLAG
    print("Received stop signal, setting stop_flag to True.")
    STOP_FLAG = True

# 注册 SIGUSR1 信号处理器
signal.signal(signal.SIGUSR1, handle_stop_signal)

def revert_inversion(df_in, zero_level=2050, out_csv="all_groups_no_inv.csv"):
    """
    将 Short/Long 列重新映射回原始 ADC 计数值，
    并将结果保存到 `out_csv`。
    """
    df = df_in.copy()
    reading_cols = [c for c in df.columns if ("Short" in c or "Long" in c)]
    df[reading_cols] = 2 * zero_level - df[reading_cols]
    df.to_csv(out_csv, index=False)
    print(f"✓ Wrote non-inverted data → {out_csv}")
    return df

def threshold_filter(df, lower_threshold=200, upper_threshold=4000, zero_level=2050, exclude_columns=None):
    """抑制数据表中的离群值。"""
    if exclude_columns is None:
        exclude_columns = []

    suppressed_df = df.copy()

    for col in df.columns:
        if col not in exclude_columns:
            suppressed_df[col] = np.where(
                (df[col] < lower_threshold) | (df[col] > upper_threshold),
                zero_level,
                df[col]
            )

    return suppressed_df

def butter_lowpass_filter(df, cutoff_hz, fs, order=4, exclude_columns=None):
    """对数据表指定列应用 Butterworth 低通滤波。"""
    if exclude_columns is None:
        exclude_columns = []

    filtered_df = df.copy()
    nyquist = 0.5 * fs
    normal_cutoff = cutoff_hz / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)

    for col in df.columns:
        if col not in exclude_columns:
            filtered_df[col] = filtfilt(b, a, df[col])

    return filtered_df

def sliding_window_rms(df, emitter_col="G0_Emitter", group_prefix="G", num_groups=8,
                         remove_dc=False, split_segments_in_half=False):
    """
    按发射器状态切换定义的分段进行 RMS 计算。
    """
    df_rms = df.copy()

    emitter_reference = df[emitter_col].values
    change_points = np.where(np.diff(emitter_reference) != 0)[0] + 1
    segments = np.split(np.arange(len(df)), change_points)

    if split_segments_in_half:
        split_segments = []
        for segment in segments:
            n = len(segment)
            if n == 0:
                continue
            mid = n // 2
            split_segments.append(segment[:mid])
            split_segments.append(segment[mid:])
        segments = split_segments

    groups = [f"{group_prefix}{i}" for i in range(num_groups)]

    for g in groups:
        short_col = f"{g}_Short"
        long1_col = f"{g}_Long1"
        long2_col = f"{g}_Long2"

        for segment in segments:
            if len(segment) == 0:
                continue
            segment_idx = segment.tolist()

            for col in [short_col, long1_col, long2_col]:
                segment_data = df.loc[segment_idx, col]
                if remove_dc:
                    segment_data = segment_data - segment_data.mean()
                rms_val = np.sqrt(np.mean(np.square(segment_data)))
                df_rms.loc[segment_idx, col] = rms_val

    return df_rms


def butter_bandpass_sos(lowcut, highcut, fs, order=4):
    """返回二阶节（SOS）形式的带通滤波器。"""
    nyq = 0.5 * fs
    sos = butter(order, [lowcut / nyq, highcut / nyq],
                 btype="band", output="sos")
    return sos

def smart_bandpass(data, fs,
                   lowcut=0.05, highcut=0.1, order=4,
                   target_fs=20.0):
    """
    沿时间轴执行零相位带通滤波。
    `data` 形状: (n_channels, n_timepoints)
    """
    # 降采样
    if fs > target_fs + 1: # 留一点裕量
        decim = int(round(fs / target_fs))
        fs_ds = fs / decim
        data_ds = resample_poly(data, up=1, down=decim, axis=1)
    else:
        decim, fs_ds, data_ds = 1, fs, data
    # 设计数值稳定的滤波器
    sos = butter_bandpass_sos(lowcut, highcut, fs_ds, order)
    # 零相位滤波
    data_bp = sosfiltfilt(sos, data_ds, axis=1, padtype="odd",
                          padlen=3 * (order + 1))
    # 若做过降采样，则升采样回原采样率
    if decim > 1:
        data_bp = resample_poly(data_bp, up=decim, down=1, axis=1)
    return data_bp

def parse_packet(data):
    """ 
    将 64 字节原始数据解析为 8×5 的传感器数组：
       [组 ID, Short, Long1, Long2, 发射器状态]。
    """
    ZERO_LEVEL = 2050
    parsed_data = np.zeros((8, 5), dtype=int)
    for i in range(8):
        offset = i * 8
        packet_identifier = data[offset]
        sensor_channel_1 = struct.unpack('>H', data[offset+1:offset+3])[0]
        sensor_channel_inv1 = 2 * ZERO_LEVEL - sensor_channel_1
        sensor_channel_2 = struct.unpack('>H', data[offset+3:offset+5])[0]
        sensor_channel_inv2 = 2 * ZERO_LEVEL - sensor_channel_2
        sensor_channel_3 = struct.unpack('>H', data[offset+5:offset+7])[0]
        sensor_channel_inv3 = 2 * ZERO_LEVEL - sensor_channel_3
        emitter_status    = data[offset+7]

        parsed_data[i] = [
            packet_identifier,
            sensor_channel_inv1,
            sensor_channel_inv2,
            sensor_channel_inv3,
            emitter_status
        ]
    return parsed_data

def capture_data(csv_filename, stop_on_enter=True):
    """
    采集 ADC 数据并写入 CSV 文件。
    
    参数:
      csv_filename (str): 输出 CSV 文件名。
      stop_on_enter (bool): 若为 True，循环会检测回车键以停止记录。
      external_stop_flag: 具有 is_set() 方法的对象（例如 threading.Event）。
                          若提供且 external_stop_flag.is_set() 为 True，
                          采集循环会停止。
    """
    global STOP_FLAG
    STOP_FLAG = False  # 启动采集前重置标志

    with open(csv_filename, mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        header = ["Time (s)"]
        for i in range(8):
            header += [f"G{i}_Short", f"G{i}_Long1", f"G{i}_Long2", f"G{i}_Emitter"]
        writer.writerow(header)

        print("Starting raw ADC logging (seconds elapsed)...")
        if stop_on_enter:
            print("Press Enter to stop logging and proceed to CSV processing.")

        start_time = time.time()

        try:
            while True:
                if STOP_FLAG:
                    print("Stop flag detected. Stopping capture...")
                    break
                data = ser.read(64)
                if len(data) == 64:
                    parsed_data = parse_packet(data)
                    elapsed_time = round(time.time() - start_time, 3)
                    flat_row = [elapsed_time]
                    for i in range(8):
                        flat_row += parsed_data[i][1:5].tolist()
                    writer.writerow(flat_row)
                    csvfile.flush()
                    print(f"{elapsed_time}s - Logged frame to CSV.")
                else:
                    print("No valid data received from the serial port.")
                    time.sleep(0.1)
        except Exception as e:
            print("An error occurred during logging:", str(e))


def interleave_mode_blocks(df, mode_col="G0_Emitter"):
    """
    按模式列对数据块进行交错重排。
    函数假设 DataFrame 中存在名为 mode_col 的列，
    每行该列值表示模式（1 或 2）。
    函数会生成新的 DataFrame：将每个 mode=1 的数据块
    与其后续的 mode=2 数据块交错排列。
    """
    # 按 mode 列变化点分组
    df["group"] = (df[mode_col] != df[mode_col].shift()).cumsum()

    # 将每个分组收集到列表
    blocks = []
    for _, block_df in df.groupby("group"):
        block_df = block_df.drop(columns="group")
        blocks.append(block_df)

    all_rows = []
    i = 0
    # 按两块一组处理；若最后有残留单块则丢弃
    while i < len(blocks) - 1:
        block1 = blocks[i].reset_index(drop=True)
        block2 = blocks[i + 1].reset_index(drop=True)

        n1, n2 = len(block1), len(block2)
        max_len = max(n1, n2)

        for j in range(max_len):
            # 从 block1 取行；越界时重复最后一行
            row1 = block1.loc[j] if j < n1 else block1.loc[n1 - 1]
            # 从 block2 取行；越界时重复最后一行
            row2 = block2.loc[j] if j < n2 else block2.loc[n2 - 1]

            all_rows.append(row1.copy())
            all_rows.append(row2.copy())

        i += 2

    final_df = pd.DataFrame(all_rows)
    if "group" in final_df.columns:
        final_df.drop(columns="group", inplace=True)
    return final_df

def build_channel_info(age, sd_short, sd_long):
    """
    构建通道名称、波长、DPF 和源探距离信息。
    共有 24 个物理通道（8 个传感器组 × 3 个探测器），
    每个通道在两个波长（660 nm 与 940 nm）下测量，
    因此总计 48 条通道数据。
    """
    physical_channels = [f"S{s}_D{d}"
                         for s in range(1, 9)      # 组号 1–8
                         for d in (1, 2, 3)]       # 探测器 1-3
    channel_names, ch_wls, ch_dpfs, ch_distances = [], [], [], []
    for name in physical_channels:
        det_num = int(name.split("_D")[1])
        dist    = sd_short if det_num == 1 else sd_long
        # 660nm
        channel_names.append(name)
        ch_wls.append(660.0)
        ch_dpfs.append(nsp.get_dpf(660.0, age))
        ch_distances.append(dist)
        # 940nm
        channel_names.append(name)
        ch_wls.append(940.0)
        ch_dpfs.append(nsp.get_dpf(940.0, age))
        ch_distances.append(dist)
    return channel_names, ch_wls, ch_dpfs, ch_distances

def combine_two_rows(row_mode1, row_mode2):
    """
    将 CSV 中两行数据（前一行为 mode=1，后一行为 mode=2）
    合并为一个 48 元素样本。

    假设每行结构如下：
      [Time, G0_Short, G0_Long1, G0_Long2, G0_Emitter,
             G1_Short, G1_Long1, G1_Long2, G1_Emitter, ...,
             G7_Short, G7_Long1, G7_Long2, G7_Emitter]

    对每个传感器组（0 到 7），提取 mode=1 行（视为 660 nm）的三路测量，
    以及 mode=2 行（视为 940 nm）的三路测量，
    再交错为 48 元素向量：
      [short_660, short_940, long1_660, long1_940, long2_660, long2_940] for each group.
    """
    sample = np.zeros(48, dtype=float)
    # 共 8 组；每组在原始行中占 4 列（不含时间列）
    for g in range(8):
        # mode=1 行（660 nm）
        short_660 = float(row_mode1[1 + 4*g])
        long1_660 = float(row_mode1[2 + 4*g])
        long2_660 = float(row_mode1[3 + 4*g])
        # mode=2 行（940 nm）
        short_940 = float(row_mode2[1 + 4*g])
        long1_940 = float(row_mode2[2 + 4*g])
        long2_940 = float(row_mode2[3 + 4*g])
        base = g * 6
        sample[base + 0] = short_660
        sample[base + 1] = short_940
        sample[base + 2] = long1_660
        sample[base + 3] = long1_940
        sample[base + 4] = long2_660
        sample[base + 5] = long2_940
    return sample

def process_csv_dataset(
    input_csv,
    output_csv,
    age=22,
    sd_short=0.6,
    sd_long=3.5,
    molar_ext_coeff_table='wray',
    bp_low=0.05,
    bp_high=0.1,
    bp_order=4
):
    """
    对 fNIRS CSV 数据集进行后处理。

    期望 CSV 具有表头，并包含如下列：
      Time, G0_Short, G0_Long1, G0_Long2, G0_Emitter, 
      G1_Short, G1_Long1, G1_Long2, G1_Emitter, ..., 
      G7_Short, G7_Long1, G7_Long2, G7_Emitter.

    行数据在 mode 1（660 nm）与 mode 2（940 nm）之间交替。
    函数将每个 mode 1 行与其后续 mode 2 行配对，并进行 OD 转换、
    MBLL 与 CBSI 处理，以计算血红蛋白浓度变化。

    输出 CSV 列格式为：
      Time, [for each processed channel: "{ChannelName}_{Type}"],
    其中 Type 为 "hbo" 或 "hbr"。
    """
    # 读取 CSV 数据（跳过表头）
    with open(input_csv, newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        data_lines = list(reader)

    # 将每行转换为浮点数组
    data_matrix = np.array([[float(x) for x in line] for line in data_lines if len(line) >= 33])
    num_rows = data_matrix.shape[0]
    if num_rows < 2:
        print("Insufficient data rows for processing.")
        return

    # 按两行配对：假设 row0 为 mode1、row1 为 mode2、row2 为 mode1，依此类推
    samples = []
    times = []
    i = 0
    while i < num_rows - 1:
        row_mode1 = data_matrix[i, :]
        row_mode2 = data_matrix[i+1, :]
        t_val = row_mode1[0]  # 使用 mode1 行的时间戳
        sample = combine_two_rows(row_mode1, row_mode2)
        samples.append(sample)
        times.append(t_val)
        i += 2

    samples = np.array(samples).T  # 形状: (48, N)

    # 根据既定模型构建通道信息
    channel_names, ch_wls, ch_dpfs, ch_distances = build_channel_info(age, sd_short, sd_long)

    # 对整段数据执行 OD 转换
    delta_od = nsp.intensities_to_od_changes(samples)

    # 对 OD 变化进行带通滤波
    # 高通 ≥ 0.1 Hz 去除漂移；低通 ≤ 0.05 Hz 去除脉搏与噪声。
    dt  = np.mean(np.diff(times))
    fs  = 1.0 / dt
    delta_od_filt = smart_bandpass(delta_od, fs, lowcut=bp_low, highcut=bp_high, order=bp_order)

    # 执行 MBLL 以计算浓度变化
    delta_c, new_ch_names, new_ch_types = nsp.mbll(
        delta_od_filt,
        channel_names,
        ch_wls,
        ch_dpfs,
        ch_distances,
        unit='cm',
        table=molar_ext_coeff_table
    )
    # 执行 CBSI 进行信号改进
    delta_c_corr, corr_ch_names, corr_ch_types = nproc.cbsi(delta_c, new_ch_names, new_ch_types)
    # delta_c_corr 形状为 (48, N)

    # 使用处理后的通道名和类型生成输出 CSV 表头
    processed_headers = [f"{name}_{ctype}" for name, ctype in zip(corr_ch_names, corr_ch_types)]
    header_out = ["Time"] + processed_headers

    # 写入输出 CSV：每个时间点一行
    with open(output_csv, "w", newline="") as f_out:
        writer = csv.writer(f_out)
        writer.writerow(header_out)
        n_cols = min(len(times), delta_c_corr.shape[1])
        for col_idx in range(n_cols):
            time_val = times[col_idx]
            row_values = [time_val] + list(delta_c_corr[:, col_idx])
            writer.writerow(row_values)

    print(f"Post-processing complete. Output saved to '{output_csv}'.")

    # 打印最后一个时间点的结果表用于核对
    last_sample = delta_c_corr[:, -1]
    table_data = []
    for i, ch in enumerate(corr_ch_names):
        table_data.append([ch, corr_ch_types[i], f"{last_sample[i]:.4e}"])
    print("\nExample: Processed concentrations at the final time sample:")
    print(tabulate(table_data, headers=["Channel", "Type", "Concentration"]))


# ------------------ Main ------------------
if __name__ == '__main__':

    # 采集数据
    STOP_FLAG=None # 可由 GUI 置为 1 以停止处理
    capture_data("all_groups.csv", stop_on_enter=True)

    # 读取 CSV 文件
    df = pd.read_csv("all_groups.csv")

    # 反转恢复（如有需要）
    #df = revert_inversion(df)

    # 从数据中估计采样率
    time = df['Time (s)']
    dt = time.diff().mean()
    fs = 1.0 / dt

    # 数据整理与处理
    # 1) 完全忽略原始时间戳：若存在则删除。
    if "Time (s)" in df.columns:
        df.drop(columns=["Time (s)"], inplace=True)

    # 2) 过滤原始模拟量数据
    exclude_cols = [c for c in df.columns if 'Emitter' in c]
    df = threshold_filter(df, exclude_columns=exclude_cols)
    df = butter_lowpass_filter(df=df, cutoff_hz=1.0, fs=fs, order=4, exclude_columns=exclude_cols)

    # 3) 将原始模拟量转换为强度表示
    df = sliding_window_rms(df=df)

    # 4) 按模式列交错重排数据块
    final_df = interleave_mode_blocks(df, mode_col="G0_Emitter")

    # 5) 以固定步长分配新时间戳（本例为 0.001 s）
    INCREMENT = 0.001
    final_df.insert(0, "Time (s)", [i * INCREMENT for i in range(len(final_df))])

    # 6) 对新时间戳四舍五入，避免浮点误差伪影
    final_df["Time (s)"] = final_df["Time (s)"].round(3)

    # 7) 将最终 DataFrame 写入 CSV
    final_df.to_csv("interleaved_output.csv", index=False)

    # 8) 输出结果 DataFrame
    print(final_df.head(20))

    # 9) 对采集数据执行后处理
    INPUT_CSV = "interleaved_output.csv"  
    OUTPUT_CSV = "processed_output.csv"
    process_csv_dataset(INPUT_CSV, OUTPUT_CSV)
