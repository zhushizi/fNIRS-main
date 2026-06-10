"""
单通道 fNIRS 采集与处理主脚本。

完整链路如下：
1. 通过 26B 协议采集原始单通道数据
2. 写入 all_groups.csv
3. 对单通道信号做阈值/低通/RMS 预处理
4. 按 config.WAVELENGTH_CHANNELS 聚合成多列光强
5. 计算 OD -> MBLL -> CBSI
6. 输出 processed_output.csv

运行模式（由 capture_data / run_pipeline 参数决定）：
- 直连串口：PC 发启停命令，按 Enter 结束采集；采集中无在线分析
- TCP 桥接：安卓独占 UART，PC 被动收 serial_data；采集中可并行在线分析并回传 live_analysis_batch
  采集结束后无论哪种模式，run_pipeline 都会对全段 CSV 做离线 MBLL（最终以离线结果为准）
"""

from __future__ import annotations

import argparse
import csv
import os
import threading
import time
from collections import deque
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
    HOST_TCP_DEFAULT_PORT,
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
from host_tcp_protocol import HostTcpSerialBridge


# 离线/在线分析共用的结果字典类型（含 ok、mean_hbo、mean_hbr、message 等）
AnalysisResult = dict[str, object]

# ---------- 在线分析参数（仅 TCP 模式启用 OnlineAnalysisWorker） ----------
ONLINE_WINDOW_SECONDS = 30.0          # 滑动分析窗口长度（秒）
ONLINE_UPDATE_INTERVAL_SECONDS = 1.0  # 后台线程分析/发送间隔（秒）
ONLINE_BUFFER_RETENTION_SECONDS = ONLINE_WINDOW_SECONDS * 2.0  # 原始缓冲保留时长，约为窗口 2 倍
ONLINE_MIN_INTERLEAVED_POINTS = max(16, 3 * (BP_ORDER + 1) + 1)  # 带通滤波所需最少配对点数


def open_serial(serial_port: str = SERIAL_PORT) -> serial.Serial:
    """按配置打开串口。"""
    return serial.Serial(serial_port, baudrate=BAUD_RATE, timeout=TIMEOUT)


def _start_enter_listener(stop_event: threading.Event) -> threading.Thread:
    """起一个后台线程监听回车，用于结束采集。"""
    def _wait_for_enter() -> None:
        try:
            input("Press Enter to stop capture and continue processing.\n")
            stop_event.set()  # 通知主采集循环退出
        except EOFError:
            return

    thread = threading.Thread(target=_wait_for_enter, daemon=True)
    thread.start()
    return thread


class OnlineSampleBuffer:
    """在线分析器所使用的线程安全原始采样缓冲区。"""

    def __init__(self, retention_seconds: float = ONLINE_BUFFER_RETENTION_SECONDS) -> None:
        self.retention_seconds = retention_seconds
        # 每行：(相对采集起点时间, 传感器号, 光强值, 波长协议码)
        self._rows: deque[tuple[float, int, float, int]] = deque()
        self._lock = threading.Lock()  # 主线程 append 与在线线程 snapshot 互斥

    def append(
        self,
        elapsed_time: float,
        sensor_id: int,
        value: float,
        wavelength_code: int,
    ) -> None:
        with self._lock:
            self._rows.append((elapsed_time, sensor_id, value, wavelength_code))
            # 丢弃过旧数据，控制内存占用
            cutoff = elapsed_time - self.retention_seconds
            while self._rows and self._rows[0][0] < cutoff:
                self._rows.popleft()

    def snapshot_recent(self, window_seconds: float) -> pd.DataFrame:
        """复制最近 window_seconds 内的原始样本，供在线分析线程使用。"""
        with self._lock:
            if not self._rows:
                rows: list[tuple[float, int, float, int]] = []
            else:
                latest_time = self._rows[-1][0]
                cutoff = latest_time - window_seconds
                rows = [row for row in self._rows if row[0] >= cutoff]

        return pd.DataFrame(
            rows,
            columns=["Time (s)", "SensorId", CHANNEL_NAME, "Wavelength"],
        )


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
        return pd.DataFrame(columns=["Time (s)", *INTENSITY_COLUMNS])

    df = raw_df.copy()
    df = df[df["Wavelength"] != WAVELENGTH_OFF_CODE].reset_index(drop=True)
    if df.empty or len(df) < 2:
        return pd.DataFrame(columns=["Time (s)", *INTENSITY_COLUMNS])

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
    delta_c_corr, _corr_ch_names, corr_ch_types = nproc.cbsi(delta_c, new_ch_names, new_ch_types)
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
) -> AnalysisResult:
    """
    离线终算入口：从 interleaved CSV 计算 HbO/HbR 并写入 processed_output.csv。

    算法与 calculate_concentration_series 相同（OD → 带通 → MBLL → CBSI），
    但面向全段文件，并返回均值摘要供 analysis_result 回传安卓。
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
    return summarize_processed_concentrations(delta_c_corr, corr_ch_types)


def summarize_processed_concentrations(
    delta_c_corr: np.ndarray,
    channel_types: list[str],
) -> AnalysisResult:
    """为安卓端测试显示提取本次 HbO/HbR 平均值。"""
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

    return {
        "ok": True,
        "mean_hbo": float(np.mean(hbo)),
        "mean_hbr": float(np.mean(hbr)),
        "sample_count": int(min(hbo.size, hbr.size)),
    }


def send_analysis_result_to_android(
    bridge: HostTcpSerialBridge | None,
    result: AnalysisResult,
) -> None:
    """把本次分析摘要回传给安卓；非 TCP 模式不做任何事。"""
    if bridge is None:
        return
    mean_hbo = result.get("mean_hbo")
    mean_hbr = result.get("mean_hbr")
    message = result.get("message")
    try:
        bridge.send_analysis_result(
            ok=bool(result.get("ok")),
            mean_hbo=float(mean_hbo) if isinstance(mean_hbo, (int, float)) else None,
            mean_hbr=float(mean_hbr) if isinstance(mean_hbr, (int, float)) else None,
            sample_count=int(result.get("sample_count", 0)),
            message=str(message) if message is not None else None,
        )
    except Exception as exc:
        print(f"Failed to send analysis_result to Android: {exc}")


class OnlineAnalysisWorker:
    """
    在线分析后台线程：滑动窗口重算 HbO/HbR，增量发送 live_analysis_batch 给安卓。

    与离线分析并行运行；仅用于实时预览，数值以采集结束后的 process_csv_dataset 为准。
    """

    def __init__(
        self,
        buffer: OnlineSampleBuffer,
        bridge: HostTcpSerialBridge,
        window_seconds: float = ONLINE_WINDOW_SECONDS,
        update_interval_seconds: float = ONLINE_UPDATE_INTERVAL_SECONDS,
    ) -> None:
        self.buffer = buffer
        self.bridge = bridge
        self.window_seconds = window_seconds
        self.update_interval_seconds = update_interval_seconds
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.last_sent_time = -float("inf")  # 已发送曲线的最后一个时间点，避免重复发包

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        """采集结束时由 capture_data 的 finally 调用，停止后台分析线程。"""
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=max(1.0, self.update_interval_seconds * 2.0))

    def _run(self) -> None:
        print(
            "Online analysis enabled: "
            f"window={self.window_seconds:.1f}s interval={self.update_interval_seconds:.1f}s."
        )
        # wait(timeout) 兼作定时器：每 update_interval_seconds 触发一次分析
        while not self.stop_event.wait(self.update_interval_seconds):
            if getattr(self.bridge, "closed", False):
                break
            try:
                self._analyze_and_send_once()
            except Exception as exc:
                print(f"Online analysis skipped one batch: {exc}")

    def _analyze_and_send_once(self) -> None:
        """单次在线分析：取 30s 窗口 → 预处理 → MBLL → 只发送新增点。"""
        raw_df = self.buffer.snapshot_recent(self.window_seconds)
        if raw_df.empty:
            return
        window_span = float(raw_df["Time (s)"].iloc[-1] - raw_df["Time (s)"].iloc[0])
        # 窗口未满（约前 30s）时不发送，避免滤波/MBLL 不稳定
        min_window_span = max(0.0, self.window_seconds - self.update_interval_seconds * 0.5)
        if window_span < min_window_span:
            return

        # 预处理：低通 → RMS → 波长配对（保留绝对时间轴）
        interleaved_df = prepare_interleaved_dataframe(raw_df, start_at_zero=False)
        if interleaved_df.empty or len(interleaved_df) < ONLINE_MIN_INTERLEAVED_POINTS:
            return

        # MBLL 浓度计算
        series = calculate_concentration_series(interleaved_df)
        if series is None:
            return
        times, hbo, hbr = series

        # 整窗重算但只增量发送，安卓端按 times 追加曲线
        new_mask = times > self.last_sent_time + 1e-9
        if not np.any(new_mask):
            return

        times_new = times[new_mask]
        hbo_new = hbo[new_mask]
        hbr_new = hbr[new_mask]
        self.last_sent_time = float(times_new[-1])
        self.bridge.send_live_analysis_batch(
            times=[float(x) for x in times_new],
            hbo=[float(x) for x in hbo_new],
            hbr=[float(x) for x in hbr_new],
            window_start_s=float(interleaved_df["Time (s)"].iloc[0]),
            window_end_s=float(interleaved_df["Time (s)"].iloc[-1]),
            ok=True,
        )


def capture_data(
    csv_filename: str = RAW_OUTPUT_CSV,
    stop_on_enter: bool = True,
    duration_seconds: float | None = None,
    intensity_ma: int = DEFAULT_INTENSITY_MA,
    serial_port: str = SERIAL_PORT,
    tcp_port: int | None = None,
    tcp_debug: bool = False,
    online_analysis_enabled: bool = True,
) -> HostTcpSerialBridge | None:
    """
    采集单通道原始数据并写 CSV。

    传输模式（二选一）：
    - tcp_port=None：直连串口，PC 发启停命令
    - tcp_port 有值：TCP 桥接，数据来自安卓 serial_data

    每收到一帧 0x02 数据帧就：
    1. 回 ACK（TCP 模式下 ACK 仍经 ser 写回，由桥接层转发）
    2. 解析波长/传感器/采样值
    3. 记录到 all_groups.csv；TCP 模式下同时写入 online_buffer 供在线分析
    """
    tcp_mode = tcp_port is not None
    # ---------- 选择传输层：真串口 或 TCP 桥（二者均实现 read/in_waiting/close） ----------
    if not tcp_mode:
        ser = open_serial(serial_port)
        print(f"Using direct serial mode on {serial_port}.")
    else:
        ser = HostTcpSerialBridge(port=tcp_port, timeout=TIMEOUT, debug=tcp_debug)
        print(f"Using Android TCP bridge mode on port {tcp_port}.")
    reader = FrameReader(ser)
    stop_event = threading.Event()
    # 直连串口默认监听 Enter；TCP 模式由 run_pipeline 传入 stop_on_enter=False
    listener = _start_enter_listener(stop_event) if stop_on_enter else None
    online_buffer: OnlineSampleBuffer | None = None
    online_worker: OnlineAnalysisWorker | None = None
    # 仅 TCP 模式启动在线分析后台线程
    if tcp_mode and online_analysis_enabled:
        online_buffer = OnlineSampleBuffer()
        online_worker = OnlineAnalysisWorker(online_buffer, ser)
        online_worker.start()

    try:
        ser.reset_input_buffer()
        # ---------- 启流：串口模式 PC 主动发命令；TCP 模式等待安卓推 serial_data ----------
        if not tcp_mode:
            started = send_frame_with_ack(
                ser,
                reader,
                build_command_frame(DEFAULT_STREAM_ENABLED, intensity_ma),
            )
            if not started:
                raise RuntimeError(
                    f"Failed to start stream: no ACK within {ACK_TIMEOUT_SECONDS * 1000:.0f} ms."
                )
        else:
            print("TCP passive mode: Android controls UART start/stop; waiting for serial_data.")

        # ---------- 主采集循环（与在线分析线程并行） ----------
        with open(csv_filename, mode="w", newline="", encoding="utf-8") as csvfile:
            writer = csv.writer(csvfile)
            # 单通道原始采样表：时间 + 传感器号 + 数值 + 波长编号
            writer.writerow(["Time (s)", "SensorId", CHANNEL_NAME, "Wavelength"])

            print("Starting single-channel raw ADC logging (seconds elapsed)...")
            start_time = time.time()

            while True:
                # 退出条件 1：用户按 Enter（仅 stop_on_enter=True 时）
                if stop_event.is_set():
                    print("Stop requested by user.")
                    break
                # 退出条件 2：达到指定采集时长
                if duration_seconds is not None and (time.time() - start_time) >= duration_seconds:
                    print("Capture duration reached.")
                    break

                frame = reader.read_frame(timeout_seconds=TIMEOUT)
                if frame is None:
                    # 退出条件 3：安卓 TCP bye（capture_finished）
                    if tcp_mode and getattr(ser, "capture_finished", False):
                        print("Android requested capture finish; starting analysis.")
                        break
                    # 退出条件 4：串口/TCP 连接已关闭
                    if getattr(ser, "closed", False) or not getattr(ser, "is_open", True):
                        print("Capture transport closed.")
                        break
                    continue
                if frame.frame_type != 0x02:
                    continue

                sample = parse_data_frame(frame)
                elapsed_time = round(time.time() - start_time, 6)
                # 同步喂给在线分析缓冲（与写 CSV 并行）
                if online_buffer is not None:
                    online_buffer.append(
                        elapsed_time,
                        sample.sensor_id,
                        sample.value,
                        sample.wavelength_code,
                    )
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
        if online_worker is not None:
            online_worker.stop()
        # 串口模式：发停流命令并关闭端口；TCP 模式保留 bridge 供 run_pipeline 发 analysis_result
        if not tcp_mode:
            try:
                send_frame_with_ack(ser, reader, build_command_frame(False, intensity_ma))
            except Exception:
                pass
            ser.close()
        if listener is not None and listener.is_alive():
            stop_event.set()
    return ser if tcp_mode else None  # TCP 时返回 bridge，离线分析完成后由 run_pipeline 关闭


# 三个结果表统一放在此目录下，每次运行占一个以时间命名的子文件夹
RESULT_TABLE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "result_table")


def run_pipeline(
    serial_port: str = SERIAL_PORT,
    tcp_port: int | None = None,
    tcp_debug: bool = False,
) -> None:
    """一键跑完整链路：采集 -> 预处理 -> 配对 -> MBLL -> CSV 输出。三个表写入 result_table/<时间>/。"""
    os.makedirs(RESULT_TABLE_DIR, exist_ok=True)
    run_id = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output_dir = os.path.join(RESULT_TABLE_DIR, run_id)
    os.makedirs(output_dir, exist_ok=True)
    raw_path = os.path.join(output_dir, "all_groups.csv")
    interleaved_path = os.path.join(output_dir, "interleaved_output.csv")
    processed_path = os.path.join(output_dir, "processed_output.csv")
    print(f"本次结果将保存到: {output_dir}")
    tcp_bridge: HostTcpSerialBridge | None = None

    try:
        # ========== 阶段 1：采集（采集中 TCP 模式可并行在线分析） ==========
        tcp_bridge = capture_data(
            csv_filename=raw_path,
            stop_on_enter=(tcp_port is None),
            serial_port=serial_port,
            tcp_port=tcp_port,
            tcp_debug=tcp_debug,
        )

        # ========== 阶段 2：离线预处理 + MBLL（全段 CSV，与在线分析独立） ==========
        df = pd.read_csv(raw_path)
        if df.empty or len(df) < 2:
            result = {"ok": False, "message": "No enough raw rows captured; skipping processing."}
            print(result["message"])
            send_analysis_result_to_android(tcp_bridge, result)
            return

        # 以下预处理步骤与 prepare_interleaved_dataframe 相同，但未调用该函数；
        # 时间轴从 0 起编，与在线 start_at_zero=False 略有差异。
        n_raw = len(df)
        df = df[df["Wavelength"] != WAVELENGTH_OFF_CODE].reset_index(drop=True)
        dropped = n_raw - len(df)
        if dropped:
            print(f"Dropped {dropped} raw row(s) with Wavelength=OFF (0x00); not used for dual-wavelength pairing.")

        if df.empty or len(df) < 2:
            result = {"ok": False, "message": "No enough non-OFF samples after filtering; skipping processing."}
            print(result["message"])
            send_analysis_result_to_android(tcp_bridge, result)
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
            result = {"ok": False, "message": "No complete wavelength cycles were formed; skipping MBLL."}
            print(result["message"])
            send_analysis_result_to_android(tcp_bridge, result)
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

        # 离线 MBLL：写 processed_output.csv，并汇总 HbO/HbR 均值
        result = process_csv_dataset(interleaved_path, processed_path)
        # TCP 模式：把离线终算摘要发回安卓（analysis_result）
        send_analysis_result_to_android(tcp_bridge, result)
    finally:
        if tcp_bridge is not None:
            tcp_bridge.close()


def parse_args() -> argparse.Namespace:
    """解析命令行：-port 直连串口，-tcp_port TCP 桥接（二者通常只选其一）。"""
    parser = argparse.ArgumentParser(description="Run fNIRS capture and processing pipeline.")
    parser.add_argument(
        "-port",
        "--port",
        dest="serial_port",
        nargs="?",
        const=SERIAL_PORT,
        default=SERIAL_PORT,
        help=(
            "Use the existing direct serial path. Optionally pass a serial port, "
            f"for example -port COM6. Default: {SERIAL_PORT}."
        ),
    )
    parser.add_argument(
        "-tcp_port",
        "--tcp_port",
        type=int,
        nargs="?",
        const=HOST_TCP_DEFAULT_PORT,
        default=None,
        help=(
            "Use Android Host TCP Protocol bridge mode on this listening port. "
            f"Recommended/default protocol port: {HOST_TCP_DEFAULT_PORT}."
        ),
    )
    parser.add_argument(
        "--tcp_debug",
        action="store_true",
        help="Print TCP protocol rx/tx logs (type/seq/byte_len) in bridge mode.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    try:
        # 入口：采集 + 离线分析一条龙
        run_pipeline(
            serial_port=args.serial_port,
            tcp_port=args.tcp_port,
            tcp_debug=args.tcp_debug,
        )
    except KeyboardInterrupt:
        print("\nStopped by user.")
