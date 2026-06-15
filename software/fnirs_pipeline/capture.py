"""安卓 TCP 桥接下的原始数据采集。"""

from __future__ import annotations

import csv
import time
from pathlib import Path

from config import (
    ANDROID_LIVE_OUTPUT_CSV,
    CHANNEL_NAME,
    HOST_TCP_DEFAULT_PORT,
    RAW_OUTPUT_CSV,
    TIMEOUT,
)
from online_android import (
    AnalysisResult,
    AndroidReporter,
    HostTcpSerialBridge,
    create_online_session,
)
from protocol import FrameReader, parse_data_frame

from .mbll import calculate_concentration_series
from .preprocessing import prepare_interleaved_dataframe


def send_analysis_result_to_android(
    bridge: HostTcpSerialBridge,
    result: AnalysisResult,
) -> None:
    """把本次分析摘要回传给安卓。"""
    AndroidReporter(bridge).send_final_result(result)


def capture_data(
    csv_filename: str = RAW_OUTPUT_CSV,
    duration_seconds: float | None = None,
    tcp_port: int = HOST_TCP_DEFAULT_PORT,
    tcp_debug: bool = False,
    live_plot: bool = False,
) -> HostTcpSerialBridge:
    """
    通过安卓 TCP 桥采集单通道原始数据并写 CSV。

    安卓独占 UART 启停；PC 被动接收 serial_data。每收到一帧 0x02 数据帧就：
    1. 解析波长/传感器/采样值
    2. 记录到 all_groups.csv，并写入在线缓冲供并行分析
    """
    bridge = HostTcpSerialBridge(port=tcp_port, timeout=TIMEOUT, debug=tcp_debug) # 创建TCP连接
    print(f"Android TCP bridge listening on port {tcp_port}.") # 打印连接信息
    reader = FrameReader(bridge) # 创建帧读取器
    android_live_output_path = str(Path(csv_filename).parent / ANDROID_LIVE_OUTPUT_CSV)
    online_session = create_online_session(
        bridge,
        prepare_interleaved=prepare_interleaved_dataframe, # 准备交错数据帧
        calculate_series=calculate_concentration_series, # 计算浓度系列
        live_plot=live_plot,
        android_live_output_path=android_live_output_path,
    )

    try:
        bridge.reset_input_buffer()
        print("Waiting for Android serial_data stream (UART controlled by Android).")

        with open(csv_filename, mode="w", newline="", encoding="utf-8") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Time (s)", "SensorId", CHANNEL_NAME, "Wavelength"])

            print("Starting single-channel raw ADC logging (seconds elapsed)...")
            start_time = time.time()

            while True:
                if duration_seconds is not None and (time.time() - start_time) >= duration_seconds:
                    print("Capture duration reached.")
                    break

                frame = reader.read_frame(timeout_seconds=TIMEOUT) # 读取帧
                if frame is None: # 帧为空
                    if bridge.capture_finished: # 捕获完成
                        print("Android requested capture finish; starting analysis.")
                        break
                    if bridge.closed or not bridge.is_open: # 连接关闭
                        print("TCP connection closed.")
                        break
                    continue # 继续读取
                if frame.frame_type != 0x02: # 帧类型不是0x02
                    continue

                sample = parse_data_frame(frame)
                elapsed_time = round(time.time() - start_time, 6)
                online_session.feed_sample(
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
        online_session.stop()
    return bridge
