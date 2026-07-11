"""安卓 TCP 桥接下的原始数据采集。"""

from __future__ import annotations

import csv
import time
from pathlib import Path

from config import (
    ANDROID_LIVE_OUTPUT_CSV,
    HOST_TCP_DEFAULT_PORT,
    OUTPUT_CHANNEL,
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
    通过安卓 TCP 桥采集双接收源五波长原始数据并写 CSV。

    安卓独占 UART 启停；PC 被动接收 serial_data。
    """
    bridge = HostTcpSerialBridge(port=tcp_port, timeout=TIMEOUT, debug=tcp_debug)
    print(f"Android TCP bridge listening on port {tcp_port}.")
    print(f"Output channel for online/offline analysis: {OUTPUT_CHANNEL}")
    reader = FrameReader(bridge)
    android_live_output_path = str(Path(csv_filename).parent / ANDROID_LIVE_OUTPUT_CSV)
    online_session = create_online_session(
        bridge,
        prepare_interleaved=prepare_interleaved_dataframe,
        calculate_series=calculate_concentration_series,
        live_plot=live_plot,
        android_live_output_path=android_live_output_path,
    )

    try:
        bridge.reset_input_buffer()
        print("Waiting for Android serial_data stream (UART controlled by Android).")

        with open(csv_filename, mode="w", newline="", encoding="utf-8") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Time (s)", "ChannelId", "DetectorId", "Channel", "Wavelength", "Value"])

            print("Starting dual-detector five-wavelength raw ADC logging (seconds elapsed)...")
            start_time = time.time()

            while True:
                if duration_seconds is not None and (time.time() - start_time) >= duration_seconds:
                    print("Capture duration reached.")
                    break

                frame = reader.read_frame(timeout_seconds=TIMEOUT)
                if frame is None:
                    if bridge.capture_finished:
                        print("Android requested capture finish; starting analysis.")
                        break
                    if bridge.closed or not bridge.is_open:
                        print("TCP connection closed.")
                        break
                    continue
                if frame.frame_type != 0x02:
                    continue

                sample = parse_data_frame(frame)
                elapsed_time = round(time.time() - start_time, 6)
                online_session.feed_sample(
                    elapsed_time,
                    sample.detector_code,
                    sample.value,
                    sample.wavelength_code,
                    sample.channel_name,
                    sample.acq_channel_code,
                )
                writer.writerow(
                    [
                        elapsed_time,
                        sample.acq_channel_code,
                        sample.detector_code,
                        sample.channel_name or "",
                        sample.wavelength_code,
                        sample.value,
                    ]
                )
                csvfile.flush()
                print(
                    f"{elapsed_time:.3f}s - value={sample.value} "
                    f"wl={int(sample.wavelength_code)} "
                    f"detector={int(sample.detector_code)} "
                    f"acq_ch={int(sample.acq_channel_code)} "
                    f"channel={sample.channel_name or 'unknown'}"
                )
    finally:
        online_session.stop()
    return bridge
