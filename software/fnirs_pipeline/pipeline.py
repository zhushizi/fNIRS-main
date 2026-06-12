"""采集 → 预处理 → MBLL 的一键离线管线。"""

from __future__ import annotations

import os
from datetime import datetime

import pandas as pd

from config import HOST_TCP_DEFAULT_PORT, WAVELENGTH_OFF_CODE
from online_android import HostTcpSerialBridge

from .capture import capture_data, send_analysis_result_to_android
from .mbll import process_csv_dataset
from .preprocessing import prepare_interleaved_dataframe

RESULT_TABLE_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "result_table")
)


def run_pipeline(
    tcp_port: int = HOST_TCP_DEFAULT_PORT,
    tcp_debug: bool = False,
) -> None:
    """一键跑完整链路：安卓 TCP 采集 -> 预处理 -> 配对 -> MBLL -> CSV 输出。"""
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
        tcp_bridge = capture_data(
            csv_filename=raw_path,
            tcp_port=tcp_port,
            tcp_debug=tcp_debug,
        )

        df = pd.read_csv(raw_path) # 
        if df.empty or len(df) < 2:
            result = {"ok": False, "message": "No enough raw rows captured; skipping processing."}
            print(result["message"])
            send_analysis_result_to_android(tcp_bridge, result)
            return

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

        dt = df["Time (s)"].diff().mean()
        fs = 1.0 / dt if pd.notna(dt) and dt > 0 else 1.0
        print(f"采样率: {fs} Hz")

        final_df = prepare_interleaved_dataframe(df, start_at_zero=True)
        if final_df.empty:
            result = {"ok": False, "message": "No complete wavelength cycles were formed; skipping MBLL."}
            print(result["message"])
            send_analysis_result_to_android(tcp_bridge, result)
            return

        final_df.to_csv(interleaved_path, index=False)
        print(final_df.head(20))

        result = process_csv_dataset(interleaved_path, processed_path)
        send_analysis_result_to_android(tcp_bridge, result)
    finally:
        if tcp_bridge is not None:
            tcp_bridge.close()
