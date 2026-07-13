"""采集 → 预处理 → MBLL 的一键离线管线（按采集通道各跑一套）。"""

from __future__ import annotations

import os
from datetime import datetime

import pandas as pd

from config import (
    ACQUISITION_CHANNELS,
    DEFAULT_CHANNEL_CODE,
    HOST_TCP_DEFAULT_PORT,
    OUTPUT_CHANNEL,
    WAVELENGTH_OFF_CODE,
    channel_name_for_code,
    with_channel_suffix,
)
from online_android import HostTcpSerialBridge, configure_logging

from .capture import capture_data, send_analysis_result_to_android
from .mbll import process_csv_dataset
from .preprocessing import prepare_interleaved_dataframe

RESULT_TABLE_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "result_table")
)


def _ordered_channel_codes(df: pd.DataFrame) -> list[int]:
    """返回 df 中出现的采集通道码，按 ACQUISITION_CHANNELS 顺序，未知码追加在后。"""
    if "ChannelId" not in df.columns:
        return [DEFAULT_CHANNEL_CODE]
    present = {int(c) for c in df["ChannelId"].to_numpy()}
    ordered = [ch.code for ch in ACQUISITION_CHANNELS if ch.code in present]
    ordered.extend(sorted(code for code in present if code not in ordered))
    return ordered


def _process_one_channel(
    tcp_bridge: HostTcpSerialBridge,
    channel_df: pd.DataFrame,
    code: int,
    interleaved_path: str,
    processed_path: str,
) -> None:
    """对单个采集通道跑 预处理 → 配对 → MBLL/SSR，并回传带 channel 字段的结果。"""
    ch_name = channel_name_for_code(code)
    print(f"\n=== 采集通道 {code} ({ch_name}) ===")

    if channel_df.empty or len(channel_df) < 2:
        result = {"ok": False, "channel": code, "message": f"channel {code}: no enough samples."}
        print(result["message"])
        send_analysis_result_to_android(tcp_bridge, result)
        return

    ch_interleaved = with_channel_suffix(interleaved_path, code)
    ch_processed = with_channel_suffix(processed_path, code)

    final_df = prepare_interleaved_dataframe(channel_df, start_at_zero=True)
    if final_df.empty:
        result = {
            "ok": False,
            "channel": code,
            "message": f"channel {code}: no complete wavelength cycles were formed.",
        }
        print(result["message"])
        send_analysis_result_to_android(tcp_bridge, result)
        return

    final_df.to_csv(ch_interleaved, index=False)
    print(final_df.head(20))

    result = process_csv_dataset(ch_interleaved, ch_processed)
    result["channel"] = code
    send_analysis_result_to_android(tcp_bridge, result)


def run_pipeline(
    tcp_port: int = HOST_TCP_DEFAULT_PORT,
    tcp_debug: bool = False,
    live_plot: bool = False,
) -> None:
    """一键跑完整链路：安卓 TCP 采集 -> 按通道拆分 -> 预处理 -> 配对 -> 广义 MBLL/SSR -> CSV。"""
    configure_logging()  # PC 入口：控制台 + 默认 online_android/logs/ 文件
    os.makedirs(RESULT_TABLE_DIR, exist_ok=True)
    run_id = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output_dir = os.path.join(RESULT_TABLE_DIR, run_id)
    os.makedirs(output_dir, exist_ok=True)
    raw_path = os.path.join(output_dir, "all_groups.csv")
    interleaved_path = os.path.join(output_dir, "interleaved_output.csv")
    processed_path = os.path.join(output_dir, "processed_output.csv")
    print(f"本次结果将保存到: {output_dir}")
    print(f"输出通道: {OUTPUT_CHANNEL}")
    tcp_bridge: HostTcpSerialBridge | None = None

    try:
        tcp_bridge = capture_data(
            csv_filename=raw_path,
            tcp_port=tcp_port,
            tcp_debug=tcp_debug,
            live_plot=live_plot,
        )

        df = pd.read_csv(raw_path)
        if df.empty or len(df) < 2:
            result = {"ok": False, "message": "No enough raw rows captured; skipping processing."}
            print(result["message"])
            send_analysis_result_to_android(tcp_bridge, result)
            return

        n_raw = len(df)
        df = df[df["Wavelength"] != WAVELENGTH_OFF_CODE].reset_index(drop=True)
        dropped = n_raw - len(df)
        if dropped:
            print(f"Dropped {dropped} raw row(s) with Wavelength=OFF (0x00); not used for wavelength pairing.")

        if df.empty or len(df) < 2:
            result = {"ok": False, "message": "No enough non-OFF samples after filtering; skipping processing."}
            print(result["message"])
            send_analysis_result_to_android(tcp_bridge, result)
            return

        if "ChannelId" not in df.columns:
            df["ChannelId"] = DEFAULT_CHANNEL_CODE

        codes = _ordered_channel_codes(df)
        print(f"采集通道: {[int(c) for c in codes]}（每个通道分别落盘）")

        for code in codes:
            channel_df = df[df["ChannelId"] == code].reset_index(drop=True)
            _process_one_channel(tcp_bridge, channel_df, code, interleaved_path, processed_path)
    finally:
        if tcp_bridge is not None:
            tcp_bridge.close()
