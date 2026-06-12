"""
单通道 fNIRS 采集与处理主脚本。

完整链路如下：
1. 通过 26B 协议采集原始单通道数据
2. 写入 all_groups.csv
3. 对单通道信号做阈值/低通/RMS 预处理
4. 按 config.WAVELENGTH_CHANNELS 聚合成多列光强
5. 计算 OD -> MBLL -> CBSI
6. 输出 processed_output.csv

运行模式：安卓独占 UART，PC 通过 TCP 被动接收 serial_data；
采集中由 online_android 包并行分析并回传 live_analysis_batch（含 rSO2）。
采集结束后 run_pipeline 对全段 CSV 做离线 MBLL，并以 analysis_result 回传完成状态（不含均值）。

在线安卓回传实现见 software/online_android/（配置 / 缓冲 / 批次构建 / 上报 / 会话）。
处理实现见 software/fnirs_pipeline/（预处理 / MBLL / 采集 / 管线编排）。
"""

from __future__ import annotations

import argparse # 

from config import HOST_TCP_DEFAULT_PORT # 导入端口号
from fnirs_pipeline import (
    aggregate_wavelength_cycles,
    calculate_concentration_series,
    capture_data,
    prepare_interleaved_dataframe,
    process_csv_dataset,
    run_pipeline,
    send_analysis_result_to_android,
    sliding_window_rms,
    summarize_processed_concentrations,
    threshold_filter,
)

__all__ = [
    "aggregate_wavelength_cycles",
    "calculate_concentration_series",
    "capture_data",
    "prepare_interleaved_dataframe",
    "process_csv_dataset",
    "run_pipeline",
    "send_analysis_result_to_android",
    "sliding_window_rms",
    "summarize_processed_concentrations",
    "threshold_filter",
]


def parse_args() -> argparse.Namespace:
    """解析命令行：安卓 TCP 桥接参数。"""
    parser = argparse.ArgumentParser(
        description="Run fNIRS capture and processing via Android TCP bridge."
    )
    parser.add_argument(
        "-tcp_port",
        "--tcp_port",
        type=int,
        nargs="?",
        const=HOST_TCP_DEFAULT_PORT,
        default=HOST_TCP_DEFAULT_PORT,
        help=f"TCP listening port for Android client. Default: {HOST_TCP_DEFAULT_PORT}.",
    )
    parser.add_argument(
        "--tcp_debug",
        action="store_true",
        help="Print TCP protocol rx/tx logs (type/seq/byte_len).",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    try:
        run_pipeline(tcp_port=args.tcp_port, tcp_debug=args.tcp_debug)
    except KeyboardInterrupt:
        print("\nStopped by user.")
