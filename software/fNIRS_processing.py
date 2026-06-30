"""
双接收源五波长 fNIRS 采集与处理主脚本。

完整链路如下：
1. 通过 26B 协议采集 S1-D1 / S1-D2 五波长原始数据（安卓 TCP 桥接）
2. 写入 all_groups.csv
3. 对每个接收源/波长组合做低通/RMS 预处理
4. 聚合成 2 接收源 x 5 波长光强宽表
5. 计算 OD -> 短距回归校正 -> 广义 MBLL -> HbO/HbR/Cyt
6. 按 config.OUTPUT_CHANNEL 输出 processed_output.csv（S1_D1 / S1_D2 / S1_D1_ssr）

采集中由 online_android 并行分析并回传 live_analysis_batch；
可选 --live_plot 在 PC 端同步绘制发往安卓的数据。
"""

from __future__ import annotations

import argparse

from config import HOST_TCP_DEFAULT_PORT, OUTPUT_CHANNEL
from fnirs_pipeline import (
    aggregate_wavelength_cycles,
    calculate_concentration_series,
    capture_data,
    prepare_interleaved_dataframe,
    process_csv_dataset,
    run_pipeline,
    send_analysis_result_to_android,
    sliding_window_rms,
    stack_intensities_for_detector,
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
    "stack_intensities_for_detector",
    "summarize_processed_concentrations",
    "threshold_filter",
]


def parse_args() -> argparse.Namespace:
    """解析命令行：安卓 TCP 桥接参数。"""
    parser = argparse.ArgumentParser(
        description="Run dual-detector five-wavelength fNIRS via Android TCP bridge."
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
    parser.add_argument(
        "--live_plot",
        action="store_true",
        help=(
            "Mirror live_analysis_batch to a local plot while sending to Android "
            "(HbO/HbR: 30s window, rSO2: 60s window)."
        ),
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    print(f"Configured output channel: {OUTPUT_CHANNEL}")
    try:
        run_pipeline(
            tcp_port=args.tcp_port,
            tcp_debug=args.tcp_debug,
            live_plot=args.live_plot,
        )
    except KeyboardInterrupt:
        print("\nStopped by user.")
