"""fNIRS 单通道采集、预处理与 MBLL 离线/在线分析管线。"""

from .capture import capture_data, send_analysis_result_to_android
from .mbll import (
    calculate_concentration_series,
    process_csv_dataset,
    summarize_processed_concentrations,
)
from .pipeline import run_pipeline
from .preprocessing import (
    aggregate_wavelength_cycles,
    prepare_interleaved_dataframe,
    sliding_window_rms,
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
