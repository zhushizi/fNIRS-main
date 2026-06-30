"""fNIRS 双接收五波长采集、预处理与 MBLL 离线/在线分析管线。"""

from .capture import capture_data, send_analysis_result_to_android
from .mbll import (
    calculate_concentration_series,
    compute_all_channel_concentrations,
    process_csv_dataset,
    summarize_processed_concentrations,
)
from .pipeline import run_pipeline
from .preprocessing import (
    aggregate_wavelength_cycles,
    prepare_interleaved_dataframe,
    sliding_window_rms,
    stack_intensities_for_detector,
    threshold_filter,
)

__all__ = [
    "aggregate_wavelength_cycles",
    "calculate_concentration_series",
    "capture_data",
    "compute_all_channel_concentrations",
    "prepare_interleaved_dataframe",
    "process_csv_dataset",
    "run_pipeline",
    "send_analysis_result_to_android",
    "sliding_window_rms",
    "stack_intensities_for_detector",
    "summarize_processed_concentrations",
    "threshold_filter",
]
