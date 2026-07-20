"""TCP 采集中在线分析并回传安卓上位机。"""

from .causal_processor import IncrementalCausalProcessor
from .online_config import DEFAULT_ONLINE_SETTINGS, OnlineSettings
from .logging_utils import configure_logging, get_logger, log_file_path
from .live_plotter import HBO_HBR_WINDOW_S, RSO2_WINDOW_S, LiveAndroidBatchPlotter
from .reporter import AndroidReporter, enrich_result_with_rso2
from .rso2 import (
    DEFAULT_BASELINE_END_S,
    DEFAULT_BASELINE_HBT_uM,
    DEFAULT_BASELINE_RSO2_PCT,
    DEFAULT_BASELINE_START_S,
    compute_rso2_series,
    compute_thi_series,
)
from .session import OnlineCaptureSession, create_online_session
from .skin_contact import (
    SkinContactDebouncer,
    SkinContactSettings,
    evaluate_skin_contact,
)
from .tcp_bridge import HostTcpProtocolError, HostTcpSerialBridge
from .types import AnalysisResult, LiveAnalysisBatch

__all__ = [
    "AnalysisResult",
    "AndroidReporter",
    "DEFAULT_BASELINE_END_S",
    "DEFAULT_BASELINE_HBT_uM",
    "DEFAULT_BASELINE_RSO2_PCT",
    "DEFAULT_BASELINE_START_S",
    "DEFAULT_ONLINE_SETTINGS",
    "HostTcpProtocolError",
    "HostTcpSerialBridge",
    "LiveAnalysisBatch",
    "LiveAndroidBatchPlotter",
    "HBO_HBR_WINDOW_S",
    "RSO2_WINDOW_S",
    "IncrementalCausalProcessor",
    "OnlineCaptureSession",
    "OnlineSettings",
    "SkinContactDebouncer",
    "SkinContactSettings",
    "compute_rso2_series",
    "evaluate_skin_contact",
    "compute_thi_series",
    "configure_logging",
    "create_online_session",
    "enrich_result_with_rso2",
    "get_logger",
    "log_file_path",
]
