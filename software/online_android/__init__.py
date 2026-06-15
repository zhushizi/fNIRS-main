"""TCP 采集中在线分析并回传安卓上位机。"""

from .config import DEFAULT_ONLINE_SETTINGS, OnlineSettings
from .live_plotter import HBO_HBR_WINDOW_S, RSO2_WINDOW_S, LiveAndroidBatchPlotter
from .reporter import AndroidReporter, enrich_result_with_rso2
from .rso2 import (
    DEFAULT_BASELINE_END_S,
    DEFAULT_BASELINE_HBT_uM,
    DEFAULT_BASELINE_RSO2_PCT,
    DEFAULT_BASELINE_START_S,
    compute_rso2_series,
)
from .session import OnlineCaptureSession, create_online_session
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
    "OnlineCaptureSession",
    "OnlineSettings",
    "compute_rso2_series",
    "create_online_session",
    "enrich_result_with_rso2",
]
