"""在线分析与安卓回传共用的数据结构。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

AnalysisResult = dict[str, Any]


@dataclass(frozen=True)
class LiveAnalysisBatch:
    """一次 live_analysis_batch 消息的载荷。"""

    times: list[float]
    hbo: list[float]
    hbr: list[float]
    window_start_s: float
    window_end_s: float
    baseline_ready: bool = False
    rso2: list[float | None] | None = None
    latest_rso2_pct: float | None = None
    baseline_rso2_pct: float | None = None
