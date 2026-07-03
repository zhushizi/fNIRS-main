"""在线分析与安卓回传共用的数据结构。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

AnalysisResult = dict[str, Any]


def compute_delta_thb(
    hbo: list[float] | tuple[float, ...],
    hbr: list[float] | tuple[float, ...],
) -> list[float]:
    """ΔtHb = HbO + HbR（与回传安卓的 hbo/hbr 同单位、同锚定）。"""
    n = min(len(hbo), len(hbr))
    out: list[float] = []
    for idx in range(n):
        value = float(hbo[idx]) + float(hbr[idx])
        out.append(value)
    return out


@dataclass(frozen=True)
class LiveAnalysisBatch:
    """一次 live_analysis_batch 消息的载荷。"""

    times: list[float]
    hbo: list[float]
    hbr: list[float]
    cyt: list[float]
    window_start_s: float
    window_end_s: float
    baseline_ready: bool = False
    rso2: list[float | None] | None = None
    latest_rso2_pct: float | None = None
    baseline_rso2_pct: float | None = None
    replace_full_series: bool = False
    unit: str = "a.u."
    # 固定基线伪绝对总血红蛋白（与 hbo/hbr 同单位、同缩放）。
    # thi = HbO_abs + HbR_abs（绝对量，≈ 基线 HbT）；delta_thi = thi − 基线 HbT（≈0 起算）。
    # 仅在基线冻结（baseline_ready=True）后才有值，热身期为 None。
    thi: list[float] | None = None
    delta_thi: list[float] | None = None
