"""在线分析 / 安卓回传的配置项。"""

from __future__ import annotations

from dataclasses import dataclass

from config import (
    BP_ORDER,
    LIVE_BASELINE_ANCHOR,
    LIVE_CONC_SCALE,
    LIVE_CONC_UNIT,
    ONLINE_MODE,
)
from .rso2 import (
    DEFAULT_BASELINE_END_S,
    DEFAULT_BASELINE_HBT_uM,
    DEFAULT_BASELINE_RSO2_PCT,
    DEFAULT_BASELINE_START_S,
)


@dataclass(frozen=True)
class OnlineSettings:
    """TCP 采集中在线分析与回传安卓的时序参数。"""

    update_interval_seconds: float = 1.0
    buffer_retention_seconds: float | None = None
    min_interleaved_points: int = max(16, 3 * (BP_ORDER + 1) + 1)
    rso2_baseline_start_s: float = DEFAULT_BASELINE_START_S
    rso2_baseline_end_s: float = DEFAULT_BASELINE_END_S
    rso2_baseline_hbt_uM: float = DEFAULT_BASELINE_HBT_uM
    rso2_baseline_rso2_pct: float = DEFAULT_BASELINE_RSO2_PCT

    # 路线选择与显示换算
    online_mode: str = ONLINE_MODE
    conc_scale: float = LIVE_CONC_SCALE
    conc_unit: str = LIVE_CONC_UNIT
    baseline_anchor: bool = LIVE_BASELINE_ANCHOR
    # 因果模式热身：累计到该时长（默认对齐 rSO₂ 基线窗末端）后冻结基准并切到追加流。
    causal_warmup_seconds: float = DEFAULT_BASELINE_END_S


DEFAULT_ONLINE_SETTINGS = OnlineSettings()
