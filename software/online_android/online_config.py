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

    # 在线带通的滤波器结构。
    #   "fir" —— 线性相位 FIR，群延迟恒为 (taps−1)/2，与频率无关。
    #            在线 t 时刻的读数 = 离线 t−延迟 时刻的读数，波形不失真。
    #   "iir" —— 四阶巴特沃斯因果 IIR（原方案），保留作回退。
    #
    # 实测（录音 2026-07-03，通带 0.010~0.100 Hz，与离线零相位对比）：
    #     结构        延迟                与离线相关
    #     IIR         6.6 ~ 71.4 s 随频率变   0.314
    #     FIR 141     70 s 恒定              0.926
    #     FIR 201     100 s                 0.975
    #     FIR 301     150 s                 0.995
    # 同样的延迟预算（70 s）下，保真度由 0.31 提到 0.93，且延迟由「随频率变」
    # 变为「已知常数」，可在界面标注、在报警逻辑中扣除。
    causal_filter_kind: str = "fir"
    causal_fir_taps: int = 141                 # 奇数；群延迟 = (taps−1)/2 个样本


DEFAULT_ONLINE_SETTINGS = OnlineSettings()
