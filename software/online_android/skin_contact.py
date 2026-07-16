"""贴肤（手臂/皮肤）实时判定（搭在 live_analysis_batch 上，按采集通道各判一次）。

两级判据（近窗默认 5s，只看短距 S1_D2 原始光强，不做 MBLL/SSR）：

  ① 压住？  S1_D2 近窗中位数 ≥ PRESS_MIN → 有东西贴住（否则悬空/未贴）
  ② 是组织？S1_D2 810nm/700nm 中位数比 ≥ TISSUE_RATIO_MIN → 手臂/皮肤；
            低于阈值 → 桌子/硬物，**不算贴肤**（返回 False）

700nm 是脱氧血红蛋白强吸收带：组织里的血把 700 压低 → 810/700 升高；桌子无血 → 比值低。
长距 S1_D1 当前采集处于饱和（对空气都满值、信息为零），故判据只用短距。再经时间防抖，
避免贴/松开过渡抖动。成本 O(近窗样本)。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
import pandas as pd

from config import (
    CODE_BY_WAVELENGTH,
    DETECTOR_CHANNELS,
    SKIN_CONTACT_DEBOUNCE_S,
    SKIN_CONTACT_MIN_SAMPLES,
    SKIN_CONTACT_PRESS_DETECTOR,
    SKIN_CONTACT_PRESS_MIN,
    SKIN_CONTACT_TISSUE_RATIO_HIGH_NM,
    SKIN_CONTACT_TISSUE_RATIO_LOW_NM,
    SKIN_CONTACT_TISSUE_RATIO_MIN,
    SKIN_CONTACT_WINDOW_S,
    WAVELENGTH_OFF_CODE,
)

_TIME = "Time (s)"


def _detector_code(name: str) -> int | None:
    for det in DETECTOR_CHANNELS:
        if det.name == name:
            return det.code
    return None


@dataclass(frozen=True)
class SkinContactSettings:
    """贴肤判定参数（默认取自 config）。"""

    window_s: float = SKIN_CONTACT_WINDOW_S
    min_samples: int = SKIN_CONTACT_MIN_SAMPLES
    debounce_s: float = SKIN_CONTACT_DEBOUNCE_S
    press_detector: str = SKIN_CONTACT_PRESS_DETECTOR
    press_min: float = SKIN_CONTACT_PRESS_MIN
    ratio_high_nm: float = SKIN_CONTACT_TISSUE_RATIO_HIGH_NM
    ratio_low_nm: float = SKIN_CONTACT_TISSUE_RATIO_LOW_NM
    ratio_min: float = SKIN_CONTACT_TISSUE_RATIO_MIN

    @classmethod
    def from_config(cls) -> "SkinContactSettings":
        return cls()


def evaluate_skin_contact(
    channel_df: pd.DataFrame,
    settings: SkinContactSettings,
) -> tuple[bool | None, dict[str, Any] | None, float]:
    """对单个采集通道的原始 df 做**未防抖**的瞬时贴肤判定。

    返回 (raw_state, detail, now_t)：
    - raw_state: True=贴肤(手臂/皮肤)，False=未贴(悬空)或桌子/硬物，None=样本不足
    - detail: {"press_median", "ratio", "ratio_min", "press_min", "reason"}，供调试/回传
    - now_t: 近窗最新样本时间（数据时钟，用于防抖）
    """
    if channel_df.empty or _TIME not in channel_df.columns:
        return None, None, 0.0

    t = channel_df[_TIME].to_numpy(dtype=float)
    now_t = float(t.max()) if t.size else 0.0

    det_code = _detector_code(settings.press_detector)
    hi_code = CODE_BY_WAVELENGTH.get(float(settings.ratio_high_nm))
    lo_code = CODE_BY_WAVELENGTH.get(float(settings.ratio_low_nm))
    if det_code is None or hi_code is None or lo_code is None:
        return None, None, now_t

    recent = channel_df[
        (channel_df[_TIME] > now_t - settings.window_s)
        & (channel_df["Wavelength"] != WAVELENGTH_OFF_CODE)
        & (channel_df["DetectorId"] == det_code)
    ]

    def _median(code: int | None) -> float | None:
        if code is None:
            vals = recent["Value"].to_numpy(dtype=float)
        else:
            vals = recent[recent["Wavelength"] == code]["Value"].to_numpy(dtype=float)
        if vals.size < settings.min_samples:
            return None
        return float(np.median(vals))

    press_med = _median(None)          # 短距全波长中位数：是否压住
    hi_med = _median(hi_code)          # 810nm 中位数
    lo_med = _median(lo_code)          # 700nm 中位数

    ratio = hi_med / lo_med if (hi_med is not None and lo_med not in (None, 0.0)) else None
    detail: dict[str, Any] = {
        "press_median": press_med,
        "ratio": ratio,
        "ratio_min": settings.ratio_min,
        "press_min": settings.press_min,
    }

    if press_med is None or ratio is None:
        detail["reason"] = "insufficient_samples"
        return None, detail, now_t
    if press_med < settings.press_min:
        detail["reason"] = "not_pressed"          # 悬空/未贴
        return False, detail, now_t
    if ratio < settings.ratio_min:
        detail["reason"] = "nonskin_surface"      # 桌子/硬物：压住但非组织
        return False, detail, now_t
    detail["reason"] = "skin"
    return True, detail, now_t


class SkinContactDebouncer:
    """按数据时间防抖：候选状态需连续保持 >= debounce_s 才提交翻转。

    每个采集通道各持有一个实例（贴肤状态互不影响）。
    """

    def __init__(self, settings: SkinContactSettings) -> None:
        self._debounce_s = float(settings.debounce_s)
        self._committed: bool | None = None
        self._candidate: bool | None = None
        self._candidate_since: float | None = None

    def peek(self) -> bool | None:
        """读当前已提交状态，不推进防抖（用于 set_baseline 整段回填等只读场景）。"""
        return self._committed

    def update(self, raw_state: bool | None, now_t: float) -> bool | None:
        """喂入一次瞬时判定，返回防抖后的已提交状态。"""
        if raw_state is None:
            # 未知：保持已提交状态，清掉候选，避免跨数据空洞累计
            self._candidate = None
            self._candidate_since = None
            return self._committed

        if self._committed is None:
            # 首个有效判定：立即提交，尽快给出状态
            self._committed = raw_state
            self._candidate = None
            self._candidate_since = None
            return self._committed

        if raw_state == self._committed:
            self._candidate = None
            self._candidate_since = None
            return self._committed

        # 与已提交相反：进入或累计候选，够时长才翻转
        if self._candidate != raw_state:
            self._candidate = raw_state
            self._candidate_since = now_t
        elif (
            self._candidate_since is not None
            and (now_t - self._candidate_since) >= self._debounce_s
        ):
            self._committed = raw_state
            self._candidate = None
            self._candidate_since = None
        return self._committed
