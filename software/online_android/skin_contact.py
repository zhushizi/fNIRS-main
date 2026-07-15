"""贴肤/未贴实时判定（搭在 live_analysis_batch 上，按采集通道各判一次）。

判据：近窗（默认 5s）内每个接收源的光强中位数，与 config 中该接收源阈值比较；
配置里出现的接收源需全部达标（AND 组合）才算贴肤——单一接收源阈值挡不住
「未贴但短距偶发高值」的情况，双接收源联合才稳。再经时间防抖，避免贴/松开
过渡的 2~3s 抖动。只看原始光强，不做 MBLL/SSR，成本 O(近窗样本)。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
import pandas as pd

from config import (
    DETECTOR_CHANNELS,
    SKIN_CONTACT_DEBOUNCE_S,
    SKIN_CONTACT_MIN_SAMPLES,
    SKIN_CONTACT_THRESHOLD,
    SKIN_CONTACT_WINDOW_S,
    WAVELENGTH_OFF_CODE,
)

_TIME = "Time (s)"


@dataclass(frozen=True)
class SkinContactSettings:
    """贴肤判定参数（默认取自 config）。"""

    window_s: float = SKIN_CONTACT_WINDOW_S
    min_samples: int = SKIN_CONTACT_MIN_SAMPLES
    debounce_s: float = SKIN_CONTACT_DEBOUNCE_S
    thresholds: dict[str, float] = field(
        default_factory=lambda: dict(SKIN_CONTACT_THRESHOLD)
    )

    @classmethod
    def from_config(cls) -> "SkinContactSettings":
        return cls()


def evaluate_skin_contact(
    channel_df: pd.DataFrame,
    settings: SkinContactSettings,
) -> tuple[bool | None, dict[str, Any] | None, float]:
    """对单个采集通道的原始 df 做**未防抖**的瞬时贴肤判定。

    返回 (raw_state, detail, now_t)：
    - raw_state: True=贴肤，False=未贴，None=样本不足/无法判定（不轻易报贴肤）
    - detail: {"median": {det: 值}, "pass": {det: bool}}，供调试/回传，可能为 None
    - now_t: 近窗最新样本时间（数据时钟，用于防抖）
    """
    if channel_df.empty or _TIME not in channel_df.columns:
        return None, None, 0.0

    t = channel_df[_TIME].to_numpy(dtype=float)
    now_t = float(t.max()) if t.size else 0.0
    recent = channel_df[
        (channel_df[_TIME] > now_t - settings.window_s)
        & (channel_df["Wavelength"] != WAVELENGTH_OFF_CODE)
    ]

    medians: dict[str, float] = {}
    passed: dict[str, bool] = {}
    enough = True
    for det in DETECTOR_CHANNELS:
        thr = settings.thresholds.get(det.name)
        if thr is None:
            continue  # 未配置阈值的接收源不参与判定
        vals = recent[recent["DetectorId"] == det.code]["Value"].to_numpy(dtype=float)
        if vals.size < settings.min_samples:
            enough = False
            continue
        med = float(np.median(vals))
        medians[det.name] = med
        passed[det.name] = med >= float(thr)

    detail = {"median": medians, "pass": passed} if medians else None
    # 任一受约束接收源样本不足，或压根没有可判定的接收源 → 未知
    if not passed or not enough:
        return None, detail, now_t
    return all(passed.values()), detail, now_t


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
