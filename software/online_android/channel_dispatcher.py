"""
按采集通道分发在线分析。

硬件现在有两个采集通道（通道1/通道2，数据帧 payload byte6），与接收源/波长正交。
每个采集通道内部仍是完整的双接收源×五波长，需要各自独立跑一整套配对 → SSR → MBLL →
rSO₂ 反演。本分发器为每个出现过的通道各维护一个处理器（因果增量或全量重发），
按通道拆分 raw_df 后分别构建 LiveAnalysisBatch，并给每条批次打上 `channel` 标签。

多个通道共享同一份 BL 假设（MutableBaseline），set_baseline 一次性对全部通道生效。
"""

from __future__ import annotations

import threading
from collections.abc import Callable
from dataclasses import replace
from typing import Optional, Union

import numpy as np
import pandas as pd

from config import ACQUISITION_CHANNELS, DEFAULT_CHANNEL_CODE

from .baseline_state import MutableBaseline
from .batch_builder import IncrementalBatchBuilder
from .causal_processor import IncrementalCausalProcessor
from .online_config import OnlineSettings
from .logging_utils import get_logger
from .skin_contact import SkinContactDebouncer, SkinContactSettings, evaluate_skin_contact
from .types import LiveAnalysisBatch

log = get_logger("dispatcher")

PrepareInterleavedFn = Callable[[pd.DataFrame, bool], pd.DataFrame]
CalculateSeriesFn = Callable[
    [pd.DataFrame], Optional[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]]
]

_CHANNEL_COL = "ChannelId"

# 运行时联合类型别名：用 typing.Union 而非 `A | B`，后者在 Python 3.9 运行时会报
# "unsupported operand type(s) for |: 'type' and 'type'"。
Processor = Union[IncrementalCausalProcessor, IncrementalBatchBuilder]


class ChannelDispatcher:
    """按采集通道拆分 raw_df，为每个通道各维护一个在线处理器。"""

    def __init__(
        self,
        settings: OnlineSettings,
        *,
        prepare_interleaved: PrepareInterleavedFn,
        calculate_series: CalculateSeriesFn,
        baseline: MutableBaseline,
        on_new_channel: Callable[[int], None] | None = None,
    ) -> None:
        self._settings = settings
        self._prepare_interleaved = prepare_interleaved
        self._calculate_series = calculate_series
        self._baseline = baseline
        self._on_new_channel = on_new_channel
        self._processors: dict[int, Processor] = {}
        # 每个采集通道各持一个贴肤防抖器，状态互不影响。
        self._skin_settings = SkinContactSettings.from_config()
        self._skin: dict[int, SkinContactDebouncer] = {}
        # 采集(worker)线程周期性 build_all 与 rx 线程的 set_baseline 会并发访问处理器，
        # 统一用一把锁串行化，避免并发创建/改写处理器内部增量状态。
        self._lock = threading.Lock()

    # ------------------------------------------------------------------ public
    def build_all(
        self,
        raw_df: pd.DataFrame,
        *,
        force_recompute: bool = False,
    ) -> list[tuple[int, LiveAnalysisBatch]]:
        """
        对每个出现的通道各构建一批，返回 [(通道码, batch), ...]。

        每个通道**独立捕获异常**：任一通道构建失败只跳过它自己，绝不影响其它
        通道本 tick 的产出——避免"一路出错导致所有通道都收不到数据"。
        """
        out: list[tuple[int, LiveAnalysisBatch]] = []
        with self._lock:
            for code in self._channels_present(raw_df):
                sub = self._sub_df(raw_df, code)
                if sub.empty:
                    continue
                try:
                    batch = self._get_processor(code).try_build(
                        sub, force_recompute=force_recompute
                    )
                except Exception:
                    # 带堆栈落盘，方便定位某通道的根因；不影响其它通道。
                    log.exception("[channel %s] build failed, skipped this tick", code)
                    continue
                if batch is not None:
                    out.append((code, self._attach_skin(code, sub, batch, advance=True)))
        return out

    def update_baseline(
        self,
        rso2_pct: float,
        hbt_uM: float | None = None,
    ) -> tuple[float, float]:
        """更新共享 BL 假设（对全部通道生效）。"""
        return self._baseline.update(rso2_pct, hbt_uM)

    def apply_baseline_and_rebuild_all(
        self,
        raw_df: pd.DataFrame,
    ) -> list[tuple[int, LiveAnalysisBatch]]:
        """set_baseline 后对每个通道全会话重算并整段回填（同样逐通道隔离异常）。"""
        out: list[tuple[int, LiveAnalysisBatch]] = []
        with self._lock:
            for code in self._channels_present(raw_df):
                sub = self._sub_df(raw_df, code)
                if sub.empty:
                    continue
                try:
                    batch = self._get_processor(code).apply_baseline_and_rebuild(sub)
                except Exception:
                    log.exception("[channel %s] baseline rebuild failed, skipped", code)
                    continue
                if batch is not None:
                    # 整段回填是只读场景：附当前贴肤状态但不推进防抖。
                    out.append((code, self._attach_skin(code, sub, batch, advance=False)))
        return out

    @property
    def active_channels(self) -> tuple[int, ...]:
        return tuple(self._processors.keys())

    # ----------------------------------------------------------------- private
    @staticmethod
    def _channel_order() -> list[int]:
        return [ch.code for ch in ACQUISITION_CHANNELS]

    def _channels_present(self, raw_df: pd.DataFrame) -> list[int]:
        if raw_df.empty:
            return []
        if _CHANNEL_COL not in raw_df.columns:
            return [DEFAULT_CHANNEL_CODE]
        present = {int(c) for c in raw_df[_CHANNEL_COL].to_numpy()}
        ordered = [code for code in self._channel_order() if code in present]
        # 防御：出现了配置表以外的通道码也照常处理，避免静默丢数据。
        ordered.extend(sorted(code for code in present if code not in ordered))
        return ordered

    def _sub_df(self, raw_df: pd.DataFrame, code: int) -> pd.DataFrame:
        if _CHANNEL_COL not in raw_df.columns:
            return raw_df.reset_index(drop=True)
        return raw_df[raw_df[_CHANNEL_COL] == code].reset_index(drop=True)

    def _get_processor(self, code: int) -> Processor:
        proc = self._processors.get(code)
        if proc is None:
            proc = self._make_processor()
            self._processors[code] = proc
            if self._on_new_channel is not None:
                self._on_new_channel(code)
        return proc

    def _attach_skin(
        self,
        code: int,
        sub: pd.DataFrame,
        batch: LiveAnalysisBatch,
        *,
        advance: bool,
    ) -> LiveAnalysisBatch:
        """给某通道的 batch 打上贴肤状态。advance=True 推进防抖，False 只读当前状态。"""
        raw_state, detail, now_t = evaluate_skin_contact(sub, self._skin_settings)
        deb = self._skin.get(code)
        if deb is None:
            deb = SkinContactDebouncer(self._skin_settings)
            self._skin[code] = deb
        committed = deb.update(raw_state, now_t) if advance else deb.peek()
        return replace(
            batch, channel=code, skin_contact=committed, skin_contact_detail=detail
        )

    def _make_processor(self) -> Processor:
        if self._settings.online_mode == "causal_incremental":
            return IncrementalCausalProcessor(
                self._settings,
                prepare_interleaved=self._prepare_interleaved,
                calculate_series=self._calculate_series,
                baseline=self._baseline,
            )
        return IncrementalBatchBuilder(
            self._settings,
            prepare_interleaved=self._prepare_interleaved,
            calculate_series=self._calculate_series,
            baseline=self._baseline,
        )
