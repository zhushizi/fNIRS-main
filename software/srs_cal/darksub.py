# -*- coding: utf-8 -*-
"""扣暗 —— 按**时刻**在前后两个暗时隙之间插值，而不是减同周期那一个。

为什么（2026-10-22，10 分钟体模静置实测）：
    噪声的主成分不是漂移，而是 **0.0868 Hz 的窄带干扰**（周期 11.5 s）。
    它在**暗时隙里也有**（占暗时隙功率 89%），LED 关着也在 —— 电路/环境来源。

    逐周期扣暗消不掉它，原因能算出来：暗时隙与信号帧**在周期内不是同一时刻**，
    最多相差 1.5 s；对 11.5 s 周期是 **47° 相位差**，相减反而残留。

    按时刻插值把这个相位差消掉。逐周期序列的 std 实测改善：

        通道        逐周期   插值    改善
        670/3cm      858     570    1.51×
        670/4cm     1052     548    1.92×
        770/3cm     1567     759    2.06×
        770/4cm     2074     720    2.88×

    ⚠️ **但这不等于 TOI 改善同样倍数。** 窗平均本身就会抵消周期成分，
       所以插值的收益随窗口变大而递减（实测 TOI std 比）：

        N=10  0.76×      N=20  0.82×      N=40  0.96×
        （N≥60 的点只有 6~13 个 TOI 值，std 本身不确定，不作结论）

       它真正的价值是**让短窗口可用**：N=20 带插值 ≈ N=28 不带，省一半滞后。

代价：插值要等**下一个**暗样本，故延迟 1 个扫描周期（约 1.5 s）。
      相对 N×1.5 s 的平均窗口可以忽略。`mode="prev"` 可退回无延迟的旧行为。
"""

from __future__ import annotations

import collections
from dataclasses import dataclass, field

MODE_INTERP = "interp"      # 前后两暗按时刻插值（延迟 1 周期）
MODE_PREV = "prev"          # 用上一个暗（无延迟，等价于旧的逐周期扣暗）

# 待解析的信号帧上限。正常只会攒一个周期（10 帧）；
# 长期攒不下去说明暗时隙没来，此时丢弃而不是无限涨。
MAX_PENDING = 200

# 两个暗之间间隔超过这么多秒就不插值了 —— 丢帧或中断，插值没有意义。
MAX_GAP_S = 10.0


@dataclass
class DarkSubtractor:
    """输入带 t_host 的帧，输出扣暗后的 (波长, 距离, 时刻, 净值)。

    用法：
        ds = DarkSubtractor()
        ds.feed_signal(670.0, 3.0, t, adc)          # 先攒着
        for wl, rho, t, net in ds.feed_dark(3.0, t2, v2):
            ...                                      # 下一个暗到了才吐出来
    """

    mode: str = MODE_INTERP
    _prev: dict = field(default_factory=dict)        # rho -> (t, v)
    _pending: dict = field(default_factory=lambda: collections.defaultdict(list))
    n_dropped_nodark: int = 0        # 没有可用暗而丢弃的信号帧
    n_wide_gap: int = 0              # 暗间隔过大、退回用上一个暗的次数

    # -- 输入 --------------------------------------------------------------
    def feed_dark(self, rho: float, t: float, v: float) -> list[tuple]:
        """喂一个暗时隙。返回因此得以解析的信号帧。"""
        rho = float(rho)
        prev = self._prev.get(rho)
        self._prev[rho] = (float(t), float(v))
        if prev is None:
            # 第一个暗之前攒的帧没有左端点，解析不了
            self.n_dropped_nodark += len(self._pending[rho])
            self._pending[rho].clear()
            return []
        return self._resolve(rho, prev, (float(t), float(v)))

    def feed_signal(self, wl: float, rho: float, t: float, adc: float) -> list[tuple]:
        """喂一个信号帧。

        `interp` 模式下先攒着，返回空；`prev` 模式下当场扣并返回。
        """
        rho = float(rho)
        if self.mode == MODE_PREV:
            prev = self._prev.get(rho)
            if prev is None:
                self.n_dropped_nodark += 1
                return []
            return [(float(wl), rho, float(t), float(adc) - prev[1])]
        buf = self._pending[rho]
        buf.append((float(t), float(wl), float(adc)))
        if len(buf) > MAX_PENDING:
            self.n_dropped_nodark += len(buf) - MAX_PENDING
            del buf[:len(buf) - MAX_PENDING]
        return []

    # -- 内部 --------------------------------------------------------------
    def _resolve(self, rho, prev, cur) -> list[tuple]:
        t1, v1 = prev
        t2, v2 = cur
        buf = self._pending[rho]
        if not buf:
            return []
        gap = t2 - t1
        wide = gap > MAX_GAP_S or gap <= 0
        if wide:
            self.n_wide_gap += 1
        out = []
        keep = []
        for t, wl, adc in buf:
            if t1 <= t <= t2:
                # 间隔异常时退回用左端点，而不是跨着大洞线性插值
                d = v1 if wide else v1 + (v2 - v1) * (t - t1) / gap
                out.append((wl, rho, t, adc - d))
            elif t > t2:
                keep.append((t, wl, adc))        # 属于下一区间，留着
            else:
                self.n_dropped_nodark += 1       # 早于 t1，无左端点
        buf[:] = keep
        return out

    def reset(self) -> None:
        self._prev.clear()
        self._pending.clear()
        self.n_dropped_nodark = 0
        self.n_wide_gap = 0

    @property
    def n_pending(self) -> int:
        return sum(len(v) for v in self._pending.values())
