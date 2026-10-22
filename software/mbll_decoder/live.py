# -*- coding: utf-8 -*-
"""把 MBLL 接到实时流上 —— 攒一段，整段重算。

为什么不做流式滤波
------------------
`decoder.decode` 用的是 `sosfiltfilt`（零相位，正反各滤一遍），
**本质上没法流式化**。换成因果 IIR 可以逐点出数，但那样：

  · 带内相位滞后，0.01~0.05 Hz 上的群延迟是几十秒量级；
  · 界面上的数会和 `python -m mbll_decoder.run` 跑出来的**不一样**。

第二条是致命的：同一份数据两条路径给两个答案，出了问题没法定位。
所以这里的做法是**攒缓冲 + 定期整段重算**，和离线路径跑的是同一个
`decode()`、同一套参数，结果逐点相同（自检 t16 验这一条）。

代价是重算开销，实测可以忽略：

    缓冲点数     时长        decode 耗时
       400      10 分钟       1.9 ms
      2000      51 分钟       1.4 ms
     20000     8.5 小时       2.6 ms
     50000      21 小时       5.2 ms

每 5 秒重算一次，CPU 占用在千分之一量级。

输入是什么
----------
**扣暗之后的光强**，和离线路径 `run.load()` 拿到的是同一个东西
（都来自同一个 `DarkSubtractor`）。这是架构里「两条链唯一的共同输入」
的含义 —— 再往下的预处理各走各的。

⚠️ 不做去尖峰：离线路径也不做。两边必须一致，否则就不是同一个算法了。
"""

from __future__ import annotations

import sys
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
if str(_HERE.parent) not in sys.path:
    sys.path.insert(0, str(_HERE.parent))

from mbll_decoder import hardware as mh           # noqa: E402
from mbll_decoder.decoder import MbllResult, decode  # noqa: E402

# 缓冲上限。与 online.py 的 MAXPTS 同量级：20000 个扫描周期，
# 按实测 0.657 周期/秒约 8.5 小时。满了丢最旧的。
MAXPTS = 20000
# 重算间隔（秒）。带通上限 0.05 Hz 对应 20 秒周期，5 秒远够用。
REDECODE_S = 5.0


@dataclass
class LiveStats:
    """给界面看的计数 —— 出问题时能分辨是「还没够」还是「一直在丢」。"""

    n_pushed: int = 0          # 收到的扣暗样本数
    n_cycles: int = 0          # 攒成完整周期的数量
    n_partial: int = 0         # 波长没凑齐就被下一周期顶掉的次数
    n_decode: int = 0          # 重算次数


@dataclass
class LiveMbll:
    """实时 MBLL 缓冲器。**不碰 Qt，也不碰 SRS 的任何状态。**

    用法：
        live = LiveMbll()
        live.push(wl, rho, t_host, net)     # 每个扣暗样本喂一次
        got = live.poll(now)                # 到点了返回新结果，否则 None
    """

    rho: float = mh.DISTANCE_CM
    wavelengths: tuple = tuple(float(w) for w in mh.WAVELENGTHS_NM)
    maxlen: int = MAXPTS
    redecode_s: float = REDECODE_S
    age: float = mh.DEFAULT_AGE
    band: tuple = (mh.BP_LOW_HZ, mh.BP_HIGH_HZ)

    stats: LiveStats = field(default_factory=LiveStats)

    def __post_init__(self) -> None:
        self._t: deque = deque(maxlen=self.maxlen)
        self._I: deque = deque(maxlen=self.maxlen)
        self._pend: dict = {}
        self._last_decode = -1e18
        self.last: MbllResult | None = None
        self.t_offset = 0.0        # 最后一次重算时，缓冲最旧样本的绝对时刻

    # -- 输入 ---------------------------------------------------------------
    def push(self, wl: float, rho: float, t: float, net: float) -> None:
        """喂一个扣暗后的样本。只收本距离、本波长集的。

        ⚠️ 这个函数在 SRS 的数据路径上被调用，**任何异常都会打断测量**。
           调用方负责包 try —— 但这里也不做任何可能抛的事。
        """
        if abs(float(rho) - self.rho) > 1e-6:
            return
        wl = float(wl)
        if wl not in self.wavelengths:
            return
        self.stats.n_pushed += 1
        if wl in self._pend:
            # 上一周期没凑齐就来了下一周期的同一波长 —— 丢掉残缺的那组。
            # 不能把两个周期的读数拼成一条样本：那会把时间轴弄错。
            self.stats.n_partial += 1
            self._pend.clear()
        self._pend[wl] = (t, float(net))
        if len(self._pend) < len(self.wavelengths):
            return
        # 凑齐一个周期。时刻取最后一个波长的 —— 与 run.load() 一致
        # （那边取 wl[-1] 的时刻），两条路径的时间轴才对得上。
        ref = self.wavelengths[-1]
        self._t.append(self._pend[ref][0])
        self._I.append([self._pend[w][1] for w in self.wavelengths])
        self._pend.clear()
        self.stats.n_cycles += 1

    # -- 输出 ---------------------------------------------------------------
    def poll(self, now: float, force: bool = False) -> MbllResult | None:
        """到点就整段重算并返回结果；没到点返回 None。

        `now` 用调用方的时钟（与 push 的 t 同源即可）。
        """
        if not force and now - self._last_decode < self.redecode_s:
            return None
        self._last_decode = now
        if len(self._t) < 8:
            return None
        t = np.fromiter(self._t, float, len(self._t))
        I = np.array(self._I, float).T
        self.stats.n_decode += 1
        self.t_offset = float(t[0])
        self.last = decode(t, I, wavelengths=np.array(self.wavelengths),
                           distance_cm=self.rho, age=self.age, band=self.band)
        return self.last

    # -- 状态 ---------------------------------------------------------------
    @property
    def span_s(self) -> float:
        """缓冲里覆盖的时长。"""
        return float(self._t[-1] - self._t[0]) if len(self._t) >= 2 else 0.0

    def need_s(self) -> float:
        """还差多少秒才够高通起步。≤0 表示够了。"""
        return max(0.0, mh.MIN_RECORD_S - self.span_s)

    def status(self) -> str:
        """一行状态，直接贴界面。"""
        if self.need_s() > 0:
            return (f"攒数据中　{self.span_s:.0f} / {mh.MIN_RECORD_S:.0f} s"
                    f"（高通 {self.band[0]} Hz 起步所需，还差 "
                    f"{self.need_s():.0f} s）")
        r = self.last
        if r is None:
            return f"已攒 {self.span_s:.0f} s，等待首次解算"
        if not r.ok:
            return "未出结果：" + "；".join(r.flags[-1:])
        return (f"{self.span_s:.0f} s　{len(self._t)} 点　"
                f"ΔHbO std {r.d_hbo.std(ddof=1):.3f} / "
                f"ΔHbR std {r.d_hbr.std(ddof=1):.3f} µM")

    def reset(self) -> None:
        self._t.clear(); self._I.clear(); self._pend.clear()
        self._last_decode = -1e18
        self.last = None
        self.t_offset = 0.0
        self.stats = LiveStats()
