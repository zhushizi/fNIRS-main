# -*- coding: utf-8 -*-
"""L2.5 质量层 —— 只读不改，只产出标志位。

架构位置（`docs/双解码器架构.md` §二）：夹在 L2 原始记录与两条解码链之间。
**两条链都读它、都不写它**；它不修改任何数据。

为什么必须有（R1）：当前 2 距离 + 2 波长，空间残差与光谱残差**都恒为零**，
设备坏了不会报警，只会安静地给个错数。这一层是唯一还能抓到异常的地方。

### 告警疲劳

标志分两类，**呈现方式必须不同**，否则常驻项会把真异常淹没：

  · `KIND_CONFIG`  常驻配置状态（如「只有 2 波长 → 无光谱自检」）。
                   每次测量都成立，应当**只报一次**或放在固定位置。
  · 其余四类        本次测量的异常，随测随报。

### 阈值出处

全部来自实测，不是拍脑袋。见各常量上方注释。
"""

from __future__ import annotations

import collections
from dataclasses import dataclass, field

import numpy as np

# --- 标志分类 --------------------------------------------------------------
KIND_CONFIG = "config"     # 常驻配置状态，不是异常
KIND_LINK = "link"         # 串口链路
KIND_DATA = "data"         # 读数本身
KIND_PHYSICS = "physics"   # 物理/生理判据（由 srs.compute_toi 产出，这里只转达）
KIND_MONITOR = "monitor"   # 停用波长监视

SEV_INFO, SEV_WARN, SEV_FAIL = "info", "warn", "fail"

# --- 阈值 ------------------------------------------------------------------
# ADC 24 位有符号，满量程 8,388,607（2026-10-09 实测削顶值）。
ADC_FULL_SCALE = 8_388_607
SATURATION_FRAC = 0.90

# 暗电平量级上限。实测（三份数据，81 个 take）：
#   正常 3cm 暗电平 −48,894 ~ −42,931，4cm +3,186 ~ +12,483
#   |暗电平| 的 50/90 分位 = 41,897 / 50,060
# 取 200,000（约为正常值的 4 倍、满量程的 2.4%），只抓环境光漏入、
# 前级饱和这类量级性异常，不参与精细判别。
DARK_LEVEL_MAX = 200_000

# 尖峰剔除率。实测在体归档（窗口 N=10，稳健聚合的 n_rejected）：
#   稳定段 0~15%　　末段 10~20%　　过渡段（LED 刚亮，数据本身是垃圾）10~35%
# 两段有重叠，故**分两级**而不是一刀切 —— 一刀切会在重叠区要么漏要么误报。
REJECT_RATE_WARN = 0.15
REJECT_RATE_FAIL = 0.25

# 停用波长比值相对**会话基线**的偏离上限。
#
# ⚠️ 刻意不用绝对阈值：体模的 μa 谱是平的（比值由 μs′ 主导），组织有血红蛋白，
#    两者的比值范围根本不重合 —— 实测在体 850/770 低到 0.46，而体模是
#    1.23~1.51。**体模标定的绝对阈值搬到人体上会误报。**
#    相对自身基线的偏离则与介质无关。
#
# 实测在体归档（78 秒稳定段 vs LED 刚亮的过渡段）：
#       比值        稳定段最大偏离    过渡段最大偏离
#     810/770          5.9%          117,902%
#     850/770         11.3%           65,677%
#     950/770          8.0%          262,103%
# 两者差四个数量级。取 50%（正常波动的 4.4 倍），两边余量都极大。
#
# ⚠️ 仅一次会话、一名受试者的数据。多人多次后应重判。
MONITOR_DEV_MAX = 0.50
MONITOR_MIN_WINDOWS = 8        # 基线至少要这么多个窗口才生效

# 4cm 的比值**不可用作判据**：实测连干净的原探头都跨 1.08~12.28
# （810/770 @4cm），没有判别力。只在 3cm 上监视。
MONITOR_RHO_CM = 3.0


@dataclass(frozen=True)
class Flag:
    code: str                  # 机器可读，稳定不变
    text: str                  # 给人看的
    kind: str
    severity: str = SEV_WARN

    def __str__(self) -> str:
        return self.text


@dataclass
class QualityMonitor:
    """逐次测量产出标志位。**不修改任何输入。**

    用法：
        q = QualityMonitor(n_rho=2, n_wl=2, kb_calibrated=False)
        for flag in q.config_flags():      # 常驻项，启动时报一次
            ...
        flags = q.update(...)              # 每次反演后
    """

    n_rho: int = 2
    n_wl: int = 2
    kb_calibrated: bool = False
    # 进反演的波长。**必须给** —— 否则无法区分「反演通道坏了」和
    # 「监视通道坏了」，后者不该否决整次测量。
    #
    # ⚠️ 2026-10-22 真机回放抓到的设计错误：首版对所有通道一视同仁，
    #    结果在体归档的稳定段里 95% 的窗口被判 fail，全因 850nm/4cm 扣暗后
    #    非正 —— 而 850 是停用波长，那段的 TOI（86~89）是用 670/770 正常
    #    算出来的。监视通道坏掉只说明**该通道的监视失效**，不说明测量无效。
    active_wl: tuple = ()
    _ratio_hist: dict = field(default_factory=lambda: collections.defaultdict(list))

    def _is_active(self, w: float) -> bool:
        """没给 active_wl 时退化为「全部当反演通道」，保持旧行为可预期。"""
        if not self.active_wl:
            return True
        return any(abs(float(w) - float(a)) < 1e-6 for a in self.active_wl)

    # -- 常驻配置项 --------------------------------------------------------
    def config_flags(self) -> list[Flag]:
        """与本次测量无关的状态。**调用方应只呈现一次**，不要每次重复。"""
        out: list[Flag] = []
        if self.n_rho < 3 and self.n_wl < 3:
            out.append(Flag("no_selfcheck",
                            f"只有 {self.n_rho} 个距离 + {self.n_wl} 个波长："
                            "空间与光谱自检**均不可用**，异常只能靠物理判据",
                            KIND_CONFIG, SEV_WARN))
        elif self.n_rho < 3:
            out.append(Flag("no_spatial_selfcheck",
                            f"只有 {self.n_rho} 个距离：无空间残差自检",
                            KIND_CONFIG, SEV_INFO))
        elif self.n_wl < 3:
            out.append(Flag("no_spectral_selfcheck",
                            f"只有 {self.n_wl} 个波长：无光谱残差自检",
                            KIND_CONFIG, SEV_INFO))
        if not self.kb_calibrated:
            out.append(Flag("kb_uncalibrated",
                            "k/b 未标定：读数不是绝对血氧值，只能看趋势",
                            KIND_CONFIG, SEV_WARN))
        return out

    # -- 逐次测量 ----------------------------------------------------------
    def update(self, *, raw_by_channel=None, net_by_channel=None,
               n_used=0, n_rejected=0, dark_level=None,
               link=None, physics_flags=()) -> list[Flag]:
        """产出本次测量的标志。所有参数都可省略，省略的那项就不检。

        raw_by_channel : {(λ, ρ): 原始读数}        —— 查饱和
        net_by_channel : {(λ, ρ): 扣暗后读数}      —— 查非正、停用波长比值
        n_used/n_rejected : 稳健聚合的计数         —— 查尖峰率
        dark_level     : {ρ: 暗时隙读数}           —— 查暗电平量级
        link           : 有 n_dropped/n_bad_checksum/n_short_cycles 的对象
        physics_flags  : srs.compute_toi 产出的 flags，原样转达
        """
        out: list[Flag] = []
        out += self._check_link(link)
        out += self._check_saturation(raw_by_channel)
        out += self._check_nonpositive(net_by_channel)
        out += self._check_reject(n_used, n_rejected)
        out += self._check_dark(dark_level)
        out += self._check_monitor(net_by_channel)
        for t in physics_flags:
            out.append(Flag("physics", str(t), KIND_PHYSICS, SEV_FAIL))
        return out

    # -- 各判据 ------------------------------------------------------------
    @staticmethod
    def _check_link(link) -> list[Flag]:
        if link is None:
            return []
        out = []
        for attr, code, name in (("n_dropped", "link_dropped", "丢弃字节"),
                                 ("n_bad_checksum", "link_checksum", "校验错帧"),
                                 ("n_short_cycles", "link_short_cycle", "不完整周期")):
            v = getattr(link, attr, 0)
            if v:
                out.append(Flag(code, f"链路{name} {v}", KIND_LINK, SEV_WARN))
        return out

    def _split(self, bad):
        """把坏通道分成「反演的」与「监视的」—— 两者后果完全不同。"""
        act = [k for k in bad if self._is_active(k[0])]
        mon = [k for k in bad if not self._is_active(k[0])]
        return act, mon

    @staticmethod
    def _names(ks) -> str:
        return "、".join(f"{w:.0f}nm/{r:.0f}cm" for w, r in sorted(ks)[:4])

    def _check_saturation(self, raw) -> list[Flag]:
        if not raw:
            return []
        lim = ADC_FULL_SCALE * SATURATION_FRAC
        act, mon = self._split([k for k, v in raw.items()
                                if v is not None and abs(v) > lim])
        out = []
        if act:
            out.append(Flag("saturation",
                            f"{len(act)} 个**反演**通道接近满量程"
                            f"（>{SATURATION_FRAC:.0%}）：{self._names(act)}",
                            KIND_DATA, SEV_FAIL))
        if mon:
            out.append(Flag("saturation_monitor",
                            f"{len(mon)} 个监视通道接近满量程：{self._names(mon)}"
                            "（不影响反演，仅该通道监视失效）",
                            KIND_MONITOR, SEV_INFO))
        return out

    def _check_nonpositive(self, net) -> list[Flag]:
        if not net:
            return []
        act, mon = self._split([k for k, v in net.items()
                                if v is not None and v <= 0])
        out = []
        if act:
            out.append(Flag("nonpositive",
                            f"{len(act)} 个**反演**通道扣暗后非正"
                            f"（无法取对数）：{self._names(act)}",
                            KIND_DATA, SEV_FAIL))
        if mon:
            out.append(Flag("nonpositive_monitor",
                            f"{len(mon)} 个监视通道扣暗后非正：{self._names(mon)}"
                            "（不影响反演，仅该通道监视失效）",
                            KIND_MONITOR, SEV_INFO))
        return out

    @staticmethod
    def _check_reject(n_used: int, n_rejected: int) -> list[Flag]:
        tot = n_used + n_rejected
        if tot <= 0:
            return []
        rate = n_rejected / tot
        if rate >= REJECT_RATE_FAIL:
            return [Flag("reject_high",
                         f"尖峰剔除率 {rate:.0%}（≥{REJECT_RATE_FAIL:.0%}）："
                         "本次数据不可信",
                         KIND_DATA, SEV_FAIL)]
        if rate >= REJECT_RATE_WARN:
            return [Flag("reject_warn",
                         f"尖峰剔除率 {rate:.0%}（≥{REJECT_RATE_WARN:.0%}）：数据可疑",
                         KIND_DATA, SEV_WARN)]
        return []

    @staticmethod
    def _check_dark(dark) -> list[Flag]:
        if not dark:
            return []
        bad = {r: v for r, v in dark.items()
               if v is not None and abs(v) > DARK_LEVEL_MAX}
        if not bad:
            return []
        return [Flag("dark_level",
                     "暗电平异常：" + "、".join(f"{r:.0f}cm {v:,.0f}"
                                            for r, v in bad.items()),
                     KIND_DATA, SEV_FAIL)]

    def _check_monitor(self, net) -> list[Flag]:
        """停用波长相对 770 的比值，与**本会话基线**比。

        基线用的是历史中位数，天然排除开机瞬态。
        """
        if not net:
            return []
        ref = net.get((770.0, MONITOR_RHO_CM))
        if ref is None or ref <= 0:
            return []
        out = []
        for (w, r), v in sorted(net.items()):
            if r != MONITOR_RHO_CM or w in (0.0, 770.0) or v is None:
                continue
            if v <= 0:
                continue          # 非正通道算不出有意义的比值，跳过而不是污染基线
            ratio = v / ref
            hist = self._ratio_hist[w]
            if len(hist) >= MONITOR_MIN_WINDOWS:
                base = float(np.median(hist))
                if base != 0:
                    dev = abs(ratio - base) / abs(base)
                    if dev > MONITOR_DEV_MAX:
                        out.append(Flag(
                            f"monitor_{int(w)}",
                            f"{w:.0f}nm/770nm 比值 {ratio:.2f} 偏离会话基线 "
                            f"{base:.2f} 达 {dev:.0%}（限 {MONITOR_DEV_MAX:.0%}）"
                            "：疑似加性伪信号或耦合变化",
                            KIND_MONITOR, SEV_WARN))
            hist.append(ratio)
            if len(hist) > 500:              # 基线不必无限长
                del hist[:len(hist) - 500]
        return out

    def reset(self) -> None:
        """换受试者/换档位后必须调用 —— 基线不能跨条件沿用。"""
        self._ratio_hist.clear()


def summarize(flags) -> str:
    """把标志压成一行给界面用。

    两条「不制造告警疲劳」的规则：
      · 配置项（KIND_CONFIG）不进这一行 —— 它每次都成立，该固定呈现一次
      · SEV_INFO 不进这一行 —— 例如「监视通道非正」只是该通道监视失效，
        测量本身有效（2026-10-22 真机回放：首版把它当 fail，
        导致稳定段 95% 的窗口被误判）
    """
    live = [f for f in flags
            if f.kind != KIND_CONFIG and f.severity != SEV_INFO]
    if not live:
        return ""
    fail = [f for f in live if f.severity == SEV_FAIL]
    head = "✗ " if fail else "⚠ "
    return head + "；".join(f.text for f in (fail or live))
