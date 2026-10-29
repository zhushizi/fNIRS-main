# -*- coding: utf-8 -*-
"""单个采集通道的测量流水线 —— 扣暗 → 滑窗 → 稳健聚合 → SRS 反演 → 派生相对量。

2026-10-29 从 `online.py` 的 `OnlineWindow` 里**原样搬出**，为多通道同时计算
做准备。改造前这些状态是 25 个散落的 `self.*` 属性，一个窗口只能跟一个通道。

⚠️ **这是搬运，不是改写。** 搬进来的算法、注释、判据一个字没动 ——
   包括那些记着"为什么不那么做"的注释（它们是这个项目最贵的部分）。
   验收标准也是按搬运定的：单通道回放改造前后，`toi_hist` / `thi_hist` /
   `d_hist` / `auc_hist` 必须**逐点逐位相同**，不是"接近"。
   自检 t21 就在做这件事。

分工：
    ChannelPipeline   一个通道的测量态与算法。不碰界面、不碰串口。
    TimeBase          各通道**共享**的时间原点，这样多通道的曲线能叠在一起比。
    OnlineWindow      只负责按 byte10 把帧分发给对应的流水线，以及画图。
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

from srs_cal.darksub import DarkSubtractor
from srs_cal.quality import QualityMonitor
from srs_cal.robust import aggregate


@dataclass
class TimeBase:
    """各通道共享的数据时间轴。

    为什么共享：多通道的 TOI 要叠在同一张图上比左右差异，各通道
    各自取原点的话，两条曲线会错开最多一个扫描周期（约 1.5 s）——
    看起来像真实的时间差，其实只是原点不同。

    为什么用**数据时刻**（帧的 t_host）而不是「反演那一刻」的
    perf_counter：interp 扣暗下两者差约一个扫描周期，而且回放归档时
    后者毫无意义（一瞬间喂完几百秒数据）。自动基线要 60 s 窗、
    AUC 要积分时间，都必须用前者。
    """

    t: float | None = None      # 最近一个样本的 t_host
    t0: float | None = None     # 第一个样本的 t_host，不随清屏重置

    def mark(self, t: float) -> None:
        self.t = float(t)
        if self.t0 is None:
            self.t0 = float(t)

    def now(self) -> float:
        """数据时刻，以第一帧为 0。还没收到帧时退回 0。"""
        if self.t is None or self.t0 is None:
            return 0.0
        return self.t - self.t0


class ChannelPipeline:
    """一个采集通道的完整测量链。

    `geom` 是这个通道的几何+标定。⚠️ 2026-10-29 起四个通道**共用 1 号通道的
    标定**（用户决定：先不等每通道各标一份）。后果：非 1 号通道的 TOI
    **绝对值不可信**，只能看相对变化 —— 这一条必须在界面上显形，
    见 `OnlineWindow._refresh_chan_label`。
    """

    def __init__(self, channel: int, geom, *, wl, rho, need: int,
                 kb_calibrated: bool, timebase: TimeBase,
                 dark_win_len: int = 600, maxpts: int = 3600) -> None:
        self.channel = int(channel)
        self.geom = geom
        self._wl = list(wl)
        self._rho = list(rho)
        self._maxpts = int(maxpts)
        self._dark_win_len = int(dark_win_len)
        self.tb = timebase
        # L2.5 质量层。只读不改，两条链都读它、都不写它。
        # active_wl 必须给 —— 否则分不清「反演通道坏了」和「监视通道坏了」，
        # 后者（810/850/950）不该否决整次测量。
        self.quality = QualityMonitor(
            n_rho=len(self._rho), n_wl=len(self._wl),
            kb_calibrated=kb_calibrated,
            active_wl=tuple(float(w) for w in self._wl))
        self._keys = [(w, r) for w in self._wl for r in self._rho]
        self._need = int(need)
        # 本通道**自建起以来**收到的帧数。
        #
        # ⚠️ 刻意放在 `_init_state` 外面 —— 它不随清屏归零。
        #    界面拿它回答的是「这个通道到底发不发帧」（链路事实），
        #    不是「现在屏上有几个点」。清屏后归零会把"刚清掉"
        #    误报成"该通道无数据"（2026-10-29 实测到）。
        self.n_frames = 0
        self._init_state(int(need))

    # ------------------------------------------------------------------
    # 状态
    # ------------------------------------------------------------------
    def _init_state(self, need: int) -> None:
        self.win = {k: deque(maxlen=need) for k in self._keys}
        # 逐周期暗时隙。cycle_dark 是本周期的暗值；dark_win 留一段历史，
        # 用来显示暗电平自己漂多少 —— 它是链路健康度最直接的指标。
        self.cycle_dark: dict[float, float] = {}
        # 扣暗器：按时刻在前后两个暗之间插值，而不是减同周期那一个。
        # 噪声主成分是 0.0868 Hz 的电路/环境干扰（暗时隙里也有），
        # 而暗与信号在周期内差最多 1.5 s = 47° 相位，逐周期相减消不掉。
        # 代价是延迟 1 个扫描周期（约 1.5 s），相对 N×1.5 s 的窗口可忽略。
        # 详见 darksub.py 与 docs/双解码器架构.md §二·十。
        self._darksub = DarkSubtractor()
        self.dark_win = {r: deque(maxlen=self._dark_win_len) for r in self._rho}
        self._no_dark = 0
        self._fresh: set = set()
        self._since = 0
        self._starved: list[str] = []
        self.last_rejected = 0       # 上一次反演剔除的尖峰样本数
        self.last_qflags: list = []
        self._derived_err = ""
        self.last_I = None
        self.last_res = None
        self.last_n = 0
        self.last_flags: list[str] = []
        self._q_net: dict | None = None
        self._q_dark: dict | None = None
        self._last_used = 0
        # 派生相对量：ΔHbO/ΔHbR/ΔtHb/THI/ΔTOI/AUC。SRS 绝对浓度减基线 ——
        # 纯算术，延迟只有 SRS 自身的。基线与 AUC 的**全部判断**在
        # BaselineManager 里（逻辑层），这里只调方法、读状态。
        from fnirs_pipeline.srs import BaselineManager
        self.bl = BaselineManager()
        self.d_hist: dict = {k: deque(maxlen=self._maxpts)
                             for k in ("d_hbo", "d_hbr", "d_thb",
                                       "d_thi", "d_toi")}
        self.td_hist: deque = deque(maxlen=self._maxpts)
        # AUC 与基线解耦，自己一条时间轴：只要有参考点就记，重抓基线时不断档
        self.ta_hist: deque = deque(maxlen=self._maxpts)
        self.auc_hist: deque = deque(maxlen=self._maxpts)
        self.last_derived = None
        self.t_hist: deque = deque(maxlen=self._maxpts)
        self.toi_hist: deque = deque(maxlen=self._maxpts)
        # THI 绝对指数（k·tHb，µM）。不依赖基线，与 TOI 同一条时间轴。
        self.thi_hist: deque = deque(maxlen=self._maxpts)
        self.mua_hist: dict = {w: deque(maxlen=self._maxpts)
                               for w in self._wl}
    def reset(self) -> None:
        """清屏。

        **不**重置 TimeBase 的原点 —— 时间轴要连续。
        **也不**重置 `n_frames` —— 见它的定义，那是链路事实不是测量态。
        """
        need = self._need
        self._init_state(need)

    def resize(self, need: int) -> None:
        """平均窗口长度变了 —— 重建队列，保留已有样本的尾部。"""
        self.win = {k: deque(list(self.win.get(k, ()))[-need:], maxlen=need)
                    for k in self._keys}
        self._need = need

    # ------------------------------------------------------------------
    # 喂帧
    # ------------------------------------------------------------------
    def feed(self, f, need: int) -> None:
        """喂进**本通道**的一帧。调用方负责按 byte10 过滤。

        暗时隙排在每个周期最前：记下本周期的暗，后面的信号帧当场扣。
        这比标定文件里的静态暗电平强得多 —— 暗电平的漂移时间常数
        只有几个周期，静态值几秒钟就过期了。
        """
        self.n_frames += 1
        nk = len(self._keys)
        if f.is_dark:
            self.cycle_dark[f.distance_cm] = float(f.adc)
            self.dark_win[f.distance_cm].append(float(f.adc))
            # 暗时隙到达 → 上一周期攒着的信号帧现在有了右端点，可解析
            for wl, rho, t, net in self._darksub.feed_dark(
                    f.distance_cm, f.t_host, float(f.adc)):
                k = (wl, rho)
                if k in self._keys:
                    self.push_net(k, net, t, need, nk)
            return
        key = (f.wavelength_nm, f.distance_cm)
        if key not in self._keys:
            return                             # 810/950 照收不反演
        # interp 模式下这里只是攒着，返回空；prev 模式下当场返回
        for wl, rho, t, net in self._darksub.feed_signal(
                f.wavelength_nm, f.distance_cm, f.t_host, float(f.adc)):
            self.push_net((wl, rho), net, t, need, nk)
        self._no_dark = self._darksub.n_dropped_nodark

    def check_starved(self, need: int) -> None:
        """某一路长期不来（LED 没亮、协议变了）。不能默默停更 ——
        界面必须说清是哪一路，否则看起来就只是「卡住了」。"""
        if self._since > max(need * 20, 400):
            self._starved = [f"{w:.0f}nm/{r:.0f}cm" for w, r in self._keys
                             if (w, r) not in self._fresh
                             or len(self.win[(w, r)]) < need]
            self._since = 0

    def push_net(self, key, net: float, t: float, need: int, nk: int) -> None:
        """压入一个扣暗后的样本；收齐一轮就反演。

        从 `_on_frames` 的循环里抽出来 —— 插值扣暗会在**一个暗时隙到达时
        成批**吐出上一周期的信号帧，原先内联在循环里的判定已不适用。

        `t` 是这个样本所属信号帧的 t_host —— 光被测到的时刻。
        """
        self.tb.mark(t)
        self.win[key].append(float(net))
        self._fresh.add(key)
        self._since += 1
        # 判定放在每压入一个样本之后：一批 feed 里可能含多个扫描周期，
        # 放在批末会让出点率被串口的读节奏绑架，而不是由设备扫描速率决定。
        #
        # ⚠️ 2026-10-26 第二轮试过「窗口没满也出点」，**实测否决**：
        #    TOI 是光强的非线性函数（μa ∝ μeff²），单周期噪声不是散开而是**有偏**。
        #    实测「n 个周期算的 TOI」减「同一时刻 20 周期算的」：
        #      n=1  中位 0.2~7.2，p90 0.4~23.1
        #      n=10 中位 0.0~2.7，p90 0.1~7.9
        #    103952 开头几点读 42，真值 74。主读数上放一个错 32 点的数不可接受。
        #    改为：窗口没满时在图里画进度，不出数。想更快就把工具栏的「平均」调小。
        if (len(self._fresh) == nk
                and all(len(self.win[k]) >= need for k in self._keys)):
            n = min(len(self.win[k]) for k in self._keys)
            # ⚠️ 稳健聚合而非算术均值：实测每通道 2~6% 的周期是尖峰、
            #    幅度约 25 倍。窗口 N=10 时约 10% 的窗口被污染，
            #    一个尖峰会把该点的均值抬高约 2.5 倍。选型见 robust.py。
            ags = {(w, r): aggregate(self.win[(w, r)])
                   for w in self._wl for r in self._rho}
            I = np.array([[ags[(w, r)].value for r in self._rho]
                          for w in self._wl])
            self.last_rejected = sum(a.n_rejected for a in ags.values())
            self._last_used = sum(a.n_used for a in ags.values())
            # 质量层要的三份输入。扣暗值即 ags；暗电平取本周期的。
            self._q_net = {(float(w), float(r)): ags[(w, r)].value
                           for w in self._wl for r in self._rho}
            self._q_dark = {float(r): self.cycle_dark.get(float(r))
                            for r in self._rho}
            self._fresh.clear()
            self._since = 0
            self._starved = []
            self.invert(I, n)

    # ------------------------------------------------------------------
    # 反演
    # ------------------------------------------------------------------
    def invert(self, I: np.ndarray, n_cycles: int, link=None) -> None:
        from fnirs_pipeline.srs import compute_toi

        res = compute_toi(I, self.geom)
        t = self.tb.now()
        self.last_I = I
        self.last_res = res
        self.last_n = n_cycles
        self.last_flags = res.flags
        # ⚠️ 物理判据由 compute_toi 产出，这里只转达；质量层不重复计算。
        self.last_qflags = self.quality.update(
            net_by_channel=self._q_net,
            n_used=self._last_used,
            n_rejected=self.last_rejected,
            dark_level=self._q_dark,
            link=link if link is not None else self._link,
            physics_flags=res.flags)   # 现在只含真异常
        if np.isfinite(res.toi):
            self.t_hist.append(t)
            self.toi_hist.append(res.toi)
            for j, w in enumerate(self._wl):
                self.mua_hist[w].append(res.mua[j])
            self.thi_hist.append(res.thb_uM)
        self.update_derived(res, I, t)

    # 链路对象（有 n_dropped / n_bad_checksum / n_short_cycles）。
    # 由窗口在每次分发前塞进来 —— 流水线自己不碰串口。
    _link = None

    def update_derived(self, res, I, t: float) -> None:
        """每次反演后调用。判断全在 BaselineManager 里，这里只记历史。

        整个包在 try 里 —— 这一格坏掉不能影响 TOI。
        """
        try:
            d = self.bl.push(t, res, I)
            self.last_derived = d
            if d is not None:
                self.td_hist.append(t)
                for k in self.d_hist:
                    self.d_hist[k].append(getattr(d, k))
            if self.bl.auc.ref_toi is not None:
                self.ta_hist.append(t)
                self.auc_hist.append(self.bl.auc.value_min)
        except Exception as e:                               # noqa: BLE001
            self._derived_err = f"{type(e).__name__}: {e}"

    def break_derived_line(self) -> None:
        """基线换了：在相对量曲线里插一个 NaN，断开前后两段。

        历史**不清掉** —— 旧的那段是相对旧基线算的，本身没错，删了反而丢信息；
        但两段不该连成一条线，否则重抓那一刻会画出一条假的跳变斜线。
        """
        if self.td_hist:
            self.td_hist.append(self.td_hist[-1])
            for d in self.d_hist.values():
                d.append(float("nan"))

    def break_auc_line(self) -> None:
        """AUC 重置：同理在 AUC 曲线里断一刀。"""
        if self.ta_hist:
            self.ta_hist.append(self.ta_hist[-1])
            self.auc_hist.append(float("nan"))

    def break_line(self) -> None:
        """这个通道停了一下又开了：在**所有**曲线里插一道断口。

        为什么不是清屏：停之前那段数据本身没有问题，恰恰常常是操作者
        想留着看的那一段。清掉等于把停它的理由也扔了。
        但两段之间有一段空白时间，直连会画出一条横跨空档的假斜线 ——
        所以插 NaN 断开（曲线侧要 `connect="finite"` 才生效）。

        沿用本项目已有的写法（见 `break_derived_line`）：时间轴重复上一个
        时刻、数值填 NaN。不往 t_hist 里塞 NaN —— x 轴范围是拿
        `ts[0]`/`ts[-1]` 算的，塞了就成 NaN 了。
        """
        if not self.t_hist:
            return
        if self.toi_hist and np.isnan(self.toi_hist[-1]):
            return                      # 已经断过了，别连插几道
        self.t_hist.append(self.t_hist[-1])
        self.toi_hist.append(float("nan"))
        self.thi_hist.append(float("nan"))
        for d in self.mua_hist.values():
            d.append(float("nan"))
        self.break_derived_line()
        self.break_auc_line()

    def capture_baseline(self) -> None:
        self.bl.capture()
        self.last_derived = None

    def __repr__(self) -> str:                               # pragma: no cover
        toi = "—" if self.last_res is None else f"{self.last_res.toi:.2f}"
        return (f"<ChannelPipeline ch{self.channel} 帧{self.n_frames} "
                f"点{len(self.toi_hist)} TOI={toi}>")
