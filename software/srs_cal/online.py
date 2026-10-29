# -*- coding: utf-8 -*-
"""在线解码器 —— 连上设备，实时出 SRS 的 TOI 与派生相对量。

    python -m srs_cal.online --cal 标定_v1.json      # 用标定文件
    python -m srs_cal.online                          # 无标定，只看相对变化
    python -m srs_cal.online --simulate               # 无硬件排练

和采集程序的分工
----------------
    acquire.py      体模标定【采数据】——  离线，12 块各点一次
    run_srs_v1.py   体模标定【算 G】  ——  离线，产出 标定参数/*.json
    online.py       这个           ——  在线，实时反演 TOI 与派生相对量

界面（2026-10-26 改版，见 docs/online优化计划.md）
------------------------------------------------
    主页    TOI（+ 基线虚线、AUC 警戒线）/ ΔHbO·ΔHbR·ΔtHb / ΔTHI·ΔTOI / AUC
    调试    各波长 μa（反演中间量）
    时间窗可选 1 分钟 / 10 分钟 / 1 小时 / 全程；可打事件标记。

TOI 逐帧独立反演，不依赖任何基线 —— 贴上就出数。
相对量（Δ 系列、THI、AUC）需要基线：开机后自动等信号稳定再设，
也可「抓取当前」或手填 TOI。基线与 AUC 的全部判断在
`fnirs_pipeline.srs.BaselineManager`，这里只调方法、读状态。

MBLL 已不在这里跑（2026-10-26）：带通 0.01~0.05 Hz 看不见持续数分钟的变化
（合成 4 分钟 −10 µM 的阻断，平台期解出 +0.09），且要等 300 s 才出数。
模块与命令行 `python -m mbll_decoder.run <归档>` 保留，归档里的原始帧随时可离线重算。

⚠️ 当前标定的状态
    810 nm 停用（4cm 读数有 54~87% 不是穿过体模的光，见 hardware.py）。
    反演走 670/770/810/850 四路。810 曾因暗电平未扣准被误判为光路泄漏，
    固件加暗时隙后实测偏离回到 ±0.03，已恢复（见 hardware.py）。

    **k/b 未标定。** 体模矩阵只能修两个探测器之间的增益失配（G），
    修不了 TOI 的绝对标度 —— 体模不含血红蛋白，测不出 SO₂ 的真值。
    所以这个程序现在给的是**趋势和相对变化**，不是可信的绝对血氧值。
    界面上会一直挂着这条提示，直到标定文件里 k/b 不是 1/0。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets

_HERE = Path(__file__).resolve().parent
CAL_DIR = _HERE / "标定参数"      # 标定 json 都放这里，由 run_srs_v1 --emit-cal 产出
# 不带 --cal 时加载哪一个。**刻意写成具名常量，不是「取目录里最新的」**：
# 那样会把一次顺手的 --emit-cal 产物静默变成在用标定（2026-10-12 已因
# 默认落点踩过两次）。换版本就改这一行，在 git 里看得见。
# 2026-10-16：v3 → v5。**标定不跨光学套件转移**，探头换了就必须换标定：
#   旧探头的 v3 配新探头，真值 70 的介质读 37.2（误差 −32.8 %O₂），
#   比完全不标定（48.2）还差 11 个点。ΔG 达 −0.218/−0.222（贴合噪声的 7.3 倍）。
# v5 = 探头优化版数据 + 停用 850（见 hardware.py），TOI 离散 ±1.59 %O₂。
# ⚠️ v5 仅 6 块 × 1 次，是**临时版**；补满 12 块 × 3 次后重出并改这一行。
DEFAULT_CAL = "标定_v5_探头优化_双波长.json"
sys.path.insert(0, str(_HERE.parent))

from srs_cal import hardware as hw                       # noqa: E402
from srs_cal.acquire import (BAUDRATES, BRIGHTNESS_SETTLE_MS,    # noqa: E402
                             DEFAULT_GEAR, DEFAULT_PORT,
                             SerialWorker, SimWorker)
from srs_cal.archive import RawArchive                   # noqa: E402
from srs_cal.robust import aggregate                     # noqa: E402
from fnirs_pipeline.srs import (AUC_REL_DROP, BASELINE_N,  # noqa: E402
                                SETTLE_MAX_REL, SETTLE_MAX_TOI_PTS,
                                SETTLE_WIN_S, BaselineManager)
from srs_cal.chpipe import ChannelPipeline, TimeBase     # noqa: E402
from srs_cal.quality import summarize                    # noqa: E402
from srs_cal.protocol import (BRIGHTNESS_DEFAULT, BRIGHTNESS_MAX,  # noqa: E402
                              BRIGHTNESS_MIN, CHANNEL_MAX, CHANNEL_MIN,
                              CMD_START, CMD_START_CH2, CMD_STOP_CH1,
                              EXPECTED_CHANNEL, build_brightness, build_cmd,
                              frames_per_cycle)

# DEFAULT_PORT / DEFAULT_GEAR 定义在 acquire.py，这里只 import（见上方 import 行）。
# 两个程序共用一份，避免各改各的、悄悄跑偏。命令行 --port / --gear 可临时覆盖。

WL = hw.WAVELENGTHS_ACTIVE_NM                 # 当前 670 / 770（810、850 已停用）
RHO = hw.RHO_CM
# 与 liveplot.py 同一套配色。键要覆盖**全部**器件波长，不能只列当前
# 在用的几路 —— WAVELENGTHS_ACTIVE_NM 变过（810 停用又恢复），
# 按当时在用的波长写死，下次一改就 KeyError。
COLOR = {0.0: "#2a78d6", 670.0: "#d62728", 770.0: "#ff7f0e",
         810.0: "#2ca02c", 850.0: "#1f77b4", 950.0: "#999999"}
# 各采集通道在 TOI 叠图里的颜色。1 号沿用原来那个绿 —— 既有的截图、
# 文档和操作者的记忆都挂在这个颜色上，不要因为加了通道就换掉它。
CHAN_COLOR = {1: "#1a7f5a", 2: "#8e44ad", 3: "#d35400", 4: "#16a085"}
MAXPTS = 20000
REFRESH_HZ = 10
# 默认合并多少个扫描周期出一个 TOI。
#
# ⚠️ 2026-10-22 由 10 改为 20，依据是 10 分钟体模静置实测（真值恒定，
#    离散全部是仪器的）。原值注释写「≈1 秒」是按早期假设的帧率写的，
#    **真实扫描速率只有 0.657 周期/秒**，10 个周期其实是 15 秒。
#
#      N    窗口 s   滞后 s   TOI std（插值扣暗后）
#      10    15.2     7.6      3.06
#      20    30.4    15.2      1.59   ← 取此
#      40    60.9    30.4      1.21
#      80   121.8    60.9      0.84
#
#    10→20 降 2.5 倍只付 2 倍滞后；20→40 只再降 1.3 倍却又付 2 倍滞后。
#    拐点在 20，且此时仪器噪声 1.59 恰好等于 v5 标定自身的块间离散
#    （±1.59 %O₂）—— 再往下降，瓶颈就不是它了。
#
#    滑动窗口：加大 N **不降低出点率**，代价只是滞后（约 N/2 个周期）。
DEFAULT_AVG = 20
# 基线攒多少个点（BASELINE_N）及其理由已移到 fnirs_pipeline/srs.py，
# 与 BaselineManager 放在一起 —— 安卓端拿到的是同一个数。
#
# 时间窗选项。2026-10-26 前固定 60 s：看得清逐点抖动，但基线漂移与 AUC
# 是几十分钟到几小时的量，60 s 窗里根本看不出来。BRS-100 的窗口可设、
# 可回看整段（说明书 p35）。历史本来就留着（MAXPTS 2 万点 ≈ 8.5 小时）。
SPAN_CHOICES = (("1 分钟", 60.0), ("10 分钟", 600.0),
                ("1 小时", 3600.0), ("全程", None))
DEFAULT_SPAN_IDX = 1


def _mmss(seconds: float) -> str:
    s = int(round(seconds))
    return f"{s // 60:02d}:{s % 60:02d}"


class Calibration:
    """标定文件。没有文件时退化为「未标定」，但绝不假装已标定。"""

    def __init__(self, doc: dict | None = None) -> None:
        self.doc = doc or {}
        self.wl_error = ""           # 波长集不匹配时的说明，空串表示通过
        self.gain = (np.asarray(doc["gain_correction"], float)
                     if doc and "gain_correction" in doc else None)
        if self.gain is not None:
            self.wl_error = self._check_wavelengths(doc)
            if self.wl_error:
                # ⚠️ 不匹配时**拒绝加载**，退化为未标定，而不是带病运行。
                self.gain = None
        # 标定文件里的暗只是留档（键名已改为 dark_level_archive_only）。
        # 在线一律用设备每周期的暗时隙 —— 静态暗几秒就过期。
        self.dark = None
        self.k = float(doc.get("k", 1.0)) if doc else 1.0
        self.b = float(doc.get("b", 0.0)) if doc else 0.0

    @staticmethod
    def _check_wavelengths(doc: dict) -> str:
        """标定文件的波长集必须与 `hw.WAVELENGTHS_ACTIVE_NM` **逐项相同**。

        ⚠️ 这条校验防的不是「长度不同」—— 那会被 numpy 广播挡下来，是响的。
           防的是**长度相同而波长不同**：那种情况减法照常成功，
           G(λ₁) 被套到 λ₂ 通道上，屏幕照常出数，**无任何提示**。
           例：将来 850 修好而 770 出问题，硬件变成 [670, 850]，
           此时加载 v5 的 [670, 770] —— 长度都是 2，静默算错。
           标定参数目录现已有 4/3/2 三种长度混放，该场景迟早撞上。
        """
        cal_wl = doc.get("wavelengths_nm")
        if cal_wl is None:
            return "标定文件缺 wavelengths_nm 字段，无法确认波长集"
        cal = [float(x) for x in cal_wl]
        cur = [float(x) for x in WL]
        if len(cal) != len(cur):
            return (f"波长数不符：标定 {len(cal)} 路 {cal}，"
                    f"硬件在用 {len(cur)} 路 {cur}")
        bad = [(a, b) for a, b in zip(cal, cur) if abs(a - b) > 1e-6]
        if bad:
            return (f"波长集不符：标定 {cal}，硬件在用 {cur}"
                    f"　（差异处 {bad}）")
        g = doc.get("gain_correction") or []
        if len(g) != len(cur):
            return f"gain_correction 长度 {len(g)} 与波长数 {len(cur)} 不符"
        return ""

    @property
    def calibrated(self) -> bool:
        return self.gain is not None

    @property
    def kb_set(self) -> bool:
        return self.k != 1.0 or self.b != 0.0

    def banner(self) -> tuple[str, bool]:
        """(提示文字, 是否可放心用)。"""
        if self.wl_error:
            # 波长集不符是**配置错误**，不是「忘了给标定」，措辞要分开。
            return (f"❌ 标定已拒绝加载：{self.wl_error}　"
                    f"→ 现按未标定运行（G=0），读数只能看相对变化。"
                    f"请换匹配的标定文件，或改 hardware.WAVELENGTHS_ACTIVE_NM。",
                    False)
        if not self.calibrated:
            return ("⚠ 未加载标定：G(λ)=0、暗电平=0。读数只能看**相对变化**，"
                    "绝对值无意义。用 --cal 指定标定文件。", False)
        c = self.doc.get("caveats", [])
        sd = self.doc.get("expected_toi_sd_pct")
        head = f"标定 {self.doc.get('version','?')}（{self.doc.get('created','?')}）"
        if sd:
            head += f"　预期重复性 ±{sd:.1f} %O₂"
        if not self.kb_set:
            head += "　⚠ k/b 未标定 → **不是绝对血氧值**，只看趋势"
        return (head + ("　｜ " + "；".join(c[:2]) if c else ""), self.kb_set)

    @staticmethod
    def load(path: Path) -> "Calibration":
        doc = json.loads(path.read_text(encoding="utf-8"))
        got = [float(x) for x in doc.get("wavelengths_nm", [])]
        if got and (len(got) != WL.size or not np.allclose(got, WL)):
            raise ValueError(
                f"标定文件的波长 {got} 与当前反演波长 {list(WL)} 不一致。\n"
                "混用会让 G(λ) 对错波长 —— 拒绝加载。")
        return Calibration(doc)


class OnlineWindow(QtWidgets.QWidget):
    def __init__(self, cal: Calibration, simulate: bool = False,
                 archive_dir=None) -> None:
        super().__init__()
        self.cal = cal
        self.simulate = simulate
        # 归档落点。None = 用 archive.DEFAULT_DIR。
        # 可配置是医疗设备的基本要求：换盘、指定数据区、测试时隔离。
        self.archive_dir = archive_dir
        self.worker: QtCore.QThread | None = None
        self.archive: RawArchive | None = None
        # 显式传 WL —— 跟着 WAVELENGTHS_ACTIVE_NM 走，别依赖 geometry() 的默认。
        # ⚠️ **不传 dark_level**：暗电平已经在 _on_frames 里用每个周期的
        #    暗时隙逐帧扣过了。再把标定文件里的静态暗传进来会扣两次。
        #    标定文件里的 dark_level 只对旧固件（无暗槽）有意义。
        #
        # ⚠️ **四个通道共用这一份 geom**（2026-10-29 用户决定：先不等每通道
        #    各标一份）。换通道 = 换探头 = 换光学套件，G(λ) 不跨套件转移，
        #    所以非 1 号通道的 TOI **绝对值不可信**，只能看相对变化。
        #    这一条在 `_refresh_chan_label` 里对操作者显形。
        self._geom_shared = hw.geometry(gain_correction=cal.gain,
                                        k=cal.k, b=cal.b, wavelengths_nm=WL)
        # 各通道共享的时间原点 —— 多通道的曲线要能叠在一张图上比左右差异。
        self.tb = TimeBase()
        # 通道号 → 该通道的测量流水线。**以回传为准**：见到哪个通道的帧
        # 就给它建一条（_on_frames 里），不按我们下发过什么建。
        # 这里先建默认通道的那条，使单通道行为与改造前完全一致。
        self.pipes: dict[int, ChannelPipeline] = {}
        self._view_ch = EXPECTED_CHANNEL      # 界面当前在看哪个通道
        # 工具栏「平均」的当前值。新建的流水线要按它开窗口，否则后来
        # 出现的通道会用默认长度，两条曲线的平滑程度不一样。
        self._need_ui = DEFAULT_AVG
        self._pipe_for(EXPECTED_CHANNEL)
        # 通道校验：**以回传为准**。下发了 byte6 不等于设备真的切了 ——
        # 2026-10-28 实测：下发通道 2，292 帧回来 byte10 全是 01，
        # 且十路读数与通道 1 相差都在 ±1% 内（就是两次采集的正常起伏）。
        # 所以不能拿「我发过」当「它切了」，只认数据帧自己标的通道号。
        self._n_chan_ok = 0
        self._n_chan_bad = 0
        self._chan_seen: set = set()
        self._frames = 0
        self._last_frames = 0
        self._fps = 0.0
        # 事件标记：(数据时刻, 文字)。界面上画竖线，并写到归档旁的 .events.csv
        # **全局**，不按通道分 —— 按一次标记是操作者的一个动作，
        # 那条竖线应该画在所有通道的图上。
        self.events: list[tuple[float, str]] = []
        self._event_lines: list = []
        self._build()
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(int(1000 / REFRESH_HZ))
        self._rate = QtCore.QTimer(self)
        self._rate.timeout.connect(self._tick_rate)
        self._rate.start(1000)

    # ------------------------------------------------------------------
    # 通道流水线
    # ------------------------------------------------------------------
    # 2026-10-29：测量态从本类搬进了 ChannelPipeline（见 chpipe.py）。
    # 下面这组转发属性让**界面代码一行不用改** —— `self.toi_hist` 仍然
    # 读得到，只是读的是"当前在看的那个通道"的。这是刻意的：
    #   ① 纯搬运才好验等价（自检 t21 要逐点逐位比对改造前的输出）；
    #   ② 把通道维散成 60 个下标，漏一处就是"ch2 的数据写进 ch1 的历史"，
    #      而这种错在界面上看不出来。集中在一处反而能查。
    _FWD = ("win", "_keys", "_need", "_fresh", "_since", "_starved",
            "cycle_dark", "dark_win", "_darksub", "_no_dark",
            "geom", "quality", "last_qflags", "_derived_err",
            "bl", "d_hist", "td_hist", "ta_hist", "auc_hist", "last_derived",
            "t_hist", "toi_hist", "thi_hist", "mua_hist",
            "last_I", "last_res", "last_n", "last_flags", "last_rejected",
            "_q_net", "_q_dark", "_last_used")

    def _pipe_for(self, ch: int) -> ChannelPipeline:
        """取（必要时新建）某通道的流水线。

        新建的时机是**见到该通道的第一帧**，不是下发该通道的指令 ——
        延续本文件一贯的"以回传为准"：下发了不等于设备开了。
        """
        p = self.pipes.get(int(ch))
        if p is None:
            p = self.pipes[int(ch)] = ChannelPipeline(
                ch, self._geom_shared, wl=WL, rho=RHO, need=self._need_ui,
                kb_calibrated=self.cal.kb_set, timebase=self.tb,
                maxpts=MAXPTS)
        return p

    @property
    def pipe(self) -> ChannelPipeline:
        """界面当前在看的那条流水线。

        ⚠️ 必须跟着通道下拉走（`_on_channel` 改 `_view_ch`）。
           漏了这一步，界面会一直显示 1 号通道的曲线却标着别的通道号 ——
           自检 t19 的「选通道 3 却只收到通道 1 的帧 → 一个点都不出」
           就是靠这一条抓出来的。
        """
        return self._pipe_for(self._view_ch)

    def _now(self) -> float:
        """数据时刻，以第一帧为 0。还没收到帧时退回 0。"""
        return self.tb.now()

    # -- UI ----------------------------------------------------------------
    def _build(self) -> None:
        self.setWindowTitle("在线解码器 —— SRS TOI 与派生相对量"
                            + ("（模拟器）" if self.simulate else ""))
        self.resize(1120, 980)
        pg.setConfigOption("background", "w")
        pg.setConfigOption("foreground", "k")
        root = QtWidgets.QVBoxLayout(self)

        bar = QtWidgets.QHBoxLayout()
        self.cmb_port = QtWidgets.QComboBox(); self.cmb_port.setMinimumWidth(210)
        self.cmb_baud = QtWidgets.QComboBox(); self.cmb_baud.addItems(BAUDRATES)
        self.cmb_baud.setEditable(True)
        self.btn_refresh = QtWidgets.QPushButton("刷新")
        self.btn_conn = QtWidgets.QPushButton("连接")
        self.spn_avg = QtWidgets.QSpinBox()
        self.spn_avg.setRange(1, 300); self.spn_avg.setValue(DEFAULT_AVG)
        self.spn_avg.setSuffix(" 周期/点")
        self.spn_avg.setToolTip("\n".join((
            "每点合并多少个扫描周期。滑动窗口 —— 加大它不降低出点率，",
            "代价只是滞后（约 N/2 个周期）。",
            "",
            "10 分钟体模静置实测（真值恒定，离散全是仪器的）：",
            "   N=10　窗口 15.2s　滞后  7.6s　TOI std 3.06",
            "   N=20　窗口 30.4s　滞后 15.2s　TOI std 1.59  ← 默认",
            "   N=40　窗口 60.9s　滞后 30.4s　TOI std 1.21",
            "   N=80　窗口121.8s　滞后 60.9s　TOI std 0.84",
            "",
            "实测扫描速率 0.657 周期/秒，一个周期约 1.5 秒。")))
        # 设备上电即连续发数据、不响应启停，但**接受光源档位指令**
        # （厂家 2026-10-10）。所以没有开始/停止按钮，只有档位。
        self.spn_bright = QtWidgets.QSpinBox()
        self.spn_bright.setRange(BRIGHTNESS_MIN, BRIGHTNESS_MAX)
        self.spn_bright.setValue(BRIGHTNESS_DEFAULT)
        self.spn_bright.setSuffix(f"/{BRIGHTNESS_MAX}")
        self.spn_bright.setToolTip(
            f"光源档位 0~{BRIGHTNESS_MAX} 共 {BRIGHTNESS_MAX + 1} 档，"
            f"上电默认 {BRIGHTNESS_DEFAULT}。改动立即下发。\n"
            "⚠️ G(λ) 是在某个档位下标定的。换档位会不会改 LED 谱形尚未验证 ——\n"
            "   在线测量建议保持与标定时相同的档位。")
        # 同 acquire：连点不逐帧下发，静默期后只发最后一次。
        self.spn_bright.setKeyboardTracking(False)
        self._bright_timer = QtCore.QTimer(self)
        self._bright_timer.setSingleShot(True)
        self._bright_timer.setInterval(BRIGHTNESS_SETTLE_MS)
        self._bright_timer.timeout.connect(self._send_brightness)
        self.spn_bright.valueChanged.connect(self._bright_timer.start)
        # 采集通道。与档位共用同一条指令帧（byte5 档位、byte6 通道），
        # 所以改通道也走 _send_brightness。
        self.cmb_chan = QtWidgets.QComboBox()
        for c in range(CHANNEL_MIN, CHANNEL_MAX + 1):
            self.cmb_chan.addItem(f"通道 {c}", c)
        self.cmb_chan.setToolTip("\n".join((
            f"采集通道 {CHANNEL_MIN}~{CHANNEL_MAX}。改动立即下发"
            "（指令帧 byte6，与光源档位同一帧）。",
            "",
            "通道是**累加**的，不是切换（厂家文档「累加依次」+ 2026-10-29 实测）：",
            "   下发 2 号之后 1 号仍在跑，设备把帧率翻倍，",
            "   每个通道自己的周期率不变（恒 0.656 周期/s）。",
            "   要关掉某个通道，用「关闭通道一」那类停止帧（byte4=0x00）。",
            "",
            "⚠️ 换通道 = 换探头 = 换光学套件，**G(λ) 标定不跨套件转移**。",
            "   当前四个通道**共用 1 号通道的标定**（2026-10-29 决定），",
            "   所以非 1 号通道的 TOI **绝对值不可信**，只能看相对变化。")))
        self.cmb_chan.currentIndexChanged.connect(self._on_channel)
        self.lbl_chan = QtWidgets.QLabel("")
        self.lbl_chan.setStyleSheet("color:#8a5a00")
        # 每通道的开/关。档位沿用上面那个 spn_bright（它按"当前通道"下发，
        # 而档位本身是按通道生效的 —— 实测），这里只管启停。
        #
        # 为什么要独立的启停：通道是**累加**的，下发某通道的启动帧只会
        # "加上"它，没有任何指令能"只留这一个"。要关掉就得对它单独发
        # byte4=0x00 的停止帧。改造前界面上唯一的停止帧是硬编码的
        # 「关闭通道一」按钮，别的通道关不掉。
        self.chk_ch: dict[int, QtWidgets.QCheckBox] = {}
        self._gear_by_ch: dict[int, int] = {}
        for c in range(CHANNEL_MIN, CHANNEL_MAX + 1):
            cb = QtWidgets.QCheckBox(str(c))
            cb.setToolTip("\n".join((
                f"通道 {c} 的开/关。",
                f"  勾选 → 下发启动帧 byte4=01、byte6={c:02X}，档位用它自己的；",
                f"  取消 → 下发停止帧 byte4=00、byte6={c:02X}。",
                "",
                "⚠️ 勾上**不代表**设备真的开了 —— 右边的回传状态才算。",
                "   设备上电默认就在发 1 号通道，所以 1 号初始是勾上的。")))
            cb.setChecked(c == EXPECTED_CHANNEL)
            cb.toggled.connect(lambda on, ch=c: self._on_chan_toggle(ch, on))
            self.chk_ch[c] = cb
            self._gear_by_ch[c] = BRIGHTNESS_DEFAULT
        self.lbl_chan_live = QtWidgets.QLabel("")
        self.lbl_chan_live.setToolTip(
            "各通道的**回传**状态：● 在发帧　○ 静默。\n"
            "以数据帧的 byte10 为准，不看我们下发过什么。")
        self.lbl_avg = QtWidgets.QLabel()
        self.btn_reset = QtWidgets.QPushButton("清屏")
        self.btn_reset.setToolTip(
            "清掉**所有通道**的曲线、基线与 AUC，自动基线重新开始等稳定。")
        # ⚠️ 必须包一层 lambda。`clicked` 会把 checked(False) 当第一个实参传进去，
        #    _reset 现在有了 `channels` 形参，直接连会收到 channels=False
        #    → `for c in False` 抛 TypeError。自检 t21 盯着这条。
        self.btn_reset.clicked.connect(lambda: self._reset())
        for w in (QtWidgets.QLabel("串口"), self.cmb_port,
                  QtWidgets.QLabel("波特率"), self.cmb_baud,
                  self.btn_refresh, self.btn_conn,
                  QtWidgets.QLabel("  光源"), self.spn_bright,
                  QtWidgets.QLabel("  查看"), self.cmb_chan, self.lbl_chan,
                  QtWidgets.QLabel("  开通道"), *self.chk_ch.values(),
                  self.lbl_chan_live,
                  QtWidgets.QLabel("  平均"), self.spn_avg, self.lbl_avg,
                  self.btn_reset):
            bar.addWidget(w)
        bar.addStretch(1)
        # 原始归档：**默认开启**。连续监护设备出了异常必须能回看，
        # 而「忘了打开」正好发生在最需要它的时候。关闭是显式动作。
        self.chk_rec = QtWidgets.QCheckBox("记录原始数据")
        self.chk_rec.setChecked(True)
        self.chk_rec.setToolTip(
            "把串口来的原始帧原样落盘，供事后重放。\n"
            "目录 srs_cal/原始归档/，每小时一个文件，总量超上限自动删最旧。\n"
            "⚠️ 归档失败不会中断测量，只在这里提示。")
        self.lbl_rec = QtWidgets.QLabel("")
        self.lbl_rec.setStyleSheet("color:#888")
        bar.addWidget(self.chk_rec)
        bar.addWidget(self.lbl_rec)
        self.lbl_link = QtWidgets.QLabel("未连接")
        self.lbl_link.setStyleSheet("color:#888")
        bar.addWidget(self.lbl_link)
        root.addLayout(bar)
        root.addLayout(self._build_bar2())
        self.chk_rec.toggled.connect(self._toggle_record)
        self.btn_refresh.clicked.connect(self._refresh_ports)
        self.btn_conn.clicked.connect(self._toggle_conn)
        if self.simulate:
            for w in (self.cmb_port, self.cmb_baud, self.btn_refresh):
                w.setEnabled(False)

        # 标定状态横幅 —— 常驻，不可关闭
        text, ok = self.cal.banner()
        # L2.5 的**常驻配置项**并入同一条横幅：它们每次测量都成立，
        # 若混进逐次异常行会把真异常淹没（告警疲劳）。这里只呈现一次。
        cfg = [f.text for f in self.quality.config_flags()
               if f.code != "kb_uncalibrated"]      # k/b 已在 banner 里说过
        if cfg:
            text += "　｜ " + "；".join(cfg)
        self.lbl_cal = QtWidgets.QLabel(text)
        self.lbl_cal.setWordWrap(True)
        self.lbl_cal.setStyleSheet(
            "padding:7px;border-radius:4px;"
            + ("background:#e4f7ee;color:#0d6e4b;border:1px solid #1baf7a"
               if ok else
               "background:#fdf3e3;color:#8a5a00;border:1px solid #e0a94f;"
               "font-weight:bold"))
        root.addWidget(self.lbl_cal)

        # 大号读数
        mid = QtWidgets.QHBoxLayout()
        self.lbl_toi = QtWidgets.QLabel("—")
        f = QtGui.QFont("Consolas"); f.setPointSize(56); f.setBold(True)
        self.lbl_toi.setFont(f)
        self.lbl_toi.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_toi.setMinimumWidth(330)
        box = QtWidgets.QGroupBox("TOI (%)")
        bv = QtWidgets.QVBoxLayout(box)
        bv.addWidget(self.lbl_toi)
        self.lbl_sub = QtWidgets.QLabel("等待数据…")
        self.lbl_sub.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_sub.setStyleSheet("color:#666")
        self.lbl_sub.setWordWrap(True)
        bv.addWidget(self.lbl_sub)
        mid.addWidget(box, 1)
        mid.addWidget(self._build_cells(), 2)

        self.tbl = QtWidgets.QTableWidget(len(WL), 4)
        self.tbl.setHorizontalHeaderLabels(["I(3cm)", "I(4cm)", "斜率", "μa"])
        self.tbl.setVerticalHeaderLabels([f"{w:.0f} nm" for w in WL])
        self.tbl.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.tbl.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.Stretch)
        mono = QtGui.QFont("Consolas"); mono.setStyleHint(QtGui.QFont.Monospace)
        self.tbl.setFont(mono)
        mid.addWidget(self.tbl, 1)
        root.addLayout(mid)

        # —— 主页：临床量四格，共用 x 轴（同一条 SRS 链、同一时刻）。
        #    调试页：μa（反演中间量）。2026-10-26 前 μa 与 MBLL 占了主页两行。
        self.tabs = QtWidgets.QTabWidget()
        glw = pg.GraphicsLayoutWidget()
        self.p_toi = glw.addPlot(row=0, col=0, title="TOI")
        self.p_toi.showGrid(x=True, y=True, alpha=0.3)
        self.p_toi.setLabel("left", "TOI (%)")
        self.p_toi.addLegend(offset=(-10, 8), labelTextSize="8pt")
        # TOI 图**叠加所有通道** —— 左右对比是双通道脑氧的临床意义本身，
        # 两条曲线不在同一张图上就比不了。
        #
        # 为什么只有这一张图叠加：4 通道 × ΔHbO/ΔHbR/ΔtHb = 12 条曲线没法读。
        # 其余五张图只画"当前查看的通道"，由工具栏的「查看」下拉切换。
        self.c_toi_ch: dict[int, object] = {}
        for c in range(CHANNEL_MIN, CHANNEL_MAX + 1):
            cur = self.p_toi.plot(
                pen=pg.mkPen(CHAN_COLOR[c],
                             width=2 if c == EXPECTED_CHANNEL else 1.6),
                name=f"TOI 通道 {c}")
            cur.setClipToView(True)
            self.c_toi_ch[c] = cur
        # `c_toi` 保留成"当前查看通道那条" —— 既有代码与自检都按这个名字读。
        self.c_toi = self.c_toi_ch[EXPECTED_CHANNEL]
        # 基线与 AUC 警戒线画成**数据曲线**而不是 InfiniteLine：后者不参与
        # 自动缩放，TOI 远高于警戒线时那条线会落在视野外，等于没画。
        self.c_bl_line = self.p_toi.plot(
            pen=pg.mkPen("#555555", width=1.4, style=QtCore.Qt.DashLine),
            name="基线")
        self.c_auc_line = self.p_toi.plot(
            pen=pg.mkPen("#c0392b", width=1.4, style=QtCore.Qt.DotLine),
            name=f"AUC 警戒线（参考×{1 - AUC_REL_DROP:.1f}）")
        # 第一个窗口攒满前画进度，而不是留一张空图 ——
        # 空图看起来是「坏了」，不是「在攒」。
        self.txt_toi = pg.TextItem(anchor=(0.5, 0.5), color="#8a5a00")
        self.p_toi.addItem(self.txt_toi)

        # THI 绝对指数 = k·tHb，**不依赖基线**，开机就有；下面三格都要等基线。
        #
        # ⚠️ 2026-10-26 这一格加了、删了、又恢复。判据是竞品界面照片：
        #    第一张只看到「THI 2.83 / ΔTHI 1.83」，差恰为 1.00，判为归一化 → 删。
        #    第二张多出「BL 82 / rSO₂ 81」—— 病人是稳的，而 THI 2.83 若是
        #    「当前 tHb ÷ 基线 tHb」就意味着总血红蛋白涨到近 3 倍，不可能。
        #    故 THI 2.83 更可能是**绝对指数**，归一化那条恒等式是另一回事。
        #    两种读法都留在界面上，免得猜错又返工（比值在读数行，指数在这一格）。
        self.p_thi = glw.addPlot(
            row=1, col=0, title="THI　（血红蛋白指数，不依赖基线）")
        self.p_thi.showGrid(x=True, y=True, alpha=0.3)
        self.p_thi.setLabel("left", "tHb (µM)")
        self.c_thi = self.p_thi.plot(pen=pg.mkPen("#8e44ad", width=1.8))

        self.p_rel = glw.addPlot(
            row=2, col=0, title="ΔHbO / ΔHbR / ΔtHb　（SRS 绝对浓度 − 基线）")
        self.p_rel.showGrid(x=True, y=True, alpha=0.3)
        self.p_rel.setLabel("left", "Δc (µM)")
        self.p_rel.addLegend(offset=(-10, 8), labelTextSize="8pt")
        self.c_rel = {
            "d_hbo": self.p_rel.plot(pen=pg.mkPen("#d62728", width=1.8),
                                     name="ΔHbO"),
            "d_hbr": self.p_rel.plot(pen=pg.mkPen("#2a78d6", width=1.8),
                                     name="ΔHbR"),
            "d_thb": self.p_rel.plot(pen=pg.mkPen("#1a7f5a", width=1.4),
                                     name="ΔtHb"),
        }
        self.txt_rel = pg.TextItem(anchor=(0.5, 0.5), color="#8a5a00")
        self.p_rel.addItem(self.txt_rel)

        # ΔTHI 与 ΔTOI 同一格：都以 0 为中心、量级相近（%）。
        # THI 本身（基线处 = 100）在读数行里给。
        self.p_idx = glw.addPlot(row=3, col=0, title="ΔTHI / ΔTOI　（相对基线）")
        self.p_idx.showGrid(x=True, y=True, alpha=0.3)
        self.p_idx.setLabel("left", "%")
        self.p_idx.addLegend(offset=(-10, 8), labelTextSize="8pt")
        # ΔTHI 画成百分比（×100）才与 ΔTOI 的量级可比、共用一根 y 轴；
        # 读数行按 MOC400 的比值口径给（THI 1.39 / ΔTHI +0.39）。
        self.c_rel["d_thi"] = self.p_idx.plot(
            pen=pg.mkPen("#8e44ad", width=1.6), name="ΔTHI (%，=比值×100)")
        self.c_rel["d_toi"] = self.p_idx.plot(
            pen=pg.mkPen("#1a7f5a", width=1.8), name="ΔTOI (%O₂)")
        self.p_idx.addItem(pg.InfiniteLine(
            pos=0.0, angle=0, pen=pg.mkPen("#999999", width=1)))

        self.p_auc = glw.addPlot(row=4, col=0, title="AUC　（去饱和负荷）")
        self.p_auc.showGrid(x=True, y=True, alpha=0.3)
        self.p_auc.setLabel("left", "%·min")
        self.p_auc.setLabel("bottom", "时间 (s)")
        self.c_auc = self.p_auc.plot(pen=pg.mkPen("#c0392b", width=1.8))
        for c in (*self.c_rel.values(), self.c_auc, self.c_thi,
                  self.c_bl_line, self.c_auc_line):
            c.setClipToView(True)
        for p in (self.p_thi, self.p_rel, self.p_idx, self.p_auc):
            p.setXLink(self.p_toi)
        # y 轴只按**可见**时间段自动缩放 —— 否则看「1 分钟」时，
        # 一小时前的一次大波动会把 y 轴撑开，眼前的变化压成一条线。
        for p in (self.p_toi, self.p_thi, self.p_rel, self.p_idx, self.p_auc):
            p.setAutoVisible(y=True)
        self.tabs.addTab(glw, "主页")

        glw_dbg = pg.GraphicsLayoutWidget()
        self.p_mua = glw_dbg.addPlot(row=0, col=0, title="各波长 μa（反演中间量）")
        self.p_mua.showGrid(x=True, y=True, alpha=0.3)
        self.p_mua.setLabel("left", "μa (cm⁻¹)")
        self.p_mua.setLabel("bottom", "时间 (s)")
        self.p_mua.addLegend(offset=(-10, 8), labelTextSize="8pt")
        self.c_mua = {w: self.p_mua.plot(pen=pg.mkPen(COLOR[w], width=1.6),
                                         name=f"{w:.0f}") for w in WL}
        for c in self.c_mua.values():
            c.setClipToView(True)
        self.p_mua.setXLink(self.p_toi)
        self.tabs.addTab(glw_dbg, "调试")

        root.addWidget(self.tabs, 1)

        self._refresh_ports()
        self._sync_avg()
        self.spn_avg.valueChanged.connect(self._sync_avg)

    # 每格：(键, 标题, 提示)。**标题必须自带限定词** ——
    # 「THI」单独出现指不清是指数还是比值，用户为此找不到读数两次。
    CELLS = (
        ("thi_abs", "THI 指数 (µM)", "k·tHb，**不依赖基线**，开机就有。\n"
                                    "k 来自 μs′ 假设，故只能叫指数、不是真浓度。"),
        ("d_hbo", "ΔHbO (µM)", "相对基线的含氧血红蛋白变化。"),
        ("d_hbr", "ΔHbR (µM)", "相对基线的脱氧血红蛋白变化。"),
        ("d_thb", "ΔtHb (µM)", "ΔHbO + ΔHbR。"),
        ("thi", "THI 比值", "当前 tHb ÷ 基线 tHb，**基线 = 1.00**。\n"
                            "这是与 MOC400 的 THI 读数同一族的量。"),
        ("d_thi", "ΔTHI", "THI − 1.00。"),
        ("d_toi", "ΔTOI (%O₂)", "当前 TOI − 基线 TOI。括号里是占基线的百分比。"),
        ("auc", "AUC (%·min)", "∫ max(警戒线 − TOI, 0) dt。\n"
                               "括号里是低于警戒线的累计时长。"),
    )

    def _build_cells(self) -> QtWidgets.QWidget:
        """带标签的读数格子。

        2026-10-26：原先是一行七项的长文本，用户两次找不到 THI。
        改成 MOC400 那样一格一个量、标题常驻 —— 扫一眼就能定位。
        """
        w = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(w)
        g.setSpacing(4)
        g.setContentsMargins(0, 0, 0, 0)
        mono = QtGui.QFont("Consolas"); mono.setPointSize(13); mono.setBold(True)
        self.cells: dict = {}
        for i, (key, title, tip) in enumerate(self.CELLS):
            fr = QtWidgets.QFrame()
            fr.setFrameShape(QtWidgets.QFrame.StyledPanel)
            fr.setToolTip(tip)
            v = QtWidgets.QVBoxLayout(fr)
            v.setSpacing(0); v.setContentsMargins(6, 3, 6, 3)
            cap = QtWidgets.QLabel(title)
            cap.setStyleSheet("color:#888;font-size:8pt")
            val = QtWidgets.QLabel("—")
            val.setFont(mono)
            val.setStyleSheet("color:#222")
            v.addWidget(cap); v.addWidget(val)
            g.addWidget(fr, i // 4, i % 4)
            self.cells[key] = val
        return w

    def _build_bar2(self) -> QtWidgets.QHBoxLayout:
        """第二行工具栏：基线 ｜ AUC ｜ 时间窗 ｜ 事件标记。

        基线与 AUC 各有各的按钮 —— 2026-10-26 前一个「校准基线」同时清 AUC，
        提示语里写着「重设会把 AUC 一并清零」，写出来了但没解决。
        MOC400 的设置页同样是两行：BL 基线、AUC 基线（各自复位）。
        """
        bar = QtWidgets.QHBoxLayout()

        # —— 基线 ——
        self.btn_capture = QtWidgets.QPushButton("抓取当前")
        self.btn_capture.setToolTip("\n".join((
            f"立即取之后 {BASELINE_N} 个点（约 {BASELINE_N / 0.657:.0f} 秒）的均值"
            "作为基线，不等稳定判据。",
            "ΔHbO / ΔHbR / ΔtHb / THI / ΔTOI 全部相对它计算。",
            "",
            "开机后会自动设基线 —— 攒够点就设，**不等信号稳定**。",
            f"稳定判据（{SETTLE_WIN_S:.0f} s 内最强一路光强变化 < {SETTLE_MAX_REL:.0%}"
            f"、TOI 变化 < {SETTLE_MAX_TOI_PTS:.0f} 点）只在定基线那一刻读一次，",
            "不通过就把基线标成「未稳定」，由你决定要不要重抓。",
            "",
            "⚠️ 基线不抗漂移：SRS 为了保住绝对值不能高通，长时间监护后应重抓。",
            "   重抓**不影响 AUC**（AUC 有自己的参考点，用右边的「AUC复位」）。")))
        self.btn_capture.clicked.connect(self._on_capture)
        self.spn_bl_toi = QtWidgets.QDoubleSpinBox()
        self.spn_bl_toi.setRange(1.0, 100.0)
        self.spn_bl_toi.setDecimals(1)
        self.spn_bl_toi.setSuffix(" %")
        self.spn_bl_toi.setToolTip(
            "手填 TOI 基线（如术前已知值）。只改 TOI 基线，\n"
            "ΔHbO/ΔHbR/ΔtHb/THI 的浓度基线仍来自抓取 —— 浓度没法手填。\n"
            "所以要先有一次抓取或自动基线，才能手填。")
        self.btn_bl_apply = QtWidgets.QPushButton("应用")
        self.btn_bl_apply.clicked.connect(self._on_manual_toi)
        self.lbl_bl = QtWidgets.QLabel("基线：—")
        self.lbl_bl.setMinimumWidth(330)

        # —— AUC ——
        self.btn_auc_reset = QtWidgets.QPushButton("AUC复位")
        self.btn_auc_reset.setToolTip("\n".join((
            "AUC 清零，参考点取**当前基线的 TOI**。基线不动。",
            "",
            f"AUC = ∫ max(警戒线 − TOI, 0) dt，警戒线 = 参考 × {1 - AUC_REL_DROP:.1f}"
            f"（相对基线下 {AUC_REL_DROP:.0%}，不可改）。",
            "警戒线必须低于基线：若贴着基线，噪声负半边会被持续收进来 ——",
            "实测静止体模 10 分钟能凭空攒 662~1013 %·s。")))
        self.btn_auc_reset.clicked.connect(self._on_auc_reset)
        self.lbl_auc = QtWidgets.QLabel("警戒线：待基线")

        # —— 时间窗 ——
        self.cmb_span = QtWidgets.QComboBox()
        for name, _ in SPAN_CHOICES:
            self.cmb_span.addItem(name)
        self.cmb_span.setCurrentIndex(DEFAULT_SPAN_IDX)
        self.cmb_span.setToolTip("曲线显示的时间范围。数据本身一直保留（约 8.5 小时）。")

        # —— 事件标记 ——
        self.txt_event = QtWidgets.QLineEdit()
        self.txt_event.setPlaceholderText("事件名（可空）")
        self.txt_event.setMaximumWidth(150)
        self.txt_event.returnPressed.connect(self._on_mark)
        self.btn_mark = QtWidgets.QPushButton("标记")
        self.btn_mark.setToolTip(
            "在当前时刻打一条竖线（如袖带充气 / 放气）。\n"
            "记录原始数据时同时写入归档旁的 <归档名>.events.csv，"
            "时刻与归档里的 t_host 同一个钟，可对齐回放。")
        self.btn_mark.clicked.connect(self._on_mark)

        # —— 两条固定的协议测试帧（2026-10-29，用户逐字节指定）——
        # 放在最右、与临床控件之间隔一个 stretch：「关闭通道一」会停数据流，
        # 监护中误点代价不小（能不能用启动帧恢复还没验过）。
        self.btn_cmd_ch2 = QtWidgets.QPushButton("启动通道2")
        self.btn_cmd_ch2.setToolTip(
            "原样下发：55 AA 1A 01 01 64 02 … 82\n"
            "（启动 | 强度 100 | 二通道）\n\n"
            "⚠️ 2026-10-28 实测：下发通道 2 后回传的 byte10 仍是 01。\n"
            "   若这次仍无变化，通道状态行会直接写出来。\n"
            "⚠️ 发出后清屏、换归档文件 —— 换通道 = 换光学套件，标定不再成立。")
        self.btn_cmd_ch2.clicked.connect(self._on_cmd_start_ch2)
        self.btn_cmd_stop = QtWidgets.QPushButton("关闭通道一")
        self.btn_cmd_stop.setStyleSheet("color:#a33208;font-weight:bold")
        self.btn_cmd_stop.setToolTip(
            "原样下发：55 AA 1A 01 00 00 01 … 1C\n"
            "（停止 | 强度 0 | 一通道）\n\n"
            "⚠️ 停止帧**从未实测过**。设备一直是上电即连续发数据。\n"
            "   生效的标志是帧率掉到 0；恢复只能靠再发启动帧，\n"
            "   万一启动帧也不被接受，就得重新上电。先在体模上试。")
        self.btn_cmd_stop.clicked.connect(self._on_cmd_stop_ch1)

        for w in (QtWidgets.QLabel("基线"), self.btn_capture,
                  QtWidgets.QLabel("TOI"), self.spn_bl_toi, self.btn_bl_apply,
                  self.lbl_bl, QtWidgets.QLabel("｜"),
                  self.btn_auc_reset, self.lbl_auc, QtWidgets.QLabel("｜ 窗口"),
                  self.cmb_span, QtWidgets.QLabel("｜ 事件"),
                  self.txt_event, self.btn_mark):
            bar.addWidget(w)
        bar.addStretch(1)
        bar.addWidget(QtWidgets.QLabel("协议测试"))
        bar.addWidget(self.btn_cmd_ch2)
        bar.addWidget(self.btn_cmd_stop)
        return bar

    def _sync_avg(self) -> None:
        """显示平均窗口对应多少秒 —— 用**实测**帧率，不用假设值。

        早先这里写死按 100 帧/s 换算。真机到底多少帧/s 从没实测过，
        按假设显示等于骗人。没数据时就老实显示「待测」。
        """
        n = self.spn_avg.value()
        cps = self._cycles_per_s()
        if cps > 0:
            self.lbl_avg.setText(f"窗口 ≈ {n / cps:.1f} s，"
                                 f"出点 {cps:.1f}/s")
        else:
            self.lbl_avg.setText("速率待测")

    def _n_active(self) -> int:
        """回传里还在发帧的通道数。**以回传为准**，不看我们下发过几个。"""
        st = getattr(self.worker, "stream", None)
        return getattr(st, "n_active", 1) if st is not None else 1

    def _cycles_per_s(self) -> float:
        """实测的「本通道」周期率 = 帧率 ÷ (12 × 活跃通道数)。

        ⚠️ 必须除以活跃通道数。多通道同采时设备把帧率成倍提高，但每个
           通道自己的周期率不变（2026-10-29 实测：双通道 15.6 帧/s，
           ch1 仍是 0.656 周期/s）。只除 12 会按通道数成倍偏高。
        """
        if self._fps <= 0:
            return 0.0
        return self._fps / frames_per_cycle(self._n_active())

    # -- 串口 --------------------------------------------------------------
    def _refresh_ports(self) -> None:
        """枚举串口，并把 DEFAULT_PORT 预选上（若在）。"""
        self.cmb_port.clear()
        try:
            from serial.tools import list_ports
            for p in list_ports.comports():
                self.cmb_port.addItem(f"{p.device} — {p.description}", p.device)
        except Exception:                                        # noqa: BLE001
            pass
        if self.cmb_port.count() == 0:
            self.cmb_port.addItem("未发现串口", None)
            return
        i = self.cmb_port.findData(DEFAULT_PORT)
        if i >= 0:
            self.cmb_port.setCurrentIndex(i)

    def _autostart(self) -> None:
        """启动时自动连 DEFAULT_PORT 并下发 DEFAULT_GEAR。

        ⚠️ 只在该串口**确实存在**时才动。找不到就什么都不做、只提示 ——
           不去连别的口：自动连到另一台设备上并给它下发最大光强，
           是个比"没自动连"糟糕得多的结果。
        """
        if self.simulate or self.worker is not None:
            return
        port = getattr(self, "auto_port", DEFAULT_PORT)
        if self.cmb_port.findData(port) < 0:
            self.lbl_link.setText(f"未发现 {port}，请手动选口")
            return
        i = self.cmb_port.findData(port)
        self.cmb_port.setCurrentIndex(i)
        self._toggle_conn()
        if self.worker is None:                 # 开口失败，_on_failed 已提示
            return
        self.spn_bright.setValue(getattr(self, "auto_gear", DEFAULT_GEAR))
        self._bright_timer.start()

    def _on_channel(self) -> None:
        """切换**查看**哪个通道。纯视角动作：不下发、不清屏。

        ⚠️ 2026-10-29 改。改造前这里会 `_send_brightness()`，而那条路
           末尾要清屏 —— 于是"看一眼 2 号再看回 1 号"会把两边的曲线**都清掉**，
           每次都得重新攒窗口。多通道同时计算的全部意义就是数据已经在后台
           算着了，切过去应该立刻就有，不该重来。

           设备控制现在归两个地方，各管各的：
             · 开/关某通道  → 工具栏的开关复选框（启动帧 / 停止帧）
             · 改某通道光强 → 档位框（作用在**当前查看**的那个通道上）
           这个下拉只决定"下面五张图画谁"，以及"档位框改的是谁"。
        """
        self._view_ch = self.channel()
        # `c_toi` 的约定是"当前查看通道那条"，切视角时跟着换。
        self.c_toi = self.c_toi_ch[self._view_ch]
        # 档位框跟着切到该通道自己的档位。
        # ⚠️ 必须 blockSignals：否则 setValue 会触发防抖定时器，
        #    把上一个通道的档位当成"改动"下发给新通道 —— 档位按通道生效，
        #    发错就真的改了它的光强。
        self.spn_bright.blockSignals(True)
        self.spn_bright.setValue(self._gear_by_ch.get(self._view_ch,
                                                      BRIGHTNESS_DEFAULT))
        self.spn_bright.blockSignals(False)
        self._bright_timer.stop()        # 丢掉可能排着队的那次下发
        self._refresh_chan_label()
        self._refresh()                  # 立刻换图，不等 10 Hz 的刷新定时器

    def _on_chan_toggle(self, ch: int, on: bool) -> None:
        """某个通道的开/关复选框。

        勾 → 启动帧（byte4=01，档位用它自己的那个）；
        取消 → 停止帧（byte4=00，档位写 0）。
        两者都只作用在 byte6 指定的那一个通道上 —— 实测（162704）：
        停 1 号时 2 号继续流，再发启动帧 1 号恢复。
        """
        if self.worker is None:
            # 发不出去就别让复选框假装生效 —— 它会被当成"设备状态"读。
            cb = self.chk_ch[ch]
            cb.blockSignals(True)
            cb.setChecked(not on)
            cb.blockSignals(False)
            self.lbl_chan.setText(f"✗ 未连接，通道 {ch} 的{'启动' if on else '停止'}"
                                  "帧没发出去")
            self.lbl_chan.setStyleSheet("color:#c33;font-weight:bold")
            return
        g = self._gear_by_ch.get(ch, BRIGHTNESS_DEFAULT) if on else 0x00
        self._send_cmd(build_cmd(on, g, ch), g, ch)

    def _refresh_chan_live(self) -> None:
        """各通道的**回传**状态条。看的是 byte10，不是复选框。"""
        st = getattr(self.worker, "stream", None)
        if st is None:
            self.lbl_chan_live.setText("")
            return
        seen = set(getattr(st, "channels_seen", ()))
        bits = []
        for c in range(CHANNEL_MIN, CHANNEL_MAX + 1):
            n = st.cycles_of(c) if c in seen else 0
            bits.append(f"<span style='color:{'#2a7' if n else '#bbb'}'>"
                        f"{'●' if n else '○'}{c}</span>")
        self.lbl_chan_live.setText("&nbsp;".join(bits))
        self.lbl_chan_live.setTextFormat(QtCore.Qt.RichText)

    def channel(self) -> int:
        return int(self.cmb_chan.currentData())

    def _send_brightness(self) -> None:
        """下发档位与通道（同一帧：byte5 档位、byte6 通道）。

        在线没有「一次采集」的概念，随时可改。但 G(λ) 是在某个档位、某个探头
        下标的 —— 换档位后 TOI 是否还成立取决于五路 LED 是否等比例变化
        （未验证）；换通道则直接换了光学套件，**标定一定不再成立**。
        所以两者都要清屏、都要轮转归档、都要在界面上留痕。
        """
        if self.worker is None:
            return
        g, ch = self.spn_bright.value(), self.channel()
        self._send_cmd(build_brightness(g, ch), g, ch)

    def _send_cmd(self, frame: bytes, g: int, ch: int) -> None:
        """发一条**已经造好**的指令帧，并走完换档位/换通道的全部善后。

        界面驱动的 `_send_brightness` 与两个固定帧按钮共用这一条路，
        否则「按钮发出去的帧」与「界面以为的状态」会分叉。
        """
        prev = getattr(self, "_gear_sent", None)
        prev_ch = getattr(self, "_chan_sent", None)
        self.worker.send(frame)
        self._gear_sent, self._chan_sent = g, ch
        # 记住这个通道的档位 —— 切回来时档位框要显示它自己的，
        # 开/关复选框重新勾上时也要用它自己的，而不是别人的。
        # 停止帧（档位写 0）不算"这个通道的档位"，不记。
        gear_was = self._gear_by_ch.get(ch)
        if frame[4] == CMD_START:
            self._gear_by_ch[ch] = g
        # 清屏还是只断一刀？三种情况要分开 —— 这一条改造前是"一律清屏"，
        # 而那会把别的通道、以及只是停一下再开的通道的历史白白扔掉。
        #
        # ⚠️ 只清**这一个通道**。档位是按通道生效的（实测：对 2 号下发
        #    档位 100，1 号的 670/3cm 纹丝不动，693,722 → 674,276；
        #    对 1 号下发才掉到 342,014）。别的通道的光强没变，历史仍然有效。
        if frame[4] == CMD_START and gear_was is not None and gear_was != g:
            # 光强真的变了：旧曲线不是同一条件下的，留着会被当成同一条读
            self._reset(channels=[ch])
        else:
            # 启停：数据本身没问题，只是中间有一段空档。
            # 留着历史、画一道断口 —— 清掉等于把"停之前是什么样"也扔了，
            # 而那恰恰是操作者停它的原因。
            self._pipe_for(ch).break_line()
        # 归档同理：换档位或换通道就开新文件，让每个归档文件的头里恰好
        # **一个**档位、**一个**通道。否则文件头记的是开始归档那一刻的值，
        # 中途改了就对不上 —— 而原始帧里**不含档位**（档位只在出站的指令帧里），
        # 事后无从还原。通道则不同：byte10 是可信的（2026-10-29 实测会回
        # 0x02），所以通道可以从帧里还原，轮转主要是为了档位。
        #
        # ⚠️ 只在**真的变了**时轮转。`_send_brightness` 会被防抖定时器
        #    重复触发（同一档位也会再发一次），不加这个守卫会产生文件碎片。
        if self.archive is not None and (prev != g or prev_ch != ch):
            self._stop_record()
            self._start_record()
        # 质量层的会话基线也不能跨档位沿用 —— 同样只重置这一个通道的。
        self._pipe_for(ch).quality.reset()
        # 复选框跟着这一帧走 —— 不管这帧是谁造的（档位框、固定按钮、
        # 还是复选框自己）。否则界面上的勾选与真正发出去的字节会分叉。
        cb = self.chk_ch.get(ch)
        if cb is not None:
            cb.blockSignals(True)
            cb.setChecked(frame[4] == CMD_START)
            cb.blockSignals(False)
        self._refresh_chan_label()

    def _send_fixed(self, frame: bytes, g: int, ch: int, name: str) -> None:
        """固定帧按钮：原样发出这 26 个字节，再把界面调到与之一致。

        ⚠️ 不走「改控件 → 触发下发」那条路：那条路会**重新造帧**，
           重造出来的 byte4 恒为启动 —— 关闭帧会当场变回启动帧。
        """
        if self.worker is None:
            self.lbl_chan.setText(f"✗ 未连接，「{name}」没发出去")
            self.lbl_chan.setStyleSheet("color:#c33;font-weight:bold")
            return
        self.spn_bright.blockSignals(True)
        self.spn_bright.setValue(g)
        self.spn_bright.blockSignals(False)
        i = self.cmb_chan.findData(ch)
        if i >= 0:
            self.cmb_chan.blockSignals(True)
            self.cmb_chan.setCurrentIndex(i)
            self.cmb_chan.blockSignals(False)
        # 控件先调好再发 —— _send_cmd 末尾的 _refresh_chan_label 读的是控件。
        self._send_cmd(frame, g, ch)

    def _on_cmd_start_ch2(self) -> None:
        self._send_fixed(CMD_START_CH2, 0x64, 0x02, "启动通道2")

    def _on_cmd_stop_ch1(self) -> None:
        self._send_fixed(CMD_STOP_CH1, 0x00, 0x01, "关闭通道一")

    def _chan_counts(self) -> tuple[int, int]:
        """(当前查看通道收到的帧数, 其他通道收到的帧数)。

        ⚠️ 取的是**各条流水线自己的** `n_frames`，不是窗口级的累计计数。
           窗口级那两个是单通道年代的东西：切换查看通道时它们不归零
           （现在切视角不清屏了），于是会把"刚才看 1 号时收的帧"算到
           "现在看 3 号"头上，把「通道 3 无数据」说成「有 1140 帧」。
        """
        mine = self.pipe.n_frames
        other = sum(p.n_frames for c, p in self.pipes.items()
                    if c != self._view_ch)
        return mine, other

    def _refresh_chan_label(self) -> None:
        """通道状态。先报「收没收到」，再报标定是否适用 —— 前者更要紧。"""
        ch = self.channel()
        mine, other = self._chan_counts()
        if other and not mine:
            seen = "、".join(str(c) for c in sorted(self._chan_seen))
            self.lbl_chan.setText(
                f"✗ 通道 {ch} 无数据：收到 {other} 帧，"
                f"都标着通道 {seen} —— 该通道没有回传")
            self.lbl_chan.setStyleSheet("color:#c33;font-weight:bold")
            return
        # ⚠️ 这里**不再**报「另有 N 帧属其他通道」。多通道同采是正常工况，
        #    `other` 恒大于 0，那条提示会把下面的标定警告永远挡住 ——
        #    而标定警告是真要紧的那条。别的通道开没开，看工具栏的 ●1 ●2。
        #
        # 标定是按某一路探头标的，换通道后 G(λ) 不再成立 —— 必须显形。
        # 2026-10-29 起四个通道共用 1 号的标定（用户决定），所以非 1 号通道
        # 屏上那个 TOI 是拿别人的 G(λ) 算出来的，**绝对值不可信**。
        # 这比「没有数据」更隐蔽：没数据看得见，算错的数看不见。
        self.lbl_chan.setText(
            "" if ch == CHANNEL_MIN
            else f"⚠ 通道 {ch}：标定 {self.cal.doc.get('version', '?')} "
                 f"是按通道 {CHANNEL_MIN} 标的，绝对值不可信（相对变化仍可看）")
        self.lbl_chan.setStyleSheet("color:#8a5a00")

    # ------------------------------------------------------------------
    # 原始归档（架构文档 R9）
    # ------------------------------------------------------------------
    def _start_record(self) -> None:
        """开始归档。失败只提示，不影响测量。"""
        if self.archive is not None or not self.chk_rec.isChecked():
            return
        try:
            kw = {} if self.archive_dir is None else {
                "directory": self.archive_dir}
            self.archive = RawArchive(**kw, meta={
                "source": "online.py",
                "simulate": bool(self.simulate),
                "port": "SIM" if self.simulate else self.cmb_port.currentData(),
                "baud": None if self.simulate else self.cmb_baud.currentText(),
                # ⚠️ getattr：_gear_sent 只在发过光强指令后才存在。
                #    连上就立刻归档时它还没有，直接取会抛 AttributeError，
                #    被下面的 except 吞掉 → 归档静默不启动。
                "gear": getattr(self, "_gear_sent", None),
                # 这里记的是我们**下发**的通道号 —— 名字带 _sent 就是这个意思。
                # 实际有哪些通道在回传，要从帧的 byte10 看（已证实可信），
                # 读归档时用 archive.replay_channels()。两者可能不一致：
                # 下发了不等于设备开了，而且通道是累加的（开 2 号时 1 号还在）。
                "channel_sent": getattr(self, "_chan_sent", None),
                "cal": self.cal.doc.get("version", "none"),
                "wavelengths_active_nm": [float(w) for w in WL],
                "rho_cm": [float(r) for r in RHO],
            })
        except Exception as e:                               # noqa: BLE001
            self.archive = None
            self.lbl_rec.setText(f"⚠ 归档未启动：{e}")
            self.lbl_rec.setStyleSheet("color:#c33")

    def _stop_record(self) -> None:
        if self.archive is not None:
            self.archive.close()
            self.lbl_rec.setText("已停止　" + self.archive.summary())
            self.archive = None

    def _toggle_record(self, on: bool) -> None:
        if on:
            if self.worker is not None:
                self._start_record()
        else:
            self._stop_record()

    def _toggle_conn(self) -> None:
        if self.worker is not None:
            self.worker.stop(); self.worker = None
            self._stop_record()
            self.btn_conn.setText("连接"); self.lbl_link.setText("未连接")
            return
        if self.simulate:
            self.worker = SimWorker()
        else:
            port = self.cmb_port.currentData()
            if not port:
                QtWidgets.QMessageBox.warning(self, "无串口", "没有可用串口。")
                return
            self.worker = SerialWorker(port, int(self.cmb_baud.currentText()))
        self.worker.framesReady.connect(self._on_frames)
        self.worker.statusMsg.connect(self.lbl_link.setText)
        self.worker.failed.connect(self._on_failed)
        self.worker.start()
        self.btn_conn.setText("断开")
        self._start_record()

    def _on_failed(self, msg: str) -> None:
        QtWidgets.QMessageBox.critical(self, "串口错误", msg)
        if self.worker is not None:
            self.worker.stop(); self.worker = None
        self.btn_conn.setText("连接")

    def _tick_rate(self) -> None:
        if self.worker is None:
            return
        fps = self._frames - self._last_frames
        self._last_frames = self._frames
        self._fps = float(fps)
        na = self._n_active()
        self.lbl_link.setText(
            f"{fps} 帧/s（{self._cycles_per_s():.2f} 周期/s"
            + (f"，{na} 通道" if na > 1 else "") + "）")
        self._refresh_chan_live()
        self._sync_avg()
        # 归档状态：有错就标红，这是操作者唯一能看到归档出问题的地方
        if self.archive is not None:
            st = self.archive.stats
            self.lbl_rec.setText("● 记录中　" + self.archive.summary())
            self.lbl_rec.setStyleSheet(
                "color:#c33" if st.n_errors else "color:#2a7")

    def _reset(self, channels=None) -> None:
        """清屏。`channels=None` 清全部（工具栏「清屏」按钮）；
        给了通道号就只清那几条（换档位时只清受影响的那一个）。

        ⚠️ 连 last_res 一起丢掉 —— 否则清屏后曲线空了、大数字和格子却停在旧值，
           进度条也因为「已有 last_res」而不出现，看起来像卡死。
           （搬进 ChannelPipeline 后它们是 None 而不是"属性不存在"，
            读的那一侧跟着改成了 `is None` 判断，语义一样。）
        """
        targets = (list(self.pipes) if channels is None
                   else [int(c) for c in channels])
        # 清屏连基线、AUC 一起清 —— 旧基线对应的是已经被清掉的那段数据。
        # 自动基线从「等待稳定」重新开始。
        for ch in targets:
            self._pipe_for(ch).reset()
        # 下面这些画的是**当前在看的那个通道**。只有它被清了才该抹掉，
        # 否则在看 1 号、改了 2 号的档位，1 号的曲线会被无故清空。
        if self._view_ch in targets:
            self.lbl_toi.setText("—")
            self.lbl_toi.setStyleSheet("color:#888")
            for lab in self.cells.values():
                lab.setText("—")
            self.c_thi.setData([], [])
            for c in (*self.c_rel.values(), self.c_auc,
                      self.c_bl_line, self.c_auc_line):
                c.setData([], [])
        # 事件标记是全局的，只在全清时清；图上的线清掉，
        # 已写进 .events.csv 的不动。
        if channels is None:
            for ln in self._event_lines:
                for p, item in ln:
                    p.removeItem(item)
            self._event_lines.clear()
            self.events.clear()
        # 这些是"见过哪些通道"的链路事实，只在**全清**时归零。
        # 按通道清屏（换档位）时留着 —— 否则提示会说"没见过别的通道"，
        # 而它们明明还在发。
        if channels is None:
            self._n_chan_ok = 0
            self._n_chan_bad = 0
            self._chan_seen.clear()

    # -- 反演 --------------------------------------------------------------
    # 扣暗 → 滑窗 → 稳健聚合 → compute_toi → 派生相对量 这一整条，
    # 2026-10-29 搬进了 `chpipe.ChannelPipeline`（每通道一条）。
    # 本类只剩"分发帧"和"画图"。

    def _on_frames(self, frames: list) -> None:
        """滑动窗口：每收齐一个新扫描周期出一个点，每点平均最近 N 个周期。

        为什么滑动而不是攒满就清空（tumbling）：
          清空式每 N 个周期才出一个点，N=10 时界面一秒才跳一次，看着像死机。
          滑动窗口的**噪声完全一样**（都是 N 个周期的平均），但出点率是
          每周期一个 —— 快 N 倍。代价只是相邻点有重叠、彼此相关，这对
          实时显示无所谓（本来就是移动平均）。

        为什么按组合各自排队、不按「扫描周期」切：丢一帧会让按周期切的
        逻辑错位，把相邻周期的读数混进同一次反演。按组合排队时，丢帧
        只让那一路暂时落后，其余等它跟上，对齐关系不会乱。
        """
        # ⚠️ 先归档再处理：存的必须是**未经任何加工**的原始帧。
        #    归档失败只计数，绝不中断测量（RawArchive.write 内部吞异常）。
        if self.archive is not None:
            self.archive.write(frames)

        need = self.spn_avg.value()
        if need != self._need_ui:
            self._resize(need)
        link = getattr(self.worker, "stream", None)
        want = self.channel()
        for f in frames:
            self._frames += 1
            # ⚠️ **通道以回传为准**：只认数据帧自己标的 byte10，不认我们
            #    下发了什么 —— 下发了不等于设备开了。
            #
            # 2026-10-29 起**每个回传过的通道各建一条流水线**，一起算。
            # 改造前这里是 `if f.channel != want: continue`，只留选中的那个
            # 通道；现在别的通道也照算，`want` 只决定界面在**看**哪一个。
            # 这样切换查看通道时不必重新攒窗口 —— 它一直在后台算着。
            self._chan_seen.add(f.channel)
            # 这两个累计数留给自检与旧代码读；**界面已不用它们**，
            # 因为切换查看通道时它们不归零（切视角不再清屏），
            # 会把"刚才看别的通道时收的帧"算到现在这个头上。
            # 权威数字是各条流水线自己的 `n_frames`，见 `_chan_counts`。
            if f.channel != want:
                self._n_chan_bad += 1
            else:
                self._n_chan_ok += 1
            if f.channel < CHANNEL_MIN or f.channel > CHANNEL_MAX:
                continue              # 协议范围外的通道号：帧本身有问题
            pipe = self._pipe_for(f.channel)
            pipe._link = link
            pipe.feed(f, need)

        for p in self.pipes.values():
            p.check_starved(need)

    def _resize(self, need: int) -> None:
        """平均窗口长度变了 —— 各通道都重建队列，保留已有样本的尾部。"""
        self._need_ui = need
        for p in self.pipes.values():
            p.resize(need)

    # -- 派生相对量 ---------------------------------------------------------
    # `_update_derived` / `_break_derived_line` 已搬进 ChannelPipeline。
    #
    # 基线与 AUC 的操作口径（2026-10-29 定）：**一次操作、各通道各算各存**。
    # 理由：抓基线是操作者在某个时刻的一个动作，这个时刻对所有通道相同；
    # 但每个通道的基线**数值**必然不同（不同探头、不同部位），
    # 所以动作广播、状态各存。

    def _on_capture(self) -> None:
        for p in self.pipes.values():
            p.capture_baseline()
            p.break_derived_line()

    def _on_manual_toi(self) -> None:
        # 先在"当前在看的"那条上试 —— 没有浓度基线时它会拒绝，
        # 此时不该把别的通道也改掉。
        if not self.pipe.bl.set_manual_toi(self.spn_bl_toi.value()):
            QtWidgets.QMessageBox.information(
                self, "手填基线",
                "还没有浓度基线 —— 先等自动基线，或点「抓取当前」。\n"
                "手填只改 TOI 基线；ΔHbO/ΔHbR/ΔtHb/THI 需要抓取得到的浓度基线。")
            return
        self.pipe.break_derived_line()
        # ⚠️ 手填的 TOI 基线**只给当前通道** —— 操作者填的是看着这一路的
        #    读数填的，广播给别的通道等于替它们编一个基线。
        #    这与「抓取当前」不同：抓取是各通道各取自己此刻的值。

    def _on_auc_reset(self) -> None:
        for p in self.pipes.values():
            p.bl.reset_auc()
            p.break_auc_line()      # 复位前后不是同一笔账，曲线断开

    def _on_mark(self) -> None:
        """事件标记：画竖线，并写进归档旁的 .events.csv。"""
        t = self._now()
        label = self.txt_event.text().strip() or f"事件 {len(self.events) + 1}"
        self.events.append((t, label))
        self.txt_event.clear()
        items = []
        for p in (self.p_toi, self.p_thi, self.p_rel, self.p_idx, self.p_auc):
            ln = pg.InfiniteLine(
                pos=t, angle=90, pen=pg.mkPen("#e67e22", width=1.2),
                label=label if p is self.p_toi else None,
                labelOpts={"position": 0.92, "color": "#e67e22",
                           "fill": (255, 255, 255, 200)})
            p.addItem(ln)
            items.append((p, ln))
        self._event_lines.append(items)
        self._write_event(t, label)

    def _write_event(self, t: float, label: str) -> None:
        """追加写一行。失败只提示 —— 标记丢了不能中断测量。

        时刻写两份：`t_host`（与归档里每帧的 t_host 同一个钟，回放时对齐用）
        和墙钟（给人看）。
        """
        path = self.archive.current_path if self.archive is not None else None
        if path is None:
            self.lbl_rec.setText(f"⚠ 事件「{label}」未写入文件（未在记录）")
            return
        try:
            ev = path.with_name(path.stem + ".events.csv")
            new = not ev.exists()
            with ev.open("a", encoding="utf-8-sig", newline="") as fh:
                if new:
                    fh.write("wall_time,t_host,t_rel_s,label\n")
                wall = time.strftime("%Y-%m-%d %H:%M:%S")
                t_host = "" if self.tb.t is None else f"{self.tb.t:.6f}"
                safe = label.replace('"', "“")          # CSV 字段里不留裸引号
                fh.write(f'{wall},{t_host},{t:.3f},"{safe}"\n')
        except Exception as e:                               # noqa: BLE001
            self.lbl_rec.setText(f"⚠ 事件未写入：{e}")

    # -- 刷新 --------------------------------------------------------------
    def _refresh(self) -> None:
        # 基线 / AUC 状态放在早退之前：开头 SRS 还没出点时，
        # 这一行正该说清「在等什么」。
        self._refresh_bars()
        self._refresh_chan_label()
        # 搬进 ChannelPipeline 后，「还没出过值」是 last_res is None
        # （改造前是"属性根本不存在"，由 _reset 里的 __dict__.pop 造出来）。
        # 两者语义相同，但 hasattr 对 property 恒为真，必须改成 None 判断。
        if self.last_res is None:
            self._refresh_wait()
            return
        self.txt_toi.setVisible(False)
        res, I = self.last_res, self.last_I
        if np.isfinite(res.toi):
            self.lbl_toi.setText(f"{res.toi:.1f}")
            self.lbl_toi.setStyleSheet("color:#1a7f5a")
        else:
            self.lbl_toi.setText("—")
            self.lbl_toi.setStyleSheet("color:#c33")
        # 各量填进带标签的格子（见 _build_cells），**不再拼成一长行** ——
        # 原先七项挤一行，用户两次找不到 THI。这里只留状态与告警。
        self._fill_cells()
        sub = f"{self.last_n} 周期平均"
        if not self.cal.kb_set:
            sub += "　｜ k/b 未标定 → 看趋势，不是绝对值"
        if self.bl.baseline is None:
            st = self.bl.state
            sub += f"　｜ 相对量等基线（采集中 {st.n_have}/{st.n_need}）"
        if self._derived_err:
            sub += "　｜ ⚠ 派生量出错：" + self._derived_err
        if self._starved:
            sub += "　｜ ⚠ 长期收不到 " + "、".join(self._starved)
        # ⚠️ 用 summarize 而不是直接拼 res.flags：
        #    它会滤掉常驻配置项与 SEV_INFO，避免告警疲劳把真异常淹没。
        qs = summarize(self.last_qflags)
        if qs:
            sub += "　｜ " + qs
        self.lbl_sub.setText(sub)
        self.lbl_sub.setStyleSheet("color:#c33" if (qs or self._starved)
                                   else "color:#666")

        for j, w in enumerate(WL):
            for c, txt in enumerate((f"{I[j,0]:,.0f}", f"{I[j,1]:,.0f}",
                                     f"{res.slope[j]:.4f}",
                                     f"{res.mua[j]:.5f}"
                                     if np.isfinite(res.mua[j]) else "—")):
                it = QtWidgets.QTableWidgetItem(txt)
                it.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                self.tbl.setItem(j, c, it)

        # TOI 叠图：**每个通道各一条**，各用自己的时间轴（共享原点）。
        # 不能共用当前通道的 ts —— 各通道的点数可能差一两个（扣暗要等
        # 下一个暗时隙，谁先到谁先出点），长度对不上 setData 会报错。
        for c, cur in self.c_toi_ch.items():
            p = self.pipes.get(c)
            if p is None or not p.t_hist:
                cur.setData([], [])
                continue
            n = min(len(p.t_hist), len(p.toi_hist))
            # connect="finite"：通道停一下再开会在历史里留一个 NaN，
            # 直连会画出一条横跨空档的假斜线。
            cur.setData(np.fromiter(p.t_hist, float, len(p.t_hist))[:n],
                        np.fromiter(p.toi_hist, float, len(p.toi_hist))[:n],
                        connect="finite")

        if self.t_hist:
            ts = np.fromiter(self.t_hist, float, len(self.t_hist))
            for w, c in self.c_mua.items():
                c.setData(ts, np.fromiter(self.mua_hist[w], float,
                                          len(self.mua_hist[w])),
                          connect="finite")
            self.c_thi.setData(ts, np.fromiter(self.thi_hist, float,
                                               len(self.thi_hist)),
                               connect="finite")
            # 基线与警戒线：横跨整段历史画平线（当前值）
            x2 = [ts[0], ts[-1]]
            b, thr = self.bl.baseline, self.bl.auc.threshold
            self.c_bl_line.setData(x2 if b else [], [b.toi] * 2 if b else [])
            self.c_auc_line.setData(x2 if thr is not None else [],
                                    [thr] * 2 if thr is not None else [])
            # 「最近 span 秒」。右端钉在最新一点，**不往后留空白** ——
            # 记录比窗口短时（如选 1 小时但才测了 8 分钟），留空白会把曲线
            # 挤到左边 14% 里看不清。不够 span 秒就显示全部。
            span = SPAN_CHOICES[self.cmb_span.currentIndex()][1]
            lo = ts[0] if span is None else max(ts[0], ts[-1] - span)
            self.p_toi.setXRange(lo, max(ts[-1], lo + 1.0), padding=0.01)

        self._refresh_derived()

    def _fill_cells(self) -> None:
        """把各量填进格子。没有基线时相对量那几格显示「—」。"""
        d, auc = self.last_derived, self.bl.auc
        thi_abs = self.last_res.thb_uM
        txt = {"thi_abs": f"{thi_abs:.1f}" if np.isfinite(thi_abs) else "—"}
        if d is not None:
            txt.update({
                "d_hbo": f"{d.d_hbo:+.2f}", "d_hbr": f"{d.d_hbr:+.2f}",
                "d_thb": f"{d.d_thb:+.2f}", "thi": f"{d.thi:.2f}",
                "d_thi": f"{d.d_thi:+.2f}",
                "d_toi": f"{d.d_toi:+.1f}（{d.d_toi_rel:+.1f}%）"})
        if auc.ref_toi is not None:
            txt["auc"] = f"{auc.value_min:.1f}（{_mmss(auc.seconds)}）"
        for k, lab in self.cells.items():
            lab.setText(txt.get(k, "—"))

    def _refresh_wait(self) -> None:
        """第一个窗口还没攒满 —— 在 TOI 图里画进度。

        TOI 要攒满 N 个周期才出第一个点（N=20 时约 30 s）。**不出半成品**：
        实测 n=1 的 TOI 与 n=20 差到 23 点（103952 读 42、真值 74），
        非线性（μa ∝ μeff²）使单周期噪声有偏，不是散开。嫌慢就把「平均」调小。
        """
        n = min((len(d) for d in self.win.values()), default=0)
        need = self._need
        self.p_toi.setXRange(0, max(need, 1), padding=0)
        self.p_toi.setYRange(0, 100, padding=0)
        self.txt_toi.setPos(need / 2.0, 50.0)
        # 一帧都没收到本通道的 —— 这不是「在攒」，是「没有」，要分开说
        mine, other = self._chan_counts()
        if other and not mine:
            seen = "、".join(str(c) for c in sorted(self._chan_seen))
            self.txt_toi.setHtml(
                "<div style='text-align:center;font-size:11pt;color:#c33'>"
                f"<b>通道 {self.channel()} 没有数据</b><br>"
                "<span style='font-size:9pt'>"
                f"收到 {other} 帧，都标着通道 {seen}。<br>"
                "该通道没有回传 —— 要么没发过它的启动帧、要么被停止帧关掉了、"
                "要么该通道无硬件</span></div>")
            self.txt_toi.setVisible(True)
            return
        cps = self._cycles_per_s()
        eta = f"，还差 {(need - n) / cps:.0f} s" if cps > 0 else ""
        self.txt_toi.setHtml(
            "<div style='text-align:center;font-size:11pt'>"
            f"<b>攒第一个平均窗　{n} / {need} 周期{eta}</b><br>"
            "<span style='font-size:9pt;color:#8b8a84'>"
            "不出半成品：单周期的 TOI 可偏 20 点以上（μa ∝ μeff² 使噪声有偏）。"
            "要更快就把上面的「平均」调小</span></div>")
        self.txt_toi.setVisible(True)

    def _refresh_bars(self) -> None:
        """第二行工具栏的状态文字。只读 BaselineManager 的状态，不做判断。"""
        st, b, auc = self.bl.state, self.bl.baseline, self.bl.auc
        if st.phase == self.bl.COLLECTING:
            how = "抓取" if st.collecting_mode == "captured" else "自动"
            txt = f"采集中 {st.n_have}/{st.n_need}（{how}）"
            if st.n_skipped:
                txt += f"　已跳过 {st.n_skipped} 点：{st.last_skip[:16]}"
            color = "#8a5a00"
        else:
            src = {"auto": "自动", "captured": "抓取", "manual": "手填"}.get(
                b.source, b.source)
            txt = f"已设 TOI {b.toi:.1f}（{src}，t={b.t:.0f}s）"
            warn = (["⚠ 设定时信号在变"] if b.unstable else []) + \
                   (["⚠ 低信号"] if b.low_signal else [])
            if warn:
                txt += "　" + " ".join(warn)
            # 当时不稳、现在稳了 → 提示重抓。**不自动改基线**（见 BaselineManager）
            if st.suggest_recapture:
                txt += "　→ 信号已稳定，建议重抓"
            color = "#c33" if warn else "#1a7f5a"
        # 实时的信号稳定性 —— 判据是指示灯，这里就是那盏灯。
        # 与基线上的标记分开：基线记的是「设定那一刻」，这里是「此刻」。
        if st.phase == self.bl.SET or st.n_have:
            if not st.gate_ready:
                txt += f"　｜ 信号：攒满 {SETTLE_WIN_S:.0f}s 才能判"
            elif st.gate_stable:
                txt += "　｜ 信号：稳定"
            else:
                txt += (f"　｜ 信号：在变（{SETTLE_WIN_S:.0f}s 内光强 "
                        f"{st.gate_metric:.1%}、TOI "
                        + ("—" if not np.isfinite(st.gate_metric_toi)
                           else f"{st.gate_metric_toi:.1f}") + " 点）")
        self.lbl_bl.setText(txt)
        self.lbl_bl.setStyleSheet(f"color:{color}")
        # 手填要先有浓度基线
        for w in (self.spn_bl_toi, self.btn_bl_apply):
            w.setEnabled(b is not None)
        if b is not None and b is not getattr(self, "_bl_shown", None) \
                and not self.spn_bl_toi.hasFocus():
            self.spn_bl_toi.setValue(b.toi)        # 基线换了 → 框里预填新值
            self._bl_shown = b
        thr = auc.threshold
        self.lbl_auc.setText("警戒线：待基线" if thr is None else
                             f"警戒线 {thr:.1f}%（参考 {auc.ref_toi:.1f}）")

    def _refresh_derived(self) -> None:
        """刷新相对量与 AUC 曲线。没有基线时在图里说明在等什么。"""
        b = self.bl.baseline
        if b is None:
            st = self.bl.state
            head = f"正在采集基线　{st.n_have}/{st.n_need}"
            tail = "保持探头不动"
            (x0, x1), (y0, y1) = self.p_rel.viewRange()
            self.txt_rel.setPos((x0 + x1) / 2, (y0 + y1) / 2)
            self.txt_rel.setHtml(
                "<div style='text-align:center;font-size:11pt'>"
                f"<b>{head}</b><br>"
                f"<span style='font-size:9pt;color:#8b8a84'>{tail}</span></div>")
            self.txt_rel.setVisible(True)
        else:
            self.txt_rel.setVisible(False)
        # 历史照画（重抓基线时旧段不删，中间用 NaN 断开）
        if self.td_hist:
            ts = np.fromiter(self.td_hist, float, len(self.td_hist))
            for k, c in self.c_rel.items():
                y = np.fromiter(self.d_hist[k], float, len(self.d_hist[k]))
                # ΔTHI 存的是比值（基线 = 1.00，MOC400 口径）。画图乘 100
                # 换成百分比，才和同一格里的 ΔTOI 量级可比、共用一根 y 轴。
                c.setData(ts, y * 100.0 if k == "d_thi" else y,
                          connect="finite")
        if self.ta_hist:
            self.c_auc.setData(
                np.fromiter(self.ta_hist, float, len(self.ta_hist)),
                np.fromiter(self.auc_hist, float, len(self.auc_hist)),
                connect="finite")

    def closeEvent(self, e) -> None:                             # noqa: N802
        if self.worker is not None:
            self.worker.stop()
        self._stop_record()          # 关窗也要把归档文件正常收尾
        e.accept()


# ---------------------------------------------------------------------------
# 把测量态转发到「当前在看的通道」的流水线上。
#
# 为什么用生成的 property 而不是手写 32 个：手写就是 32 处可以各自写错的
# 地方，而且以后往 ChannelPipeline 加状态时必然忘记同步。_FWD 是唯一的名单。
#
# 为什么连 setter 也给：`_on_capture` 之类的老代码会直接赋值
# （如 `self.last_derived = None`）。给了 setter，那些代码一行不用改。
def _install_forwarding(cls) -> None:
    for name in cls._FWD:
        def _mk(n=name):
            def _get(self):
                return getattr(self.pipe, n)

            def _set(self, v):
                setattr(self.pipe, n, v)
            return property(_get, _set, doc=f"转发到 self.pipe.{n}")
        if name in cls.__dict__:                 # 别覆盖手写的同名实现
            raise RuntimeError(f"{cls.__name__} 已自有 {name}，不能再转发")
        setattr(cls, name, _mk())


_install_forwarding(OnlineWindow)


# ---------------------------------------------------------------------------
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="在线 SRS —— 实时 TOI")
    ap.add_argument("--cal", default=DEFAULT_CAL,
                    help=f"标定文件（run_srs_v1.py --emit-cal 产出）。"
                         f"默认 {DEFAULT_CAL}；传 none 则不加载（G=0）")
    ap.add_argument("--simulate", action="store_true", help="无硬件排练")
    ap.add_argument("--archive-dir", default=None,
                    help="原始归档落点，默认 srs_cal/原始归档/")
    ap.add_argument("--no-archive", action="store_true",
                    help="启动时不开归档（默认开启）")
    ap.add_argument("--port", default=DEFAULT_PORT,
                    help=f"启动时自动连的串口，默认 {DEFAULT_PORT}")
    ap.add_argument("--gear", type=int, default=DEFAULT_GEAR,
                    help=f"启动时下发的光源档位，默认 {DEFAULT_GEAR}")
    ap.add_argument("--no-autostart", action="store_true",
                    help="不自动连口、不自动下发档位")
    a = ap.parse_args(argv)

    cal = Calibration()
    if a.cal and a.cal.lower() == "none":
        a.cal = None
    if a.cal and not (CAL_DIR / a.cal).exists() and not Path(a.cal).exists() \
            and a.cal == DEFAULT_CAL:
        # 默认文件不在 —— 不要静默改用别的版本，说清楚再退回 G=0。
        have = sorted(p.name for p in CAL_DIR.glob("*.json"))
        print(f"⚠ 默认标定 {DEFAULT_CAL} 不存在于 {CAL_DIR}")
        print(f"  现有：{have if have else '（空）'}")
        print(f"  用 --cal <文件名> 指定，或改 online.py 的 DEFAULT_CAL")
        a.cal = None
    if a.cal:
        p = Path(a.cal)
        if not p.is_absolute() and not p.exists():
            # 相对路径依次试：当前目录（符合直觉）→ 标定参数/ → srs_cal/。
            # 只给文件名就能用，不必关心在哪个目录下启动。
            for cand in (CAL_DIR / p, _HERE / p):
                if cand.exists():
                    p = cand
                    break
            else:
                p = CAL_DIR / p          # 都不存在时，报错信息指向标定参数/
        cal = Calibration.load(p)
        print(f"已加载标定：{p}")
        for c in cal.doc.get("caveats", []):
            print(f"  ⚠ {c}")
    else:
        print("⚠ 未指定 --cal：G(λ)=0、暗电平=0，读数只能看相对变化。")

    app = QtWidgets.QApplication(sys.argv if argv is None else [])
    win = OnlineWindow(cal, simulate=a.simulate, archive_dir=a.archive_dir)
    win.auto_port, win.auto_gear = a.port, a.gear
    if a.no_archive:
        win.chk_rec.setChecked(False)
    win.show()
    # 自动连口 + 下发档位放在 show() 之后：失败时的提示框要有父窗口，
    # 否则弹不出来（或弹到屏幕外）。延一拍让事件循环先跑起来。
    if not a.no_autostart:
        QtCore.QTimer.singleShot(0, win._autostart)
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
