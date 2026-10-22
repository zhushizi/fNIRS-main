# -*- coding: utf-8 -*-
"""在线双解码器 —— 连上设备，一个界面里同时出 SRS 的 TOI 和 MBLL 的 ΔHb。

    python -m srs_cal.online --cal 标定_v1.json      # 用标定文件
    python -m srs_cal.online                          # 无标定，只看相对变化
    python -m srs_cal.online --simulate               # 无硬件排练

和采集程序的分工
----------------
    acquire.py      体模标定【采数据】——  离线，12 块各点一次
    run_srs_v1.py   体模标定【算 G】  ——  离线，产出 标定参数/*.json
    online.py       这个           ——  在线，实时反演 TOI + MBLL 相对量

界面上三格
----------
    TOI             SRS 绝对量，逐周期出点，看最近 SPAN_S 秒
    各波长 μa       SRS 的反演中间量，与 TOI 共用 x 轴
    ΔHbO / ΔHbR     MBLL 相对量，看**整段**记录（见下）

两条链怎么并存
--------------
    共用的只有**扣暗后的光强**（`_push_net` 是唯一入口，在那里分流）。
    再往下各走各的：SRS 禁止高通（会毁掉绝对值），MBLL 必须高通
    （否则仪器漂移淹没相对变化）。配置、时间轴、缓冲都各自独立。

    MBLL 那一格开头几分钟是空的 —— 高通 0.01 Hz 需要 ≥300 s 记录才起步，
    状态行会一直显示还差多少秒。这是物理约束，不是没在跑。

    MBLL 出任何异常都会被就地摘掉并在状态行标红，**绝不影响 TOI**。
    界面上的 ΔHb 与 `python -m mbll_decoder.run` 跑同一份归档逐点相同
    （自检 t16 验这一条）。

SRS 的在线反演【逐帧独立】，不依赖任何时间基线 —— 这正是它能出绝对值
而 MBLL 只能出相对量的原因。所以这里没有「基线采集」步骤，贴上就出数。

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
                             FRAMES_PER_CYCLE, SerialWorker, SimWorker)
from srs_cal.archive import RawArchive                   # noqa: E402
from srs_cal.robust import aggregate                     # noqa: E402
from srs_cal.darksub import DarkSubtractor               # noqa: E402
from srs_cal.quality import QualityMonitor, summarize    # noqa: E402
from srs_cal.protocol import (BRIGHTNESS_DEFAULT, BRIGHTNESS_MAX,  # noqa: E402
                              BRIGHTNESS_MIN, build_brightness)

# L3B（MBLL）。**导入失败不能影响 SRS** —— 这一格是附加显示，
# 绝对血氧那一路不依赖它。import 本身也包起来，缺包时降级而不是启动不了。
try:
    from mbll_decoder import hardware as mbll_hw            # noqa: E402
    from mbll_decoder.live import LiveMbll                  # noqa: E402
except Exception as _e:                                     # noqa: BLE001
    LiveMbll = mbll_hw = None
    _MBLL_ERR = str(_e)
else:
    _MBLL_ERR = ""


def mbll_band_text() -> str:
    """带通参数取自 MBLL 自己的配置，不在这里写死 —— 那边改了这里不会过期。"""
    if mbll_hw is None:
        return "—"
    return f"{mbll_hw.BP_LOW_HZ} ~ {mbll_hw.BP_HIGH_HZ} Hz"

# DEFAULT_PORT / DEFAULT_GEAR 定义在 acquire.py，这里只 import（见上方 import 行）。
# 两个程序共用一份，避免各改各的、悄悄跑偏。命令行 --port / --gear 可临时覆盖。

WL = hw.WAVELENGTHS_ACTIVE_NM                 # 当前 670 / 770（810、850 已停用）
RHO = hw.RHO_CM
# 与 liveplot.py 同一套配色。键要覆盖**全部**器件波长，不能只列当前
# 在用的几路 —— WAVELENGTHS_ACTIVE_NM 变过（810 停用又恢复），
# 按当时在用的波长写死，下次一改就 KeyError。
COLOR = {0.0: "#2a78d6", 670.0: "#d62728", 770.0: "#ff7f0e",
         810.0: "#2ca02c", 850.0: "#1f77b4", 950.0: "#999999"}
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
SPAN_S = 60


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
        self.last_rejected = 0       # 上一次反演剔除的尖峰样本数
        # L2.5 质量层。只读不改，两条链都读它、都不写它。
        # active_wl 必须给 —— 否则分不清「反演通道坏了」和「监视通道坏了」，
        # 后者（810/850/950）不该否决整次测量。
        self.quality = QualityMonitor(
            n_rho=len(RHO), n_wl=len(WL),
            kb_calibrated=cal.kb_set,
            active_wl=tuple(float(w) for w in WL))
        self.last_qflags: list = []
        self._mbll_err = ""
        # 显式传 WL —— 跟着 WAVELENGTHS_ACTIVE_NM 走，别依赖 geometry() 的默认。
        # ⚠️ **不传 dark_level**：暗电平已经在 _on_frames 里用每个周期的
        #    暗时隙逐帧扣过了。再把标定文件里的静态暗传进来会扣两次。
        #    标定文件里的 dark_level 只对旧固件（无暗槽）有意义。
        self.geom = hw.geometry(gain_correction=cal.gain, k=cal.k, b=cal.b,
                                wavelengths_nm=WL)
        self._keys = [(w, r) for w in WL for r in RHO]
        self._need = DEFAULT_AVG
        self.win = {k: deque(maxlen=DEFAULT_AVG) for k in self._keys}
        # 逐周期暗时隙。cycle_dark 是本周期的暗值；dark_win 留一段历史，
        # 用来显示暗电平自己漂多少 —— 它是链路健康度最直接的指标。
        self.cycle_dark: dict[float, float] = {}
        # 扣暗器：按时刻在前后两个暗之间插值，而不是减同周期那一个。
        # 噪声主成分是 0.0868 Hz 的电路/环境干扰（暗时隙里也有），
        # 而暗与信号在周期内差最多 1.5 s = 47° 相位，逐周期相减消不掉。
        # 代价是延迟 1 个扫描周期（约 1.5 s），相对 N×1.5 s 的窗口可忽略。
        # 详见 darksub.py 与 docs/双解码器架构.md §二·十。
        self._darksub = DarkSubtractor()
        # L3B：MBLL 相对量。吃的是**同一个扣暗器的输出**（架构里「两条链
        # 唯一的共同输入」），再往下各走各的预处理 —— SRS 禁止高通，
        # MBLL 必须高通。它只读不写 SRS 的任何状态，出不出数都不影响 TOI。
        self._mbll = LiveMbll() if LiveMbll is not None else None
        self.dark_win = {r: deque(maxlen=600) for r in RHO}
        self._no_dark = 0
        self._fresh: set = set()
        self._since = 0
        self._starved: list[str] = []
        self._frames = 0
        self._last_frames = 0
        self._fps = 0.0
        self._t0 = time.perf_counter()
        self.t_hist: deque = deque(maxlen=MAXPTS)
        self.toi_hist: deque = deque(maxlen=MAXPTS)
        self.mua_hist: dict = {w: deque(maxlen=MAXPTS) for w in WL}
        self.last_flags: list[str] = []
        self._build()
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(int(1000 / REFRESH_HZ))
        self._rate = QtCore.QTimer(self)
        self._rate.timeout.connect(self._tick_rate)
        self._rate.start(1000)

    # -- UI ----------------------------------------------------------------
    def _build(self) -> None:
        self.setWindowTitle("在线双解码器 —— SRS 绝对 TOI ＋ MBLL 相对 ΔHb"
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
        self.lbl_avg = QtWidgets.QLabel()
        self.btn_reset = QtWidgets.QPushButton("清屏")
        self.btn_reset.clicked.connect(self._reset)
        for w in (QtWidgets.QLabel("串口"), self.cmb_port,
                  QtWidgets.QLabel("波特率"), self.cmb_baud,
                  self.btn_refresh, self.btn_conn,
                  QtWidgets.QLabel("  光源"), self.spn_bright,
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

        glw = pg.GraphicsLayoutWidget()
        self.p_toi = glw.addPlot(row=0, col=0, title="TOI")
        self.p_toi.showGrid(x=True, y=True, alpha=0.3)
        self.p_toi.setLabel("left", "TOI (%)")
        self.c_toi = self.p_toi.plot(pen=pg.mkPen("#1a7f5a", width=2))
        self.c_toi.setClipToView(True)
        self.p_mua = glw.addPlot(row=1, col=0, title="各波长 μa（反演中间量）")
        self.p_mua.showGrid(x=True, y=True, alpha=0.3)
        self.p_mua.setLabel("left", "μa (cm⁻¹)")
        self.p_mua.setLabel("bottom", "时间 (s)")
        self.p_mua.addLegend(offset=(-10, 8), labelTextSize="8pt")
        self.c_mua = {w: self.p_mua.plot(pen=pg.mkPen(COLOR[w], width=1.6),
                                         name=f"{w:.0f}") for w in WL}
        for c in self.c_mua.values():
            c.setClipToView(True)
        self.p_mua.setXLink(self.p_toi)

        # —— L3B：MBLL 相对量。**刻意不 setXLink**：
        #    上面两格看的是最近 SPAN_S 秒的实时窗，这一格看的是整段记录
        #    （高通 0.01 Hz 本来就要 ≥300 s 才起步）。强行共用 x 轴，
        #    要么上面被拉得看不清，要么下面只剩一个点。
        self.p_hb = glw.addPlot(
            row=2, col=0, title="ΔHbO / ΔHbR　（MBLL —— 相对变化，整段记录）")
        self.p_hb.showGrid(x=True, y=True, alpha=0.3)
        self.p_hb.setLabel("left", "Δc (µM)")
        self.p_hb.setLabel("bottom", "时间 (s)")
        self.p_hb.addLegend(offset=(-10, 8), labelTextSize="8pt")
        self.c_hbo = self.p_hb.plot(pen=pg.mkPen("#d62728", width=1.6),
                                    name="ΔHbO")
        self.c_hbr = self.p_hb.plot(pen=pg.mkPen("#2a78d6", width=1.6),
                                    name="ΔHbR")
        # 开头几分钟这一格必然是空的（高通 0.01 Hz 要 ≥300 s 才起步）。
        # 把倒计时写在**图里**而不是只写在窗口最底下的状态行 ——
        # 一张空图配一行灰字，看起来就是「坏了」，而不是「在攒」。
        self.txt_hb = pg.TextItem(anchor=(0.5, 0.5), color="#8a5a00")
        self.p_hb.addItem(self.txt_hb)
        self._hb_started = False
        root.addWidget(glw, 1)

        self.lbl_mbll = QtWidgets.QLabel()
        self.lbl_mbll.setWordWrap(True)
        self.lbl_mbll.setStyleSheet("color:#666")
        self.lbl_mbll.setToolTip("\n".join((
            "MBLL 给的是**相对变化**，不是绝对值 —— 绝对血氧由上面的 SRS 给。",
            "两条链相互独立：各自的预处理、各自的配置、各自的时间轴，",
            "只共用同一份扣暗后的光强（架构里「两条链唯一的共同输入」）。",
            "",
            f"带通 {mbll_band_text()}　距离 3.0cm　波长 670/770",
            "⚠️ 高通 0.01 Hz 需要 ≥300 s 记录才起步，所以开头几分钟这一格是空的。",
            "⚠️ 两波长时 ΔHbO 与 ΔHbR 被约束在一条直线上，强负相关是数学必然，",
            "   不是生理证据。",
            "",
            "界面上的数与 `python -m mbll_decoder.run` 跑同一份归档**逐点相同**"
            "（自检 t16 验这一条）。")))
        root.addWidget(self.lbl_mbll)

        self._refresh_ports()
        self._sync_avg()
        self.spn_avg.valueChanged.connect(self._sync_avg)

    def _sync_avg(self) -> None:
        """显示平均窗口对应多少秒 —— 用**实测**帧率，不用假设值。

        早先这里写死按 100 帧/s 换算。真机到底多少帧/s 从没实测过，
        按假设显示等于骗人。没数据时就老实显示「待测」。
        """
        n = self.spn_avg.value()
        if self._fps > 0:
            cps = self._fps / FRAMES_PER_CYCLE   # 一周期 = (1暗+5波长)×2距离
            self.lbl_avg.setText(f"窗口 ≈ {n / cps:.1f} s，"
                                 f"出点 {cps:.1f}/s")
        else:
            self.lbl_avg.setText("速率待测")

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

    def _send_brightness(self) -> None:
        """下发档位。在线没有「一次采集」的概念，随时可改。

        但 G(λ) 是在某个档位下标的 —— 换档位后 TOI 是否还成立取决于
        五路 LED 是否等比例变化（未验证）。所以要在界面上留痕。
        """
        if self.worker is None:
            return
        g = self.spn_bright.value()
        prev = getattr(self, "_gear_sent", None)
        self.worker.send(build_brightness(g))
        self._gear_sent = g
        self._reset()               # 换了光源，之前的曲线不是同一条件，清掉
        # 归档同理：换档位就开新文件，让每个归档文件的头里恰好**一个**档位。
        # 否则文件头记的是开始归档那一刻的档位，中途改了就对不上 ——
        # 而原始帧里**不含档位**（档位只在出站的指令帧里），事后无从还原。
        #
        # ⚠️ 只在档位**真的变了**时轮转。`_send_brightness` 会被防抖定时器
        #    重复触发（同一档位也会再发一次），不加这个守卫会产生文件碎片。
        if self.archive is not None and prev != g:
            self._stop_record()
            self._start_record()
        # 质量层的会话基线也不能跨档位沿用 —— 换了档位比值会整体平移。
        self.quality.reset()

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
        self.lbl_link.setText(f"{fps} 帧/s（{fps/FRAMES_PER_CYCLE:.1f} 周期/s）")
        self._sync_avg()
        # 归档状态：有错就标红，这是操作者唯一能看到归档出问题的地方
        if self.archive is not None:
            st = self.archive.stats
            self.lbl_rec.setText("● 记录中　" + self.archive.summary())
            self.lbl_rec.setStyleSheet(
                "color:#c33" if st.n_errors else "color:#2a7")

    def _reset(self) -> None:
        self.t_hist.clear(); self.toi_hist.clear()
        for d in self.mua_hist.values():
            d.clear()
        for d in self.win.values():
            d.clear()
        for d in self.dark_win.values():
            d.clear()
        self.cycle_dark.clear()
        self._darksub.reset()
        if self._mbll is not None:
            self._mbll.reset()      # 清屏要连 MBLL 的缓冲一起清，
            self.c_hbo.setData([], [])   # 否则新旧两段会被当成一条连续记录
            self.c_hbr.setData([], [])
            self._hb_started = False     # 倒计时重新开始
        self._no_dark = 0
        self._fresh.clear()
        self._since = 0
        self._starved = []

    # -- 反演 --------------------------------------------------------------

    def _push_net(self, key, net: float, t: float, need: int, nk: int) -> None:
        """压入一个扣暗后的样本；收齐一轮就反演。

        从 `_on_frames` 的循环里抽出来 —— 插值扣暗会在**一个暗时隙到达时
        成批**吐出上一周期的信号帧，原先内联在循环里的判定已不适用。

        这里是「一个扣暗样本进入系统」的唯一入口，所以也在这里分流给
        MBLL —— 两条链拿到的是**同一个值**，差异只可能来自算法。
        """
        # ⚠️ MBLL 永远不能打断 SRS 出数。出过一次异常就彻底摘掉这一格，
        #    不反复抛；界面上会显示「已停用」，不会静默。
        if self._mbll is not None:
            try:
                self._mbll.push(key[0], key[1], t, net)
            except Exception as e:                           # noqa: BLE001
                self._mbll = None
                self._mbll_err = str(e)
        self.win[key].append(float(net))
        self._fresh.add(key)
        self._since += 1
        # 判定放在每压入一个样本之后：一批 feed 里可能含多个扫描周期，
        # 放在批末会让出点率被串口的读节奏绑架，而不是由设备扫描速率决定。
        if (len(self._fresh) == nk
                and all(len(self.win[k]) >= need for k in self._keys)):
                # ⚠️ 稳健聚合而非算术均值：实测每通道 2~6% 的周期是尖峰、
                #    幅度约 25 倍。窗口 N=10 时约 10% 的窗口被污染，
                #    一个尖峰会把该点的均值抬高约 2.5 倍。选型见 robust.py。
                ags = {(w, r): aggregate(self.win[(w, r)])
                       for w in WL for r in RHO}
                I = np.array([[ags[(w, r)].value for r in RHO] for w in WL])
                self.last_rejected = sum(a.n_rejected for a in ags.values())
                self._last_used = sum(a.n_used for a in ags.values())
                # 质量层要的三份输入。扣暗值即 ags；暗电平取本周期的。
                self._q_net = {(float(w), float(r)): ags[(w, r)].value
                               for w in WL for r in RHO}
                self._q_dark = {float(r): self.cycle_dark.get(float(r))
                                for r in RHO}
                self._fresh.clear()
                self._since = 0
                self._starved = []
                self._invert(I, need)

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
        if need != self._need:
            self._resize(need)
        nk = len(self._keys)
        for f in frames:
            self._frames += 1
            # 暗时隙排在每个周期最前：记下本周期的暗，后面的信号帧当场扣。
            # 这比标定文件里的静态暗电平强得多 —— 暗电平的漂移时间常数
            # 只有几个周期，静态值几秒钟就过期了。
            if f.is_dark:
                self.cycle_dark[f.distance_cm] = float(f.adc)
                self.dark_win[f.distance_cm].append(float(f.adc))
                # 暗时隙到达 → 上一周期攒着的信号帧现在有了右端点，可解析
                for wl, rho, t, net in self._darksub.feed_dark(
                        f.distance_cm, f.t_host, float(f.adc)):
                    k = (wl, rho)
                    if k in self._keys:
                        self._push_net(k, net, t, need, nk)
                continue
            key = (f.wavelength_nm, f.distance_cm)
            if key not in self._keys:
                continue                       # 810/950 照收不反演
            # interp 模式下这里只是攒着，返回空；prev 模式下当场返回
            for wl, rho, t, net in self._darksub.feed_signal(
                    f.wavelength_nm, f.distance_cm, f.t_host, float(f.adc)):
                self._push_net((wl, rho), net, t, need, nk)
            self._no_dark = self._darksub.n_dropped_nodark
            continue
            self._push_net(key, net, need, nk)

        if self._since > max(need * 20, 400):
            # 某一路长期不来（LED 没亮、协议变了）。不能默默停更 ——
            # 界面必须说清是哪一路，否则看起来就只是「卡住了」。
            self._starved = [f"{w:.0f}nm/{r:.0f}cm" for w, r in self._keys
                             if (w, r) not in self._fresh
                             or len(self.win[(w, r)]) < need]
            self._since = 0

    def _resize(self, need: int) -> None:
        """平均窗口长度变了 —— 重建队列，保留已有样本的尾部。"""
        self.win = {k: deque(list(self.win.get(k, ()))[-need:], maxlen=need)
                    for k in self._keys}
        self._need = need

    def _invert(self, I: np.ndarray, n_cycles: int) -> None:
        from fnirs_pipeline.srs import compute_toi

        res = compute_toi(I, self.geom)
        t = time.perf_counter() - self._t0
        self.last_I = I
        self.last_res = res
        self.last_n = n_cycles
        self.last_flags = res.flags
        # ⚠️ 物理判据由 compute_toi 产出，这里只转达；质量层不重复计算。
        self.last_qflags = self.quality.update(
            net_by_channel=getattr(self, "_q_net", None),
            n_used=getattr(self, "_last_used", 0),
            n_rejected=self.last_rejected,
            dark_level=getattr(self, "_q_dark", None),
            link=getattr(self.worker, "stream", None),
            physics_flags=res.flags)   # 现在只含真异常
        if np.isfinite(res.toi):
            self.t_hist.append(t)
            self.toi_hist.append(res.toi)
            for j, w in enumerate(WL):
                self.mua_hist[w].append(res.mua[j])

    # -- 刷新 --------------------------------------------------------------
    def _refresh(self) -> None:
        # MBLL 放在 SRS 的早退之前：开头几分钟 SRS 还没出第一个点，
        # 而这一格正该显示「还差多少秒」—— 空着会让人以为程序没在跑。
        self._refresh_mbll()
        if not hasattr(self, "last_res"):
            return
        res, I = self.last_res, self.last_I
        if np.isfinite(res.toi):
            self.lbl_toi.setText(f"{res.toi:.1f}")
            self.lbl_toi.setStyleSheet("color:#1a7f5a")
        else:
            self.lbl_toi.setText("—")
            self.lbl_toi.setStyleSheet("color:#c33")
        sub = f"{self.last_n} 周期平均"
        if not self.cal.kb_set:
            sub += "　｜ k/b 未标定 → 看趋势，不是绝对值"
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

        if self.t_hist:
            ts = np.fromiter(self.t_hist, float, len(self.t_hist))
            self.c_toi.setData(ts, np.fromiter(self.toi_hist, float,
                                               len(self.toi_hist)))
            for w, c in self.c_mua.items():
                c.setData(ts, np.fromiter(self.mua_hist[w], float,
                                          len(self.mua_hist[w])))
            self.p_toi.setXRange(max(0.0, ts[-1] - SPAN_S),
                                 max(SPAN_S, ts[-1]), padding=0)

    def _refresh_mbll(self) -> None:
        """刷新 MBLL 那一格。**整个函数包在 try 里** —— 这一格坏掉
        不能把 TOI 的刷新一起拖死。出过一次异常就摘掉，并在界面上说明。
        """
        if self._mbll is None:
            self.lbl_mbll.setText(
                "MBLL　已停用：" + (self._mbll_err or _MBLL_ERR or "未知原因"))
            self.lbl_mbll.setStyleSheet("color:#c33")
            return
        try:
            need = self._mbll.need_s()
            if need > 0:
                # 还没到起步线：画进度，而不是留一张空图。
                # x 轴固定成 0~MIN_RECORD_S，进度条就是「走到哪了」本身。
                total = self._mbll.span_s + need
                self.p_hb.setXRange(0, total, padding=0)
                self.p_hb.setYRange(-1, 1, padding=0)
                self.txt_hb.setPos(total / 2.0, 0.0)
                self.txt_hb.setHtml(
                    "<div style='text-align:center;font-size:11pt'>"
                    f"<b>攒数据中　{self._mbll.span_s:.0f} / {total:.0f} s</b>"
                    f"<br>还差 <b>{need:.0f}</b> 秒出第一条曲线<br>"
                    "<span style='font-size:9pt;color:#8b8a84'>"
                    "高通 0.01 Hz 的瞬态要 3 倍时间常数才衰减干净，"
                    "不够就强滤出来的是滤波器瞬态，不是生理信号</span></div>")
                self.txt_hb.setVisible(True)
            else:
                r = self._mbll.poll(time.perf_counter())
                if r is not None and r.ok:
                    if not self._hb_started:
                        # 第一次出曲线：把占位文字撤掉，交还自动缩放
                        self.txt_hb.setVisible(False)
                        self.p_hb.enableAutoRange()
                        self._hb_started = True
                    # 换算到与 TOI 同一条时间轴（都是「连接以来的秒数」），
                    # 两格的时间才对得上 —— 虽然 x 轴范围各看各的。
                    off = self._mbll.t_offset - self._t0
                    self.c_hbo.setData(r.t + off, r.d_hbo)
                    self.c_hbr.setData(r.t + off, r.d_hbr)
            self.lbl_mbll.setText("MBLL（相对量）　" + self._mbll.status())
            self.lbl_mbll.setStyleSheet("color:#666")
        except Exception as e:                               # noqa: BLE001
            self._mbll = None
            self._mbll_err = str(e)

    def closeEvent(self, e) -> None:                             # noqa: N802
        if self.worker is not None:
            self.worker.stop()
        self._stop_record()          # 关窗也要把归档文件正常收尾
        e.accept()


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
