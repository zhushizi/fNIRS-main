# -*- coding: utf-8 -*-
"""在线 SRS —— 连上设备，实时出 TOI。

    python -m srs_cal.online --cal 标定_v1.json      # 用标定文件
    python -m srs_cal.online                          # 无标定，只看相对变化
    python -m srs_cal.online --simulate               # 无硬件排练

和采集程序的分工
----------------
    acquire.py      体模标定【采数据】——  离线，12 块各点一次
    run_srs_v1.py   体模标定【算 G】  ——  离线，产出 标定_v1.json
    online.py       这个           ——  在线，实时反演 TOI

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
sys.path.insert(0, str(_HERE.parent))

from srs_cal import hardware as hw                       # noqa: E402
from srs_cal.acquire import (BAUDRATES, BRIGHTNESS_SETTLE_MS,    # noqa: E402
                             FRAMES_PER_CYCLE, SerialWorker, SimWorker)
from srs_cal.protocol import (BRIGHTNESS_DEFAULT, BRIGHTNESS_MAX,  # noqa: E402
                              BRIGHTNESS_MIN, build_brightness)

WL = hw.WAVELENGTHS_ACTIVE_NM                 # 670 / 770 / 850
RHO = hw.RHO_CM
# 与 liveplot.py 同一套配色。键要覆盖**全部**器件波长，不能只列当前
# 在用的几路 —— WAVELENGTHS_ACTIVE_NM 变过（810 停用又恢复），
# 按当时在用的波长写死，下次一改就 KeyError。
COLOR = {0.0: "#2a78d6", 670.0: "#d62728", 770.0: "#ff7f0e",
         810.0: "#2ca02c", 850.0: "#1f77b4", 950.0: "#999999"}
MAXPTS = 20000
REFRESH_HZ = 10
DEFAULT_AVG = 10          # 默认合并多少个扫描周期出一个 TOI（≈1 秒）
SPAN_S = 60


class Calibration:
    """标定文件。没有文件时退化为「未标定」，但绝不假装已标定。"""

    def __init__(self, doc: dict | None = None) -> None:
        self.doc = doc or {}
        self.gain = (np.asarray(doc["gain_correction"], float)
                     if doc and "gain_correction" in doc else None)
        # 标定文件里的暗只是留档（键名已改为 dark_level_archive_only）。
        # 在线一律用设备每周期的暗时隙 —— 静态暗几秒就过期。
        self.dark = None
        self.k = float(doc.get("k", 1.0)) if doc else 1.0
        self.b = float(doc.get("b", 0.0)) if doc else 0.0

    @property
    def calibrated(self) -> bool:
        return self.gain is not None

    @property
    def kb_set(self) -> bool:
        return self.k != 1.0 or self.b != 0.0

    def banner(self) -> tuple[str, bool]:
        """(提示文字, 是否可放心用)。"""
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
    def __init__(self, cal: Calibration, simulate: bool = False) -> None:
        super().__init__()
        self.cal = cal
        self.simulate = simulate
        self.worker: QtCore.QThread | None = None
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
        self.setWindowTitle("在线 SRS —— TOI 实时反演"
                            + ("（模拟器）" if self.simulate else ""))
        self.resize(1120, 780)
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
        self.spn_avg.setToolTip(
            "合并多少个扫描周期出一个 TOI。\n"
            "单周期的散粒噪声很大（1% 亮度下 670/4cm 单帧 RSD 约 17%），\n"
            "必须平均。10 个周期约 1 秒。")
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
        self.lbl_link = QtWidgets.QLabel("未连接")
        self.lbl_link.setStyleSheet("color:#888")
        bar.addWidget(self.lbl_link)
        root.addLayout(bar)
        self.btn_refresh.clicked.connect(self._refresh_ports)
        self.btn_conn.clicked.connect(self._toggle_conn)
        if self.simulate:
            for w in (self.cmb_port, self.cmb_baud, self.btn_refresh):
                w.setEnabled(False)

        # 标定状态横幅 —— 常驻，不可关闭
        text, ok = self.cal.banner()
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
        root.addWidget(glw, 1)

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
        self.cmb_port.clear()
        try:
            from serial.tools import list_ports
            for p in list_ports.comports():
                self.cmb_port.addItem(f"{p.device} — {p.description}", p.device)
        except Exception:                                        # noqa: BLE001
            pass
        if self.cmb_port.count() == 0:
            self.cmb_port.addItem("未发现串口", None)

    def _send_brightness(self) -> None:
        """下发档位。在线没有「一次采集」的概念，随时可改。

        但 G(λ) 是在某个档位下标的 —— 换档位后 TOI 是否还成立取决于
        五路 LED 是否等比例变化（未验证）。所以要在界面上留痕。
        """
        if self.worker is None:
            return
        g = self.spn_bright.value()
        self.worker.send(build_brightness(g))
        self._gear_sent = g
        self._reset()               # 换了光源，之前的曲线不是同一条件，清掉

    def _toggle_conn(self) -> None:
        if self.worker is not None:
            self.worker.stop(); self.worker = None
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

    def _reset(self) -> None:
        self.t_hist.clear(); self.toi_hist.clear()
        for d in self.mua_hist.values():
            d.clear()
        for d in self.win.values():
            d.clear()
        for d in self.dark_win.values():
            d.clear()
        self.cycle_dark.clear()
        self._no_dark = 0
        self._fresh.clear()
        self._since = 0
        self._starved = []

    # -- 反演 --------------------------------------------------------------
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
                continue
            key = (f.wavelength_nm, f.distance_cm)
            if key not in self._keys:
                continue                       # 810/950 照收不反演
            d = self.cycle_dark.get(f.distance_cm)
            if d is None:
                self._no_dark += 1             # 从周期中途接上的，等下一周期
                continue
            self.win[key].append(float(f.adc) - d)
            self._fresh.add(key)
            self._since += 1
            # 判定放在**循环内**：一批里可能含多个扫描周期（串口按块读，
            # 设备快时一批能有十几个周期）。放在循环外就变成「一批一个点」，
            # 出点率被串口的读节奏绑架，而不是由设备的扫描速率决定。
            if (len(self._fresh) == nk
                    and all(len(self.win[k]) >= need for k in self._keys)):
                I = np.array([[float(np.mean(self.win[(w, r)])) for r in RHO]
                              for w in WL])
                self._fresh.clear()
                self._since = 0
                self._starved = []
                self._invert(I, need)

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
        if np.isfinite(res.toi):
            self.t_hist.append(t)
            self.toi_hist.append(res.toi)
            for j, w in enumerate(WL):
                self.mua_hist[w].append(res.mua[j])

    # -- 刷新 --------------------------------------------------------------
    def _refresh(self) -> None:
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
        if res.flags:
            sub += "　｜ ⚠ " + "；".join(res.flags)
        self.lbl_sub.setText(sub)
        self.lbl_sub.setStyleSheet("color:#c33" if (res.flags or self._starved)
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

    def closeEvent(self, e) -> None:                             # noqa: N802
        if self.worker is not None:
            self.worker.stop()
        e.accept()


# ---------------------------------------------------------------------------
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="在线 SRS —— 实时 TOI")
    ap.add_argument("--cal", help="标定文件（run_srs_v1.py --emit-cal 产出）")
    ap.add_argument("--simulate", action="store_true", help="无硬件排练")
    a = ap.parse_args(argv)

    cal = Calibration()
    if a.cal:
        p = Path(a.cal)
        if not p.is_absolute():
            p = _HERE / p
        cal = Calibration.load(p)
        print(f"已加载标定：{p}")
        for c in cal.doc.get("caveats", []):
            print(f"  ⚠ {c}")
    else:
        print("⚠ 未指定 --cal：G(λ)=0、暗电平=0，读数只能看相对变化。")

    app = QtWidgets.QApplication(sys.argv if argv is None else [])
    win = OnlineWindow(cal, simulate=a.simulate)
    win.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
