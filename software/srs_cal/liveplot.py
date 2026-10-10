# -*- coding: utf-8 -*-
"""实时曲线 —— 两个探测器各 5 条波长，外加 SRS 真正用到的空间斜率。

为什么是三格而不是两格：
  标定看的从来不是单个探测器的绝对读数，而是 log10(I3/I4)。两个探测器
  各画各的只能看出「亮不亮、稳不稳」；串扰、泄漏这类毛病要在比值上才现形。
  2026-10-09 实测的 810 泄漏就是现成例子 —— 810 在 3cm 一切正常，排在
  770 和 850 中间；问题全在 4cm 偏高，只有第三格能一眼看出它的斜率掉到
  其余三路下面 0.33~0.87。

为什么要能扣暗电平：
  实测暗电平 3cm 是 **负的**（−43,981），4cm 是正的（+9,019），而且五个
  波长完全一致 —— 那是电子偏置不是光。不扣掉，log10(I3/I4) 会整体偏，
  斜率算出来没有意义。勾上后会从主窗口最近一次 DARK 采集自动取值。

窗口自身不碰串口，只按序号从主窗口的环形缓冲 raw_buf 里增量取帧，
和 rawview 一样 —— 开着它不影响采集。
"""

from __future__ import annotations

from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets

from srs_cal import hardware as hw
from srs_cal.protocol import DISTANCE_BY_CODE, WAVELENGTH_BY_CODE

WLS = [WAVELENGTH_BY_CODE[c] for c in sorted(WAVELENGTH_BY_CODE)]   # 670..950
RHOS = [DISTANCE_BY_CODE[c] for c in sorted(DISTANCE_BY_CODE)]      # 3.0, 4.0
WL_SRS = [float(w) for w in hw.WAVELENGTHS_NM]                      # 进反演的四路
WL_REF = [float(w) for w in hw.WL_REFERENCE_ONLY_NM]                # 950，只存档

# 每条曲线最多留多少点。单组合的点率 = 实测帧率 ÷ 10 个组合；
# 按模拟器的 100 帧/s 算是 10 点/s，6000 点 ≈ 10 分钟。
# ⚠️ 真机帧率未实测，实际覆盖时长按连接栏的实测值换算。
MAXPTS = 6000
REFRESH_HZ = 10
# 配对窗口：3cm 和 4cm 在同一个扫描周期里只差一帧（~10 ms）。超过 1 秒
# 还没等到配对的 4cm，说明中间丢过帧或设备没在扫全部波长，这个点就不要了。
PAIR_MAX_S = 1.0
DEFAULT_SPAN_S = 30

COLOR = {0.0: "#2a78d6", 670.0: "#d62728", 770.0: "#ff7f0e",
         810.0: "#2ca02c", 850.0: "#1f77b4", 950.0: "#999999"}


class LivePlotWindow(QtWidgets.QDialog):
    """从父窗口的环形缓冲里取帧画图。自身不碰串口。"""

    def __init__(self, parent, buf: deque) -> None:
        super().__init__(parent)
        self.buf = buf
        self.paused = False
        self._seen = -1
        self._dark = {3.0: 0.0, 4.0: 0.0}
        self._slot_dark: dict[float, float] = {}
        self._has_slot = False
        self._n_bad_log = 0        # 扣暗后 ≤0、算不了斜率的点数
        self._build()
        self._autofill_dark()
        self._rebuild()
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.refresh)
        self._timer.start(int(1000 / REFRESH_HZ))

    # -- UI ----------------------------------------------------------------
    def _build(self) -> None:
        self.setWindowTitle("实时曲线 —— 两个探测器 + 空间斜率")
        self.resize(1180, 800)
        pg.setConfigOption("background", "w")
        pg.setConfigOption("foreground", "k")
        v = QtWidgets.QVBoxLayout(self)

        # ---- 工具条 ----
        bar = QtWidgets.QHBoxLayout()
        self.btn_pause = QtWidgets.QPushButton("暂停")
        self.btn_pause.setCheckable(True)
        self.btn_pause.toggled.connect(self._on_pause)

        self.spn_span = QtWidgets.QSpinBox()
        self.spn_span.setRange(5, 600)
        self.spn_span.setValue(DEFAULT_SPAN_S)
        self.spn_span.setSuffix(" s 窗口")

        self.chk_log = QtWidgets.QCheckBox("对数纵轴")
        self.chk_log.setToolTip(
            "3cm 比 4cm 亮 10~35 倍，线性轴上 4cm 会被压成一条平线。\n"
            "⚠️ 对数轴画不了 ≤0 的点（遮光时 3cm 是负的），那些点会断开。")
        self.chk_log.toggled.connect(self._apply_log)

        self.chk_dark = QtWidgets.QCheckBox("扣暗电平")
        self.chk_dark.setToolTip(
            "新固件每周期带一帧暗时隙，勾上即**逐周期就近扣**；\n"
            "旁边两个数字只在没有暗时隙时（旧固件）作为静态值生效。\n"
            "暗现在占原始读数 90%+，不扣的话斜率完全没有意义。")
        self.chk_dark.toggled.connect(self._rebuild)
        self.spn_d3 = QtWidgets.QDoubleSpinBox()
        self.spn_d4 = QtWidgets.QDoubleSpinBox()
        for s in (self.spn_d3, self.spn_d4):
            s.setRange(-1e9, 1e9)
            s.setDecimals(0)
            s.setSingleStep(1000)
            s.setMinimumWidth(110)
            s.valueChanged.connect(self._rebuild)
        self.btn_dark_auto = QtWidgets.QPushButton("重取暗")
        self.btn_dark_auto.setToolTip(
            "优先用帧流里的暗时隙（每周期一帧 LED 全灭，波长码 0x00）；\n"
            "没有暗时隙时（旧固件）才退回主窗口的手动暗采记录。")
        self.btn_dark_auto.clicked.connect(lambda: self._autofill_dark(force=True))

        self.btn_clear = QtWidgets.QPushButton("清屏")
        self.btn_clear.clicked.connect(self._rebuild)
        self.btn_png = QtWidgets.QPushButton("导出 PNG")
        self.btn_png.setToolTip("把当前三格存成图片 —— 给硬件看图比报数字管用")
        self.btn_png.clicked.connect(self._export_png)

        for w in (self.btn_pause, self.spn_span, self.chk_log,
                  _sep(), self.chk_dark, QtWidgets.QLabel("3cm"), self.spn_d3,
                  QtWidgets.QLabel("4cm"), self.spn_d4, self.btn_dark_auto,
                  _sep(), self.btn_clear, self.btn_png):
            bar.addWidget(w)
        bar.addStretch(1)
        self.lbl_stat = QtWidgets.QLabel("—")
        self.lbl_stat.setStyleSheet("color:#666")
        bar.addWidget(self.lbl_stat)
        v.addLayout(bar)

        # ---- 共用图例 ----
        # 不再用 pyqtgraph 的内嵌图例：6 条曲线时它比格子还高，底下几条
        # 会被裁掉（实测只显示到 810）。改成顶部一条横排图例，既不占
        # 纵向空间，也不会被裁。
        lg = QtWidgets.QHBoxLayout()
        lg.addWidget(QtWidgets.QLabel("<b>图例</b>"))
        for w in [0.0] + list(WLS):
            sq = QtWidgets.QLabel()
            sq.setFixedSize(26, 4)
            sq.setStyleSheet(f"background:{COLOR[w]}")
            tx = QtWidgets.QLabel("暗" if w == 0.0 else
                                  f"{w:.0f}" + ("(参考)" if w in WL_REF else ""))
            if w in WL_REF:
                tx.setStyleSheet("color:#777")
            lg.addWidget(sq)
            lg.addWidget(tx)
            lg.addSpacing(10)
        lg.addStretch(1)
        v.addLayout(lg)

        # ---- 四格图 ----
        self.glw = pg.GraphicsLayoutWidget()
        v.addWidget(self.glw, 1)
        self.curve: dict[tuple[float, float], pg.PlotDataItem] = {}
        self.plots: list[pg.PlotItem] = []

        # 前两格：各探测器的**信号**（扣暗开关决定扣不扣）。暗自己不画在
        # 这里 —— 它比信号大一个量级，画进来会把信号压成贴底的平线。
        for row, rho in enumerate(RHOS):
            p = self.glw.addPlot(row=row, col=0, title=f"探测器 {rho:.0f} cm")
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setLabel("left", "ADC")
            for w in WLS:
                pen = pg.mkPen(COLOR[w], width=1.6,
                               style=QtCore.Qt.DashLine if w in WL_REF
                               else QtCore.Qt.SolidLine)
                c = p.plot(pen=pen)
                c.setDownsampling(auto=True)
                c.setClipToView(True)
                self.curve[(w, rho)] = c
            self.plots.append(p)

        # 第三格：暗时隙本身。单独一格是因为量级差一个数量级，
        # 而且它是链路健康度指标，值得一直盯着。
        pd = self.glw.addPlot(row=2, col=0, title="暗时隙（LED 全灭）—— 扣暗的依据")
        pd.showGrid(x=True, y=True, alpha=0.3)
        pd.setLabel("left", "ADC")
        self.dcurve: dict[float, pg.PlotDataItem] = {}
        for rho in RHOS:
            c = pd.plot(pen=pg.mkPen(COLOR[0.0], width=1.6,
                                     style=QtCore.Qt.SolidLine if rho == 3.0
                                     else QtCore.Qt.DashLine))
            c.setDownsampling(auto=True)
            c.setClipToView(True)
            self.dcurve[rho] = c
        self.plots.append(pd)

        ps = self.glw.addPlot(row=3, col=0,
                              title="空间斜率 log₁₀(I₃cm / I₄cm) —— SRS 真正用的量")
        ps.showGrid(x=True, y=True, alpha=0.3)
        ps.setLabel("left", "斜率")
        ps.setLabel("bottom", "时间 (s)")
        self.scurve: dict[float, pg.PlotDataItem] = {}
        for w in WLS:
            pen = pg.mkPen(COLOR[w], width=2.0,
                           style=QtCore.Qt.DashLine if w in WL_REF
                           else QtCore.Qt.SolidLine)
            c = ps.plot(pen=pen)
            c.setDownsampling(auto=True)
            c.setClipToView(True)
            self.scurve[w] = c
        self.plots.append(ps)
        for p in self.plots[1:]:
            p.setXLink(self.plots[0])
        for i, f in enumerate((3, 3, 2, 4)):
            self.glw.ci.layout.setRowStretchFactor(i, f)

        # ---- 读数表 ----
        self.table = QtWidgets.QTableWidget(len(WLS), 5)
        self.table.setHorizontalHeaderLabels(
            ["I(3cm)", "I(4cm)", "I₃/I₄", "斜率", "偏离光谱趋势"])
        self.table.setVerticalHeaderLabels([f"{w:.0f} nm" for w in WLS])
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.Stretch)
        self.table.setFixedHeight(
            self.table.horizontalHeader().height()
            + sum(self.table.rowHeight(r) for r in range(len(WLS))) + 4)
        mono = QtGui.QFont("Consolas")
        mono.setStyleHint(QtGui.QFont.Monospace)
        self.table.setFont(mono)
        v.addWidget(self.table)

        hint = QtWidgets.QLabel(
            "<b>怎么看</b>　上两格是两个探测器各自的 5 条波长，用来判「亮不亮、"
            "稳不稳、有没有饱和或漂移」。<b>第三格才是标定用的量</b> —— 同一块"
            "体模上，四条实线应当随波长平缓变化、互不交叉；某一路明显掉下去，"
            "就是那一路有串扰/泄漏。第三格是暗时隙本身 —— 它的抖动决定扣暗"
            "能扣多准，实测白噪声地板 474/588，明显更大就是链路在漂。"
            "最后一列「偏离光谱趋势」= 该路斜率减去"
            "<b>其余各路连成的直线</b>在该波长的取值，正常应在 ±0.05 以内；"
            "基准线只用拟合得最好的三路，所以一路坏不会把其他三路也染红。<br>"
            "表里的斜率是 log₁₀(均值I₃/均值I₄)，和标定算的是同一个数；第三格"
            "画的是逐帧斜率，4cm 噪声大时曲线中心会比表里略高，属正常。")
        hint.setWordWrap(True)
        hint.setStyleSheet("background:#f6f6f4;padding:7px;border:1px solid #e3e2dd")
        v.addWidget(hint)

    # -- 数据 --------------------------------------------------------------
    def _autofill_dark(self, force: bool = False) -> None:
        """取暗电平。优先用帧流里的**暗时隙**，没有才退回手动暗采。

        2026-10-10 的固件在每个周期最前发一帧 LED 全灭的暗（波长码 0x00）。
        它比任何静态值都准 —— 暗电平的漂移时间常数只有几个周期。
        手动暗采的那一路保留，只为还能读旧固件的数据。
        """
        got = {}
        for _seq, _t, f in self.buf:
            if getattr(f, "is_dark", False):
                got.setdefault(f.distance_cm, []).append(float(f.adc))
        if got:
            for rho in RHOS:
                if got.get(rho):
                    self._dark[rho] = float(np.mean(got[rho]))
            self.spn_d3.setValue(self._dark[3.0])
            self.spn_d4.setValue(self._dark[4.0])
            self.chk_dark.setChecked(True)
            self._has_slot = True
            return
        self._has_slot = False
        takes = [t for t in getattr(self.parent(), "takes", [])
                 if t.block == "DARK" and t.mean]
        if not takes:
            if force:
                QtWidgets.QMessageBox.information(
                    self, "没有暗电平",
                    "帧流里没有暗时隙（旧固件？），主窗口也没有「暗电平」"
                    "采集记录。\n先遮死探头点一次暗采，或在这里手填。")
            return
        tk = takes[-1]
        for rho in RHOS:
            vals = [v for (w, r), v in tk.mean.items() if r == rho]
            if vals:
                self._dark[rho] = float(np.mean(vals))
        self.spn_d3.setValue(self._dark[3.0])
        self.spn_d4.setValue(self._dark[4.0])
        self.chk_dark.setChecked(True)

    def _offset(self, rho: float) -> float:
        """该用哪个暗值。

        有暗时隙时用**逐周期**的值（_feed 里按周期更新 _slot_dark）；
        没有则退回手填的静态值。
        """
        if not self.chk_dark.isChecked():
            return 0.0
        if self._has_slot and rho in self._slot_dark:
            return self._slot_dark[rho]
        return self.spn_d3.value() if rho == 3.0 else self.spn_d4.value()

    def _rebuild(self) -> None:
        """清空并从环形缓冲重放。

        扣暗开关、暗电平数值一变，历史点就都算错了 —— 与其打补丁，不如
        整段重算。缓冲上限两万帧，重放一次只有毫秒级。
        """
        self.series = {(w, r): (deque(maxlen=MAXPTS), deque(maxlen=MAXPTS))
                       for w in [0.0] + list(WLS) for r in RHOS}
        self._slot_dark: dict[float, float] = {}
        self.slope = {w: (deque(maxlen=MAXPTS), deque(maxlen=MAXPTS))
                      for w in WLS}
        self._last3: dict[float, tuple[float, float]] = {}
        self._n_bad_log = 0
        self._seen = -1
        # 标题随扣暗开关走 —— 否则看一眼图不知道画的是原始还是扣暗后，
        # 而暗占读数 90%+，两者差一个量级。
        if getattr(self, "plots", None):
            on = self.chk_dark.isChecked()
            for i, rho in enumerate(RHOS):
                self.plots[i].setTitle(
                    f"探测器 {rho:.0f} cm —— "
                    + ("<b>扣暗后</b>" if on else
                       "<span style='color:#c33'>原始值（未扣暗）</span>"))
        for seq, t, f in list(self.buf):
            self._feed(seq, t, f)

    def _feed(self, seq: int, t: float, f) -> None:
        self._seen = seq
        rho = f.distance_cm
        wl = f.wavelength_nm
        if getattr(f, "is_dark", False):
            # 暗时隙：更新「本周期的暗」，供后面的信号帧扣。它自己也画出来
            # —— 暗电平在漂多少，是链路健康度最直接的指标。
            self._slot_dark[rho] = float(f.adc)
            ts, vs = self.series[(0.0, rho)]
            ts.append(t)
            vs.append(float(f.adc))
            return
        if (wl, rho) not in self.series:
            return                              # 协议外的组合，忽略
        val = f.adc - self._offset(rho)
        ts, vs = self.series[(wl, rho)]
        ts.append(t)
        vs.append(val)
        if rho == 3.0:
            self._last3[wl] = (t, val)
            return
        prev = self._last3.get(wl)
        if prev is None or t - prev[0] > PAIR_MAX_S:
            return
        if prev[1] <= 0 or val <= 0:
            # 遮光或扣暗过头时会出现，取不了对数。记一笔，不画点。
            self._n_bad_log += 1
            return
        st, sv = self.slope[wl]
        st.append(t)
        sv.append(float(np.log10(prev[1] / val)))

    def _pull(self) -> None:
        for seq, t, f in list(self.buf):
            if seq > self._seen:
                self._feed(seq, t, f)

    # -- 刷新 --------------------------------------------------------------
    def _on_pause(self, on: bool) -> None:
        self.paused = on
        self.btn_pause.setText("继续" if on else "暂停")

    def _apply_log(self, on: bool) -> None:
        for p in self.plots[:2]:                # 斜率那格本身就是对数量，不再取对数
            p.setLogMode(x=False, y=on)

    def refresh(self) -> None:
        self._pull()                            # 暂停时照收，只是不画
        if self.paused or not self.isVisible():
            self._stat()
            return
        for k, c in self.curve.items():
            ts, vs = self.series[k]
            c.setData(np.fromiter(ts, float, len(ts)),
                      np.fromiter(vs, float, len(vs)))
        for rho, c in self.dcurve.items():
            ts, vs = self.series[(0.0, rho)]
            c.setData(np.fromiter(ts, float, len(ts)),
                      np.fromiter(vs, float, len(vs)))
        for w, c in self.scurve.items():
            ts, vs = self.slope[w]
            c.setData(np.fromiter(ts, float, len(ts)),
                      np.fromiter(vs, float, len(vs)))
        t_end = max((ts[-1] for ts, _ in self.series.values() if ts),
                    default=0.0)
        span = self.spn_span.value()
        self.plots[0].setXRange(max(0.0, t_end - span), max(span, t_end),
                                padding=0)
        self._fill_table(t_end - span)
        self._stat()

    def _fill_table(self, t_from: float) -> None:
        """表里给窗口内的**均值**，不是最新一帧 —— 单帧跳动看不出门道。

        ⚠️ 斜率这一列是 log₁₀(均值I₃ / 均值I₄)，**不是**逐帧斜率的平均。
        两者不等（Jensen），而且差得不小：C2 的 670 nm 上差 0.085 —— 4cm
        只有 9 千多个计数，相对噪声大，先取对数再平均会被系统性抬高。
        这里跟 run_calibration 保持一致（它读「汇总」表的 adc_mean），
        否则现场看到的数和最后算出来的 G 对不上。
        第三格画的仍是逐帧斜率 —— 那一格要的是**形状**（漂移、跳变），
        所以 4cm 噪声大时，曲线的视觉中心会比表里的数略高，这是正常的。
        """
        def win_mean(seq_t, seq_v):
            a = [v for t, v in zip(seq_t, seq_v) if t >= t_from]
            return float(np.mean(a)) if a else float("nan")

        i3 = {w: win_mean(*self.series[(w, 3.0)]) for w in WLS}
        i4 = {w: win_mean(*self.series[(w, 4.0)]) for w in WLS}
        sl = {w: (float(np.log10(i3[w] / i4[w]))
                  if np.isfinite(i3[w]) and np.isfinite(i4[w])
                  and i3[w] > 0 and i4[w] > 0 else float("nan"))
              for w in WLS}
        dev = _trend_dev({w: sl[w] for w in WL_SRS if np.isfinite(sl[w])})

        for r, w in enumerate(WLS):
            ratio = i3[w] / i4[w] if i4[w] else float("nan")
            cells = [_fmt(i3[w], ",.0f"), _fmt(i4[w], ",.0f"), _fmt(ratio, ".2f"),
                     _fmt(sl[w], ".3f"),
                     _fmt(dev.get(w, float("nan")), "+.3f")]
            for c, txt in enumerate(cells):
                it = QtWidgets.QTableWidgetItem(txt)
                it.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                if w in WL_REF:
                    it.setForeground(QtGui.QBrush(QtGui.QColor("#999")))
                if c == 4 and np.isfinite(dev.get(w, float("nan"))):
                    d = abs(dev[w])
                    if d > 0.15:
                        it.setForeground(QtGui.QBrush(QtGui.QColor("#c33")))
                        it.setFont(_bold(self.table.font()))
                    elif d > 0.05:
                        it.setForeground(QtGui.QBrush(QtGui.QColor("#b8860b")))
                self.table.setItem(r, c, it)

    def _stat(self) -> None:
        n = sum(len(ts) for ts, _ in self.series.values())
        bad = (f"　<span style='color:#c33'>{self._n_bad_log:,} 点取不了对数</span>"
               if self._n_bad_log else "")
        self.lbl_stat.setText(
            f"缓冲 {len(self.buf):,} 帧　已画 {n:,} 点{bad}")
        self.lbl_stat.setTextFormat(QtCore.Qt.RichText)

    def _export_png(self) -> None:
        import pyqtgraph.exporters
        default = Path(__file__).resolve().parent / \
            f"实时曲线_{datetime.now():%Y%m%d_%H%M%S}.png"
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "导出曲线", str(default), "PNG (*.png)")
        if not path:
            return
        try:
            ex = pg.exporters.ImageExporter(self.glw.ci)
            ex.parameters()["width"] = 1800
            ex.export(path)
        except Exception as e:                                   # noqa: BLE001
            QtWidgets.QMessageBox.critical(self, "导出失败", str(e))
            return
        self.lbl_stat.setText(f"已导出 {Path(path).name}")

    def showEvent(self, e) -> None:                              # noqa: N802
        """重新打开时把定时器开回来。

        ⚠️ 这是必须的，不是保险：closeEvent 里停了定时器，而父窗口复用
        同一个对象（`_plot_win` 非空就直接 show）。只停不启 → 窗口能显示
        但永不刷新，看起来就是「卡死」。
        """
        super().showEvent(e)
        if not self._timer.isActive():
            self._timer.start(int(1000 / REFRESH_HZ))

    def closeEvent(self, e) -> None:                             # noqa: N802
        # 只停定时器、不发 finished —— 窗口和已画的历史都留着，
        # 下次打开接着画。重新启动定时器由 showEvent 负责。
        self._timer.stop()
        e.accept()


# ---------------------------------------------------------------------------
def _trend_dev(sl: dict[float, float]) -> dict[float, float]:
    """每一路的斜率偏离【其余各路连成的光谱趋势】多少。

    做法：四个波长里任取三个拟合一条直线，取**自身拟合得最好**的那个三元组
    当基准，再报全部四路相对这条线的残差。四个波长上斜率随 λ 近似线性，
    直线够用。

    为什么不是简单的留一法：只有一路坏的时候，留一法会互相牵连 ——
    给 850 拟合基准时 810 还在里面，把线拉歪，850 跟着显示 +0.60，
    看上去四路都有毛病。2026-10-09 的 C2 实测，换成这里的做法：
    670/770/850 残差都在 ±0.001 以内，810 是 −0.657，一眼知道该找谁。

    为什么不先设个阈值挑坏点：四路都正常时，四个三元组拟合得一样好，
    选中哪个都给出接近零的残差 —— 退化情况自己就对了，不需要阈值。
    """
    wl = sorted(sl)
    if len(wl) < 4:
        return {}
    x = np.array(wl, dtype=float)
    y = np.array([sl[w] for w in wl], dtype=float)
    best: tuple[float, float, float] | None = None
    for i in range(len(wl)):
        m = np.arange(len(wl)) != i
        k, b = np.polyfit(x[m], y[m], 1)
        rms = float(np.sqrt(np.mean((y[m] - (k * x[m] + b)) ** 2)))
        if best is None or rms < best[0]:
            best = (rms, float(k), float(b))
    _, k, b = best
    return {w: float(y[i] - (k * w + b)) for i, w in enumerate(wl)}


def _fmt(v: float, spec: str) -> str:
    return "—" if not np.isfinite(v) else format(v, spec)


def _bold(f: QtGui.QFont) -> QtGui.QFont:
    g = QtGui.QFont(f)
    g.setBold(True)
    return g


def _sep() -> QtWidgets.QFrame:
    f = QtWidgets.QFrame()
    f.setFrameShape(QtWidgets.QFrame.VLine)
    f.setFrameShadow(QtWidgets.QFrame.Sunken)
    return f
