# -*- coding: utf-8 -*-
"""原始帧查看器 —— 按协议字段分段显示每一帧的 26 字节。

两个用途：
  ① 现场核协议。字段分段 + 校验结果 + 保留区是否全零，一眼能看出设备
     是不是按我们反推的格式在发。保留区如果不是全零，说明里面还有东西，
     或者协议改过了 —— 这一条值得盯着。
  ② 导出原始帧做噪声频谱。目前只存每组合的均值/标准差，看不出噪声是
     白噪声、50Hz 工频还是慢漂移。勾上「记录到文件」把逐帧值存下来，
     才能做频谱，把 `fig_noise.py` 的 (b) 格从排除法变成直接认人。
"""

from __future__ import annotations

import csv
from collections import deque
from datetime import datetime
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets

from srs_cal.protocol import FRAME_LEN, checksum, split_fields

MONO = "Consolas, 'Courier New', monospace"
MAX_ROWS = 400                      # 表里最多显示多少行
REFRESH_HZ = 5                      # 刷新频率。帧率高时逐帧刷新会卡


class RawFrameViewer(QtWidgets.QDialog):
    """从父窗口的环形缓冲里取帧显示。自身不碰串口。"""

    HEADERS = ["#", "时间", "波长", "距离", "ADC(hex)", "ADC(十进制)",
               "通道", "校验", "保留区", "完整 26 字节（按字段分段）"]

    def __init__(self, parent, buf: deque) -> None:
        super().__init__(parent)
        self.buf = buf
        self.paused = False
        self._logger: csv.writer | None = None
        self._logfile = None
        self._logged = 0
        self._last_seq = -1
        self._build()
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.refresh)
        self._timer.start(int(1000 / REFRESH_HZ))

    def _build(self) -> None:
        self.setWindowTitle("原始帧 —— 26 字节")
        self.resize(1180, 620)
        v = QtWidgets.QVBoxLayout(self)

        bar = QtWidgets.QHBoxLayout()
        self.btn_pause = QtWidgets.QPushButton("暂停")
        self.btn_pause.setCheckable(True)
        self.btn_pause.toggled.connect(self._on_pause)
        self.cmb_wl = QtWidgets.QComboBox(); self.cmb_wl.addItem("全部波长", None)
        self.cmb_rho = QtWidgets.QComboBox(); self.cmb_rho.addItem("全部距离", None)
        from srs_cal.protocol import DISTANCE_BY_CODE, WAVELENGTH_BY_CODE
        for c in sorted(WAVELENGTH_BY_CODE):
            self.cmb_wl.addItem(f"{WAVELENGTH_BY_CODE[c]:.0f} nm", WAVELENGTH_BY_CODE[c])
        for c in sorted(DISTANCE_BY_CODE):
            self.cmb_rho.addItem(f"{DISTANCE_BY_CODE[c]:.0f} cm", DISTANCE_BY_CODE[c])
        self.chk_bad = QtWidgets.QCheckBox("只看异常帧")
        self.chk_log = QtWidgets.QCheckBox("记录原始帧到文件（供频谱分析）")
        self.chk_log.toggled.connect(self._on_log)
        self.btn_copy = QtWidgets.QPushButton("复制可见行")
        self.btn_copy.clicked.connect(self._copy)
        for w in (self.btn_pause, self.cmb_wl, self.cmb_rho, self.chk_bad,
                  self.chk_log, self.btn_copy):
            bar.addWidget(w)
        bar.addStretch(1)
        self.lbl_stat = QtWidgets.QLabel("—")
        self.lbl_stat.setStyleSheet("color:#666")
        bar.addWidget(self.lbl_stat)
        v.addLayout(bar)

        self.table = QtWidgets.QTableWidget(0, len(self.HEADERS))
        self.table.setHorizontalHeaderLabels(self.HEADERS)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.verticalHeader().setVisible(False)
        h = self.table.horizontalHeader()
        h.setSectionResizeMode(len(self.HEADERS) - 1, QtWidgets.QHeaderView.Stretch)
        v.addWidget(self.table, 1)

        lg = QtWidgets.QLabel(
            "<b>字段</b>　"
            "<span style='color:#888'>55AA</span> 帧头 ｜ "
            "<span style='color:#888'>1A</span> 长度 ｜ "
            "<span style='color:#888'>02</span> 类型 ｜ "
            "<b style='color:#2a78d6'>波长</b> 01~05 = 670/770/810/850/950 ｜ "
            "<b style='color:#1baf7a'>距离</b> 01=3cm 02=4cm ｜ "
            "<b style='color:#eb6834'>ADC</b> 大端 uint32 ｜ "
            "<b>通</b> 通道 ｜ 保留 14B ｜ "
            "<b>校验</b> = sum(b[2:25]) &amp; 0xFF")
        lg.setWordWrap(True)
        lg.setStyleSheet("background:#f6f6f4;padding:7px;border:1px solid #e3e2dd")
        v.addWidget(lg)

    # ------------------------------------------------------------------
    def _on_pause(self, on: bool) -> None:
        self.paused = on
        self.btn_pause.setText("继续" if on else "暂停")

    def _on_log(self, on: bool) -> None:
        if on:
            default = Path(__file__).resolve().parent / \
                f"原始帧_{datetime.now():%Y%m%d_%H%M%S}.csv"
            path, _ = QtWidgets.QFileDialog.getSaveFileName(
                self, "记录原始帧", str(default), "CSV (*.csv)")
            if not path:
                self.chk_log.setChecked(False)
                return
            self._logfile = open(path, "w", newline="", encoding="utf-8")
            self._logger = csv.writer(self._logfile)
            self._logger.writerow(["seq", "t_s", "wavelength_nm", "distance_cm",
                                   "adc", "channel", "checksum_ok", "hex"])
            self._logged = 0
            self._last_seq = -1
        else:
            if self._logfile is not None:
                self._logfile.close()
                QtWidgets.QMessageBox.information(
                    self, "已停止记录", f"共写入 {self._logged:,} 帧。")
            self._logger = self._logfile = None

    def _flush_log(self) -> None:
        if self._logger is None:
            return
        for seq, t, f in list(self.buf):
            if seq <= self._last_seq:
                continue
            self._last_seq = seq
            ok = checksum(f.raw) == f.raw[FRAME_LEN - 1]
            self._logger.writerow([seq, f"{t:.4f}", f"{f.wavelength_nm:.0f}",
                                   f"{f.distance_cm:.0f}", f.adc, f.channel,
                                   int(ok), f.raw.hex().upper()])
            self._logged += 1

    def _visible(self) -> list:
        wl = self.cmb_wl.currentData()
        rho = self.cmb_rho.currentData()
        out = []
        for seq, t, f in self.buf:
            if wl is not None and f.wavelength_nm != wl:
                continue
            if rho is not None and f.distance_cm != rho:
                continue
            if self.chk_bad.isChecked():
                bad = (checksum(f.raw) != f.raw[FRAME_LEN - 1]
                       or any(f.raw[11:25]) or f.channel != 1)
                if not bad:
                    continue
            out.append((seq, t, f))
        return out[-MAX_ROWS:]

    def refresh(self) -> None:
        self._flush_log()
        if self.paused or not self.isVisible():
            self._stat()
            return
        rows = self._visible()
        self.table.setRowCount(len(rows))
        mono = QtGui.QFont("Consolas"); mono.setStyleHint(QtGui.QFont.Monospace)
        for r, (seq, t, f) in enumerate(rows):
            raw = f.raw
            ok = checksum(raw) == raw[FRAME_LEN - 1]
            resv_zero = not any(raw[11:25])
            seg = split_fields(raw)
            cells = [
                str(seq), f"{t:8.3f}", f"{f.wavelength_nm:.0f}",
                f"{f.distance_cm:.0f}", seg[5], f"{f.adc:,}", f"{f.channel}",
                ("OK " if ok else "错 ") + seg[8],
                "全零" if resv_zero else "非零!",
                f"{seg[0]} {seg[1]} {seg[2]} {seg[3]} {seg[4]} {seg[5]} "
                f"{seg[6]} {seg[7]} {seg[8]}",
            ]
            for c, txt in enumerate(cells):
                it = QtWidgets.QTableWidgetItem(txt)
                if c in (4, 9):
                    it.setFont(mono)
                if c == 7 and not ok:
                    it.setForeground(QtGui.QBrush(QtGui.QColor("#c33")))
                if c == 8 and not resv_zero:
                    it.setForeground(QtGui.QBrush(QtGui.QColor("#c33")))
                if c == 6 and f.channel != 1:
                    it.setForeground(QtGui.QBrush(QtGui.QColor("#d9a521")))
                self.table.setItem(r, c, it)
        self.table.resizeColumnsToContents()
        self.table.horizontalHeader().setSectionResizeMode(
            len(self.HEADERS) - 1, QtWidgets.QHeaderView.Stretch)
        self.table.scrollToBottom()
        self._stat()

    def _stat(self) -> None:
        n = len(self.buf)
        extra = f"　已记录 {self._logged:,} 帧" if self._logger is not None else ""
        self.lbl_stat.setText(f"缓冲 {n:,} 帧（上限 {self.buf.maxlen:,}）"
                              f"　显示 {self.table.rowCount()} 行{extra}")

    def _copy(self) -> None:
        lines = ["\t".join(self.HEADERS)]
        for r in range(self.table.rowCount()):
            lines.append("\t".join(self.table.item(r, c).text()
                                   for c in range(self.table.columnCount())))
        QtWidgets.QApplication.clipboard().setText("\n".join(lines))
        self.lbl_stat.setText(f"已复制 {self.table.rowCount()} 行到剪贴板")

    def showEvent(self, e) -> None:                              # noqa: N802
        """重开时恢复刷新 —— 同 liveplot，只停不启会让窗口看起来卡死。"""
        super().showEvent(e)
        if not self._timer.isActive():
            self._timer.start(int(1000 / REFRESH_HZ))

    def closeEvent(self, e) -> None:                             # noqa: N802
        if self._logfile is not None:
            self._logfile.close()
            self._logger = self._logfile = None
            self.chk_log.setChecked(False)
        self._timer.stop()
        e.accept()
