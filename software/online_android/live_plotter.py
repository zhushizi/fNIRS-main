"""实时绘制回传安卓的 live_analysis_batch 数据。"""

from __future__ import annotations

import threading
from typing import TYPE_CHECKING

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets

if TYPE_CHECKING:
    from .types import LiveAnalysisBatch

HBO_HBR_WINDOW_S = 30.0
RSO2_WINDOW_S = 60.0
RETENTION_BUFFER_S = 15.0


class _PlotSignals(QtCore.QObject):
    batch_received = QtCore.pyqtSignal(object)


class _LivePlotWindow(QtWidgets.QWidget):
    def __init__(
        self,
        hbo_hbr_window_s: float = HBO_HBR_WINDOW_S,
        rso2_window_s: float = RSO2_WINDOW_S,
    ) -> None:
        super().__init__()
        self.hbo_hbr_window_s = hbo_hbr_window_s
        self.rso2_window_s = rso2_window_s
        self._retention_s = max(hbo_hbr_window_s, rso2_window_s) + RETENTION_BUFFER_S
        self._times: list[float] = []
        self._hbo: list[float] = []
        self._hbr: list[float] = []
        self._rso2_times: list[float] = []
        self._rso2: list[float] = []

        self.setWindowTitle("Live Android Batch Mirror")
        layout = QtWidgets.QVBoxLayout(self)

        self.hbo_hbr_plot = pg.PlotWidget(
            title=f"HbO / HbR sent to Android (last {hbo_hbr_window_s:.0f}s)",
        )
        self.hbo_hbr_plot.addLegend()
        self.hbo_hbr_plot.showGrid(x=True, y=True, alpha=0.3)
        self.hbo_hbr_plot.setLabel("bottom", "Time (s)")
        self.hbo_hbr_plot.setLabel("left", "Δ conc.")
        self.hbo_curve = self.hbo_hbr_plot.plot(
            pen=pg.mkPen("#c0392b", width=2),
            name="HbO",
        )
        self.hbr_curve = self.hbo_hbr_plot.plot(
            pen=pg.mkPen("#2980b9", width=2),
            name="HbR",
        )
        layout.addWidget(self.hbo_hbr_plot)

        self.rso2_plot = pg.PlotWidget(
            title=f"rSO2 sent to Android (last {rso2_window_s:.0f}s)",
        )
        self.rso2_plot.addLegend()
        self.rso2_plot.showGrid(x=True, y=True, alpha=0.3)
        self.rso2_plot.setLabel("bottom", "Time (s)")
        self.rso2_plot.setLabel("left", "rSO2 (%)")
        self.rso2_curve = self.rso2_plot.plot(
            pen=pg.mkPen("#27ae60", width=2),
            name="rSO2",
        )
        layout.addWidget(self.rso2_plot)

        self.status_label = QtWidgets.QLabel("Waiting for live_analysis_batch...")
        layout.addWidget(self.status_label)

    def append_batch(self, batch: LiveAnalysisBatch) -> None:
        if not batch.times:
            return

        for elapsed_time, hbo, hbr in zip(batch.times, batch.hbo, batch.hbr):
            self._times.append(float(elapsed_time))
            self._hbo.append(float(hbo))
            self._hbr.append(float(hbr))

        if batch.rso2:
            for elapsed_time, rso2 in zip(batch.times, batch.rso2):
                if rso2 is not None and np.isfinite(rso2):
                    self._rso2_times.append(float(elapsed_time))
                    self._rso2.append(float(rso2))

        self._prune_old_points()
        self._refresh_hbo_hbr()
        self._refresh_rso2()

        latest_rso2 = batch.latest_rso2_pct
        rso2_txt = f"{latest_rso2:.1f}%" if latest_rso2 is not None else "N/A"
        self.status_label.setText(
            f"TX window {batch.window_start_s:.1f}s–{batch.window_end_s:.1f}s | "
            f"new points={len(batch.times)} | latest rSO2={rso2_txt}"
        )

    def _prune_old_points(self) -> None:
        if not self._times:
            return
        cutoff = self._times[-1] - self._retention_s
        while self._times and self._times[0] < cutoff:
            self._times.pop(0)
            self._hbo.pop(0)
            self._hbr.pop(0)
        while self._rso2_times and self._rso2_times[0] < cutoff:
            self._rso2_times.pop(0)
            self._rso2.pop(0)

    @staticmethod
    def _slice_window(
        times: list[float],
        values: list[float],
        window_s: float,
    ) -> tuple[list[float], list[float]]:
        if not times:
            return [], []
        cutoff = times[-1] - window_s
        xs: list[float] = []
        ys: list[float] = []
        for elapsed_time, value in zip(times, values):
            if elapsed_time >= cutoff:
                xs.append(elapsed_time)
                ys.append(value)
        return xs, ys

    def _refresh_hbo_hbr(self) -> None:
        t_hbo, hbo = self._slice_window(self._times, self._hbo, self.hbo_hbr_window_s)
        _, hbr = self._slice_window(self._times, self._hbr, self.hbo_hbr_window_s)
        self.hbo_curve.setData(t_hbo, hbo)
        self.hbr_curve.setData(t_hbo, hbr)
        if t_hbo:
            self.hbo_hbr_plot.setXRange(t_hbo[0], t_hbo[-1], padding=0.02)

    def _refresh_rso2(self) -> None:
        t_rso2, rso2 = self._slice_window(self._rso2_times, self._rso2, self.rso2_window_s)
        self.rso2_curve.setData(t_rso2, rso2)
        if t_rso2:
            self.rso2_plot.setXRange(t_rso2[0], t_rso2[-1], padding=0.02)


class _LivePlotThread(QtCore.QThread):
    """专用 QThread：所有 Qt / pyqtgraph 对象只在此线程创建与更新。"""

    def __init__(
        self,
        hbo_hbr_window_s: float,
        rso2_window_s: float,
        ready_event: threading.Event,
        signals_holder: list[_PlotSignals],
    ) -> None:
        super().__init__()
        self._hbo_hbr_window_s = hbo_hbr_window_s
        self._rso2_window_s = rso2_window_s
        self._ready_event = ready_event
        self._signals_holder = signals_holder

    def run(self) -> None:
        pg.setConfigOptions(antialias=True)
        app = QtWidgets.QApplication.instance()
        if app is None:
            app = QtWidgets.QApplication([])

        signals = _PlotSignals()
        window = _LivePlotWindow(self._hbo_hbr_window_s, self._rso2_window_s)
        signals.batch_received.connect(
            window.append_batch,
            QtCore.Qt.QueuedConnection,
        )
        window.show()

        self._signals_holder.append(signals)
        self._ready_event.set()
        app.exec_()


class LiveAndroidBatchPlotter:
    """在独立 QThread 中实时绘制发往安卓的 live_analysis_batch。"""

    def __init__(
        self,
        hbo_hbr_window_s: float = HBO_HBR_WINDOW_S,
        rso2_window_s: float = RSO2_WINDOW_S,
    ) -> None:
        self._ready = threading.Event()
        self._signals_holder: list[_PlotSignals] = []
        self._thread = _LivePlotThread(
            hbo_hbr_window_s,
            rso2_window_s,
            self._ready,
            self._signals_holder,
        )
        self._thread.start()
        if not self._ready.wait(timeout=10.0):
            raise RuntimeError("Live plot window failed to start within 10s.")

    def update(self, batch: LiveAnalysisBatch) -> None:
        if not self._signals_holder:
            return
        # 从在线分析线程跨线程投递；由 QueuedConnection 在 QThread 内刷新 UI。
        self._signals_holder[0].batch_received.emit(batch)
