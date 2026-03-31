"""
单通道实时 HbO / HbR 曲线查看器。

从串口持续读取 660/940 光强，用前几秒建立基线，再对每个 660–940 对做 OD 与 MBLL，
将得到的 HbO、HbR 实时画在坐标图上。不做带通和 CBSI，曲线会比离线 processed_output 更抖。
"""

from __future__ import annotations

import sys
import time
from collections import deque

import numpy as np
import nirsimple.preprocessing as nsp
import pyqtgraph as pg
import serial
from PyQt5 import QtCore, QtWidgets

from config import (
    ACK_TIMEOUT_SECONDS,
    BAUD_RATE,
    CHANNEL_NAME,
    DEFAULT_INTENSITY_MA,
    MAX_RETRIES,
    SERIAL_PORT,
    SOURCE_DETECTOR_DISTANCE_CM,
    TIMEOUT,
    WAVELENGTH_660_CODE,
    WAVELENGTH_940_CODE,
)
from protocol import FrameReader, build_command_frame, parse_data_frame, send_frame_with_ack

# 用前若干秒的平均光强作为 OD 基线
BASELINE_SECONDS = 5.0
# MBLL 更新节流：至少间隔多少秒再算下一对（避免 GUI 卡顿）
UPDATE_INTERVAL_S = 0.1
# 绘图时间窗（秒）
WINDOW_SECONDS = 60.0
MBLL_AGE = 22


def open_serial() -> serial.Serial:
    return serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=TIMEOUT)


def _channel_info():
    """单通道双波长的通道名、波长、DPF、源探距离，与 fNIRS_processing 一致。"""
    ch_names = [CHANNEL_NAME, CHANNEL_NAME]
    ch_wls = [660.0, 940.0]
    ch_dpfs = [nsp.get_dpf(660.0, MBLL_AGE), nsp.get_dpf(940.0, MBLL_AGE)]
    ch_distances = [SOURCE_DETECTOR_DISTANCE_CM, SOURCE_DETECTOR_DISTANCE_CM]
    return ch_names, ch_wls, ch_dpfs, ch_distances


class SerialReaderThread(QtCore.QThread):
    """后台串口读线程，与 adc_live 相同逻辑：发启动命令、持续读 0x02、回 ACK、发信号。"""
    newData = QtCore.pyqtSignal(float, int, int)

    def __init__(self, ser: serial.Serial, parent=None):
        super().__init__(parent)
        self.ser = ser
        self.reader = FrameReader(ser)
        self.running = True
        self.start_time = time.time()

    def _emit(self, sample):
        t = time.time() - self.start_time
        self.newData.emit(t, sample.wavelength_code, sample.value)

    def _start_stream(self) -> bool:
        cmd = build_command_frame(True, DEFAULT_INTENSITY_MA)
        deadline = time.time() + max(ACK_TIMEOUT_SECONDS * 10, 0.2)
        for attempt in range(MAX_RETRIES + 1):
            self.ser.write(cmd)
            while time.time() < deadline:
                frame = self.reader.read_frame(timeout_seconds=min(TIMEOUT, max(0.01, deadline - time.time())))
                if frame is None:
                    continue
                if frame.frame_type == 0x02:
                    self._emit(parse_data_frame(frame))
                    return True
            deadline = time.time() + max(ACK_TIMEOUT_SECONDS * 10, 0.2)
        return False

    def run(self):
        started = self._start_stream()
        if not started:
            print("[hbo_hbr_live] Start command sent (max attempts reached). Keep waiting for data frames...")
        while self.running:
            frame = self.reader.read_frame(timeout_seconds=TIMEOUT)
            if frame is None or frame.frame_type != 0x02:
                continue
            self._emit(parse_data_frame(frame))
        try:
            send_frame_with_ack(self.ser, self.reader, build_command_frame(False, DEFAULT_INTENSITY_MA))
        except Exception:
            pass

    def stop(self):
        self.running = False


class MainWindow(QtWidgets.QWidget):
    """实时 HbO / HbR 坐标图。前几秒仅收数据建基线，之后每收到新 660–940 对就更新一条 MBLL 点。"""
    def __init__(self):
        super().__init__()
        pg.setConfigOption("background", "w")
        pg.setConfigOption("foreground", "k")
        self.setWindowTitle("S1_D1 HbO / HbR 实时")

        self.start_time = time.time()
        self.ref_660: float | None = None
        self.ref_940: float | None = None
        self.baseline_660: list[float] = []
        self.baseline_940: list[float] = []
        self.last_mbll_time = 0.0

        self.time_660 = deque(maxlen=5000)
        self.data_660 = deque(maxlen=5000)
        self.time_940 = deque(maxlen=5000)
        self.data_940 = deque(maxlen=5000)

        self.time_hbo = deque(maxlen=3000)
        self.hbo_vals = deque(maxlen=3000)
        self.hbr_vals = deque(maxlen=3000)

        layout = QtWidgets.QVBoxLayout(self)
        self.plot = pg.PlotWidget(title="HbO / HbR 实时")
        self.plot.showGrid(x=True, y=True)
        self.plot.setLabel("bottom", "Time (s)")
        self.plot.setLabel("left", "Concentration (Δ)")
        self.plot.addLegend()
        self.plot.setXRange(0, WINDOW_SECONDS, padding=0)
        self.curve_hbo = self.plot.plot(pen=pg.mkPen("r", width=2), name="HbO")
        self.curve_hbr = self.plot.plot(pen=pg.mkPen("b", width=2), name="HbR")
        layout.addWidget(self.plot)

        self.status = QtWidgets.QLabel("建立基线…")
        layout.addWidget(self.status)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._update_plot)
        self.timer.start(50)

    @QtCore.pyqtSlot(float, int, int)
    def on_new_data(self, elapsed: float, wavelength_code: int, value: int):
        if value <= 0:
            return
        if wavelength_code == WAVELENGTH_660_CODE:
            self.time_660.append(elapsed)
            self.data_660.append(float(value))
            if self.ref_660 is None:
                self.baseline_660.append(float(value))
        elif wavelength_code == WAVELENGTH_940_CODE:
            self.time_940.append(elapsed)
            self.data_940.append(float(value))
            if self.ref_940 is None:
                self.baseline_940.append(float(value))

        # 用串口时间戳判断：满 BASELINE_SECONDS 且有两路数据则固定基线
        if self.ref_660 is None and elapsed >= BASELINE_SECONDS and self.baseline_660 and self.baseline_940:
            self.ref_660 = float(np.mean(self.baseline_660))
            self.ref_940 = float(np.mean(self.baseline_940))
            self.status.setText(f"基线就绪 ref_660={self.ref_660:.0f} ref_940={self.ref_940:.0f}")

        if self.ref_660 is not None and self.ref_940 is not None:
            if (elapsed - self.last_mbll_time) < UPDATE_INTERVAL_S:
                return
            if not self.data_660 or not self.data_940:
                return
            t_660, t_940 = self.time_660[-1], self.time_940[-1]
            t = (t_660 + t_940) * 0.5
            i_660 = self.data_660[-1]
            i_940 = self.data_940[-1]
            i_660 = max(i_660, 1.0)
            i_940 = max(i_940, 1.0)
            delta_od_660 = -np.log10(i_660 / self.ref_660)
            delta_od_940 = -np.log10(i_940 / self.ref_940)
            delta_od = np.array([[delta_od_660], [delta_od_940]], dtype=float)
            ch_names, ch_wls, ch_dpfs, ch_distances = _channel_info()
            try:
                delta_c, names, types = nsp.mbll(
                    delta_od,
                    ch_names,
                    ch_wls,
                    ch_dpfs,
                    ch_distances,
                    unit="cm",
                    table="wray",
                )
            except Exception:
                return
            flat = np.ravel(delta_c)
            if flat.size < 2:
                return
            hbo_val = float(flat[0])
            hbr_val = float(flat[1])
            self.last_mbll_time = elapsed
            self.time_hbo.append(t)
            self.hbo_vals.append(hbo_val)
            self.hbr_vals.append(hbr_val)
            self.status.setText(f"HbO={hbo_val:.4e}  HbR={hbr_val:.4e}  t={t:.2f}s")

    def _update_plot(self):
        if self.time_hbo:
            self.curve_hbo.setData(list(self.time_hbo), list(self.hbo_vals))
            self.curve_hbr.setData(list(self.time_hbo), list(self.hbr_vals))
            t_max = max(self.time_hbo)
            x_min = max(0.0, t_max - WINDOW_SECONDS)
            self.plot.setXRange(x_min, max(t_max, WINDOW_SECONDS), padding=0)


def main():
    print(f"[hbo_hbr_live] Opening {SERIAL_PORT} @ {BAUD_RATE} ...")
    ser = open_serial()
    ser.reset_input_buffer()
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    thread = SerialReaderThread(ser)
    thread.newData.connect(window.on_new_data)
    thread.start()
    app.aboutToQuit.connect(lambda: (thread.stop(), thread.wait(2000), ser.close()))
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
