"""
单通道实时 HbO / HbR 曲线查看器。

从串口持续读取双波长光强（协议码 0x01 / 0x02），用前几秒建立基线，再对每个波长对做 OD 与 MBLL，
将得到的 HbO、HbR 实时画在坐标图上。MBLL 波长与 data_analysis.py 一致：860 nm + 660 nm；
OD 矩阵行顺序与 data_analysis 相同（Wavelength=1 在上，Wavelength=2 在下）。带通 0.05–0.5 Hz；不做 CBSI。
"""

from __future__ import annotations

import sys
import time
from collections import deque

import numpy as np
from fnirs_pipeline.mbll_core import get_dpf, mbll_hbo_hbr
import pyqtgraph as pg
import serial
from PyQt5 import QtCore, QtWidgets
from scipy.signal import butter, resample_poly, sosfiltfilt

from config import (
    ACK_TIMEOUT_SECONDS,
    BAUD_RATE,
    CHANNEL_NAME,
    DEFAULT_INTENSITY_MA,
    MAX_RETRIES,
    MBLL_DEFAULT_AGE,
    MBLL_WAVELENGTH_WL1_NM,
    MBLL_WAVELENGTH_WL2_NM,
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
BP_LOW_HZ = 0.05
BP_HIGH_HZ = 0.5
BP_ORDER = 4
BP_TARGET_FS = 20.0


def butter_bandpass_sos(lowcut: float, highcut: float, fs: float, order: int = 4):
    """构造带通滤波器的 SOS 形式。"""
    nyq = 0.5 * fs
    if nyq <= 0 or lowcut >= highcut or highcut >= nyq:
        return None
    return butter(order, [lowcut / nyq, highcut / nyq], btype="band", output="sos")


def smart_bandpass(
    data: np.ndarray,
    fs: float,
    lowcut: float = BP_LOW_HZ,
    highcut: float = BP_HIGH_HZ,
    order: int = BP_ORDER,
    target_fs: float = BP_TARGET_FS,
) -> np.ndarray:
    """
    对 OD 数据做稳健带通。

    当采样率过高时，先降采样再滤波，最后升采样回来，
    可以减少数值不稳定和不必要的计算量。
    """
    if data.shape[1] < max(16, 3 * (order + 1) + 1):
        return data

    if fs > target_fs + 1:
        decim = int(round(fs / target_fs))
        fs_ds = fs / decim
        data_ds = resample_poly(data, up=1, down=decim, axis=1)
    else:
        decim, fs_ds, data_ds = 1, fs, data

    sos = butter_bandpass_sos(lowcut, highcut, fs_ds, order)
    if sos is None or data_ds.shape[1] < max(16, 3 * (order + 1) + 1):
        return data

    padlen = min(data_ds.shape[1] - 1, 3 * (order + 1))
    if padlen <= 0:
        return data

    data_bp = sosfiltfilt(sos, data_ds, axis=1, padtype="odd", padlen=padlen)
    if decim > 1:
        data_bp = resample_poly(data_bp, up=decim, down=1, axis=1)
    return data_bp


def open_serial() -> serial.Serial:
    return serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=TIMEOUT)


def _channel_info():
    """单通道双波长的通道名、波长、DPF、源探距离，与 data_analysis._channel_info 一致。"""
    ch_names = [CHANNEL_NAME, CHANNEL_NAME]
    ch_wls = [MBLL_WAVELENGTH_WL1_NM, MBLL_WAVELENGTH_WL2_NM]
    ch_dpfs = [
        get_dpf(MBLL_WAVELENGTH_WL1_NM, MBLL_DEFAULT_AGE),
        get_dpf(MBLL_WAVELENGTH_WL2_NM, MBLL_DEFAULT_AGE),
    ]
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
    """实时 HbO / HbR 坐标图。前几秒仅收数据建基线，之后每收到新双波长对就更新一条 MBLL 点。"""
    def __init__(self):
        super().__init__()
        pg.setConfigOption("background", "w")
        pg.setConfigOption("foreground", "k")
        self.setWindowTitle("S1_D1 HbO / HbR 实时")

        self.start_time = time.time()
        # 协议码 0x02=660 nm；0x01 在 MBLL 中按 data_analysis 使用 860 nm
        self.ref_660: float | None = None
        self.ref_wl1: float | None = None
        self.baseline_660: list[float] = []
        self.baseline_wl1: list[float] = []
        self.last_mbll_time = 0.0
        self.current_phase_wl: int | None = None
        self.current_phase_vals: list[float] = []
        self.current_phase_times: list[float] = []
        self.latest_rms_660: float | None = None
        self.latest_rms_wl1: float | None = None
        self.latest_rms_time_660: float | None = None
        self.latest_rms_time_wl1: float | None = None
        self.last_pair_time = -1.0

        self.time_660 = deque(maxlen=5000)
        self.data_660 = deque(maxlen=5000)
        self.time_wl1 = deque(maxlen=5000)
        self.data_wl1 = deque(maxlen=5000)

        self.time_hbo = deque(maxlen=3000)
        self.hbo_vals = deque(maxlen=3000)
        self.hbr_vals = deque(maxlen=3000)
        self.time_od = deque(maxlen=3000)
        # 行顺序与 data_analysis samples vstack 一致：wl1(0x01) 在上，660(0x02) 在下
        self.od_wl1_vals = deque(maxlen=3000)
        self.od_660_vals = deque(maxlen=3000)

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
        if wavelength_code not in (WAVELENGTH_660_CODE, WAVELENGTH_940_CODE):
            return

        # 与离线 sliding_window_rms 一致：按波长连续段切段，段结束时用 RMS 代表该段。
        if self.current_phase_wl is None:
            self.current_phase_wl = wavelength_code
        elif wavelength_code != self.current_phase_wl:
            self._finalize_phase()
            self.current_phase_wl = wavelength_code
            self.current_phase_vals.clear()
            self.current_phase_times.clear()

        self.current_phase_vals.append(float(value))
        self.current_phase_times.append(elapsed)

    def _finalize_phase(self):
        if self.current_phase_wl is None or not self.current_phase_vals:
            return

        phase_rms = float(np.sqrt(np.mean(np.square(np.asarray(self.current_phase_vals, dtype=float)))))
        phase_time = float(np.mean(self.current_phase_times))

        if self.current_phase_wl == WAVELENGTH_660_CODE:
            self.latest_rms_660 = phase_rms
            self.latest_rms_time_660 = phase_time
            self.time_660.append(phase_time)
            self.data_660.append(phase_rms)
            if self.ref_660 is None and phase_time <= BASELINE_SECONDS:
                self.baseline_660.append(phase_rms)
        elif self.current_phase_wl == WAVELENGTH_940_CODE:
            self.latest_rms_wl1 = phase_rms
            self.latest_rms_time_wl1 = phase_time
            self.time_wl1.append(phase_time)
            self.data_wl1.append(phase_rms)
            if self.ref_wl1 is None and phase_time <= BASELINE_SECONDS:
                self.baseline_wl1.append(phase_rms)

        # 用串口时间戳判断：满 BASELINE_SECONDS 且有两路数据则固定基线
        if (
            self.ref_660 is None
            and self.ref_wl1 is None
            and phase_time >= BASELINE_SECONDS
            and self.baseline_660
            and self.baseline_wl1
        ):
            self.ref_660 = float(np.mean(self.baseline_660))
            self.ref_wl1 = float(np.mean(self.baseline_wl1))
            self.status.setText(
                f"基线就绪 ref_660={self.ref_660:.0f} "
                f"ref_wl1({int(MBLL_WAVELENGTH_WL1_NM)}nm)={self.ref_wl1:.0f}"
            )

        if (
            self.ref_660 is not None
            and self.ref_wl1 is not None
            and self.latest_rms_660 is not None
            and self.latest_rms_wl1 is not None
            and self.latest_rms_time_660 is not None
            and self.latest_rms_time_wl1 is not None
        ):
            t = (self.latest_rms_time_660 + self.latest_rms_time_wl1) * 0.5
            if t <= self.last_pair_time:
                return
            if (t - self.last_mbll_time) < UPDATE_INTERVAL_S:
                return

            i_660 = max(self.latest_rms_660, 1.0)
            i_wl1 = max(self.latest_rms_wl1, 1.0)
            delta_od_660 = -np.log10(i_660 / self.ref_660)
            delta_od_wl1 = -np.log10(i_wl1 / self.ref_wl1)
            self.time_od.append(t)
            self.od_wl1_vals.append(delta_od_wl1)
            self.od_660_vals.append(delta_od_660)

            delta_od_hist = np.vstack(
                [
                    np.asarray(self.od_wl1_vals, dtype=float),
                    np.asarray(self.od_660_vals, dtype=float),
                ]
            )
            od_times = np.asarray(self.time_od, dtype=float)
            dt = np.median(np.diff(od_times)) if od_times.size >= 2 else np.nan
            fs = 1.0 / dt if np.isfinite(dt) and dt > 0 else 1.0
            delta_od_filt = smart_bandpass(
                delta_od_hist,
                fs,
                lowcut=BP_LOW_HZ,
                highcut=BP_HIGH_HZ,
                order=BP_ORDER,
            )
            delta_od = delta_od_filt[:, -1:].astype(float, copy=False)
            ch_names, ch_wls, ch_dpfs, ch_distances = _channel_info()
            try:
                delta_c, names, types = mbll_hbo_hbr(
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
            self.last_mbll_time = t
            self.last_pair_time = t
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
