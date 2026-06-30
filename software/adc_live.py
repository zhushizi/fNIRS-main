"""
双接收源实时 ADC 曲线查看器（默认显示 S1_D1@850nm 与 S1_D2@850nm）。
"""

from __future__ import annotations

import sys
import time
from collections import deque

import numpy as np
import pyqtgraph as pg
import serial
from PyQt5 import QtCore, QtWidgets
from scipy.signal import butter, lfilter, sosfilt

from config import (
    ACK_TIMEOUT_SECONDS,
    BAUD_RATE,
    DEFAULT_INTENSITY_MA,
    DETECTOR_CHANNELS,
    MAX_RETRIES,
    SERIAL_PORT,
    TIMEOUT,
    WAVELENGTH_850_CODE,
    WAVELENGTH_OFF_CODE,
)
from protocol import FrameReader, build_command_frame, parse_data_frame, send_frame_with_ack


def open_serial() -> serial.Serial:
    """按 config.py 中的参数打开串口。"""
    return serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=TIMEOUT)


def frame_to_hex(frame_bytes: bytes) -> str:
    """把二进制帧转成十六进制字符串，便于调试打印。"""
    return " ".join(f"{b:02X}" for b in frame_bytes)


class SerialReaderThread(QtCore.QThread):
    """后台串口读取线程，避免 GUI 主线程被串口阻塞。"""
    newData = QtCore.pyqtSignal(float, int, int, int)

    def __init__(self, ser: serial.Serial, parent=None):
        super().__init__(parent)
        self.ser = ser
        self.reader = FrameReader(ser)
        self.running = True
        self.start_time = time.time()

    def _emit_sample(self, sample):
        """把协议层的 DataSample 转成 GUI 能直接消费的信号。"""
        elapsed = time.time() - self.start_time
        print(
            "[adc_live] Data frame received:",
            f"detector={sample.detector_code}",
            f"wavelength_code=0x{sample.wavelength_code:02X}",
            f"value={sample.value}",
        )
        self.newData.emit(elapsed, sample.detector_code, sample.wavelength_code, sample.value)

    def _start_stream(self) -> bool:
        """
        启动采集。

        当前固件版本不返回 ACK。
        这里只检查是否开始收到 0x02 数据帧，收到即视为启动成功。
        """
        command = build_command_frame(True, DEFAULT_INTENSITY_MA)
        startup_timeout = max(ACK_TIMEOUT_SECONDS * 10, 0.2)

        for attempt in range(MAX_RETRIES + 1):
            print(f"[adc_live] Sending start command frame (attempt {attempt + 1})...")
            print(f"[adc_live] TX start raw={frame_to_hex(command)}")
            self.ser.write(command)

            deadline = time.time() + startup_timeout
            while time.time() < deadline:
                frame = self.reader.read_frame(timeout_seconds=min(TIMEOUT, max(0.01, deadline - time.time())))
                if frame is None:
                    continue
                if frame.frame_type == 0x02:
                    sample = parse_data_frame(frame)
                    print("[adc_live] Stream became active.")
                    self._emit_sample(sample)
                    return True
                print(f"[adc_live] Ignored startup non-data frame type: 0x{frame.frame_type:02X}")

        print("[adc_live] No data frame received after start command. Reader thread exits.")
        return False

    def run(self):
        """线程主循环：先启动流，再持续读取数据帧。"""
        started = self._start_stream()
        if not started:
            print("[adc_live] Start command sent (max attempts reached). Keep waiting for data frames...")

        while self.running:
            frame = self.reader.read_frame(timeout_seconds=TIMEOUT)
            if frame is None:
                continue
            if frame.frame_type != 0x02:
                print(f"[adc_live] Ignored frame type: 0x{frame.frame_type:02X}")
                continue

            sample = parse_data_frame(frame)
            self._emit_sample(sample)

        try:
            print("[adc_live] Sending stop command frame...")
            stop_command = build_command_frame(False, 0x00)
            print(f"[adc_live] TX stop raw={frame_to_hex(stop_command)}")
            send_frame_with_ack(
                self.ser,
                self.reader,
                stop_command,
            )
        except Exception:
            pass

    def stop(self):
        self.running = False


class MainWindow(QtWidgets.QWidget):
    """双接收 @850nm 实时曲线（S1_D1 / S1_D2）。"""
    def __init__(self):
        super().__init__()
        pg.setConfigOption("background", "w")
        pg.setConfigOption("foreground", "k")
        self.setWindowTitle("双接收 ADC 实时监视 (850nm)")

        self.window_seconds = 10.0
        self.max_points = 3000
        d1_name = DETECTOR_CHANNELS[0].name
        d2_name = DETECTOR_CHANNELS[1].name
        self.data_d1 = deque(maxlen=self.max_points)
        self.data_d2 = deque(maxlen=self.max_points)
        self.time_d1 = deque(maxlen=self.max_points)
        self.time_d2 = deque(maxlen=self.max_points)

        main_layout = QtWidgets.QVBoxLayout(self)
        top_layout = QtWidgets.QHBoxLayout()
        top_layout.addStretch()
        for color, label in (("red", f"{d1_name}@850"), ("green", f"{d2_name}@850")):
            item = QtWidgets.QWidget()
            item_layout = QtWidgets.QHBoxLayout(item)
            item_layout.setContentsMargins(0, 0, 0, 0)
            square = QtWidgets.QLabel()
            square.setFixedSize(15, 15)
            square.setStyleSheet(f"background-color: {color}; border: 1px solid black;")
            text_label = QtWidgets.QLabel(label)
            item_layout.addWidget(square)
            item_layout.addWidget(text_label)
            top_layout.addWidget(item)
        top_layout.addStretch()
        main_layout.addLayout(top_layout)

        # 滤波器控制区：None / Lowpass / Bandpass / Threshold
        filter_layout = QtWidgets.QGridLayout()
        filter_layout.setHorizontalSpacing(8)
        filter_layout.setVerticalSpacing(6)

        self.filter_mode_combo = QtWidgets.QComboBox()
        self.filter_mode_combo.addItems(["None", "Lowpass", "Bandpass", "Threshold"])
        filter_layout.addWidget(QtWidgets.QLabel("滤波模式"), 0, 0)
        filter_layout.addWidget(self.filter_mode_combo, 0, 1)

        self.lowpass_cutoff_spin = QtWidgets.QDoubleSpinBox()
        self.lowpass_cutoff_spin.setRange(0.01, 1000.0)
        self.lowpass_cutoff_spin.setDecimals(3)
        self.lowpass_cutoff_spin.setValue(1.0)
        self.lowpass_cutoff_spin.setSuffix(" Hz")
        filter_layout.addWidget(QtWidgets.QLabel("低通截止"), 0, 2)
        filter_layout.addWidget(self.lowpass_cutoff_spin, 0, 3)

        self.band_low_spin = QtWidgets.QDoubleSpinBox()
        self.band_low_spin.setRange(0.001, 1000.0)
        self.band_low_spin.setDecimals(3)
        self.band_low_spin.setValue(0.05)
        self.band_low_spin.setSuffix(" Hz")
        filter_layout.addWidget(QtWidgets.QLabel("带通下限"), 0, 4)
        filter_layout.addWidget(self.band_low_spin, 0, 5)

        self.band_high_spin = QtWidgets.QDoubleSpinBox()
        self.band_high_spin.setRange(0.001, 1000.0)
        self.band_high_spin.setDecimals(3)
        self.band_high_spin.setValue(0.1)
        self.band_high_spin.setSuffix(" Hz")
        filter_layout.addWidget(QtWidgets.QLabel("带通上限"), 0, 6)
        filter_layout.addWidget(self.band_high_spin, 0, 7)

        self.filter_order_spin = QtWidgets.QSpinBox()
        self.filter_order_spin.setRange(1, 12)
        self.filter_order_spin.setValue(4)
        filter_layout.addWidget(QtWidgets.QLabel("阶数"), 0, 8)
        filter_layout.addWidget(self.filter_order_spin, 0, 9)

        self.th_lower_spin = QtWidgets.QDoubleSpinBox()
        self.th_lower_spin.setRange(-1e9, 1e9)
        self.th_lower_spin.setDecimals(1)
        self.th_lower_spin.setValue(50000.0)
        filter_layout.addWidget(QtWidgets.QLabel("阈值下限"), 1, 0)
        filter_layout.addWidget(self.th_lower_spin, 1, 1)

        self.th_upper_spin = QtWidgets.QDoubleSpinBox()
        self.th_upper_spin.setRange(-1e9, 1e9)
        self.th_upper_spin.setDecimals(1)
        self.th_upper_spin.setValue(300000.0)
        filter_layout.addWidget(QtWidgets.QLabel("阈值上限"), 1, 2)
        filter_layout.addWidget(self.th_upper_spin, 1, 3)

        self.th_replacement_spin = QtWidgets.QDoubleSpinBox()
        self.th_replacement_spin.setRange(-1e9, 1e9)
        self.th_replacement_spin.setDecimals(1)
        self.th_replacement_spin.setValue(170000.0)
        filter_layout.addWidget(QtWidgets.QLabel("替换值"), 1, 4)
        filter_layout.addWidget(self.th_replacement_spin, 1, 5)

        main_layout.addLayout(filter_layout)

        d1_name = DETECTOR_CHANNELS[0].name
        d2_name = DETECTOR_CHANNELS[1].name
        self.plot_widget = pg.PlotWidget(title="双接收 ADC 实时波形 (850nm)")
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.setLabel("bottom", "时间 (s)")
        self.plot_widget.setLabel("left", "ADC 数值")
        self.plot_widget.addLegend()
        self.plot_widget.setXRange(0, self.window_seconds, padding=0)
        self.curve_d1 = self.plot_widget.plot(pen=pg.mkPen("r", width=2), name=f"{d1_name}@850")
        self.curve_d2 = self.plot_widget.plot(pen=pg.mkPen("g", width=2), name=f"{d2_name}@850")
        main_layout.addWidget(self.plot_widget)

        self.fft_min_samples = 64
        self.spectrum_widget = pg.PlotWidget(title="ADC 频谱（FFT 幅值）")
        self.spectrum_widget.showGrid(x=True, y=True)
        self.spectrum_widget.setLabel("bottom", "频率 (Hz)")
        self.spectrum_widget.setLabel("left", "幅值")
        self.spectrum_widget.addLegend()
        self.spectrum_curve_d1 = self.spectrum_widget.plot(
            pen=pg.mkPen("r", width=2), name=f"{d1_name}@850"
        )
        self.spectrum_curve_d2 = self.spectrum_widget.plot(
            pen=pg.mkPen("g", width=2), name=f"{d2_name}@850"
        )
        main_layout.addWidget(self.spectrum_widget)

        self.status_label = QtWidgets.QLabel("等待数据...")
        main_layout.addWidget(self.status_label)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(50)

    @staticmethod
    def _spectrum_from_timeseries(times, values, min_samples: int):
        """
        由非均匀时间戳估计平均采样间隔，对去直流后的序列加 Hann 窗后做 rFFT。
        返回 (freq_hz, magnitude) ；样本不足或无效时返回 (None, None)。
        """
        if len(times) < min_samples or len(values) < min_samples:
            return None, None
        t = np.asarray(times, dtype=float)
        y = np.asarray(values, dtype=float)
        n = min(len(t), len(y))
        t, y = t[-n:], y[-n:]
        dt = np.diff(t)
        if dt.size == 0:
            return None, None
        mean_dt = float(np.mean(dt))
        if mean_dt <= 0 or not np.isfinite(mean_dt):
            return None, None
        fs = 1.0 / mean_dt
        y = y - np.mean(y)
        win = np.hanning(len(y))
        y_w = y * win
        spec = np.fft.rfft(y_w)
        freq = np.fft.rfftfreq(len(y), d=1.0 / fs)
        # 单边谱幅度归一化（与窗能量无关的直观量级：除以 n，非 DC/Nyquist 乘 2）
        mag = np.abs(spec)
        if len(mag) > 0:
            mag = mag * (2.0 / len(y))
            mag[0] *= 0.5
            if len(mag) > 1 and len(y) % 2 == 0:
                mag[-1] *= 0.5
        return freq, mag

    @staticmethod
    def _estimate_fs(times: np.ndarray) -> float | None:
        if len(times) < 2:
            return None
        dt = np.diff(times)
        if dt.size == 0:
            return None
        mean_dt = float(np.mean(dt))
        if mean_dt <= 0 or not np.isfinite(mean_dt):
            return None
        return 1.0 / mean_dt

    def _apply_filter(self, times, values):
        mode = self.filter_mode_combo.currentText()
        y = np.asarray(values, dtype=float)
        if y.size == 0:
            return y
        if mode == "None":
            return y

        if mode == "Threshold":
            lower = float(self.th_lower_spin.value())
            upper = float(self.th_upper_spin.value())
            replacement = float(self.th_replacement_spin.value())
            if lower > upper:
                lower, upper = upper, lower
            return np.where((y < lower) | (y > upper), replacement, y)

        t = np.asarray(times, dtype=float)
        fs = self._estimate_fs(t)
        if fs is None:
            return y
        nyquist = 0.5 * fs
        order = int(self.filter_order_spin.value())
        min_len = max(16, 3 * (order + 1) + 1)
        if y.size < min_len:
            return y

        try:
            if mode == "Lowpass":
                cutoff = float(self.lowpass_cutoff_spin.value())
                if cutoff <= 0 or cutoff >= nyquist:
                    return y
                b, a = butter(order, cutoff / nyquist, btype="low", analog=False)
                return lfilter(b, a, y)

            if mode == "Bandpass":
                lowcut = float(self.band_low_spin.value())
                highcut = float(self.band_high_spin.value())
                if lowcut > highcut:
                    lowcut, highcut = highcut, lowcut
                if lowcut <= 0 or highcut >= nyquist or lowcut >= highcut:
                    return y
                sos = butter(
                    order,
                    [lowcut / nyquist, highcut / nyquist],
                    btype="band",
                    output="sos",
                )
                return sosfilt(sos, y)
        except Exception as exc:
            print(f"[adc_live] Filter failed ({mode}): {exc}")
        return y

    @QtCore.pyqtSlot(float, int, int, int)
    def on_new_data(self, elapsed: float, detector_code: int, wavelength_code: int, value: int):
        """按接收源把 850nm 点分发到两条曲线。"""
        d1_code = DETECTOR_CHANNELS[0].code
        d2_code = DETECTOR_CHANNELS[1].code
        if wavelength_code != WAVELENGTH_850_CODE:
            if wavelength_code == WAVELENGTH_OFF_CODE:
                wave_label = "OFF"
            else:
                wave_label = f"wl=0x{wavelength_code:02X}"
            self.status_label.setText(f"最新数据：{wave_label} det={detector_code} 数值={value}")
            return

        if detector_code == d1_code:
            self.time_d1.append(elapsed)
            self.data_d1.append(value)
            wave_label = f"{DETECTOR_CHANNELS[0].name}@850nm"
        elif detector_code == d2_code:
            self.time_d2.append(elapsed)
            self.data_d2.append(value)
            wave_label = f"{DETECTOR_CHANNELS[1].name}@850nm"
        else:
            wave_label = f"UnknownDet({detector_code})@850nm"

        self.status_label.setText(f"最新数据：{wave_label} 数值={value}")

    def update_plots(self):
        """定时刷新曲线，并把 X 轴滚动到最近 10 秒窗口。"""
        if self.data_d1:
            t1 = list(self.time_d1)
            y1 = self._apply_filter(t1, list(self.data_d1))
            self.curve_d1.setData(t1, y1)
        if self.data_d2:
            t2 = list(self.time_d2)
            y2 = self._apply_filter(t2, list(self.data_d2))
            self.curve_d2.setData(t2, y2)

        latest_time = 0.0
        if self.time_d1:
            latest_time = max(latest_time, self.time_d1[-1])
        if self.time_d2:
            latest_time = max(latest_time, self.time_d2[-1])

        x_min = max(0.0, latest_time - self.window_seconds)
        x_max = max(self.window_seconds, latest_time)
        self.plot_widget.setXRange(x_min, x_max, padding=0)

        y1_fft = self._apply_filter(list(self.time_d1), list(self.data_d1)) if self.data_d1 else []
        y2_fft = self._apply_filter(list(self.time_d2), list(self.data_d2)) if self.data_d2 else []
        f1, m1 = self._spectrum_from_timeseries(self.time_d1, y1_fft, self.fft_min_samples)
        f2, m2 = self._spectrum_from_timeseries(self.time_d2, y2_fft, self.fft_min_samples)
        if f1 is not None and m1 is not None:
            self.spectrum_curve_d1.setData(f1, m1)
        else:
            self.spectrum_curve_d1.clear()
        if f2 is not None and m2 is not None:
            self.spectrum_curve_d2.setData(f2, m2)
        else:
            self.spectrum_curve_d2.clear()

        mode_map = {
            "None": "不滤波",
            "Lowpass": "低通",
            "Bandpass": "带通",
            "Threshold": "阈值",
        }
        mode_cn = mode_map.get(self.filter_mode_combo.currentText(), self.filter_mode_combo.currentText())
        self.status_label.setText(f"滤波模式={mode_cn} | 当前窗口={self.window_seconds:.1f}s")


def main():
    """程序入口：创建窗口并启动串口线程。"""
    print(f"[adc_live] Opening serial port {SERIAL_PORT} @ {BAUD_RATE} ...")
    ser = open_serial()
    ser.reset_input_buffer()
    print("[adc_live] Serial input buffer cleared.")

    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()

    reader_thread = SerialReaderThread(ser)
    reader_thread.newData.connect(window.on_new_data)
    reader_thread.start()

    def cleanup():
        reader_thread.stop()
        reader_thread.wait(2000)
        ser.close()

    app.aboutToQuit.connect(cleanup)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
