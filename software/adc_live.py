"""
单通道实时 ADC 曲线查看器。

这个脚本的职责很单纯：
1. 给下位机发启动命令
2. 持续读取 0x02 数据帧
3. 把 660nm / 940nm 分成两条曲线显示
"""

from __future__ import annotations

import sys
import time
from collections import deque

import pyqtgraph as pg
import serial
from PyQt5 import QtCore, QtWidgets

from config import (
    ACK_TIMEOUT_SECONDS,
    BAUD_RATE,
    DEFAULT_INTENSITY_MA,
    MAX_RETRIES,
    SERIAL_PORT,
    TIMEOUT,
    WAVELENGTH_660_CODE,
    WAVELENGTH_940_CODE,
)
from protocol import FrameReader, build_command_frame, parse_data_frame, send_frame_with_ack


def open_serial() -> serial.Serial:
    """按 config.py 中的参数打开串口。"""
    return serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=TIMEOUT)


class SerialReaderThread(QtCore.QThread):
    """后台串口读取线程，避免 GUI 主线程被串口阻塞。"""
    newData = QtCore.pyqtSignal(float, int, int)

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
            f"sensor={sample.sensor_id}",
            f"wavelength_code=0x{sample.wavelength_code:02X}",
            f"value={sample.value}",
        )
        self.newData.emit(elapsed, sample.wavelength_code, sample.value)

    def _start_stream(self) -> bool:
        """
        启动采集。

        优先等显式 ACK；如果 ACK 因时序问题晚到，但 0x02 数据已经开始流动，
        也视为“采集已经启动成功”，避免线程直接退出。
        """
        command = build_command_frame(True, DEFAULT_INTENSITY_MA)
        startup_timeout = max(ACK_TIMEOUT_SECONDS * 10, 0.2)

        for attempt in range(MAX_RETRIES + 1):
            print(f"[adc_live] Sending start command frame (attempt {attempt + 1})...")
            self.ser.write(command)

            deadline = time.time() + startup_timeout
            while time.time() < deadline:
                frame = self.reader.read_frame(timeout_seconds=min(TIMEOUT, max(0.01, deadline - time.time())))
                if frame is None:
                    continue
                if frame.frame_type == 0x03:
                    print("[adc_live] Start command ACK success. Begin reading data frames.")
                    return True
                if frame.frame_type == 0x02:
                    # If data is already flowing, treat the stream as active even if
                    # the explicit ACK was missed or arrived out of order.
                    sample = parse_data_frame(frame)
                    print("[adc_live] Stream became active before explicit ACK.")
                    self._emit_sample(sample)
                    return True
                print(f"[adc_live] Ignored startup frame type: 0x{frame.frame_type:02X}")

        print("[adc_live] Start command ACK timeout. Reader thread exits.")
        return False

    def run(self):
        """线程主循环：先启动流，再持续读取数据帧。"""
        started = self._start_stream()
        if not started:
            return

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
            send_frame_with_ack(
                self.ser,
                self.reader,
                build_command_frame(False, DEFAULT_INTENSITY_MA),
            )
        except Exception:
            pass

    def stop(self):
        self.running = False


class MainWindow(QtWidgets.QWidget):
    """简单的双曲线窗口：红色 660nm，绿色 940nm。"""
    def __init__(self):
        super().__init__()
        pg.setConfigOption("background", "w")
        pg.setConfigOption("foreground", "k")
        self.setWindowTitle("Single-Channel ADC Live Mode")

        # 固定只显示最近 10 秒。
        self.window_seconds = 10.0
        self.max_points = 3000
        self.data_660 = deque(maxlen=self.max_points)
        self.data_940 = deque(maxlen=self.max_points)
        self.time_660 = deque(maxlen=self.max_points)
        self.time_940 = deque(maxlen=self.max_points)

        main_layout = QtWidgets.QVBoxLayout(self)
        top_layout = QtWidgets.QHBoxLayout()
        top_layout.addStretch()
        for color, label in (("red", "660nm"), ("green", "940nm")):
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

        self.plot_widget = pg.PlotWidget(title="S1_D1 Live ADC Readings")
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.setLabel("bottom", "Time (s)")
        self.plot_widget.setLabel("left", "ADC Value")
        self.plot_widget.addLegend()
        self.plot_widget.setXRange(0, self.window_seconds, padding=0)
        self.curve_660 = self.plot_widget.plot(pen=pg.mkPen("r", width=2), name="660nm")
        self.curve_940 = self.plot_widget.plot(pen=pg.mkPen("g", width=2), name="940nm")
        main_layout.addWidget(self.plot_widget)

        self.status_label = QtWidgets.QLabel("Waiting for data...")
        main_layout.addWidget(self.status_label)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(50)

    @QtCore.pyqtSlot(float, int, int)
    def on_new_data(self, elapsed: float, wavelength_code: int, value: int):
        """按波长把点分发到两条曲线各自的缓存里。"""
        if wavelength_code == WAVELENGTH_660_CODE:
            self.time_660.append(elapsed)
            self.data_660.append(value)
            wave_label = "660nm"
        elif wavelength_code == WAVELENGTH_940_CODE:
            self.time_940.append(elapsed)
            self.data_940.append(value)
            wave_label = "940nm"
        else:
            wave_label = f"Unknown({wavelength_code})"
            print(f"[adc_live] Unknown wavelength code: 0x{wavelength_code:02X}")

        self.status_label.setText(f"Latest: {wave_label} value={value}")

    def update_plots(self):
        """定时刷新曲线，并把 X 轴滚动到最近 10 秒窗口。"""
        if self.data_660:
            self.curve_660.setData(list(self.time_660), list(self.data_660))
        if self.data_940:
            self.curve_940.setData(list(self.time_940), list(self.data_940))

        latest_time = 0.0
        if self.time_660:
            latest_time = max(latest_time, self.time_660[-1])
        if self.time_940:
            latest_time = max(latest_time, self.time_940[-1])

        x_min = max(0.0, latest_time - self.window_seconds)
        x_max = max(self.window_seconds, latest_time)
        self.plot_widget.setXRange(x_min, x_max, padding=0)


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
