"""
adc_live.py
==================
该脚本从串口读取数据，并通过 PyQt5 + pyqtgraph 图形界面实时显示。
用于可视化通过 USB 连接设备输出的 ADC 读数。
"""

import sys
import struct
import numpy as np
import serial
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from config import SERIAL_PORT, BAUD_RATE, TIMEOUT

PACKET_SIZE = 64

# 打开串口：参数来自 config.py（端口、波特率、超时）
ser = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=TIMEOUT)

# 定义曲线颜色与图例标签。
trace_colors = ["red", "green", "blue"]
trace_labels = ["Channel 1", "Channel 2", "Channel 3"]

# 解析串口接收到的数据包。
def parse_packet(data):
    """
    将输入数据包解析为结构化数组。
    每个数据包包含 8 组数据，每组 5 个字段：
    - 组 ID
    - Short 值
    - Long1 值
    - Long2 值
    - Emitter 值
    返回形状为 (8, 5) 的二维 numpy 数组：
    每一行对应一组，每一列对应一个字段。
    """
    parsed_data = np.zeros((8, 5), dtype=int)
    for i in range(8):
        # 每组固定 8 字节，因此第 i 组偏移量为 i*8
        offset = i * 8
        group_id = data[offset]
        # 固件按大端序发送 16 位 ADC 值
        raw_short = struct.unpack('>H', data[offset+1:offset+3])[0]
        raw_long1 = struct.unpack('>H', data[offset+3:offset+5])[0]
        raw_long2 = struct.unpack('>H', data[offset+5:offset+7])[0]
        emitter   = data[offset+7]
        # 输出格式：[组ID, Short, Long1, Long2, 发射器状态]
        parsed_data[i] = [group_id, raw_short, raw_long1, raw_long2, emitter]
    return parsed_data

class SerialReaderThread(QtCore.QThread):
    """
    串口读取线程。
    持续从串口读取字节流；当接收到完整数据包后，
    发出包含解析结果的 Qt 信号。
    """
    newData = QtCore.pyqtSignal(np.ndarray)
    def __init__(self, parent=None):
        super().__init__(parent)
        self.running = True
    def run(self):
        """
        持续读取串口并拆分为完整数据包。
        每解析出一帧数据就发信号给主线程。
        """
        read_buffer = b""
        while self.running:
            # 一次多读一些字节，减少系统调用开销
            chunk = ser.read(256)
            if chunk:
                read_buffer += chunk
                # 只要缓冲区够 1 帧（64B）就持续拆包
                while len(read_buffer) >= PACKET_SIZE:
                    packet = read_buffer[:PACKET_SIZE]
                    read_buffer = read_buffer[PACKET_SIZE:]
                    arr_8x5 = parse_packet(packet)
                    # 通过 Qt 信号把新数据发给主线程（线程安全）
                    self.newData.emit(arr_8x5)
            QtCore.QThread.msleep(1)  # 避免忙等
    def stop(self):
        """
        停止线程读取循环。
        """
        self.running = False


class MainWindow(QtWidgets.QWidget):
    """
    主窗口类：用于显示 ADC 实时曲线。
    基于 PyQtGraph 绘制 8 组 3 通道波形。
    """
    def __init__(self):
        super().__init__()
        # 设置 PyQtGraph 全局显示样式
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')
        self.setWindowTitle("ADC Live Mode")

        self.max_len = 0

        # 主纵向布局
        main_layout = QtWidgets.QVBoxLayout(self)
        # 顶部横向布局：图例 + 重置按钮
        top_layout = QtWidgets.QHBoxLayout()
        top_layout.addStretch()
        # 为每个通道添加图例项
        for color, label in zip(trace_colors, trace_labels):
            legend_item = QtWidgets.QWidget()
            legend_layout = QtWidgets.QHBoxLayout(legend_item)
            legend_layout.setContentsMargins(0, 0, 0, 0)
            square = QtWidgets.QLabel()
            square.setFixedSize(15, 15)
            square.setStyleSheet(f"background-color: {color}; border: 1px solid black;")
            text_label = QtWidgets.QLabel(label)
            legend_layout.addWidget(square)
            legend_layout.addWidget(text_label)
            top_layout.addWidget(legend_item)

        # 在同一行加入“Reset All”按钮
        btn_reset = QtWidgets.QPushButton("Reset All")
        btn_reset.clicked.connect(self.reset_plots)
        top_layout.addWidget(btn_reset)
        top_layout.addStretch()

        # 将顶部布局添加到主布局
        main_layout.addLayout(top_layout)

        # 数据缓存：8组 * 3通道
        self.max_points = 3000
        self.data = [[[ ] for _ in range(3)] for _ in range(8)]

        # 创建绘图控件
        self.plot_widget = pg.GraphicsLayoutWidget(title="Live ADC Readings")
        self.plot_widget.resize(1200, 800)
        main_layout.addWidget(self.plot_widget)

        # 创建分组子图与曲线对象
        self.plots = []
        self.curves = []
        self.pg_trace_colors = [pg.mkPen('r', width=2),
                                 pg.mkPen('g', width=2),
                                 pg.mkPen('b', width=2)]
        for g in range(8):
            p = self.plot_widget.addPlot()
            p.setTitle(f"Sensor Group {g+1}", size="16pt")
            p.showGrid(x=True, y=True)
            p.setLabel('bottom', 'Time (ms)')
            p.setLabel('left', 'ADC Value')
            p.setYRange(0, 4095, padding=0)
            p.disableAutoRange()
            group_curves = []
            for ch_idx in range(3):
                c = p.plot(pen=self.pg_trace_colors[ch_idx])
                group_curves.append(c)
            self.plots.append(p)
            self.curves.append(group_curves)
            if g % 2 == 1:
                self.plot_widget.nextRow()

        # 定时刷新绘图
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(50)

    @QtCore.pyqtSlot(np.ndarray)
    def on_new_data(self, arr_8x5):
        """
        槽函数：处理串口线程送来的新数据。
        """
        # arr_8x5: 8组 x 5字段；这里只取 3 路 ADC（Short/Long1/Long2）
        for g in range(8):
            self.data[g][0].append(arr_8x5[g][1])
            self.data[g][1].append(arr_8x5[g][2])
            self.data[g][2].append(arr_8x5[g][3])
            for ch_idx in range(3):
                if len(self.data[g][ch_idx]) > self.max_points:
                    self.data[g][ch_idx].pop(0)

    def update_plots(self):
        """
        使用当前缓存数据刷新曲线。
        """
        # 定时器周期性刷新曲线；x 轴使用样本索引，y 轴为 ADC 值
        for g in range(8):
            for ch_idx in range(3):
                d = self.data[g][ch_idx]
                if d:
                    self.curves[g][ch_idx].setData(range(len(d)), d)
            self.plots[g].setYRange(0, 4095, padding=0)

    def reset_plots(self):
        """
        清空所有曲线与缓存数据。
        """
        self.data = [[[ ] for _ in range(3)] for _ in range(8)]
        for g in range(8):
            for ch_idx in range(3):
                self.curves[g][ch_idx].setData([])

def main():
    """
    程序入口：初始化 GUI 并启动串口读取线程。
    将读取线程信号连接到窗口数据处理槽函数。
    """
    # 清空串口输入缓冲，避免历史残留数据影响显示
    ser.reset_input_buffer()
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.showFullScreen()  # 全屏显示界面
    reader_thread = SerialReaderThread()
    reader_thread.newData.connect(window.on_new_data)
    reader_thread.start()
    # 关闭应用时先停线程再退出，避免串口占用/崩溃
    app.aboutToQuit.connect(lambda: (reader_thread.stop(), reader_thread.wait()))
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
