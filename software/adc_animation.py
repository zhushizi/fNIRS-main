"""
回放 all_groups.csv 中的单通道原始 ADC 数据。

这个脚本不连串口，只是把已经采好的 CSV 按时间顺序播放出来。
"""

from __future__ import annotations

import collections
import os
import signal
import sys

import pandas as pd
import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets

from config import CHANNEL_NAME, WAVELENGTH_660_CODE, WAVELENGTH_940_CODE


# demo 模式下从 sample_data 读样例，否则读当前目录下的真实输出文件。
DEMO = any(arg.lower() == "demo" for arg in sys.argv[1:])
DATA_DIR = "sample_data" if DEMO else "."
CSV_PATH = os.path.join(DATA_DIR, "all_groups.csv")


def signal_handler(*_args):
    """允许 Ctrl+C 时优雅退出图形界面。"""
    app.quit()


signal.signal(signal.SIGINT, signal_handler)

pg.setConfigOption("antialias", True)
pg.setConfigOption("background", "w")
pg.setConfigOption("foreground", "k")

df = pd.read_csv(CSV_PATH)
n_rows = df.shape[0]

app = QtWidgets.QApplication(sys.argv)
main_window = QtWidgets.QWidget()
main_layout = QtWidgets.QVBoxLayout(main_window)

plot = pg.PlotWidget(title="Single-Channel Raw ADC Animation")
plot.addLegend()
plot.showGrid(x=True, y=True, alpha=0.3)
plot.setLabel("bottom", "Time (s)")
plot.setLabel("left", "ADC Value")
curve_660 = plot.plot(pen=pg.mkPen("r", width=2), name="660nm")
curve_940 = plot.plot(pen=pg.mkPen("g", width=2), name="940nm")
main_layout.addWidget(plot)

status_label = QtWidgets.QLabel(f"Channel: {CHANNEL_NAME}")
main_layout.addWidget(status_label)

data_660 = collections.deque(maxlen=5000)
data_940 = collections.deque(maxlen=5000)
time_660 = collections.deque(maxlen=5000)
time_940 = collections.deque(maxlen=5000)

CURRENT_INDEX = 0


def update():
    """逐行读取 CSV，并按波长分别回放到 660/940 两条曲线上。"""
    global CURRENT_INDEX
    if CURRENT_INDEX >= n_rows:
        timer.stop()
        app.quit()
        return

    row = df.iloc[CURRENT_INDEX]
    wave = int(row["Wavelength"])
    value = float(row[CHANNEL_NAME])
    timestamp = float(row["Time (s)"])
    if wave == WAVELENGTH_660_CODE:
        time_660.append(timestamp)
        data_660.append(value)
    elif wave == WAVELENGTH_940_CODE:
        time_940.append(timestamp)
        data_940.append(value)

    curve_660.setData(list(time_660), list(data_660))
    curve_940.setData(list(time_940), list(data_940))
    CURRENT_INDEX += 1


timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(100)

main_window.show()
sys.exit(app.exec_())
