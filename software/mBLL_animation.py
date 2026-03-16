"""
回放 processed_output.csv 中的单通道 MBLL 结果。

这个脚本适合在离线场景下查看最终 HbO / HbR 曲线。
"""

from __future__ import annotations

import collections
import os
import signal
import sys

import pandas as pd
import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets


# demo 模式下从 sample_data 读取样例文件，否则读取当前目录输出。
DEMO = any(arg.lower() == "demo" for arg in sys.argv[1:])
DATA_DIR = "sample_data" if DEMO else "."
CSV_PATH = os.path.join(DATA_DIR, "processed_output.csv")


def signal_handler(*_args):
    """允许 Ctrl+C 时优雅退出图形界面。"""
    app.quit()


signal.signal(signal.SIGINT, signal_handler)

pg.setConfigOption("antialias", True)
pg.setConfigOption("background", "w")
pg.setConfigOption("foreground", "k")

df = pd.read_csv(CSV_PATH)
n_rows = df.shape[0]
value_columns = [col for col in df.columns if col != "Time"]

app = QtWidgets.QApplication(sys.argv)
main_window = QtWidgets.QWidget()
main_layout = QtWidgets.QVBoxLayout(main_window)

plot = pg.PlotWidget(title="Single-Channel mBLL Animation")
plot.addLegend()
plot.showGrid(x=True, y=True, alpha=0.3)
plot.setLabel("bottom", "Time (s)")
plot.setLabel("left", "Concentration")
main_layout.addWidget(plot)

colors = ["r", "g", "b", "m", "c", "y"]
curves = []
series = []
times = collections.deque(maxlen=5000)
for idx, column in enumerate(value_columns):
    curves.append(plot.plot(pen=pg.mkPen(colors[idx % len(colors)], width=2), name=column))
    series.append(collections.deque(maxlen=5000))

CURRENT_INDEX = 0


def update():
    """按行回放处理后的浓度结果。"""
    global CURRENT_INDEX
    if CURRENT_INDEX >= n_rows:
        timer.stop()
        app.quit()
        return

    row = df.iloc[CURRENT_INDEX]
    times.append(float(row["Time"]))
    for idx, column in enumerate(value_columns):
        series[idx].append(float(row[column]))
        curves[idx].setData(list(times), list(series[idx]))
    CURRENT_INDEX += 1


timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(100)

main_window.show()
sys.exit(app.exec_())
