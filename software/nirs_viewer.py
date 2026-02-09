"""
nirs_viewer.py
=============
UI tool to read and visualize .nirs (MATLAB) or CSV datasets.

Supported .nirs variables:
  - t: time vector
  - d: data matrix (time x channels or channels x time)

Supported CSV format:
  - Time or Time (s) column
  - Other columns treated as channels
"""

from __future__ import annotations

import os
from typing import List, Tuple

import numpy as np
import pandas as pd
import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets
from scipy.io import loadmat


def _map_type_code(code: int) -> str:
    if code == 1:
        return "HbO"
    if code == 2:
        return "HbR"
    if code == 3:
        return "HbT"
    return "Unknown"


def _infer_nirs_types(mat: dict, n_channels: int) -> List[str]:
    sd = mat.get("SD")
    if sd is None:
        return ["Unknown"] * n_channels

    meas_list = None
    if hasattr(sd, "MeasList"):
        meas_list = sd.MeasList
    elif hasattr(sd, "ML"):
        meas_list = sd.ML

    if meas_list is None:
        return ["Unknown"] * n_channels

    meas_list = np.asarray(meas_list)
    if meas_list.ndim != 2 or meas_list.shape[0] < n_channels:
        return ["Unknown"] * n_channels

    if meas_list.shape[1] >= 4:
        return [_map_type_code(int(code)) for code in meas_list[:n_channels, 3]]

    return ["Unknown"] * n_channels


def _extract_nirs_pairs(mat: dict) -> List[Tuple[int, int, str]]:
    sd = mat.get("SD")
    if sd is None:
        return []

    meas_list = None
    if hasattr(sd, "MeasList"):
        meas_list = sd.MeasList
    elif hasattr(sd, "ML"):
        meas_list = sd.ML

    if meas_list is None:
        return []

    meas_list = np.asarray(meas_list)
    if meas_list.ndim != 2 or meas_list.shape[1] < 4:
        return []

    pairs: List[Tuple[int, int, str]] = []
    for row in meas_list:
        src = int(row[0])
        det = int(row[1])
        meas_type = _map_type_code(int(row[3]))
        pairs.append((src, det, meas_type))
    return pairs


def _infer_physical_channels(mat: dict) -> int | None:
    sd = mat.get("SD")
    if sd is None:
        return None

    meas_list = None
    if hasattr(sd, "MeasList"):
        meas_list = sd.MeasList
    elif hasattr(sd, "ML"):
        meas_list = sd.ML

    if meas_list is None:
        return None

    meas_list = np.asarray(meas_list)
    if meas_list.ndim != 2 or meas_list.shape[1] < 2:
        return None

    pairs = {(int(row[0]), int(row[1])) for row in meas_list}
    return len(pairs)


def load_nirs(
    path: str,
) -> Tuple[np.ndarray, np.ndarray, List[str], str, int | None, List[Tuple[int, int, str]]]:
    mat = loadmat(path, squeeze_me=True, struct_as_record=False)
    if "t" not in mat or "d" not in mat:
        raise ValueError("未找到变量 t 或 d（.nirs 标准字段）。")

    t = np.asarray(mat["t"]).reshape(-1)
    data = np.asarray(mat["d"])

    if data.ndim != 2:
        raise ValueError("d 不是二维矩阵。")

    if data.shape[0] == t.size:
        data_t = data
    elif data.shape[1] == t.size:
        data_t = data.T
    else:
        raise ValueError("d 的维度与 t 不匹配。")

    types = _infer_nirs_types(mat, data_t.shape[1])
    channel_names = [
        f"{types[i]} Ch {i + 1}" if types[i] != "Unknown" else f"Ch {i + 1}"
        for i in range(data_t.shape[1])
    ]
    type_label = _summarize_types(types)
    physical_count = _infer_physical_channels(mat)
    pairs = _extract_nirs_pairs(mat)
    return t, data_t, channel_names, type_label, physical_count, pairs


def _summarize_types(types: List[str]) -> str:
    unique = sorted({t for t in types if t != "Unknown"})
    if not unique:
        return "未知"
    if len(unique) == 1:
        return unique[0]
    return "/".join(unique)


def load_csv(
    path: str,
) -> Tuple[np.ndarray, np.ndarray, List[str], str, int | None, List[Tuple[int, int, str]]]:
    df = pd.read_csv(path)
    time_col = "Time"
    if "Time (s)" in df.columns:
        time_col = "Time (s)"
    elif "Time" not in df.columns:
        raise ValueError("CSV 缺少 Time 或 Time (s) 列。")

    t = df[time_col].to_numpy()
    channel_cols = [c for c in df.columns if c != time_col]
    if not channel_cols:
        raise ValueError("CSV 没有可视化通道列。")

    data_t = df[channel_cols].to_numpy()
    type_label = _summarize_types(_infer_csv_types(channel_cols))
    return t, data_t, channel_cols, type_label, None, []


def _infer_csv_types(columns: List[str]) -> List[str]:
    types: List[str] = []
    for col in columns:
        lowered = col.lower()
        if lowered.endswith("_hbo"):
            types.append("HbO")
        elif lowered.endswith("_hbr"):
            types.append("HbR")
        elif lowered.endswith("_hbt"):
            types.append("HbT")
        else:
            types.append("Unknown")
    return types


class NirsViewer(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("NIRS 数据可视化")
        self.resize(1200, 700)

        self._time: np.ndarray | None = None
        self._data: np.ndarray | None = None
        self._channel_names: List[str] = []
        self._data_type_label: str = "未知"
        self._physical_channels: int | None = None
        self._pair_info: List[Tuple[int, int, str]] = []

        self._build_ui()

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(central)

        controls = QtWidgets.QWidget()
        controls_layout = QtWidgets.QVBoxLayout(controls)
        controls_layout.setContentsMargins(10, 10, 10, 10)

        self.path_label = QtWidgets.QLabel("未选择文件")
        self.path_label.setWordWrap(True)
        open_btn = QtWidgets.QPushButton("打开文件")
        open_btn.clicked.connect(self._open_file)

        self.info_label = QtWidgets.QLabel("通道: 0 | 采样点: 0")
        self.info_label.setWordWrap(True)
        self.type_label = QtWidgets.QLabel("数据类型：未知")
        self.type_label.setWordWrap(True)
        self.physical_label = QtWidgets.QLabel("物理通道数：未知")
        self.physical_label.setWordWrap(True)
        self.pair_box = QtWidgets.QPlainTextEdit()
        self.pair_box.setReadOnly(True)
        self.pair_box.setMinimumHeight(120)

        self.channel_list = QtWidgets.QListWidget()
        self.channel_list.itemChanged.connect(self._update_plot)

        btn_row = QtWidgets.QHBoxLayout()
        select_all_btn = QtWidgets.QPushButton("全选")
        clear_btn = QtWidgets.QPushButton("清空")
        select_all_btn.clicked.connect(self._select_all)
        clear_btn.clicked.connect(self._clear_all)
        btn_row.addWidget(select_all_btn)
        btn_row.addWidget(clear_btn)

        self.status_label = QtWidgets.QLabel("")
        self.status_label.setStyleSheet("color: #c0392b;")

        controls_layout.addWidget(self.path_label)
        controls_layout.addWidget(open_btn)
        controls_layout.addWidget(self.info_label)
        controls_layout.addWidget(self.type_label)
        controls_layout.addWidget(self.physical_label)
        controls_layout.addWidget(QtWidgets.QLabel("源-探测器对应："))
        controls_layout.addWidget(self.pair_box)
        controls_layout.addWidget(QtWidgets.QLabel("通道选择："))
        controls_layout.addWidget(self.channel_list, stretch=1)
        controls_layout.addLayout(btn_row)
        controls_layout.addWidget(self.status_label)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.showGrid(x=True, y=True, alpha=0.2)
        self.plot_widget.setLabel("bottom", "Time (s)")
        self.plot_widget.setLabel("left", "Value")
        self.plot_widget.addLegend(offset=(10, 10))

        layout.addWidget(controls, stretch=1)
        layout.addWidget(self.plot_widget, stretch=3)
        self.setCentralWidget(central)

    def _open_file(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "选择 NIRS 数据文件",
            "",
            "NIRS/CSV Files (*.nirs *.csv);;All Files (*)",
        )
        if not path:
            return

        try:
            ext = os.path.splitext(path)[1].lower()
            if ext == ".nirs":
                t, data_t, names, type_label, physical_count, pair_info = load_nirs(path)
            elif ext == ".csv":
                t, data_t, names, type_label, physical_count, pair_info = load_csv(path)
            else:
                raise ValueError("仅支持 .nirs 或 .csv 文件。")

            self._time = t
            self._data = data_t
            self._channel_names = names
            self._data_type_label = type_label
            self._physical_channels = physical_count
            self._pair_info = pair_info
            self.path_label.setText(path)
            self.info_label.setText(
                f"通道: {data_t.shape[1]} | 采样点: {data_t.shape[0]}"
            )
            self.type_label.setText(f"数据类型：{type_label}")
            if physical_count is None:
                self.physical_label.setText("物理通道数：未知")
            else:
                self.physical_label.setText(f"物理通道数：{physical_count}")
            self._update_pair_box()
            self._populate_channels()
            self.status_label.setText("")
            self._update_plot()
        except Exception as exc:
            self.status_label.setText(f"加载失败：{exc}")

    def _populate_channels(self) -> None:
        self.channel_list.blockSignals(True)
        self.channel_list.clear()
        for name in self._channel_names:
            item = QtWidgets.QListWidgetItem(name)
            item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
            item.setCheckState(QtCore.Qt.Unchecked)
            self.channel_list.addItem(item)
        self.channel_list.blockSignals(False)

    def _selected_indices(self) -> List[int]:
        indices: List[int] = []
        for i in range(self.channel_list.count()):
            item = self.channel_list.item(i)
            if item.checkState() == QtCore.Qt.Checked:
                indices.append(i)
        return indices

    def _select_all(self) -> None:
        for i in range(self.channel_list.count()):
            self.channel_list.item(i).setCheckState(QtCore.Qt.Checked)
        self._update_plot()

    def _clear_all(self) -> None:
        for i in range(self.channel_list.count()):
            self.channel_list.item(i).setCheckState(QtCore.Qt.Unchecked)
        self._update_plot()

    def _update_plot(self) -> None:
        self.plot_widget.clear()
        self.plot_widget.addLegend(offset=(10, 10))
        if self._time is None or self._data is None:
            return

        indices = self._selected_indices()
        if not indices:
            return

        for idx, ch in enumerate(indices):
            pen = pg.mkPen(color=pg.intColor(idx, hues=max(len(indices), 1)))
            self.plot_widget.plot(
                self._time,
                self._data[:, ch],
                pen=pen,
                name=self._channel_names[ch],
            )

    def _update_pair_box(self) -> None:
        if not self._pair_info:
            self.pair_box.setPlainText("无（仅 .nirs 文件可解析）")
            return

        lines = [
            f"S{src}-D{det} : {meas_type}"
            for src, det, meas_type in self._pair_info
        ]
        self.pair_box.setPlainText("\n".join(lines))


def main() -> None:
    app = QtWidgets.QApplication([])
    viewer = NirsViewer()
    viewer.show()
    app.exec_()


if __name__ == "__main__":
    main()
