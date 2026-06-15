"""将发往安卓的 live_analysis_batch 累积并保存为 processed_output 同款 CSV。"""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

from config import CHANNEL_NAME

from .types import LiveAnalysisBatch

HBO_COL = f"{CHANNEL_NAME}_hbo"
HBR_COL = f"{CHANNEL_NAME}_hbr"
PROCESSED_HEADER = ["Time", HBO_COL, HBR_COL]


def align_times_like_processed(times: list[float]) -> list[float]:
    """
    重建与 prepare_interleaved(start_at_zero=True) / processed_output.csv 一致的时间轴。

    在线批次保留窗口内真实起始时间；落盘时按平均配对间隔从 0 起编，便于与离线终算逐行对比。
    """
    if not times:
        return []
    arr = np.asarray(times, dtype=float)
    if len(arr) == 1:
        return [0.0]
    diffs = np.diff(arr)
    valid = diffs[np.isfinite(diffs) & (diffs > 0)]
    increment = float(np.mean(valid)) if valid.size else 0.001
    return [round(i * increment, 6) for i in range(len(arr))]


class AndroidLiveOutputRecorder:
    """累积每次 send_live_batch 的 HbO/HbR，采集结束时写入 android_live_output.csv。"""

    def __init__(self, output_path: str | Path | None) -> None:
        self.output_path = Path(output_path) if output_path else None
        self._times: list[float] = []
        self._hbo: list[float] = []
        self._hbr: list[float] = []

    @property
    def sample_count(self) -> int:
        return len(self._times)

    def append_batch(self, batch: LiveAnalysisBatch) -> None:
        if self.output_path is None:
            return
        for time_s, hbo, hbr in zip(batch.times, batch.hbo, batch.hbr):
            self._times.append(float(time_s))
            self._hbo.append(float(hbo))
            self._hbr.append(float(hbr))

    def flush(self) -> str | None:
        if self.output_path is None or not self._times:
            return None

        aligned_times = align_times_like_processed(self._times)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.output_path, "w", newline="", encoding="utf-8") as f_out:
            writer = csv.writer(f_out)
            writer.writerow(PROCESSED_HEADER)
            for time_s, hbo, hbr in zip(aligned_times, self._hbo, self._hbr):
                writer.writerow([time_s, hbo, hbr])

        path_str = str(self.output_path)
        print(
            f"Android live HbO/HbR saved to '{path_str}' "
            f"({len(self._times)} samples, Time aligned from 0 like processed_output.csv)."
        )
        return path_str
