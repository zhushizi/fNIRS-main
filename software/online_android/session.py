"""TCP 采集期间的在线分析会话：缓冲 + 后台 worker 生命周期。"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING

import pandas as pd

from .batch_builder import IncrementalBatchBuilder
from .buffer import OnlineSampleBuffer
from .config import DEFAULT_ONLINE_SETTINGS, OnlineSettings
from .live_plotter import HBO_HBR_WINDOW_S, RSO2_WINDOW_S, LiveAndroidBatchPlotter
from .reporter import AndroidReporter
from .worker import OnlineAnalysisWorker

if TYPE_CHECKING:
    from .tcp_bridge import HostTcpSerialBridge

PrepareInterleavedFn = Callable[[pd.DataFrame, bool], pd.DataFrame]
CalculateSeriesFn = Callable[[pd.DataFrame], object]


@dataclass
class OnlineCaptureSession:
    """一次 TCP 采集对应的在线分析上下文。"""

    buffer: OnlineSampleBuffer
    worker: OnlineAnalysisWorker
    reporter: AndroidReporter
    plotter: LiveAndroidBatchPlotter | None = None

    def feed_sample(
        self,
        elapsed_time: float,
        sensor_id: int,
        value: float,
        wavelength_code: int,
    ) -> None:
        self.buffer.append(elapsed_time, sensor_id, value, wavelength_code)

    def stop(self) -> None:
        self.worker.stop()


def create_online_session(
    bridge: HostTcpSerialBridge,
    *,
    settings: OnlineSettings = DEFAULT_ONLINE_SETTINGS,
    prepare_interleaved: PrepareInterleavedFn,
    calculate_series: CalculateSeriesFn,
    live_plot: bool = False,
    live_plot_hbo_hbr_window_s: float = HBO_HBR_WINDOW_S,
    live_plot_rso2_window_s: float = RSO2_WINDOW_S,
) -> OnlineCaptureSession:
    """创建并启动在线分析会话（buffer + worker）。"""
    buffer = OnlineSampleBuffer(settings)
    batch_builder = IncrementalBatchBuilder(
        settings,
        prepare_interleaved=prepare_interleaved,
        calculate_series=calculate_series,
    )
    plotter: LiveAndroidBatchPlotter | None = None
    if live_plot:
        plotter = LiveAndroidBatchPlotter(
            hbo_hbr_window_s=live_plot_hbo_hbr_window_s,
            rso2_window_s=live_plot_rso2_window_s,
        )
        print(
            "Live plot enabled: "
            f"HbO/HbR window={live_plot_hbo_hbr_window_s:.0f}s, "
            f"rSO2 window={live_plot_rso2_window_s:.0f}s."
        )
    reporter = AndroidReporter(bridge, settings, plotter=plotter)
    worker = OnlineAnalysisWorker(
        buffer,
        bridge,
        batch_builder,
        settings=settings,
        reporter=reporter,
    )
    worker.start()
    return OnlineCaptureSession(
        buffer=buffer,
        worker=worker,
        reporter=reporter,
        plotter=plotter,
    )
