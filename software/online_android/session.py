"""TCP 采集期间的在线分析会话：缓冲 + 后台 worker 生命周期。"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import pandas as pd

from config import with_channel_suffix

from .baseline_state import MutableBaseline
from .batch_recorder import AndroidLiveOutputRecorder
from .buffer import OnlineSampleBuffer
from .channel_dispatcher import ChannelDispatcher
from .config import DEFAULT_ONLINE_SETTINGS, OnlineSettings
from .live_plotter import HBO_HBR_WINDOW_S, RSO2_WINDOW_S, LiveAndroidBatchPlotter
from .logging_utils import get_logger
from .reporter import AndroidReporter
from .worker import OnlineAnalysisWorker

if TYPE_CHECKING:
    from .tcp_bridge import HostTcpSerialBridge

log = get_logger("session")

PrepareInterleavedFn = Callable[[pd.DataFrame, bool], pd.DataFrame]
CalculateSeriesFn = Callable[[pd.DataFrame], object]


@dataclass
class OnlineCaptureSession:
    """一次 TCP 采集对应的在线分析上下文。"""

    buffer: OnlineSampleBuffer
    worker: OnlineAnalysisWorker
    reporter: AndroidReporter
    dispatcher: ChannelDispatcher
    prepare_interleaved: PrepareInterleavedFn
    calculate_series: CalculateSeriesFn
    plotter: LiveAndroidBatchPlotter | None = None
    # {通道码: 录制器}，采集结束时逐通道 flush。
    recorders: dict[int, AndroidLiveOutputRecorder] = field(default_factory=dict)

    def feed_sample(
        self,
        elapsed_time: float,
        detector_id: int,
        value: float,
        wavelength_code: int,
        channel_name: str = "",
        acq_channel_code: int = 0x01,
    ) -> None:
        self.buffer.append(
            elapsed_time, detector_id, value, wavelength_code, channel_name, acq_channel_code
        )

    def stop(self) -> None:
        self.worker.stop()
        # 逐通道落盘「实际回传安卓的曲线」本身（μM/M、已锚定、因果），
        # 每个采集通道各写一份 android_live_output_ch{n}.csv，与安卓所见一致。
        for recorder in self.recorders.values():
            recorder.flush()


def create_online_session(
    bridge: HostTcpSerialBridge,
    *,
    settings: OnlineSettings = DEFAULT_ONLINE_SETTINGS,
    prepare_interleaved: PrepareInterleavedFn,
    calculate_series: CalculateSeriesFn,
    live_plot: bool = False,
    live_plot_hbo_hbr_window_s: float = HBO_HBR_WINDOW_S,
    live_plot_rso2_window_s: float = RSO2_WINDOW_S,
    android_live_output_path: str | None = None,
) -> OnlineCaptureSession:
    """创建并启动在线分析会话（buffer + 按通道分发的 worker）。"""
    baseline = MutableBaseline(
        rso2_pct=settings.rso2_baseline_rso2_pct,
        hbt_uM=settings.rso2_baseline_hbt_uM,
    )
    buffer = OnlineSampleBuffer(settings)

    plotter: LiveAndroidBatchPlotter | None = None
    if live_plot:
        plotter = LiveAndroidBatchPlotter(
            hbo_hbr_window_s=live_plot_hbo_hbr_window_s,
            rso2_window_s=live_plot_rso2_window_s,
        )
        log.info(
            "Live plot enabled: HbO/HbR window=%.0fs, rSO2 window=%.0fs (per acquisition channel).",
            live_plot_hbo_hbr_window_s,
            live_plot_rso2_window_s,
        )

    recorders: dict[int, AndroidLiveOutputRecorder] = {}
    reporter = AndroidReporter(bridge, settings, plotter=plotter, recorders=recorders)

    def on_new_channel(code: int) -> None:
        """某个采集通道首次出现：建立它自己的落盘录制器。"""
        if android_live_output_path:
            # recorders 与 reporter.recorders 是同一对象，写一处即可。
            recorders[code] = AndroidLiveOutputRecorder(
                with_channel_suffix(android_live_output_path, code)
            )
        log.info("Acquisition channel %s detected; online analysis started for it.", int(code))

    dispatcher = ChannelDispatcher(
        settings,
        prepare_interleaved=prepare_interleaved,
        calculate_series=calculate_series,
        baseline=baseline,
        on_new_channel=on_new_channel,
    )
    if settings.online_mode == "causal_incremental":
        log.info("Online mode: causal_incremental (causal IIR + append-only), per channel.")
    else:
        log.info("Online mode: full_replace (non-causal, full-series replace), per channel.")

    worker = OnlineAnalysisWorker(
        buffer,
        bridge,
        dispatcher,
        settings=settings,
        reporter=reporter,
    )
    bridge.register_set_baseline_handler(worker.handle_set_baseline)
    worker.start()
    return OnlineCaptureSession(
        buffer=buffer,
        worker=worker,
        reporter=reporter,
        dispatcher=dispatcher,
        prepare_interleaved=prepare_interleaved,
        calculate_series=calculate_series,
        plotter=plotter,
        recorders=recorders,
    )
