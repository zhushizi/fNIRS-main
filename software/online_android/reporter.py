"""将分析结果通过 Host TCP 协议回传安卓。"""

from __future__ import annotations

import numpy as np

from .batch_recorder import AndroidLiveOutputRecorder
from .config import OnlineSettings
from .live_plotter import LiveAndroidBatchPlotter
from .rso2 import compute_rso2_series
from .tcp_bridge import HostTcpSerialBridge
from .types import AnalysisResult, LiveAnalysisBatch


class AndroidReporter:
    """封装 live_analysis_batch 与 analysis_result 的发送。"""

    def __init__(
        self,
        bridge: HostTcpSerialBridge,
        settings: OnlineSettings | None = None,
        plotter: LiveAndroidBatchPlotter | None = None,
        recorder: AndroidLiveOutputRecorder | None = None,
    ) -> None:
        self.bridge = bridge
        self.settings = settings or OnlineSettings()
        self.plotter = plotter
        self.recorder = recorder

    def send_live_batch(self, batch: LiveAnalysisBatch) -> None:
        if self.recorder is not None:
            self.recorder.append_batch(batch)
        if self.plotter is not None:
            try:
                self.plotter.update(batch)
            except Exception as exc:
                print(f"Live plot update skipped: {exc}")
        try:
            self.bridge.send_live_analysis_batch(
                times=batch.times,
                hbo=batch.hbo,
                hbr=batch.hbr,
                cyt=batch.cyt,
                window_start_s=batch.window_start_s,
                window_end_s=batch.window_end_s,
                ok=True,
                rso2=batch.rso2,
                baseline_ready=batch.baseline_ready,
                latest_rso2_pct=batch.latest_rso2_pct,
                baseline_rso2_pct=batch.baseline_rso2_pct,
                replace_full_series=batch.replace_full_series,
                unit=batch.unit,
                thi=batch.thi,
                delta_thi=batch.delta_thi,
            )
        except Exception as exc:
            print(f"Failed to send live_analysis_batch to Android: {exc}")

    def send_final_result(self, result: AnalysisResult) -> None:
        if self.bridge is None:
            return
        try:
            self.bridge.send_analysis_result(
                ok=bool(result.get("ok")),
                message=str(result["message"]) if result.get("message") is not None else None,
            )
        except Exception as exc:
            print(f"Failed to send analysis_result to Android: {exc}")


def enrich_result_with_rso2(
    result: AnalysisResult,
    *,
    times: np.ndarray | None,
    hbo_series: np.ndarray | None,
    hbr_series: np.ndarray | None,
    settings: OnlineSettings | None = None,
) -> AnalysisResult:
    """在离线终算摘要上附加 rSO2 统计（若基线窗内有样本）。"""
    if not result.get("ok") or times is None or hbo_series is None or hbr_series is None:
        return result
    if times.size < 2:
        return result

    cfg = settings or OnlineSettings()
    n = min(times.size, hbo_series.size, hbr_series.size)
    rso2_series = compute_rso2_series(
        times[:n],
        hbo_series[:n],
        hbr_series[:n],
        baseline_start_s=cfg.rso2_baseline_start_s,
        baseline_end_s=cfg.rso2_baseline_end_s,
        baseline_hbt_uM=cfg.rso2_baseline_hbt_uM,
        baseline_rso2_pct=cfg.rso2_baseline_rso2_pct,
    )
    if rso2_series is None:
        return result

    finite_rso2 = rso2_series[np.isfinite(rso2_series)]
    if finite_rso2.size == 0:
        return result

    enriched = dict(result)
    enriched["mean_rso2_pct"] = float(np.mean(finite_rso2))
    enriched["latest_rso2_pct"] = float(finite_rso2[-1])
    enriched["baseline_rso2_pct"] = cfg.rso2_baseline_rso2_pct
    return enriched
