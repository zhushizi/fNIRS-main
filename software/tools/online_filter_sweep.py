# -*- coding: utf-8 -*-
"""在线滤波器选型扫描 —— 高通截止频率与滤波器结构的权衡。

背景
    现有带通 0.010~0.100 Hz 四阶巴特沃斯是【为离线设计的】：离线有
    sosfiltfilt（双向、零相位）与两端各 300 点外推兜底，0.01 Hz 的代价被
    抵消掉了。直接搬到在线，代价全部暴露：
      · 群延迟 6.6~71.4 s，随频率变 -> 波形被色散重塑，平移救不回来
        （实测：把因果滤波正向做两遍使幅度响应与离线一致后，
          最佳平移下相关仍只有 0.186）
      · 阶跃瞬态衰减到 5% 需 133 s

    线性相位 FIR 的群延迟恒为 (N−1)/2，与频率无关，因果输出平移该值后
    与零相位输出【逐位相同】（实测相关 1.00000000，最大差 0.00e+00）。
    代价是低频截止越低、所需抽头越多、固定延迟越大。

本脚本对每个候选高通频率给出：
    IIR  群延迟范围、阶跃响应时间、与离线的相关
    FIR  达到同等过渡带所需抽头数、固定延迟、与离线的相关
    并列出该截止频率会滤掉哪些生理成分。

用法
    python tools/online_filter_sweep.py [--csv 路径]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.signal import (firwin, group_delay, lfilter, sos2tf, sosfilt,
                          sosfilt_zi, sosfiltfilt)

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))
sys.path.insert(0, str(_HERE))
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

import warnings
warnings.filterwarnings("ignore")

from config import DETECTOR_CHANNELS, mbll_wavelengths_nm            # noqa: E402
from fnirs_pipeline.mbll import (BP_HIGH_HZ, BP_LOW_HZ, BP_ORDER,    # noqa: E402
                                 butter_bandpass_sos,
                                 calculate_concentration_series,
                                 generalized_mbll)
from fnirs_pipeline.mbll_core import get_dpf, intensities_to_od_changes  # noqa: E402
from fnirs_pipeline.preprocessing import (prepare_interleaved_dataframe,  # noqa: E402
                                          stack_intensities_for_detector)

DEFAULT_CSV = _HERE.parent / "result_table" / "2026-07-03_14-59-48" / "all_groups.csv"
# 候选高通截止 (Hz)。0.010 为现值。
CANDIDATES = [0.005, 0.010, 0.015, 0.020, 0.030, 0.040, 0.050]
# 生理成分参考（周期 -> 频率），用于说明各截止频率滤掉了什么
PHYSIO = [("超低频/自主调节", 0.005), ("Mayer 波", 0.10), ("呼吸", 0.25), ("心跳", 1.0)]


def load(csv: Path):
    d = pd.read_csv(csv)
    d.columns = [c.strip() for c in d.columns]
    inter = prepare_interleaved_dataframe(d, start_at_zero=False)
    t = np.asarray(calculate_concentration_series(inter)[0], float)
    fs = 1.0 / float(np.median(np.diff(t)))
    det = next(x for x in DETECTOR_CHANNELS if x.name == "S1_D1")
    od = intensities_to_od_changes(stack_intensities_for_detector(inter, det))
    wl = mbll_wavelengths_nm()
    dpf = [get_dpf(w, 30) for w in wl]
    return od, fs, wl, dpf, t


def step_settle(sos, fs, frac, nmax=4000):
    """阶跃响应衰减到峰值 frac 以下所需秒数。"""
    y = sosfilt(sos, np.ones(nmax))
    idx = np.where(np.abs(y) > frac * np.abs(y).max())[0]
    return (idx[-1] / fs) if idx.size else 0.0


def fir_taps_for(lowcut, fs, atten_db=60.0):
    """凯泽窗估计：过渡带取 lowcut 的一半时所需抽头数（奇数）。"""
    trans = max(lowcut * 0.5, 1e-4)                 # 过渡带宽 Hz
    n = int(np.ceil((atten_db - 8) / (2.285 * 2 * np.pi * trans / fs)))
    return n + 1 if n % 2 == 0 else n


def corr(a, b):
    m = min(len(a), len(b))
    a, b = a[:m], b[:m]
    ok = np.isfinite(a) & np.isfinite(b)
    return float(np.corrcoef(a[ok], b[ok])[0, 1]) if ok.sum() > 50 else np.nan


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    ap.add_argument("--high", type=float, default=BP_HIGH_HZ, help="低通截止，固定")
    a = ap.parse_args()

    od, fs, wl, dpf, t = load(a.csv)
    N = od.shape[1]
    mb = lambda x: generalized_mbll(x, wl, dpf, 3.0)[0]      # noqa: E731

    # 离线基准：当前参数、零相位
    sos_ref = butter_bandpass_sos(BP_LOW_HZ, a.high, fs, order=BP_ORDER)
    ref = mb(sosfiltfilt(sos_ref, od, axis=1))

    print("=" * 96)
    print("在线滤波器选型扫描")
    print("=" * 96)
    print("  录音 %s   %d 帧   fs=%.4f Hz   低通固定 %.3f Hz   IIR 四阶巴特沃斯"
          % (a.csv.parent.name, N, fs, a.high))
    print("  离线基准 = 现参数 %.3f~%.3f Hz 零相位（sosfiltfilt）"
          % (BP_LOW_HZ, a.high))
    print()
    print("  高通     ── IIR 因果 ──────────────────────   ── 线性相位 FIR ──────────────")
    print("  (Hz)     群延迟范围     阶跃到5%   与离线相关   抽头   固定延迟   与离线相关")
    print("  " + "-" * 92)

    rows = []
    for lc in CANDIDATES:
        sos = butter_bandpass_sos(lc, a.high, fs, order=BP_ORDER)
        if sos is None:
            continue
        # --- IIR ---
        bb, aa = sos2tf(sos)
        w, gd = group_delay((bb, aa), w=4096, fs=fs)
        inb = (w >= lc) & (w <= a.high)
        gmin, gmax = float(gd[inb].min()), float(gd[inb].max())
        settle = step_settle(sos, fs, 0.05)
        zi0 = sosfilt_zi(sos)[:, None, :] * od[:, 0][None, :, None]
        y_iir = mb(sosfilt(sos, od, axis=1, zi=zi0)[0])
        r_iir = corr(ref, y_iir)

        # --- FIR ---
        nt = fir_taps_for(lc, fs)
        g = (nt - 1) // 2
        if nt < N - 10:
            fir = firwin(nt, [lc, a.high], pass_zero=False, fs=fs)
            y_fir = mb(lfilter(fir, [1.0], od, axis=1))
            r_fir = corr(ref[: N - g], y_fir[g:])       # 按固定群延迟对齐后再比
        else:
            r_fir = np.nan

        mark = "  <- 现值" if abs(lc - BP_LOW_HZ) < 1e-9 else ""
        print("  %.3f   %5.1f ~ %5.1f s   %6.0f s   %+.4f      %5d  %6d s    %s%s"
              % (lc, gmin, gmax, settle, r_iir, nt, g,
                 ("%+.4f" % r_fir) if np.isfinite(r_fir) else "  数据不足", mark))
        rows.append(dict(lc=lc, gmin=gmin, gmax=gmax, settle=settle,
                         r_iir=r_iir, nt=nt, g=g, r_fir=r_fir))

    print()
    print("  说明")
    print("    · IIR 群延迟范围 = 通带内最小~最大；跨度就是色散量，平移无法补偿")
    print("    · FIR 抽头数按凯泽窗估计（阻带 60 dB，过渡带取高通截止的一半），")
    print("      固定延迟 = (抽头−1)/2，与频率无关，平移即可精确还原波形")
    print("    · FIR 的「与离线相关」已按固定群延迟对齐；IIR 的未对齐（也无法对齐）")
    print("    · 数据长度 %d 帧，抽头数接近该值时结果不可信" % N)
    print()
    print("  各高通截止滤掉的周期")
    print("    高通(Hz)   截止周期(s)   保留")
    for r_ in rows:
        print("    %.3f      %7.0f     周期短于 %.0f s 的成分" % (r_["lc"], 1 / r_["lc"], 1 / r_["lc"]))
    print()
    print("  生理成分频率参考：", "  ".join("%s≈%.3fHz" % (n, f) for n, f in PHYSIO))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
