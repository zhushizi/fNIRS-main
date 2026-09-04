# -*- coding: utf-8 -*-
"""在线因果链路 vs 离线链路：同一份原始光强，两条曲线差多少、为什么。

四张图
    A  全程叠加            两条曲线放一起看
    B  局部放大            差异是波形重塑，不是整体平移
    C  差异来源分解        只改滤波 vs 只改配对，各自贡献多少
    D  群延迟              根因：通带内各频率被延迟的量相差 65 s

用法
    python tools/fig_online_vs_offline.py [--csv 路径] [--out 输出.png]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import group_delay, sos2tf, sosfilt, sosfilt_zi, sosfiltfilt

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
from fnirs_pipeline.mbll_core import (get_dpf,                       # noqa: E402
                                      intensities_to_od_changes)
from fnirs_pipeline.preprocessing import (prepare_interleaved_dataframe,   # noqa: E402
                                          stack_intensities_for_detector)
from online_replay import reconstruct                                # noqa: E402

SURF, INK, INK2, RULE = "#fcfcfb", "#0b0b0b", "#52514e", "#e3e2dd"
C_OFF, C_ON, C_MID, C_BAD = "#2a78d6", "#eb6834", "#8e5bc9", "#c0392b"
plt.rcParams.update({
    "font.sans-serif": ["Microsoft YaHei", "SimHei", "DengXian", "sans-serif"],
    "axes.unicode_minus": False, "figure.facecolor": SURF, "axes.facecolor": SURF,
    "savefig.facecolor": SURF, "text.color": INK, "axes.labelcolor": INK2,
    "axes.edgecolor": RULE, "xtick.color": INK2, "ytick.color": INK2,
    "axes.titlesize": 11.5, "axes.labelsize": 9.5, "legend.fontsize": 9,
    "xtick.labelsize": 8.5, "ytick.labelsize": 8.5,
    "axes.spines.top": False, "axes.spines.right": False,
    "grid.color": RULE, "grid.linewidth": 0.7,
})

DEFAULT_CSV = _HERE.parent / "result_table" / "2026-07-03_14-59-48" / "all_groups.csv"


def build(csv: Path):
    d = pd.read_csv(csv)
    d.columns = [c.strip() for c in d.columns]

    # 离线：完整管线
    inter = prepare_interleaved_dataframe(d, start_at_zero=False)
    t, hbo_off, _, _ = calculate_concentration_series(inter)
    t = np.asarray(t, float)
    hbo_off = np.asarray(hbo_off, float)
    fs = 1.0 / float(np.median(np.diff(t)))

    # 在线：按 1 Hz tick 回放后重建
    hbo_on = reconstruct(d, "time:1.0")["hbo"]

    # 分解用：同一份 ΔOD，只改滤波方式
    det = next(x for x in DETECTOR_CHANNELS if x.name == "S1_D1")
    od = intensities_to_od_changes(stack_intensities_for_detector(inter, det))
    wl = mbll_wavelengths_nm()
    dpf = [get_dpf(w, 30) for w in wl]
    sos = butter_bandpass_sos(BP_LOW_HZ, BP_HIGH_HZ, fs, order=BP_ORDER)
    zero_phase = generalized_mbll(sosfiltfilt(sos, od, axis=1), wl, dpf, 3.0)[0]
    zi0 = sosfilt_zi(sos)[:, None, :] * od[:, 0][None, :, None]
    causal = generalized_mbll(sosfilt(sos, od, axis=1, zi=zi0)[0], wl, dpf, 3.0)[0]

    n = min(len(t), len(hbo_off), len(hbo_on), len(zero_phase), len(causal))
    return dict(t=t[:n] - t[0], off=hbo_off[:n], on=hbo_on[:n],
                zp=zero_phase[:n], ca=causal[:n], fs=fs, sos=sos, name=csv.parent.name)


def r(a, b):
    m = np.isfinite(a) & np.isfinite(b)
    return float(np.corrcoef(a[m], b[m])[0, 1]) if m.sum() > 10 else np.nan


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    ap.add_argument("--out", type=Path, default=_HERE.parent / "在线vs离线波形.png")
    a = ap.parse_args()

    D = build(a.csv)
    t, off, on, zp, ca = D["t"], D["off"], D["on"], D["zp"], D["ca"]
    U = 1e6                                     # 摩尔 -> μM

    fig = plt.figure(figsize=(14.6, 9.6))
    gs = fig.add_gridspec(3, 2, height_ratios=[1.05, 1.0, 1.0], hspace=0.42, wspace=0.22,
                          left=0.062, right=0.975, top=0.9, bottom=0.07)

    fig.text(0.062, 0.962, "同一份原始光强，在线因果链路 vs 离线链路", fontsize=17, fontweight="bold")
    fig.text(0.062, 0.932,
             "录音 %s   %.0f s   %d 帧   采样 %.3f Hz   带通 %.3f~%.3f Hz 四阶巴特沃斯"
             % (D["name"], t[-1], len(t), D["fs"], BP_LOW_HZ, BP_HIGH_HZ),
             fontsize=9.5, color=INK2)

    # ---------- A 全程 ----------
    ax = fig.add_subplot(gs[0, :])
    ax.plot(t, off * U, color=C_OFF, lw=1.15, label="离线（零相位滤波，可用未来数据）")
    ax.plot(t, on * U, color=C_ON, lw=1.15, alpha=.9, label="在线（因果滤波 + 冻结基线）")
    ax.axvspan(0, 60, color=C_BAD, alpha=.07)
    ax.text(2, ax.get_ylim()[1] * .82, "热身 60 s", fontsize=8.5, color=C_BAD)
    ax.set_title("A  全程叠加 —— 相关系数 %.3f，在线幅度是离线的 %.2f 倍"
                 % (r(off, on), on.std() / off.std()), loc="left")
    ax.set_xlabel("时间 (s)"); ax.set_ylabel("ΔHbO (μM)")
    ax.grid(alpha=.5); ax.legend(frameon=False, ncol=2, loc="upper right")

    # ---------- B 局部放大 ----------
    ax = fig.add_subplot(gs[1, 0])
    lo, hi = 400, 700
    m = (t >= lo) & (t <= hi)
    ax.plot(t[m], off[m] * U, color=C_OFF, lw=1.7, label="离线")
    ax.plot(t[m], on[m] * U, color=C_ON, lw=1.7, label="在线")
    ax.set_title("B  放大 %d~%d s —— 不是整体平移，是波形被重塑" % (lo, hi), loc="left")
    ax.set_xlabel("时间 (s)"); ax.set_ylabel("ΔHbO (μM)")
    ax.grid(alpha=.5); ax.legend(frameon=False)

    # ---------- C 差异来源分解 ----------
    ax = fig.add_subplot(gs[1, 1])
    names = ["只改滤波\n零相位→因果", "只改配对\n1Hz低通→分段RMS", "两者都改\n离线→在线"]
    vals = [r(zp, ca), r(ca, on), r(off, on)]
    cols = [C_BAD, C_MID, "#555"]
    ax.bar(range(3), vals, color=cols, width=.55)
    for i, v in enumerate(vals):          # 负值的标签放柱子下方，避免压到 x 轴文字
        off_y, va = (.04, "bottom") if v >= 0 else (-.05, "top")
        ax.text(i, v + off_y, "%.3f" % v, ha="center", va=va,
                fontsize=11, fontweight="bold", color=cols[i])
    ax.axhline(1.0, color=RULE, lw=1, ls="--")
    ax.axhline(0.0, color=INK2, lw=1)
    ax.text(2.45, 1.03, "完全一致", fontsize=8, color=INK2, ha="right")
    ax.set_xticks(range(3)); ax.set_xticklabels(names, fontsize=8.5)
    ax.set_ylim(min(-0.45, min(vals) - .25), 1.18); ax.set_ylabel("与基准的相关系数")
    ax.set_title("C  差异来源 —— 滤波方式是主因", loc="left")
    ax.grid(alpha=.5, axis="y")

    # ---------- D 群延迟 ----------
    ax = fig.add_subplot(gs[2, 0])
    bb, aa = sos2tf(D["sos"])
    w, gd = group_delay((bb, aa), w=4096, fs=D["fs"])
    inb = (w >= BP_LOW_HZ) & (w <= BP_HIGH_HZ)
    ax.semilogx(w[inb], gd[inb], color=C_BAD, lw=2)
    for f0 in (0.01, 0.02, 0.05, 0.1):
        i = int(np.argmin(np.abs(w - f0)))
        ax.plot(f0, gd[i], "o", color=C_BAD, ms=5)
        ax.annotate("%.0f s" % gd[i], (f0, gd[i]), textcoords="offset points",
                    xytext=(6, 6), fontsize=8.5, color=C_BAD)
    ax.set_title("D  因果滤波器群延迟 —— 通带内相差 %.0f s" % (gd[inb].max() - gd[inb].min()), loc="left")
    ax.set_xlabel("频率 (Hz)"); ax.set_ylabel("群延迟 (s)"); ax.grid(alpha=.5, which="both")

    # ---------- E 滑窗相关 ----------
    ax = fig.add_subplot(gs[2, 1])
    W, S = 200, 25
    xs, ys = [], []
    for s0 in range(0, len(t) - W, S):
        xs.append(t[s0 + W // 2]); ys.append(r(off[s0:s0 + W], on[s0:s0 + W]))
    ax.plot(xs, ys, color=C_MID, lw=1.6)
    ax.axhline(0, color=RULE, lw=1)
    ax.fill_between(xs, 0, ys, color=C_MID, alpha=.15)
    ax.set_ylim(-1, 1)
    ax.set_title("E  滑窗相关（窗长 %d s）—— 不随时间改善，故不是启动瞬态" % W, loc="left")
    ax.set_xlabel("时间 (s)"); ax.set_ylabel("相关系数"); ax.grid(alpha=.5)

    a.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.out, dpi=140)
    print("已写出 %s" % a.out)
    print("  全程相关 %.4f   幅度比 %.3f" % (r(off, on), on.std() / off.std()))
    print("  只改滤波 %.4f   只改配对 %.4f" % (r(zp, ca), r(ca, on)))
    print("  群延迟 %.1f ~ %.1f s" % (gd[inb].min(), gd[inb].max()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
