# -*- coding: utf-8 -*-
"""在线滤波结构选型对比图 —— IIR 因果 / 线性相位 FIR / 滑动基线相减。

四张图
    A  阶跃响应       血氧真的变了，各结构多久能看出来（最贴临床的一张）
    B  实测片段       同一段真实信号，四条曲线叠加
    C  延迟 vs 高通   IIR 是范围（色散，不可补），FIR/滑动基线是确定值（可补）
    D  波形保真度     同一通带下与离线零相位的相关

用法
    python tools/fig_online_filter.py [--csv 路径] [--out 输出.png]
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
from online_filter_sweep import fir_taps_for                          # noqa: E402

SURF, INK, INK2, RULE = "#fcfcfb", "#0b0b0b", "#52514e", "#e3e2dd"
C_REF, C_IIR, C_FIR, C_MOV = "#2a78d6", "#c0392b", "#1baf7a", "#a86a12"
plt.rcParams.update({
    "font.sans-serif": ["Microsoft YaHei", "SimHei", "DengXian", "sans-serif"],
    "axes.unicode_minus": False, "figure.facecolor": SURF, "axes.facecolor": SURF,
    "savefig.facecolor": SURF, "text.color": INK, "axes.labelcolor": INK2,
    "axes.edgecolor": RULE, "xtick.color": INK2, "ytick.color": INK2,
    "axes.titlesize": 11.5, "axes.labelsize": 9.5, "legend.fontsize": 8.8,
    "xtick.labelsize": 8.5, "ytick.labelsize": 8.5,
    "axes.spines.top": False, "axes.spines.right": False,
    "grid.color": RULE, "grid.linewidth": 0.7,
})

DEFAULT_CSV = _HERE.parent / "result_table" / "2026-07-03_14-59-48" / "all_groups.csv"
CANDS = [0.010, 0.015, 0.020, 0.030, 0.040, 0.050]


def moving_baseline(x, win):
    """因果滑动中值基线相减：只用当前及过去 win 个样本。延迟约 win/2。"""
    x = np.atleast_2d(x)
    out = np.empty_like(x)
    for i in range(x.shape[0]):
        v = x[i]
        base = np.empty_like(v)
        for k in range(v.size):
            lo = max(0, k - win + 1)
            base[k] = np.median(v[lo:k + 1])
        out[i] = v - base
    return out


def resp90(y, target):
    """阶跃后达到终值 90% 并保持所需的样本数；未达到返回 nan。"""
    thr = 0.9 * target
    ok = np.abs(y) >= abs(thr)
    if not ok.any():
        return np.nan
    for k in range(ok.size):
        if ok[k:].all():
            return k
    return np.nan


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    ap.add_argument("--out", type=Path, default=_HERE.parent / "在线滤波结构选型.png")
    a = ap.parse_args()

    # ---------- 数据 ----------
    d = pd.read_csv(a.csv); d.columns = [c.strip() for c in d.columns]
    inter = prepare_interleaved_dataframe(d, start_at_zero=False)
    t = np.asarray(calculate_concentration_series(inter)[0], float)
    fs = 1.0 / float(np.median(np.diff(t)))
    det = next(x for x in DETECTOR_CHANNELS if x.name == "S1_D1")
    od = intensities_to_od_changes(stack_intensities_for_detector(inter, det))
    wl = mbll_wavelengths_nm(); dpf = [get_dpf(w, 30) for w in wl]
    mb = lambda x: generalized_mbll(x, wl, dpf, 3.0)[0]       # noqa: E731
    N = od.shape[1]
    tt = t - t[0]

    LC = BP_LOW_HZ                                            # 主对比用现值
    sos = butter_bandpass_sos(LC, BP_HIGH_HZ, fs, order=BP_ORDER)
    ref = mb(sosfiltfilt(sos, od, axis=1))                    # 离线基准
    zi0 = sosfilt_zi(sos)[:, None, :] * od[:, 0][None, :, None]
    y_iir = mb(sosfilt(sos, od, axis=1, zi=zi0)[0])
    nt = fir_taps_for(LC, fs); g = (nt - 1) // 2
    fir = firwin(nt, [LC, BP_HIGH_HZ], pass_zero=False, fs=fs)
    y_fir = mb(lfilter(fir, [1.0], od, axis=1))
    WIN = int(round(1.0 / LC))                                # 滑动窗 ≈ 1/f_c
    y_mov = mb(moving_baseline(od, WIN))

    fig = plt.figure(figsize=(14.6, 9.8))
    gs = fig.add_gridspec(3, 2, height_ratios=[1.0, 1.0, 1.0], hspace=0.46, wspace=0.2,
                          left=0.062, right=0.975, top=0.895, bottom=0.065)
    fig.text(0.062, 0.958, "在线滤波结构选型：IIR 因果 / 线性相位 FIR / 滑动基线相减",
             fontsize=17, fontweight="bold")
    fig.text(0.062, 0.928,
             "录音 %s   fs=%.3f Hz   通带 %.3f~%.3f Hz   离线基准为同参数零相位 sosfiltfilt"
             % (a.csv.parent.name, fs, LC, BP_HIGH_HZ), fontsize=9.5, color=INK2)

    # ---------- A 阶跃响应 ----------
    ax = fig.add_subplot(gs[0, :])
    L = 1200
    step = np.zeros((5, L)); step[:, 300:] = 1.0             # 第 300 s 起阶跃
    ts = np.arange(L)
    s_iir = sosfilt(sos, step, axis=1)[0]
    s_fir = np.roll(lfilter(fir, [1.0], step, axis=1)[0], -g)   # 补偿已知固定延迟
    s_mov = moving_baseline(step, WIN)[0]
    ax.plot(ts - 300, step[0], color=INK2, lw=1.4, ls=":", label="真实变化（阶跃并保持）")
    for y, c, nm in ((s_iir, C_IIR, "IIR 因果（现方案）"),
                     (s_fir, C_FIR, "线性相位 FIR，%d 抽头（已补偿 %d s）" % (nt, g)),
                     (s_mov, C_MOV, "滑动基线相减，窗 %d s" % WIN)):
        ax.plot(ts - 300, y, color=c, lw=1.7, label=nm)
    ax.axvline(0, color=INK2, lw=1.2, ls="--")
    ax.axhline(0, color=RULE, lw=1)
    ax.set_xlim(-40, 260); ax.set_ylim(-0.75, 1.25)
    ax.annotate("三种结构都是高通：\n持续不变的血氧最终都会衰减回零",
                xy=(150, 0.02), xytext=(120, 0.72), fontsize=9.5, color=C_IIR,
                arrowprops=dict(arrowstyle="->", color=C_IIR, lw=1.2))
    ax.set_title("A  阶跃响应 —— 血氧掉下去并保持，读数怎么走（这是高通的固有性质，不是缺陷）", loc="left")
    ax.set_xlabel("阶跃后时间 (s)"); ax.set_ylabel("归一化响应")
    ax.grid(alpha=.5); ax.legend(frameon=False, ncol=2, loc="upper right", fontsize=8.4)

    # ---------- B 实测片段 ----------
    ax = fig.add_subplot(gs[1, :])
    lo, hi = 380, 720
    m = (tt >= lo) & (tt <= hi)
    U = 1e6
    ax.plot(tt[m], ref[m] * U, color=C_REF, lw=2.0, label="离线（零相位，用了未来数据）")
    ax.plot(tt[m], y_iir[m] * U, color=C_IIR, lw=1.5, label="IIR 因果（现方案）")
    sh = np.roll(y_fir, -g)
    ax.plot(tt[m], sh[m] * U, color=C_FIR, lw=1.5, ls="--",
            label="FIR 因果，已补偿固定延迟 %d s" % g)
    ax.plot(tt[m], y_mov[m] * U * 0.35, color=C_MOV, lw=1.5, alpha=.85,
            label="滑动基线相减（幅度已缩放便于比形状）")
    ax.set_title("B  实测片段 %d~%d s —— FIR 补偿延迟后贴合离线，IIR 无法靠平移贴合" % (lo, hi), loc="left")
    ax.set_xlabel("时间 (s)"); ax.set_ylabel("ΔHbO (μM)")
    ax.grid(alpha=.5); ax.legend(frameon=False, ncol=2, loc="upper right")

    # ---------- C 延迟 vs 高通 ----------
    ax = fig.add_subplot(gs[2, 0])
    gmins, gmaxs, fgs, mvs = [], [], [], []
    for lc in CANDS:
        s = butter_bandpass_sos(lc, BP_HIGH_HZ, fs, order=BP_ORDER)
        bb, aa = sos2tf(s)
        w, gd = group_delay((bb, aa), w=4096, fs=fs)
        inb = (w >= lc) & (w <= BP_HIGH_HZ)
        gmins.append(gd[inb].min()); gmaxs.append(gd[inb].max())
        fgs.append((fir_taps_for(lc, fs) - 1) // 2)
        mvs.append(int(round(1.0 / lc)) / 2)
    ax.fill_between(CANDS, gmins, gmaxs, color=C_IIR, alpha=.22,
                    label="IIR 群延迟范围（色散，不可补偿）")
    ax.plot(CANDS, gmaxs, color=C_IIR, lw=1.4)
    ax.plot(CANDS, gmins, color=C_IIR, lw=1.4)
    ax.plot(CANDS, fgs, "o-", color=C_FIR, lw=1.8, ms=5, label="FIR 固定延迟（可补偿）")
    ax.plot(CANDS, mvs, "s-", color=C_MOV, lw=1.8, ms=5, label="滑动基线延迟（可补偿）")
    ax.set_title("C  延迟 vs 高通截止", loc="left")
    ax.set_xlabel("高通截止 (Hz)"); ax.set_ylabel("延迟 (s)")
    ax.grid(alpha=.5); ax.legend(frameon=False, fontsize=8.2)

    # ---------- D 保真度 ----------
    ax = fig.add_subplot(gs[2, 1])
    r_iir, r_fir = [], []
    for lc in CANDS:
        s = butter_bandpass_sos(lc, BP_HIGH_HZ, fs, order=BP_ORDER)
        base = mb(sosfiltfilt(s, od, axis=1))                 # 同通带的零相位基准
        z = sosfilt_zi(s)[:, None, :] * od[:, 0][None, :, None]
        yi = mb(sosfilt(s, od, axis=1, zi=z)[0])
        r_iir.append(np.corrcoef(base, yi)[0, 1])
        n2 = fir_taps_for(lc, fs); g2 = (n2 - 1) // 2
        if n2 < N - 20:
            f2 = firwin(n2, [lc, BP_HIGH_HZ], pass_zero=False, fs=fs)
            yf = mb(lfilter(f2, [1.0], od, axis=1))
            aa2, bb2 = base[n2:N - n2], yf[n2 + g2:N - n2 + g2]
            mm = min(len(aa2), len(bb2))
            r_fir.append(np.corrcoef(aa2[:mm], bb2[:mm])[0, 1] if mm > 50 else np.nan)
        else:
            r_fir.append(np.nan)
    ax.plot(CANDS, r_iir, "o-", color=C_IIR, lw=1.8, ms=5, label="IIR 因果")
    ax.plot(CANDS, r_fir, "o-", color=C_FIR, lw=1.8, ms=5, label="FIR 因果（补偿延迟后）")
    ax.axhline(1.0, color=RULE, lw=1, ls="--")
    ax.set_ylim(0, 1.1)
    ax.set_title("D  波形保真度 —— 与【同通带】离线零相位的相关", loc="left")
    ax.set_xlabel("高通截止 (Hz)"); ax.set_ylabel("相关系数")
    ax.grid(alpha=.5); ax.legend(frameon=False, loc="lower left")

    a.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.out, dpi=140)
    print("已写出 %s" % a.out)
    # 高通对阶跃的响应必然衰减回零，"达到终值 90%" 无意义，改报峰值与衰减
    def prof(y):
        pk = int(np.argmax(np.abs(y[300:]))) 
        half = np.where(np.abs(y[300 + pk:]) < 0.5 * abs(y[300 + pk]))[0]
        return pk, (int(half[0]) if half.size else -1)
    for nm, y in (("IIR", s_iir), ("FIR(已补偿)", s_fir), ("滑动基线", s_mov)):
        pk, hf = prof(y)
        print("  %-12s 峰值出现在阶跃后 %3d s，峰值 %.2f，衰减到峰值一半再需 %s s"
              % (nm, pk, y[300 + pk], hf if hf >= 0 else ">窗长"))
    for i, lc in enumerate(CANDS):
        print("  %.3f Hz  IIR %.1f~%.1f s r=%.3f | FIR %d s r=%s | 滑动 %.0f s"
              % (lc, gmins[i], gmaxs[i], r_iir[i], fgs[i],
                 ("%.3f" % r_fir[i]) if np.isfinite(r_fir[i]) else "NA", mvs[i]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
