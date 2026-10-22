# -*- coding: utf-8 -*-
"""第 8 条验证图：MBLL 解码器 —— 带通上限选型 + 负对照噪声底。

所有数字都来自真实归档，不是示意。
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter, sosfiltfilt

ROOT = Path(r"D:\WORK_space\Code_WS\code_localized\fNIRS-main")
sys.path.insert(0, str(ROOT / "software"))
sys.stdout.reconfigure(encoding="utf-8")

from mbll_decoder import hardware as mh
from mbll_decoder.decoder import decode
from mbll_decoder.run import load
from srs_cal import archive

SURF, INK, INK2, INK3, RULE = "#fcfcfb", "#0b0b0b", "#52514e", "#8b8a84", "#e3e2dd"
OK, BAD, NEU = "#1baf7a", "#eb6834", "#2a78d6"
plt.rcParams.update({
    "font.sans-serif": ["Microsoft YaHei", "SimHei", "DengXian", "sans-serif"],
    "axes.unicode_minus": False, "figure.facecolor": SURF, "axes.facecolor": SURF,
    "savefig.facecolor": SURF, "text.color": INK, "axes.labelcolor": INK2,
    "axes.edgecolor": RULE, "xtick.color": INK2, "ytick.color": INK2,
    "axes.titlesize": 11, "axes.labelsize": 9.5, "legend.fontsize": 8.5,
    "xtick.labelsize": 9, "ytick.labelsize": 9,
    "axes.spines.top": False, "axes.spines.right": False,
    "grid.color": RULE, "grid.linewidth": .7})

# --- 取最大的一份归档（10 分钟体模静置 = 负对照：真值恒定）-----------------
path = max(archive.list_archives(), key=lambda p: p.stat().st_size)
t3, I3 = load(path, 3.0)
t4, I4 = load(path, 4.0)
print(f"{path.name}  3cm {t3.size} 样本 {t3[-1]-t3[0]:.0f}s   4cm {t4.size} 样本")

r3 = decode(t3, I3, distance_cm=3.0)
r4 = decode(t4, I4, distance_cm=4.0)
assert r3.ok and r4.ok

# --- ΔOD 频谱：高通 0.01 去漂移后才看得见 0.0868 -----------------------------
# ⚠️ 未高通的频谱被漂移（0.0015 Hz = 记录长度本身）主宰，看不到干扰峰。
dt = float(np.median(np.diff(t3)))
fs = 1.0 / dt
grid = np.arange(t3[0], t3[-1] + dt * .5, dt)
dOD = np.interp(grid, t3,
                -np.log10(I3[1] / I3[1].mean()))      # 770nm 那一路
sos_hp = butter(4, 0.01 / (fs / 2), btype="high", output="sos")
hp = sosfiltfilt(sos_hp, dOD, padlen=min(dOD.size - 1, int(3 * fs / 0.01)))


def spec(y):
    y = y - y.mean()
    return (np.fft.rfftfreq(y.size, dt),
            np.abs(np.fft.rfft(y * np.hanning(y.size))) ** 2)


f, P = spec(hp)
m = (f >= 0.012) & (f <= fs / 2 * .97)
fpk = f[m][np.argmax(P[m])]
print(f"高通 0.01 后主峰 {fpk:.4f} Hz（周期 {1/fpk:.1f} s）")

# --- 各上限：ΔHbO 噪声底 + 干扰带占输出的比例 -------------------------------
# 干扰带 0.075~0.100 Hz（峰的实测展宽）。分母是输出总功率，
# 所以滤波器滚降的泄漏也算进来 —— 不靠「标称带外」自我安慰。
PK_LO, PK_HI = 0.075, 0.100
cuts = [0.03, 0.05, 0.07, 0.10]
sds, resid = [], []
for fc in cuts:
    rr = decode(t3, I3, distance_cm=3.0, band=(0.01, fc))
    f2, P2 = spec(rr.d_hbo)
    sds.append(rr.d_hbo.std(ddof=1))
    resid.append(100.0 * P2[(f2 >= PK_LO) & (f2 <= PK_HI)].sum()
                 / P2[f2 > 0].sum())
base = sds[cuts.index(0.05)]
for c, sd, rr_ in zip(cuts, sds, resid):
    print(f"  上限 {c:.2f}  std {sd:.4f} µM  干扰带占输出 {rr_:5.2f}%  "
          f"{sd/base:.2f}×")

# ===========================================================================
fig = plt.figure(figsize=(13.6, 7.4))
gs = fig.add_gridspec(2, 3, hspace=.42, wspace=.3,
                      left=.055, right=.985, top=.88, bottom=.08)

# (1) 负对照时间序列 ---------------------------------------------------------
ax = fig.add_subplot(gs[0, :2])
ax.axhline(0, color=RULE, lw=.8)
ax.plot(r3.t, r3.d_hbo, lw=.9, color=BAD, label=f"ΔHbO  std {r3.d_hbo.std(ddof=1):.4f} µM")
ax.plot(r3.t, r3.d_hbr, lw=.9, color=NEU, label=f"ΔHbR  std {r3.d_hbr.std(ddof=1):.4f} µM")
ax.set_title("① 负对照：体模静置 644 s，ρ=3cm —— 真值恒定，测出多少就是噪声底",
             loc="left", fontweight="bold")
ax.set_xlabel("时间 / s"); ax.set_ylabel("Δ浓度 / µM")
ax.legend(frameon=False, ncol=2, loc="upper right"); ax.grid(axis="y")
ax.text(.012, .04, "这是可写进规格的仪器噪声底", transform=ax.transAxes,
        fontsize=8.5, color=INK3)

# (2) 3cm vs 4cm -------------------------------------------------------------
ax = fig.add_subplot(gs[0, 2])
v3, v4 = r3.d_hbo.std(ddof=1), r4.d_hbo.std(ddof=1)
b = ax.bar(["3 cm\n(MBLL 用这路)", "4 cm"], [v3, v4],
           color=[OK, BAD], width=.56, edgecolor="none")
for rect, v in zip(b, [v3, v4]):
    ax.text(rect.get_x() + rect.get_width() / 2, v, f"  {v:.3f}",
            ha="center", va="bottom", fontsize=9.5, color=INK)
ax.set_title(f"② ΔHbO 噪声：4cm 差 {v4/v3:.1f} 倍", loc="left", fontweight="bold")
ax.set_ylabel("std / µM"); ax.grid(axis="y")
ax.set_ylim(0, v4 * 1.22)

# (3) 频谱 + 带边 ------------------------------------------------------------
ax = fig.add_subplot(gs[1, :2])
m = (f > 0.012) & (f <= 0.33)
ax.semilogy(f[m], P[m] / P[m].max(), lw=.9, color=INK2)
ax.axvspan(0.01, 0.05, color=OK, alpha=.13)
ax.axvspan(0.05, 0.10, color=BAD, alpha=.13)
ax.axvline(fpk, color=BAD, lw=1.3, ls="--")
ax.annotate(f"{fpk:.4f} Hz 窄带干扰（周期 {1/fpk:.1f} s）\n"
            "暗时隙里也有 → 电路/环境，非光学",
            xy=(fpk, 1.0), xytext=(fpk + 0.045, .30), fontsize=8.5, color=BAD,
            arrowprops=dict(arrowstyle="-", color=BAD, lw=.9))
lo_y = (P[m] / P[m].max()).min() * 1.6
ax.text(0.030, lo_y, "定版带\n0.01~0.05", ha="center", fontsize=8.5,
        color="#117a55")
ax.text(0.075, lo_y, "原定多出的\n0.05~0.10", ha="center", fontsize=8.5,
        color=BAD)
ax.set_title("③ ΔOD 频谱（770nm/3cm，已高通 0.01 去漂移）"
             "—— 原定 0.1 Hz 上限正好把干扰收进来",
             loc="left", fontweight="bold")
ax.set_xlabel("频率 / Hz"); ax.set_ylabel("归一功率"); ax.set_xlim(0, .33)
ax.grid(axis="y")

# (4) 噪声底 + 干扰占比 vs 上限 ----------------------------------------------
ax = fig.add_subplot(gs[1, 2])
x = np.arange(len(cuts))
cols = [OK if c <= 0.05 else (INK3 if c < 0.1 else BAD) for c in cuts]
b = ax.bar(x, sds, color=cols, width=.56, edgecolor="none")
for rect, sd, rr_ in zip(b, sds, resid):
    ax.text(rect.get_x() + rect.get_width() / 2, sd,
            f"{sd:.3f}\n干扰 {rr_:.0f}%", ha="center", va="bottom",
            fontsize=8.5, color=INK)
ax.set_xticks(x); ax.set_xticklabels([f"{c:.2f}" for c in cuts])
ax.set_title("④ 噪声底 vs 带通上限", loc="left", fontweight="bold")
ax.set_xlabel("带通上限 / Hz"); ax.set_ylabel("ΔHbO std / µM")
ax.grid(axis="y"); ax.set_ylim(0, max(sds) * 1.42)
ax.text(.03, .88, "0.10 噪声高 1.5 倍、\n21% 功率是干扰 → 取 0.05",
        transform=ax.transAxes, fontsize=8.5, color=BAD, va="top")
ax.text(.03, .60, "0.03 更静但切掉\n快的去饱和事件",
        transform=ax.transAxes, fontsize=8.5, color=INK3, va="top")

fig.suptitle("MBLL 解码器验证（第 8 条）　真实归档 "
             f"{path.name}　波长 670/770　2026-10-22",
             x=.055, ha="left", fontsize=13, fontweight="bold")
fig.text(.985, .952,
         "注：corr(ΔHbO,ΔHbR)=−0.897 是两波长下的数学必然，不是生理证据",
         ha="right", fontsize=8.5, color=INK3)

out = ROOT / "docs" / "验证图" / "MBLL解码器验证.png"
fig.savefig(out, dpi=170)
print(f"\n已写出 {out}")
