# -*- coding: utf-8 -*-
"""买的信号线 vs 自制线：问题不在噪声，在 4cm 通道被带偏。

产物：docs/验证图/信号线对比.png
数据：software/srs_cal/标定采集/标定采集_20261027_{150217,151756}.xlsx
"""
import sys
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
plt.rcParams["axes.unicode_minus"] = False
# ⚠️ U+2082（₂）不在 Microsoft YaHei 里，一律避开下标

WLS = [670, 770, 810, 850, 950]
SRC = {"买的线": "150217", "自制线": "151756"}
COL = {"买的线": "#c0392b", "自制线": "#1b7837"}

D, NET, DARK = {}, {}, {}
for lab, nm in SRC.items():
    d = pd.read_excel(ROOT / f"software/srs_cal/标定采集/标定采集_20261027_{nm}.xlsx",
                      "逐帧")
    D[lab] = d
    DARK[lab] = {r: d[(d.wavelength_nm == 0) & (d.distance_cm == r)].adc_raw.median()
                 for r in (3, 4)}
    NET[lab] = {(w, r): d[(d.wavelength_nm == w) & (d.distance_cm == r)].adc_raw.median()
                - DARK[lab][r] for w in WLS for r in (3, 4)}

fig = plt.figure(figsize=(13.6, 9.0), dpi=130)
fig.suptitle("换上买的信号线后：4cm 通道被 3cm 带偏，不是噪声\n"
             "A2 体模，档位 255，两次采集相隔 15 分钟（15:02 买的线 / 15:17 自制线）",
             fontsize=13, fontweight="bold", y=0.975)

# ① 扣暗后的光强
ax = fig.add_subplot(2, 2, 1)
x = np.arange(len(WLS)); w = 0.2
for i, lab in enumerate(SRC):
    for j, r in enumerate((3, 4)):
        v = [NET[lab][(wl, r)] for wl in WLS]
        ax.bar(x + (i * 2 + j - 1.5) * w, v, w, color=COL[lab],
               alpha=1.0 if r == 3 else 0.45,
               label=f"{lab} {r}cm" if True else None)
ax.axhline(0, color="k", lw=1)
ax.set_xticks(x); ax.set_xticklabels([f"{wl}" for wl in WLS])
ax.set_xlabel("波长 (nm)"); ax.set_ylabel("扣暗后 ADC")
ax.set_title("① 买的线：810/4cm 与 850/4cm 扣暗后为**负**\n"
             "   负的光强不可能，这不是噪声能造成的", fontsize=10.5, loc="left")
ax.legend(fontsize=8, ncol=2)
ax.grid(alpha=.3, axis="y")

# ② 810/3cm 一致 → 光路没变
ax = fig.add_subplot(2, 2, 2)
rat = [NET["买的线"][(wl, 3)] / NET["自制线"][(wl, 3)] for wl in WLS]
rat4 = [NET["买的线"][(wl, 4)] / NET["自制线"][(wl, 4)] for wl in WLS]
ax.plot(x, rat, "o-", color="#2166ac", lw=2, ms=8, label="3cm 买/自")
ax.plot(x, rat4, "s-", color="#c0392b", lw=2, ms=8, label="4cm 买/自")
ax.axhline(1.0, color="#555", ls="--", lw=1)
ax.annotate("810/3cm 两次只差 0.3%\n→ 探头、体模、档位都没变",
            xy=(2, rat[2]), xytext=(2.3, 2.0), fontsize=9,
            arrowprops=dict(arrowstyle="->", color="#2166ac"))
ax.set_xticks(x); ax.set_xticklabels([f"{wl}" for wl in WLS])
ax.set_xlabel("波长 (nm)"); ax.set_ylabel("买的线 / 自制线")
ax.set_title("② 光路没变，变的是电路\n   否则所有通道会一起变", fontsize=10.5, loc="left")
ax.legend(fontsize=9); ax.grid(alpha=.3)

# ③ 3cm 与 4cm 的逐帧相关
ax = fig.add_subplot(2, 2, 3)
for i, lab in enumerate(SRC):
    cs = []
    for wl in WLS:
        d = D[lab]
        a = d[(d.wavelength_nm == wl) & (d.distance_cm == 3)].adc_raw.to_numpy(float)
        b = d[(d.wavelength_nm == wl) & (d.distance_cm == 4)].adc_raw.to_numpy(float)
        n = min(len(a), len(b)); a, b = a[:n], b[:n]
        t = np.arange(n)
        da = a - np.polyval(np.polyfit(t, a, 1), t)
        db = b - np.polyval(np.polyfit(t, b, 1), t)
        cs.append(np.corrcoef(da, db)[0, 1])
    ax.bar(x + (i - 0.5) * 0.35, cs, 0.35, color=COL[lab], label=lab)
ax.axhline(0, color="k", lw=1)
ax.set_xticks(x); ax.set_xticklabels([f"{wl}" for wl in WLS])
ax.set_xlabel("波长 (nm)"); ax.set_ylabel("corr(3cm, 4cm) 逐帧")
ax.set_title("③ 相关**变号**：自制线全为正（共模，正常）\n"
             "   买的线 770/810/850 变负 —— 一个涨另一个就跌",
             fontsize=10.5, loc="left")
ax.legend(fontsize=9); ax.grid(alpha=.3, axis="y")

# ④ SRS 入口：μeff
ax = fig.add_subplot(2, 2, 4)
LN10 = np.log(10.0)
for i, lab in enumerate(SRC):
    mu = []
    for wl in WLS:
        I3, I4 = NET[lab][(wl, 3)], NET[lab][(wl, 4)]
        mu.append(np.nan if (I3 <= 0 or I4 <= 0)
                  else LN10 * (np.log10(I3) - np.log10(I4)) - 2.0 / 3.5)
    ax.bar(x + (i - 0.5) * 0.35, mu, 0.35, color=COL[lab], label=lab)
    for xi, m in zip(x, mu):
        if np.isnan(m):
            ax.text(xi + (i - 0.5) * 0.35, 0.03, "负光强\n算不了", ha="center",
                    fontsize=7.5, color=COL[lab])
ax.axhline(0, color="k", lw=1.2)
ax.set_xticks(x); ax.set_xticklabels([f"{wl}" for wl in WLS])
ax.set_xlabel("波长 (nm)"); ax.set_ylabel("μeff (1/cm)")
ax.set_title("④ SRS 的入口量 μeff：买的线 670 nm 为**负**\n"
             "   → μa 无解 → 不出 TOI（归档实测 33 点里 28 点报此错）",
             fontsize=10.5, loc="left")
ax.legend(fontsize=9); ax.grid(alpha=.3, axis="y")

fig.text(0.012, 0.012,
         "结论：买的线使 4cm 通道的暗电平从 +8,590 移到 −25,547，且读数随同一时隙的 3cm 反向起伏 —— "
         "两个探测通道之间出现了耦合。这是硬件接线问题，算法层面无法补救（负的光强取不了对数）。",
         fontsize=9.5, color="#333")

fig.tight_layout(rect=[0, 0.032, 1, 0.945])
out = HERE / "验证图" / "信号线对比.png"
out.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(out, facecolor="white")
print("已保存", out)
for lab in SRC:
    print(f"  {lab}: 暗 3cm {DARK[lab][3]:>9.0f}  4cm {DARK[lab][4]:>9.0f}")
