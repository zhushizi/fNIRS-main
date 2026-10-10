# -*- coding: utf-8 -*-
"""srs_cal 验证图 —— 输出到 docs/验证图/。

    python -m srs_cal.figures
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                  # noqa: E402
import numpy as np                                               # noqa: E402

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from fnirs_pipeline.mbll_core import hemoglobin_extinctions      # noqa: E402
from fnirs_pipeline.srs import compute_toi                       # noqa: E402
from srs_cal import hardware as hw                               # noqa: E402
from srs_cal import phantom                                      # noqa: E402
from srs_cal.calibrate import calibrate_gain                     # noqa: E402
from srs_cal.forward import attenuation_matrix                   # noqa: E402

OUT_DIR = _HERE.parent.parent / "docs" / "验证图"

SURF, INK, INK2, INK3, RULE = "#fcfcfb", "#0b0b0b", "#52514e", "#8b8a84", "#e3e2dd"
OK, BAD, NEU, WARN = "#1baf7a", "#eb6834", "#2a78d6", "#d9a521"
plt.rcParams.update({
    "font.sans-serif": ["Microsoft YaHei", "SimHei", "DengXian", "sans-serif"],
    "axes.unicode_minus": False, "figure.facecolor": SURF, "axes.facecolor": SURF,
    "savefig.facecolor": SURF, "text.color": INK, "axes.labelcolor": INK2,
    "axes.edgecolor": RULE, "xtick.color": INK2, "ytick.color": INK2,
    "axes.titlesize": 11, "axes.labelsize": 9.5, "legend.fontsize": 8.5,
    "xtick.labelsize": 9, "ytick.labelsize": 9, "axes.spines.top": False,
    "axes.spines.right": False, "grid.color": RULE, "grid.linewidth": .7})

LN10 = np.log(10.0)
WL, RHO = hw.WAVELENGTHS_NM, hw.RHO_CM
N_PH = phantom.REFRACTIVE_INDEX_ASSUMED


def main() -> int:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    mua_d, musp_d = phantom.interpolate_to(WL)
    fig, axes = plt.subplots(2, 2, figsize=(12.6, 8.4))
    fig.subplots_adjust(top=.845, hspace=.36, wspace=.24,
                        left=.072, right=.975, bottom=.075)
    fig.text(.062, .952, "体模 P3137034 标定可行性：认证 4 波长 → 器件 4 波长",
             fontsize=17, fontweight="bold", color=INK)
    fig.text(.062, .906,
             "12 块均质圆柱，认证于 685/808/830/975 nm；器件 670/770/810/850 nm"
             "（950 nm 因水吸收占 64% 已剔出反演）",
             fontsize=9.6, color=INK2)

    # (a) 认证谱与插值
    ax = axes[0, 0]
    grid = np.linspace(640, 1000, 200)
    for i in (0, 3, 6, 9):      # A2/B2/C2/D2，四个散射档各取一块
        a, b = phantom.fit_musp_powerlaw(phantom.CERT_WL_NM, phantom.MUSP_CERT[i])
        ax.plot(grid, phantom.eval_musp(a, b, grid), "-", lw=1.5, color=NEU, alpha=.55)
        ax.plot(phantom.CERT_WL_NM, phantom.MUSP_CERT[i], "o", ms=5.5,
                color=INK, zorder=3)
        ax.plot(WL, musp_d[i], "D", ms=5, color=BAD, zorder=4)
        ax.text(1004, phantom.eval_musp(a, b, 1000), phantom.BLOCK_CODES[i],
                fontsize=8.5, color=INK3, va="center")
    ax.axvspan(640, phantom.CERT_WL_NM.min(), color=WARN, alpha=.10)
    ax.text(655, ax.get_ylim()[1] * .93, "670 外推区", fontsize=8.4, color=WARN)
    ax.plot([], [], "o", color=INK, label="认证值（685/808/830/975）")
    ax.plot([], [], "-", color=NEU, label="幂律 μs′=a(λ/800)^−b")
    ax.plot([], [], "D", color=BAD, label="插到器件波长")
    ax.set_xlabel("波长 (nm)"); ax.set_ylabel("μs′ (cm$^{-1}$)")
    ax.set_title("(a) 散射谱：幂律拟合 4 点，插到器件波长", loc="left")
    ax.legend(frameon=False, loc="upper right"); ax.grid(alpha=.5)
    ax.set_xlim(640, 1020)

    # (b) 留一交叉验证
    ax = axes[0, 1]
    loo = phantom.loocv_error()
    x = np.arange(len(loo)); w = .36
    ea = [v[0] * 100 for v in loo.values()]
    es = [v[1] * 100 for v in loo.values()]
    quad = [74.77, 6.58, 7.29, 115.45]          # selftest 实测，见 phantom.fit_mua_linear
    ax.bar(x - w / 2, ea, w, color=NEU, label="μa 线性（采用）")
    ax.bar(x + w / 2, es, w, color=OK, label="μs′ 幂律（采用）")
    for i, q in enumerate(quad):
        ax.plot([x[i] - w / 2], [min(q, 19)], "v", ms=8, color=BAD, zorder=5)
        if q > 19:
            ax.text(x[i] - w / 2, 19.4, f"{q:.0f}%", fontsize=8.2, color=BAD,
                    ha="center", fontweight="bold")
    ax.plot([], [], "v", color=BAD, label="μa 二次（弃用，端点崩）")
    ax.set_xticks(x); ax.set_xticklabels([f"{w_:.0f}" for w_ in loo])
    ax.set_ylim(0, 22); ax.set_xlabel("留出的认证波长 (nm)")
    ax.set_ylabel("留一预测误差 (%)")
    ax.set_title("(b) 插值质量：μa 必须用线性，二次在端点崩", loc="left")
    ax.legend(frameon=False, loc="upper center"); ax.grid(alpha=.5, axis="y")

    # (c) 12 块的信号跨度
    ax = axes[1, 0]
    for i, code in enumerate(phantom.BLOCK_CODES):
        A = attenuation_matrix(mua_d[i], musp_d[i], RHO, N_PH)
        c = [NEU, OK, WARN, BAD][i // 3]
        ax.plot([i, i], [A[:, 0].min(), A[:, 0].max()], lw=6, color=c, alpha=.35,
                solid_capstyle="round")
        ax.plot([i, i], [A[:, 1].min(), A[:, 1].max()], lw=6, color=c,
                solid_capstyle="round")
    for j, (lbl, c) in enumerate(zip(["A μs′≈5", "B μs′≈12", "C μs′≈18", "D μs′≈25"],
                                     [NEU, OK, WARN, BAD])):
        ax.plot([], [], lw=6, color=c, label=lbl)
    ax.axhline(np.log10(1e12 / 2000), color=INK, ls="--", lw=1.2)
    ax.text(.3, np.log10(1e12 / 2000) + .12,
            "MIN_COUNTS=2000 对应上限（按仿真 scale=1e12）", fontsize=8.3, color=INK)
    ax.set_xticks(range(12)); ax.set_xticklabels(phantom.BLOCK_CODES, fontsize=8.4)
    ax.set_ylabel("衰减 A (OD)")
    ax.set_title("(c) 12 块的动态范围（浅=3cm，深=4cm）—— 真机须实测 A2 与 D4",
                 loc="left")
    ax.legend(frameon=False, ncol=4, loc="lower right"); ax.grid(alpha=.5, axis="y")

    # (d) 闭环：标定前后
    ax = axes[1, 1]
    rng = np.random.default_rng(0)
    G_true = np.array([0.020, -0.012, 0.008, -0.018])
    E = hemoglobin_extinctions(list(map(float, WL)), hw.EXTINCTION_TABLE)
    musp_t = hw.MUSP_REF_800 * (WL / 800.0) ** (-hw.MUSP_EXPONENT)

    def toi_at(so2, gain, G):
        mua = (E[:, 0] * so2 + E[:, 1] * (1 - so2)) * 60e-6 * LN10
        A = attenuation_matrix(mua, musp_t, RHO, hw.N_TISSUE)
        if gain is not None:
            A = A + gain[:, None] * (RHO - RHO.mean())[None, :]
        return compute_toi(10 ** (-A) * 1e12, hw.geometry(gain_correction=G)).toi_raw

    I = np.zeros((12, WL.size, RHO.size))
    for i in range(12):
        A = attenuation_matrix(mua_d[i], musp_d[i], RHO, N_PH)
        A = A + G_true[:, None] * (RHO - RHO.mean())[None, :]
        A = A + rng.normal(0, 0.002, A.shape)
        I[i] = 10 ** (-A) * 1e12
    G_fit = calibrate_gain(I, WL, RHO, refractive_index=N_PH,
                           baseline_dispersion=np.full(WL.size, 0.004)).gain_correction

    s = np.linspace(0.35, 0.90, 24)
    ref = np.array([toi_at(v, None, None) for v in s])
    unc = np.array([toi_at(v, G_true, None) for v in s])
    cal = np.array([toi_at(v, G_true, G_fit) for v in s])
    ax.plot(s * 100, ref, "-", lw=2.4, color=INK, label="真值链路（无增益误差）")
    ax.plot(s * 100, unc, "-", lw=1.9, color=BAD,
            label=f"不标定（偏 {np.mean(unc-ref):+.2f} %O$_2$）")
    ax.plot(s * 100, cal, "--", lw=2.0, color=OK,
            label=f"12 块标定后（偏 {np.mean(cal-ref):+.3f} %O$_2$）")
    ax.set_xlabel("真实 SO$_2$ (%)"); ax.set_ylabel("SRS 读数 TOI (%O$_2$)")
    ax.set_title("(d) 闭环验证：标定把增益误差拉回 0.1 %O$_2$ 以内", loc="left")
    ax.legend(frameon=False, loc="upper left"); ax.grid(alpha=.5)

    out = OUT_DIR / "SRS体模标定_P3137034.png"
    fig.savefig(out, dpi=165)
    print(f"已输出 {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
