# -*- coding: utf-8 -*-
"""标定数据的三个维度 —— 12 块 × 5 波长 × 2 距离。

    python -m srs_cal.fig_grid

界面上只有 12 个按钮，容易误以为标定是二维的。实际上每按一次按钮，
采到的是 5 波长 × 2 距离共 10 个格子；而 G(λ) 是**逐波长各拟一次**的，
所以每个波长都有自己的一张 12 点 (μa, μs′) 网格，位置还各不相同。
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

from fnirs_pipeline.mbll_core import hemoglobin_extinctions      # noqa: E402
from srs_cal import hardware as hw                               # noqa: E402
from srs_cal import phantom                                      # noqa: E402

OUT_DIR = _HERE.parent.parent / "docs" / "验证图"
SURF, INK, INK2, INK3, RULE = "#fcfcfb", "#0b0b0b", "#52514e", "#8b8a84", "#e3e2dd"
OK, BAD, NEU, WARN = "#1baf7a", "#eb6834", "#2a78d6", "#d9a521"
plt.rcParams.update({
    "font.sans-serif": ["Microsoft YaHei", "SimHei", "DengXian", "sans-serif"],
    "axes.unicode_minus": False, "figure.facecolor": SURF, "axes.facecolor": SURF,
    "savefig.facecolor": SURF, "text.color": INK, "axes.labelcolor": INK2,
    "axes.edgecolor": RULE, "xtick.color": INK2, "ytick.color": INK2,
    "axes.titlesize": 11.5, "axes.labelsize": 10, "legend.fontsize": 8.6,
    "xtick.labelsize": 9, "ytick.labelsize": 9, "axes.spines.top": False,
    "axes.spines.right": False, "grid.color": RULE, "grid.linewidth": .7})

LN10 = np.log(10.0)
WL = np.array([670., 770., 810., 850., 950.])
WL_COLORS = {670.: "#c0392b", 770.: "#d9a521", 810.: "#1baf7a",
             850.: "#2a78d6", 950.: "#7d4bbf"}


def main() -> int:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    mua, musp = phantom.interpolate_to(WL)

    fig = plt.figure(figsize=(13.2, 6.6))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.25, 1.0],
                          left=.062, right=.975, top=.80, bottom=.095, wspace=.22)
    fig.text(.062, .925, "标定数据是三维的：12 块 × 5 波长 × 2 距离 = 120 个格子",
             fontsize=17, fontweight="bold", color=INK)
    fig.text(.062, .868,
             "界面上只有 12 个按钮，是因为只有「块」这一维需要人动手换；"
             "波长由设备自己扫，两个距离同时读",
             fontsize=9.8, color=INK2)

    # ---- (a) (μa, μs′) 平面：每个波长一张网格 ----
    ax = fig.add_subplot(gs[0, 0])
    for a, w in enumerate(WL):
        c = WL_COLORS[w]
        ax.scatter(mua[:, a], musp[:, a], s=46, color=c, zorder=3,
                   edgecolor="w", linewidth=.8, label=f"{w:.0f} nm")
    # 同一块在五个波长间连线，显示"一块 = 一条轨迹"
    for i in range(12):
        ax.plot(mua[i], musp[i], "-", lw=1.0, color=INK3, alpha=.55, zorder=2)
        ax.annotate(phantom.BLOCK_CODES[i], (mua[i, 0], musp[i, 0]),
                    fontsize=8.2, color=INK2, xytext=(7, 3),
                    textcoords="offset points", zorder=4)
    # 组织落点
    E = hemoglobin_extinctions(list(map(float, WL)), hw.EXTINCTION_TABLE)
    mua_t = (E[:, 0] * 0.70 + E[:, 1] * 0.30) * 60e-6 * LN10
    musp_t = hw.MUSP_REF_800 * (WL / 800.) ** (-hw.MUSP_EXPONENT)
    ax.plot(mua_t, musp_t, "*-", ms=17, lw=2.0, color=INK, zorder=6,
            markeredgecolor="w", markeredgewidth=.9, label="成人前额组织")
    ax.set_xlabel("吸收 μa (cm$^{-1}$)")
    ax.set_ylabel("约化散射 μs′ (cm$^{-1}$)")
    ax.set_title("(a) 每个波长都有自己的一张 12 点网格 —— 位置还不一样", loc="left")
    ax.legend(frameon=False, ncol=2, loc="upper left")
    ax.grid(alpha=.5)
    ax.text(.985, .30,
            "灰线＝同一块体模在五个波长间的轨迹\n"
            "波长越长 → 散射越弱、吸收越低 → 整张网格往左下平移\n"
            "组织（★）落在 B 行附近，但 μa 比体模最高档还高",
            transform=ax.transAxes, fontsize=8.6, color=INK2, ha="right", va="bottom",
            bbox=dict(boxstyle="round,pad=0.45", fc="#f4f6fa", ec=RULE, lw=.8))

    # ---- (b) 维度示意 ----
    bx = fig.add_subplot(gs[0, 1]); bx.axis("off")
    bx.set_title("(b) 三个维度在软件里分别落在哪", loc="left")

    rows = [
        ("维度", "大小", "界面上在哪", "谁来换", None),
        ("体模块", "12", "12 个按钮（A2…D4）", "你，手动换块", NEU),
        ("波长", "5", "实时表的 5 行", "设备自动扫", OK),
        ("源探距离", "2", "实时表的 2 列", "两个探测器同时读", OK),
    ]
    y = .90
    for i, (c1, c2, c3, c4, col) in enumerate(rows):
        fw = "bold" if i == 0 else "normal"
        bx.text(.00, y, c1, fontsize=10, fontweight=fw,
                color=INK if i == 0 else (col or INK2), transform=bx.transAxes)
        bx.text(.20, y, c2, fontsize=10, fontweight=fw,
                color=INK if i == 0 else INK2, transform=bx.transAxes)
        bx.text(.30, y, c3, fontsize=9.4, fontweight=fw,
                color=INK if i == 0 else INK2, transform=bx.transAxes)
        bx.text(.67, y, c4, fontsize=9.4, fontweight=fw,
                color=INK if i == 0 else INK3, transform=bx.transAxes)
        if i == 0:
            bx.plot([0, 1], [y - .028, y - .028], color=RULE, lw=1,
                    transform=bx.transAxes, clip_on=False)
        y -= .105

    bx.text(.00, y - .02,
            "按一次按钮 = 采下 5 × 2 = 10 个格子\n"
            "采完 12 块 = 120 个格子",
            fontsize=10, color=INK, transform=bx.transAxes, va="top",
            fontweight="bold")
    y -= .20

    bx.text(.00, y,
            "G(λ) 怎么算\n"
            "──────────────────────────────────────────────\n"
            "每个波长单独拟一次：\n"
            "    G(λ) = 该波长下 12 块的 (实测斜率 − 预测斜率) 平均\n"
            "斜率由该波长的 3cm / 4cm 两个格子定出。\n"
            "所以 670nm 的 G 只用 670nm 那 24 个格子，与别的波长无关 ——\n"
            "四个在用波长各得一个数，共 4 个标定常数。\n\n"
            "这也是为什么网格必须二维：单靠一行（只变 μa）或一列（只变 μs′）\n"
            "都定不住 G —— §4.3 实测，2×2 的斜率噪声是 3×4 的 2.3 倍。",
            fontsize=9.2, color=INK2, transform=bx.transAxes, va="top",
            bbox=dict(boxstyle="round,pad=0.55", fc="#fff6e8", ec=WARN, lw=1))

    out = OUT_DIR / "标定三个维度.png"
    fig.savefig(out, dpi=165)
    print(f"已输出 {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
