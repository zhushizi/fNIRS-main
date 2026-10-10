# -*- coding: utf-8 -*-
"""噪声结构诊断图 —— 基于 B3 实测（20 帧/组合）。

    python -m srs_cal.fig_noise --xlsx srs_cal/标定采集/标定采集_YYYYMMDD_HHMMSS.xlsx

⚠️ 输入只有每组合的均值与标准差（10 个数），**不含原始帧**。
   这能判定噪声的【标度律】（加性 / 散粒 / 乘性），
   但**不能**判定噪声的【来源】（环境光 / 工频 / 串扰 / 驱动）——
   那需要暗电平试验和原始帧频谱。图中第 (b) 格给的是判别方法，不是结论。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                  # noqa: E402
import numpy as np                                               # noqa: E402

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))

from srs_cal import phantom                                      # noqa: E402
from srs_cal.forward import attenuation_matrix                   # noqa: E402

OUT_DIR = _HERE.parent.parent / "docs" / "验证图"
SURF, INK, INK2, INK3, RULE = "#fcfcfb", "#0b0b0b", "#52514e", "#8b8a84", "#e3e2dd"
OK, BAD, NEU, WARN = "#1baf7a", "#eb6834", "#2a78d6", "#d9a521"
plt.rcParams.update({
    "font.sans-serif": ["Microsoft YaHei", "SimHei", "DengXian", "sans-serif"],
    "axes.unicode_minus": False, "figure.facecolor": SURF, "axes.facecolor": SURF,
    "savefig.facecolor": SURF, "text.color": INK, "axes.labelcolor": INK2,
    "axes.edgecolor": RULE, "xtick.color": INK2, "ytick.color": INK2,
    "axes.titlesize": 11, "axes.labelsize": 9.5, "legend.fontsize": 8.3,
    "xtick.labelsize": 9, "ytick.labelsize": 9, "axes.spines.top": False,
    "axes.spines.right": False, "grid.color": RULE, "grid.linewidth": .7})

WL = np.array([670., 770., 810., 850., 950.])
RHO = np.array([3., 4.])
SEM_LIMIT = 0.007


def load(path: Path):
    from openpyxl import load_workbook
    ws = load_workbook(path, data_only=True)["明细"]
    M, S, N, blk = {}, {}, {}, None
    for r in ws.iter_rows(min_row=2, values_only=True):
        blk = r[0]
        M[(float(r[3]), float(r[4]))] = r[5]
        S[(float(r[3]), float(r[4]))] = r[6]
        N[(float(r[3]), float(r[4]))] = r[-1]
    return M, S, N, blk


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--xlsx", type=Path,
                    default=_HERE / "标定采集" / "标定采集_20261009_112330.xlsx")
    a = ap.parse_args(argv)
    M, S, N, blk = load(a.xlsx)
    nf = N[(670., 4.)]
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    g950 = np.mean([S[(950., d)] ** 2 / M[(950., d)] for d in RHO])   # 散粒系数
    floor = {d: np.mean([np.sqrt(max(S[(w, d)] ** 2 - g950 * M[(w, d)], 0))
                         for w in WL[:4]]) for d in RHO}

    fig, ax = plt.subplots(2, 2, figsize=(12.8, 8.8))
    fig.subplots_adjust(top=.845, hspace=.40, wspace=.25,
                        left=.072, right=.975, bottom=.075)
    fig.text(.062, .952, "噪声结构诊断：加性本底压住了暗通道",
             fontsize=17, fontweight="bold", color=INK)
    fig.text(.062, .906,
             f"数据：{blk} 块实测，{nf} 帧/组合。标度律可判，来源不可判 —— "
             "需暗电平试验与原始帧频谱（见右上）",
             fontsize=9.6, color=INK2)

    # (a) std vs mean
    A = ax[0, 0]
    x = np.logspace(4.2, 6.3, 100)
    A.loglog(x, np.sqrt(g950 * x), "-", lw=1.8, color=OK,
             label=f"散粒噪声限  std=√({g950:.1f}·I)")
    for d, mk, c in zip(RHO, ("o", "s"), (NEU, BAD)):
        A.loglog(x, np.sqrt(g950 * x + floor[d] ** 2), "--", lw=1.5, color=c,
                 label=f"散粒 + 加性本底 {floor[d]:,.0f}（{d:.0f}cm）")
        for w in WL:
            A.loglog(M[(w, d)], S[(w, d)], mk, ms=9, color=c,
                     mec="w", mew=1.2, zorder=4)
            A.annotate(f"{w:.0f}", (M[(w, d)], S[(w, d)]), fontsize=7.6,
                       color=INK2, xytext=(6, -3), textcoords="offset points")
    A.set_xlabel("信号均值（ADC 计数）"); A.set_ylabel("单帧标准差（计数）")
    A.set_title("(a) 950 nm 落在散粒限上，其余四路高出一个加性本底", loc="left")
    A.legend(frameon=False, loc="upper left"); A.grid(alpha=.5, which="both")

    # (b) 判别表
    B = ax[0, 1]; B.axis("off")
    B.set_title("(b) 噪声来源：本数据能排除什么、不能排除什么", loc="left")
    rows = [
        ("候选来源", "该来源的特征", "本数据", ""),
        ("散粒噪声（光子统计）", "std ∝ √I，无法再降", "部分成立", OK),
        ("环境光漏入", "加性，但应五路都有", "X 950 不符", BAD),
        ("暗电流 / 电子学本底", "加性，五路相同", "X 950 不符", BAD),
        ("工频 50Hz 混叠", "加性，五路相同", "X 950 不符", BAD),
        ("LED 驱动电流噪声", "乘性，std ∝ I", "X 乘性模型离散 41%", BAD),
        ("波长间串扰 / 余辉", "随邻道信号变，非常数", "? 需原始帧", WARN),
        ("时序抖动 / 建立时间不足", "与各路驻留时间有关", "? 需原始帧", WARN),
        ("各路增益档位不同", "折算到输入的本底不同", "? 需问硬件", WARN),
    ]
    yy = .93
    for i, (c1, c2, c3, col) in enumerate(rows):
        fw = "bold" if i == 0 else "normal"
        B.text(.00, yy, c1, fontsize=9.0, color=INK if i == 0 else INK2,
               fontweight=fw, transform=B.transAxes)
        B.text(.40, yy, c2, fontsize=8.6, color=INK if i == 0 else INK3,
               fontweight=fw, transform=B.transAxes)
        B.text(.78, yy, c3, fontsize=8.8, color=INK if i == 0 else col,
               fontweight="bold" if i else fw, transform=B.transAxes)
        if i == 0:
            B.plot([0, 1], [yy - .022, yy - .022], color=RULE, lw=1,
                   transform=B.transAxes, clip_on=False)
        yy -= .098
    B.text(.00, yy - .02,
           "→ 下一步：① 采暗电平（遮探头/关光源）。std≈5,100 → 与光无关，查电子学；\n"
           "   std 远小于 5,100 → 与光路/LED 有关。  ② 存原始帧做频谱：\n"
           "   50/100 Hz 线谱=工频，1/f=漂移，平坦=本底。  ③ 问硬件 950 那一路有何不同。",
           fontsize=8.7, color=INK, transform=B.transAxes, va="top",
           bbox=dict(boxstyle="round,pad=0.5", fc="#fff6e8", ec=WARN, lw=1))

    # (c) 12 块落点
    C = ax[1, 0]
    mua, musp = phantom.interpolate_to(WL)
    iB = phantom.BLOCK_CODES.index(blk)
    AB = attenuation_matrix(mua[iB], musp[iB], RHO, phantom.REFRACTIVE_INDEX)
    sig, lbl = [], []
    for i, c in enumerate(phantom.BLOCK_CODES):
        Ai = attenuation_matrix(mua[i], musp[i], RHO, phantom.REFRACTIVE_INDEX)
        sig.append(M[(670., 4.)] * 10 ** (-(Ai[0, 1] - AB[0, 1]))); lbl.append(c)
    sig = np.array(sig)
    rsd = np.sqrt(g950 * sig + floor[4.] ** 2) / sig
    good = sig / floor[4.] > 1.5
    C.bar(np.arange(12)[good], rsd[good] * 100, color=OK, label="可测")
    C.bar(np.arange(12)[~good], rsd[~good] * 100, color=BAD, label="淹没")
    C.axhline(SEM_LIMIT * 100 * np.sqrt(nf), color=INK, ls="--", lw=1.3)
    C.text(.15, SEM_LIMIT * 100 * np.sqrt(nf) * 1.25,
           f"{nf} 帧下的合格线（标准误 0.7%）", fontsize=8.4, color=INK)
    C.set_yscale("log"); C.set_xticks(range(12)); C.set_xticklabels(lbl, fontsize=8.4)
    C.set_ylabel("最暗组合 670nm/4cm 的单帧 RSD (%)")
    C.set_title("(c) 后果：12 块里只有 6 块露得出噪声", loc="left")
    C.legend(frameon=False); C.grid(alpha=.5, axis="y", which="both")

    # (d) 降噪收益
    D = ax[1, 1]
    facs = [1, 2, 5, 10, 20]
    nblk, nmin = [], []
    for f in facs:
        fl = floor[4.] / f
        ok_ = sig / fl > 1.5
        nblk.append(int(ok_.sum()))
        r = np.sqrt(g950 * sig + fl ** 2) / sig
        need = np.where(ok_, np.ceil((r / SEM_LIMIT) ** 2) * 10, 0)
        nmin.append(need[ok_].sum() / 10 / 60)
    D.bar(np.arange(len(facs)), nblk, .52, color=OK)
    D.set_ylabel("可测块数", color=OK); D.set_ylim(0, 14)
    D.set_xticks(range(len(facs)))
    D.set_xticklabels([("现状" if f == 1 else f"÷{f}") for f in facs])
    D.set_xlabel("加性本底降低倍数")
    for i, v in enumerate(nblk):
        D.text(i, v + .3, str(v), ha="center", fontsize=11, color=OK,
               fontweight="bold")
    D2 = D.twinx(); D2.spines["top"].set_visible(False)
    D2.plot(range(len(facs)), nmin, "o-", lw=2, ms=7, color=NEU, zorder=5)
    D2.set_ylabel("采完所有可测块的耗时（分钟）", color=NEU)
    D2.set_ylim(0, max(nmin) * 1.35)
    for i, v in enumerate(nmin):
        D2.annotate(f"{v:.0f}′", (i, v), fontsize=9, color=NEU, fontweight="bold",
                    xytext=(0, 9), textcoords="offset points", ha="center")
    D.set_title("(d) 本底降一档，可测块数就多一档", loc="left")
    D.grid(alpha=.5, axis="y")
    D.text(.02, .035,
           "耗时先升后降：本底一降，更暗的块够得着了，\n"
           "而新进来的块本身最暗、最吃帧数，于是总时间先被它们顶上去。",
           transform=D.transAxes, fontsize=8.3, color=INK2, va="bottom",
           bbox=dict(boxstyle="round,pad=0.4", fc="#f4f6fa", ec=RULE, lw=.8))

    out = OUT_DIR / "噪声结构诊断.png"
    fig.savefig(out, dpi=165)
    print(f"已输出 {out}")
    print(f"  散粒系数 g = {g950:.2f}（由 950 nm 两个距离定出）")
    for d in RHO:
        print(f"  {d:.0f}cm 加性本底 = {floor[d]:,.0f} counts")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
