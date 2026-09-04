# -*- coding: utf-8 -*-
"""两探测器探头的距离选型 —— 用 MC 分层头模型给出最优 (ρ₁, ρ₂)。

背景
    硬件定为【1 光源 + 2 探测器】，故只有 2 个距离：
      · 双斜率（需 2 光源）不可用 -> 探测器间增益必须靠体模矩阵标定
      · 三点残差自检不可用       -> 改用五波长的光谱残差做自检
      · 无短距 -> 无 SSR、无头皮参考
    剩下唯一还能优化的就是这两个距离怎么选，且探头定版后改不动。

两个方向互相拉扯
    基线 Δρ 越宽：斜率噪声 ∝ 1/Δρ 越小；渐近近似的系统偏差越小；深度对比越好
    基线 Δρ 越宽：远距光子急剧减少 -> 信噪比变差

方法
    每个波长跑一次 MC，一次性放置 CAND_MM 里所有候选距离的探测器
    （MC 只跑一次，任意距离对都靠 ppath 重加权评估，不必重跑）。
    对每个候选对给出：
      脑灵敏度      ∂读数/∂脑StO₂        越大越好
      颅外灵敏度    ∂读数/∂颅外StO₂      越小越好
      脑/颅外比                          越大越好
      远距光子数                          决定信噪比够不够
      相对噪声      归一到 (30,40) 基准   越小越好
      品质因数      脑灵敏度 / 相对噪声   综合排序用

⚠️ 与 srs_mc_layered.py 共用光学参数与层厚假设，故同样继承那边的限定：
   Strangman 表 μs/μs′ 的解读未核对原文；单一几何、无误差棒。
   本脚本给的是【相对排序】，不是绝对性能预测。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent))
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

import srs_mc_layered as M                                     # noqa: E402
from fnirs_pipeline.srs import (MUSP_EXPONENT, MUSP_REF_800,   # noqa: E402
                                mua_from_mueff, mua_to_toi,
                                musp_power_law, slope_to_mueff)

LN10 = np.log(10.0)
CAND_MM = np.array([20., 25., 30., 35., 40., 45., 50., 55.])
BASE_PAIR = (30., 40.)          # 噪声归一基准
NPHOTON = 2e9                   # 比 srs_mc_layered 多一倍：55mm 处光子很少


def run_all(scalp_mm: float, nphoton: float):
    """每波长一次 MC，探测器一次性放满 CAND_MM。返回 (det_id, ppath) 列表。"""
    vol = M.build_volume(scalp_mm, M.LAYERS["skull"], M.LAYERS["csf"], M.LAYERS["gray"])
    saved_rho = M.RHO_MM
    M.RHO_MM = CAND_MM                       # run_mc 按 M.RHO_MM 摆探测器
    try:
        out, t0 = [], time.time()
        for wl in M.WL:
            d, p = M.run_mc(vol, M.musp_at(wl), nphoton=nphoton)
            out.append((d, p))
            n = [int((d == i + 1).sum()) for i in range(CAND_MM.size)]
            print("    %d nm  各距离探测光子 %s" % (wl, n))
        print("  MC 用时 %.0f s\n" % (time.time() - t0))
    finally:
        M.RHO_MM = saved_rho
    return out


def toi_of(runs, pair_idx, sto2_layers, thb) -> float:
    """给定两个距离，两点斜率 -> μeff -> μa -> TOI。不走 SrsGeometry（它要求 ≥3）。"""
    i, j = pair_idx
    rho = np.array([CAND_MM[i], CAND_MM[j]]) / 10.0          # cm
    A = np.empty((M.WL.size, 2))
    for k, wl in enumerate(M.WL):
        det, pp = runs[k]
        mua = M.mua_of(sto2_layers, thb, wl)
        w = np.exp(-(pp * mua[None, :pp.shape[1]]).sum(axis=1))
        I = np.array([w[det == i + 1].sum(), w[det == j + 1].sum()])
        if np.any(I <= 0):
            return np.nan
        A[k] = -np.log10(I)
    slope = (A[:, 1] - A[:, 0]) / (rho[1] - rho[0])          # 两点斜率
    mueff = slope_to_mueff(slope, rho)
    if np.any(mueff <= 0):
        return np.nan
    musp = musp_power_law(M.WL, MUSP_REF_800, MUSP_EXPONENT)
    return mua_to_toi(mua_from_mueff(mueff, musp), M.WL, "wray")[0]


def sensitivity(runs, pair_idx, thb, which: str) -> float:
    """脑或颅外的 StO2 从 60 扫到 80，回归斜率 = ∂读数/∂真值。"""
    xs, ys = [], []
    for s in (0.60, 0.65, 0.70, 0.75, 0.80):
        lay = np.full(5, 0.70)
        if which == "brain":
            lay[3] = lay[4] = s
        else:
            lay[0] = lay[1] = s
        v = toi_of(runs, pair_idx, lay, thb)
        if np.isfinite(v):
            xs.append(s * 100)
            ys.append(v)
    if len(xs) < 3:
        return np.nan
    return float(np.polyfit(xs, ys, 1)[0])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scalp", type=float, default=M.LAYERS["scalp"])
    ap.add_argument("--nphoton", type=float, default=NPHOTON)
    a = ap.parse_args()

    print("=" * 90)
    print("两探测器探头 —— 距离选型")
    print("=" * 90)
    print("  层厚 mm：头皮 %.1f  颅骨 %.1f  CSF %.1f  灰质 %.1f  白质 填满"
          % (a.scalp, M.LAYERS["skull"], M.LAYERS["csf"], M.LAYERS["gray"]))
    print("  候选距离 %s mm   每波长 %.0e 光子\n"
          % (list(CAND_MM.astype(int)), a.nphoton))

    runs = run_all(a.scalp, a.nphoton)
    thb = M.calibrate_thb()
    counts = np.array([[int((runs[k][0] == i + 1).sum()) for i in range(CAND_MM.size)]
                       for k in range(M.WL.size)]).min(axis=0)     # 各距离最差波长的计数

    rows = []
    for i in range(CAND_MM.size):
        for j in range(i + 1, CAND_MM.size):
            if CAND_MM[j] - CAND_MM[i] < 10:        # 基线 <10mm 噪声放大过大，不考虑
                continue
            sb = sensitivity(runs, (i, j), thb, "brain")
            se = sensitivity(runs, (i, j), thb, "extra")
            if not np.isfinite(sb) or not np.isfinite(se):
                continue
            # 斜率噪声 ∝ sqrt(1/N₁ + 1/N₂) / Δρ，远距主导
            noise = np.sqrt(1 / counts[i] + 1 / counts[j]) / (CAND_MM[j] - CAND_MM[i])
            rows.append(dict(i=i, j=j, r1=CAND_MM[i], r2=CAND_MM[j],
                             sb=sb, se=se, ratio=sb / se if se > 0 else np.inf,
                             nfar=counts[j], noise=noise))

    base = next((r["noise"] for r in rows
                 if (r["r1"], r["r2"]) == BASE_PAIR), rows[0]["noise"])
    for r in rows:
        r["rel"] = r["noise"] / base
        r["fom"] = r["sb"] / r["rel"]

    rows.sort(key=lambda r: -r["fom"])
    print("  按品质因数（脑灵敏度 / 相对噪声）排序：\n")
    print("   ρ₁   ρ₂   Δρ   脑灵敏度  颅外灵敏度  脑/颅外  远距光子   相对噪声  品质因数")
    for r in rows:
        print("  %3.0f  %3.0f  %3.0f    %6.4f    %6.4f   %6.2f  %9d   %7.2f   %7.4f"
              % (r["r1"], r["r2"], r["r2"] - r["r1"], r["sb"], r["se"],
                 r["ratio"], r["nfar"], r["rel"], r["fom"]))

    print("\n  说明：")
    print("    · 相对噪声归一到 (%.0f, %.0f)；品质因数越大越好" % BASE_PAIR)
    print("    · 远距光子数是 %.0e 发射光子下的 MC 计数，只用于横向比较，" % a.nphoton)
    print("      不代表真实硬件信噪比（取决于光源功率、探测器面积、积分时间）")
    print("    · 脑/颅外比同时要看：比值高但脑灵敏度低的组合，抗头皮干扰强但量程压缩重")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
