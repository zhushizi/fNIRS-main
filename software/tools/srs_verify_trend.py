# -*- coding: utf-8 -*-
"""用公开血液体模数据验证 SRS 降氧/升氧【方向】是否与参考一致。

数据来源：Sudakou et al., Biomed. Opt. Express 2023, DOI 10.1364/BOE.492168
GitHub: https://github.com/asudakou/Analyzing_TD-NIRS
文件：DATA calculated/Blood_Exp1_LMA1_MuaMusp_vr1.mat（深层同质血液，6 轮降氧实验之一）

流程：
    实测 μa(t,λ)、μs′(t,λ)  [TD-NIRS 反演，非 CW 原始光强]
        → 正演 3/4/5 cm 衰减
        → srs.py 反演 TOI(t)
        → 与同一 μa 谱解出的 StO₂_ref(t) 比【方向】

⚠️ 不是血气仪金标准；μa 与 StO₂_ref 共用血红蛋白模型。
⚠️ 多距离由正演得到（与 B 组相同），不是实测多距 CW。

用法：
    python tools/srs_verify_trend.py
    python tools/srs_verify_trend.py --mat path/to/Blood_Exp1_LMA1_MuaMusp_vr1.mat
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import scipy.io as sio

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))
sys.path.insert(0, str(_HERE))
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from fnirs_pipeline.mbll_core import hemoglobin_extinctions
from fnirs_pipeline.srs import SrsGeometry, compute_toi
from srs_reference import attenuation

LN10 = np.log(10.0)
WL = np.array([850.0, 810.0, 770.0, 730.0, 700.0])
RHO = np.array([3.0, 4.0, 5.0])
N_TISSUE = 1.4
SCALE = 1e12
# Blood_main.m: wavelengths = 680:12.5:868，本数据为 14 个通道
WL_SRC = np.arange(680.0, 680.0 + 14 * 12.5, 12.5)
DEFAULT_MAT = _HERE / "sudakou_Blood_Exp1_LMA1.mat"
MAT_URL = (
    "https://github.com/asudakou/Analyzing_TD-NIRS/raw/main/"
    "DATA%20calculated/Blood_Exp1_LMA1_MuaMusp_vr1.mat"
)


def _sto2_from_mua(mua: np.ndarray) -> float:
    E = hemoglobin_extinctions(list(WL), "wray")
    c, *_ = np.linalg.lstsq(E * LN10, mua, rcond=None)
    if c[0] + c[1] <= 0:
        return np.nan
    return 100.0 * c[0] / (c[0] + c[1])


def _toi_from_optical(mua: np.ndarray, musp: np.ndarray, geom: SrsGeometry) -> float:
    I = np.array([10 ** (-attenuation(RHO, mua[i], musp[i], N_TISSUE))
                  for i in range(WL.size)]) * SCALE
    return float(compute_toi(I, geom).toi_raw)


def _sign_agreement(a: np.ndarray, b: np.ndarray) -> float:
    """相邻帧变化方向一致的比例。"""
    da = np.diff(a)
    db = np.diff(b)
    ok = (da != 0) & (db != 0)
    if ok.sum() == 0:
        return np.nan
    return float(np.mean(np.sign(da[ok]) == np.sign(db[ok])))


def run(mat_path: Path, step: int = 5) -> int:
    if not mat_path.exists():
        print(f"未找到 {mat_path}")
        print(f"请下载：{MAT_URL}")
        return 1

    raw = sio.loadmat(mat_path, squeeze_me=True, struct_as_record=False)
    deep = raw["Exp1_LMA1_MuaMusp"].Deep  # (T, 14, 2)  mua, musp
    mua_all = deep[:, :, 0]
    musp_all = deep[:, :, 1]
    valid = np.isfinite(mua_all).all(axis=1) & np.isfinite(musp_all).all(axis=1)
    mua_all = mua_all[valid]
    musp_all = musp_all[valid]
    n = mua_all.shape[0]
    idx = np.arange(0, n, step)

    geom = SrsGeometry(rho_cm=RHO, wavelengths_nm=WL)
    ref, read = [], []
    for t in idx:
        mua = np.interp(WL, WL_SRC, mua_all[t])
        musp = np.interp(WL, WL_SRC, musp_all[t])
        ref.append(_sto2_from_mua(mua))
        read.append(_toi_from_optical(mua, musp, geom))
    ref = np.array(ref)
    read = np.array(read)
    ok = np.isfinite(ref) & np.isfinite(read)
    idx_ok = idx[ok]
    ref, read = ref[ok], read[ok]

    r = float(np.corrcoef(ref, read)[0, 1])
    sign_ok = _sign_agreement(ref, read)
    mono_ref_down = float(np.mean(np.diff(ref) < 0))
    mono_read_down = float(np.mean(np.diff(read) < 0))

    print("=" * 72)
    print("SRS 趋势方向验证 — Sudakou 血液体模 Exp1（深层同质血液）")
    print("=" * 72)
    print(f"  数据：{mat_path.name}")
    print(f"  有效帧 {n}，抽样步长 {step} → {len(ref)} 点")
    print(f"  参考 StO₂ 跨度 {ref.min():.1f} ~ {ref.max():.1f} %  （Δ {ref.ptp():.1f}）")
    print(f"  SRS 读数跨度 {read.min():.1f} ~ {read.max():.1f} %  （Δ {read.ptp():.1f}）")
    print()
    print(f"  相关系数 r           = {r:.4f}")
    print(f"  相邻帧方向一致率     = {sign_ok * 100:.1f}%")
    print(f"  参考序列下降步占比   = {mono_ref_down * 100:.1f}%  （降氧为主）")
    print(f"  读数序列下降步占比   = {mono_read_down * 100:.1f}%")
    print()

    # 判据：同向变化 → 正相关；逐步方向多数一致
    pass_r = r > 0.9
    pass_sign = sign_ok > 0.55
    overall_down = ref[0] > ref[-1] and read[0] > read[-1]

    print(f"  [{'通过' if pass_r else '未通过'}] 整体正相关 r > 0.9（同向变化）")
    print(f"  [{'通过' if pass_sign else '未通过'}] 逐步方向一致 > 55%")
    print(f"  [{'通过' if overall_down else '未通过'}] 首末点：参考与读数均下降")
    print()
    print("  抽样轨迹（参考 → 读数）：")
    for i in (0, len(ref) // 4, len(ref) // 2, 3 * len(ref) // 4, -1):
        print(f"    t[{idx_ok[i]:4d}]  ref {ref[i]:6.2f}%   read {read[i]:6.2f}%")
    print()
    print("  ⚠️ 参考 StO₂ 由 μa 谱 + wray 表解出；多距离为正演。验证的是【方向】，不是绝对精度。")

    ok_all = pass_r and pass_sign and overall_down
    print("=" * 72)
    print(f"  结论：{'方向一致' if ok_all else '方向验证未通过'}")
    print("=" * 72)
    return 0 if ok_all else 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mat", type=Path, default=DEFAULT_MAT)
    ap.add_argument("--step", type=int, default=5, help="时间下采样步长（帧）")
    args = ap.parse_args()
    return run(args.mat, args.step)


if __name__ == "__main__":
    raise SystemExit(main())
