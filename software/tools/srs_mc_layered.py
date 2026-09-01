# -*- coding: utf-8 -*-
"""用 Monte Carlo 分层头模型验证 SRS —— 打破 inverse crime，量化分层的影响。

为什么要做
    srs.py 的验证状态表里「分层几何 ❌ 未验证」是最大的一块空白，而且此前
    所有 SRS 验证都是「扩散近似正演 → 扩散近似反演」，同一族模型。
    Monte Carlo 直接解辐射传输，不做扩散近似 —— 用 MC 正演、扩散反演，
    才是真正的跨模型检验。而且 MC 里每层的 StO2 是我们自己设的，
    真值精确已知，这是任何真实实验都做不到的。

方法
    ① 分层平板体素模型，1 mm 体素，五层：头皮/颅骨/CSF/灰质/白质。
    ② 每个波长跑一次 MC（μs′ 随波长变），MC 内把 μa 全设为 0，
       保存被探测光子在【各层】的路径长度 ppath。
    ③ 之后任意 StO2 组合只需重新加权 I = Σ exp(−Σ_j μa_j·L_j)，
       不必重跑 MC —— 扫描血氧几乎免费。
    ④ 把各距离的 I 喂进 fnirs_pipeline/srs.py，比对读数与真值。

光学参数出处
    μs′ 与基线 μa：Strangman, Li & Zhang, PLoS ONE 8(8):e66319, 2013, Table 1
        Gray 0.0195/1.10, White 0.0169/1.35, CSF 0.0025/0.01,
        Skull 0.011925/0.92, Scalp 0.017275/0.72   (μa/μs′, mm⁻¹)
        原表列名为 ms，但 0.72~1.35 mm⁻¹ = 7.2~13.5 cm⁻¹ 是约化散射系数的
        量级，故按 μs′ 处理；MC 用 g=0.89、μs=μs′/(1−g)，
        相似关系在 3~5 cm 的深扩散区成立。
    头皮厚度：TD-DOS 数据集 S01 前额超声实测 3.81 / 4.98 mm
        （Zenodo 10.5281/zenodo.15828014），默认取 4.5 mm。
    其余层厚为成人前额文献典型值，见 LAYERS，可用 --scalp 扫描。

⚠️ MC 仍然是模型，不是测量。它验的是「扩散近似 + 均匀假设」在分层几何下
   的偏差，验不了真实组织。体模与人体实验不能被它替代。

用法
    python tools/srs_mc_layered.py --validate    # 均匀介质对解析解，先验 MC 本身
    python tools/srs_mc_layered.py --run         # 分层实验（需 GPU，约 10 分钟）
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

LN10 = np.log(10.0)
WL = np.array([850.0, 810.0, 770.0, 730.0, 700.0])
RHO_MM = np.array([30.0, 40.0, 50.0])
G, N_TISSUE = 0.89, 1.4
DET_R = 3.0                      # 探测器半径 mm（实际光纤束 1~3 mm）
NPHOTON = 1e9
SRC_XY = (20.0, 45.0)
SHAPE = (150, 90, 70)

LAYER_NAMES = ["头皮", "颅骨", "CSF", "灰质", "白质"]
MUSP_800 = np.array([0.72, 0.92, 0.01, 1.10, 1.35])          # mm⁻¹
MUA_800 = np.array([0.017275, 0.011925, 0.002500, 0.019500, 0.016900])
CSF_IDX = 2                      # CSF 几乎无血红蛋白，μa 固定不随 StO2 变
MUSP_B = 1.0
LAYERS = dict(scalp=4.5, skull=7.0, csf=2.0, gray=4.0)


# ---------------------------------------------------------------------------
def musp_at(wl: float) -> np.ndarray:
    return MUSP_800 * (wl / 800.0) ** (-MUSP_B)


def build_volume(scalp, skull, csf, gray, shape=SHAPE) -> np.ndarray:
    """分层平板，z 向下为深度，1 mm 体素，白质填满剩余。"""
    vol = np.full(shape, 5, dtype=np.uint8)
    z = 0
    for lab, th in ((1, scalp), (2, skull), (3, csf), (4, gray)):
        n = int(round(th))
        vol[:, :, z:z + n] = lab
        z += n
    return vol


def run_mc(vol, musp, nphoton=NPHOTON, quiet=True):
    """跑一次 MC，返回 (探测器编号 1..3, 各层路径长度 mm)。μa 设为 0，靠 ppath 重加权。"""
    import contextlib
    import io as _io

    import pmcx

    n_media = int(vol.max())
    mus = np.asarray(musp)[:n_media] / (1.0 - G)
    prop = [[0.0, 0.0, 1.0, 1.0]] + [[0.0, float(m), G, N_TISSUE] for m in mus]
    cfg = dict(nphoton=int(nphoton), vol=vol, tstart=0, tend=5e-9, tstep=5e-9,
               srcpos=[SRC_XY[0], SRC_XY[1], 0.0], srcdir=[0, 0, 1], prop=prop,
               detpos=[[SRC_XY[0] + r, SRC_XY[1], 0.0, DET_R] for r in RHO_MM],
               unitinmm=1.0, issavedet=1, issrcfrom0=1,
               maxdetphoton=int(5e7), autopilot=1, gpuid=1)
    sink = _io.StringIO()
    with contextlib.redirect_stdout(sink if quiet else sys.stdout):
        res = pmcx.run(cfg)
    d = pmcx.detphoton(np.asarray(res["detp"]), n_media, "dp")
    det_id = np.asarray(d["detid"]).ravel().astype(int)
    ppath = np.asarray(d["ppath"], dtype=float)
    if ppath.shape[0] != det_id.size:
        ppath = ppath.T
    return det_id, ppath


def intensity(det_id, ppath, mua) -> np.ndarray:
    """由各层 μa (mm⁻¹) 重加权得各探测器相对光强。"""
    w = np.exp(-(ppath * np.asarray(mua)[None, :ppath.shape[1]]).sum(axis=1))
    return np.array([w[det_id == i + 1].sum() for i in range(RHO_MM.size)])


def calibrate_thb(sto2_ref=0.70) -> np.ndarray:
    """定出每层 tHb (M)，使 StO2=sto2_ref 时 μa(800nm) 复现 Strangman 表的值。"""
    from fnirs_pipeline.mbll_core import hemoglobin_extinctions
    E = hemoglobin_extinctions([800.0], "wray")[0]
    denom = (E[0] * sto2_ref + E[1] * (1 - sto2_ref)) * LN10      # 1/(M·cm)
    return (MUA_800 * 10.0) / denom                               # μa cm⁻¹ / 上式


def mua_of(sto2_layers, thb, wl) -> np.ndarray:
    """各层 μa (mm⁻¹)。CSF 固定不随 StO2 变。"""
    from fnirs_pipeline.mbll_core import hemoglobin_extinctions
    E = hemoglobin_extinctions([float(wl)], "wray")[0]
    s = np.asarray(sto2_layers, dtype=float)
    mua_cm = (E[0] * s + E[1] * (1 - s)) * np.asarray(thb) * LN10
    mua = mua_cm / 10.0
    mua[CSF_IDX] = MUA_800[CSF_IDX]
    return mua


def read_toi(det_id_all, ppath_all, sto2_layers, thb) -> float:
    """五个波长各自重加权 -> 光强矩阵 -> srs.py 反演 TOI。"""
    from fnirs_pipeline.srs import SrsGeometry, compute_toi
    I = np.empty((WL.size, RHO_MM.size))
    for i, wl in enumerate(WL):
        I[i] = intensity(det_id_all[i], ppath_all[i], mua_of(sto2_layers, thb, wl))
    I = I / I.max() * 1e12                       # 只影响截距，不影响斜率
    geom = SrsGeometry(rho_cm=RHO_MM / 10.0, wavelengths_nm=WL)
    return compute_toi(I, geom).toi_raw


# ---------------------------------------------------------------------------
def validate() -> int:
    """均匀介质：MC 正演的斜率 vs Farrell-Patterson 解析解。先验 MC 本身没搭错。"""
    from srs_reference import attenuation
    from fnirs_pipeline.srs import fit_slopes, slope_to_mueff

    print("=" * 76)
    print("MC 自检：均匀介质，MC 正演 vs Farrell-Patterson 解析解")
    print("=" * 76)
    print("  两者都是【正演】，反演方式相同，故差异只反映 MC 与扩散近似之别。\n")
    print("   μa      μs'    解析μeff   MC反推   扩散解反推    MC-解析   扩散-解析")
    vol = np.ones(SHAPE, dtype=np.uint8)
    ok = True
    for mua_cm, musp_cm in ((0.10, 10.0), (0.15, 12.0), (0.20, 9.0)):
        mua, musp = mua_cm / 10.0, musp_cm / 10.0
        det_id, ppath = run_mc(vol, np.array([musp]), nphoton=3e8)
        I = intensity(det_id, ppath, np.array([mua]))
        A_mc = -np.log10(I / I.max())
        A_an = attenuation(RHO_MM / 10.0, mua_cm, musp_cm, N_TISSUE)
        exact = np.sqrt(3.0 * mua_cm * (mua_cm + musp_cm))
        m_mc = float(slope_to_mueff(fit_slopes(A_mc[None, :], RHO_MM / 10.0)[0],
                                    RHO_MM / 10.0)[0])
        m_an = float(slope_to_mueff(fit_slopes(A_an[None, :], RHO_MM / 10.0)[0],
                                    RHO_MM / 10.0)[0])
        ok &= abs(m_mc - exact) / exact < 0.15
        print("  %5.2f  %5.1f    %7.4f  %7.4f    %7.4f    %+6.1f%%    %+6.1f%%"
              % (mua_cm, musp_cm, exact, m_mc, m_an,
                 100 * (m_mc - exact) / exact, 100 * (m_an - exact) / exact))
    print("\n  判据：MC 与解析解的偏差应同量级；差太多说明 MC 几何/参数搭错了。")
    print("  结论：%s" % ("通过" if ok else "未通过"))
    return 0 if ok else 1


def experiments(scalp_mm: float) -> None:
    thb = calibrate_thb()
    print("  每层 tHb（由 Strangman 的 μa(800) 反推，StO2=70%）：")
    for i, nm in enumerate(LAYER_NAMES):
        print("    %-4s tHb = %6.1f μM%s" % (nm, thb[i] * 1e6,
                                             "   (CSF 的 μa 固定，不参与)" if i == CSF_IDX else ""))
    print()

    vol = build_volume(scalp_mm, LAYERS["skull"], LAYERS["csf"], LAYERS["gray"])
    t0 = time.time()
    det_all, pp_all = [], []
    for wl in WL:
        d, p = run_mc(vol, musp_at(wl))
        det_all.append(d)
        pp_all.append(p)
        print("    %d nm  探测光子 30/40/50mm = %d / %d / %d"
              % (wl, (d == 1).sum(), (d == 2).sum(), (d == 3).sum()))
    print("  MC 用时 %.0f s\n" % (time.time() - t0))

    # 平均光程占比 —— 各层对信号的几何贡献
    print("  平均光程占比（50 mm 通道，810 nm）：")
    d, p = det_all[1], pp_all[1]
    sel = p[d == 3]
    frac = sel.mean(axis=0) / sel.mean(axis=0).sum()
    for i, nm in enumerate(LAYER_NAMES):
        print("    %-4s %5.1f %%" % (nm, 100 * frac[i]))
    brain_frac = frac[3] + frac[4]
    print("    -> 脑（灰+白）合计 %.1f %%，颅外（头皮+颅骨）%.1f %%\n"
          % (100 * brain_frac, 100 * (frac[0] + frac[1])))

    base = np.array([0.70, 0.70, 0.70, 0.70, 0.70])
    r0 = read_toi(det_all, pp_all, base, thb)
    print("  基线：各层 StO2 均为 70%%  ->  SRS 读数 %.2f %%O2\n" % r0)

    print("  实验一：只改【脑】(灰+白)，头皮/颅骨固定 70%")
    print("    脑真值    SRS读数    Δ读数    灵敏度 ∂读数/∂脑")
    br = []
    for s in (0.50, 0.60, 0.70, 0.80, 0.90):
        v = base.copy(); v[3] = v[4] = s
        r = read_toi(det_all, pp_all, v, thb)
        br.append((s * 100, r))
        print("    %5.0f%%   %8.2f  %+7.2f" % (s * 100, r, r - r0))
    br = np.array(br)
    k_brain = np.polyfit(br[:, 0], br[:, 1], 1)[0]
    print("    -> 斜率 %.4f（1 表示完全跟随脑，0 表示完全看不到脑）\n" % k_brain)

    print("  实验二：只改【头皮+颅骨】，脑固定 70%")
    print("    颅外真值  SRS读数    Δ读数")
    ex = []
    for s in (0.50, 0.60, 0.70, 0.80, 0.90):
        v = base.copy(); v[0] = v[1] = s
        r = read_toi(det_all, pp_all, v, thb)
        ex.append((s * 100, r))
        print("    %5.0f%%   %8.2f  %+7.2f" % (s * 100, r, r - r0))
    ex = np.array(ex)
    k_extra = np.polyfit(ex[:, 0], ex[:, 1], 1)[0]
    print("    -> 斜率 %.4f\n" % k_extra)

    print("  " + "-" * 68)
    print("  脑灵敏度 %.4f   颅外灵敏度 %.4f   比值 脑/颅外 = %.2f"
          % (k_brain, k_extra, k_brain / k_extra if k_extra else np.inf))
    print("  脑占总灵敏度 %.1f %%" % (100 * k_brain / (k_brain + k_extra)))
    print("  头皮 StO2 变 1 点 = 读数变 %.3f 点；要抵消脑变 1 点，"
          "头皮需反向变 %.2f 点" % (k_extra, k_brain / k_extra if k_extra else np.inf))
    print("  " + "-" * 68)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--validate", action="store_true")
    ap.add_argument("--run", action="store_true")
    ap.add_argument("--scalp", type=float, default=LAYERS["scalp"],
                    help="头皮厚度 mm（默认 4.5，取自超声实测 3.81~4.98）")
    a = ap.parse_args()
    if a.validate:
        return validate()
    if a.run:
        print("=" * 76)
        print("MC 分层头模型 —— SRS 读数对脑 / 颅外的灵敏度")
        print("=" * 76)
        print("  层厚 mm：头皮 %.1f  颅骨 %.1f  CSF %.1f  灰质 %.1f  白质 填满"
              % (a.scalp, LAYERS["skull"], LAYERS["csf"], LAYERS["gray"]))
        print("  源探距离 %s mm   探测器半径 %.1f mm   每波长 %.0e 光子\n"
              % (list(RHO_MM.astype(int)), DET_R, NPHOTON))
        experiments(a.scalp)
        return 0
    ap.print_help()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
