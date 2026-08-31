"""
空间分辨光谱（SRS）参考实现 —— 绝对组织氧饱和度反演。

原理为公开的稳态扩散理论（Farrell & Patterson 半无限解 + 大 ρ 渐近斜率法），
NIRO-200NX / EGOS-600 等商用设备均基于此，但各厂商的标定常数、
μs' 模型与各类修正为私有内容，本文件不包含也无法复现那些部分。

⚠️ 重要性质：μs' 的【绝对值】不影响 TOI（比值中约掉），
   但 μs' 的【光谱形状】直接决定 TOI。见 selftest 第 3 项。

⚠️ 本实现仅供算法验证与体模标定实验，未经临床验证，不可用于诊断。
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from fnirs_pipeline.mbll_core import hemoglobin_extinctions  # noqa: E402

LN10 = np.log(10.0)


# ---------------------------------------------------------------------------
# 正向模型
# ---------------------------------------------------------------------------
def r_eff(n: float) -> float:
    """内反射系数（Groenhuis / Egan-Hilgeman 经验式）。"""
    return -1.440 / n ** 2 + 0.710 / n + 0.668 + 0.0636 * n


def reflectance(rho, mua, musp, n=1.4):
    """半无限介质稳态漫反射 R(ρ)，Farrell-Patterson 解。

    rho: (n_rho,) cm   mua/musp: 标量 cm^-1   返回 (n_rho,)
    """
    rho = np.asarray(rho, float)
    mueff = np.sqrt(3.0 * mua * (mua + musp))
    z0 = 1.0 / (mua + musp)
    D = 1.0 / (3.0 * (mua + musp))
    zb = 2.0 * D * (1.0 + r_eff(n)) / (1.0 - r_eff(n))
    r1 = np.sqrt(rho ** 2 + z0 ** 2)
    r2 = np.sqrt(rho ** 2 + (z0 + 2 * zb) ** 2)
    return (1.0 / (4 * np.pi)) * (
        z0 * (mueff + 1.0 / r1) * np.exp(-mueff * r1) / r1 ** 2
        + (z0 + 2 * zb) * (mueff + 1.0 / r2) * np.exp(-mueff * r2) / r2 ** 2
    )


def attenuation(rho, mua, musp, n=1.4, gain_db=None):
    """A(ρ) = -log10(R)。gain_db: 各探测器的增益偏差(dB)，模拟标定残差。"""
    A = -np.log10(reflectance(rho, mua, musp, n))
    if gain_db is not None:
        A = A - np.asarray(gain_db, float) / 10.0
    return A


# ---------------------------------------------------------------------------
# 反演
# ---------------------------------------------------------------------------
def slope_to_mua(rho, A, musp):
    """SRS 工作方程：dA/dρ -> μeff -> μa。

    ∂A/∂ρ = (μeff + 2/ρ̄)/ln10   =>   μeff = ln10·(∂A/∂ρ) − 2/ρ̄
    μa = μeff² / (3·μs′)
    """
    rho = np.asarray(rho, float)
    slope = np.polyfit(rho, np.asarray(A, float), 1)[0]
    mueff = LN10 * slope - 2.0 / rho.mean()
    if mueff <= 0:
        return np.nan, slope, mueff
    return mueff ** 2 / (3.0 * musp), slope, mueff


def toi(rho, A_by_wl, wavelengths, musp_by_wl, table="wray"):
    """多波长 SRS -> 组织氧饱和度 TOI (%)。

    A_by_wl:    (n_wl, n_rho) 各波长的 A(ρ)
    musp_by_wl: (n_wl,) 假设的约化散射系数谱
    """
    A_by_wl = np.atleast_2d(A_by_wl)
    mua = np.array([slope_to_mua(rho, A_by_wl[i], musp_by_wl[i])[0]
                    for i in range(A_by_wl.shape[0])])
    if np.any(~np.isfinite(mua)):
        return np.nan, mua
    E = hemoglobin_extinctions(list(map(float, wavelengths)), table)   # 1/(M·cm)
    c, *_ = np.linalg.lstsq(E * LN10, mua, rcond=None)                # [c_HbO, c_HbR]
    s = c[0] + c[1]
    return (100.0 * c[0] / s if s != 0 else np.nan), mua


def toi_fit_scattering(rho, A_by_wl, wavelengths, a_init=10.0, b_prior=1.0,
                       b_bounds=(0.5, 1.6), b_weight=0.0, table="wray"):
    """四参数 SRS：同时反演 [c_HbO, c_HbR, a, b]，不预先假设 μs' 谱指数。

    约束 μs'(λ) = a·(λ/800)^(-b)，于是五波长给出 5 个 μeff 而未知数只有 4 个，
    超定，可把 b 一并解出。相对 toi() 的差别：

      toi()                 假设 musp_by_wl 已知 -> b 偏 ±0.4 时 TOI 偏 ±3.7 %O2
      toi_fit_scattering()  自解 b               -> b 在 0.6~1.4 内 TOI 稳定在 +0.7

    代价是多一个自由参数也会吸收模型误差：μs' 谱偏离幂律时本函数比 toi() 更敏感
    （见 selftest 第 5 项）。b_weight > 0 可加正则项把 b 往 b_prior 拉以缓解。

    ⚠️ 上述结论均在均匀半无限介质下得到。真实头部为分层结构，本函数在分层几何
       下的表现未经验证。

    返回 (TOI%, b, μa_by_wl)。拟合失败返回 (nan, nan, None)。
    """
    from scipy.optimize import least_squares

    rho = np.asarray(rho, float)
    A_by_wl = np.atleast_2d(A_by_wl)
    wl = np.asarray(list(map(float, wavelengths)), float)
    obs = np.array([np.polyfit(rho, A_by_wl[i], 1)[0] for i in range(A_by_wl.shape[0])])
    E = hemoglobin_extinctions(list(map(float, wl)), table)      # 1/(M·cm)
    inv_rho = 2.0 / rho.mean()

    # 参数归一化到 O(1)，否则浓度(1e-4)与散射(1e1)尺度相差 1e5，
    # least_squares 会在初值处直接判定收敛（实测会静默停在起点）。
    def _mua_musp(p):
        mua = (E[:, 0] * p[0] * 1e-4 + E[:, 1] * p[1] * 1e-4) * LN10
        musp = p[2] * 10.0 * (wl / 800.0) ** (-p[3])
        return mua, musp

    def _resid(p):
        mua, musp = _mua_musp(p)
        pred = (np.sqrt(3.0 * mua * (mua + musp)) + inv_rho) / LN10
        r = (pred - obs) * 1e3
        if b_weight > 0:
            r = np.append(r, np.sqrt(b_weight) * (p[3] - b_prior))
        return r

    p0 = [0.42, 0.18, a_init / 10.0, b_prior]
    lo = [0.0, 0.0, 0.1, b_bounds[0]]
    hi = [3.0, 3.0, 4.0, b_bounds[1]]
    try:
        res = least_squares(_resid, p0, bounds=(lo, hi),
                            xtol=1e-14, ftol=1e-14, gtol=1e-14)
    except Exception:
        return np.nan, np.nan, None
    c_hbo, c_hbr = res.x[0] * 1e-4, res.x[1] * 1e-4
    s = c_hbo + c_hbr
    mua, _ = _mua_musp(res.x)
    return (100.0 * c_hbo / s if s != 0 else np.nan), res.x[3], mua


def mua_of(so2, thb_M, wavelengths, table="wray"):
    """由 (SO2, 总血红蛋白) 生成各波长 μa，用于正向仿真。"""
    E = hemoglobin_extinctions(list(map(float, wavelengths)), table)
    return (E[:, 0] * so2 + E[:, 1] * (1 - so2)) * thb_M * LN10


# ---------------------------------------------------------------------------
def selftest():
    from config import mbll_wavelengths_nm
    WL = np.asarray(mbll_wavelengths_nm(), float)
    rho = np.array([3.0, 4.0, 5.0])
    THB, SO2_T, N = 60e-6, 0.70, 1.4
    musp_true = 10.0 * (WL / 800.0) ** (-1.0)

    print("=" * 84)
    print("SRS 参考实现自检")
    print("=" * 84)
    print(f"  波长 {[int(w) for w in WL]} nm   距离 {list(rho)} cm   n={N}")
    print(f"  真值 SO2={SO2_T*100:.0f}%  tHb={THB*1e6:.0f}μM  μs'(800nm)=10.0 cm⁻¹")
    print()

    mua_t = mua_of(SO2_T, THB, WL)
    A = np.array([attenuation(rho, mua_t[i], musp_true[i], N) for i in range(len(WL))])

    ok = True
    v, mua_r = toi(rho, A, WL, musp_true)
    err = abs(v - SO2_T * 100)
    ok &= err < 6.0
    print(f"  [1] 往返：TOI = {v:.2f}%  误差 {v - SO2_T*100:+.2f} %O2")
    print(f"      μa 反演 {np.array2string(mua_r, precision=4)}")
    print(f"      μa 真值 {np.array2string(mua_t, precision=4)}")
    print(f"      -> 渐近斜率法相对精确解有系统偏差，需由体模标定吸收 {'OK' if err < 6 else 'FAIL'}")
    print()

    print("  [2] 不同真实饱和度下的单调性与偏差：")
    for s in (0.30, 0.50, 0.70, 0.90):
        m = mua_of(s, THB, WL)
        AA = np.array([attenuation(rho, m[i], musp_true[i], N) for i in range(len(WL))])
        vv, _ = toi(rho, AA, WL, musp_true)
        print(f"      真值 {s*100:3.0f}%  ->  {vv:6.2f}%   偏差 {vv - s*100:+6.2f}")
    print()

    print("  [3] μs' 绝对值 vs 光谱形状（关键性质）：")
    for scale in (0.7, 1.0, 1.5):
        vv, _ = toi(rho, A, WL, musp_true * scale)
        print(f"      μs' 整体 ×{scale:.1f}      -> TOI {vv:6.2f}%   偏差 {vv - v:+6.2f}  (应≈0)")
    for b in (0.6, 1.0, 1.4):
        vv, _ = toi(rho, A, WL, 10.0 * (WL / 800.0) ** (-b))
        print(f"      μs'∝λ^-{b:.1f} (真 1.0) -> TOI {vv:6.2f}%   偏差 {vv - v:+6.2f}")
    print()

    print("  [4] 探测器增益偏差的影响：")
    for db in (0.05, 0.1, 0.2):
        g = np.array([0.0, 0.0, db])
        AA = np.array([attenuation(rho, mua_t[i], musp_true[i], N, g) for i in range(len(WL))])
        vv, _ = toi(rho, AA, WL, musp_true)
        print(f"      最远探测器偏 {db:.2f} dB ({100*(10**(db/10)-1):.1f}%) -> TOI {vv:6.2f}%  偏差 {vv - v:+6.2f}")
    print()

    print("  [5] 自解 μs' 谱指数 b（toi_fit_scattering）vs 固定假设（toi）：")
    print("      真实 μs' 谱形                     自解b   自解SO2  固定b=1.0")
    cases = [("纯幂律 b=0.6", 10.0 * (WL / 800.0) ** (-0.6)),
             ("纯幂律 b=1.0", 10.0 * (WL / 800.0) ** (-1.0)),
             ("纯幂律 b=1.4", 10.0 * (WL / 800.0) ** (-1.4)),
             ("两层混合 b=0.6 与 1.3 各半",
              5 * (WL / 800.0) ** (-0.6) + 5 * (WL / 800.0) ** (-1.3)),
             ("幂律叠加 4% 正弦畸变",
              10.0 * (WL / 800.0) ** (-1.0) * (1 + 0.04 * np.sin((WL - 700) / 150 * np.pi)))]
    fixed_ref = 10.0 * (WL / 800.0) ** (-1.0)
    spread = []
    for nm, ms in cases:
        AA = np.array([attenuation(rho, mua_t[i], ms[i], N) for i in range(len(WL))])
        sf, bf, _ = toi_fit_scattering(rho, AA, WL)
        sx, _ = toi(rho, AA, WL, fixed_ref)
        if nm.startswith("纯幂律"):
            spread.append(sf)
        print(f"      {nm:<30s} {bf:5.2f}  {sf:6.2f}   {sx:6.2f}")
    rng_b = max(spread) - min(spread)
    ok &= rng_b < 1.0
    print(f"      -> 真实 b 在 0.6~1.4 变化时，自解 SO2 波动 {rng_b:.2f} %O2 "
          f"（固定假设为 7.40）{'OK' if rng_b < 1.0 else 'FAIL'}")
    print("      -> 但 μs' 偏离幂律时，自解反而更敏感（见最后一行）")
    print()
    print(f"自检结论：{'通过' if ok else '未通过'}")
    return ok


if __name__ == "__main__":
    raise SystemExit(0 if selftest() else 1)
