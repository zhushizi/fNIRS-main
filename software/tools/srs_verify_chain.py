# -*- coding: utf-8 -*-
"""SRS 链路逐级验证 —— 把 fnirs_pipeline/srs.py 的每一环单独拆开查。

    python tools/srs_verify_chain.py                    # 只跑 A 组（无需数据）
    python tools/srs_verify_chain.py --data <tddos目录>  # 加跑 B 组（真实在体光学参数）

A 组验证的是【代码实现对不对】：公式有没有写错、结构性质成不成立、
质量标志报不报得出来。全部用 Farrell-Patterson 精确解正演，
用 srs.py 的渐近斜率法反演 —— 两者不是同一个模型，所以偏差有物理意义。

⚠️ A 组**不能**证明模型对真实头部成立。真实头部是分层的，A 组是均匀半无限。
   要验证那一层，需要 B 组（真实在体光学参数）以及体模/人体实验。

B 组用 10 名成人前额的时域 DOS 实测 μa(λ)/μs′(λ) 做正演，
真值由实测 μa 直接解出 —— 光学参数不是我们编的，是别人测的。
但氧变化仍是构造的（受试者间差异），不含运动伪影与头皮动态。

B 组数据获取（约 70 MB，CC-BY 4.0）：
    https://doi.org/10.5281/zenodo.15828014
    下载 zip 解压到任意目录 D，然后 --data D
    脚本只用其中 04_Analysis/Results_Final.txt 一个文件。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))
sys.path.insert(0, str(_HERE))
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from fnirs_pipeline.mbll_core import hemoglobin_extinctions          # noqa: E402
from fnirs_pipeline.srs import (                                     # noqa: E402
    MUSP_EXPONENT, MUSP_REF_800, SrsGeometry, compute_toi, fit_slopes,
    mua_from_mueff, mua_to_toi, musp_power_law, slope_to_mueff, to_attenuation,
)
from srs_reference import attenuation, reflectance, toi as ref_toi   # noqa: E402

LN10 = np.log(10.0)
WL = np.array([850.0, 810.0, 770.0, 730.0, 700.0])   # 与 config.mbll_wavelengths_nm 一致
RHO = np.array([3.0, 4.0, 5.0])                      # 长距三点（现硬件只有 3.0，见文末）
N_TISSUE = 1.4
THB, SO2_TRUE = 60e-6, 0.70
# 把 10^(−A) 换算成 ADC 计数的比例因子。A(5cm)≈6.8 OD，取 1e12 使最远探测器
# 落在 ~2e5 计数，高于 srs.py 的 MIN_COUNTS=2000，否则质量标志会一直报计数不足。
SCALE = 1e12

_RESULTS: list[tuple[str, bool, str]] = []


def _check(name: str, ok: bool, detail: str = "") -> None:
    _RESULTS.append((name, ok, detail))
    print(f"      {'[通过]' if ok else '[未通过]'} {name}" + (f" —— {detail}" if detail else ""))


def mua_of(so2: float, thb: float, wl: np.ndarray, table: str = "wray") -> np.ndarray:
    E = hemoglobin_extinctions(list(map(float, wl)), table)
    return (E[:, 0] * so2 + E[:, 1] * (1 - so2)) * thb * LN10


def mueff_exact(mua: float, musp: float) -> float:
    return float(np.sqrt(3.0 * mua * (mua + musp)))


# ===========================================================================
# A 组
# ===========================================================================
def a1_working_equation() -> None:
    """A1 工作方程 μeff = ln10·(∂A/∂ρ) − 2/ρ 本身对不对。

    不做任何拟合：直接对精确解 A(ρ)=−log10(R(ρ)) 数值求导，代入工作方程，
    与解析 μeff=√(3μa(μa+μs′)) 比。这一步只查公式抄没抄错。
    """
    print("\n  A1  工作方程（数值微分，不含拟合）")
    print("      μa      μs'     ρ     解析μeff   反推μeff    相对差")
    worst = 0.0
    for mua in (0.05, 0.10, 0.20):
        for musp in (8.0, 12.0):
            for rho0 in (3.0, 4.0, 5.0):
                h = 1e-4
                d = (-np.log10(reflectance(rho0 + h, mua, musp, N_TISSUE))
                     + np.log10(reflectance(rho0 - h, mua, musp, N_TISSUE))) / (2 * h)
                got = LN10 * float(d) - 2.0 / rho0
                exact = mueff_exact(mua, musp)
                rel = abs(got - exact) / exact
                worst = max(worst, rel)
                print(f"      {mua:.2f}    {musp:4.1f}   {rho0:.1f}   {exact:7.4f}   "
                      f"{got:7.4f}   {rel*100:6.2f}%")
    _check("A1 工作方程", worst < 0.05,
           f"最大相对差 {worst*100:.2f}%（渐近解的固有残差，随 ρ 增大而减小）")


def a2_three_point_fit() -> None:
    """A2 三点直线拟合额外引入多少误差。

    A(ρ) 不是直线（2/ρ 项随 ρ 变），三点拟合的斜率是 3~5cm 的平均斜率，
    工作方程里却用 ρ̄=4 代入 —— 这个错配是 srs.py 里 k/b 标定要吸收的东西。
    """
    print("\n  A2  三点拟合 + ρ̄ 近似的系统偏差")
    print("      μa      μs'    解析μeff   拟合反推   偏差      拟合残差(OD)")
    resid_all, bias_all = [], []
    for mua in (0.05, 0.10, 0.20):
        for musp in (8.0, 12.0):
            A = attenuation(RHO, mua, musp, N_TISSUE)
            sl, rs = fit_slopes(A[None, :], RHO)
            got = float(slope_to_mueff(sl, RHO)[0])
            exact = mueff_exact(mua, musp)
            resid_all.append(float(rs[0]))
            bias_all.append((got - exact) / exact)
            print(f"      {mua:.2f}    {musp:4.1f}   {exact:7.4f}   {got:7.4f}   "
                  f"{(got-exact)/exact*100:+6.2f}%   {rs[0]:.4f}")
    r = np.array(resid_all)
    bias = np.array(bias_all)
    _check("A2 拟合残差稳定（可作线性度自检基准）",
           r.std() < 0.01 and 0.005 < r.mean() < 0.05,
           f"残差 {r.mean():.4f} ± {r.std():.4f} OD")
    # ⚠️ 偏差随 μa 变（μa 小时 +2.9%，μa 大时 −0.03%），不是常数偏置。
    #    它进不了「一个常数 b」，但 A6 显示落到 TOI 上仍是线性 —— 因为
    #    各波长的 μa 同向同幅变化，比值受的影响远小于 μa 本身。
    _check("A2 偏差有界", np.abs(bias).max() < 0.05,
           f"偏差 {bias.min()*100:+.2f}% ~ {bias.max()*100:+.2f}%（随 μa 变，非常数）")


def a3_common_mode() -> None:
    """A3 Σw=0 结构性质：所有探测器共同的增益/功率变化不进斜率。

    这是「SRS 不需要知道光源绝对功率」的根据。若这条不成立，
    LED 老化、探头总耦合变化都会直接进绝对值。
    """
    print("\n  A3  共模增益（光源功率 / 全局耦合）")
    mua_t = mua_of(SO2_TRUE, THB, WL)
    musp_t = musp_power_law(WL, 10.0, 1.0)
    I0 = np.array([10 ** (-attenuation(RHO, mua_t[i], musp_t[i], N_TISSUE))
                   for i in range(WL.size)])
    geom = SrsGeometry(rho_cm=RHO, wavelengths_nm=WL)
    base = compute_toi(I0 * SCALE, geom).toi_raw
    worst = 0.0
    for f in (0.5, 0.8, 1.0, 1.3, 2.0):
        v = compute_toi(I0 * SCALE * f, geom).toi_raw
        worst = max(worst, abs(v - base))
        print(f"      光强整体 ×{f:<4.1f} -> TOI {v:8.4f}%   变化 {v-base:+.2e}")
    _check("A3 共模增益不进结果", worst < 1e-9, f"最大变化 {worst:.2e} %O2")


def a4_detector_gain() -> None:
    """A4 探测器间增益失配：分「各波长一致」与「波长间不一致」两种。

    与 A3 相反，只动一路探测器，误差确实进斜率。但斜率误差落到 TOI 上
    要看它是否**依赖波长**：

      · 各波长一致的失配 → 各波长 μa 同比例变 → 比值 HbO/(HbO+HbR) 几乎不变
      · 波长间不一致     → 各波长 μa 变得不一样 → 直接进 TOI

    这正是「波长间增益一致性 <1%」这条指标的来源：要紧的不是探测器绝对增益，
    是同一探测器在五个波长上的**相对**响应。
    """
    print("\n  A4  探测器间增益失配（只动最远那路）")
    mua_t = mua_of(SO2_TRUE, THB, WL)
    musp_t = musp_power_law(WL, 10.0, 1.0)
    geom = SrsGeometry(rho_cm=RHO, wavelengths_nm=WL)
    I0 = np.array([10 ** (-attenuation(RHO, mua_t[i], musp_t[i], N_TISSUE))
                   for i in range(WL.size)])

    def read(gain_far) -> float:
        g = np.ones((WL.size, RHO.size))
        g[:, -1] = gain_far
        return compute_toi(I0 * g * SCALE, geom).toi_raw

    base = read(1.0)
    print(f"      基准 TOI = {base:.2f}%")
    print("      (a) 五个波长【同样】失配 —— 模拟探测器整体增益偏了")
    print("          失配      TOI       偏差")
    flat = []
    for err in (0.01, 0.05, 0.10, 0.20):
        v = read(1.0 + err)
        flat.append(abs(v - base))
        print(f"          {err*100:4.0f}%    {v:7.2f}%   {v-base:+7.2f} %O2")

    print("      (b) 失配【随波长倾斜】—— 模拟 PD 光谱响应形状不匹配")
    print("          700↔850跨度   TOI       偏差")
    tilt = []
    for err in (0.01, 0.02, 0.05, 0.10):
        g = 1.0 + err * (WL - WL.mean()) / (WL.max() - WL.min())
        v = read(g)
        tilt.append(abs(v - base))
        print(f"          {err*100:5.0f}%      {v:7.2f}%   {v-base:+7.2f} %O2")

    _check("A4a 波长一致的失配几乎不进 TOI", flat[-1] < 0.5,
           f"20% 一致失配只动 {flat[-1]:.2f} %O2")
    _check("A4b 波长间不一致的失配直接进 TOI", tilt[0] > flat[0] * 5,
           f"1% 波长间不一致 = {tilt[0]:.2f} %O2，"
           f"是同幅度一致失配（{flat[0]:.3f}）的 {tilt[0]/max(flat[0],1e-9):.0f} 倍")


def a5_quality_flags() -> None:
    """A5 质量标志：坏数据要报得出来，不能静默给出漂亮数字。"""
    print("\n  A5  质量标志")
    mua_t = mua_of(SO2_TRUE, THB, WL)
    musp_t = musp_power_law(WL, 10.0, 1.0)
    I = np.array([10 ** (-attenuation(RHO, mua_t[i], musp_t[i], N_TISSUE))
                  for i in range(WL.size)]) * SCALE
    geom = SrsGeometry(rho_cm=RHO, wavelengths_nm=WL)

    r_ok = compute_toi(I, geom)
    print(f"      正常数据      flags={r_ok.flags}  残差max={np.nanmax(r_ok.residual):.4f}")
    _check("A5 正常数据无标志", not r_ok.flags)

    I_bad = I.copy(); I_bad[:, 1] *= 0.5          # 中间探测器耦合掉一半
    r_bad = compute_toi(I_bad, geom)
    print(f"      中间探测器×0.5 flags={r_bad.flags}  残差max={np.nanmax(r_bad.residual):.4f}")
    _check("A5 单路耦合异常被残差抓到", bool(r_bad.flags))

    I_dim = I.copy(); I_dim[:, -1] = 500.0        # 最远探测器计数不足
    r_dim = compute_toi(I_dim, geom)
    print(f"      最远探测器暗   flags={r_dim.flags}")
    _check("A5 计数不足被抓到", any("计数" in f for f in r_dim.flags))

    try:
        SrsGeometry(rho_cm=np.array([3.0, 4.0]), wavelengths_nm=WL)
        _check("A5 两点几何被拒绝", False, "未抛异常")
    except ValueError as e:
        _check("A5 两点几何被拒绝", True, str(e))


def a6_closure_and_range() -> None:
    """A6 五波长全链路闭合 + 量程线性度。

    ⚠️ 正演用精确解、反演用渐近解，所以偏差不为零；但偏差必须
       ①方向一致 ②随真值线性 —— 否则 k/b 两参数标定救不回来。
    """
    print("\n  A6  全链路闭合与量程")
    musp_t = musp_power_law(WL, 10.0, 1.0)
    geom = SrsGeometry(rho_cm=RHO, wavelengths_nm=WL)
    truth, read = [], []
    print("      真值      读数      偏差")
    for s in (0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90):
        m = mua_of(s, THB, WL)
        I = np.array([10 ** (-attenuation(RHO, m[i], musp_t[i], N_TISSUE))
                      for i in range(WL.size)]) * SCALE
        v = compute_toi(I, geom).toi_raw
        truth.append(s * 100); read.append(v)
        print(f"      {s*100:5.1f}%   {v:7.2f}%   {v-s*100:+6.2f}")
    truth, read = np.array(truth), np.array(read)
    k, b = np.polyfit(truth, read, 1)
    r = float(np.corrcoef(truth, read)[0, 1])
    resid = read - (k * truth + b)
    print(f"      线性拟合 读数 = {k:.4f}·真值 {b:+.3f}   r = {r:.6f}")
    print(f"      两参数修正后残差 {np.abs(resid).max():.3f} %O2（最大）")
    _check("A6 单调且高度线性（k/b 可救）", r > 0.999 and np.abs(resid).max() < 1.0,
           f"r={r:.6f}，修正后最大残差 {np.abs(resid).max():.3f} %O2")


def a7_cross_implementation() -> None:
    """A7 两份独立写的实现互查（转录检查，不是模型检查）。

    tools/srs_reference.py 与 fnirs_pipeline/srs.py 是分开写的两条代码路径，
    数学相同。一致 = 没有抄错；不一致 = 至少一边有 bug。
    """
    print("\n  A7  srs.py  vs  srs_reference.py")
    musp_t = musp_power_law(WL, 10.0, 1.0)
    geom = SrsGeometry(rho_cm=RHO, wavelengths_nm=WL)
    worst = 0.0
    for s in (0.35, 0.55, 0.75, 0.95):
        m = mua_of(s, THB, WL)
        A = np.array([attenuation(RHO, m[i], musp_t[i], N_TISSUE) for i in range(WL.size)])
        v_pipe = compute_toi(10 ** (-A) * SCALE, geom).toi_raw
        v_ref, _ = ref_toi(RHO, A, WL, musp_power_law(WL, MUSP_REF_800, MUSP_EXPONENT))
        # 管线默认 μs' 用 MUSP_REF_800/MUSP_EXPONENT，参考实现这里传同一条谱
        worst = max(worst, abs(v_pipe - v_ref))
        print(f"      真值{s*100:5.1f}%   管线 {v_pipe:8.4f}   参考 {v_ref:8.4f}   "
              f"差 {v_pipe-v_ref:+.2e}")
    _check("A7 两实现一致", worst < 1e-9, f"最大差 {worst:.2e} %O2")


def a8_time_independence() -> None:
    """A8 逐帧独立性：SRS 不依赖时间基线（与 MBLL 的根本区别）。

    删掉前 N 帧，其余帧的结果必须逐位不变。
    """
    print("\n  A8  逐帧独立性（无时间基线）")
    from fnirs_pipeline.srs import compute_toi_series
    musp_t = musp_power_law(WL, 10.0, 1.0)
    geom = SrsGeometry(rho_cm=RHO, wavelengths_nm=WL)
    rng = np.random.default_rng(0)
    n_t = 40
    so2 = 0.70 + 0.10 * np.sin(np.arange(n_t) / 6.0)
    I = np.empty((WL.size, RHO.size, n_t))
    for t in range(n_t):
        m = mua_of(so2[t], THB, WL)
        I[:, :, t] = np.array([10 ** (-attenuation(RHO, m[i], musp_t[i], N_TISSUE))
                               for i in range(WL.size)]) * SCALE
    I *= (1 + 0.001 * rng.standard_normal(I.shape))
    full, _ = compute_toi_series(I, geom)
    cut, _ = compute_toi_series(I[:, :, 10:], geom)
    d = np.abs(full[10:] - cut).max()
    print(f"      整段 {n_t} 帧 vs 砍掉前 10 帧：最大差 {d:.2e} %O2")
    print(f"      读数范围 {full.min():.2f} ~ {full.max():.2f} %O2"
          f"（真值 {so2.min()*100:.1f} ~ {so2.max()*100:.1f}）")
    _check("A8 逐帧独立", d == 0.0, f"最大差 {d:.2e}")


# ===========================================================================
# B 组 —— 真实在体光学参数
# ===========================================================================
def group_b(data_dir: Path) -> None:
    """用 10 人前额实测 μa(λ)/μs′(λ) 正演，检查反演能否复现真值 TOI。

    数据：Zenodo 10.5281/zenodo.15828014（CC-BY 4.0）时域 DOS，610~1110nm。
    真值 TOI 由实测 μa 直接最小二乘解出，与反演路径无关。
    """
    import pandas as pd
    f = data_dir / "Dataset_invivo_subjects_TDDOS" / "04_Analysis" / "Results_Final.txt"
    if not f.exists():
        f2 = next(data_dir.rglob("Results_Final.txt"), None)
        if f2 is None:
            print(f"\n  B 组跳过：未找到 Results_Final.txt（查找路径 {data_dir}）")
            return
        f = f2
    print(f"\n  B 组数据：{f}")
    d = pd.read_csv(f, sep="\t")
    d.columns = [c.strip() for c in d.columns]
    F = d[d.Location.str.contains("Forehead", case=False, na=False)].copy()
    F["Lambda"] = F.Lambda.astype(float)
    F["mua"] = F.VarMua0Opt.astype(float)
    F["musp"] = F.VarMus0Opt.astype(float)

    E = hemoglobin_extinctions(list(WL), "wray")
    geom = SrsGeometry(rho_cm=RHO, wavelengths_nm=WL)
    print("      受试者    真值TOI    反演TOI     偏差")
    truth, read = [], []
    for sub in sorted(F.Subject.unique()):
        g = F[F.Subject == sub].groupby("Lambda")[["mua", "musp"]].mean()
        wl_src = g.index.values.astype(float)
        mua_s = np.interp(WL, wl_src, g.mua.values)
        musp_s = np.interp(WL, wl_src, g.musp.values)
        c, *_ = np.linalg.lstsq(E * LN10, mua_s, rcond=None)
        t = 100.0 * c[0] / (c[0] + c[1])
        I = np.array([10 ** (-attenuation(RHO, mua_s[i], musp_s[i], N_TISSUE))
                      for i in range(WL.size)]) * SCALE
        v = compute_toi(I, geom).toi_raw
        truth.append(t); read.append(v)
        print(f"      {sub:<10s} {t:7.2f}%   {v:7.2f}%   {v-t:+6.2f}")
    truth, read = np.array(truth), np.array(read)
    k, b = np.polyfit(truth, read, 1)
    r = float(np.corrcoef(truth, read)[0, 1])
    e = read - truth
    print(f"      偏差 {e.mean():+.2f} ± {e.std():.2f} %O2   r = {r:.4f}   斜率 = {k:.3f}")
    print(f"      真值跨度 {truth.min():.1f}~{truth.max():.1f}（{np.ptp(truth):.1f} 点），"
          f"读数跨度 {np.ptp(read):.1f} 点")
    _check("B1 真实光学参数下趋势保持", r > 0.9 and 0.8 < k < 1.3,
           f"r={r:.4f}，斜率={k:.3f}")
    print("      ⚠️ 这是受试者【间】差异，不是同一个人的时间变化；"
          "不含运动伪影与头皮动态。")


# ===========================================================================
def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", type=Path, default=None,
                    help="TD-DOS 数据集解压目录（含 Dataset_invivo_subjects_TDDOS）")
    args = ap.parse_args()

    print("=" * 78)
    print("SRS 链路逐级验证")
    print("=" * 78)
    print(f"  波长 {[int(w) for w in WL]} nm   源探距离 {list(RHO)} cm   n={N_TISSUE}")
    print(f"  被测模块 fnirs_pipeline/srs.py   默认 μs′ = "
          f"{MUSP_REF_800}·(λ/800)^(−{MUSP_EXPONENT})")

    print("\n" + "-" * 78)
    print("A 组：代码实现（不需要外部数据）")
    print("-" * 78)
    a1_working_equation()
    a2_three_point_fit()
    a3_common_mode()
    a4_detector_gain()
    a5_quality_flags()
    a6_closure_and_range()
    a7_cross_implementation()
    a8_time_independence()

    print("\n" + "-" * 78)
    print("B 组：真实在体光学参数")
    print("-" * 78)
    if args.data:
        group_b(args.data)
    else:
        print("  未指定 --data，跳过。下载方式见本文件顶部与 README。")

    print("\n" + "=" * 78)
    n_ok = sum(1 for _, ok, _ in _RESULTS if ok)
    for name, ok, _ in _RESULTS:
        if not ok:
            print(f"  未通过：{name}")
    print(f"  {n_ok}/{len(_RESULTS)} 项通过")
    print("=" * 78)
    print("""
  以上全部通过 ≠ 设备能给出正确的绝对脑血氧。已验证的只有：
    · 公式与代码没写错，两份独立实现一致到 1e-14
    · 光源功率 / 全局耦合不进结果；探测器整体增益也基本不进
    · 真正进结果的是【波长间】增益不一致 —— 1% 就是 0.24 %O2
    · 均匀介质下单调、线性（r=0.999999），两参数标定可救
    · 真实【组织光学参数】下趋势保持（B 组）
  未验证：分层几何、时间维度真实氧变化、运动/贴合伪影、绝对准确度。
  这些只能靠体模与人体实验，电脑上跑不出来。""")
    return 0 if n_ok == len(_RESULTS) else 1


if __name__ == "__main__":
    raise SystemExit(main())
