# -*- coding: utf-8 -*-
"""srs_cal 自检 —— 无需硬件，全部在仿真闭环里跑。

    python -m srs_cal.selftest          （在 software/ 目录下）

查七件事：
  1 正演与 tools/srs_reference.py 是否仍然一致
  2 光谱插值质量（留一交叉验证）
  3 12 块在 3.0/4.0 cm 下的信号是否落在可测范围
  4 闭环：注入已知 G(λ)，标定能否原样恢复
  5 插值误差 → TOI 的误差地板
  6 验收判据能否抓住串扰
  7 剔除 950 nm 的量化依据（在体实测）
  8 采集链路：造帧 → 解析 → xlsx → 标定

⚠️ 本自检证明的是【代码链路自洽】，不证明【模型对真实头部成立】。
   真实头部为分层结构，本包全部基于均匀半无限假设。
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))
sys.path.insert(0, str(_HERE.parent / "tools"))
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from fnirs_pipeline.mbll_core import hemoglobin_extinctions      # noqa: E402
from fnirs_pipeline.srs import MIN_COUNTS, compute_toi           # noqa: E402
from srs_cal import hardware as hw                               # noqa: E402
from srs_cal import phantom                                      # noqa: E402
from srs_cal.calibrate import calibrate_gain                     # noqa: E402
from srs_cal.forward import attenuation_matrix, intensity_matrix # noqa: E402

LN10 = np.log(10.0)
WL = hw.WAVELENGTHS_NM
RHO = hw.RHO_CM
N_PH = phantom.REFRACTIVE_INDEX
_RESULTS: list[tuple[str, bool]] = []


def _check(name: str, ok: bool, detail: str = "") -> None:
    _RESULTS.append((name, ok))
    print(f"      {'[通过]' if ok else '[未通过]'} {name}" + (f" —— {detail}" if detail else ""))


def tissue_intensity(so2: float = 0.70, thb: float = 60e-6, gain=None):
    """组织的模拟光强。gain: 斜率域真实增益误差 (n_wl,)。"""
    E = hemoglobin_extinctions(list(map(float, WL)), hw.EXTINCTION_TABLE)
    mua = (E[:, 0] * so2 + E[:, 1] * (1 - so2)) * thb * LN10
    musp = hw.MUSP_REF_800 * (WL / 800.0) ** (-hw.MUSP_EXPONENT)
    A = attenuation_matrix(mua, musp, RHO, hw.N_TISSUE)
    if gain is not None:
        A = A + np.asarray(gain)[:, None] * (RHO - RHO.mean())[None, :]
    return 10 ** (-A) * 1e12


# ---------------------------------------------------------------------------
def t1_forward_matches_reference() -> None:
    print("\n  1  正演 vs tools/srs_reference.py")
    from srs_cal.forward import reflectance as ours
    from srs_reference import reflectance as ref
    worst = 0.0
    for mua in (0.05, 0.10, 0.15):
        for musp in (5.0, 12.0, 25.0):
            for n in (1.4, 1.54):
                a = np.asarray(ours(RHO, mua, musp, n), float)
                b = np.asarray(ref(RHO, mua, musp, n), float)
                worst = max(worst, float(np.abs(a / b - 1).max()))
    _check("1 两份正演逐点一致", worst < 1e-12, f"最大相对差 {worst:.2e}")


def t2_interpolation_quality() -> None:
    print("\n  2  光谱插值质量（留一交叉验证，12 块中位）")
    loo = phantom.loocv_error()
    print("      认证λ      μa 误差     μs′ 误差")
    worst_a = worst_s = 0.0
    for w, (ea, es) in loo.items():
        print(f"      {w:6.0f}    {ea*100:7.2f}%    {es*100:8.2f}%")
        worst_a, worst_s = max(worst_a, ea), max(worst_s, es)
    _check("2 插值误差在 10% 以内", worst_a < 0.10 and worst_s < 0.10,
           f"μa 最差 {worst_a*100:.2f}%，μs′ 最差 {worst_s*100:.2f}%")
    print("      器件λ 的插值状态：", end="")
    print("、".join(f"{w:.0f}={phantom.interpolation_mode(w)}" for w in WL))


def t3_signal_range() -> None:
    print("\n  3  12 块在 3.0/4.0 cm 下的信号范围")
    mua, musp = phantom.interpolate_to(WL)
    A4_lo, A4_hi, cnt_min = 9e9, 0.0, 9e9
    for i, code in enumerate(phantom.BLOCK_CODES):
        A = attenuation_matrix(mua[i], musp[i], RHO, N_PH)
        I = intensity_matrix(mua[i], musp[i], RHO, N_PH)
        A4_lo, A4_hi = min(A4_lo, A[:, -1].min()), max(A4_hi, A[:, -1].max())
        cnt_min = min(cnt_min, I[:, -1].min())
    print(f"      A(4cm) 跨度 {A4_lo:.2f} ~ {A4_hi:.2f} OD"
          f"（最暗/最亮差 {10**(A4_hi-A4_lo):.0f} 倍）")
    print(f"      最暗块在 scale=1e12 下的最远端计数 {cnt_min:.3g}"
          f"（MIN_COUNTS={MIN_COUNTS}）")
    _check("3 最暗块仍高于计数下限（按仿真 scale）", cnt_min > MIN_COUNTS,
           f"{cnt_min:.3g} > {MIN_COUNTS}")
    print("      ⚠️ scale 为仿真假设值，真机动态范围须实测 A2（最亮）与 D4（最暗）")


def t4_closed_loop() -> None:
    print("\n  4  闭环：注入已知 G(λ)，标定能否恢复")
    rng = np.random.default_rng(0)
    G_true = np.array([0.020, -0.012, 0.008, -0.018])
    mua, musp = phantom.interpolate_to(WL)
    I = np.zeros((12, WL.size, RHO.size))
    for i in range(12):
        A = attenuation_matrix(mua[i], musp[i], RHO, N_PH)
        A = A + G_true[:, None] * (RHO - RHO.mean())[None, :]
        A = A + rng.normal(0, 0.002, A.shape)
        I[i] = 10 ** (-A) * 1e12
    res = calibrate_gain(I, WL, RHO, refractive_index=N_PH,
                         baseline_dispersion=np.full(WL.size, 0.004))
    err = np.abs(res.gain_correction - G_true).max()
    print("      " + res.report().replace("\n", "\n      "))
    _check("4 G(λ) 恢复误差 < 0.002", err < 0.002, f"最大 {err:.5f}")
    # 注入的是常数 G，一次干净标定不该报任何异常。这一条专抓判据误报 ——
    # 最初 TREND_T_CRITICAL 用裸 2.228 时，这里稳定报 670/810 两处假漂移。
    _check("4 干净标定不触发任何验收异常", res.passed,
           "；".join(res.flags) if res.flags else "")
    toi_un = compute_toi(tissue_intensity(gain=G_true), hw.geometry()).toi_raw
    toi_cal = compute_toi(tissue_intensity(gain=G_true),
                          hw.geometry(gain_correction=res.gain_correction)).toi_raw
    toi_ref = compute_toi(tissue_intensity(), hw.geometry()).toi_raw
    print(f"      组织 TOI：真值链路 {toi_ref:.2f}，不校正 {toi_un:.2f}，"
          f"校正后 {toi_cal:.2f} %O₂")
    _check("4 校正把 TOI 拉回真值链路 0.3 %O₂ 内",
           abs(toi_cal - toi_ref) < 0.3, f"残差 {toi_cal-toi_ref:+.3f} %O₂")


def t5_interpolation_floor() -> None:
    print("\n  5  插值误差 → TOI 的误差地板")
    rng = np.random.default_rng(1)
    G_true = np.array([0.020, -0.012, 0.008, -0.018])
    mua, musp = phantom.interpolate_to(WL)
    toi_ref = compute_toi(tissue_intensity(), hw.geometry()).toi_raw
    print("      插值误差    标定后 TOI 偏差（中位，60 次）")
    floor = {}
    for lvl in (0.0, 0.02, 0.05):
        errs = []
        for _ in range(60):
            I = np.zeros((12, WL.size, RHO.size))
            for i in range(12):
                A = attenuation_matrix(mua[i], musp[i], RHO, N_PH)
                A = A + G_true[:, None] * (RHO - RHO.mean())[None, :]
                A = A + rng.normal(0, 0.002, A.shape)
                I[i] = 10 ** (-A) * 1e12
            # 认证值带误差 ⇒ 预测斜率带误差 ⇒ G 带误差
            saved = phantom.MUA_CERT.copy(), phantom.MUSP_CERT.copy()
            phantom.MUA_CERT[:] = saved[0] * (1 + lvl * rng.normal(0, 1, saved[0].shape))
            phantom.MUSP_CERT[:] = saved[1] * (1 + lvl * rng.normal(0, 1, saved[1].shape))
            try:
                res = calibrate_gain(I, WL, RHO, refractive_index=N_PH,
                                     baseline_dispersion=np.full(WL.size, 1e9))
            finally:
                phantom.MUA_CERT[:], phantom.MUSP_CERT[:] = saved
            t = compute_toi(tissue_intensity(gain=G_true),
                            hw.geometry(gain_correction=res.gain_correction)).toi_raw
            errs.append(abs(t - toi_ref))
        floor[lvl] = float(np.median(errs))
        print(f"      {lvl*100:7.0f}%    {floor[lvl]:.3f} %O₂")
    _check("5 认证不确定度 2% 下误差地板 < 1 %O₂", floor[0.02] < 1.0,
           f"{floor[0.02]:.3f} %O₂")


def t6_qc_catches_crosstalk() -> None:
    print("\n  6  验收判据能否抓住串扰（近→远）")
    rng = np.random.default_rng(2)
    G_true = np.array([0.020, -0.012, 0.008, -0.018])
    mua, musp = phantom.interpolate_to(WL)

    def run(c: float) -> float:
        I = np.zeros((12, WL.size, RHO.size))
        for i in range(12):
            A = attenuation_matrix(mua[i], musp[i], RHO, N_PH)
            A = A + G_true[:, None] * (RHO - RHO.mean())[None, :]
            A = A + rng.normal(0, 0.002, A.shape)
            J = 10 ** (-A) * 1e12
            J[:, 1] = J[:, 1] + c * J[:, 0]          # 近端漏进远端
            I[i] = J
        return float(calibrate_gain(I, WL, RHO, refractive_index=N_PH,
                                    baseline_dispersion=np.full(WL.size, 1e9)
                                    ).dispersion.max())

    base = run(0.0)
    print(f"      无串扰基线离散 {base:.5f}")
    print("      串扰      块间离散     倍数     3× 判据")
    ok = True
    for c in (0.001, 0.005, 0.01):
        d = run(c)
        hit = d > 3.0 * base
        print(f"      {c*100:5.2f}%   {d:9.5f}   {d/base:6.1f}x   "
              f"{'抓住' if hit else '漏掉'}")
        if c >= 0.005 and not hit:
            ok = False
    _check("6 ≥0.5% 串扰被 3× 判据抓住", ok)
    print("      注：0.1% 串扰低于判据灵敏度，但其 TOI 影响本身也 <0.05 %O₂")

    # 族系误报率 —— 判据必须不冤枉好机器
    n_fp = 0
    for _ in range(120):
        I = np.zeros((12, WL.size, RHO.size))
        for i in range(12):
            A = attenuation_matrix(mua[i], musp[i], RHO, N_PH)
            A = A + G_true[:, None] * (RHO - RHO.mean())[None, :]
            A = A + rng.normal(0, 0.002, A.shape)
            I[i] = 10 ** (-A) * 1e12
        r = calibrate_gain(I, WL, RHO, refractive_index=N_PH,
                           baseline_dispersion=np.full(WL.size, 3.0 * base))
        n_fp += 0 if r.passed else 1
    rate = n_fp / 120.0
    _check("6 干净系统的族系误报率 < 15%", rate < 0.15, f"{rate*100:.1f}%（120 次）")


# 10 名成人前额时域 DOS 实测（Zenodo 10.5281/zenodo.15828014）导出的结论：
# 用本器件四波长拟纯 HbO/HbR，各波长残差 =（实测 − 预测）/ 实测。
# 由 `validate_950.py` 从原始数据集复现；此处作为常量固化，使自检不依赖数据集。
INVIVO_RESIDUAL = {670.0: (-0.1, 0.4), 770.0: (+0.3, 1.3),
                   810.0: (-0.5, 0.7), 850.0: (+0.2, 0.5), 950.0: (+40.0, 2.5)}
INVIVO_SO2_4WL = (77.6, 3.6)
INVIVO_SO2_5WL = (87.2, 3.1)


def t7_why_drop_950() -> None:
    print("\n  7  剔除 950 nm 的量化依据（10 人前额在体实测）")
    print("      λ(nm)   纯血红蛋白模型残差")
    for w in hw.WAVELENGTHS_ALL_NM:
        m, s = INVIVO_RESIDUAL[float(w)]
        tag = "   <- 带外" if w in hw.WL_REFERENCE_ONLY_NM else ""
        print(f"      {w:5.0f}   {m:+6.1f}% ± {s:4.1f}%{tag}")
    inband = max(abs(INVIVO_RESIDUAL[float(w)][0]) for w in WL)
    out = INVIVO_RESIDUAL[950.0][0]
    _check("7 在用四波长的纯血红蛋白残差均 <2%", inband < 2.0, f"最大 {inband:.1f}%")
    _check("7 950nm 带外残差比带内大一个量级以上", out / max(inband, 1e-9) > 10,
           f"{out:.1f}% vs {inband:.1f}%")
    d = INVIVO_SO2_5WL[0] - INVIVO_SO2_4WL[0]
    print(f"      四波长 SO2 {INVIVO_SO2_4WL[0]:.1f} ± {INVIVO_SO2_4WL[1]:.1f} %"
          f"  →  加 950 后 {INVIVO_SO2_5WL[0]:.1f} ± {INVIVO_SO2_5WL[1]:.1f} %"
          f"（{d:+.1f}）")
    _check("7 加入 950 会显著抬高 SO2，故剔除", d > 5.0, f"{d:+.1f} %O2")


def t8_acquisition_chain() -> None:
    """协议 → xlsx → 标定 的端到端链路。守住采集软件与标定之间的接口。"""
    print("\n  8  采集链路：造帧 → 解析 → xlsx → 标定")
    import os
    import tempfile
    from datetime import datetime

    from srs_cal.acquire import RHOS, WLS, Take, write_workbook
    from srs_cal.protocol import FrameStream, build_frame
    from srs_cal.run_calibration import load_xlsx

    G_true = np.array([0.020, -0.012, 0.008, -0.018])
    wl5, rho = np.array(WLS), np.array(RHOS)
    mua5, musp5 = phantom.interpolate_to(wl5)
    G5 = np.append(G_true, 0.0)                    # 950 不注入
    rng = np.random.default_rng(42)

    takes = []
    for i, code in enumerate(phantom.BLOCK_CODES):
        A = attenuation_matrix(mua5[i], musp5[i], rho, N_PH)
        A = A + G5[:, None] * (rho - rho.mean())[None, :]
        I0 = 10 ** (-A) * 4e11
        blob = b""
        for _ in range(20):
            for a, w in enumerate(WLS):
                for b, r in enumerate(RHOS):
                    v = I0[a, b] * (1 + 0.004 * rng.standard_normal())
                    blob += build_frame(w, r, int(np.clip(v, 0, 2 ** 32 - 1)))
        st = FrameStream()
        frames = st.feed(blob)
        buf: dict = {}
        for f in frames:
            buf.setdefault(f.key, []).append(f.adc)
        tk = Take(block=code, index=1, timestamp=datetime.now().strftime("%F %T"))
        for k, v in buf.items():
            a_ = np.asarray(v, float)
            tk.mean[k], tk.std[k], tk.count[k] = a_.mean(), a_.std(ddof=1), a_.size
        takes.append(tk)
    _check("8 解析出的帧覆盖全部 10 个波长×距离组合",
           all(t.complete() for t in takes))
    _check("8 流式拆包无丢弃、无校验错",
           st.n_dropped == 0 and st.n_bad_checksum == 0,
           f"丢弃 {st.n_dropped}B，校验错 {st.n_bad_checksum}")

    path = Path(tempfile.gettempdir()) / "_srs_cal_chain.xlsx"
    try:
        n = write_workbook(path, takes, simulate=True)
        I, codes = load_xlsx(path)
        _check("8 xlsx 往返：块数与波长数正确（950 自动跳过）",
               I.shape == (12, WL.size, 2) and len(codes) == 12, f"{I.shape}，{n} 行明细")
        res = calibrate_gain(I, WL, RHO, block_codes=codes, refractive_index=N_PH,
                             baseline_dispersion=np.full(WL.size, 0.004))
        err = np.abs(res.gain_correction - G_true).max()
        _check("8 端到端恢复 G(λ)，误差 < 0.001", err < 0.001, f"最大 {err:.5f}")
        _check("8 链路数据不触发验收异常", res.passed,
               "；".join(res.flags) if res.flags else "")
    finally:
        if path.exists():
            os.unlink(path)


def main() -> int:
    print("=" * 72)
    print("srs_cal 自检 —— 新硬件 (670/770/810/850 nm, ρ=3.0/4.0 cm)")
    print(f"体模组 P3137034，12 块，认证于 {'/'.join(f'{w:.0f}' for w in phantom.CERT_WL_NM)} nm")
    print("=" * 72)
    for fn in (t1_forward_matches_reference, t2_interpolation_quality,
               t3_signal_range, t4_closed_loop, t5_interpolation_floor,
               t6_qc_catches_crosstalk, t7_why_drop_950, t8_acquisition_chain):
        fn()
    n_ok = sum(1 for _, ok in _RESULTS if ok)
    print("\n" + "=" * 72)
    print(f"{n_ok}/{len(_RESULTS)} 通过")
    for name, ok in _RESULTS:
        if not ok:
            print(f"  [未通过] {name}")
    print("=" * 72)
    return 0 if n_ok == len(_RESULTS) else 1


if __name__ == "__main__":
    raise SystemExit(main())
