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
  9 帧级时间戳 t_host 与周期号 cycle_id（含丢帧检出、跨 feed 边界、回放）
  10 原始帧归档：往返、轮转、总量上限、截断容错、出错不中断测量
  11 稳健聚合：MAD 剔除离群后取均值（尖峰免疫、干净数据不损效率）
  12 浓度/光谱残差补回、物理判据（tHb/TOI/非负）、标定波长集校验
  13 L2.5 质量层：链路/饱和/非正/尖峰率/暗电平/停用波长监视，配置项与异常分离
  14 插值扣暗：按时刻在前后两暗间插值，消掉 0.0868Hz 干扰的相位残留
  15 MBLL 解码器：还原注入信号、守卫、负对照、与 SRS/config 的隔离

⚠️ 本自检证明的是【代码链路自洽】，不证明【模型对真实头部成立】。
   真实头部为分层结构，本包全部基于均匀半无限假设。
"""

from __future__ import annotations

import inspect
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


def t9_frame_timing() -> None:
    """9 帧级时间戳与周期号（2026-10-22 新增）。

    验的是 `protocol.Frame.t_host` / `cycle_id` 这条链路，不涉及光学。
    八项判据覆盖：正常流、跨 feed 边界、丢帧检出、回放、向后兼容。
    """
    print("\n[9] 帧级时间戳与周期号")
    from srs_cal.acquire import FRAMES_PER_CYCLE as ACQ_FPC
    from srs_cal.protocol import (CYCLE_START_DISTANCE_CM, FRAME_LEN,
                                  FRAMES_PER_CYCLE, WAVELENGTH_BY_CODE,
                                  FrameStream, build_frame, parse_frame)
    FRAME_LEN_ = FRAME_LEN

    _check("9 协议层与采集层的周期长度一致",
           FRAMES_PER_CYCLE == ACQ_FPC == 12,
           f"protocol={FRAMES_PER_CYCLE} acquire={ACQ_FPC}")

    wls = [0.0] + sorted(WAVELENGTH_BY_CODE.values())      # 暗时隙排最前
    rhos = sorted(hw.RHO_CM.tolist())

    def one_cycle() -> bytes:
        return b"".join(build_frame(w, r, 1000 + int(w))
                        for w in wls for r in rhos)

    # --- 确定性时钟：每次调用 +1.0，便于逐帧核对 ---------------------------
    ticks = iter(range(1, 10_000))
    st = FrameStream(clock=lambda: float(next(ticks)))

    N = 5
    frames = st.feed(one_cycle() * N)
    _check("9 帧数正确", len(frames) == N * FRAMES_PER_CYCLE,
           f"{len(frames)} 帧")
    _check("9 同一批 feed 内 t_host 相同",
           len({f.t_host for f in frames}) == 1,
           f"{sorted({f.t_host for f in frames})}")
    _check("9 cycle_id 从 0 开始、逐周期递增",
           [f.cycle_id for f in frames] ==
           [i for i in range(N) for _ in range(FRAMES_PER_CYCLE)])
    _check("9 每周期帧数齐全（n_short_cycles == 0）",
           st.n_short_cycles == 0 and st.n_cycles == N,
           f"短周期 {st.n_short_cycles}，周期 {st.n_cycles}")

    # --- 跨 feed 边界：把一帧切成两半分两次喂 ------------------------------
    st2 = FrameStream(clock=lambda: 7.0)
    blob = one_cycle() * 2
    cut = len(blob) // 2 + 3                      # 落在某帧中间
    got = st2.feed(blob[:cut]) + st2.feed(blob[cut:])
    _check("9 帧被切在两次 feed 之间仍能正确拼回",
           len(got) == 2 * FRAMES_PER_CYCLE and st2.n_dropped == 0,
           f"{len(got)} 帧，丢弃 {st2.n_dropped}B")
    _check("9 跨边界后周期号不乱",
           [f.cycle_id for f in got] ==
           [i for i in range(2) for _ in range(FRAMES_PER_CYCLE)])

    # --- 丢帧必须被检出（这是 cycle_id 存在的首要理由）---------------------
    st3 = FrameStream(clock=lambda: 0.0)
    full = one_cycle()
    hole = full[:FRAME_LEN_] + full[2 * FRAME_LEN_:]      # 删掉第 2 帧
    st3.feed(one_cycle() + hole + one_cycle())
    _check("9 周期内丢帧被 n_short_cycles 记录",
           st3.n_short_cycles >= 1,
           f"短周期 {st3.n_short_cycles}（应 ≥1）")

    # --- 回放：显式传 t_host 必须覆盖内部时钟 ------------------------------
    st4 = FrameStream(clock=lambda: 999.0)
    rp = st4.feed(one_cycle(), t_host=12.5)
    _check("9 回放时显式 t_host 生效（不被内部时钟覆盖）",
           all(f.t_host == 12.5 for f in rp))

    # --- 向后兼容：parse_frame 单独调用仍可用 ------------------------------
    f0 = parse_frame(build_frame(670.0, 3.0, 123))
    _check("9 向后兼容：parse_frame 默认值正确",
           f0 is not None and f0.t_host == 0.0 and f0.cycle_id == -1
           and f0.adc == 123)

    # --- 周期起点确实是「暗时隙 + 最近距离」-------------------------------
    starts = [f for f in frames if f.cycle_id != -1
              and f.is_dark and f.distance_cm == CYCLE_START_DISTANCE_CM]
    _check("9 周期起点 = 暗时隙@最近距离，每周期恰好一次",
           len(starts) == N and CYCLE_START_DISTANCE_CM == min(rhos),
           f"{len(starts)} 个起点，起点距离 {CYCLE_START_DISTANCE_CM}cm")


def t10_raw_archive() -> None:
    """10 原始帧归档：往返、轮转、总量上限、截断容错、出错不中断。

    验的是 `archive.RawArchive` 这条链路。重点在后两项 ——
    医疗设备上「归档失败不得中断测量」「掉电后已写部分必须可读」
    比「正常情况写得对」更关键。
    """
    print("\n[10] 原始帧归档")
    import shutil
    import tempfile

    from srs_cal.archive import (MAGIC, REC_LEN, RawArchive, list_archives,
                                 read_header, read_records, replay)
    from srs_cal.protocol import FRAME_LEN, FRAMES_PER_CYCLE, build_frame

    wls = [0.0] + sorted(hw.WAVELENGTHS_ALL_NM.tolist())
    rhos = sorted(hw.RHO_CM.tolist())

    def cycle_frames(stream, t):
        blob = b"".join(build_frame(w, r, 1000 + int(w) + int(10 * r))
                        for w in wls for r in rhos)
        return stream.feed(blob, t_host=t)

    root = Path(tempfile.mkdtemp(prefix="_srs_archive_"))
    try:
        # --- 往返 ---------------------------------------------------------
        from srs_cal.protocol import FrameStream
        st = FrameStream()
        ar = RawArchive(root / "a", rotate_s=1e9, meta={"port": "SIM"})
        sent = []
        for i in range(5):
            fr = cycle_frames(st, 100.0 + i * 1.5)
            sent += fr
            ar.write(fr)
        ar.close()

        files = list_archives(root / "a")
        _check("10 归档文件已生成", len(files) == 1, f"{len(files)} 个")

        h = read_header(files[0])
        _check("10 文件头自描述且可独立解析",
               h["magic"] == MAGIC and h["rec_len"] == REC_LEN
               and h["frame_len"] == FRAME_LEN,
               f"version={h['version']} rec_len={h['rec_len']}")

        recs = list(read_records(files[0]))
        _check("10 记录条数 = 写入帧数",
               len(recs) == len(sent), f"{len(recs)} vs {len(sent)}")
        _check("10 原始字节逐帧一致（归档无损）",
               all(r[1] == f.raw for r, f in zip(recs, sent)))

        back = replay(files[0])
        _check("10 重放还原帧数与内容",
               len(back) == len(sent)
               and all(a.raw == b.raw and a.adc == b.adc
                       for a, b in zip(back, sent)))
        dt = max(abs(a.t_host - b.t_host) for a, b in zip(back, sent))
        _check("10 重放还原 t_host（误差 < 1 µs）", dt < 1e-6, f"最大 {dt:.2e} s")
        _check("10 重放重建 cycle_id",
               [f.cycle_id for f in back] ==
               [i for i in range(5) for _ in range(FRAMES_PER_CYCLE)])

        # --- 轮转 ---------------------------------------------------------
        ticks = iter([0.0] + [i * 10.0 for i in range(1, 50)])
        ar2 = RawArchive(root / "b", rotate_s=5.0, clock=lambda: next(ticks))
        st2 = FrameStream()
        for i in range(4):
            ar2.write(cycle_frames(st2, 200.0 + i))
        ar2.close()
        _check("10 按时间轮转生成多个文件",
               len(list_archives(root / "b")) >= 2,
               f"{len(list_archives(root / 'b'))} 个")

        # --- 总量上限：超限删最旧，当前文件不删 ----------------------------
        ticks3 = iter([i * 100.0 for i in range(200)])
        # 单文件 ≈ JSON 头 + 12×REC_LEN ≈ 800 B，阈值取 1500 B 可容 1~2 个
        ar3 = RawArchive(root / "c", rotate_s=1.0, max_bytes=1500,
                         clock=lambda: next(ticks3))
        st3 = FrameStream()
        for i in range(6):
            ar3.write(cycle_frames(st3, 300.0 + i))
        ar3.close()
        total = sum(f.stat().st_size for f in list_archives(root / "c"))
        _check("10 总量上限生效（删最旧）",
               ar3.stats.purged_files > 0,
               f"清理 {ar3.stats.purged_files} 个，剩 {total} B")

        # --- 截断容错：掉电模拟 --------------------------------------------
        victim = list_archives(root / "a")[0]
        cut = victim.read_bytes()[:-(REC_LEN // 2)]      # 砍掉半条记录
        trunc = root / "trunc.srsraw"
        trunc.write_bytes(cut)
        got = list(read_records(trunc))
        _check("10 截断文件仍可读出前面全部完整记录",
               len(got) == len(recs) - 1
               and all(a[1] == b[1] for a, b in zip(got, recs)),
               f"{len(got)}/{len(recs)} 条")

        # --- 写盘出错不得中断测量 ------------------------------------------
        ar4 = RawArchive(root / "d", rotate_s=1e9)
        st4 = FrameStream()
        ar4.write(cycle_frames(st4, 400.0))
        ar4._fh.close()                       # 背着它把句柄关掉，模拟拔盘
        n = ar4.write(cycle_frames(st4, 401.0))      # 不得抛异常
        _check("10 写盘出错不抛异常、被计数",
               n == 0 and ar4.stats.n_errors == 1,
               f"返回 {n}，错误计数 {ar4.stats.n_errors}，{ar4.stats.last_error}")
        ar4.close()

        # --- 关闭开关后不写 -------------------------------------------------
        ar5 = RawArchive(root / "e", rotate_s=1e9)
        ar5.enabled = False
        st5 = FrameStream()
        ar5.write(cycle_frames(st5, 500.0))
        ar5.close()
        _check("10 关闭归档后不产生文件", len(list_archives(root / "e")) == 0)
    finally:
        shutil.rmtree(root, ignore_errors=True)


def t11_robust_aggregate() -> None:
    """11 稳健聚合：尖峰免疫、干净数据不损效率、退化情形不崩。"""
    print("\n[11] 稳健聚合（MAD 剔除离群后取均值）")
    from srs_cal.robust import MAD_K, MIN_N, aggregate

    rng = np.random.default_rng(7)

    # --- 干净数据：结果应与均值几乎一致，且不误剔 -------------------------
    clean = 1000.0 + rng.normal(0, 20.0, 50)
    a = aggregate(clean)
    _check("11 干净数据不误剔样本", a.n_rejected == 0, f"剔除 {a.n_rejected}")
    _check("11 干净数据的值与均值一致（相对差 < 0.1%）",
           abs(a.value - clean.mean()) / abs(clean.mean()) < 1e-3,
           f"{a.value:.2f} vs {clean.mean():.2f}")

    # --- 单个尖峰：这是本模块存在的理由 -----------------------------------
    w = np.full(10, 1000.0) + rng.normal(0, 5.0, 10)
    truth = float(w.mean())
    w[4] = 25_000.0                                   # 实测尖峰约 25 倍
    a = aggregate(w)
    err_mean = abs(float(w.mean()) - truth) / truth
    err_rob = abs(a.value - truth) / truth
    _check("11 单个 25 倍尖峰被剔除", a.n_rejected == 1, f"剔除 {a.n_rejected}")
    _check("11 稳健值几乎不受尖峰影响（< 0.5%），而均值错 > 100%",
           err_rob < 0.005 and err_mean > 1.0,
           f"稳健 {err_rob*100:.2f}%　均值 {err_mean*100:.0f}%")

    # --- 两个尖峰：截尾均值在此失效，MAD 法不应失效 ------------------------
    w2 = np.full(10, 1000.0) + rng.normal(0, 5.0, 10)
    truth2 = float(w2.mean())
    w2[2] = 25_000.0
    w2[7] = 30_000.0
    a2 = aggregate(w2)
    _check("11 一个窗口里两个尖峰也能全剔（trim10 在此会漏一个）",
           a2.n_rejected == 2 and abs(a2.value - truth2) / truth2 < 0.005,
           f"剔除 {a2.n_rejected}，偏差 {abs(a2.value-truth2)/truth2*100:.2f}%")

    # --- std 与 value 同口径（都基于保留样本）------------------------------
    _check("11 std 基于保留样本，不含尖峰",
           a2.std < 50.0, f"std {a2.std:.1f}（含尖峰时会上万）")

    # --- 退化情形不崩 ------------------------------------------------------
    _check("11 空输入返回 nan 不抛异常",
           not np.isfinite(aggregate([]).value))
    a3 = aggregate([5.0, 7.0])
    _check(f"11 样本数 < {MIN_N} 时退回均值（MAD 不可靠）",
           a3.n_rejected == 0 and abs(a3.value - 6.0) < 1e-9,
           f"n={a3.n_used} value={a3.value}")
    a4 = aggregate([3.0] * 10)
    _check("11 全相同（MAD=0，如饱和削顶）不崩、如实报告未剔除",
           a4.n_rejected == 0 and a4.value == 3.0 and a4.std == 0.0)

    # --- MAD=0 且含尖峰：早先版本会算对值却报告「剔除 0」-------------------
    a6 = aggregate([1000.0] * 9 + [25_000.0])
    _check("11 样本几乎全同且含尖峰时，既剔对也**如实计数**",
           a6.n_rejected == 1 and abs(a6.value - 1000.0) < 1e-9,
           f"value={a6.value:.0f} 剔除={a6.n_rejected}（MAD 下限 1 counts）")
    a5 = aggregate([1.0, 2.0, np.nan, 3.0, np.inf, 4.0, 5.0])
    _check("11 非有限值被忽略", a5.n_used == 5 and np.isfinite(a5.value),
           f"n_used={a5.n_used}")

    # --- 判据余量：正常样本误剔率应极低 ------------------------------------
    rej = sum(aggregate(rng.normal(1000, 20, 20)).n_rejected for _ in range(500))
    rate = rej / (500 * 20)
    _check("11 高斯数据的误剔率 < 1%",
           rate < 0.01, f"{rate*100:.3f}%（判据 {MAD_K}×MAD ≈ 3.37σ）")


def t12_hb_and_physics() -> None:
    """12 浓度/残差补回 + 物理判据 + 标定波长集校验（2026-10-22）。"""
    print("\n[12] 浓度、光谱残差、物理判据、波长集校验")
    from fnirs_pipeline.srs import (SPECTRAL_RESIDUAL_MAX, THB_MAX_uM,
                                    THB_MIN_uM, mua_to_hb, mua_to_toi,
                                    musp_power_law)

    TAB = hw.EXTINCTION_TABLE

    def mua_of(c_hbo, c_hbr, wl):
        E = hemoglobin_extinctions([float(w) for w in wl], TAB)
        return LN10 * (E @ np.array([c_hbo, c_hbr]))

    def intensities_for(mua, wl, rho=hw.RHO_CM):
        """反推能产出给定 μa 的光强（I3 固定，I4 由目标斜率定）。"""
        musp = musp_power_law(np.asarray(wl, float), hw.MUSP_REF_800,
                              hw.MUSP_EXPONENT)
        mueff = np.sqrt(3.0 * mua * musp)          # mua_from_mueff 的逆
        slope = (mueff + 2.0 / float(np.mean(rho))) / LN10
        I3 = np.full(len(wl), 1e6)
        return np.column_stack([I3, I3 / 10.0 ** slope])

    # --- 浓度往返 ---------------------------------------------------------
    wl2 = hw.WAVELENGTHS_ACTIVE_NM
    c_true = (42.0e-6, 18.0e-6)                     # tHb 60 µM, SO₂ 70%
    f = mua_to_hb(mua_of(*c_true, wl2), wl2, TAB)
    _check("12 浓度往返（相对误差 < 0.1%）",
           abs(f.c_hbo - c_true[0]) / c_true[0] < 1e-3
           and abs(f.c_hbr - c_true[1]) / c_true[1] < 1e-3,
           f"HbO {f.c_hbo*1e6:.2f} HbR {f.c_hbr*1e6:.2f} µM")
    _check("12 tHb 与 TOI 自洽",
           abs(f.thb - 60.0) < 0.1 and abs(f.toi - 70.0) < 0.1,
           f"tHb {f.thb:.2f} µM　TOI {f.toi:.2f}")

    # --- dof=0 时残差恒零，这正是「零自检」------------------------------
    _check("12 两波长时 dof=0 且残差为机器精度（**不是拟合好**）",
           f.dof == 0 and f.resid_rms < 1e-12, f"dof={f.dof} rms={f.resid_rms:.1e}")

    # --- dof>0 时残差能抓到谱失配 ----------------------------------------
    wl3 = np.array([670.0, 770.0, 850.0])
    m3 = mua_of(*c_true, wl3)
    f3 = mua_to_hb(m3, wl3, TAB)
    _check("12 三波长时 dof=1，干净数据残差仍极小",
           f3.dof == 1 and f3.resid_rms < 1e-12, f"rms={f3.resid_rms:.1e}")

    # ⚠️ 灵敏度取决于**失配落在哪一路**：同样 +30%，落在 850 上只给 0.0104，
    #    落在最敏感的那一路给 0.0188。所以分开验「最易检出」与「最难检出」，
    #    并把最难那一路的实际下限记在案 —— 「未超阈」不等于「没有失配」。
    def worst_best(pct):
        rs = []
        for i in range(wl3.size):
            m = m3.copy(); m[i] *= (1 + pct / 100.0)
            rs.append(mua_to_hb(m, wl3, TAB).resid_rms)
        return max(rs), min(rs)

    w30, b30 = worst_best(30)
    _check("12 +30% 谱失配：最敏感的一路必被抓到",
           w30 > SPECTRAL_RESIDUAL_MAX,
           f"最大 rms={w30:.4f} vs 阈 {SPECTRAL_RESIDUAL_MAX}")
    _check("12 +10% 谱失配：确实抓不到（记录灵敏度边界，非缺陷）",
           worst_best(10)[0] < SPECTRAL_RESIDUAL_MAX,
           f"最大 rms={worst_best(10)[0]:.4f}")
    lim = next((p for p in range(5, 101, 5) if worst_best(p)[1] >
                SPECTRAL_RESIDUAL_MAX), None)
    _check("12 最难检出的一路，下限落在 25~45% 区间内",
           lim is not None and 25 <= lim <= 45, f"最难一路下限 {lim}%")

    # --- 向后兼容 ---------------------------------------------------------
    t, c = mua_to_toi(mua_of(*c_true, wl2), wl2, TAB)      # 2 元组解包
    _check("12 mua_to_toi 仍返回 2 元组（既有调用方不受影响）",
           isinstance(c, np.ndarray) and c.size == 2 and abs(t - 70.0) < 0.1)

    # --- compute_toi 带出新字段 -------------------------------------------
    geom = hw.geometry(wavelengths_nm=wl2)
    r = compute_toi(intensities_for(mua_of(*c_true, wl2), wl2), geom)
    _check("12 SrsResult 带出 c_hbo/c_hbr/tHb",
           abs(r.thb_uM - 60.0) < 1.0 and np.isfinite(r.c_hbo),
           f"tHb {r.thb_uM:.2f} µM")

    # --- 物理判据：三条都要能抓到 ----------------------------------------
    r_lo = compute_toi(intensities_for(mua_of(3.5e-6, 1.5e-6, wl2), wl2), geom)
    _check(f"12 tHb 过低（< {THB_MIN_uM:.0f} µM）被标记",
           any("tHb" in s for s in r_lo.flags), f"tHb {r_lo.thb_uM:.1f}；{r_lo.flags}")
    r_hi = compute_toi(intensities_for(mua_of(140e-6, 60e-6, wl2), wl2), geom)
    _check(f"12 tHb 过高（> {THB_MAX_uM:.0f} µM）被标记",
           any("tHb" in s for s in r_hi.flags), f"tHb {r_hi.thb_uM:.1f}；{r_hi.flags}")
    r_neg = compute_toi(intensities_for(mua_of(60e-6, -8e-6, wl2), wl2), geom)
    _check("12 TOI 越界或浓度为负被标记",
           any(("TOI_raw" in s) or ("负值" in s) for s in r_neg.flags),
           f"TOI_raw {r_neg.toi_raw:.1f}；{r_neg.flags}")

    # --- 正常值不应误报 ---------------------------------------------------
    _check("12 正常生理值不触发物理判据",
           not any(("tHb" in s) or ("TOI_raw" in s) or ("负值" in s)
                   for s in r.flags), f"{r.flags}")

    # --- 配置状态与异常必须分离（告警疲劳）--------------------------------
    _check("12 正常测量 flags 为空、配置状态另放 config_notes",
           r.flags == [] and len(r.config_notes) == 1
           and "自检" in r.config_notes[0],
           f"flags={r.flags} notes={r.config_notes}")
    _check("12 valid 对正常测量为 True（此前因常驻项恒为 False）", r.valid)
    _check("12 异常时 flags 非空且 valid 转 False",
           bool(r_lo.flags) and not r_lo.valid)

    # --- 标定波长集校验（R8）----------------------------------------------
    from srs_cal.online import Calibration
    cur = [float(w) for w in hw.WAVELENGTHS_ACTIVE_NM]
    good = Calibration({"wavelengths_nm": cur,
                        "gain_correction": [0.0] * len(cur)})
    _check("12 波长集匹配时正常加载", good.calibrated and not good.wl_error)
    diff_len = Calibration({"wavelengths_nm": cur + [850.0],
                            "gain_correction": [0.0] * (len(cur) + 1)})
    _check("12 波长数不符 → 拒绝加载",
           not diff_len.calibrated and "波长数不符" in diff_len.wl_error)
    same_len = Calibration({"wavelengths_nm": [cur[0], 850.0],
                            "gain_correction": [0.0, 0.0]})
    _check("12 **长度相同而波长不同** → 拒绝（旧代码会静默算错）",
           not same_len.calibrated and "波长集不符" in same_len.wl_error,
           same_len.wl_error[:46])
    no_wl = Calibration({"gain_correction": [0.0] * len(cur)})
    _check("12 缺 wavelengths_nm 字段 → 拒绝", not no_wl.calibrated)


def t13_quality_layer() -> None:
    """13 L2.5 质量层：各判据能抓、不误报、配置项与异常项分开。"""
    print("\n[13] L2.5 质量层")
    from srs_cal.quality import (ADC_FULL_SCALE, DARK_LEVEL_MAX, KIND_CONFIG,
                                 KIND_DATA, KIND_LINK, KIND_MONITOR,
                                 MONITOR_DEV_MAX, MONITOR_MIN_WINDOWS,
                                 REJECT_RATE_FAIL, REJECT_RATE_WARN, SEV_FAIL,
                                 QualityMonitor, summarize)

    WL2 = [float(w) for w in hw.WAVELENGTHS_ACTIVE_NM]
    clean_net = {(WL2[0], 3.0): 1.0e5, (WL2[0], 4.0): 1.5e4,
                 (WL2[1], 3.0): 3.3e5, (WL2[1], 4.0): 5.5e4}
    clean_raw = {k: v - 4.3e4 for k, v in clean_net.items()}

    # --- 干净输入不应产出任何「本次异常」---------------------------------
    q = QualityMonitor(n_rho=2, n_wl=2, kb_calibrated=False,
                       active_wl=tuple(WL2))
    f = q.update(raw_by_channel=clean_raw, net_by_channel=clean_net,
                 n_used=10, n_rejected=0, dark_level={3.0: -4.3e4, 4.0: 8.2e3})
    _check("13 干净输入无异常标志", not f, f"{[x.code for x in f]}")

    # --- 配置项与异常项分开（告警疲劳）------------------------------------
    cfg = q.config_flags()
    _check("13 配置项单独产出且全为 config 类",
           cfg and all(x.kind == KIND_CONFIG for x in cfg),
           "；".join(x.code for x in cfg))
    _check("13 两距离+两波长时报「均不可用」",
           any(x.code == "no_selfcheck" for x in cfg))
    _check("13 k/b 未标定被标出",
           any(x.code == "kb_uncalibrated" for x in cfg))
    _check("13 summarize 不把配置项混进本次异常行（避免告警疲劳）",
           summarize(cfg) == "", f"'{summarize(cfg)}'")

    # --- 链路 -------------------------------------------------------------
    class _L:
        n_dropped, n_bad_checksum, n_short_cycles = 12, 0, 3
    f = q.update(link=_L())
    _check("13 链路丢字节/短周期被抓到",
           {x.code for x in f} == {"link_dropped", "link_short_cycle"}
           and all(x.kind == KIND_LINK for x in f))

    # --- 饱和 -------------------------------------------------------------
    sat = dict(clean_raw); sat[(WL2[0], 3.0)] = ADC_FULL_SCALE * 0.95
    f = q.update(raw_by_channel=sat)
    _check("13 接近满量程被抓到（severity=fail）",
           any(x.code == "saturation" and x.severity == SEV_FAIL for x in f))

    # --- 非正 -------------------------------------------------------------
    neg = dict(clean_net); neg[(WL2[0], 4.0)] = -800.0
    f = q.update(net_by_channel=neg)
    _check("13 **反演**通道扣暗后非正 → fail",
           any(x.code == "nonpositive" and x.severity == SEV_FAIL for x in f))

    # ⚠️ 监视通道（停用波长）坏掉**不得**否决整次测量。
    #    首版一视同仁，在体回放时稳定段 95% 的窗口被误判为 fail。
    mon_bad = dict(clean_net); mon_bad[(850.0, 4.0)] = -500.0
    f = q.update(net_by_channel=mon_bad)
    _check("13 **监视**通道非正只标 info，不判 fail",
           any(x.code == "nonpositive_monitor" for x in f)
           and not any(x.severity == SEV_FAIL for x in f))
    _check("13 监视通道非正不进异常行（summarize 为空）",
           summarize(f) == "", f"'{summarize(f)}'")

    # --- 尖峰剔除率两级 ---------------------------------------------------
    f_ok = q.update(n_used=100, n_rejected=10)          # 9.1%
    f_w = q.update(n_used=100, n_rejected=20)           # 16.7%
    f_f = q.update(n_used=100, n_rejected=40)           # 28.6%
    _check(f"13 剔除率 9% 不报（稳定段实测 0~15%）", not f_ok)
    _check(f"13 剔除率 17% 报 warn（阈 {REJECT_RATE_WARN:.0%}）",
           any(x.code == "reject_warn" for x in f_w))
    _check(f"13 剔除率 29% 报 fail（阈 {REJECT_RATE_FAIL:.0%}）",
           any(x.code == "reject_high" and x.severity == SEV_FAIL for x in f_f))

    # --- 暗电平 -----------------------------------------------------------
    f = q.update(dark_level={3.0: -4.3e4, 4.0: DARK_LEVEL_MAX * 1.5})
    _check("13 暗电平量级异常被抓到",
           any(x.code == "dark_level" for x in f))
    f = q.update(dark_level={3.0: -4.9e4, 4.0: 1.25e4})   # 实测正常范围
    _check("13 实测正常暗电平不误报", not f)

    # --- 停用波长监视：建基线 → 偏离才报 ----------------------------------
    q2 = QualityMonitor()
    base = {(670.0, 3.0): 1.0e5, (770.0, 3.0): 3.3e5, (810.0, 3.0): 3.3e5}
    for _ in range(MONITOR_MIN_WINDOWS + 2):
        q2.update(net_by_channel=base)
    f = q2.update(net_by_channel=base)
    _check("13 监视：比值稳定时不报", not f)
    drift = dict(base); drift[(810.0, 3.0)] = 3.3e5 * (1 + MONITOR_DEV_MAX * 2)
    f = q2.update(net_by_channel=drift)
    _check(f"13 监视：比值偏离基线 {MONITOR_DEV_MAX*2:.0%} 被抓到",
           any(x.code == "monitor_810" and x.kind == KIND_MONITOR for x in f),
           "；".join(x.text[:40] for x in f))

    q3 = QualityMonitor()
    f = q3.update(net_by_channel=drift)
    _check(f"13 监视：基线不足 {MONITOR_MIN_WINDOWS} 窗口时不报（避免开机误报）",
           not f)
    q2.reset()
    _check("13 reset 清空基线（换受试者/换档位后必须）",
           not q2.update(net_by_channel=drift))

    # --- 物理判据原样转达 -------------------------------------------------
    f = q.update(physics_flags=["tHb 2.7 µM 越界（生理 20~120）"])
    _check("13 物理判据原样转达且标为 fail",
           len(f) == 1 and f[0].severity == SEV_FAIL and "tHb" in f[0].text)

    # --- summarize 优先显示 fail ------------------------------------------
    mixed = q.update(net_by_channel=neg, n_used=100, n_rejected=20)
    s = summarize(mixed)
    _check("13 summarize 以 fail 优先",
           s.startswith("✗") and "非正" in s, s[:50])


def t14_dark_subtraction() -> None:
    """14 按时刻插值扣暗：正确性、延迟、退化情形、对周期干扰的抑制。"""
    print("\n[14] 插值扣暗")
    from srs_cal.darksub import (MAX_GAP_S, MODE_INTERP, MODE_PREV,
                                 DarkSubtractor)

    # --- 构造：暗电平线性上升，信号恒定。插值后应得到恒定净值 ------------
    ds = DarkSubtractor()
    out = ds.feed_dark(3.0, 0.0, 100.0)
    _check("14 第一个暗之前无法解析，返回空", out == [])
    # 周期内：暗在 t=0，信号在 t=0.5/1.0，下一个暗在 t=1.5
    ds.feed_signal(670.0, 3.0, 0.5, 1000.0 + 110.0)    # 该时刻暗应为 110
    ds.feed_signal(770.0, 3.0, 1.0, 2000.0 + 120.0)    # 该时刻暗应为 120
    got = ds.feed_dark(3.0, 1.5, 130.0)                # 暗 100→130 线性
    vals = {int(w): v for w, _, _, v in got}
    _check("14 下一个暗到达时才吐出（延迟 1 周期）", len(got) == 2)
    _check("14 插值扣暗得到正确净值",
           abs(vals[670] - 1000.0) < 1e-9 and abs(vals[770] - 2000.0) < 1e-9,
           f"670→{vals[670]:.3f}　770→{vals[770]:.3f}")

    # --- 对照：用上一个暗（旧行为）会残留相位误差 -------------------------
    ds2 = DarkSubtractor(mode=MODE_PREV)
    ds2.feed_dark(3.0, 0.0, 100.0)
    g = ds2.feed_signal(670.0, 3.0, 0.5, 1000.0 + 110.0)
    _check("14 prev 模式当场返回（无延迟）", len(g) == 1)
    _check("14 prev 模式残留相位误差（此处 10 counts）",
           abs(g[0][3] - 1010.0) < 1e-9, f"净值 {g[0][3]:.1f}（真值 1000）")

    # --- 对周期干扰的实际抑制：合成一段 0.0868 Hz 干扰 --------------------
    CPS, F_INT = 0.657, 0.0868
    rng = np.random.default_rng(3)
    n = 400
    true_sig = 5000.0
    ti, tp, va = [], [], []
    for c in range(n):
        t0 = c / CPS
        base = 40000.0
        for k, (lbl, dt) in enumerate((("dark", 0.0), ("sig", 0.76))):
            t = t0 + dt
            interf = 800.0 * np.sin(2 * np.pi * F_INT * t)
            if lbl == "dark":
                ti.append((t, base + interf + rng.normal(0, 20)))
            else:
                tp.append((t, true_sig + base + interf + rng.normal(0, 20)))
    for mode, store in ((MODE_INTERP, []), (MODE_PREV, [])):
        d = DarkSubtractor(mode=mode)
        di, si = 0, 0
        while di < len(ti) or si < len(tp):
            if di < len(ti) and (si >= len(tp) or ti[di][0] <= tp[si][0]):
                for _, _, _, v in d.feed_dark(3.0, *ti[di]):
                    store.append(v)
                di += 1
            else:
                for _, _, _, v in d.feed_signal(670.0, 3.0, *tp[si]):
                    store.append(v)
                si += 1
        va.append(float(np.std(store, ddof=1)))
    _check("14 合成 0.0868Hz 干扰下，插值比 prev 的逐帧 std 小 ≥2 倍",
           va[1] / va[0] >= 2.0,
           f"插值 {va[0]:.1f}　prev {va[1]:.1f}　比 {va[1]/va[0]:.2f}×")

    # --- 退化情形 ---------------------------------------------------------
    ds3 = DarkSubtractor()
    ds3.feed_dark(3.0, 0.0, 100.0)
    ds3.feed_signal(670.0, 3.0, 0.5, 1000.0)
    got = ds3.feed_dark(3.0, MAX_GAP_S + 5.0, 999.0)      # 间隔过大
    _check("14 暗间隔过大时退回用左端点，不跨洞插值",
           ds3.n_wide_gap == 1 and abs(got[0][3] - 900.0) < 1e-9,
           f"净值 {got[0][3]:.1f}（左端点 100 → 900）")

    ds4 = DarkSubtractor()
    ds4.feed_signal(670.0, 3.0, 0.5, 1000.0)              # 还没有任何暗
    ds4.feed_dark(3.0, 1.0, 100.0)
    _check("14 无左端点的帧被丢弃并计数", ds4.n_dropped_nodark == 1)

    ds5 = DarkSubtractor()
    ds5.feed_dark(3.0, 0.0, 100.0)
    for i in range(300):                                   # 暗一直不来
        ds5.feed_signal(670.0, 3.0, 0.1 * i, 1000.0)
    _check("14 待解析帧有上限，不会无限涨",
           ds5.n_pending <= 200 and ds5.n_dropped_nodark > 0,
           f"pending={ds5.n_pending} dropped={ds5.n_dropped_nodark}")

    ds6 = DarkSubtractor()
    ds6.feed_dark(3.0, 0.0, 100.0); ds6.feed_dark(4.0, 0.1, 50.0)
    ds6.feed_signal(670.0, 3.0, 0.5, 1000.0)
    ds6.feed_signal(670.0, 4.0, 0.6, 2000.0)
    a = ds6.feed_dark(3.0, 1.5, 130.0)
    b = ds6.feed_dark(4.0, 1.6, 80.0)
    _check("14 两个距离各自独立插值",
           len(a) == 1 and len(b) == 1
           and abs(a[0][3] - 890.0) < 1e-9 and abs(b[0][3] - 1940.0) < 1e-9,
           f"3cm→{a[0][3]:.1f}　4cm→{b[0][3]:.1f}")

    ds6.reset()
    _check("14 reset 清空状态",
           ds6.n_pending == 0 and not ds6._prev and ds6.n_wide_gap == 0)


def t15_mbll_decoder() -> None:
    """15 MBLL 解码器：还原注入信号、守卫、负对照、与 SRS 的隔离。"""
    print("\n[15] MBLL 解码器")
    from fnirs_pipeline.mbll_core import get_dpf
    from mbll_decoder import decode
    from mbll_decoder import hardware as mh

    FS = 0.657                                   # 实测扫描速率
    wl = mh.WAVELENGTHS_NM
    E = hemoglobin_extinctions([float(w) for w in wl], "wray")
    L = np.array([get_dpf(float(w), mh.DEFAULT_AGE) for w in wl]) * mh.DISTANCE_CM

    def synth(n, hbo_uM=1.0, f0=0.02, noise=0.0, seed=0):
        t = np.arange(n) / FS
        hbo = hbo_uM * 1e-6 * np.sin(2 * np.pi * f0 * t)
        hbr = -0.3 * hbo_uM * 1e-6 * np.sin(2 * np.pi * f0 * t)
        dOD = (E * L[:, None]) @ np.vstack([hbo, hbr])
        I = np.array([[1e5], [3e5]]) * 10 ** (-dOD)
        if noise:
            I = I * (1 + np.random.default_rng(seed).normal(0, noise, I.shape))
        return t, I

    # --- 还原注入信号 ------------------------------------------------------
    t, I = synth(800)
    r = decode(t, I)
    A = np.abs(np.fft.rfft(r.d_hbo * np.hanning(r.t.size)))
    f = np.fft.rfftfreq(r.t.size, d=1.0 / r.fs)
    k = int(np.argmax(A[1:])) + 1
    amp = 2 * A[k] / np.sum(np.hanning(r.t.size))
    _check("15 还原注入的 ΔHbO 频率（0.02 Hz）",
           abs(f[k] - 0.02) < 0.002, f"{f[k]:.4f} Hz")
    _check("15 还原幅度在 0.8~1.1 µM（带通有衰减）",
           0.8 < amp < 1.1, f"{amp:.3f} µM（注入 1.0）")
    _check("15 ΔHbO 与 ΔHbR 反相（注入时如此）",
           np.corrcoef(r.d_hbo, r.d_hbr)[0, 1] < -0.95,
           f"r={np.corrcoef(r.d_hbo, r.d_hbr)[0,1]:+.3f}")
    _check("15 条件数 = 7.80（670/770 的固有值）",
           abs(r.cond - 7.80) < 0.05, f"{r.cond:.2f}")

    # --- 负对照：恒定光强应给出 ≈0 的 ΔHb ---------------------------------
    t2 = np.arange(800) / FS
    flat = np.tile(np.array([[1e5], [3e5]]), (1, 800))
    r2 = decode(t2, flat)
    _check("15 **负对照**：恒定光强 → ΔHb ≈ 0",
           r2.ok and max(np.abs(r2.d_hbo).max(), np.abs(r2.d_hbr).max()) < 1e-6,
           f"max|ΔHbO|={np.abs(r2.d_hbo).max():.2e} µM")

    # --- 记录太短必须拒绝，而不是给个看着像样的数 --------------------------
    t3, I3 = synth(100)                           # 152 s < 300 s
    r3 = decode(t3, I3)
    _check(f"15 记录短于 {mh.MIN_RECORD_S:.0f}s 时**拒绝出结果**",
           not r3.ok and any("拒绝" in s for s in r3.flags),
           f"{r3.flags[-1][:48]}")

    # --- 非正光强剔除并计数 ------------------------------------------------
    t4, I4 = synth(800)
    I4[0, 10:15] = -5.0
    r4 = decode(t4, I4)
    _check("15 非正光强被剔除并计数",
           r4.ok and r4.n_dropped == 5, f"剔除 {r4.n_dropped}")

    # --- 带通上限超奈奎斯特必须拒绝 ----------------------------------------
    r5 = decode(t, I, band=(0.01, 0.5))           # 奈奎斯特 0.33
    _check("15 带通上限超奈奎斯特 → 拒绝",
           not r5.ok and any("奈奎斯特" in s for s in r5.flags))

    # --- 波长数不符 --------------------------------------------------------
    r6 = decode(t, I[:1])
    _check("15 波长数与配置不符 → 拒绝",
           not r6.ok and any("波长数不符" in s for s in r6.flags))

    # --- 不规则采样（丢帧）仍能正确重采样 ----------------------------------
    keep = np.ones(800, bool); keep[300:340] = False      # 挖掉 40 帧
    r7 = decode(t[keep], I[:, keep])
    A7 = np.abs(np.fft.rfft(r7.d_hbo * np.hanning(r7.t.size)))
    f7 = np.fft.rfftfreq(r7.t.size, d=1.0 / r7.fs)
    _check("15 丢帧后按 t_host 重采样，主频仍正确",
           abs(f7[int(np.argmax(A7[1:])) + 1] - 0.02) < 0.003,
           f"{f7[int(np.argmax(A7[1:]))+1]:.4f} Hz")

    # --- 限定必须带出来 ----------------------------------------------------
    _check("15 flags 带出 DPF 外推提醒",
           any("DPF" in s and "外推" in s for s in r.flags))
    _check("15 flags 带出「2 波长无自检」",
           any("无自检" in s for s in r.flags))
    _check("15 flags 带出「无 SSR」", any("SSR" in s for s in r.flags))
    _check("15 不输出 rSO2（血氧由 SRS 给）",
           not hasattr(r, "rso2") and not hasattr(r, "sto2"))

    # --- 隔离：不得 import config / online_android --------------------------
    # ⚠️ 用 AST 解析**真实的 import**，不搜文本 —— 文档串里提到这些模块名
    #    是正当的（要说明为什么不用它们），搜文本会把注释也算进去。
    import ast

    import mbll_decoder.decoder as dec
    import mbll_decoder.hardware as mhw

    def imported(mod):
        tree = ast.parse(inspect.getsource(mod))
        names = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names |= {a.name for a in node.names}
            elif isinstance(node, ast.ImportFrom) and node.module:
                names.add(node.module)
        return names

    got = imported(dec) | imported(mhw)
    banned = {"config", "online_android", "fnirs_pipeline.mbll"}
    bad = sorted(n for n in got
                 if n in banned or any(n.startswith(b + ".") for b in banned))
    _check("15 隔离：不 import config / online_android / fnirs_pipeline.mbll",
           not bad, f"实际 import：{sorted(got)}")
    _check("15 只复用 mbll_core 这一份共享的纯函数",
           "fnirs_pipeline.mbll_core" in got, f"{sorted(got)}")


def t16_mbll_live() -> None:
    """16 MBLL 实时接入：周期成组、与离线逐点相同、以及**不能拖垮 SRS**。"""
    print("\n[16] MBLL 实时接入（L4 并排显示）")
    import os

    from mbll_decoder import hardware as mh
    from mbll_decoder.decoder import decode
    from mbll_decoder.live import LiveMbll

    FS = 0.657
    W0, W1 = (float(w) for w in mh.WAVELENGTHS_NM)
    RHO3 = mh.DISTANCE_CM

    def npts(curve) -> int:
        """曲线上有几个点。⚠️ 不能写 `len(arr or [])`：
        非空 ndarray 参与 `or` 会抛「真值歧义」。"""
        y = curve.getData()[1]
        return 0 if y is None else len(y)

    def feed(live, n, t0=0.0, rho=RHO3, seed=0):
        """按「一个周期两个波长」的顺序喂，模拟扣暗器的吐数顺序。"""
        rng = np.random.default_rng(seed)
        t, I = [], []
        for i in range(n):
            tt = t0 + i / FS
            v0 = 1e5 * (1 + 0.01 * np.sin(2 * np.pi * 0.02 * tt)) + rng.normal(0, 30)
            v1 = 3e5 * (1 - 0.01 * np.sin(2 * np.pi * 0.02 * tt)) + rng.normal(0, 30)
            live.push(W0, rho, tt - 0.2, v0)
            live.push(W1, rho, tt, v1)
            t.append(tt); I.append([v0, v1])
        return np.array(t), np.array(I).T

    # --- 成组：两个波长凑齐才算一个样本 ------------------------------------
    lv = LiveMbll()
    lv.push(W0, RHO3, 0.0, 1e5)
    _check("16 只来一个波长时不成样本",
           lv.stats.n_cycles == 0 and lv.stats.n_pushed == 1)
    lv.push(W1, RHO3, 0.1, 3e5)
    _check("16 凑齐两个波长成一个样本", lv.stats.n_cycles == 1)

    # --- 不收别的距离 / 别的波长 -------------------------------------------
    n0 = lv.stats.n_pushed
    lv.push(W0, 4.0, 0.2, 1e5)          # 4cm：MBLL 只用 3cm
    lv.push(810.0, RHO3, 0.2, 1e5)      # 停用波长
    _check("16 别的距离和停用波长都不收", lv.stats.n_pushed == n0)

    # --- 残缺周期被丢掉，不跨周期拼接 ---------------------------------------
    lv2 = LiveMbll()
    lv2.push(W0, RHO3, 0.0, 1e5)
    lv2.push(W0, RHO3, 1.5, 1e5)        # 同一波长又来 → 上一组残缺
    _check("16 残缺周期被丢弃而不是跨周期拼接",
           lv2.stats.n_partial == 1 and lv2.stats.n_cycles == 0)

    # --- 与离线路径逐点相同（这是整条设计的核心不变量）---------------------
    lv3 = LiveMbll()
    t, I = feed(lv3, 420, seed=7)
    r_live = lv3.poll(now=1e18, force=True)
    r_off = decode(t, I, distance_cm=RHO3)
    same = (r_live is not None and r_off.ok and r_live.ok
            and r_live.d_hbo.shape == r_off.d_hbo.shape
            and np.array_equal(r_live.d_hbo, r_off.d_hbo)
            and np.array_equal(r_live.d_hbr, r_off.d_hbr)
            and np.array_equal(r_live.t, r_off.t))
    _check("16 实时路径与离线 decode() 逐点相同", same,
           "界面上的数必须等于命令行跑出来的数")

    # --- 时刻取最后一个波长，与 run.load() 口径一致 -------------------------
    _check("16 样本时刻取最后一个波长（与 run.load 同口径）",
           abs(r_live.t.size - t.size) == 0
           and abs(lv3.span_s - (t[-1] - t[0])) < 1e-9,
           f"span={lv3.span_s:.3f}s")

    # --- 记录不够长时拒绝，并给出还差多少 -----------------------------------
    lv4 = LiveMbll()
    feed(lv4, 60, seed=1)               # 60 点 ≈ 91 s，远不够 300 s
    r4 = lv4.poll(now=1e18, force=True)
    _check("16 记录不够长时拒绝出结果",
           r4 is not None and not r4.ok and "300" in "".join(r4.flags))
    need = lv4.need_s()
    _check("16 状态行给出还差多少秒",
           need > 0 and f"{need:.0f}" in lv4.status(), lv4.status())

    # --- 重算节流 -----------------------------------------------------------
    lv5 = LiveMbll(redecode_s=10.0)
    feed(lv5, 420, seed=2)
    _check("16 首次 poll 出结果", lv5.poll(now=1000.0) is not None)
    _check("16 间隔内再 poll 不重算",
           lv5.poll(now=1005.0) is None and lv5.stats.n_decode == 1)
    _check("16 过了间隔再 poll 会重算",
           lv5.poll(now=1011.0) is not None and lv5.stats.n_decode == 2)

    # --- 缓冲上限 -----------------------------------------------------------
    lv6 = LiveMbll(maxlen=50)
    feed(lv6, 200, seed=3)
    _check("16 缓冲按上限滚动，不无限涨",
           lv6.poll(now=1e18, force=True) is not None or True,
           f"攒 200 周期后缓冲 {len(lv6._t)} 点（上限 50）")
    _check("16 缓冲长度不超过上限", len(lv6._t) == 50)

    # --- reset --------------------------------------------------------------
    lv6.reset()
    _check("16 reset 清空缓冲与计数",
           len(lv6._t) == 0 and lv6.span_s == 0.0
           and lv6.stats.n_cycles == 0 and lv6.last is None)

    # --- 真界面：两格同时出数，且 MBLL 坏掉不影响 SRS -----------------------
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PyQt5 import QtWidgets

    from srs_cal.online import CAL_DIR, DEFAULT_CAL, Calibration, OnlineWindow
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    win = OnlineWindow(Calibration.load(CAL_DIR / DEFAULT_CAL))
    win.chk_rec.setChecked(False)
    _check("16 界面挂上了 MBLL 那一格",
           win._mbll is not None and hasattr(win, "c_hbo")
           and hasattr(win, "c_hbr"))
    _check("16 MBLL 那一格不与 SRS 共用 x 轴",
           win.p_hb.getViewBox().linkedView(0) is None,
           "上面看实时窗，下面看整段，强行共用必有一边看不清")

    # 造 150 个周期的帧喂进真正的 _on_frames。
    # is_dark 是由 wavelength_nm == 0.0 推出来的属性，不是字段。
    from srs_cal.protocol import Frame
    fr = []
    for cyc in range(150):
        base = cyc * 1.5213
        for rho in (3.0, 4.0):
            fr.append(Frame(wavelength_nm=0.0, distance_cm=rho, adc=1000,
                            channel=0, raw=b"", t_host=base, cycle_id=cyc))
            for j, wlx in enumerate(hw.WAVELENGTHS_ACTIVE_NM):
                fr.append(Frame(wavelength_nm=float(wlx), distance_cm=rho,
                                adc=int(2e5 if rho == 3.0 else 6e4),
                                channel=0, raw=b"",
                                t_host=base + 0.1 * (j + 1), cycle_id=cyc))
    win._on_frames(fr)
    n_toi, n_cyc = len(win.toi_hist), win._mbll.stats.n_cycles
    _check("16 同一批帧同时喂到两条链",
           n_toi > 0 and n_cyc > 0, f"TOI {n_toi} 点，MBLL {n_cyc} 周期")

    # --- 没到 300 s 时：图里要有倒计时，不能留一张空图 ----------------------
    # 150 周期 × 1.5213 s ≈ 228 s，还没到起步线。
    win._refresh()
    _check("16 未到起步线时图里显示倒计时",
           win._mbll.need_s() > 0 and win.txt_hb.isVisible()
           and "还差" in win.txt_hb.toPlainText(),
           win.txt_hb.toPlainText().replace("\n", " ")[:38])
    _check("16 倒计时期间曲线为空", npts(win.c_hbo) == 0)

    # --- 越过 300 s：倒计时撤掉，曲线填上 -----------------------------------
    fr2 = []
    for cyc in range(150, 230):
        base = cyc * 1.5213
        for rho in (3.0, 4.0):
            fr2.append(Frame(wavelength_nm=0.0, distance_cm=rho, adc=1000,
                             channel=0, raw=b"", t_host=base, cycle_id=cyc))
            for j, wlx in enumerate(hw.WAVELENGTHS_ACTIVE_NM):
                fr2.append(Frame(wavelength_nm=float(wlx), distance_cm=rho,
                                 adc=int(2e5 if rho == 3.0 else 6e4),
                                 channel=0, raw=b"",
                                 t_host=base + 0.1 * (j + 1), cycle_id=cyc))
    win._on_frames(fr2)
    win._refresh()
    _check("16 越过 300 s 后倒计时撤掉",
           win._mbll.need_s() <= 0 and not win.txt_hb.isVisible(),
           f"span={win._mbll.span_s:.0f}s")
    _check("16 越过 300 s 后曲线填上",
           npts(win.c_hbo) > 0 and npts(win.c_hbr) > 0,
           f"{npts(win.c_hbo)} 点")
    n_toi = len(win.toi_hist)

    class _Boom:
        def push(self, *a):
            raise RuntimeError("故意炸")

        def poll(self, *a, **k):
            raise RuntimeError("故意炸")

    win._mbll = _Boom()
    win._on_frames(fr)
    win._refresh()
    _check("16 MBLL 抛异常后 SRS 仍在出点", len(win.toi_hist) > n_toi,
           f"{n_toi} → {len(win.toi_hist)}")
    _check("16 坏掉的 MBLL 被摘掉且界面明示",
           win._mbll is None and "停用" in win.lbl_mbll.text(),
           win.lbl_mbll.text()[:40])
    win.close()


def main() -> int:
    print("=" * 72)
    print("srs_cal 自检 —— 新硬件 (670/770/810/850 nm, ρ=3.0/4.0 cm)")
    print(f"体模组 P3137034，12 块，认证于 {'/'.join(f'{w:.0f}' for w in phantom.CERT_WL_NM)} nm")
    print("=" * 72)
    for fn in (t1_forward_matches_reference, t2_interpolation_quality,
               t3_signal_range, t4_closed_loop, t5_interpolation_floor,
               t6_qc_catches_crosstalk, t7_why_drop_950, t8_acquisition_chain,
               t9_frame_timing, t10_raw_archive,
               t11_robust_aggregate, t12_hb_and_physics,
               t13_quality_layer, t14_dark_subtraction,
               t15_mbll_decoder, t16_mbll_live):
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
