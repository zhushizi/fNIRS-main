"""
GB 9706.271-2022 附录 BB 透射式体模试验 —— 离线计算与判据核验。

用法
----
自检（合成数据跑通全流程，不需要任何实测文件）：
    python tools/verify_bb.py --selftest

用预先整理好的光强表（参考侧、设备侧各一个 CSV）：
    python tools/verify_bb.py --ref ref.csv --dev dev.csv

    CSV 格式（表头固定，5 行，波长顺序任意）：
        wavelength_nm,I_A,I_B
        850,1.000e-6,4.466e-7
        ...

设备侧直接从 all_groups.csv 切段：
    python tools/verify_bb.py --ref ref.csv \
        --raw result_table/xxx/all_groups.csv \
        --acq-channel 2 --detector S1_D1 \
        --seg-a 30:90 --seg-b 130:190

说明
----
本脚本【只做离线计算】。附录 BB 要求参考系统先测、设备后测再比较，
天然是两阶段流程；且 A/B 两态需取 >=60 s 稳定段均值，不存在在线版本。

刻意绕过在线管线的两处：
  - intensities_to_od_changes(refs=None)：会用本段时间均值做基线，
    不是「状态 A 相对状态 B」，故本脚本一律传显式 refs=I_A。
  - smart_bandpass / compute_all_channel_concentrations：0.01~0.1 Hz
    带通会把 A->B 阶跃削掉。

关键参数 dpfs=1、distance_cm=1：
    generalized_mbll 中 a_hb = hb_ex * pathlength[:,None]，
    pathlength = DPF x distance_cm。令其为 1 时 a_hb 退化为纯 epsilon 矩阵，
    输出即标准的被测量 Delta_c * L（M·cm），而非除过光程的 Delta_c。
    实测与 pinv(E) 独立实现偏差 0.000e+00。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

if hasattr(sys.stdout, "reconfigure"):        # Windows 控制台默认 GBK，中文会乱码
    sys.stdout.reconfigure(encoding="utf-8")

from config import (  # noqa: E402
    DETECTOR_CHANNELS,
    WAVELENGTH_CHANNELS,
    mbll_wavelengths_nm,
)
from fnirs_pipeline.mbll import generalized_mbll  # noqa: E402
from fnirs_pipeline.mbll_core import (  # noqa: E402
    hemoglobin_extinctions,
    intensities_to_od_changes,
)

WL: list[float] = list(mbll_wavelengths_nm())
N_WL = len(WL)

# 判据（A1 为 BB.3.1 规范性要求，其余为自定，送检前应试跑校准）
CRIT_A1_PCT = 5.0    # 设备值与参考值的差值
CRIT_A2_PCT = 10.0   # 逐波长 dA 偏差（波长间增益一致性）
CRIT_A4_PCT = 10.0   # 重复性 CV
CRIT_WL_CV_PCT = 1.0   # 参考侧五波长 dA 一致性
CRIT_APERTURE_PCT = 5.0  # 孔径面积比 vs 光强比


# ---------------------------------------------------------------------------
# 核心计算
# ---------------------------------------------------------------------------
def delta_a(i_a: np.ndarray, i_b: np.ndarray) -> np.ndarray:
    """BB.2：dA(lambda) = log10[I_A/I_B]。使用项目自身的 OD 函数，传显式 refs。"""
    return intensities_to_od_changes(np.asarray(i_b, float).reshape(-1, 1),
                                     refs=np.asarray(i_a, float))[:, 0]


def dc_l_project(d_a: np.ndarray) -> np.ndarray:
    """BB.3~BB.5：用项目的 generalized_mbll，令 pathlength=1 使输出为 Delta_c*L。"""
    out = generalized_mbll(np.asarray(d_a, float).reshape(-1, 1), WL, [1.0] * N_WL, 1.0)
    return out[:2, 0]          # 只取 HbO / HbR；Cyt 不属于附录 BB 的被测量


def dc_l_independent(d_a: np.ndarray) -> np.ndarray:
    """独立实现（不调用项目任何反演代码），用于交叉核对。"""
    return np.linalg.pinv(hemoglobin_extinctions(WL)) @ np.asarray(d_a, float)


def dc_l_bb45(d_a: np.ndarray, i1: int = 0, i2: int = N_WL - 1) -> np.ndarray:
    """
    BB.4 / BB.5 双波长闭式解，作第三方核对。

    警告：GB 9706.271-2022 印刷的 BB.5 分子两项次序颠倒，直接实现会使
    Delta_c_HHb*L 反号。用标准自身算例可验证（690/830 nm、dA=log10(2)，
    标准给出 204 / 111 且注 1 载明「均为阳性」；按原文实现得 -111）。
    此处采用克拉默法则的正确形式。
    """
    e = hemoglobin_extinctions(WL)
    eo1, eh1 = e[i1]
    eo2, eh2 = e[i2]
    den = eo1 * eh2 - eo2 * eh1
    hbo = (eh2 * d_a[i1] - eh1 * d_a[i2]) / den
    hbr = (eo1 * d_a[i2] - eo2 * d_a[i1]) / den      # 已更正符号
    return np.array([hbo, hbr])


# ---------------------------------------------------------------------------
# 数据读取
# ---------------------------------------------------------------------------
def load_intensity_csv(path: str) -> tuple[np.ndarray, np.ndarray]:
    """读 wavelength_nm,I_A,I_B 三列，按 config 的波长顺序重排。"""
    df = pd.read_csv(path)
    need = {"wavelength_nm", "I_A", "I_B"}
    if not need.issubset(df.columns):
        raise ValueError(f"{path} 缺列，需要 {sorted(need)}，实际 {list(df.columns)}")
    by_wl = {float(r.wavelength_nm): (float(r.I_A), float(r.I_B))
             for r in df.itertuples()}
    missing = [w for w in WL if w not in by_wl]
    if missing:
        raise ValueError(f"{path} 缺少波长 {missing}；需要 {WL}")
    i_a = np.array([by_wl[w][0] for w in WL])
    i_b = np.array([by_wl[w][1] for w in WL])
    return i_a, i_b


def _parse_seg(text: str) -> tuple[float, float]:
    lo, _, hi = text.partition(":")
    return float(lo), float(hi)


def extract_from_raw(path: str, acq_channel: int, detector: str,
                     seg_a: str, seg_b: str, skip_s: float) -> tuple[np.ndarray, np.ndarray]:
    """
    从 all_groups.csv 切出 A/B 两段稳定段均值。

    列：Time (s), ChannelId, DetectorId, Channel, Wavelength, Value
    """
    df = pd.read_csv(path)
    det = next((d for d in DETECTOR_CHANNELS if d.name == detector), None)
    if det is None:
        raise ValueError(f"未知接收源 {detector}；可选 {[d.name for d in DETECTOR_CHANNELS]}")

    df = df[(df["ChannelId"] == acq_channel) & (df["DetectorId"] == det.code)]
    if df.empty:
        raise ValueError(f"筛选后无数据：ChannelId={acq_channel} DetectorId={det.code}")

    code_by_nm = {w.mbll_nm: w.code for w in WAVELENGTH_CHANNELS}

    def seg_mean(rng: str) -> np.ndarray:
        t0, t1 = _parse_seg(rng)
        t0 += skip_s                                   # 去掉切换后过渡段
        sub = df[(df["Time (s)"] >= t0) & (df["Time (s)"] <= t1)]
        out = np.empty(N_WL)
        for k, nm in enumerate(WL):
            v = sub[sub["Wavelength"] == code_by_nm[nm]]["Value"]
            if v.empty:
                raise ValueError(f"段 {rng} 波长 {nm}nm 无样本")
            out[k] = float(v.mean())
        return out

    return seg_mean(seg_a), seg_mean(seg_b)


# ---------------------------------------------------------------------------
# 报告
# ---------------------------------------------------------------------------
def _fmt(v: np.ndarray) -> str:
    return "  ".join(f"{x:+9.4f}" for x in v)


def report(i_a_ref, i_b_ref, i_a_dev, i_b_dev, aperture_ratio=None) -> bool:
    ok = True
    print("=" * 84)
    print("GB 9706.271-2022 附录 BB  透射式体模试验 —— 离线核验")
    print("=" * 84)
    print(f"波长 (nm): {[int(w) for w in WL]}\n")

    da_ref = delta_a(i_a_ref, i_b_ref)
    da_dev = delta_a(i_a_dev, i_b_dev)

    print("[1] BB.2  dA = log10(I_A/I_B)")
    print(f"    参考侧  {_fmt(da_ref)}   均值 {da_ref.mean():.4f} OD ({da_ref.mean()*10:.2f} dB)")
    print(f"    设备侧  {_fmt(da_dev)}   均值 {da_dev.mean():.4f} OD ({da_dev.mean()*10:.2f} dB)")

    cv = 100 * da_ref.std() / abs(da_ref.mean())
    good = cv <= CRIT_WL_CV_PCT
    ok &= good
    print(f"\n    参考侧五波长一致性 CV = {cv:.2f}%  (<={CRIT_WL_CV_PCT}%)  "
          f"{'PASS' if good else 'FAIL  -> 孔径前后散射材料可能不够厚'}")

    if aperture_ratio is not None:
        meas = float(np.mean(np.asarray(i_b_ref) / np.asarray(i_a_ref)))
        dev_pct = 100 * abs(meas - aperture_ratio) / aperture_ratio
        good = dev_pct <= CRIT_APERTURE_PCT
        ok &= good
        print(f"    孔径几何校验 S_apB/S_apA={aperture_ratio:.4f} vs 实测光强比 {meas:.4f}  "
              f"偏差 {dev_pct:.1f}%  (<={CRIT_APERTURE_PCT}%)  {'PASS' if good else 'FAIL'}")

    print("\n[2] 反演实现交叉核对（堵住「两侧同一 bug 互相抵消」）")
    p_ref = dc_l_project(da_ref)
    n_ref = dc_l_independent(da_ref)
    dmax = float(np.max(np.abs(p_ref - n_ref) / np.maximum(np.abs(n_ref), 1e-30)))
    good = dmax < 1e-9
    ok &= good
    print(f"    项目 generalized_mbll(dpf=1,d=1)  vs  独立 pinv(E)")
    print(f"    最大相对差 {dmax:.3e}   {'PASS' if good else 'FAIL  -> 项目反演实现有问题'}")

    bb = dc_l_bb45(da_ref)
    print(f"    BB.4/BB.5 双波长({int(WL[0])}/{int(WL[-1])}nm)闭式解  "
          f"HbO {bb[0]*1e6:+8.2f}  HbR {bb[1]*1e6:+8.2f} uM·cm")
    print(f"      与五波长最小二乘差 {100*(bb[0]-n_ref[0])/n_ref[0]:+.2f}% / "
          f"{100*(bb[1]-n_ref[1])/n_ref[1]:+.2f}%  (超定解与双波长解的正常差异)")

    print("\n[3] BB.3~BB.5  Delta_c*L  (uM·cm)")
    p_dev = dc_l_project(da_dev)
    print(f"    参考值  HbO {p_ref[0]*1e6:+9.2f}   HbR {p_ref[1]*1e6:+9.2f}")
    print(f"    设备值  HbO {p_dev[0]*1e6:+9.2f}   HbR {p_dev[1]*1e6:+9.2f}")

    print("\n[4] 判据")
    a1 = 100 * np.abs(p_dev - p_ref) / np.abs(p_ref)
    for k, nm in enumerate(("HbO", "HbR")):
        good = a1[k] < CRIT_A1_PCT
        ok &= good
        print(f"    A1  {nm} 相对偏差 {a1[k]:6.2f}%  (<{CRIT_A1_PCT}%, BB.3.1 规范性)  "
              f"{'PASS' if good else 'FAIL'}")

    a2 = 100 * np.abs(da_dev - da_ref) / np.abs(da_ref)
    good = bool(np.all(a2 <= CRIT_A2_PCT))
    ok &= good
    print(f"    A2  逐波长 dA 偏差 max {a2.max():5.2f}%  (<={CRIT_A2_PCT}%)  "
          f"{'PASS' if good else 'FAIL'}   [{_fmt(a2)}]")

    same = (p_dev[0] > 0) == (p_dev[1] > 0)
    ok &= same
    print(f"    A3  HbO 与 HbR 同向  {'PASS' if same else 'FAIL'}  "
          f"(灰变化的预期行为，见 BB.3.1 注 1)")
    print(f"    A4  重复性 CV<={CRIT_A4_PCT}%  —— 需 >=5 次重复，本脚本单次运行不判")

    print("\n" + "=" * 84)
    print(f"结论：{'全部通过' if ok else '存在不通过项'}")
    print("=" * 84)
    return ok


# ---------------------------------------------------------------------------
# 自检
# ---------------------------------------------------------------------------
def selftest() -> bool:
    print("=" * 84)
    print("自检：合成数据跑通全流程")
    print("=" * 84)
    e = hemoglobin_extinctions(WL)
    ok = True

    # 1) 标准算例复现（BB.3.1，690/830 nm，dA=log10(2)）
    eo1, eh1, eo2, eh2 = 0.3123, 2.1382, 1.0507, 0.7804
    d = np.log10(2.0)
    den = eo1 * eh2 - eo2 * eh1
    hbo = (eh2 * d - eh1 * d) / den * 1000
    hbr = (eo1 * d - eo2 * d) / den * 1000
    good = abs(hbo - 204) < 1.5 and abs(hbr - 111) < 1.5
    ok &= good
    print(f"[1] 标准 BB.3.1 算例复现  HbO {hbo:.1f} (期望 204)  HbR {hbr:.1f} (期望 111)  "
          f"{'PASS' if good else 'FAIL'}")

    # 2) 往返：给定真值 -> 正向造 dA -> 反演
    truth = np.array([1.0e-4, -5.0e-5])
    da = e @ truth
    rec = dc_l_project(da)
    err = float(np.max(np.abs(rec - truth) / np.abs(truth)))
    good = err < 1e-9
    ok &= good
    print(f"[2] 往返验证（含反号输入）最大相对误差 {err:.3e}  {'PASS' if good else 'FAIL'}")

    # 3) 项目 vs 独立实现
    err = float(np.max(np.abs(dc_l_project(da) - dc_l_independent(da))))
    good = err < 1e-18
    ok &= good
    print(f"[3] 项目实现 vs 独立实现  最大绝对差 {err:.3e}  {'PASS' if good else 'FAIL'}")

    # 4) BB.2 与手写 log10 一致
    i_a = np.array([1.000, 0.982, 0.951, 0.903, 0.874]) * 1e-6
    i_b = np.array([0.4466, 0.4372, 0.4270, 0.4022, 0.3928]) * 1e-6
    err = float(np.max(np.abs(delta_a(i_a, i_b) - np.log10(i_a / i_b))))
    good = err < 1e-15
    ok &= good
    print(f"[4] BB.2 与手写 log10(I_A/I_B) 一致  最大差 {err:.3e}  {'PASS' if good else 'FAIL'}")

    # 5) 端到端：设备侧无偏差，应全部通过
    print("\n[5] 端到端演示 A —— 设备侧与参考侧一致，应全部 PASS\n")
    pass_ok = report(i_a, i_b, i_a, i_b, aperture_ratio=0.447)

    # 6) 端到端：设备侧加 8% 单波长增益偏差，A1 应判 FAIL（证明判据会咬）
    print("\n[6] 端到端演示 B —— 设备侧 850nm 人为加 8% 增益偏差，应触发 FAIL\n")
    i_b_dev = i_b.copy()
    i_b_dev[0] *= 1.08
    fail_ok = report(i_a, i_b, i_a, i_b_dev, aperture_ratio=0.447)

    good = pass_ok and not fail_ok
    ok &= good
    print(f"\n[7] 判据自身有效性：无偏差时通过={pass_ok}，有偏差时通过={fail_ok}  "
          f"{'PASS' if good else 'FAIL  -> 判据未生效'}")

    print(f"\n自检结论：{'通过' if ok else '未通过'}")
    return ok


def main() -> int:
    p = argparse.ArgumentParser(description="附录 BB 透射式体模试验离线核验")
    p.add_argument("--selftest", action="store_true", help="合成数据自检，不需实测文件")
    p.add_argument("--ref", help="参考系统光强 CSV (wavelength_nm,I_A,I_B)")
    p.add_argument("--dev", help="设备光强 CSV，与 --raw 二选一")
    p.add_argument("--raw", help="all_groups.csv，配合 --seg-a/--seg-b 切段")
    p.add_argument("--acq-channel", type=int, default=2, help="ChannelId，默认 2（ch2 接收）")
    p.add_argument("--detector", default="S1_D1", help="接收源名，默认 S1_D1")
    p.add_argument("--seg-a", help="状态 A 时间段，格式 起:止（秒）")
    p.add_argument("--seg-b", help="状态 B 时间段")
    p.add_argument("--skip", type=float, default=10.0, help="每段开头跳过的过渡秒数，默认 10")
    p.add_argument("--aperture-ratio", type=float, help="设计孔径面积比 S_apB/S_apA，用于几何校验")
    a = p.parse_args()

    if a.selftest:
        return 0 if selftest() else 1

    if not a.ref:
        p.error("需要 --ref，或用 --selftest")
    i_a_ref, i_b_ref = load_intensity_csv(a.ref)

    if a.dev:
        i_a_dev, i_b_dev = load_intensity_csv(a.dev)
    elif a.raw:
        if not (a.seg_a and a.seg_b):
            p.error("--raw 需要同时给 --seg-a 和 --seg-b")
        i_a_dev, i_b_dev = extract_from_raw(a.raw, a.acq_channel, a.detector,
                                            a.seg_a, a.seg_b, a.skip)
    else:
        p.error("需要 --dev 或 --raw 之一")

    return 0 if report(i_a_ref, i_b_ref, i_a_dev, i_b_dev, a.aperture_ratio) else 1


if __name__ == "__main__":
    raise SystemExit(main())
