# -*- coding: utf-8 -*-
"""SSR 的收益随平均次数怎么变 —— 从 29 个 trial 一路降到 1 个。

动机
    ssr_v2.py 测出的「CNR 改善 5.3%」，是在【事件锁定平均 29~38 个 trial】
    之后的边际收益。29 次平均已经把不相关噪声压了 √29≈5.4 倍，留给 SSR 的
    空间本来就小。

    但本项目的产品形态是 1 Hz 连续监护、**没有平均**
    （config.OUTPUT_CHANNEL = "S1_D1_ssr"），每一帧当场出结果。那种场景下
    SSR 面对的是未经平均的原始生理噪声。

    所以真正该问的是：SSR 的相对收益随平均次数怎么变？
    若 N 越小收益越大，则 5.3% 是下界，产品场景下应显著更高。

方法
    每个通道的三条浓度时序（no_ssr / ssr_good / ssr_bad）只算一次，
    然后对同一条时序反复抽 N 个事件做锁定平均，重复 N_BOOT 次取中位。
    这样 N 之间的差异只来自平均次数，不含滤波/回归的随机性。

    判据仍是 ssr_good 与 ssr_bad 的对比 —— 坏短距是阴性对照，
    它的曲线若也随 N 变化，说明看到的是平均次数本身的效应，与 SSR 无关。

用法
    SSR_DATA_ROOT=<数据目录> python tools/ssr_trial_sweep.py

---------------------------------------------------------------------------
实测结果（2026-08-31，Dataset II，14 被试，阳性组 132 通道，每点重抽 60 次）
---------------------------------------------------------------------------
  平均次数   no_ssr CNR   ssr_good CNR   good/no_ssr   bad/no_ssr
         1         8.93           8.51      1.0164       0.9709
         2         9.44           9.58      0.9869       1.0085
         4        10.39          10.59      1.0172       1.0084
         8        11.85          11.92      1.0205       0.9763
        16        13.40          13.43      1.0392       0.9930
        29        14.39          14.03      1.0573       0.9999

⚠️ **本文件立项时的假设被推翻了。** 原假设是「平均次数越少，SSR 越值钱」，
   实测方向相反：收益随平均次数【增加】而增加（1 次 1.6% -> 29 次 5.7%），
   N=2 时甚至跌到 0.987，低于阴性对照。

   一个说得通但未经验证的解释：SSR 去掉的是与短距相关的【系统性生理】成分，
   不是随机噪声。随机噪声按 1/√N 被平均掉，系统性成分不会，所以平均次数越多，
   残余基线噪声里系统性的占比越高，SSR 能拿掉的比例也越大。

   阴性对照 bad/no_ssr 全程停在 0.97~1.01，未随 N 漂移 —— 故上述趋势
   不是平均次数本身造成的假象。

⚠️ 因此 ssr_v2.py 的 5.3% 是【上限】，不是下限。此前把它当下限是错的。

⚠️ 但这个结论**不能直接搬到产品场景**：本文件的 CNR 定义在事件锁定平均上，
   N=1 仍是「已知 onset 的单次锁定」，与 1 Hz 连续监护（根本没有事件）
   不是一回事。连续监护下 SSR 的价值应换指标衡量 —— 例如静息态下
   ΔHb 走线的方差抑制、或与系统性生理信号的相关下降，
   数据集里的 resting.snirf 正好可以直接做这件事。
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent))
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

import ssr_v2                                                    # noqa: E402
import ssr_validate as S                                         # noqa: E402
from fnirs_pipeline.mbll import (short_separation_regression,    # noqa: E402
                                 smart_bandpass)
from fnirs_pipeline.mbll_core import (hemoglobin_extinctions,    # noqa: E402
                                      intensities_to_od_changes)

N_TRIALS = (1, 2, 4, 8, 16, 29)
N_BOOT = 60
PRE, POST = 5.0, 25.0
PEAK_LO, PEAK_HI = 3.0, 12.0
RNG = np.random.default_rng(20260831)


def series_for_subject(sub: str):
    """返回 [(通道键, 分组, {路径: HbO 浓度时序}), ...] 与 (onsets, fs)。"""
    rec = S.load(S.ROOT / sub / "resting_hrf_100.snirf")
    fs = rec["fs"]
    longs, shorts = S.channels(rec)
    if len(shorts) < 2:
        return [], None, None
    qs = {sk: ssr_v2.short_quality(rec, sk, shorts) for sk in shorts}
    sk_good, sk_bad = max(qs, key=qs.get), min(qs, key=qs.get)
    if qs[sk_good] <= 0.5:                     # 与 ssr_v2 的统计口径一致
        return [], None, None
    never = S.never_injected(sub, "100")

    ex = hemoglobin_extinctions([float(w) for w in rec["wl"]])
    Ainv = np.linalg.pinv(ex * (ssr_v2.PPF * ssr_v2.D_CM))
    ons = (rec["st"][:, 0] * fs).astype(int)
    sod = {sk: intensities_to_od_changes(
               rec["d"][:, [shorts[sk][w] for w in sorted(shorts[sk])]].T)
           for sk in (sk_good, sk_bad)}

    out = []
    for lk, wm in longs.items():
        raw = rec["d"][:, [wm[w] for w in sorted(wm)]].T
        if np.any(raw <= 0):
            continue
        lod = intensities_to_od_changes(raw)
        paths = {"no_ssr": lod}
        for tag, sk in (("ssr_good", sk_good), ("ssr_bad", sk_bad)):
            paths[tag], _ = short_separation_regression(lod, sod[sk])
        conc = {t: (Ainv @ smart_bandpass(od, fs))[0] for t, od in paths.items()}
        out.append((lk, lk in never, conc))
    return out, ons, fs


def cnr_at(x, onsets, fs, n_sub) -> float:
    """抽 n_sub 个事件做锁定平均，返回 CNR 中位（N_BOOT 次）。"""
    vals = []
    reps = 1 if n_sub >= len(onsets) else N_BOOT
    for _ in range(reps):
        sel = onsets if n_sub >= len(onsets) else RNG.choice(
            onsets, size=n_sub, replace=False)
        avg, tt = S.epoch(x, sel, fs, PRE, POST)
        if avg is None:
            continue
        base = avg[tt < 0].std()
        if base <= 0:
            continue
        w = (tt >= PEAK_LO) & (tt <= PEAK_HI)
        vals.append(abs(avg[w][np.argmax(np.abs(avg[w]))]) / base)
    return float(np.median(vals)) if vals else np.nan


def main() -> int:
    subs = sorted(d.name for d in S.ROOT.iterdir() if d.is_dir())
    # acc[n][path] = 该平均次数下、各阳性通道的 CNR
    acc = {n: {p: [] for p in ("no_ssr", "ssr_good", "ssr_bad")} for n in N_TRIALS}
    n_ch = 0
    for sub in subs:
        chans, ons, fs = series_for_subject(sub)
        if not chans:
            continue
        # 阳性组判定与 ssr_v2 一致：用 no_ssr 路径的全量恢复率 > 0.5
        for lk, never, conc in chans:
            if never:
                continue
            avg, tt = S.epoch(conc["no_ssr"], ons, fs, PRE, POST)
            if avg is None:
                continue
            w = (tt >= PEAK_LO) & (tt <= PEAK_HI)
            if avg[w][np.argmax(np.abs(avg[w]))] * 1e6 / S.HRF_TRUE_UM["100"]["HbO"] <= 0.5:
                continue
            n_ch += 1
            for n in N_TRIALS:
                for p in acc[n]:
                    acc[n][p].append(cnr_at(conc[p], ons, fs, n))
        print(f"  {sub}: 累计阳性通道 {n_ch}")

    print("\n" + "=" * 78)
    print(f"SSR 收益 vs 平均次数（阳性组 {n_ch} 通道，每点 {N_BOOT} 次重抽）")
    print("=" * 78)
    print("  平均次数   no_ssr CNR   ssr_good CNR   ssr_bad CNR   good/no_ssr   bad/no_ssr")
    for n in N_TRIALS:
        a = np.array(acc[n]["no_ssr"])
        g = np.array(acc[n]["ssr_good"])
        b = np.array(acc[n]["ssr_bad"])
        m = np.isfinite(a) & np.isfinite(g) & np.isfinite(b) & (a > 0)
        print("  %8d %12.2f %14.2f %13.2f %13.4f %12.4f"
              % (n, np.median(a[m]), np.median(g[m]), np.median(b[m]),
                 np.median(g[m] / a[m]), np.median(b[m] / a[m])))
    print("\n  读法：bad/no_ssr 是阴性对照，全程应停在 1.0 附近；它若跟着动，")
    print("        说明看到的是平均次数本身的效应，与 SSR 的空间特异性无关。")
    print("        good/no_ssr 的走向由数据决定，见文件头「实测结果」。")
    return 0


if __name__ == "__main__":
    if not S.ROOT.exists():
        raise SystemExit(f"找不到 {S.ROOT}，设置 SSR_DATA_ROOT 或下载数据")
    raise SystemExit(main())
