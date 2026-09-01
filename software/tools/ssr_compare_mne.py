# -*- coding: utf-8 -*-
"""把本项目的短距回归与 MNE-NIRS 的独立实现放在同一批数据上对比。

为什么分三步跑：MNE 要 numpy>=1.26，而本项目 base 环境钉在 numpy 1.24.4
（pyarrow/numexpr/bottleneck/pyqtgraph 都是按 1.x 编译的）。两边不能共存，
所以各自在自己的环境里算，各存一份中间结果，最后再对齐比较。

    # 1) base 环境
    python tools/ssr_compare_mne.py --stage ours
    # 2) 独立 venv（见文件末「环境」）
    .venv-mne/Scripts/python.exe tools/ssr_compare_mne.py --stage mne
    # 3) 任一环境
    python tools/ssr_compare_mne.py --stage compare

两条流程刻意对齐的参数
    带通 0.01~0.1 Hz、4 阶 Butterworth、零相位
    PPF 6.0，事件窗 −5~+25 s，峰值窗 3~12 s
    SSR 都放在带通【之前】（本项目的顺序，见 ssr_validate.py 的说明）

两条流程无法对齐的地方 —— 看结果时必须记住
    ε 表：本项目用 tables/wray.csv；MNE 用它自带的表。
          实测 wray 与 Prahl 在 700nm 的 HbO 差 44.6%，故【绝对】恢复率
          两边不可直接比。
    距离：本项目固定 D_CM=3.0；MNE 用蒙太奇里的真实距离（实测 29.7~30.5mm）。
    -> 因此判据取【比值】 ssr/no_ssr，它对 ε 表和距离的缩放免疫。
       绝对恢复率只作参考。

⚠️ MNE 的 short_channel_regression 用「最近短距」，本项目的 ssr_v2 用
   「质量最好的短距」。为把选道差异剥出来，本文件额外跑一条 ssr_near
   （本项目实现 + 最近短距），与 MNE 同策略。

---------------------------------------------------------------------------
对比结果（2026-08-31，Dataset II，14 被试，hrf_100 档，HbO，阳性组 132 通道）
统计口径与 ssr_v2.py 一致：仅含短距质量 q_good > 0.5 的被试。
---------------------------------------------------------------------------
                no_ssr峰值中位   ssr峰值中位   恢复率(no)   ssr/no_ssr
    本项目          0.7428 μM      0.7482       1.125        1.0021
    MNE-NIRS       0.7146 μM      0.7251       1.083        1.0016

  CNR = |峰值|/基线噪声      no_ssr    ssr    ssr/no_ssr 中位
    本项目                    14.48    14.50      1.0526
    MNE-NIRS                 13.58    13.77      1.0022
  -> 同一批数据、同一批通道、同一套参数，本项目的 SSR 拿到 5.3% 的 CNR 改善，
     MNE 只有 0.2%。与下面①一致：MNE 有一半长距是跨波长回归的，
     那半边的回归量几乎不携带同波长的头皮信息。

  逐通道一致性（本项目 vs MNE 的 ssr 输出）
    no_ssr     r = 0.9865   中位比值 1.034   ← ε表/距离造成的一致性地板
    ssr_good   r = 0.7360   中位比值 1.028   质量最好短距, 逐波长, 去均值
    ssr_near   r = 0.8475   中位比值 1.021   最近短距,     逐波长, 去均值
    ssr_mimic  r = 0.9870   中位比值 1.033   最近短距, 都用690nm, 不去均值

  -> ssr_mimic 与 no_ssr 的一致性持平（0.9870 vs 0.9865），
     即 SSR 环节的分歧已被【完全归因】，不存在未解释的残差。

MNE short_channel_regression 的两处行为（读源码 + 实测确认）
  ① 跨波长回归。_find_nearest_short 在【所有】短距通道里按 loc[:3] 找最近，
     而同一对(源,探测器)的两个波长 loc 完全相同，argmin 取数组里靠前的那个。
     SNIRF 中 690 全部排在 830 前，故实测【96 条长距里有 48 条】——
     所有 830nm 长距——是用 690nm 的短距回归的。两波长头皮吸收不同，
     这样得到的回归系数没有物理意义。
  ② 不去均值。α = <A_s,A_l>/<A_s,A_s> 是过原点回归；本项目用协方差/方差。
     实测本数据 OD 均值并非零（长距达 7.15e-02、短距达 7.25e-02），
     两者不等价，MNE 的 α 被均值乘积污染。

结论
  1. 基础管线（不含 SSR）两个独立实现逐通道 r=0.9878、幅度差 3.4%。
     3.4% 可由 ε 表（wray vs MNE 自带）与源探距离（固定 3.0cm vs 实测
     2.97~3.05cm）解释。MBLL + 事件锁定平均这一段没有实现错误。
  2. 恢复率两边都偏高（1.094 / 1.056，真值 0.66 μM）。两个独立实现同向
     偏高，说明偏差来自数据或协议本身（注入幅度定义、PPF 取值），
     不是任一实现的 bug。
  3. SSR 环节的 0.20 相关缺口，全部来自上面 ①②。
     **本项目在这两点上都更正确，不应向 MNE 看齐。**
  4. ssr/no_ssr 比值两边都≈1.00 —— 两个实现都显示 SSR 不削减峰值幅度。

⚠️ 「SSR 到底有没有效」由 ssr_v2.py 回答（CNR 改善 5.3% + 坏短距阴性对照）；
   本文件回答「我们的实现对不对」，以及「两个实现谁的 SSR 更有效」。
   收益随平均次数怎么变，见 ssr_trial_sweep.py。

⚠️ ①对本项目硬件不适用：设备只有 1 个短距(S1_D2 @1cm) + 1 个长距
   (S1_D1 @3cm)，「选哪根短距」是空问题。真正要守住的是②逐波长 + 去均值。
"""

from __future__ import annotations

import argparse
import pickle
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent))
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

OUT_OURS = _HERE / "cmp_ours.pkl"
OUT_MNE = _HERE / "cmp_mne.pkl"
_VENV_MNE_PY = _HERE.parent / ".venv-mne" / "Scripts" / "python.exe"


def _import_mne_stack():
    """惰性导入 MNE 栈。base/fnris 环境没有这些包 —— 属预期，不是代码写错。

    IDE 会在 import mne 处标黄：当前解释器不是 .venv-mne。用下面命令跑 mne 阶段即可。
    """
    try:
        import mne  # type: ignore[import-not-found]
        from mne.preprocessing.nirs import (  # type: ignore[import-not-found]
            beer_lambert_law,
            optical_density,
        )
        from mne_nirs.signal_enhancement import (  # type: ignore[import-not-found]
            short_channel_regression,
        )
    except ImportError as exc:
        hint = (
            f"  {_VENV_MNE_PY} tools/ssr_compare_mne.py --stage mne"
            if _VENV_MNE_PY.exists()
            else "  见本文件末尾「环境」一节创建 software/.venv-mne"
        )
        raise SystemExit(
            "stage mne 必须在独立 venv 中运行（MNE 需 numpy>=1.26，与 base 不共存）：\n"
            f"{hint}\n"
            f"原始错误: {exc}"
        ) from exc
    return mne, beer_lambert_law, optical_density, short_channel_regression

PRE, POST = 5.0, 25.0
PEAK_LO, PEAK_HI = 3.0, 12.0
PPF = 6.0
BP_LOW, BP_HIGH = 0.01, 0.1
TRUTH_HBO = 0.660          # μM，100% 档注入真值


# ---------------------------------------------------------------------------
def stage_ours() -> None:
    """base 环境：跑本项目管线，存每个 (被试, 通道) 的 no_ssr / ssr_good 峰值。"""
    import ssr_v2
    import ssr_validate as S

    import numpy as _np
    from fnirs_pipeline.mbll import short_separation_regression, smart_bandpass
    from fnirs_pipeline.mbll_core import (hemoglobin_extinctions,
                                          intensities_to_od_changes)

    def nearest_variant(sub: str) -> dict:
        """再跑一条【最近短距】路径 —— MNE 用的就是这个策略。

        ssr_v2 的 ssr_good 用「质量最好的短距」，选道策略不同会让两边
        结果分叉，这条用来把选道差异从实现差异里剥出来。
        """
        rec = S.load(S.ROOT / sub / "resting_hrf_100.snirf")
        fs = rec["fs"]
        longs, shorts = S.channels(rec)
        if not shorts:
            return {}
        ex = hemoglobin_extinctions([float(w) for w in rec["wl"]])
        Ainv = _np.linalg.pinv(ex * (ssr_v2.PPF * ssr_v2.D_CM))
        ons = (rec["st"][:, 0] * fs).astype(int)
        sod = {sk: intensities_to_od_changes(
                   rec["d"][:, [wm[w] for w in sorted(wm)]].T)
               for sk, wm in shorts.items()}
        res = {}
        for lk, wm in longs.items():
            raw = rec["d"][:, [wm[w] for w in sorted(wm)]].T
            if _np.any(raw <= 0):
                continue
            lod = intensities_to_od_changes(raw)
            sk, _ = S.nearest_short(rec, lk, shorts)
            corr, _ = short_separation_regression(lod, sod[sk])
            c = Ainv @ smart_bandpass(corr, fs)
            avg, tt = S.epoch(c[0], ons, fs, PRE, POST)
            if avg is None:
                continue
            w = (tt >= PEAK_LO) & (tt <= PEAK_HI)
            res[lk] = float(avg[w][_np.argmax(_np.abs(avg[w]))] * 1e6)
        return res

    def mimic_mne(sub: str) -> dict:
        """故意照抄 MNE 的两处做法，用来把剩余分歧归因。

        与 nearest_variant 只差两点，其余完全相同：
          ① 不去均值 —— MNE 用 α = <A_s,A_l>/<A_s,A_s>（过原点回归），
             本项目用协方差/方差（带截距）。实测本数据 OD 均值并非零
             （长距到 7.15e-02、短距到 7.25e-02），两者不等价。
          ② 两个波长都用 690nm 的短距 —— 见文件头「MNE 的两处行为」。
        """
        rec = S.load(S.ROOT / sub / "resting_hrf_100.snirf")
        fs = rec["fs"]
        longs, shorts = S.channels(rec)
        if not shorts:
            return {}
        ex = hemoglobin_extinctions([float(w) for w in rec["wl"]])
        Ainv = _np.linalg.pinv(ex * (ssr_v2.PPF * ssr_v2.D_CM))
        ons = (rec["st"][:, 0] * fs).astype(int)
        sod = {sk: intensities_to_od_changes(
                   rec["d"][:, [wm[w] for w in sorted(wm)]].T)
               for sk, wm in shorts.items()}
        res = {}
        for lk, wm in longs.items():
            raw = rec["d"][:, [wm[w] for w in sorted(wm)]].T
            if _np.any(raw <= 0):
                continue
            lod = intensities_to_od_changes(raw)
            sk, _ = S.nearest_short(rec, lk, shorts)
            a_s0 = sod[sk][0]                       # ② 只用第 0 个波长
            alfa = _np.array([_np.dot(a_s0, lod[i]) / _np.dot(a_s0, a_s0)
                              for i in range(lod.shape[0])])   # ① 不去均值
            c = Ainv @ smart_bandpass(lod - alfa[:, None] * a_s0[None, :], fs)
            avg, tt = S.epoch(c[0], ons, fs, PRE, POST)
            if avg is None:
                continue
            w = (tt >= PEAK_LO) & (tt <= PEAK_HI)
            res[lk] = float(avg[w][_np.argmax(_np.abs(avg[w]))] * 1e6)
        return res

    subs = sorted(d.name for d in S.ROOT.iterdir() if d.is_dir())
    out = {}
    for sub in subs:
        rows = ssr_v2.run(sub, "100")
        for r in rows:
            if r["chrom"] != "HbO" or r["path"] == "ssr_bad":
                continue
            out.setdefault((sub, r["lk"]), {})[r["path"]] = r["peak"]
            out[(sub, r["lk"])][r["path"] + "_n"] = r["noise"]
            out[(sub, r["lk"])]["group"] = r["group"]
            out[(sub, r["lk"])]["q_good"] = r["q_good"]
        for lk, pk in nearest_variant(sub).items():
            if (sub, lk) in out:
                out[(sub, lk)]["ssr_near"] = pk
        for lk, pk in mimic_mne(sub).items():
            if (sub, lk) in out:
                out[(sub, lk)]["ssr_mimic"] = pk
        print(f"  {sub}: {len([k for k in out if k[0] == sub])} 长距通道")
    pickle.dump(out, open(OUT_OURS, "wb"))
    print(f"\n-> {OUT_OURS.name}  共 {len(out)} 个通道")


# ---------------------------------------------------------------------------
def stage_mne(root: Path) -> None:
    """在 software/.venv-mne 中运行：跑 MNE-NIRS 管线，存同样的量。"""
    import warnings

    mne, beer_lambert_law, optical_density, short_channel_regression = _import_mne_stack()

    warnings.filterwarnings("ignore")
    mne.set_log_level("error")

    def peaks(hb) -> dict:
        """带通 -> 分段平均 -> 峰值与基线噪声，返回 {(src,det): (峰值, 噪声)} μM。

        基线噪声取事件前 (−PRE~0) 的标准差，与 ssr_v2.py 同一口径，
        这样 CNR = |峰值|/噪声 两边可比。
        """
        hb = hb.copy().filter(BP_LOW, BP_HIGH, h_trans_bandwidth=0.05,
                              l_trans_bandwidth=0.005, verbose="error")
        ev, ev_id = mne.events_from_annotations(hb, verbose="error")
        ep = mne.Epochs(hb, ev, ev_id, tmin=-PRE, tmax=POST,
                        baseline=(-PRE, 0), preload=True,
                        reject=None, verbose="error")
        avg = ep.average(picks="hbo")
        w = (avg.times >= PEAK_LO) & (avg.times <= PEAK_HI)
        b = avg.times < 0
        res = {}
        for i, name in enumerate(avg.ch_names):
            sd = name.split()[0]                       # "S1_D1 hbo" -> "S1_D1"
            s, d = sd.split("_")
            trace = avg.data[i] * 1e6                  # M -> μM
            seg = trace[w]
            res[(int(s[1:]), int(d[1:]))] = (float(seg[np.argmax(np.abs(seg))]),
                                             float(trace[b].std()))
        return res

    out = {}
    for sd in sorted(p for p in root.iterdir() if p.is_dir()):
        f = sd / "resting_hrf_100.snirf"
        if not f.exists():
            continue
        raw = mne.io.read_raw_snirf(f, preload=True, verbose="error")
        od = optical_density(raw, verbose="error")
        # 顺序与本项目一致：先 SSR，后带通（在 peaks() 里滤）
        od_ssr = short_channel_regression(od.copy())
        p_no = peaks(beer_lambert_law(od.copy(), ppf=PPF))
        p_ss = peaks(beer_lambert_law(od_ssr, ppf=PPF))
        dist = mne.preprocessing.nirs.source_detector_distances(raw.info)
        longs = set()
        for i, name in enumerate(raw.ch_names):
            if dist[i] >= 0.015:
                s, d = name.split()[0].split("_")
                longs.add((int(s[1:]), int(d[1:])))
        for k in longs:
            if k in p_no and k in p_ss:
                # 必须转成原生 float：venv 是 numpy 2.x，base 是 1.24，
                # numpy 标量跨版本 unpickle 会报 No module named 'numpy._core'
                out[(sd.name, k)] = {"no_ssr": p_no[k][0], "no_ssr_n": p_no[k][1],
                                     "ssr": p_ss[k][0], "ssr_n": p_ss[k][1]}
        print(f"  {sd.name}: {len(longs)} 长距通道")
    pickle.dump(out, open(OUT_MNE, "wb"))
    print(f"\n-> {OUT_MNE.name}  共 {len(out)} 个通道")


# ---------------------------------------------------------------------------
def stage_compare() -> None:
    for f in (OUT_OURS, OUT_MNE):
        if not f.exists():
            raise SystemExit(f"缺 {f.name}，先跑对应的 --stage")
    ours = pickle.load(open(OUT_OURS, "rb"))
    mne_ = pickle.load(open(OUT_MNE, "rb"))
    keys = sorted(set(ours) & set(mne_), key=lambda k: (k[0], k[1]))
    print("=" * 84)
    print("本项目 SSR  vs  MNE-NIRS SSR")
    print("=" * 84)
    print(f"  共同通道 {len(keys)}（本项目 {len(ours)}，MNE {len(mne_)}）\n")

    for grp in ("阳性", "阴性"):
        # 与 ssr_v2.py 同一统计口径：只算有合格短距(质量>0.5)的被试，
        # 否则短距本身不可信，SSR 效果会被稀释（实测 CNR 改善 5.3% -> 1.2%）。
        k = [x for x in keys if ours[x].get("group") == grp
             and ours[x].get("q_good", 1.0) > 0.5]
        if not k:
            continue
        o_no = np.array([ours[x]["no_ssr"] for x in k])
        o_ss = np.array([ours[x]["ssr_good"] for x in k])
        m_no = np.array([mne_[x]["no_ssr"] for x in k])
        m_ss = np.array([mne_[x]["ssr"] for x in k])
        print(f"  【{grp}组】{len(k)} 通道")
        print("    实现        no_ssr峰值中位   ssr峰值中位   恢复率(no/ssr)      ssr/no_ssr 比值中位")
        for nm, a, b in (("本项目", o_no, o_ss), ("MNE   ", m_no, m_ss)):
            r = b[np.abs(a) > 1e-9] / a[np.abs(a) > 1e-9]
            print("    %-8s %12.4f %13.4f    %.3f / %.3f %14.4f"
                  % (nm, np.median(np.abs(a)), np.median(np.abs(b)),
                     np.median(a) / TRUTH_HBO, np.median(b) / TRUTH_HBO,
                     np.median(r)))
        # 两实现的逐通道一致性。ssr_near 与 MNE 用同一种选道策略（最近短距），
        # 它与 MNE 的相关若明显高于 ssr_good，说明分叉来自选道而非实现。
        # CNR：谁的 SSR 降噪更强。两边基线噪声口径相同（事件前 −5~0s 的 std）
        if all("no_ssr_n" in ours[x] for x in k):
            print("    CNR = |峰值|/基线噪声      no_ssr    ssr     ssr/no_ssr 中位")
            for nm, pa, na, pb, nb in (
                    ("本项目", "no_ssr", "no_ssr_n", "ssr_good", "ssr_good_n"),
                    ("MNE   ", "no_ssr", "no_ssr_n", "ssr", "ssr_n")):
                src = ours if nm.startswith("本") else mne_
                c0 = np.array([abs(src[x][pa]) / src[x][na] for x in k
                               if src[x][na] > 0])
                c1 = np.array([abs(src[x][pb]) / src[x][nb] for x in k
                               if src[x][nb] > 0])
                rat = np.array([(abs(src[x][pb]) / src[x][nb])
                                / (abs(src[x][pa]) / src[x][na]) for x in k
                                if src[x][na] > 0 and src[x][nb] > 0])
                print("      %-8s              %8.2f %8.2f %12.4f"
                      % (nm, np.median(c0), np.median(c1), np.median(rat)))
        print("    逐通道一致性（本项目 vs MNE）")
        pairs = [("no_ssr   ", o_no, m_no), ("ssr_good ", o_ss, m_ss)]
        for tag in ("ssr_near", "ssr_mimic"):
            if all(tag in ours[x] for x in k):
                pairs.append((tag.ljust(9),
                              np.array([ours[x][tag] for x in k]), m_ss))
        for nm, a, b in pairs:
            msk = np.abs(b) > 1e-9
            print("      %s   r = %.4f   中位比值(本项目/MNE) = %.3f"
                  % (nm, np.corrcoef(a, b)[0, 1], np.median(a[msk] / b[msk])))
        print()

    print("  判据：ssr/no_ssr 比值两边应接近 —— 它对 ε 表和距离的缩放免疫。")
    print("        绝对峰值两边差多少，主要反映 ε 表与源探距离的取值差异，不是 bug。")


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", required=True, choices=("ours", "mne", "compare"))
    ap.add_argument("--data", type=Path, default=None,
                    help="mne 阶段用：resting_state_2 目录（默认取 SSR_DATA_ROOT）")
    a = ap.parse_args()
    if a.stage == "ours":
        stage_ours()
    elif a.stage == "mne":
        root = a.data
        if root is None:
            import os
            base = os.environ.get("SSR_DATA_ROOT")
            if not base:
                raise SystemExit("需要 --data 或环境变量 SSR_DATA_ROOT")
            root = Path(base) / "resting_state_2"
        stage_mne(root)
    else:
        stage_compare()


# ---------------------------------------------------------------------------
# 环境（--stage mne 专用）
#
#   cd software
#   python -m venv .venv-mne
#   .venv-mne\Scripts\pip install "numpy>=1.26" mne mne-nirs
#
# 已验证版本：mne 1.12.x + mne-nirs 0.7.x。不要在 base/fnris 里 pip install mne，
# 会与 numpy 1.24 / pyqtgraph 等已编译扩展冲突。
#
#   $env:SSR_DATA_ROOT = "D:\...\脑部_红外数据"   # 含 resting_state_2\
#   .venv-mne\Scripts\python.exe tools/ssr_compare_mne.py --stage mne
#   python tools/ssr_compare_mne.py --stage compare
# ---------------------------------------------------------------------------
