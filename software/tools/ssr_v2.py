"""SSR 验证 v2：先按信号质量筛选短距通道，再比较三条路径。

三条路径（其余完全相同）：
  no_ssr    不做短距回归
  ssr_good  用【质量最好】的短距通道回归
  ssr_bad   用【质量最差】的短距通道回归（阴性对照）

真值：HRF 只注入长距通道，短距为纯生理噪声。
      事件锁定平均后，HRF 峰值应出现在 3~12 s 窗内。
指标：CNR = |峰值| / 事件前基线 std。SSR 有效则 CNR 应上升。
"""
import pickle
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, ".")
sys.stdout.reconfigure(encoding="utf-8")

import ssr_validate as S
from fnirs_pipeline.mbll import short_separation_regression, smart_bandpass
from fnirs_pipeline.mbll_core import hemoglobin_extinctions, intensities_to_od_changes

PPF, D_CM = 6.0, 3.0
PRE, POST = 5.0, 25.0


def short_quality(rec, sk, shorts):
    """生理带 (0.005~0.5 Hz) 功率占比，越高说明短距越可信。"""
    idx = [shorts[sk][w] for w in sorted(shorts[sk])]
    od = intensities_to_od_changes(rec["d"][:, idx].T)
    f = np.fft.rfftfreq(od.shape[1], 1 / rec["fs"])
    q = []
    for i in range(od.shape[0]):
        P = np.abs(np.fft.rfft(od[i] - od[i].mean())) ** 2
        q.append(P[(f >= 0.005) & (f < 0.5)].sum() / P.sum())
    return float(np.mean(q))


def run(sub, level="100"):
    p = S.ROOT / sub / f"resting_hrf_{level}.snirf"
    if not p.exists():
        return []
    rec = S.load(p)
    fs = rec["fs"]
    longs, shorts = S.channels(rec)
    if len(shorts) < 2:
        return []
    qs = {sk: short_quality(rec, sk, shorts) for sk in shorts}
    sk_good = max(qs, key=qs.get)
    sk_bad = min(qs, key=qs.get)

    ex = hemoglobin_extinctions([float(w) for w in rec["wl"]])
    Ainv = np.linalg.pinv(ex * (PPF * D_CM))
    ons = (rec["st"][:, 0] * fs).astype(int)

    sod = {}
    for sk in (sk_good, sk_bad):
        idx = [shorts[sk][w] for w in sorted(shorts[sk])]
        sod[sk] = intensities_to_od_changes(rec["d"][:, idx].T)   # 未滤波，同生产顺序

    rows = []
    for lk, wmap in longs.items():
        idx = [wmap[w] for w in sorted(wmap)]
        raw = rec["d"][:, idx].T
        if np.any(raw <= 0):
            continue
        lod = intensities_to_od_changes(raw)
        variants = {"no_ssr": (lod, np.nan)}
        for tag, sk in (("ssr_good", sk_good), ("ssr_bad", sk_bad)):
            corr, beta = short_separation_regression(lod, sod[sk])
            variants[tag] = (corr, float(np.mean(beta)))

        for tag, (od, beta) in variants.items():
            c = Ainv @ smart_bandpass(od, fs)
            for ci, chrom in ((0, "HbO"), (1, "HbR")):
                avg, tt = S.epoch(c[ci], ons, fs, PRE, POST)
                if avg is None:
                    continue
                base = avg[tt < 0].std()
                win = (tt >= 3) & (tt <= 12)
                pk = avg[win][np.argmax(np.abs(avg[win]))]
                rows.append(dict(sub=sub, level=level, lk=lk, path=tag, chrom=chrom,
                                 peak=pk * 1e6, noise=base * 1e6,
                                 cnr=abs(pk) / base if base > 0 else np.nan,
                                 beta=beta, q_good=qs[sk_good], q_bad=qs[sk_bad],
                                 avg=avg * 1e6, tt=tt))
    return rows


if __name__ == "__main__":
    subs = sorted([d.name for d in S.ROOT.iterdir() if d.is_dir()])
    allrows = []
    print("被试   合格短距质量  长距通道  β中位(good)  β中位(bad)")
    for sb in subs:
        r = run(sb)
        if not r:
            continue
        allrows += r
        hb = [x for x in r if x["chrom"] == "HbO"]
        bg = [x["beta"] for x in hb if x["path"] == "ssr_good"]
        bb = [x["beta"] for x in hb if x["path"] == "ssr_bad"]
        print("  %-8s %.3f / %.3f   %3d      %+8.4f    %+8.4f"
              % (sb, r[0]["q_good"], r[0]["q_bad"], len(bg),
                 np.median(bg), np.median(bb)))
    pickle.dump(allrows, open(Path(__file__).parent / "ssr_v2.pkl", "wb"))

    print()
    print("=" * 92)
    print("三条路径的 HRF 恢复（HbO，全部被试合并）")
    print("=" * 92)
    hb = [x for x in allrows if x["chrom"] == "HbO"]
    ok = [x for x in hb if x["q_good"] > 0.5]      # 只统计有合格短距的被试
    print("  统计口径：仅含【有合格短距(质量>0.5)】的被试，共 %d 通道\n" % (len(ok) // 3))
    print("  路径        |峰值| 中位   基线噪声中位    CNR 中位    CNR 均值")
    for tag in ("no_ssr", "ssr_good", "ssr_bad"):
        v = [x for x in ok if x["path"] == tag]
        pk = np.median([abs(x["peak"]) for x in v])
        ns = np.median([x["noise"] for x in v])
        cn = np.array([x["cnr"] for x in v])
        print("  %-10s %9.4f      %9.4f    %8.2f    %8.2f"
              % (tag, pk, ns, np.median(cn), cn.mean()))
    base = np.array([x["cnr"] for x in ok if x["path"] == "no_ssr"])
    good = np.array([x["cnr"] for x in ok if x["path"] == "ssr_good"])
    bad = np.array([x["cnr"] for x in ok if x["path"] == "ssr_bad"])
    n = min(len(base), len(good), len(bad))
    print()
    print("  逐通道配对比较（n=%d）:" % n)
    print("    ssr_good 优于 no_ssr 的通道占比: %.1f%%   CNR 比值中位 %.4f"
          % (100 * np.mean(good[:n] > base[:n]), np.median(good[:n] / base[:n])))
    print("    ssr_bad  优于 no_ssr 的通道占比: %.1f%%   CNR 比值中位 %.4f"
          % (100 * np.mean(bad[:n] > base[:n]), np.median(bad[:n] / base[:n])))
