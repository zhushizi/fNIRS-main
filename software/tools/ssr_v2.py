"""SSR 验证 v2：先按信号质量筛选短距通道，再比较三条路径。

数据与真值见 ssr_validate.py 的文件头。默认走 Dataset II（8 个短距通道）。

三条路径（其余完全相同）：
  no_ssr    不做短距回归
  ssr_good  用【质量最好】的短距通道回归
  ssr_bad   用【质量最差】的短距通道回归（阴性对照）

⭐ 阴性对照是本文件的要害：任何回归都会减小方差，光看「SSR 后噪声变小」
   什么都证明不了 —— 拿一路垃圾短距回归照样能让噪声变小。只有 ssr_good
   明显优于 ssr_bad，才说明捞回来的是空间特异性，不是普遍的方差缩减。
   ⚠️ Dataset I 只有 2 个短距通道，此时「最好 vs 最差」退化成二选一，
      对照强度大打折扣。故默认用 Dataset II。

两个指标（必须一起看）：
  CNR      = |峰值| / 事件前基线 std        —— 只反映信噪比
  recovery = 反演峰值 / 注入真值            —— 反映幅度有没有被削掉

  单看 CNR 会被「回归过度」骗过去：把真信号连同噪声一起削掉时，
  CNR 可能反而更好看，而 recovery 会掉下来。

通道分组（HRF 只注入随机一半的长距通道，不分组会被未注入的那一半稀释）：
  阴性  hrf_100 与 hrf_20 逐点相减恒为零 -> 确定未注入。见 ssr_validate.never_injected
  阳性  非阴性 且 no_ssr 路径恢复率 > 0.5
  存疑  其余（多为仅在低幅度档注入的通道）
  分组只用 no_ssr 路径判定，一次定死、三条路径共用，故三路对比是公平的。

---------------------------------------------------------------------------
验证结果（2026-08-31，Dataset II，14 被试，hrf_100 档，HbO）
---------------------------------------------------------------------------
分组规模：阳性 132 / 阴性 129 / 存疑 65 通道（统计口径：短距质量 > 0.5）

  组      路径        恢复率中位   CNR中位
  阳性    no_ssr         1.125      14.48
  阴性    no_ssr         0.146       6.54

  -> 注入真值 0.660 μM，阳性组反演出 0.743 μM，偏高 12.5%；阴性组 0.146。
     两组分离 7.7 倍。这同时验证了整条 MBLL 链路的【绝对幅度】，
     真值是数据集合成注入的，不是本项目推算的。

阳性组逐通道配对（n=132）：
  对比                 优于no_ssr占比   CNR比值中位   恢复率比值中位
  ssr_good vs no_ssr        65.2%         1.0526        1.0021
  ssr_bad  vs no_ssr        47.7%         0.9999        1.0000

  -> SSR 有效：好短距使 CNR 中位改善 5.3%，2/3 的通道受益；
     坏短距 47.7%（≈掷硬币）、CNR 比值 0.9999（≈完全无作用），
     阴性对照排除了「任何回归都会缩减方差」这一平凡解释。
     恢复率比值 1.0021 -> 信号未被削掉，排除回归过度。
  -> 阴性组恢复率 0.146 -> 0.148，SSR 未凭空造出信号。

⚠️ 边界：本结果是【事件锁定平均 29~38 个 trial】下的。本项目的产品形态是
   1 Hz 连续监护、无平均（config.OUTPUT_CHANNEL = "S1_D1_ssr"），
   届时 SSR 面对的是未经平均的生理噪声，收益可能显著大于 5.3%。
   5.3% 是下界，不是产品场景下的预期值。
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
    never = S.never_injected(sub, level)     # 确定未注入的长距通道

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
                truth = S.HRF_TRUE_UM[level][chrom]      # μM，带符号
                rows.append(dict(sub=sub, level=level, lk=lk, path=tag, chrom=chrom,
                                 peak=pk * 1e6, noise=base * 1e6,
                                 cnr=abs(pk) / base if base > 0 else np.nan,
                                 truth=truth, recovery=(pk * 1e6) / truth,
                                 never=lk in never,
                                 beta=beta, q_good=qs[sk_good], q_bad=qs[sk_bad],
                                 avg=avg * 1e6, tt=tt))

    # 阳性组判定：只看 no_ssr 路径的恢复率，一次定死，三条路径共用同一套分组。
    # 用被评估的那条路径去分组才会循环；用固定的第三方分组则不会。
    # 阈值 0.5：100% 档注入的恢复率≈1，仅在 20% 档注入的≈0.2，中间没有样本。
    pos = {r["lk"] for r in rows
           if r["path"] == "no_ssr" and r["chrom"] == "HbO"
           and not r["never"] and r["recovery"] > 0.5}
    for r in rows:
        r["group"] = ("阴性" if r["never"] else
                      "阳性" if r["lk"] in pos else "存疑")
    return rows


if __name__ == "__main__":
    if not S.ROOT.exists():
        raise SystemExit(
            f"找不到数据目录 {S.ROOT}\n"
            "从 https://www.nitrc.org/projects/luhmann20synhrf/ 下载 "
            "resting_state_2（Dataset II），解压到该路径。")
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
    truth = S.HRF_TRUE_UM["100"]["HbO"]
    print("  注入真值 HbO 峰值 = %+.3f μM\n" % truth)
    print("  路径        |峰值|中位  基线噪声中位   CNR中位   CNR均值   恢复率中位")
    for tag in ("no_ssr", "ssr_good", "ssr_bad"):
        v = [x for x in ok if x["path"] == tag]
        pk = np.median([abs(x["peak"]) for x in v])
        ns = np.median([x["noise"] for x in v])
        cn = np.array([x["cnr"] for x in v])
        rc = np.array([x["recovery"] for x in v])
        print("  %-10s %9.4f     %9.4f  %8.2f  %8.2f   %9.3f"
              % (tag, pk, ns, np.median(cn), cn.mean(), np.median(rc)))

    # 分组由文件互减（阴性）+ no_ssr 路径的恢复率（阳性）确定，与被比较的
    # 三条路径无关，故三路对比是公平的。
    print()
    for grp, note in (("阳性", "确有注入 —— 看 SSR 能不能把信号捞得更干净"),
                      ("阴性", "确定无注入 —— 看 SSR 会不会凭空造出信号"),
                      ("存疑", "仅在低幅度档注入或判据边缘，仅供参考")):
        g = [x for x in ok if x["group"] == grp]
        if not g:
            continue
        print("  【%s组】%s   共 %d 通道" % (grp, note, len(g) // 3))
        print("    路径        |峰值|中位   基线噪声中位   CNR中位   恢复率中位")
        for tag in ("no_ssr", "ssr_good", "ssr_bad"):
            v = [x for x in g if x["path"] == tag]
            if not v:
                continue
            print("    %-10s %9.4f      %9.4f  %8.2f  %9.3f"
                  % (tag, np.median([abs(x["peak"]) for x in v]),
                     np.median([x["noise"] for x in v]),
                     np.median([x["cnr"] for x in v]),
                     np.median([x["recovery"] for x in v])))
        print()
    # 逐通道配对：显式按 (被试, 通道键) 配对，不靠列表顺序隐式对齐。
    print("=" * 92)
    print("逐通道配对比较（只在阳性组，即确有注入的通道上）")
    print("=" * 92)
    pos = [x for x in ok if x["group"] == "阳性"]
    idx = {}
    for x in pos:
        idx.setdefault((x["sub"], x["lk"]), {})[x["path"]] = x
    paired = [v for v in idx.values() if len(v) == 3]
    print("  配对通道数 n = %d\n" % len(paired))
    print("  对比            优于 no_ssr 占比   CNR比值中位   恢复率比值中位")
    for tag in ("ssr_good", "ssr_bad"):
        rc = np.array([v[tag]["cnr"] / v["no_ssr"]["cnr"] for v in paired
                       if v["no_ssr"]["cnr"] > 0])
        rr = np.array([v[tag]["recovery"] / v["no_ssr"]["recovery"] for v in paired
                       if abs(v["no_ssr"]["recovery"]) > 1e-9])
        print("  %-14s %12.1f%%      %10.4f    %12.4f"
              % (tag + " vs no_ssr", 100 * np.mean(rc > 1),
                 np.median(rc), np.median(rr)))
    print("\n  判据：ssr_good 要明显优于 ssr_bad，才说明改善来自空间特异性")
    print("        而不是任何回归都会带来的方差缩减；同时恢复率不应下降。")
