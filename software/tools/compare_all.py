"""多被试：原始光强走本项目管线 vs 数据集发布的处理结果。"""
import pickle
import sys
from pathlib import Path

import numpy as np
from scipy.io import loadmat

sys.path.insert(0, '.')
sys.path.insert(0, r"D:\WORK_space\Code_WS\code_localized\fNIRS-main\software")
sys.stdout.reconfigure(encoding="utf-8")

import match_epo as M

HERE = M.HERE


def is_fnirs_epo(p):
    try:
        e = loadmat(str(p), struct_as_record=False, squeeze_me=True)["epo"]
        return hasattr(e, "oxy")
    except Exception:
        return False


def col_stats(a, b):
    """逐通道 (r, k)：a ≈ k·b。"""
    n = min(a.shape[0], b.shape[0])
    r, k = [], []
    for j in range(min(a.shape[1], b.shape[1])):
        u = a[:n, j] - a[:n, j].mean()
        v = b[:n, j] - b[:n, j].mean()
        if u.std() <= 0 or v.std() <= 0:
            continue
        r.append(float(u @ v / (n * u.std() * v.std())))
        k.append(float((u @ v) / (v @ v)))
    return np.array(r), np.array(k)


if __name__ == "__main__":
    raws = sorted(HERE.glob("hefmi/s*.nirs")) + sorted(HERE.glob("hefmi/t*.nirs")) \
        + [HERE / "hefmi" / "1.nirs"]
    print("预计算 %d 个原始文件的管线输出 ..." % len(raws))
    cache = {}
    for p in raws:
        hbo, hbr, s, fs, keys = M.pipeline(p)
        cache[p.stem] = (hbo, hbr, s, fs)
        print(" ", p.stem, end="", flush=True)
    print("\n")

    epos = sorted(HERE.glob("hefmi/e*.mat"), key=lambda x: int(x.stem[1:]))
    epos = [p for p in epos if is_fnirs_epo(p)]
    print("fNIRS epo 文件: %s\n" % [p.stem for p in epos])

    results = []
    for ep in epos:
        e = loadmat(str(ep), struct_as_record=False, squeeze_me=True)["epo"]
        EO = np.asarray(e.oxy.EO, float)
        best, bestr = None, -2.0
        for nm, (hbo, hbr, s, fs) in cache.items():
            seg = M.seg_at_marker(hbo, s, 31, fs, EO.shape[0])
            if seg is None:
                continue
            med, _ = M.corr_cols(seg, EO)
            if med > bestr:
                best, bestr = nm, med
        if bestr <= 0.9:
            print("  %-6s 未配对（最佳 %s r=%+.4f）" % (ep.stem, best, bestr))
            continue

        hbo, hbr, s, fs = cache[best]
        rec = {}
        for seg_name, mark in (("EO", 31), ("EC", 30)):
            ref_o = np.asarray(getattr(e.oxy, seg_name), float)
            ref_d = np.asarray(getattr(e.doxy, seg_name), float)
            so = M.seg_at_marker(hbo, s, mark, fs, ref_o.shape[0])
            sr = M.seg_at_marker(hbr, s, mark, fs, ref_o.shape[0])
            if so is None:
                continue
            rec[seg_name] = dict(
                hbo=col_stats(so, ref_o),
                hbt=col_stats(so + sr, ref_d),      # doxy 实为 HbT
                seg_mine=so, seg_ref=ref_o,
                seg_mine_t=so + sr, seg_ref_t=ref_d,
            )
        r, k = rec["EO"]["hbo"]
        print("  %-6s <-> %-6s   HbO: r中位 %+.5f (min %+.4f)  k中位 %.4f"
              % (ep.stem, best, np.median(r), r.min(), np.median(k)))
        results.append(dict(epo=ep.stem, raw=best, rec=rec))

    pickle.dump(results, open(HERE / "compare_results.pkl", "wb"))
    print("\n配对成功 %d 组 -> compare_results.pkl" % len(results))

    if results:
        allr = np.concatenate([x["rec"]["EO"]["hbo"][0] for x in results])
        allk = np.concatenate([x["rec"]["EO"]["hbo"][1] for x in results])
        art = np.concatenate([x["rec"]["EO"]["hbt"][0] for x in results])
        print("\n合计 %d 通道:" % len(allr))
        print("  HbO  r 中位 %.5f  最差 %.4f  >0.99 占 %.1f%%   k 中位 %.4f ± %.4f"
              % (np.median(allr), allr.min(), 100 * (allr > 0.99).mean(),
                 np.median(allk), allk.std()))
        print("  HbT  r 中位 %.5f  最差 %.4f" % (np.median(art), art.min()))
