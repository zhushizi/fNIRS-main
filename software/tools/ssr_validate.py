"""
用 luhmann20synhrf 数据集验证本项目的短距回归（SSR）。

数据：NITRC luhmann20synhrf / resting_state_1
      14 被试 × {resting, hrf_20, hrf_50, hrf_100}
      50 Hz · 690/830 nm · 短距 8 mm(8 通道) · 长距 30 mm(26 通道)
      HRF 只注入长距，短距为纯生理噪声；部分长距未注入 -> 天然阴性对照

方法：对每个 hrf 文件用【它自己的 stim 向量】做事件锁定平均，
      比较「做 SSR」与「不做 SSR」两条路径的 HRF 恢复效果。
      不依赖文件相减（三档事件位置不同，基底也不同）。
"""
import sys
from pathlib import Path

import h5py
import numpy as np

sys.path.insert(0, r"D:\WORK_space\Code_WS\code_localized\fNIRS-main\software")
sys.stdout.reconfigure(encoding="utf-8")

from fnirs_pipeline.mbll import short_separation_regression, smart_bandpass
from fnirs_pipeline.mbll_core import hemoglobin_extinctions, intensities_to_od_changes

ROOT = Path(__file__).parent / "ssr" / "resting_state_1"
PRE, POST = 5.0, 25.0          # 事件锁定窗口 (s)
PPF, D_LONG = 6.0, 3.0         # DPF 与长距源探距离 (cm)
SHORT_MM, LONG_MM = 15.0, 15.0  # <15mm 判为短距


def load(path):
    with h5py.File(path, "r") as f:
        n = f["nirs"]
        dd = n["data1"]
        d = np.array(dd["dataTimeSeries"])
        t = np.array(dd["time"]).ravel()
        mls = sorted([k for k in dd if k.startswith("measurementList")],
                     key=lambda s: int(s[15:]))
        ml = np.array([[int(np.array(dd[k]["sourceIndex"]).ravel()[0]),
                        int(np.array(dd[k]["detectorIndex"]).ravel()[0]),
                        int(np.array(dd[k]["wavelengthIndex"]).ravel()[0])] for k in mls])
        sp = np.array(n["probe"]["sourcePos2D"])
        dp = np.array(n["probe"]["detectorPos2D"])
        wl = np.array(n["probe"]["wavelengths"]).ravel()
        st = None
        for k in n:
            if k.startswith("stim") and "data" in n[k]:
                st = np.array(n[k]["data"])
                break
    sep = np.linalg.norm(sp[ml[:, 0] - 1] - dp[ml[:, 1] - 1], axis=1)
    return dict(d=d, t=t, ml=ml, wl=wl, st=st, sep=sep,
                fs=1.0 / float(np.median(np.diff(t))), sp=sp, dp=dp)


def channels(rec):
    """{(src,det): {wlIdx: measIdx}}，并按距离分成长/短。"""
    g = {}
    for i, r in enumerate(rec["ml"]):
        g.setdefault((int(r[0]), int(r[1])), {})[int(r[2])] = i
    nwl = len(rec["wl"])
    g = {k: v for k, v in g.items() if len(v) == nwl}
    longs = {k: v for k, v in g.items() if rec["sep"][list(v.values())[0]] >= LONG_MM}
    shorts = {k: v for k, v in g.items() if rec["sep"][list(v.values())[0]] < SHORT_MM}
    return longs, shorts


def nearest_short(rec, lkey, shorts):
    """给长距通道找几何上最近的短距通道（按源位置）。"""
    sp, dp = rec["sp"], rec["dp"]
    lc = (sp[lkey[0] - 1] + dp[lkey[1] - 1]) / 2.0
    best, bd = None, 1e9
    for sk in shorts:
        sc = (sp[sk[0] - 1] + dp[sk[1] - 1]) / 2.0
        dist = np.linalg.norm(lc - sc)
        if dist < bd:
            best, bd = sk, dist
    return best, bd


def epoch(x, onsets, fs, pre, post):
    a, b = int(pre * fs), int(post * fs)
    segs = [x[o - a:o + b] for o in onsets if o - a >= 0 and o + b <= len(x)]
    if not segs:
        return None, None
    s = np.stack(segs)
    s = s - s[:, :a].mean(axis=1, keepdims=True)      # 事件前基线归零
    return s.mean(0), (np.arange(a + b) - a) / fs


def run_subject(sub, level):
    p = ROOT / sub / f"resting_hrf_{level}.snirf"
    if not p.exists():
        return None
    rec = load(p)
    fs = rec["fs"]
    longs, shorts = channels(rec)
    if not shorts:
        return None
    ex = hemoglobin_extinctions([float(w) for w in rec["wl"]])
    Ainv = np.linalg.pinv(ex * (PPF * D_LONG))
    ons = (rec["st"][:, 0] * fs).astype(int)

    # 短距 OD —— 【不滤波】，与生产代码 mbll.py:191 的顺序一致：
    # 先在原始 ΔOD 上做 SSR，再带通。先滤会把系统性生理成分去掉，
    # 短距就没有可回归的共模分量了。
    sod = {}
    for sk, wmap in shorts.items():
        idx = [wmap[k] for k in sorted(wmap)]
        raw = rec["d"][:, idx].T
        sod[sk] = intensities_to_od_changes(raw)

    rows = []
    for lk, wmap in longs.items():
        idx = [wmap[k] for k in sorted(wmap)]
        raw = rec["d"][:, idx].T
        if np.any(raw <= 0):
            continue
        lod = intensities_to_od_changes(raw)                 # 未滤波
        sk, gdist = nearest_short(rec, lk, shorts)
        corr, beta = short_separation_regression(lod, sod[sk])   # 先 SSR

        for tag, od in (("no_ssr", lod), ("ssr", corr)):
            c = Ainv @ smart_bandpass(od, fs)    # 后带通，再 MBLL
            for ci, chrom in ((0, "HbO"), (1, "HbR")):
                avg, tt = epoch(c[ci], ons, fs, PRE, POST)
                if avg is None:
                    continue
                base = avg[tt < 0]
                win = (tt >= 3) & (tt <= 12)     # HRF 峰值窗
                pk = avg[win][np.argmax(np.abs(avg[win]))]
                rows.append(dict(sub=sub, level=level, lk=lk, sk=sk, gdist=gdist,
                                 path=tag, chrom=chrom,
                                 peak=pk * 1e6, noise=base.std() * 1e6,
                                 beta=float(np.mean(beta))))
    return rows


if __name__ == "__main__":
    subs = sorted([d.name for d in ROOT.iterdir() if d.is_dir()]) if ROOT.exists() else []
    print("被试:", subs)
    out = []
    for s in subs:
        for lv in ("100", "50", "20"):
            r = run_subject(s, lv)
            if r:
                out.append(r)
                print(f"  {s} hrf_{lv}: {len(r)//4} 长距通道")
    flat = [x for r in out for x in r]
    np.save(Path(__file__).parent / "ssr_rows.npy", np.array(flat, dtype=object))
    print(f"\n共 {len(flat)} 条记录 -> ssr_rows.npy")
