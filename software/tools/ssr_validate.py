"""
用 luhmann20synhrf 数据集验证本项目的短距回归（SSR）。

数据来源
    von Lühmann A 等, "Open Access Multimodal fNIRS Resting State Dataset
    With and Without Synthetic Hemodynamic Responses",
    Front Neurosci 14:579353, 2020.
    下载 https://www.nitrc.org/projects/luhmann20synhrf/

    仪器 TechEn CW6，50 Hz，690/830 nm，SNIRF v1.0（HDF5）。
    每个被试 4 个文件：resting / resting_hrf_20 / _50 / _100。

    ┌──────────────┬──────┬──────┬───────┬──────────┐
    │              │ 长距 │ 短距 │ 时长  │ trial/人 │
    ├──────────────┼──────┼──────┼───────┼──────────┤
    │ Dataset I    │  26  │   2  │ 5 min │   ~15    │
    │ Dataset II   │  48  │   8  │10 min │  29~38   │
    └──────────────┴──────┴──────┴───────┴──────────┘
    Dataset II 实测：16 源 / 32 探测器，112 条 measurement，
    源探距离只有两种 —— 短距 8.0 mm、长距 30.5 mm，591 s，50 Hz。
    两个 subset 的 SNIRF 结构完全一致。

    ⚠️ 本文件早先写的是「长距 30 mm(26 通道)」，26 是 Dataset I 的数，
       而 ROOT 当时又指向 resting_state_1（只有 2 个短距）—— 描述与实
       际加载的 subset 对不上。短距 8 mm 这个数本身是对的。

真值
    合成 HRF 为 gamma 函数，峰值时间 6 s、总时长 16.5 s，在【光强域】注入，
    因此会完整走过本项目的 MBLL 链路。三档幅度 100/50/20 %，
    100 % 档对应 HbO +0.66 μM、HbR −0.23 μM。
    每 20 s 窗内随机 onset(0~3.5 s)，且【只注入随机一半的长距通道】
    —— 未注入的那一半是天然阴性对照。

方法：对每个 hrf 文件用【它自己的 stim 向量】做事件锁定平均，
      比较「做 SSR」与「不做 SSR」两条路径的 HRF 恢复效果。
      不依赖文件相减（三档事件位置不同，基底也不同）。
"""
import os
import sys
from pathlib import Path

import h5py
import numpy as np

sys.path.insert(0, r"D:\WORK_space\Code_WS\code_localized\fNIRS-main\software")
sys.stdout.reconfigure(encoding="utf-8")

from fnirs_pipeline.mbll import short_separation_regression, smart_bandpass
from fnirs_pipeline.mbll_core import hemoglobin_extinctions, intensities_to_od_changes

# 数据集有 2.3 GB，不必搬进仓库 —— 用环境变量指到实际解压位置即可：
#   PowerShell:  $env:SSR_DATA_ROOT = "D:\...\脑部_红外数据"
#   bash      :  export SSR_DATA_ROOT=/d/.../脑部_红外数据
# 该目录下应直接包含 resting_state_2/（和可选的 resting_state_1/）。
# 不设时回落到 tools/ssr/。
#
# 默认走 Dataset II：ssr_v2.py 的「最好短距 vs 最差短距」阴性对照需要
# 足够多的短距候选才有意义，Dataset I 只有 2 个，那个对照会退化成二选一。
_SSR_BASE = Path(os.environ.get("SSR_DATA_ROOT") or (Path(__file__).parent / "ssr"))
ROOT = _SSR_BASE / "resting_state_2"
if not ROOT.exists() and (_SSR_BASE / "resting_state_1").exists():
    ROOT = _SSR_BASE / "resting_state_1"    # 只下了 Dataset I 时回落
    # 两个都没下时 ROOT 保持指向 resting_state_2，好让报错指向该下的那个

# 合成 HRF 的真值峰值 (μM)，用于算幅度恢复率。
# 100 % 档取自 von Lühmann 2020 正文，其余两档按标称比例缩放。
HRF_TRUE_UM = {"100": {"HbO": 0.660, "HbR": -0.230},
               "50":  {"HbO": 0.330, "HbR": -0.115},
               "20":  {"HbO": 0.132, "HbR": -0.046}}

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
        # 长短距分界写死为 15（见 SHORT_MM/LONG_MM），前提是坐标单位为 mm。
        # 换成 cm 的文件会让全部通道被判成短距且不报错，故显式核对一次。
        unit = None
        try:
            raw_u = np.array(n["metaDataTags"]["LengthUnit"]).ravel()[0]
            unit = raw_u.decode() if isinstance(raw_u, bytes) else str(raw_u)
        except (KeyError, IndexError):
            pass
        if unit is not None and unit.strip().lower() not in ("mm", "millimeter"):
            raise ValueError(f"{path} 的 LengthUnit 是 {unit!r}，"
                             f"但长短距判据按 mm 写死，需换算后再用")
    sep = np.linalg.norm(sp[ml[:, 0] - 1] - dp[ml[:, 1] - 1], axis=1)
    return dict(d=d, t=t, ml=ml, wl=wl, st=st, sep=sep,
                fs=1.0 / float(np.median(np.diff(t))), sp=sp, dp=dp)


def never_injected(sub, level="100", other="20"):
    """定出【确定没有被注入 HRF】的长距通道，返回通道键集合。

    三个 hrf_* 文件共享同一段静息底数据（实测：同被试跨档的短距通道相关
    为 1.0，相减后逐点恒为零），因此 hrf_A − hrf_B 会把底数据精确抵消，
    只剩两次注入。于是：

        差恒为零  ->  A、B 两档都没注入这个通道  -> 确定阴性
        差非零    ->  至少一档注入了

    ⚠️ 只能定出「确定阴性」，定不出「确定阳性」。因为三档的注入 onset
       只在各自 20 s 窗内抖动 0~3.5 s，事件窗大量重叠，锁 A 的 onset 也
       能捞到 B 的注入（实测幅度比仅 1.1），无法据此分开 A、B 的注入集。
       阳性组需另行判定，见 ssr_v2.py。

    ⚠️ 不能用 resting.snirf 来做这件事：实测部分被试（如 Subj86）的
       resting.snirf 与 hrf_* 相关仅 0.92，是另一段录音；Subj100 则为
       0.9997。逐被试不一致，只有 hrf_* 之间才同底。
    """
    a = load(ROOT / sub / f"resting_hrf_{level}.snirf")
    b = load(ROOT / sub / f"resting_hrf_{other}.snirf")
    if a["d"].shape != b["d"].shape:
        return set()
    D = a["d"] - b["d"]
    longs, _ = channels(a)
    return {k for k, v in longs.items()
            if not np.any(D[:, [v[w] for w in sorted(v)]])}


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
