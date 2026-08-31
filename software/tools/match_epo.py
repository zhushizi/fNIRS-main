"""把 .nirs 原始光强走本项目管线，与 _epo.mat 的发布结果配对并比对。

配对指纹：epo.oxy.EO = 从 marker 31 起的 60 s 静息段（661 × 90 通道）。
对每个 .nirs 用本项目管线算出同一段，逐通道相关；同一被试应接近 1。
"""
import sys
from pathlib import Path

import numpy as np
from scipy.io import loadmat

sys.path.insert(0, r"D:\WORK_space\Code_WS\code_localized\fNIRS-main\software")
sys.stdout.reconfigure(encoding="utf-8")

from config import BP_HIGH_HZ, BP_LOW_HZ
from fnirs_pipeline.mbll import smart_bandpass
from fnirs_pipeline.mbll_core import hemoglobin_extinctions, intensities_to_od_changes

HERE = Path(__file__).parent
PPF, D_CM = 6.0, 3.0            # HC_Calculate.m: ppf=[6 6 6]；SD 距离 3 cm
EO_MARK, EC_MARK = 31, 30       # MakeDatasetFromRaw.m: find(mrks==31) / ==30


def pipeline(path):
    """.nirs -> 本项目管线 -> (HbO, HbR) 各 (n_time, 90)，单位 M。"""
    m = loadmat(str(path), struct_as_record=False, squeeze_me=True)
    d = np.asarray(m["d"], float)
    t = np.asarray(m["t"], float).ravel()
    s = np.asarray(m["s"])
    sd = m["SD"]
    wls = [float(w) for w in np.asarray(sd.Lambda).ravel()]
    ml = np.asarray(sd.MeasList, int)
    fs = 1.0 / float(np.median(np.diff(t)))

    g = {}
    for i, r in enumerate(ml):
        g.setdefault((int(r[0]), int(r[1])), {})[int(r[3])] = i
    keys = [k for k in sorted(g) if len(g[k]) == len(wls)]

    Ainv = np.linalg.pinv(hemoglobin_extinctions(wls) * (PPF * D_CM))
    hbo = np.empty((d.shape[0], len(keys)))
    hbr = np.empty_like(hbo)
    for j, k in enumerate(keys):
        idx = [g[k][w] for w in sorted(g[k])]
        raw = d[:, idx].T
        od = intensities_to_od_changes(raw)          # 全程均值做基线，同 Homer2
        c = Ainv @ smart_bandpass(od, fs)            # 带通 0.01~0.1 -> MBLL
        hbo[:, j], hbr[:, j] = c[0], c[1]
    return hbo, hbr, s, fs, keys


def seg_at_marker(x, s, mark, fs, n_out):
    """取 marker 起 n_out 个样本。s 为 (n_time, n_mark) 事件矩阵。"""
    col = mark - 1
    if col >= s.shape[1]:
        return None
    on = np.flatnonzero(np.asarray(s)[:, col] > 0)
    if len(on) == 0:
        return None
    i0 = int(on[0])
    if i0 + n_out > len(x):
        return None
    return x[i0:i0 + n_out]


def corr_cols(a, b):
    """逐列相关，返回中位数与各列值。"""
    n = min(a.shape[0], b.shape[0])
    m = min(a.shape[1], b.shape[1])
    out = []
    for j in range(m):
        u = a[:n, j] - a[:n, j].mean()
        v = b[:n, j] - b[:n, j].mean()
        if u.std() <= 0 or v.std() <= 0:
            continue
        out.append(float(u @ v / (n * u.std() * v.std())))
    return (np.median(out) if out else np.nan), np.array(out)


if __name__ == "__main__":
    epo_path = HERE / "hefmi" / (sys.argv[1] if len(sys.argv) > 1 else "e0.mat")
    e = loadmat(str(epo_path), struct_as_record=False, squeeze_me=True)["epo"]
    EO = np.asarray(e.oxy.EO, float)
    print(f"参照 {epo_path.name}:  epo.oxy.EO {EO.shape}  std {EO.std():.3e} M")
    print()

    files = sorted(HERE.glob("hefmi/s*.nirs")) + sorted(HERE.glob("hefmi/t*.nirs")) \
        + [HERE / "hefmi" / "1.nirs"]
    print(f"{len(files)} 个原始文件，逐个比对 EO 段：")
    print("  文件      通道  相关中位数   >0.9的通道数")
    best = (None, -2, None)
    for p in files:
        try:
            hbo, hbr, s, fs, keys = pipeline(p)
        except Exception as ex:
            print(f"  {p.stem:8s} 读取失败 {ex}")
            continue
        seg = seg_at_marker(hbo, s, EO_MARK, fs, EO.shape[0])
        if seg is None:
            print(f"  {p.stem:8s} {len(keys):4d}  无 marker {EO_MARK} 或长度不足")
            continue
        med, allc = corr_cols(seg, EO)
        n9 = int((np.abs(allc) > 0.9).sum())
        print(f"  {p.stem:8s} {len(keys):4d}  {med:+9.5f}   {n9:3d}/{len(allc)}")
        if med > best[1]:
            best = (p.stem, med, allc)
    print()
    print(f"最佳匹配: {best[0]}  相关中位数 {best[1]:+.5f}")
    if best[1] > 0.9:
        print("  -> 配对成功，可做完整比对")
    else:
        print("  -> 未配对成功；需要换其他 _epo.mat 继续试")
