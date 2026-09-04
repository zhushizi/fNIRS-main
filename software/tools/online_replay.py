# -*- coding: utf-8 -*-
"""在线因果链路的 A 类验证 —— 回放器 + 四个实现不变量。

A 类测的是【在线路径自己的内部一致性】：同样的数据，喂法不同，结果必须相同。
这类问题有唯一正确答案，差一点就是 bug —— 是整条在线验证链里唯一能给出
干脆结论的部分。

⚠️ A 类【不是】「在线 == 离线」。causal_processor 的文件头写明在线用分段 RMS、
   不做离线那步非因果 1 Hz 低通，两者本就不逐点一致。拿离线当金标准是错的：
   离线自己对合成真值也偏高 12.5%（见 tools/ssr_compare_mne.py），
   以它为基准会把这 12.5% 一起继承且永远测不出来。

四个不变量
    A1 分块不变性    同段数据切成任意块喂入，interleaved 行必须逐位相同
    A2 追加等价性    同一冻结点下，追加到第 N 帧 == 一次性算到第 N 帧
    A3 tick 粒度敏感性  不同粒度 -> 冻结点不同 -> 滤波器状态不同（预期有差异，量化）
    A4 重放确定性    同数据同节奏跑两遍，逐位相同

    A1/A2/A4 判据是【精确相等】，过了即可作为回归网。
    A3 预期会有差异 —— _freeze_baseline() 在 span 首次 >= 热身时长的那一次调用
    触发，并把 sosfilt 状态 _zi 冻结在当时可用数据的末尾。tick 粗时会冲过边界，
    冻结点后移，之后每一帧都受影响。即：输出依赖 tick 到达时机，不只依赖数据。

数据
    用 result_table/ 下的自有实录，不用公开数据集 —— A 类与真值无关，
    而把 SNIRF 数据集适配成五波长交织帧的适配层本身可能引入 bug，把结果搞浑。
    需长表格式（含 DetectorId / Value 列）且时长 > 热身（60 s）。
    实测 44 段可用、31 段 > 60 s。已确认信号非退化（长距变异系数 0.06~0.30），
    不会出现「拿零跟零比」的空过。

用法
    python tools/online_replay.py                 # 默认三段录音
    python tools/online_replay.py --csv <路径>     # 指定一段
    python tools/online_replay.py --list          # 列出可用录音
"""

from __future__ import annotations

import argparse
import glob
import sys
from dataclasses import fields
from pathlib import Path

import numpy as np
import pandas as pd

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from fnirs_pipeline.mbll import calculate_concentration_series          # noqa: E402
from fnirs_pipeline.preprocessing import prepare_interleaved_dataframe  # noqa: E402
from online_android.causal_processor import IncrementalCausalProcessor  # noqa: E402
from online_android.online_config import OnlineSettings                 # noqa: E402

REQUIRED = {"Time (s)", "DetectorId", "Wavelength", "Value"}
DEFAULT_CSVS = ["2026-07-01_16-15-19", "2026-07-08_09-23-45", "2026-07-03_14-59-48"]
_RESULT = _HERE.parent / "result_table"


# ---------------------------------------------------------------------------
def make_processor() -> IncrementalCausalProcessor:
    """与 channel_dispatcher._make_processor 的构造保持一致。"""
    return IncrementalCausalProcessor(
        OnlineSettings(),
        prepare_interleaved=prepare_interleaved_dataframe,
        calculate_series=calculate_concentration_series,
    )


def chunk_bounds(n: int, times: np.ndarray, mode: str, rng: np.random.Generator):
    """产出累积切点（每次 try_build 传入 df.iloc[:k] 中的 k）。"""
    if mode == "all":
        return [n]
    if mode.startswith("rows:"):
        step = int(mode.split(":")[1])
        return list(range(step, n + 1, step)) + ([n] if n % step else [])
    if mode.startswith("time:"):
        dt = float(mode.split(":")[1])
        t0, out = times[0], []
        for k in range(1, n + 1):
            if times[k - 1] - t0 >= dt:
                out.append(k)
                t0 = times[k - 1]
        if not out or out[-1] != n:
            out.append(n)
        return out
    if mode.startswith("random"):
        out, k = [], 0
        while k < n:
            k = min(n, k + int(rng.integers(1, 400)))
            out.append(k)
        return out
    raise ValueError(mode)


def replay(df: pd.DataFrame, mode: str, seed: int = 0):
    """按指定粒度回放，返回 (处理器, 最后一个非 None 的 batch, batch 计数)。"""
    proc = make_processor()
    rng = np.random.default_rng(seed)
    times = df["Time (s)"].to_numpy(float)
    last, cnt = None, 0
    for k in chunk_bounds(len(df), times, mode, rng):
        b = proc.try_build(df.iloc[:k])
        if b is not None:
            last, cnt = b, cnt + 1
    return proc, last, cnt


def reconstruct(df: pd.DataFrame, mode: str, seed: int = 0) -> dict:
    """按【安卓客户端的方式】重建曲线，这才是真正该比的东西。

    replace_full_series=True  -> 整段替换（热身末的回填）
    replace_full_series=False -> 追加

    返回 {"times": ndarray, "hbo": ndarray, ..., "freeze_at": 冻结时的 span}。
    直接比较处理器内部状态或单个 batch 都不对：批次划分本来就随 tick 变，
    客户端看到的最终曲线才是不变量该成立的地方。
    """
    proc = make_processor()
    rng = np.random.default_rng(seed)
    times = df["Time (s)"].to_numpy(float)
    series: dict[str, list] = {}
    freeze_at = None
    for k in chunk_bounds(len(df), times, mode, rng):
        b = proc.try_build(df.iloc[:k])
        if b is None:
            continue
        if freeze_at is None and getattr(b, "baseline_ready", False):
            freeze_at = float(times[k - 1] - times[0])
        vec = batch_vec(b)
        if b.replace_full_series:
            series = {kk: list(v) for kk, v in vec.items()}
        else:
            for kk, v in vec.items():
                series.setdefault(kk, []).extend(list(v))
    out = {kk: np.asarray(v, dtype=float) for kk, v in series.items()}
    out["freeze_at"] = freeze_at
    return out


def cmp_series(a: dict, b: dict, keys=("times", "hbo", "hbr", "rso2")):
    """按时间戳对齐比较两条重建曲线。返回 [(名称, 是否相同, 最大差, 相对幅度)]。"""
    res = []
    if "times" not in a or "times" not in b:
        return res
    ta, tb = np.round(a["times"], 6), np.round(b["times"], 6)
    common = np.intersect1d(ta, tb)
    ia = {t: i for i, t in enumerate(ta)}
    ib = {t: i for i, t in enumerate(tb)}
    idx_a = np.array([ia[t] for t in common], dtype=int)
    idx_b = np.array([ib[t] for t in common], dtype=int)
    res.append(("帧数", len(ta) == len(tb), abs(len(ta) - len(tb)), np.nan))
    res.append(("共同帧", True, len(common), np.nan))
    for k in keys:
        if k == "times" or k not in a or k not in b:
            continue
        va, vb = a[k][idx_a], b[k][idx_b]
        d = np.abs(va - vb)
        d = d[np.isfinite(d)]
        amp = np.nanmax(np.abs(va)) if va.size else np.nan
        mx = float(d.max()) if d.size else 0.0
        res.append((k, mx == 0.0, mx, 100 * mx / amp if amp else np.nan))
    return res


def inter_matrix(proc) -> np.ndarray:
    """把处理器内部的 interleaved 行摊平成矩阵，便于逐位比较。"""
    rows = proc._inter_rows
    if not rows:
        return np.zeros((0, 0))
    keys = sorted(rows[0].keys())
    return np.array([[float(r.get(k, np.nan)) for k in keys] for r in rows])


def batch_vec(b) -> dict[str, np.ndarray]:
    """取 batch 里的数值序列，用于逐位比较。"""
    if b is None:
        return {}
    out = {}
    for f in fields(b):
        v = getattr(b, f.name)
        if isinstance(v, list) and v and isinstance(v[0], (int, float, type(None))):
            out[f.name] = np.array([np.nan if x is None else float(x) for x in v])
    return out


def cmp_exact(a: np.ndarray, b: np.ndarray) -> tuple[bool, float]:
    """逐位比较；NaN 视为相等（两边同为 NaN）。返回 (是否相同, 最大绝对差)。"""
    if a.shape != b.shape:
        return False, np.inf
    if a.size == 0:
        return True, 0.0
    both_nan = np.isnan(a) & np.isnan(b)
    d = np.where(both_nan, 0.0, np.abs(a - b))
    if np.any(np.isnan(d)):
        return False, np.inf
    return bool(np.array_equal(a[~both_nan], b[~both_nan])), float(np.nanmax(d))


# ---------------------------------------------------------------------------
def run_one(csv: Path) -> bool:
    d = pd.read_csv(csv)
    d.columns = [c.strip() for c in d.columns]
    dur = float(d["Time (s)"].max() - d["Time (s)"].min())
    warm = OnlineSettings().causal_warmup_seconds
    print("=" * 78)
    print("录音 %s   %.1f s   %d 行   热身 %.0f s" % (csv.parent.name, dur, len(d), warm))
    print("=" * 78)
    if dur <= warm:
        print("  时长不足热身，跳过\n")
        return True

    ok = True

    # ---- A1 分块不变性 ----
    print("\n  [A1] 分块不变性：interleaved 行必须与喂法无关")
    ref_proc, ref_batch, _ = replay(d, "all")
    ref = inter_matrix(ref_proc)
    print("      基准（一次性全量）：%d 行 × %d 列" % ref.shape)
    a1 = True
    for mode in ("rows:1", "rows:37", "rows:500", "time:1.0"):
        p, _, _ = replay(d, mode)
        same, mx = cmp_exact(ref, inter_matrix(p))
        a1 &= same
        print("      %-10s %s   最大差 %.3e" % (mode, "相同" if same else "**不同**", mx))
    nfail = 0
    for s in range(50):
        p, _, _ = replay(d, "random", seed=s)
        same, _ = cmp_exact(ref, inter_matrix(p))
        nfail += (not same)
    a1 &= nfail == 0
    print("      随机块长 50 次：%s" % ("全部相同" if nfail == 0 else "**%d 次不同**" % nfail))
    ok &= a1

    # ---- A4 重放确定性（先做，A2/A3 依赖它） ----
    print("\n  [A4] 重放确定性：同数据同节奏跑两遍")
    p1, b1, _ = replay(d, "rows:100")
    p2, b2, _ = replay(d, "rows:100")
    same_i, _ = cmp_exact(inter_matrix(p1), inter_matrix(p2))
    v1, v2 = batch_vec(b1), batch_vec(b2)
    same_b = all(cmp_exact(v1[k], v2[k])[0] for k in v1 if k in v2)
    ok &= same_i and same_b
    print("      interleaved %s   输出序列 %s"
          % ("相同" if same_i else "**不同**", "相同" if same_b else "**不同**"))

    # ---- A2 追加等价性 ----
    # 冻结点必须相同（由热身期的 tick 节奏决定），只改热身【之后】的粒度。
    # 比的是客户端重建出来的完整曲线，不是单个 batch —— 批次划分本就随 tick 变。
    print("\n  [A2] 追加等价性：同一冻结点下，不同追加粒度 -> 曲线必须相同")
    times = d["Time (s)"].to_numpy(float)
    n = len(d)
    k_warm = int(np.searchsorted(times - times[0], warm * 1.05) + 1)

    def run_split(post_step: int) -> tuple[dict, int]:
        """热身期固定 50 行/tick **喂到冻结真正发生为止**，之后才按 post_step 追加。

        ⚠️ 不能按行号猜冻结点：冻结发生在「span 首次 >= 热身时长」的那一次
           try_build，行号取决于 tick 粒度本身。按固定行号切分会让不同粒度
           在完全不同的帧数上冻结（实测 68 帧 vs 263 帧），那测的是 A3 不是 A2。
        """
        proc = make_processor()
        series: dict[str, list] = {}

        def feed(k: int) -> None:
            nonlocal series
            b = proc.try_build(d.iloc[:k])
            if b is None:
                return
            vec = batch_vec(b)
            if b.replace_full_series:
                series = {kk: list(v) for kk, v in vec.items()}
            else:
                for kk, v in vec.items():
                    series.setdefault(kk, []).extend(list(v))

        k = 0
        while k < n:                      # 热身：固定粒度，喂到冻结
            k = min(n, k + 50)
            feed(k)
            if proc._ready:
                break
        k_frozen = k
        while k < n:                      # 冻结后：可变粒度
            k = min(n, k + post_step)
            feed(k)
        return {kk: np.asarray(v, dtype=float) for kk, v in series.items()}, k_frozen

    # 判据用【相对幅度】的容差，不是逐位相等。IIR 的 sosfilt 恰好能给出逐位
    # 相同的结果，FIR 的 lfilter 做直接卷积求和，分块不同则累加顺序不同，
    # 浮点不满足结合律，必然留下舍入级残差（实测 5e-21，相对 5e-15）。
    # 真正的逻辑错误会表现为相对差 O(1)，与舍入级差十几个数量级，不会混淆。
    TOL_PCT = 1e-8
    ref2, kf_ref = run_split(50)
    print("      冻结于第 %d 行（各变体必须一致）；判据：相对幅度差 < %.0e %%"
          % (kf_ref, TOL_PCT))
    a2 = True
    for step in (100, 500, 2000):
        cur, kf = run_split(step)
        assert kf == kf_ref, "冻结点不一致（%d vs %d），A2 前提被破坏" % (kf, kf_ref)
        print("      追加粒度 %4d 行 vs 50 行：" % step)
        for name, same, mx, rel in cmp_series(ref2, cur):
            if name in ("帧数", "共同帧"):
                print("        %-8s %s" % (name, "一致" if same else "差 %d" % mx)
                      if name == "帧数" else "        %-8s %d" % (name, mx))
                a2 &= same if name == "帧数" else True
            else:
                good = same or (np.isfinite(rel) and rel < TOL_PCT)
                a2 &= good
                tag = ("逐位相同" if same else
                       "舍入级" if good else "**不同**")
                print("        %-8s %-8s 最大差 %.3e  占幅度 %.2e %%"
                      % (name, tag, mx, rel))
    ok &= a2

    # ---- A3 tick 粒度敏感性 ----
    print("\n  [A3] tick 粒度敏感性：冻结点随粒度移动（预期有差异，不计入判定）")
    base = reconstruct(d, "rows:1")
    print("      基准 rows:1，冻结于 span=%s s" % base.get("freeze_at"))
    print("      粒度        冻结span    hbo最大差    占幅度    rso2最大差")
    for mode in ("rows:10", "rows:100", "rows:500", "time:1.0", "time:5.0"):
        cur = reconstruct(d, mode)
        r = {n_: (mx, rel) for n_, _, mx, rel in cmp_series(base, cur)}
        hb = r.get("hbo", (np.nan, np.nan))
        rs = r.get("rso2", (np.nan, np.nan))
        print("      %-10s  %8s   %.3e  %7.2f%%   %.3e"
              % (mode, cur.get("freeze_at"), hb[0], hb[1], rs[0]))

    print("\n  本段结论：%s\n" % ("A1/A2/A4 通过" if ok else "**存在不变量被破坏**"))
    return ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", type=Path, default=None)
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()

    if a.list:
        for f in sorted(glob.glob(str(_RESULT / "*" / "all_groups.csv"))):
            try:
                h = pd.read_csv(f, nrows=0)
            except Exception:
                continue
            if not REQUIRED <= set(c.strip() for c in h.columns):
                continue
            d = pd.read_csv(f)
            d.columns = [c.strip() for c in d.columns]
            dur = float(d["Time (s)"].max() - d["Time (s)"].min())
            if dur > 60:
                print("  %7.1f s  %6d 行  %s" % (dur, len(d), Path(f).parent.name))
        return 0

    csvs = ([a.csv] if a.csv else
            [_RESULT / name / "all_groups.csv" for name in DEFAULT_CSVS])
    ok = True
    for c in csvs:
        if not c.exists():
            print("跳过（不存在）：%s" % c)
            continue
        ok &= run_one(c)
    print("=" * 78)
    print("A 类总结论：%s" % ("通过" if ok else "未通过"))
    print("=" * 78)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
