# -*- coding: utf-8 -*-
"""在线（因果）与离线（零相位）的波形一致性 —— 多录音批量核验。

在线的 FIR 结构下，t 时刻的读数等于离线 t−延迟 时刻的读数（波形不失真，
只是晚了一个已知常数）。因此比较时按该固定延迟对齐；IIR 结构没有单一延迟
可对齐（群延迟随频率变），故其数值只能作参考。

⚠️ 离线不是金标准，它自身对合成真值也偏高约 12.5%。这里量的是【因果性的
   代价】，前提是 tools/online_replay.py 的 A 类不变量已通过。

用法
    python tools/online_vs_offline_check.py            # 全部可用录音
    python tools/online_vs_offline_check.py --limit 6  # 只跑前 6 段
"""

from __future__ import annotations

import argparse
import glob
import sys
from pathlib import Path

import numpy as np
import pandas as pd

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))
sys.path.insert(0, str(_HERE))
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

import warnings
warnings.filterwarnings("ignore")

from fnirs_pipeline.mbll import calculate_concentration_series          # noqa: E402
from fnirs_pipeline.preprocessing import prepare_interleaved_dataframe  # noqa: E402
from online_android.online_config import OnlineSettings                 # noqa: E402
from online_replay import reconstruct                                   # noqa: E402

REQUIRED = {"Time (s)", "DetectorId", "Wavelength", "Value"}
_RESULT = _HERE.parent / "result_table"


def usable(min_dur: float):
    out = []
    for f in sorted(glob.glob(str(_RESULT / "*" / "all_groups.csv"))):
        try:
            h = pd.read_csv(f, nrows=0)
        except Exception:
            continue
        if not REQUIRED <= {c.strip() for c in h.columns}:
            continue
        d = pd.read_csv(f)
        d.columns = [c.strip() for c in d.columns]
        if len(d) < 100:
            continue
        dur = float(d["Time (s)"].max() - d["Time (s)"].min())
        if dur > min_dur:
            out.append((dur, Path(f), d))
    out.sort(key=lambda r: -r[0])
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--min-dur", type=float, default=180.0)
    ap.add_argument("--kind", choices=["fir", "iir"], default=None,
                    help="覆盖 OnlineSettings 的滤波结构，用于改动前后对比")
    ap.add_argument("--taps", type=int, default=None)
    a = ap.parse_args()

    import dataclasses
    import online_replay
    S = OnlineSettings()
    if a.kind or a.taps:
        S = dataclasses.replace(S,
                                causal_filter_kind=a.kind or S.causal_filter_kind,
                                causal_fir_taps=a.taps or S.causal_fir_taps)
        online_replay.make_processor = (lambda _s=S: __import__(
            "online_android.causal_processor", fromlist=["IncrementalCausalProcessor"]
        ).IncrementalCausalProcessor(
            _s,
            prepare_interleaved=prepare_interleaved_dataframe,
            calculate_series=calculate_concentration_series))
    kind = getattr(S, "causal_filter_kind", "iir")
    taps = int(getattr(S, "causal_fir_taps", 0))
    print("=" * 84)
    print("在线 vs 离线 波形一致性（多录音）")
    print("=" * 84)
    print("  在线滤波结构：%s%s" % (kind, ("，%d 抽头" % taps) if kind == "fir" else ""))

    recs = usable(a.min_dur)
    if a.limit:
        recs = recs[: a.limit]
    print("  录音：时长 > %.0f s 的共 %d 段\n" % (a.min_dur, len(recs)))

    print("  录音                    时长    帧数   延迟   相关(对齐后)  相关(未对齐)  幅度比")
    print("  " + "-" * 80)
    rows = []
    for dur, f, d in recs:
        inter = prepare_interleaved_dataframe(d, start_at_zero=False)
        res = calculate_concentration_series(inter)
        if res is None:
            continue
        t = np.asarray(res[0], float)
        off = np.asarray(res[1], float)
        fs = 1.0 / float(np.median(np.diff(t)))
        on = reconstruct(d, "time:1.0").get("hbo")
        if on is None or len(on) < 200:
            continue
        n = min(len(off), len(on))
        off, on = off[:n], on[:n]
        g = int(round((taps - 1) / 2)) if kind == "fir" else 0

        raw = float(np.corrcoef(off, on)[0, 1])
        # 起始要去掉滤波器自身的建立段：FIR 需 taps 个样本才完全稳定，
        # 而热身只有 60 s < 141 s，冻结时滤波器尚未填满。
        skip = max(taps, int(S.causal_warmup_seconds)) if kind == "fir" else \
            int(S.causal_warmup_seconds)
        a1, b1 = off[: n - g], on[g:]
        m = min(len(a1), len(b1))
        a1, b1 = a1[skip:m], b1[skip:m]
        if len(a1) > 150 and a1.std() > 0:
            al = float(np.corrcoef(a1, b1)[0, 1])
            amp = float(b1.std() / a1.std())
        else:
            al = amp = np.nan
        rows.append((al, raw, amp))
        print("  %-22s %6.0fs %6d %5ds   %s     %+8.4f    %s"
              % (f.parent.name, dur, n, g,
                 ("%+8.4f" % al) if np.isfinite(al) else "  长度不足", raw,
                 ("%6.2f" % amp) if np.isfinite(amp) else "   —"))

    if rows:
        al = np.array([r[0] for r in rows]); raw = np.array([r[1] for r in rows])
        amp = np.array([r[2] for r in rows])
        ok = np.isfinite(al)
        print("  " + "-" * 80)
        print("  %d 段汇总   对齐后相关 中位 %.4f  最小 %.4f  最大 %.4f"
              % (ok.sum(), np.median(al[ok]), al[ok].min(), al[ok].max()))
        print("             未对齐相关 中位 %.4f     幅度比 中位 %.2f"
              % (np.median(raw[np.isfinite(raw)]), np.median(amp[np.isfinite(amp)])))
        print()
        print("  解读：对齐后相关反映【波形保真度】；未对齐相关反映【客户端直接看到的差异】，")
        print("        两者之差就是那个固定延迟造成的。FIR 下延迟是常数，可在界面标注；")
        print("        IIR 下群延迟随频率变，没有单一延迟可扣，未对齐相关就是全部真相。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
