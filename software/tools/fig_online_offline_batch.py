# -*- coding: utf-8 -*-
"""五组实测录音：在线（因果 FIR）与离线（零相位）的 HbO / HbR 对比图。

每段两栏（HbO / HbR），每栏三条线：
    离线                  零相位滤波，用了未来数据，作参照
    在线（已对齐）         按 FIR 固定群延迟平移后 —— 反映【波形保真度】
    在线（实际时序）       客户端真正看到的曲线 —— 晚固定延迟秒

⚠️ 平移【只用于评估】。实时运行时补不了延迟，那要用未来数据。
   准确表述：在线 t 时刻的读数 = 离线 t−延迟 时刻的读数。

⚠️ 灰色区为滤波器建立段 max(抽头, 热身)。当前热身 60 s < 141 抽头，
   冻结基线时滤波器尚未填满，该段读数不可用于判定。

用法
    python tools/fig_online_offline_batch.py [--out 目录]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
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

SURF, INK, INK2, RULE = "#fcfcfb", "#0b0b0b", "#52514e", "#e3e2dd"
C_OFF, C_ON, C_RAW, C_WARM = "#2a78d6", "#eb6834", "#c9a227", "#c0392b"
plt.rcParams.update({
    "font.sans-serif": ["Microsoft YaHei", "SimHei", "DengXian", "sans-serif"],
    "axes.unicode_minus": False, "figure.facecolor": SURF, "axes.facecolor": SURF,
    "savefig.facecolor": SURF, "text.color": INK, "axes.labelcolor": INK2,
    "axes.edgecolor": RULE, "xtick.color": INK2, "ytick.color": INK2,
    "axes.titlesize": 10.5, "axes.labelsize": 9, "legend.fontsize": 8.5,
    "xtick.labelsize": 8, "ytick.labelsize": 8,
    "axes.spines.top": False, "axes.spines.right": False,
    "grid.color": RULE, "grid.linewidth": 0.7,
})

RECS = ["2026-07-08_09-23-45", "2026-07-01_16-15-19", "2026-07-03_14-59-48",
        "2026-06-30_14-51-30", "2026-06-30_14-06-02"]
_RESULT = _HERE.parent / "result_table"


def corr(a, b):
    m = np.isfinite(a) & np.isfinite(b)
    return float(np.corrcoef(a[m], b[m])[0, 1]) if m.sum() > 50 else np.nan


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=_HERE.parent / "验证图")
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)

    S = OnlineSettings()
    taps = int(getattr(S, "causal_fir_taps", 141))
    g = (taps - 1) // 2
    settle = max(taps, int(S.causal_warmup_seconds))
    U = 1e6

    fig = plt.figure(figsize=(15.2, 17.4))
    gs = fig.add_gridspec(len(RECS), 2, hspace=0.44, wspace=0.17,
                          left=0.055, right=0.985, top=0.935, bottom=0.038)
    fig.text(0.055, 0.972, "在线（因果 FIR）与离线（零相位）波形对比 —— 五组实测录音",
             fontsize=17, fontweight="bold")
    fig.text(0.055, 0.951,
             "在线带通：线性相位 FIR %d 抽头，群延迟恒为 %d s（在线 t 时刻 = 离线 t−%d s）；"
             "离线带通：四阶巴特沃斯 sosfiltfilt 零相位   灰区 = 滤波器建立段 %d s"
             % (taps, g, g, settle), fontsize=9.5, color=INK2)

    summary = []
    for r, name in enumerate(RECS):
        f = _RESULT / name / "all_groups.csv"
        d = pd.read_csv(f); d.columns = [c.strip() for c in d.columns]
        res = calculate_concentration_series(
            prepare_interleaved_dataframe(d, start_at_zero=False))
        on = reconstruct(d, "time:1.0")
        t = np.asarray(res[0], float); t = t - t[0]
        for c, (key, off_arr, lab) in enumerate((
                ("hbo", np.asarray(res[1], float), "ΔHbO"),
                ("hbr", np.asarray(res[2], float), "ΔHbR"))):
            ax = fig.add_subplot(gs[r, c])
            onl = on.get(key)
            n = min(len(off_arr), len(onl))
            off_v, on_v = off_arr[:n], onl[:n]
            tt = t[:n]
            sh = np.roll(on_v, -g)                      # 仅用于评估的对齐
            sh[-g:] = np.nan

            ax.axvspan(0, settle, color=C_WARM, alpha=.07)
            ax.plot(tt, off_v * U, color=C_OFF, lw=1.5, label="离线（零相位）")
            ax.plot(tt, sh * U, color=C_ON, lw=1.3, ls="--",
                    label="在线（按 %d s 延迟对齐）" % g)
            ax.plot(tt, on_v * U, color=C_RAW, lw=1.0, alpha=.75,
                    label="在线（实际时序）")

            sl = slice(settle, n - g)
            r_al = corr(off_v[sl], on_v[settle + g: n])
            r_raw = corr(off_v[sl], on_v[sl])
            summary.append((name, lab, r_al, r_raw))
            ax.set_title("%s   %s   对齐后 r=%s  实际 r=%s"
                         % (name, lab,
                            ("%.3f" % r_al) if np.isfinite(r_al) else "—",
                            ("%.3f" % r_raw) if np.isfinite(r_raw) else "—"),
                         loc="left")
            ax.set_xlabel("时间 (s)"); ax.set_ylabel("%s (μM)" % lab)
            ax.grid(alpha=.5)
            if r == 0 and c == 0:
                ax.legend(frameon=False, ncol=1, loc="upper right", fontsize=8)

    out = a.out / "在线离线对比_五组.png"
    fig.savefig(out, dpi=130)
    print("已写出 %s" % out)
    print()
    print("  录音                    量      对齐后 r    实际 r")
    for nm, lab, ra, rr in summary:
        print("  %-22s %-6s %8s   %8s"
              % (nm, lab, ("%.4f" % ra) if np.isfinite(ra) else "—",
                 ("%.4f" % rr) if np.isfinite(rr) else "—"))
    va = np.array([s[2] for s in summary if s[1] == "ΔHbO" and np.isfinite(s[2])])
    vb = np.array([s[2] for s in summary if s[1] == "ΔHbR" and np.isfinite(s[2])])
    print("\n  HbO 对齐后 r 中位 %.4f    HbR 对齐后 r 中位 %.4f"
          % (np.median(va), np.median(vb)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
