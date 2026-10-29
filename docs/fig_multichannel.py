# -*- coding: utf-8 -*-
"""多通道同时计算的验证图 —— 把真实双通道归档回放进 online 的生产链路。
产出 docs/验证图/多通道同时计算.png

⚠️ 走的是 `OnlineWindow._on_frames` 本身，不是另写一份算法。
   另写一份只能验"我对算法的理解"，验不了"代码到底算了什么"。

用法（在仓库根目录）：
    python docs/fig_multichannel.py
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "software"))

import matplotlib                                            # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt                              # noqa: E402
import numpy as np                                           # noqa: E402
from PyQt5 import QtWidgets                                  # noqa: E402

from srs_cal.archive import read_header, read_records         # noqa: E402
from srs_cal.online import (CAL_DIR, CHAN_COLOR, DEFAULT_CAL,  # noqa: E402
                            Calibration, OnlineWindow)
from srs_cal.protocol import FrameStream, frames_per_cycle    # noqa: E402

for fam in ("Microsoft YaHei", "SimHei", "DejaVu Sans"):
    matplotlib.rcParams["font.sans-serif"] = [fam]
    break
matplotlib.rcParams["axes.unicode_minus"] = False

D = ROOT / "software/srs_cal/原始归档"
# 时间顺序：只开 ch1 → 开 ch2（累加）→ 关 ch1（只剩 ch2）
FILES = [("20261029_162203_001.srsraw", "只开通道 1"),
         ("20261029_162520.srsraw", "通道 1+2 同采"),
         ("20261029_162716.srsraw", "关掉通道 1，只剩通道 2")]


def replay_window(name: str):
    """喂进 OnlineWindow，返回各通道流水线与链路统计。"""
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    win = OnlineWindow(Calibration.load(CAL_DIR / DEFAULT_CAL))
    win.chk_rec.setChecked(False)
    win.archive = None
    h = read_header(D / name)
    t0 = float(h.get("t0_perf", 0.0))
    st = FrameStream()
    last = t0
    for t_rel, raw in read_records(D / name):
        for f in st.feed(raw, t_host=t0 + t_rel):
            win._on_frames([f])
            last = f.t_host
    return win, st, last - t0


fig, axes = plt.subplots(3, 1, figsize=(11.5, 10.2))
rows = []
for ax, (name, tag) in zip(axes, FILES):
    win, st, dur = replay_window(name)
    chs = st.channels_seen
    for c in sorted(win.pipes):
        p = win.pipes[c]
        n = min(len(p.t_hist), len(p.toi_hist))
        if n == 0:
            # 出不了值也要显形 —— 空着比画一条假线诚实。
            # 「没有回传」和「有帧但算不出」是两回事，必须分开说：
            # 前者是设备/指令的问题，后者是信号本身的问题。
            r = p.last_res
            if p.n_frames == 0:
                why = "该通道没有回传"
            elif r is not None and r.flags:
                why = "；".join(r.flags)
            else:
                why = f"收到 {p.n_frames} 帧，窗口未攒满"
            ax.plot([], [], color=CHAN_COLOR[c], lw=2,
                    label=f"通道 {c}：无曲线（{why}）")
            continue
        t = np.fromiter(p.t_hist, float, len(p.t_hist))[:n]
        y = np.fromiter(p.toi_hist, float, len(p.toi_hist))[:n]
        ax.plot(t, y, color=CHAN_COLOR[c], lw=1.9,
                label=f"通道 {c}：{n} 点，中位 {np.median(y):.2f}，"
                      f"σ {np.std(y, ddof=1):.3f}")
    cps = " / ".join(f"ch{c} {st.cycles_of(c)/dur:.3f}" for c in chs)
    ax.set_title(f"{tag}　（{name[:15]}）　回传通道 {list(chs)}　"
                 f"周期率 {cps} 周期/s　短周期 {st.n_short_cycles}/{st.n_cycles}",
                 fontsize=10.5)
    ax.set_ylabel("TOI (%)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8.5, loc="lower right")
    rows.append((tag, name, chs, st.n_short_cycles, st.n_cycles, dur,
                 {c: len(win.pipes[c].toi_hist) for c in sorted(win.pipes)}))
    win.worker = None
    win.close()

axes[-1].set_xlabel("数据时刻 (s，以第一帧为 0)")
fig.suptitle("多通道同时计算 —— 真实归档回放进 online 的生产链路\n"
             "通道累加（开 2 号时 1 号仍在跑）；每通道各一条流水线，"
             "互不污染；四通道共用 1 号的标定，故非 1 号的绝对值不可信",
             fontsize=11.5)
fig.tight_layout(rect=(0, 0, 1, 0.945))
out = ROOT / "docs/验证图/多通道同时计算.png"
fig.savefig(out, dpi=135)
print(f"→ {out}")
print()
print(f"{'工况':<24}{'回传通道':>10}{'短周期':>10}{'时长s':>8}{'各通道TOI点数':>18}")
for tag, name, chs, ns, nc, dur, pts in rows:
    print(f"{tag:<24}{str(list(chs)):>10}{f'{ns}/{nc}':>10}{dur:>8.1f}"
          f"{str(pts):>18}")
print()
print(f"frames_per_cycle: 1通道={frames_per_cycle(1)} "
      f"2通道={frames_per_cycle(2)} 4通道={frames_per_cycle(4)}")
