# -*- coding: utf-8 -*-
"""对一份原始归档跑 MBLL，输出 ΔHbO / ΔHbR。

    python -m mbll_decoder.run                      （最新一份归档）
    python -m mbll_decoder.run --file xxx.srsraw --rho 3.0 --csv out.csv

为什么入口是「归档」而不是实时流：
    0.01 Hz 高通需要 ≥300 秒记录（瞬态特征时间约 1/fc，需 3 倍衰减干净）。
    实时流要攒够这么久才能出第一个点，所以 MBLL 的主用场景是**事后分析**。
    SRS 才是实时那一路。

与 SRS 共用同一份 L2 原始记录和同一个插值扣暗器 —— 这正是架构里
「两条链唯一的共同输入」的含义：差异只能来自算法，不能来自输入。
"""

from __future__ import annotations

import argparse
import collections
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
if str(_HERE.parent) not in sys.path:
    sys.path.insert(0, str(_HERE.parent))
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from mbll_decoder import hardware as mh  # noqa: E402
from mbll_decoder.decoder import decode  # noqa: E402
from srs_cal import archive  # noqa: E402
from srs_cal.darksub import DarkSubtractor  # noqa: E402


def load(path: Path, rho: float, channel: int | None = None):
    """归档 → (t, I)。扣暗用与 SRS 同一个插值扣暗器。

    ⚠️ **必须限定一个采集通道**（2026-10-29）。多通道归档里两个通道的帧是
       交错的，而这里只有**一个** `DarkSubtractor`、`per` 又按 `cycle_id`
       建键 —— 不筛通道会同时犯两个错：
         ① 把另一通道的暗时隙扣到这一通道的信号上
            （实测 ch2 暗峰峰 786,447，ch1 只有 11,461，量级差 60 倍）；
         ② `cycle_id` 现在是**按通道各自编号**的，ch1 的第 5 周期与 ch2 的
            第 5 周期会落进同一个 `per[5]`，后写覆盖先写。
       `channel=None` 时取归档里**出现的第一个**通道，并在上层打印出来 ——
       不默认 1 号，因为归档完全可能只有 2 号（实测 20261029_162716 就是）。
    """
    wl = [float(w) for w in mh.WAVELENGTHS_NM]
    ds = DarkSubtractor()
    per: dict = collections.defaultdict(dict)
    frames = archive.replay(path)
    if channel is None:
        channel = next((int(f.channel) for f in frames), 1)
    frames = [f for f in frames if int(f.channel) == int(channel)]
    for f in frames:
        if f.cycle_id < 0:
            continue
        got = (ds.feed_dark(f.distance_cm, f.t_host, float(f.adc))
               if f.is_dark else
               ds.feed_signal(f.wavelength_nm, f.distance_cm,
                              f.t_host, float(f.adc)))
        for w, r, t, v in got:
            if abs(r - rho) < 1e-6 and w in wl:
                per[f.cycle_id][w] = (t, v)
    cyc = sorted(c for c in per if all(w in per[c] for w in wl))
    if not cyc:
        return np.array([]), np.zeros((len(wl), 0)), int(channel)
    ref = wl[-1]
    t = np.array([per[c][ref][0] for c in cyc])
    I = np.vstack([[per[c][w][1] for c in cyc] for w in wl])
    return t, I, int(channel)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="对原始归档跑 MBLL")
    ap.add_argument("--file", default=None, help="归档路径，默认取最新一份")
    ap.add_argument("--rho", type=float, default=mh.DISTANCE_CM,
                    help=f"用哪个距离，默认 {mh.DISTANCE_CM}")
    ap.add_argument("--age", type=float, default=mh.DEFAULT_AGE)
    ap.add_argument("--low", type=float, default=mh.BP_LOW_HZ)
    ap.add_argument("--high", type=float, default=mh.BP_HIGH_HZ)
    ap.add_argument("--csv", default=None, help="把时间序列写成 CSV")
    ap.add_argument("--channel", type=int, default=None,
                    help="采集通道（数据帧 byte10）。不给则取归档里出现的"
                         "第一个 —— 多通道归档必须指定，否则只算到其中一个")
    a = ap.parse_args(argv)

    if a.file:
        path = Path(a.file)
    else:
        files = archive.list_archives()
        if not files:
            print("没有归档文件。先用 online.py 采一段。")
            return 2
        path = max(files, key=lambda p: p.stat().st_size)
        print(f"（未指定 --file，取最大的一份：{path.name}）")
    if not path.exists():
        print(f"找不到 {path}")
        return 2

    seen = sorted({int(f.channel) for f in archive.replay(path)})
    t, I, ch = load(path, a.rho, a.channel)
    print("=" * 72)
    print(f"MBLL　{path.name}　ρ={a.rho:.0f}cm　"
          f"波长 {[int(w) for w in mh.WAVELENGTHS_NM]}　通道 {ch}")
    # 归档里有多个通道时必须说出来：这一次只算了其中一个，
    # 不说的话下面那串数字看起来就像"这份归档的全部结果"。
    if len(seen) > 1:
        print(f"  ⚠ 本归档含 {len(seen)} 个采集通道 {seen}，"
              f"这里只算了通道 {ch}。换 --channel 看别的。")
    print("=" * 72)
    if t.size == 0:
        print("  该距离上没有可用样本")
        return 1
    print(f"  样本 {t.size}　时长 {t[-1]-t[0]:.0f} s")

    r = decode(t, I, distance_cm=a.rho, age=a.age, band=(a.low, a.high))
    print(f"  带通 {a.low}~{a.high} Hz　年龄 {a.age:.0f}")
    print()
    if not r.ok:
        print("  ✗ 未出结果：")
        for s in r.flags:
            print(f"    · {s}")
        return 1

    print(f"  采样率 {r.fs:.3f} Hz　奈奎斯特 {r.fs/2:.3f} Hz　"
          f"条件数 {r.cond:.2f}　剔除 {r.n_dropped}/{r.n_in}")
    print()
    print(f"  {'量':<8}{'std µM':>10}{'峰峰 µM':>11}{'最小':>10}{'最大':>10}")
    print("  " + "-" * 50)
    for lbl, v in (("ΔHbO", r.d_hbo), ("ΔHbR", r.d_hbr)):
        print(f"  {lbl:<8}{v.std(ddof=1):10.4f}{np.ptp(v):11.4f}"
              f"{v.min():10.4f}{v.max():10.4f}")
    cc = float(np.corrcoef(r.d_hbo, r.d_hbr)[0, 1])
    print(f"\n  corr(ΔHbO, ΔHbR) = {cc:+.3f}")
    if mh.WAVELENGTHS_NM.size < 3:
        print("    ⚠️ 两波长时二者被约束在一条直线上，**噪声必然产生反相的一对**，"
              "\n       强负相关是数学必然，不是生理证据。")
    print()
    for s in r.flags:
        print(f"    · {s}")

    if a.csv:
        out = Path(a.csv)
        with out.open("w", encoding="utf-8", newline="") as fh:
            fh.write("t_s,d_hbo_uM,d_hbr_uM\n")
            for tt, ho, hr in zip(r.t, r.d_hbo, r.d_hbr):
                fh.write(f"{tt:.4f},{ho:.6f},{hr:.6f}\n")
        print(f"\n  已写出 {out}（{r.t.size} 行）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
