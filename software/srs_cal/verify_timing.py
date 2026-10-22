# -*- coding: utf-8 -*-
"""真机验证 L1 的 t_host / cycle_id —— 仿真自检覆盖不到的那一层。

    python -m srs_cal.verify_timing                 （在 software/ 下）
    python -m srs_cal.verify_timing --port COM16 --baud 115200 --seconds 30

仿真自检（selftest 第 9 项）证明的是**逻辑正确**：周期号递增、丢帧能检出、
跨 feed 边界不乱。它证明不了**真实时序**：

  · 串口缓冲与操作系统调度带来的 t_host 抖动有多大
  · 设备实际扫描速率是多少（此前只能从 take 时间戳倒推，跨度 37 倍）
  · 真实链路上丢帧/校验错的发生率
  · 一批 feed 里通常含几个周期（决定 t_host 的有效分辨率）

这四项只有连真机才能测。本脚本只读不写，不发任何指令，不改设备状态。
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from collections import Counter
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from srs_cal.acquire import DEFAULT_PORT                      # noqa: E402
from srs_cal.protocol import (FRAMES_PER_CYCLE, FrameStream)  # noqa: E402


def run(port: str, baud: int, seconds: float) -> int:
    try:
        import serial
    except ImportError:
        print("缺 pyserial：pip install pyserial")
        return 2
    try:
        ser = serial.Serial(port, baud, timeout=0.2)
    except Exception as e:                                    # noqa: BLE001
        print(f"打不开 {port} @ {baud}：{e}")
        return 2

    print("=" * 72)
    print(f"真机时序验证　{port} @ {baud}　采 {seconds:.0f} 秒")
    print("=" * 72)

    st = FrameStream()
    batch_sizes: list[int] = []       # 每次 feed 解出多少帧
    batch_times: list[float] = []     # 每次 feed 的 t_host
    cycle_first_t: dict[int, float] = {}
    t0 = time.perf_counter()
    try:
        while time.perf_counter() - t0 < seconds:
            # ⚠️ 不要用 ser.read(4096)：它会阻塞到**超时**才返回，
            #    时间戳因而晚于字节的真实到达时刻，凭空制造抖动。
            #    改为：阻塞等第一个字节 → 立刻打戳 → 再把缓冲里剩下的读空。
            head = ser.read(1)
            if not head:
                continue
            t_arrive = time.perf_counter()
            rest = ser.read(ser.in_waiting) if ser.in_waiting else b""
            frames = st.feed(head + rest, t_host=t_arrive)
            if not frames:
                continue
            batch_sizes.append(len(frames))
            batch_times.append(frames[0].t_host)
            for f in frames:
                cycle_first_t.setdefault(f.cycle_id, f.t_host)
    except KeyboardInterrupt:
        print("\n（手动中断）")
    finally:
        ser.close()

    elapsed = time.perf_counter() - t0
    if st.n_frames == 0:
        print("一帧都没收到 —— 查线、查端口、查设备是否上电")
        return 1

    ok = True

    def chk(name: str, good: bool, detail: str = "") -> None:
        nonlocal ok
        ok &= good
        print(f"  {'[通过]' if good else '[未通过]'} {name}"
              + (f" —— {detail}" if detail else ""))

    print(f"\n收到 {st.n_frames} 帧 / {st.n_cycles} 周期，耗时 {elapsed:.1f} s\n")

    # --- 1 链路质量（protocol 本来就在数，这里只是读出来）-------------------
    chk("链路无丢弃字节", st.n_dropped == 0, f"丢弃 {st.n_dropped} B")
    chk("链路无校验错", st.n_bad_checksum == 0, f"校验错 {st.n_bad_checksum} 帧")

    # --- 2 扫描速率：这是此前完全不知道的量 ---------------------------------
    cps = st.n_cycles / elapsed if elapsed > 0 else 0.0
    fps = st.n_frames / elapsed if elapsed > 0 else 0.0
    print(f"\n  实测扫描速率　{cps:.2f} 周期/秒　（{fps:.0f} 帧/秒）")
    ceiling = baud / 10 / 26 / FRAMES_PER_CYCLE
    print(f"  波特率上限　　{ceiling:.2f} 周期/秒　"
          f"（占用 {100 * cps / ceiling:.0f}%）")
    chk("扫描速率不超波特率上限", cps <= ceiling * 1.05,
        f"{cps:.2f} vs 上限 {ceiling:.2f}")

    # --- 3 周期完整性 -------------------------------------------------------
    chk("周期帧数齐全", st.n_short_cycles == 0,
        f"短周期 {st.n_short_cycles}/{st.n_cycles}")

    # --- 3.5 奈奎斯特：扫描速率决定了哪些生理成分根本测不到 -----------------
    #
    # ⚠️ 2026-10-22 补。首版漏了这一条，于是 0.66 周期/秒（奈奎斯特 0.33 Hz）
    #    也被判成「时序可用」—— 而该速率下心搏完全混叠。判据必须对着
    #    **要测什么**，不能只看链路干不干净。
    nyq = cps / 2.0
    print(f"\n  奈奎斯特频率　{nyq:.3f} Hz　（= 扫描速率 / 2）")
    print(f"  {'生理成分':<16}{'频率 Hz':>12}{'可测?':>8}")
    print("  " + "-" * 38)
    bands = [("去饱和事件", 0.01, 0.1), ("Mayer 波", 0.08, 0.12),
             ("呼吸", 0.20, 0.30), ("心搏", 0.80, 2.00)]
    worst = None
    for name, lo, hi in bands:
        okb = hi <= nyq * 0.8          # 留 20% 余量，别贴着奈奎斯特用
        print(f"  {name:<16}{lo:.2f}~{hi:.2f}{'✓' if okb else '✗ 混叠':>10}")
        if not okb and worst is None:
            worst = (name, hi)
    chk("去饱和事件带（0.01~0.1 Hz）可测 —— 脑血氧的核心需求",
        0.1 <= nyq * 0.8, f"需 ≥ {0.1 / 0.8 * 2:.2f} 周期/秒")
    chk("呼吸带（~0.3 Hz）可测", 0.3 <= nyq * 0.8,
        f"需 ≥ {0.3 / 0.8 * 2:.2f} 周期/秒")
    chk("心搏带（~2 Hz）可测 —— 贴合质量判据依赖它", 2.0 <= nyq * 0.8,
        f"需 ≥ {2.0 / 0.8 * 2:.2f} 周期/秒，当前 {cps:.2f}")

    # --- 4 t_host 的实际分辨率 ---------------------------------------------
    ids = sorted(cycle_first_t)
    gaps = [cycle_first_t[b] - cycle_first_t[a]
            for a, b in zip(ids, ids[1:]) if b == a + 1]
    # ⚠️ 2026-10-22：首版用 5~95 百分位算抖动，在本设备上**不可用** ——
    #    0.66 周期/秒时 30 秒只有约 19 个间隔，`quantiles(n=20)` 基本等于
    #    取极值，一个离群点就能让读数从 7% 跳到 29%（实测两次如此）。
    #    改用中位绝对偏差（MAD）衡量常态离散，离群单独计数。
    MIN_GAPS = 30
    if len(gaps) >= 8:
        med = statistics.median(gaps)
        mad = statistics.median([abs(g - med) for g in gaps])
        outl = [g for g in gaps if abs(g - med) > 0.20 * med]
        print(f"\n  周期间隔　中位 {med * 1e3:.2f} ms　"
              f"MAD {mad * 1e3:.2f} ms（{100 * mad / med:.1f}%）")
        print(f"  偏离中位 >20% 的间隔：{len(outl)}/{len(gaps)}"
              + (f"　最大偏离 {max(abs(g - med) for g in outl) * 1e3:.0f} ms"
                 if outl else ""))
        # t_host 按批打，批内时间跨度构成量化地板
        floor = ((statistics.median(batch_sizes) / fps) / med
                 if batch_sizes and fps > 0 and med > 0 else 0.0)
        print(f"  量化地板约 {floor * 100:.1f}%（t_host 按批打，不是按帧）")
        chk("常态周期间隔稳定（MAD < 5%）", mad / med < 0.05,
            f"MAD {100 * mad / med:.1f}%")
        chk("离群间隔 < 10%", len(outl) <= max(1, len(gaps) // 10),
            f"{len(outl)}/{len(gaps)}")
        if len(gaps) < MIN_GAPS:
            print(f"  ⚠️ 只有 {len(gaps)} 个间隔，离群计数不稳；"
                  f"要可信需 ≥{MIN_GAPS} 个，即 --seconds "
                  f"{MIN_GAPS / cps * 1.2:.0f} 以上")
    else:
        chk("周期数足够估抖动", False, f"只有 {len(gaps)} 个间隔，延长采集时间")

    # --- 5 一批 feed 含几个周期 → t_host 的有效粒度 -------------------------
    if batch_sizes:
        c = Counter(batch_sizes)
        med_b = statistics.median(batch_sizes)
        print(f"\n  每批 feed 解出帧数　中位 {med_b:.0f}　"
              f"（最常见 {c.most_common(3)}）")
        per_cycle = med_b / FRAMES_PER_CYCLE
        print(f"  即每批约 {per_cycle:.2f} 个周期　"
              f"→ 同批帧共用一个 t_host，批内时间误差约 "
              f"{per_cycle / cps * 1e3 if cps else 0:.1f} ms")
        chk("每批不超过 2 个周期（批内误差可接受）", per_cycle <= 2.0,
            f"{per_cycle:.2f} 周期/批；超了就减小 ser.read 的块大小")

    print("\n" + "=" * 72)
    print("结论：" + ("真机时序可用" if ok else "有未通过项，见上"))
    print("=" * 72)
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=60.0)
    a = ap.parse_args()
    return run(a.port, a.baud, a.seconds)


if __name__ == "__main__":
    raise SystemExit(main())
