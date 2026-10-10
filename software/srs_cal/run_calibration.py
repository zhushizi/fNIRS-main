# -*- coding: utf-8 -*-
"""吃实测数据，产出 G(λ) 与验收报告。

    python -m srs_cal.run_calibration --csv 实测.csv --baseline 基线.csv
    python -m srs_cal.run_calibration --csv 实测.csv --emit-template 模板.csv

CSV 格式（每块每波长一行）：

    block,wavelength_nm,I_3cm,I_4cm
    A2,670,123456,8901
    A2,770,...

  - block          体模外壳上的编号（A2/A3/A4/B2…D4）
  - wavelength_nm  器件波长，须是 670/770/810/850 之一（950 不参与）
  - I_3cm / I_4cm  该波长下两个探测器的原始计数，**已扣暗电平**或配合 --dark

基线 CSV（可选但强烈建议）给出无故障系统的块间离散，格式：

    wavelength_nm,dispersion
    670,0.0033

未给基线时离散判据不执行，程序会显式标出「本次结果不可直接放行」。
基线从哪来：同一台确认良好的系统重复标定 ≥5 轮，取各波长离散的中位数；
或先用 `python -m srs_cal.selftest` 第 6 项的仿真基线 0.0033 作临时值，
但那是仿真值，正式放行前必须换成实测基线。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from srs_cal import hardware as hw          # noqa: E402
from srs_cal import phantom                 # noqa: E402
from srs_cal.calibrate import calibrate_gain  # noqa: E402


def emit_template(path: Path) -> None:
    rows = ["block,wavelength_nm,I_3cm,I_4cm"]
    for code in phantom.BLOCK_CODES:
        for w in hw.WAVELENGTHS_NM:
            rows.append(f"{code},{w:.0f},,")
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")
    print(f"已写出空模板（{len(phantom.BLOCK_CODES)} 块 × {hw.WAVELENGTHS_NM.size} 波长"
          f" = {len(rows)-1} 行）：{path}")


def load_csv(path: Path) -> tuple[np.ndarray, tuple[str, ...]]:
    import csv

    wl_list = [float(w) for w in hw.WAVELENGTHS_NM]
    table: dict[str, dict[float, tuple[float, float]]] = {}
    with path.open(encoding="utf-8-sig", newline="") as f:
        for row in csv.DictReader(f):
            code = row["block"].strip()
            w = float(row["wavelength_nm"])
            if w not in wl_list:
                if w in [float(x) for x in hw.WL_REFERENCE_ONLY_NM]:
                    continue                      # 950 是参考通道，静默跳过
                raise ValueError(f"{code} 的波长 {w:.0f} 不在反演波长 {wl_list} 内")
            table.setdefault(code, {})[w] = (float(row["I_3cm"]), float(row["I_4cm"]))

    codes = [c for c in phantom.BLOCK_CODES if c in table]
    unknown = sorted(set(table) - set(phantom.BLOCK_CODES))
    if unknown:
        raise ValueError(f"CSV 里有未知块号 {unknown}，应为 {phantom.BLOCK_CODES}")
    if not codes:
        raise ValueError("CSV 里一块有效数据都没有")

    I = np.zeros((len(codes), len(wl_list), 2))
    for i, c in enumerate(codes):
        missing = [w for w in wl_list if w not in table[c]]
        if missing:
            raise ValueError(f"块 {c} 缺波长 {[f'{w:.0f}' for w in missing]}")
        for j, w in enumerate(wl_list):
            I[i, j] = table[c][w]
    if len(codes) < len(phantom.BLOCK_CODES):
        print(f"  ⚠️ 只有 {len(codes)}/{len(phantom.BLOCK_CODES)} 块："
              f"{','.join(codes)}。块数减少会直接抬高 G 的不确定度"
              f"（σ ∝ 1/√N，9 块是 12 块的 1.14 倍）")
    return I, tuple(codes)


def load_xlsx(path: Path) -> tuple[np.ndarray, tuple[str, ...]]:
    """读 `srs_cal.acquire` 产出的工作簿，取「汇总」表。

    同一块有多次 take 时，acquire 已在该表里取好各次均值，这里直接用。
    暗电平行（block=DARK）在此跳过 —— 它经 --dark 或单独扣除，不是体模块。
    """
    from openpyxl import load_workbook

    wb = load_workbook(path, data_only=True)
    if "汇总" not in wb.sheetnames:
        raise ValueError(f"{path} 里没有「汇总」工作表，可能不是 acquire 产出的文件")
    ws = wb["汇总"]
    head = [c.value for c in ws[1]]
    for need in ("block", "wavelength_nm", "I_3cm", "I_4cm"):
        if need not in head:
            raise ValueError(f"「汇总」表缺列 {need}")
    ib, iw, i3, i4 = (head.index(k) for k in
                      ("block", "wavelength_nm", "I_3cm", "I_4cm"))

    wl_list = [float(w) for w in hw.WAVELENGTHS_NM]
    ref_only = [float(w) for w in hw.WL_REFERENCE_ONLY_NM]
    table: dict[str, dict[float, tuple[float, float]]] = {}
    for row in ws.iter_rows(min_row=2, values_only=True):
        if row[ib] is None:
            continue
        code = str(row[ib]).strip()
        if code.upper() == "DARK":
            continue
        w = float(row[iw])
        if w in ref_only:
            continue                       # 950 只存档，不进反演
        if w not in wl_list:
            raise ValueError(f"{code} 的波长 {w:.0f} 既不在反演波长也不在参考波长里")
        table.setdefault(code, {})[w] = (float(row[i3]), float(row[i4]))

    codes = [c for c in phantom.BLOCK_CODES if c in table]
    unknown = sorted(set(table) - set(phantom.BLOCK_CODES))
    if unknown:
        raise ValueError(f"「汇总」表里有未知块号 {unknown}")
    if not codes:
        raise ValueError("「汇总」表里一块有效体模数据都没有")

    I = np.zeros((len(codes), len(wl_list), 2))
    for i, c in enumerate(codes):
        missing = [w for w in wl_list if w not in table[c]]
        if missing:
            raise ValueError(f"块 {c} 缺波长 {[f'{w:.0f}' for w in missing]}")
        for j, w in enumerate(wl_list):
            I[i, j] = table[c][w]
    if len(codes) < len(phantom.BLOCK_CODES):
        print(f"  ⚠️ 只有 {len(codes)}/{len(phantom.BLOCK_CODES)} 块："
              f"{','.join(codes)}。块数减少会直接抬高 G 的不确定度"
              f"（σ ∝ 1/√N，9 块是 12 块的 1.14 倍）")
    return I, tuple(codes)


def load_baseline(path: Path) -> np.ndarray:
    import csv

    d = {}
    with path.open(encoding="utf-8-sig", newline="") as f:
        for row in csv.DictReader(f):
            d[float(row["wavelength_nm"])] = float(row["dispersion"])
    missing = [w for w in hw.WAVELENGTHS_NM if float(w) not in d]
    if missing:
        raise ValueError(f"基线 CSV 缺波长 {[f'{w:.0f}' for w in missing]}")
    return np.array([d[float(w)] for w in hw.WAVELENGTHS_NM])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="SRS 体模增益标定（P3137034）")
    ap.add_argument("--csv", type=Path, help="实测光强 CSV")
    ap.add_argument("--xlsx", type=Path, help="srs_cal.acquire 产出的工作簿")
    ap.add_argument("--baseline", type=Path, help="块间离散基线 CSV")
    ap.add_argument("--dark", type=float, default=0.0,
                    help="统一暗电平计数，从所有光强中扣除（默认 0，表示已扣）")
    ap.add_argument("--n-phantom", type=float, default=phantom.REFRACTIVE_INDEX,
                    help=f"体模折射率（默认 {phantom.REFRACTIVE_INDEX}，厂家答复值）")
    ap.add_argument("--emit-template", type=Path, help="写出空白录入模板后退出")
    ap.add_argument("--out", type=Path, help="把 G(λ) 存成 .npy")
    args = ap.parse_args(argv)

    if args.emit_template:
        emit_template(args.emit_template)
        return 0
    if args.csv and args.xlsx:
        ap.error("--csv 与 --xlsx 只能给一个")
    if not (args.csv or args.xlsx):
        ap.error("需要 --xlsx（采集软件产出）或 --csv，"
                 "或用 --emit-template 先生成录入模板")

    I, codes = load_xlsx(args.xlsx) if args.xlsx else load_csv(args.csv)
    if args.dark:
        I = I - args.dark
    base = load_baseline(args.baseline) if args.baseline else None

    print("=" * 66)
    print(f"体模组 P3137034 | 波长 {'/'.join(f'{w:.0f}' for w in hw.WAVELENGTHS_NM)} nm"
          f" | ρ = {'/'.join(f'{r:.1f}' for r in hw.RHO_CM)} cm")
    print(f"体模折射率 n = {args.n_phantom}"
          + ("（厂家 2026-10 答复）"
             if args.n_phantom == phantom.REFRACTIVE_INDEX else ""))
    print("=" * 66)

    res = calibrate_gain(I, hw.WAVELENGTHS_NM, hw.RHO_CM, block_codes=codes,
                         refractive_index=args.n_phantom, baseline_dispersion=base)
    print(res.report())

    if args.out:
        np.save(args.out, res.gain_correction)
        print(f"\nG(λ) 已存至 {args.out}")
        print("用法： hardware.geometry(gain_correction=np.load(...))")
    return 0 if res.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
