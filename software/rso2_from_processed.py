from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd


DEFAULT_INPUT = Path("software/result_table/2026-04-14_16-00-31/processed_output.csv")


def _pick_column(columns: list[str], keyword: str) -> str:
    matches = [col for col in columns if keyword.lower() in col.lower()]
    if not matches:
        raise ValueError(f"未找到包含 '{keyword}' 的列，现有列: {columns}")
    if len(matches) > 1:
        raise ValueError(f"找到多个包含 '{keyword}' 的列，请手动指定。现有匹配: {matches}")
    return matches[0]


def compute_rso2_table(
    input_csv: Path,
    output_csv: Path | None = None,
    baseline_start_s: float = 0.0,
    baseline_end_s: float = 15.0,
    baseline_hbt_uM: float = 80.0,
    baseline_rso2_pct: float = 65.0,
) -> Path:
    df = pd.read_csv(input_csv)
    if df.empty:
        raise ValueError("输入 CSV 为空。")

    columns = list(df.columns)
    time_col = _pick_column(columns, "Time")
    hbo_col = _pick_column(columns, "hbo")
    hbr_col = _pick_column(columns, "hbr")

    times = df[time_col].to_numpy(dtype=float)
    delta_hbo = df[hbo_col].to_numpy(dtype=float)
    delta_hbr = df[hbr_col].to_numpy(dtype=float)

    baseline_mask = (times >= baseline_start_s) & (times <= baseline_end_s)
    if not np.any(baseline_mask):
        raise ValueError(
            f"在 {baseline_start_s:.1f}s ~ {baseline_end_s:.1f}s 内没有找到基线样本，请调整基线时间窗口。"
        )

    # processed_output.csv 里是浓度变化量，这里按照“固定基线 HbT 和基线 rSO2”
    # 先反推出基线绝对 HbO/HbR，再把实时变化量叠加回去。
    baseline_hbt_M = baseline_hbt_uM * 1e-6
    baseline_hbo_abs_M = baseline_hbt_M * (baseline_rso2_pct / 100.0)
    baseline_hbr_abs_M = baseline_hbt_M - baseline_hbo_abs_M

    baseline_delta_hbo_mean = float(np.mean(delta_hbo[baseline_mask]))
    baseline_delta_hbr_mean = float(np.mean(delta_hbr[baseline_mask]))

    hbo_abs_M = baseline_hbo_abs_M + (delta_hbo - baseline_delta_hbo_mean)
    hbr_abs_M = baseline_hbr_abs_M + (delta_hbr - baseline_delta_hbr_mean)
    hbo_abs_M = np.maximum(hbo_abs_M, 0.0)
    hbr_abs_M = np.maximum(hbr_abs_M, 0.0)
    hbt_abs_M = hbo_abs_M + hbr_abs_M

    with np.errstate(divide="ignore", invalid="ignore"):
        rso2_pct = np.where(hbt_abs_M > 0, 100.0 * hbo_abs_M / hbt_abs_M, np.nan)

    result_mask = times > baseline_end_s
    if not np.any(result_mask):
        raise ValueError(f"在 {baseline_end_s:.1f}s 之后没有可输出的 rSO2 数据。")

    out_df = pd.DataFrame(
        {
            "Time": times[result_mask],
            "rso2_pct": rso2_pct[result_mask],
        }
    )

    if output_csv is None:
        output_csv = input_csv.with_name("rso2_output.csv")
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    out_df.to_csv(output_csv, index=False)

    mean_baseline_rso2 = float(np.nanmean(rso2_pct[baseline_mask]))
    post_rso2 = rso2_pct[result_mask]
    print(f"输入文件: {input_csv}")
    print(f"输出文件: {output_csv}")
    print(
        "基线设定: "
        f"HbT={baseline_hbt_uM:.2f} uM, "
        f"rSO2={baseline_rso2_pct:.2f}%, "
        f"窗口={baseline_start_s:.1f}s~{baseline_end_s:.1f}s"
    )
    print(
        "基线窗内校准后均值: "
        f"HbO={np.mean(hbo_abs_M[baseline_mask]) * 1e6:.3f} uM, "
        f"HbR={np.mean(hbr_abs_M[baseline_mask]) * 1e6:.3f} uM, "
        f"rSO2={mean_baseline_rso2:.3f}%"
    )
    print(
        f"{baseline_end_s:.1f}s 后 rSO2: "
        f"min={np.nanmin(post_rso2):.3f}%, "
        f"max={np.nanmax(post_rso2):.3f}%, "
        f"mean={np.nanmean(post_rso2):.3f}%"
    )
    return output_csv


def main() -> None:
    parser = argparse.ArgumentParser(description="根据 processed_output.csv 计算 rSO2 表。")
    parser.add_argument(
        "--input",
        type=Path,
        default=DEFAULT_INPUT,
        help="输入 processed_output.csv 路径",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="输出 rSO2 CSV 路径，默认与输入同目录下的 rso2_output.csv",
    )
    parser.add_argument("--baseline-start", type=float, default=0.0, help="基线起始时间（秒）")
    parser.add_argument("--baseline-end", type=float, default=15.0, help="基线结束时间（秒）")
    parser.add_argument("--baseline-hbt", type=float, default=80.0, help="固定基线 HbT（uM）")
    parser.add_argument("--baseline-rso2", type=float, default=65.0, help="固定基线 rSO2（%%）")
    args = parser.parse_args()

    compute_rso2_table(
        input_csv=args.input,
        output_csv=args.output,
        baseline_start_s=args.baseline_start,
        baseline_end_s=args.baseline_end,
        baseline_hbt_uM=args.baseline_hbt,
        baseline_rso2_pct=args.baseline_rso2,
    )


if __name__ == "__main__":
    main()
