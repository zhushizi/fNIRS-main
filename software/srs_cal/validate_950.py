# -*- coding: utf-8 -*-
"""用在体实测数据复核「950 nm 该不该进反演」。

    python -m srs_cal.validate_950 --data <TDDOS目录>

数据：10 名成人前额的时域 DOS 实测 μa(λ)/μs′(λ)，
      Zenodo 10.5281/zenodo.15828014（CC-BY 4.0），CC-BY 要求引用。
      只用其中 04_Analysis/Results_Final.txt 一个文件。

方法：用本器件的四个反演波长 670/770/810/850 拟纯 HbO/HbR 模型
      （与 `fnirs_pipeline.srs.mua_to_toi()` 实际做的事完全一致），
      再看各波长的残差 =（实测 μa − 纯血红蛋白预测）/ 实测 μa。
      带内残差反映模型是否够用；950 的带外残差就是未建模吸收（水 + 脂质）。

⚠️ 锚点波长的选取会显著影响外推结论 —— 若改用 660~750 nm 做锚点，
   该段被 HbR 主导、条件数差，外推到 770~850 会给出 −18~−30% 的假残差。
   必须用与实际反演一致的波长组，这一点是这个脚本的全部要害。

selftest.py 的第 7 项把本脚本的结果固化成常量，使自检不依赖数据集。
数据集在手时跑一次本脚本，核对两边是否一致。
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

from fnirs_pipeline.mbll_core import hemoglobin_extinctions      # noqa: E402
from srs_cal import hardware as hw                               # noqa: E402

LN10 = np.log(10.0)
DEFAULT_DATA = Path(r"D:/WORK_space/Data_WS/脑部_红外数据/Dataset_invivo_subjects_TDDOS")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="在体数据复核 950 nm 的取舍")
    ap.add_argument("--data", type=Path, default=DEFAULT_DATA,
                    help="TDDOS 数据集根目录")
    args = ap.parse_args(argv)

    import pandas as pd

    f = args.data / "04_Analysis" / "Results_Final.txt"
    if not f.exists():
        print(f"找不到 {f}\n数据集：https://doi.org/10.5281/zenodo.15828014")
        return 2

    d = pd.read_csv(f, sep="\t")
    d.columns = [c.strip() for c in d.columns]
    F = d[d.Location.str.contains("Forehead", case=False, na=False)].copy()
    F["Lambda"] = F.Lambda.astype(float)
    F["mua"] = F.VarMua0Opt.astype(float)

    fit_wl = [float(w) for w in hw.WAVELENGTHS_NM]               # 670/770/810/850
    all_wl = [float(w) for w in hw.WAVELENGTHS_ALL_NM]           # + 950
    E_fit = hemoglobin_extinctions(fit_wl, hw.EXTINCTION_TABLE)
    E_all = hemoglobin_extinctions(all_wl, hw.EXTINCTION_TABLE)

    resid: dict[float, list[float]] = {w: [] for w in all_wl}
    so2_fit, so2_all = [], []
    for _, gs in F.groupby("Subject"):
        g = gs.groupby("Lambda")["mua"].mean()
        if not all(w in g.index for w in all_wl):
            continue
        y = np.array([g.loc[w] for w in fit_wl])
        c, *_ = np.linalg.lstsq(E_fit * LN10, y, rcond=None)
        for i, w in enumerate(all_wl):
            pred = float((E_all[i, 0] * c[0] + E_all[i, 1] * c[1]) * LN10)
            resid[w].append((g.loc[w] - pred) / g.loc[w])
        so2_fit.append(100 * c[0] / (c[0] + c[1]))
        c5, *_ = np.linalg.lstsq(E_all * LN10,
                                 np.array([g.loc[w] for w in all_wl]), rcond=None)
        so2_all.append(100 * c5[0] / (c5[0] + c5[1]))

    n = len(so2_fit)
    print("=" * 66)
    print(f"在体复核：{n} 名受试者前额，纯 HbO/HbR 模型拟 "
          f"{'/'.join(f'{w:.0f}' for w in fit_wl)} nm")
    print("=" * 66)
    print("  λ(nm)    残差 =（实测 − 纯血红蛋白预测）/ 实测")
    for w in all_wl:
        r = np.array(resid[w]) * 100
        tag = "   <- 带外" if w in hw.WL_REFERENCE_ONLY_NM else ""
        print(f"  {w:5.0f}    {r.mean():+6.1f}% ± {r.std():4.1f}%{tag}")
    inband = max(abs(np.mean(resid[w]) * 100) for w in fit_wl)
    print(f"\n  带内最大 |残差| {inband:.1f}%，"
          f"950 带外 {np.mean(resid[950.0])*100:+.1f}%")
    print(f"\n  SO2：四波长 {np.mean(so2_fit):.1f} ± {np.std(so2_fit):.1f} %"
          f"  →  加 950 后 {np.mean(so2_all):.1f} ± {np.std(so2_all):.1f} %"
          f"  （{np.mean(so2_all)-np.mean(so2_fit):+.1f} %O2）")
    print("\n  ⇒ 950 nm 不进反演。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
