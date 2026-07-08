"""MBLL 基础公式与消光系数表（替代 nirsimple.preprocessing）。"""

from __future__ import annotations

import warnings
from pathlib import Path

import numpy as np
import pandas as pd

_TABLES_DIR = Path(__file__).resolve().parent / "tables"


def get_dpf(wavelength_nm: float, age: float) -> float:
    """
    差分路径长度因子 DPF（Scholkmann & Wolf, 2013）。

    与 nirsimple.preprocessing.get_dpf 公式一致。
    """
    wl = float(wavelength_nm)
    return (
        223.3
        + 0.05624 * float(age) ** 0.8493
        - 5.723e-7 * wl**3
        + 0.001245 * wl**2
        - 0.9025 * wl
    )


def intensities_to_od_changes(
    intensities: np.ndarray,
    refs: list[float] | np.ndarray | None = None,
) -> np.ndarray:
    """
    光强 → 相对光密度变化 ΔOD。

    ΔOD = -log10(I_t / I_ref)；无 refs 时 I_ref 为各通道时间均值。
    形状：(n_channels, n_timepoints)。
    """
    data = np.asarray(intensities, dtype=float)
    if refs is None:
        means = np.mean(np.abs(data), axis=1, keepdims=True)
        delta_od = -np.log10(np.abs(data) / means)
    else:
        references = np.asarray(refs, dtype=float).reshape(-1, 1)
        delta_od = -np.log10(np.abs(data) / references)
        if np.any(references <= 0):
            warnings.warn("some references are negative or equal to zero", stacklevel=2)
    if np.any(data <= 0):
        warnings.warn("some intensities are negative or equal to zero", stacklevel=2)
    return delta_od


def hemoglobin_extinctions(
    wavelengths: list[float],
    table: str = "wray",
) -> np.ndarray:
    """
    读取 HbO/HbR 摩尔消光系数表并插值到目标波长。

    返回 shape (n_wavelengths, 2)，列顺序为 [hbo, hbr]。
    """
    ex_path = _TABLES_DIR / f"{table}.csv"
    ex_df = pd.read_csv(ex_path)
    table_wls = ex_df["lambda"].to_numpy(dtype=float)
    hbo = ex_df["hbo"].to_numpy(dtype=float)
    hbr = ex_df["hbr"].to_numpy(dtype=float)
    return np.column_stack(
        [
            np.interp(wavelengths, table_wls, hbo),
            np.interp(wavelengths, table_wls, hbr),
        ]
    )


def mbll_hbo_hbr(
    delta_od: np.ndarray,
    ch_names: list[str],
    ch_wls: list[float],
    ch_dpfs: list[float],
    ch_distances: list[float],
    *,
    unit: str = "cm",
    table: str = "wray",
) -> tuple[np.ndarray, list[str], list[str]]:
    """
    双波长经典 MBLL，反演 HbO/HbR（兼容旧脚本 nirsimple.mbll 接口）。

    返回 (delta_c, new_ch_names, new_ch_types)。
    delta_c shape: (2 * n_logical_channels, n_timepoints)，行交替 hbo/hbr。
    """
    if unit == "cm":
        distances = [float(d) for d in ch_distances]
    elif unit == "mm":
        distances = [float(d) / 10.0 for d in ch_distances]
    else:
        raise ValueError("unit should be 'cm' or 'mm'")

    delta_c_blocks: list[np.ndarray] = []
    new_ch_names: list[str] = []
    new_ch_types: list[str] = []

    for name in np.unique(ch_names):
        idx_pair = [idx for idx, ch in enumerate(ch_names) if ch == name]
        if len(idx_pair) != 2:
            raise ValueError(f"channel {name!r} must have exactly two wavelengths for mbll_hbo_hbr")

        idx_1, idx_2 = idx_pair
        sub_delta_od = np.stack([delta_od[idx_1], delta_od[idx_2]], axis=0)
        sub_delta_od = np.swapaxes(sub_delta_od, 0, 1)[:, :, np.newaxis]

        ex = hemoglobin_extinctions([ch_wls[idx_1], ch_wls[idx_2]], table=table)
        ex_inv = np.linalg.inv(ex)
        ex_inv = np.tile(ex_inv, (sub_delta_od.shape[0], 1, 1))

        pathlength = np.array(
            [
                ch_dpfs[idx_1] * distances[idx_1],
                ch_dpfs[idx_2] * distances[idx_2],
            ],
            dtype=float,
        )
        pathlength = np.tile(pathlength, (sub_delta_od.shape[0], 1))[:, :, np.newaxis]

        sub_delta_c = np.matmul(ex_inv, sub_delta_od / pathlength)
        sub_delta_c = np.swapaxes(sub_delta_c, 0, 1)

        delta_c_blocks.append(sub_delta_c[0])
        delta_c_blocks.append(sub_delta_c[1])
        new_ch_names.extend([name, name])
        new_ch_types.extend(["hbo", "hbr"])

    delta_c = np.squeeze(np.array(delta_c_blocks))
    return delta_c, new_ch_names, new_ch_types
