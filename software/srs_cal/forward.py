# -*- coding: utf-8 -*-
"""正向模型：Farrell-Patterson 半无限稳态漫反射解。

标定需要「已知 μa/μs′ 的块**应该**给出多大斜率」这个预测值，
G(λ) = 实测斜率 − 预测斜率。所以正演是标定的一半。

本文件与 `tools/srs_reference.py` 的 `reflectance()` 是同一个公式，
刻意写成独立副本以保持本包自足；`selftest.py` 第 1 项会逐点比对两者，
任何一边改动而另一边没跟上都会被自检抓住。
"""

from __future__ import annotations

import numpy as np

LN10 = np.log(10.0)


def r_eff(n: float) -> float:
    """内反射系数（Groenhuis / Egan-Hilgeman 经验式）。"""
    return -1.440 / n ** 2 + 0.710 / n + 0.668 + 0.0636 * n


def reflectance(rho, mua: float, musp: float, n: float = 1.4) -> np.ndarray:
    """半无限介质稳态漫反射 R(ρ)。rho: (n_rho,) cm；mua/musp: 标量 cm⁻¹。"""
    rho = np.asarray(rho, float)
    mueff = np.sqrt(3.0 * mua * (mua + musp))
    z0 = 1.0 / (mua + musp)
    D = 1.0 / (3.0 * (mua + musp))
    zb = 2.0 * D * (1.0 + r_eff(n)) / (1.0 - r_eff(n))
    r1 = np.sqrt(rho ** 2 + z0 ** 2)
    r2 = np.sqrt(rho ** 2 + (z0 + 2 * zb) ** 2)
    return (1.0 / (4 * np.pi)) * (
        z0 * (mueff + 1.0 / r1) * np.exp(-mueff * r1) / r1 ** 2
        + (z0 + 2 * zb) * (mueff + 1.0 / r2) * np.exp(-mueff * r2) / r2 ** 2
    )


def attenuation(rho, mua: float, musp: float, n: float = 1.4) -> np.ndarray:
    """A(ρ) = −log10(R(ρ))。"""
    return -np.log10(reflectance(rho, mua, musp, n))


def attenuation_matrix(mua, musp, rho, n: float = 1.4) -> np.ndarray:
    """多波长版本。mua/musp: (n_wl,)，返回 (n_wl, n_rho)。"""
    mua = np.asarray(mua, float)
    musp = np.asarray(musp, float)
    return np.array([attenuation(rho, mua[i], musp[i], n) for i in range(mua.size)])


def predicted_slope(mua, musp, rho, n: float = 1.4) -> np.ndarray:
    """每个波长的 dA/dρ 预测值 —— 标定的参照量。

    用【最小二乘拟合】而不是解析渐近式 (μeff+2/ρ̄)/ln10，因为实测侧
    `fnirs_pipeline.srs.fit_slopes()` 也是拟合的。两边必须用同一个定义，
    否则渐近式与精确解之间的差（ρ=3~4 cm、μeff≈1 时约 2%）会被
    整个吞进 G(λ)，变成一个与波长相关的假增益误差。
    """
    rho = np.asarray(rho, float)
    A = attenuation_matrix(mua, musp, rho, n)
    return np.array([np.polyfit(rho, A[i], 1)[0] for i in range(A.shape[0])])


def intensity_matrix(mua, musp, rho, n: float = 1.4,
                     scale: float = 1e12) -> np.ndarray:
    """A → 模拟 ADC 计数，供喂给 `compute_toi()`。

    scale 把 10^(−A) 换算成计数。A(4cm) 在本体模集上跨 3.55~7.97 OD，
    1e12 使最暗的 D4 块落在 ~1e4 计数，高于 srs.py 的 MIN_COUNTS=2000。
    ⚠️ 真机的换算比例由 LED 功率、PD 响应度与积分时间决定，须实测后替换。
    """
    return 10 ** (-attenuation_matrix(mua, musp, rho, n)) * scale
