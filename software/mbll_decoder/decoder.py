# -*- coding: utf-8 -*-
"""MBLL 解码 —— 由光强时间序列解出 ΔHbO / ΔHbR。

**输出只有相对变化。血氧绝对值由 SRS 给出**（2026-10-22 定版决策），
故本模块不含 rSO2、不需要基线假设、不需要标定。

与 `fnirs_pipeline/mbll.py` 的关系：
    那一份耦合在 `config`（产品线硬件）与 `online_android`（rSO2）上，
    且其入口硬性要求 `Time (s)` 列。本模块另起，只复用
    `mbll_core` 的纯函数。两者互不影响。

与 SRS 侧的关系：
    **预处理不共享** —— 需求相反：SRS 禁止高通（会毁掉绝对值），
    MBLL 必须高通（否则仪器漂移淹没相对变化）。
    共享的只有 L2 原始记录与 `mbll_core` 的物理常数。
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.signal import butter, sosfiltfilt

_HERE = Path(__file__).resolve().parent
if str(_HERE.parent) not in sys.path:
    sys.path.insert(0, str(_HERE.parent))

from fnirs_pipeline.mbll_core import (get_dpf,  # noqa: E402
                                      hemoglobin_extinctions)
from mbll_decoder import hardware as mh  # noqa: E402

LN10 = np.log(10.0)


@dataclass
class MbllResult:
    """一次解码的结果。`flags` 不可丢 —— 里面有 DPF 外推等限定。"""

    t: np.ndarray                      # 重采样后的时间轴，秒
    d_hbo: np.ndarray                  # ΔHbO，µM
    d_hbr: np.ndarray                  # ΔHbR，µM
    fs: float                          # 实际采样率，Hz
    n_in: int = 0                      # 输入样本数
    n_dropped: int = 0                 # 因非正光强等丢弃的样本数
    cond: float = np.nan               # 消光矩阵条件数
    flags: list = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.t.size > 0


def _resample_uniform(t, Y):
    """按 t_host 重采样到均匀栅格。

    ⚠️ 必须做：带通滤波器假定等间隔采样。设备时基虽稳
       （周期 1521.53 ms，MAD 0.04 ms），但丢帧会留下空洞，
       直接当等间隔会把空洞处的时间算错。
       这是 L1 记录 `t_host` 带来的能力 —— 没有逐帧时刻做不了。
    """
    t = np.asarray(t, float)
    n = t.size
    if n < 2:
        return t, Y, np.nan
    dt = float(np.median(np.diff(t)))
    if not np.isfinite(dt) or dt <= 0:
        return t, Y, np.nan
    grid = np.arange(t[0], t[-1] + dt * 0.5, dt)
    out = np.vstack([np.interp(grid, t, y) for y in Y])
    return grid, out, 1.0 / dt


def decode(t, intensities, wavelengths=None, *,
           distance_cm: float = mh.DISTANCE_CM,
           age: float = mh.DEFAULT_AGE,
           band=(mh.BP_LOW_HZ, mh.BP_HIGH_HZ),
           order: int = mh.BP_ORDER,
           table: str = "wray",
           min_record_s: float = mh.MIN_RECORD_S) -> MbllResult:
    """由光强序列解出 ΔHbO/ΔHbR。

    t            : 每个样本的时刻（秒），取自 `Frame.t_host`
    intensities  : (n_wl, n_t) **已扣暗**的光强
    wavelengths  : 默认 `hardware.WAVELENGTHS_NM`
    """
    wl = (mh.WAVELENGTHS_NM if wavelengths is None
          else np.asarray(wavelengths, float))
    I = np.atleast_2d(np.asarray(intensities, float))
    t = np.asarray(t, float)
    flags = list(mh.dpf_note(wl))
    n_in = t.size

    if I.shape[0] != wl.size:
        return MbllResult(np.array([]), np.array([]), np.array([]), np.nan,
                          n_in, n_in, np.nan,
                          flags + [f"波长数不符：光强 {I.shape[0]} 路，"
                                   f"配置 {wl.size} 路"])

    # --- 非正光强无法取对数 ------------------------------------------------
    good = np.isfinite(t) & np.isfinite(I).all(axis=0) & (I > 0).all(axis=0)
    n_drop = int((~good).sum())
    if n_drop:
        flags.append(f"{n_drop}/{n_in} 个样本含非正或非有限光强，已剔除")
    t, I = t[good], I[:, good]
    if t.size < 8:
        return MbllResult(np.array([]), np.array([]), np.array([]), np.nan,
                          n_in, n_drop, np.nan,
                          flags + ["有效样本不足"])

    # --- 重采样到均匀栅格（用 t_host）--------------------------------------
    grid, Iu, fs = _resample_uniform(t, I)
    if not np.isfinite(fs):
        return MbllResult(np.array([]), np.array([]), np.array([]), np.nan,
                          n_in, n_drop, np.nan, flags + ["无法确定采样率"])

    span = float(grid[-1] - grid[0])
    lo, hi = float(band[0]), float(band[1])

    # --- 记录是否够长 ------------------------------------------------------
    # ⚠️ 不够长时**拒绝出结果**。高通瞬态的特征时间约 1/fc，需约 3 倍
    #    才衰减干净；强行滤出来的曲线看着像样，其实是滤波器的瞬态。
    if span < min_record_s:
        return MbllResult(np.array([]), np.array([]), np.array([]), fs,
                          n_in, n_drop, np.nan,
                          flags + [f"记录 {span:.0f}s 短于 {min_record_s:.0f}s"
                                   f"（高通 {lo} Hz 所需），拒绝出结果"])
    nyq = fs / 2.0
    if hi >= nyq:
        return MbllResult(np.array([]), np.array([]), np.array([]), fs,
                          n_in, n_drop, np.nan,
                          flags + [f"带通上限 {hi} Hz 超过奈奎斯特 {nyq:.3f} Hz"])

    # --- ΔOD --------------------------------------------------------------
    # 参考取全段均值：MBLL 给的本就是相对量，参考点的选择只平移曲线。
    dOD = -np.log10(Iu / Iu.mean(axis=1, keepdims=True))

    # --- 带通 --------------------------------------------------------------
    sos = butter(order, [lo / nyq, hi / nyq], btype="band", output="sos")
    # padlen 由**最低截止频率**决定，不是由阶数决定。
    # scipy 的默认 3*(order+1)=15 对 0.01 Hz 短两个数量级。
    want = max(3 * (order + 1), int(round(3.0 * fs / lo)))
    padlen = min(dOD.shape[1] - 1, want)
    dOD = sosfiltfilt(sos, dOD, axis=1, padtype="odd", padlen=padlen)

    # --- 解 HbO/HbR --------------------------------------------------------
    E = hemoglobin_extinctions([float(w) for w in wl], table)   # 1/(M·cm)
    L = np.array([get_dpf(float(w), age) for w in wl]) * float(distance_cm)
    A = E * L[:, None]
    cond = float(np.linalg.cond(A))
    c = np.linalg.pinv(A) @ dOD                                 # mol/L

    if not mh.cyt_solvable(wl):
        flags.append(f"仅 {wl.size} 个波长：Cyt 不可解，已跳过")
    if wl.size < 3:
        flags.append("2 波长 → 矩阵恰定，**残差恒为零，无自检**："
                     "任何输入（含纯噪声）都会被映射成一对浓度")
    if not mh.HAS_SHORT_SEPARATION:
        flags.append("无短距通道 → 未做 SSR 去头皮")

    return MbllResult(grid - grid[0], c[0] * 1e6, c[1] * 1e6, fs,
                      n_in, n_drop, cond, flags)
