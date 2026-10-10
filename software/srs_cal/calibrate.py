# -*- coding: utf-8 -*-
"""由 12 块体模的实测光强求增益修正 G(λ)，并给出验收判据。

原理
----
SRS 的斜率域里，光源功率与**波长共模**的增益都已被 Σw=0 结构约掉，
剩下的是**探测器间**、且**随波长变化**的增益失配。它在斜率上表现为
一个与 ρ 无关的常数偏移，故每个波长只需一个数：

    G(λ) = 实测斜率(λ) − 预测斜率(λ)

预测斜率由 `forward.predicted_slope()` 从体模的认证 μa/μs′ 算出。

为什么 12 块全部用于拟合、不分标定集/验证集
--------------------------------------------
每波长只有 1 个待定参数、12 个观测，过拟合不是风险 —— 切分要防的病
这里没有。而切分有确定代价：9 块拟合的 σ 恒为 12 块的 √(12/9)=1.14 倍
（实测 1.14×，三个噪声水平下一致）。且设定错误时 9 块反而更差
（近→远串扰 1% 时偏差 1.385 vs 1.302 %O₂）—— 花精度换零信息。

真正的验证用**块间离散**与**趋势检验**，两者都用满 12 块、不花数据，
且比留一交叉验证更灵敏（1% 串扰：离散 27.5× 基线，LOOCV 仅 16.2×）。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from . import phantom
from .forward import predicted_slope

# 块间离散的报警阈值，以无故障基线的倍数计。
# 依据：近→远串扰 0.1% 即使离散升到基线的 4.1 倍，0.5% 升到 16.5 倍。
# 取 3.0 使 0.1% 量级的串扰即可被抓住，同时给测量噪声留出余量。
DISPERSION_ALARM_RATIO = 3.0
# 趋势检验的 |t| 阈值。12 块、2 参数 → df=10。
#
# ⚠️ **不能用裸的 2.228**（df=10 双侧 0.05）。每次标定要跑
#    n_wl × 2 = 8 次检验（每个波长各对 μa、μs′ 查一遍），逐项 5% 的
#    误报率累起来就是族系误报率。实测（400 次干净闭环，注入的是常数 G）：
#
#        阈值     至少报一次假警    单项误报率
#        2.228        29.8%           4.62%
#        3.169         8.5%           1.16%
#        3.448         ~5%            ~0.7%
#        4.000         1.2%           0.16%
#
#    取 Bonferroni 修正值 α=0.05/8=0.00625 → t(df=10)=3.448，
#    使整次标定的族系误报率回到 5% 量级。出厂判据宁可漏报也不能天天误报。
TREND_T_CRITICAL = 3.448
# 检验次数随波长数变化时，阈值应随之重算：
#   scipy.stats.t.ppf(1 - 0.05 / (2 * n_wl * 2), n_blocks - 2)


@dataclass
class CalibrationResult:
    """G(λ) 及其验收指标。"""

    gain_correction: np.ndarray          # G(λ)，(n_wl,)，直接喂 SrsGeometry
    wavelengths_nm: np.ndarray
    per_block_gain: np.ndarray           # (n_blocks, n_wl)，每块单独反推的 G
    block_codes: tuple[str, ...]
    dispersion: np.ndarray               # (n_wl,) 块间标准差
    trend_t_mua: np.ndarray              # (n_wl,) G 对 μa 的回归 t 值
    trend_t_musp: np.ndarray             # (n_wl,) G 对 μs′ 的回归 t 值
    flags: list[str] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return not self.flags

    def report(self) -> str:
        lines = ["G(λ) 标定结果", "-" * 62,
                 "  λ(nm)      G(λ)      块间离散    t(vs μa)   t(vs μs′)"]
        for i, w in enumerate(self.wavelengths_nm):
            lines.append(f"  {w:6.0f}   {self.gain_correction[i]:+9.5f}"
                         f"   {self.dispersion[i]:9.5f}"
                         f"   {self.trend_t_mua[i]:+8.2f}   {self.trend_t_musp[i]:+9.2f}")
        lines.append("-" * 62)
        lines.append("  [通过] 无异常" if self.passed
                     else "\n".join(f"  [异常] {f}" for f in self.flags))
        return "\n".join(lines)


def _regression_t(x: np.ndarray, y: np.ndarray) -> float:
    """y 对 x 做一元线性回归，返回斜率的 t 统计量。

    用来判断「G 是否随块的光学参数系统漂移」。离散小但有趋势，说明
    G-as-常数 这个模型形式本身错了 —— 而体模的 μa/μs′ 与组织并不重合，
    这个趋势会在外推到组织时变成系统偏差。离散检验抓不到它。
    """
    n = x.size
    if n < 3 or np.std(x) == 0:
        return 0.0
    k, b0 = np.polyfit(x, y, 1)
    resid = y - (k * x + b0)
    dof = n - 2
    s2 = float(resid @ resid) / dof
    sxx = float(((x - x.mean()) ** 2).sum())
    se = np.sqrt(s2 / sxx) if sxx > 0 and s2 > 0 else 0.0
    return float(k / se) if se > 0 else 0.0


def calibrate_gain(intensities: np.ndarray,
                   wavelengths_nm,
                   rho_cm,
                   *,
                   block_codes: tuple[str, ...] = phantom.BLOCK_CODES,
                   refractive_index: float = phantom.REFRACTIVE_INDEX_ASSUMED,
                   dark_level: np.ndarray | None = None,
                   baseline_dispersion: np.ndarray | None = None,
                   ) -> CalibrationResult:
    """从 12 块的实测光强求 G(λ)。

    intensities : (n_blocks, n_wl, n_rho) 各块的原始光强（ADC 计数）
    dark_level  : 同形状或 (n_wl, n_rho) 的暗电平，None 时不扣
    baseline_dispersion : 无故障系统的块间离散基线 (n_wl,)。给定时按
                  DISPERSION_ALARM_RATIO 判定；未给定时跳过该项检查
                  并在 flags 里注明 —— **不要默认它通过**。
    """
    from fnirs_pipeline.srs import fit_slopes, to_attenuation

    I = np.asarray(intensities, dtype=float)
    wl = np.asarray(wavelengths_nm, dtype=float)
    rho = np.asarray(rho_cm, dtype=float)
    n_blocks = I.shape[0]
    if n_blocks != len(block_codes):
        raise ValueError(f"光强给了 {n_blocks} 块，block_codes 有 {len(block_codes)} 个")

    mua, musp = phantom.interpolate_to(wl)

    per_block = np.zeros((n_blocks, wl.size))
    for i in range(n_blocks):
        idx = phantom.BLOCK_CODES.index(block_codes[i])
        A = to_attenuation(I[i], dark_level)
        s_meas, _ = fit_slopes(A, rho)
        s_pred = predicted_slope(mua[idx], musp[idx], rho, refractive_index)
        per_block[i] = s_meas - s_pred

    G = per_block.mean(axis=0)
    disp = per_block.std(axis=0, ddof=1)

    sel = [phantom.BLOCK_CODES.index(c) for c in block_codes]
    t_mua = np.array([_regression_t(mua[sel, j], per_block[:, j]) for j in range(wl.size)])
    t_musp = np.array([_regression_t(musp[sel, j], per_block[:, j]) for j in range(wl.size)])

    flags: list[str] = []
    if baseline_dispersion is None:
        flags.append("未提供块间离散基线，离散判据未执行 —— 本次结果不可直接放行")
    else:
        base = np.asarray(baseline_dispersion, float)
        bad = np.where(disp > DISPERSION_ALARM_RATIO * base)[0]
        for j in bad:
            flags.append(f"{wl[j]:.0f}nm 块间离散 {disp[j]:.5f} 超基线 "
                         f"{disp[j]/base[j]:.1f} 倍（限 {DISPERSION_ALARM_RATIO}）"
                         f" —— 查串扰/非线性")
    for j in range(wl.size):
        if abs(t_mua[j]) > TREND_T_CRITICAL:
            flags.append(f"{wl[j]:.0f}nm 的 G 随 μa 系统漂移（t={t_mua[j]:+.2f}）"
                         f" —— G 不是常数，外推到组织会有系统偏差")
        if abs(t_musp[j]) > TREND_T_CRITICAL:
            flags.append(f"{wl[j]:.0f}nm 的 G 随 μs′ 系统漂移（t={t_musp[j]:+.2f}）")

    return CalibrationResult(gain_correction=G, wavelengths_nm=wl,
                             per_block_gain=per_block, block_codes=tuple(block_codes),
                             dispersion=disp, trend_t_mua=t_mua, trend_t_musp=t_musp,
                             flags=flags)
