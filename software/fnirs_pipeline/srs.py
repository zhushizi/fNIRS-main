"""
空间分辨光谱（SRS）反演管线 —— 由多距离衰减斜率求绝对组织氧饱和度 TOI。

与 MBLL 通路**并联**，不替换：MBLL 出 Δ 浓度（相对），SRS 出 TOI（绝对）。

    MBLL：时间差分  ΔA = log10(I_t0/I_t)   -> 增益约掉，只得相对量
    SRS ：空间差分  斜率 Σw·A(ρ)，Σw=0     -> 光源功率约掉，得绝对量
                                              但探测器【间】增益差不约掉，须标定

处理链：

    原始光强 I[探测器, 波长]
      ↓ 扣暗电平
      ↓ 应用增益修正 G(λ)          ← 体模矩阵标定的产物，每波长一个数
      ↓ A = −log10(I)
      ↓ 对 ρ 拟合直线 → 斜率 + 残差 ← 残差是线性度自检（≈0.018 OD 为正常签名）
      ↓ μeff = ln10·斜率 − 2/ρ̄
      ↓ μa = μeff² / (3·μs′)        ← μs′ 固定假设 或 五波长自解谱指数
      ↓ ε 表反演 → c_HbO, c_HbR
      ↓ TOI = c_HbO/(c_HbO+c_HbR)
      ↓ 两参数线性修正 k·TOI + b    ← 降氧标定的产物

⚠️ **适用边界**：均匀半无限介质 + Farrell-Patterson 稳态扩散解。
   真实头部为分层结构，本模块在分层几何下的表现**未经验证**。
   TOI 反映的是「脑 + 头皮混合」组织的血氧，不是纯脑血氧。

⚠️ **未经临床验证，不可用于诊断。**

---------------------------------------------------------------------------
验证状态（截至最后一次更新）
---------------------------------------------------------------------------

| 层次                       | 状态 | 依据                                    |
|----------------------------|------|-----------------------------------------|
| 代码实现无误（符号/量纲）  | ✅   | tools/srs_reference.py 自检 10 项       |
| 正演模型能描述真实介质     | ✅   | BRUNO 血液体模实测，全谱拟合残差 0.3%   |
| 真实组织光学参数下的趋势   | ✅   | 见下「趋势验证」                        |
| 自解谱指数在真实数据上可用 | ❌   | 失效，b 顶边界；默认已关闭              |
| 时间维度上的真实氧变化     | ❌   | 无此类公开数据                          |
| 分层几何                   | ❌   | 未验证                                  |
| 绝对准确度                 | ❌   | 需人体受控降氧标定                      |

**趋势验证**：取 10 名成人前额的时域 DOS 实测 μa(λ)/μs′(λ)（Zenodo
10.5281/zenodo.15828014，CC-BY 4.0）正演多距离衰减，再用本模块反演：

    固定假设 μs′   偏差 −0.47 ± 1.78 %O2   与真值相关 0.9411   斜率 1.086
    自解谱指数     偏差 +5.18 ± 1.62 %O2   与真值相关 0.9454   斜率 1.044

真值跨度 66.3~79.6 %O2（10 人，13.3 点），读数跨度 16.9 点 —— **无量程压缩**。

⚠️ 该验证用的是**受试者间**的真实差异，不是**同一个人的时间变化**；
   不含运动伪影、贴合波动、头皮动态。**趋势方向与斜率已验，时间响应未验。**

⚠️ 文献对照：真实血液体模上，SRS 类算法仅恢复出 39~80%（真值 0~100%），
   Dual Slope 误差 ~20~30%（Biomed Opt Express 12(2):907；arXiv 2512.04787）。
   **仿真与真实测量之间存在显著差距，本模块的仿真表现不可外推。**
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field, replace

import numpy as np

from .mbll_core import hemoglobin_extinctions

LN10 = np.log(10.0)

# ---------------------------------------------------------------------------
# 默认参数
# ---------------------------------------------------------------------------
# 标称 μs′ —— 取自成人前额的时域 DOS 实测人群均值（10 人）
# 数据来源：Zenodo 10.5281/zenodo.15828014（CC-BY 4.0），见文件末「验证状态」
MUSP_REF_800 = 10.83            # μs′(800nm) cm⁻¹（绝对值不影响 TOI，但影响 μa）
MUSP_EXPONENT = 0.932           # μs′ ∝ λ^(−b) 的谱指数，人群均值
# 自解谱指数的生理约束。实测逐人 b = 0.538~1.303（均值 0.927 ± 0.191），
# 下界取 0.4 留余量 —— 原先的 0.5 会把低散射个体卡在边界上。
B_BOUNDS = (0.4, 1.6)
SLOPE_RESIDUAL_MAX = 0.05       # 三点拟合残差上限（理论值 ≈0.018 OD）
MIN_COUNTS = 2000               # 最远探测器的有效计数下限

# ---------------------------------------------------------------------------
# 物理判据（2026-10-22 新增）—— 不需要任何外部参考，只靠生理学与物理上限
# ---------------------------------------------------------------------------
# 两探测器 + 两波长时空间残差与光谱残差**都恒为零**（见 compute_toi 的 flags），
# 设备出了异常不会报警。这组判据是当前唯一还能抓到异常的东西。
#
# tHb 窗口取 20~120 µM：本仓库自己的 MBLL 基线假设 80 µM
# （online_android/rso2.py: DEFAULT_BASELINE_HBT_uM），成人组织常见 40~80 µM，
# 两侧各留约 2 倍余量。**刻意取宽** —— 它要抓的是「2.66 µM」「TOI 106.8%」
# 这种量级的失败（2026-10-13/14 在体实测出现过），不是精细判别。
THB_MIN_uM, THB_MAX_uM = 20.0, 120.0

# μa 谱拟合残差上限，cm⁻¹。⚠️ 仅在 dof > 0（≥3 波长）时生效。
#
# 定值依据（正演 + 实测噪声，2026-10-22）。**误报率与灵敏度要一起看** ——
# 初版只按误报率取了 0.015，漏掉灵敏度那一半，自检当场抓到。
#
#   残差本底（正确系统，仅模型近似误差）      0.00024 ~ 0.00035
#   叠加实测通道噪声 3.6% 后的 99 分位        0.0096 ~ 0.0105
#   （3.6% 取自 2026-10-22 在体归档，差分法实测）
#
#     阈值    误报率        单路失配检出下限
#     0.010   1.1 ~ 1.8%   ~20%
#     0.012   0.24~0.29%   ~25 ~ 35%     ← 取此
#     0.015   0.01~0.03%   ~25 ~ 43%
#     0.020   0.00%        ~35 ~ 58%（太钝）
#
# 取 0.012：按 0.66 周期/秒、10 周期一个窗口算，误报约每 1.5 小时一次，
# 而它只是**质量标志**不是报警，这个代价换灵敏度划算。
#
# ⚠️ 检出下限是个**区间**，因为灵敏度取决于失配发生在哪一路：
#    同样 +30% 失配，落在 850 上只产生 0.0104 残差，落在最敏感的那一路
#    产生 0.0188。拟合能部分吸收与张成空间较对齐的那一路的偏差。
#    所以「未超阈」不等于「没有失配」，只能说「没有大到这个程度」。
SPECTRAL_RESIDUAL_MAX = 0.012


# ---------------------------------------------------------------------------
@dataclass
class SrsGeometry:
    """探头几何与标定常数。"""

    rho_cm: np.ndarray                      # 长距探测器的源探距离，升序
    wavelengths_nm: np.ndarray              # 波长，与光强矩阵的行对应
    gain_correction: np.ndarray | None = None   # G(λ)，斜率域加性修正，长度 = n_wl
    dark_level: np.ndarray | None = None        # 暗电平 (n_wl, n_rho)
    k: float = 1.0                          # 降氧标定：TOI_真 = k·TOI_读 + b
    b: float = 0.0

    def __post_init__(self) -> None:
        self.rho_cm = np.asarray(self.rho_cm, dtype=float)
        self.wavelengths_nm = np.asarray(self.wavelengths_nm, dtype=float)
        if self.rho_cm.size < 2:
            raise ValueError("SRS 至少需要 2 个长距探测器才能定出斜率")
        if np.any(np.diff(self.rho_cm) <= 0):
            raise ValueError("rho_cm 必须严格升序")


@dataclass
class SrsResult:
    """单个时刻的反演结果与质量指标。"""

    toi: float                              # 修正后的 TOI (%)
    toi_raw: float                          # 修正前
    mua: np.ndarray                         # 各波长反演的 μa (cm⁻¹)
    mueff: np.ndarray                       # 各波长的 μeff (cm⁻¹)
    slope: np.ndarray                       # 各波长的 dA/dρ
    residual: np.ndarray                    # 各波长的三点拟合残差 RMS (OD)
    b_fitted: float = np.nan                # 自解得到的 μs′ 谱指数
    # flags：**本次测量的异常**。随测随报。
    # config_notes：与本次测量无关的**常驻配置状态**（如「只有 2 波长」）。
    #   ⚠️ 2026-10-22 分出来。此前两者混在 flags 里，后果有二：
    #     ① `valid` 恒为 False —— 常驻项永远在，条件 `not self.flags` 永不成立
    #     ② 下游把它当异常逐次呈现，每次都显示 ✗，真异常被淹没
    #        （真机回放当场抓到：30 次反演全部显示 fail）
    #   常驻项应由界面固定呈现一次，不进逐次异常行。
    flags: list[str] = field(default_factory=list)
    config_notes: list[str] = field(default_factory=list)

    # --- 2026-10-22 补回。此前这三项算出来就被丢掉 ------------------------
    # c_hbo/c_hbr：绝对浓度（mol/L）。`mua_to_toi` 一直在算，调用处用 `_` 丢弃。
    #   它们是 TOI 的来源，且 tHb = c_hbo + c_hbr 构成一条**不需要任何外部
    #   参考**的物理判据：组织 tHb 应在 20~120 µM。
    # spectral_residual：μa 谱的拟合残差（cm⁻¹）。
    #   ⚠️ 只有 `spectral_dof > 0` 时才有自检意义。当前两波长 dof=0，
    #      残差恒为零；恢复到 ≥3 波长后自动生效。
    #   ⚠️ 自解散射分支下恒为 NaN —— 那条路的 mua 是由拟合浓度**重建**的
    #      （见 fit_scattering_and_toi 末尾的 `_unpack`），再去拟合它残差
    #      必然为零，不构成自检。真正的残差在 least_squares 的斜率域里。
    c_hbo: float = np.nan
    c_hbr: float = np.nan
    spectral_residual: np.ndarray | None = None
    spectral_dof: int = 0

    @property
    def valid(self) -> bool:
        return bool(np.isfinite(self.toi)) and not self.flags

    @property
    def thb_uM(self) -> float:
        """总血红蛋白，µM。NaN 表示未解出。"""
        if not (np.isfinite(self.c_hbo) and np.isfinite(self.c_hbr)):
            return float("nan")
        return (self.c_hbo + self.c_hbr) * 1e6

    @property
    def spectral_resid_rms(self) -> float:
        """μa 谱残差 RMS（cm⁻¹）。dof=0 时恒为 0，**不代表拟合好**。"""
        if self.spectral_residual is None:
            return float("nan")
        r = np.asarray(self.spectral_residual, float)
        return float(np.sqrt(np.mean(r ** 2))) if r.size else float("nan")


# ---------------------------------------------------------------------------
# 预处理
# ---------------------------------------------------------------------------
def to_attenuation(intensities: np.ndarray,
                   dark_level: np.ndarray | None = None) -> np.ndarray:
    """光强 → 衰减 A = −log10(I)。

    intensities: (n_wl, n_rho) 原始 ADC 计数
    dark_level : 同形状的暗电平，None 时不扣

    ⚠️ 与 MBLL 的 ΔOD 不同：这里不取时间比值，A 的绝对值参与斜率拟合。
       常数项（光源功率、共同耦合）会在 Σw=0 的斜率中约掉，不必扣除。
    """
    I = np.asarray(intensities, dtype=float)
    if dark_level is not None:
        I = I - np.asarray(dark_level, dtype=float)
    I = np.where(I > 0, I, np.nan)
    return -np.log10(I)


def fit_slopes(A: np.ndarray, rho: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """对每个波长，把 A(ρ) 拟合成直线，返回 (斜率, 残差RMS)。

    ⚠️ A(ρ) 严格说不是直线 —— 工作方程 ∂A/∂ρ = (μeff + 2/ρ)/ln10 中的 2/ρ 项
       使局部斜率随 ρ 变化（3→5cm 变 10.5%）。直线是渐近近似，残差不为零。

    ⭐ 残差是**线性度自检**：理论值 ≈0.018 OD 且几乎不随光学参数变化。
       实测远大于此 → 某探测器耦合异常 / 增益不对 / 有串扰。
       2 个探测器时残差恒为零，这正是必须 ≥3 点的原因。
    """
    A = np.atleast_2d(A)
    n_wl = A.shape[0]
    slope = np.full(n_wl, np.nan)
    resid = np.full(n_wl, np.nan)
    for i in range(n_wl):
        y = A[i]
        if np.any(~np.isfinite(y)):
            continue
        p = np.polyfit(rho, y, 1)
        slope[i] = p[0]
        resid[i] = float(np.sqrt(np.mean((y - np.polyval(p, rho)) ** 2)))
    return slope, resid


def slope_to_mueff(slope: np.ndarray, rho: np.ndarray) -> np.ndarray:
    """SRS 工作方程：μeff = ln10·(∂A/∂ρ) − 2/ρ̄。

    ⚠️ 2/ρ̄ 用平均距离代表，这是渐近近似引入的系统偏差来源
       （实测使解出的 μs′ 谱指数恒偏 +0.10），由体模标定吸收。
    """
    return LN10 * np.asarray(slope, dtype=float) - 2.0 / float(np.mean(rho))


# ---------------------------------------------------------------------------
# 反演
# ---------------------------------------------------------------------------
def musp_power_law(wavelengths: np.ndarray, a_800: float, b: float) -> np.ndarray:
    """μs′(λ) = a·(λ/800)^(−b)。"""
    return a_800 * (np.asarray(wavelengths, dtype=float) / 800.0) ** (-b)


def mua_from_mueff(mueff: np.ndarray, musp: np.ndarray) -> np.ndarray:
    """μa = μeff² / (3·μs′)。μeff ≤ 0 时置 NaN（信号不足或斜率异常）。"""
    mueff = np.asarray(mueff, dtype=float)
    musp = np.asarray(musp, dtype=float)
    return np.where(mueff > 0, mueff ** 2 / (3.0 * musp), np.nan)


@dataclass(frozen=True)
class HbFit:
    """μa 谱 → 血红蛋白浓度的完整分解。

    2026-10-22 新增。此前 `mua_to_toi` 只把 TOI 和浓度返回出去，
    **拟合残差被 `*_` 丢弃** —— 而那正是光谱自检。
    """

    toi: float                  # 100·HbO/(HbO+HbR)，%
    c_hbo: float                # mol/L
    c_hbr: float                # mol/L
    residual: np.ndarray        # 各波长的 μa 拟合残差，cm⁻¹
    dof: int                    # 自由度 = 波长数 − 2

    @property
    def thb(self) -> float:
        """总血红蛋白，µM。组织典型 40~80 µM —— 越界即为物理判据失败。"""
        return (self.c_hbo + self.c_hbr) * 1e6

    @property
    def resid_rms(self) -> float:
        r = np.asarray(self.residual, float)
        return float(np.sqrt(np.mean(r ** 2))) if r.size else float("nan")

    @property
    def resid_rel(self) -> float:
        """残差相对 μa 量级。dof=0 时恒为 0，无自检意义。"""
        r = np.asarray(self.residual, float)
        return float("nan") if r.size == 0 else self.resid_rms


def mua_to_hb(mua: np.ndarray, wavelengths: np.ndarray,
              table: str = "wray") -> HbFit:
    """由各波长 μa 最小二乘解出 HbO/HbR，并**保留残差**。

    ⚠️ **dof = 波长数 − 2。dof = 0 时残差恒为零，不是「拟合完美」。**
       当前在用两波长（670/770），矩阵是 2×2 可逆方阵，任何输入
       （包括纯噪声）都会被映射成一对浓度，残差为零 —— 这正是
       `docs/双解码器架构.md` R1 说的「零自检」。恢复到 ≥3 波长后，
       本残差自动变成真正的光谱自检，无需改代码。
    """
    mua = np.asarray(mua, dtype=float)
    wl = list(map(float, np.atleast_1d(wavelengths)))
    dof = max(len(wl) - 2, 0)
    if mua.size == 0 or np.any(~np.isfinite(mua)):
        return HbFit(np.nan, np.nan, np.nan, np.full(len(wl), np.nan), dof)
    E = hemoglobin_extinctions(wl, table)                 # 1/(M·cm)
    A = E * LN10
    c, *_ = np.linalg.lstsq(A, mua, rcond=None)
    # ⚠️ 不用 lstsq 返回的 residuals：它在**欠定或恰定**时是空数组，
    #    而我们恰恰工作在恰定（2 波长）状态。显式算，口径才一致。
    resid = mua - A @ c
    total = c[0] + c[1]
    toi = 100.0 * c[0] / total if total != 0 else np.nan
    return HbFit(float(toi), float(c[0]), float(c[1]), resid, dof)


def mua_to_toi(mua: np.ndarray, wavelengths: np.ndarray,
               table: str = "wray") -> tuple[float, np.ndarray]:
    """由各波长 μa 解出 (c_HbO, c_HbR)，返回 (TOI%, 浓度)。

    ⚠️ ε 表的选择直接进结果，且**没有退路**：
       MBLL 送检时两侧用同一张表可抵消，SRS 出绝对值，表错了就是错了。
       实测 wray 与 Prahl 在 700nm 的 HbO 差 44.6%，导致 TOI 差 4~6 %O₂。

    本函数是 `mua_to_hb` 的薄包装，**签名刻意保持不变** ——
    既有调用方（run_srs_v1、tools/ 下多个脚本）用的是 2 元组解包或 `[0]`。
    要残差请直接用 `mua_to_hb`。
    """
    f = mua_to_hb(mua, wavelengths, table)
    if not np.isfinite(f.c_hbo):
        return np.nan, np.full(2, np.nan)
    return f.toi, np.array([f.c_hbo, f.c_hbr], dtype=float)


def fit_scattering_and_toi(slope: np.ndarray, rho: np.ndarray,
                           wavelengths: np.ndarray,
                           a_init: float = MUSP_REF_800,
                           b_prior: float = MUSP_EXPONENT,
                           b_bounds: tuple[float, float] = B_BOUNDS,
                           b_weight: float = 0.0,
                           table: str = "wray") -> tuple[float, float, np.ndarray]:
    """四参数拟合：由五波长斜率同时解出 (c_HbO, c_HbR, a, b)，不预设 μs′ 谱指数。

    五波长给出 5 个 μeff，未知数 4 个，超定。相对固定假设的收益：

        固定假设 b   真实 b 在 0.6~1.4 变化时 TOI 波动 7.40 %O₂
        自解 b       同上条件下波动仅         0.03 %O₂

    ⚠️ 代价：多一个自由参数也吸收模型误差。μs′ 偏离幂律时自解反而更差
       （实测 4% 正弦畸变：自解 +3.79 vs 固定 −0.21）。b_bounds 与 b_weight
       用于把 b 约束在生理区间；不加约束时 b 会在畸变下顶到边界。

    ⚠️ 三波长（4 未知 / 3 方程）欠定，本函数需 ≥4 波长。

    ⚠️⚠️ **在真实在体数据上实测失效**：b 被推到上界，TOI 偏高约 +5 %O2。
       原因是水/脂质吸收未建模。仅在已确认无未建模吸收体的场合使用
       （如纯血红蛋白+Intralipid 体模），或先补齐吸收体模型。
       详见 compute_toi 的说明与文件末「验证状态」。

    返回 (TOI%, 解出的 b, μa)。
    """
    from scipy.optimize import least_squares

    wl = np.asarray(wavelengths, dtype=float)
    obs = np.asarray(slope, dtype=float)
    if wl.size < 4:
        raise ValueError("自解谱指数需 ≥4 个波长（4 未知数）")
    if np.any(~np.isfinite(obs)):
        return np.nan, np.nan, np.full(wl.size, np.nan)

    E = hemoglobin_extinctions(list(map(float, wl)), table)
    inv_rho = 2.0 / float(np.mean(rho))

    # 参数归一化到 O(1)。浓度(1e-4)与散射(1e1)相差 1e5，不归一化时
    # least_squares 会在初值处直接判定收敛（实测静默停在起点）。
    def _unpack(p):
        mua = (E[:, 0] * p[0] * 1e-4 + E[:, 1] * p[1] * 1e-4) * LN10
        musp = p[2] * 10.0 * (wl / 800.0) ** (-p[3])
        return mua, musp

    def _resid(p):
        mua, musp = _unpack(p)
        pred = (np.sqrt(3.0 * mua * (mua + musp)) + inv_rho) / LN10
        r = (pred - obs) * 1e3
        if b_weight > 0:
            r = np.append(r, np.sqrt(b_weight) * (p[3] - b_prior))
        return r

    p0 = [0.42, 0.18, a_init / 10.0, b_prior]
    lo = [0.0, 0.0, 0.1, b_bounds[0]]
    hi = [3.0, 3.0, 4.0, b_bounds[1]]
    try:
        res = least_squares(_resid, p0, bounds=(lo, hi),
                            xtol=1e-14, ftol=1e-14, gtol=1e-14)
    except Exception:
        return np.nan, np.nan, np.full(wl.size, np.nan)

    c_hbo, c_hbr = res.x[0] * 1e-4, res.x[1] * 1e-4
    total = c_hbo + c_hbr
    mua, _ = _unpack(res.x)
    return (100.0 * c_hbo / total if total != 0 else np.nan), float(res.x[3]), mua


# ---------------------------------------------------------------------------
# 主入口
# ---------------------------------------------------------------------------
def compute_toi(intensities: np.ndarray, geom: SrsGeometry,
                solve_scattering: bool = False,
                table: str = "wray") -> SrsResult:
    """单个时刻的 SRS 反演。

    intensities      : (n_wl, n_rho) 长距探测器的原始光强
    solve_scattering : True 走四参数自解谱指数；False（默认）用标称 μs′

    ⚠️ **默认关闭自解谱指数。** 在真实在体数据上（10 人前额时域 DOS）实测发现：
       自解版把 b 全部推到上界 1.600（真实 b 为 0.538~1.303），TOI 系统偏高 +5.18 %O2；
       固定假设版偏差仅 −0.47 ± 1.78 %O2。
       根因不是 μs′ 偏离幂律（实测幂律残差仅 0.34~0.94%），而是**水与脂质吸收未建模**
       （纯血红蛋白模型对实测 μa 的残差 2.3~4.2%，970nm 处水峰使 μa 从 0.157 升到 0.377）——
       多出的自由参数把这部分未建模吸收错误地吸进了 b。
       仿真条件下自解版优于固定假设，真实组织上相反。"""
    wl = geom.wavelengths_nm
    rho = geom.rho_cm
    flags: list[str] = []
    notes: list[str] = []

    A = to_attenuation(intensities, geom.dark_level)
    slope, resid = fit_slopes(A, rho)

    # 增益修正：体模矩阵标定的产物。斜率只受一个线性组合影响，
    # 故每波长只需一个数，五波长共 5 个，不是 n_wl × n_rho 个。
    if geom.gain_correction is not None:
        slope = slope - np.asarray(geom.gain_correction, dtype=float)

    # 质量检查
    I = np.asarray(intensities, dtype=float)
    if np.nanmin(I[:, -1]) < MIN_COUNTS:
        flags.append("最远探测器计数不足")
    # 2 个探测器时残差恒为零，空间自检不可用 —— 必须显式标出来，
    # 否则 residual=0 会被误读成「拟合极好」。
    # ⚠️ 2026-10-22 修正措辞：原文是「请依赖光谱残差」，但当前只有两个波长，
    #    光谱残差同样恒为零（实测 1.1e-17，即机器精度）。两条自检**都没有**，
    #    这句话原样留着会把人引到一条不存在的退路上。
    #    真正的退路是下面的物理判据（tHb / TOI 越界 / 浓度非负）。
    if rho.size < 3:
        if wl.size < 3:
            notes.append(f"仅 {rho.size} 长距 + {wl.size} 波长："
                         "空间与光谱自检**均不可用**，只能依赖物理判据")
        else:
            notes.append(f"仅 {rho.size} 个长距：无空间残差自检，"
                         "请依赖光谱残差")
    elif np.nanmax(resid) > SLOPE_RESIDUAL_MAX:
        flags.append(f"斜率拟合残差 {np.nanmax(resid):.4f} 超限（理论 ≈0.018）")

    mueff = slope_to_mueff(slope, rho)
    if np.any(mueff <= 0):
        flags.append("μeff 非正，信号不足或斜率异常")

    if solve_scattering and wl.size >= 4:
        toi_raw, b_fit, mua = fit_scattering_and_toi(slope, rho, wl, table=table)
        # b 顶到约束边界 = 模型与数据不匹配的信号（典型原因：未建模的吸收体）
        if np.isfinite(b_fit) and min(abs(b_fit - B_BOUNDS[0]),
                                      abs(b_fit - B_BOUNDS[1])) < 1e-3:
            flags.append(f"自解谱指数 b={b_fit:.3f} 顶到边界，模型可能不匹配")
        # 浓度仍要解出来（物理判据要用），但 **残差置 NaN**：
        # 这条路的 mua 是由拟合浓度重建的，再拟合必然零残差，不是自检。
        hb = mua_to_hb(mua, wl, table)
        hb = HbFit(hb.toi, hb.c_hbo, hb.c_hbr,
                   np.full(wl.size, np.nan), 0)
    else:
        b_fit = MUSP_EXPONENT
        musp = musp_power_law(wl, MUSP_REF_800, b_fit)
        mua = mua_from_mueff(mueff, musp)
        hb = mua_to_hb(mua, wl, table)      # 这条路的残差**是**真正的光谱自检
        toi_raw = hb.toi

    # 物理判据：不需要任何外部参考，只靠生理学。组织 tHb 应在 20~120 µM。
    # ⚠️ 这是当前「零自检」状态下**唯一**还能抓到异常的东西，别省。
    thb = hb.thb
    if np.isfinite(thb) and not (THB_MIN_uM <= thb <= THB_MAX_uM):
        flags.append(f"tHb {thb:.1f} µM 越界（生理 {THB_MIN_uM:.0f}~{THB_MAX_uM:.0f}）")
    if np.isfinite(hb.c_hbo) and (hb.c_hbo < 0 or hb.c_hbr < 0):
        flags.append("解出的 HbO/HbR 有负值，模型与数据不符")
    if np.isfinite(toi_raw) and not (0.0 <= toi_raw <= 100.0):
        flags.append(f"TOI_raw {toi_raw:.1f}% 越界（物理上限 0~100）")
    if hb.dof > 0 and hb.resid_rms > SPECTRAL_RESIDUAL_MAX:
        flags.append(f"光谱残差 {hb.resid_rms:.4f} 超限（限 {SPECTRAL_RESIDUAL_MAX}）")

    toi = geom.k * toi_raw + geom.b if np.isfinite(toi_raw) else np.nan
    return SrsResult(toi=toi, toi_raw=toi_raw, mua=mua, mueff=mueff,
                     slope=slope, residual=resid, b_fitted=b_fit,
                     flags=flags, config_notes=notes,
                     c_hbo=hb.c_hbo, c_hbr=hb.c_hbr,
                     spectral_residual=hb.residual, spectral_dof=hb.dof)


def compute_toi_series(intensities: np.ndarray, geom: SrsGeometry,
                       **kwargs) -> tuple[np.ndarray, list[SrsResult]]:
    """时间序列版本。intensities: (n_wl, n_rho, n_time)。

    ⚠️ 每个时刻**独立**反演，不依赖任何时间基线 —— 这正是 SRS 能给绝对值、
       而 MBLL 只能给相对量的根本原因。删掉第一帧不影响其余任何一帧。
    """
    I = np.asarray(intensities, dtype=float)
    n_t = I.shape[2]
    out = [compute_toi(I[:, :, t], geom, **kwargs) for t in range(n_t)]
    return np.array([r.toi for r in out]), out


# ---------------------------------------------------------------------------
# 短距质量指标（不参与斜率）
# ---------------------------------------------------------------------------
def scalp_confidence(long_toi: np.ndarray, short_signal: np.ndarray,
                     window: int = 30) -> np.ndarray:
    """用短距通道判断长距 TOI 变化的可信度。

    短距（~1cm）主要采样头皮，不参与 SRS 斜率计算，作独立监测：

        长距↓ + 短距不动  →  真的是脑           （置信度高）
        长距↓ + 短距↓     →  全身性，需结合 SpO2 （中）
        长距弱↓ + 短距↑   →  ⚠️ 头皮反向可能掩盖了脑的下降（低）

    ⚠️ 抵消门槛：Δ头皮/Δ脑 = (1−f)/f。f=25% 时需头皮反向动 3 倍；
       f=50% 时只需 1 倍。文献实测纯颅外变化可使商用设备读数动 6.8~16.6 点，
       故此风险真实存在。

    返回逐点的相关系数（滑窗），越接近 +1 表示长距变化越可能来自浅层。
    """
    L = np.asarray(long_toi, dtype=float)
    S = np.asarray(short_signal, dtype=float)
    n = min(L.size, S.size)
    out = np.full(n, np.nan)
    for i in range(window, n):
        a, b = L[i - window:i], S[i - window:i]
        if np.std(a) > 0 and np.std(b) > 0:
            out[i] = float(np.corrcoef(a, b)[0, 1])
    return out


# ---------------------------------------------------------------------------
# 派生相对量（2026-10-23 新增）
# ---------------------------------------------------------------------------
# 为什么从 SRS 的绝对浓度减基线，而不是跑 MBLL：
#
#   MBLL 要 **300 秒**才出第一个点（高通 0.01 Hz 的起步门槛），而且依赖
#   `scipy.signal` 的零相位滤波 —— 移植到安卓端很麻烦。
#   SRS 的 `compute_toi` **本来就逐点算出 c_hbo / c_hbr / tHb**，减一条
#   基线就是全部需要的相对量：延迟只有 SRS 自身的 N/2 个周期，而且
#   **纯算术，没有滤波、没有 scipy**。
#
#   这也是竞品 BRS-100 的做法（说明书 p17/p31：输出 CHb/CHbO2/CtHb 绝对浓度
#   + 基线 + 与基线的差值比率 + AUC，整机不含 MBLL）。
#
# 代价（实测，体模静置 141147）：
#   ΔHbO 噪声底 1.486 µM，是 MBLL 带通路径（0.124 µM）的 12 倍。
#   但真实生理信号远大于此 —— 阻断实验实测 ΔtHb 67 µM、ΔHbO 63 µM，
#   信噪比 42~70 倍，绰绰有余。各量的噪声底与阻断信噪比：
#
#       量      噪声底        阻断信号    信噪比
#       ΔtHb    0.955 µM     67.1 µM      70×
#       ΔTHI    0.0205       0.966        47×   （比值口径；原表按 % 记为 2.05 / 96.6）
#       ΔHbO    1.486 µM     62.8 µM      42×
#       ΔHbR    0.741 µM      5.2 µM       7×
#       ΔTOI    1.868 %O₂     —           见下
#
#   ⚠️ ΔTOI 的噪声底 1.868 %O₂ 不算小。文献报道动脉阻断的去饱和速率是
#      6~16 %O₂/分钟（前臂 15mm 探头 −6.4、25mm −10.0、大鱼际 −12.5），
#      3 分钟可降 20~30 点 —— 那时信噪比 10~16 倍。但**小幅去饱和会被淹没**。
#
# ⚠️ 基线不抗漂移。SRS 浓度 63% 的噪声在 <0.01 Hz，而 SRS **不能高通**
#    （会毁掉绝对值）。所以界面必须提供「重设基线」，由操作者按需校准 ——
#    竞品也是这么做的（说明书 p32「点击后，系统重新校准基线值」）。


@dataclass(frozen=True)
class Baseline:
    """一次基线标定。**不可变** —— 重设就新建一个，不就地改。

    这样任何一个 `Derived` 都能追溯到它是相对哪一条基线算的。
    """

    t: float                 # 设定时刻（调用方的时钟）
    n: int                   # 参与平均的点数
    hbo: float               # µM
    hbr: float               # µM
    thb: float               # µM
    toi: float               # %
    # --- 2026-10-26 新增，均带默认值，既有调用不受影响 ---------------------
    source: str = "captured"     # auto=自动 / captured=按钮抓取 / manual=TOI 手填过
    low_signal: bool = False     # 攒的点里有「最远探测器计数不足」
    unstable: bool = False       # 自动基线等稳定超时后照设的

    @property
    def ok(self) -> bool:
        """tHb 为 0 时 THI 无定义 —— 那条基线不能用。"""
        return (self.n > 0 and np.isfinite(self.thb) and self.thb > 0
                and np.isfinite(self.hbo) and np.isfinite(self.toi))


def make_baseline(results, t: float = 0.0) -> Baseline | None:
    """由一串 `SrsResult` 求基线。非有限值自动跳过；一个都不剩则返回 None。

    ⚠️ 用算术均值而非中位数：调用方喂进来的应当是**已经过稳健聚合**的点
       （`robust.aggregate` 在更上游剔过尖峰了），这里再做一次中位数
       只会白白损失效率。若调用方没做剔除，责任在调用方。
    """
    hbo, hbr, thb, toi = [], [], [], []
    for r in results:
        if not (np.isfinite(r.toi) and np.isfinite(r.thb_uM)):
            continue
        hbo.append(r.c_hbo * 1e6); hbr.append(r.c_hbr * 1e6)
        thb.append(r.thb_uM); toi.append(r.toi)
    if not thb:
        return None
    return Baseline(t=float(t), n=len(thb),
                    hbo=float(np.mean(hbo)), hbr=float(np.mean(hbr)),
                    thb=float(np.mean(thb)), toi=float(np.mean(toi)))


@dataclass(frozen=True)
class Derived:
    """相对基线的派生量。全部是减法/除法，没有滤波。"""

    d_hbo: float             # µM
    d_hbr: float             # µM
    d_thb: float             # µM
    # THI：当前 tHb ÷ 基线 tHb，**基线处 = 1.00**。
    # 2026-10-26 由「% 基线（基线=100）」改成比值，对齐 MOC400 的读数口径 ——
    # 其界面实测 THI 2.83 / ΔTHI 1.83，差恰为 1.00，可见基线归一到 1
    # （与 NIRO 的 nTHI 同一约定）。
    thi: float
    d_thi: float             # THI − 1.00
    d_toi: float             # %O₂
    # ΔTOI 占基线的百分比。与 AUC 的相对警戒线、BRS-100 的「差值比率」同一口径。
    d_toi_rel: float = float("nan")


def derive(res: "SrsResult", bl: Baseline | None) -> Derived | None:
    """`SrsResult` + 基线 → 相对量。基线缺失或本次无效时返回 None。

    返回 None 而不是 NaN：调用方必须显式处理「还没设基线」这个状态，
    而不是把 NaN 画到曲线上。
    """
    if bl is None or not bl.ok:
        return None
    if not (np.isfinite(res.toi) and np.isfinite(res.thb_uM)):
        return None
    thb = res.thb_uM
    thi = thb / bl.thb                      # 基线处 = 1.00
    d_toi = res.toi - bl.toi
    return Derived(d_hbo=res.c_hbo * 1e6 - bl.hbo,
                   d_hbr=res.c_hbr * 1e6 - bl.hbr,
                   d_thb=thb - bl.thb,
                   thi=thi, d_thi=thi - 1.0,
                   d_toi=d_toi,
                   d_toi_rel=100.0 * d_toi / bl.toi if bl.toi > 0 else float("nan"))


# ---------------------------------------------------------------------------
# AUC 与基线状态机（2026-10-26 改造）
# ---------------------------------------------------------------------------
# AUC 警戒线 = 参考 TOI × (1 − AUC_REL_DROP)。
#
# 为什么不是「低于基线即累计」（2026-10-23 的旧实现）：那等于警戒线贴着基线，
#   噪声的负半边全被收进去、正半边不抵消，基线一偏还会线性累积。
#   实测体模（真实去饱和恒为 0）跑 10 分钟：141147 凭空攒 662 %·s、164708 攒 1013。
#   文献口径的警戒线**永远严格低于基线**（绝对 50%，或相对基线下 10/20/30/50%），
#   那段间隔本身就是死区（Slater 2009 PubMed 19101265；PMC8437017）。
#
# 为什么是相对 20% 而不是减 20 个点：文献相对口径里有「低于基线 50%」，
#   只能是相对值；BRS-100 说明书 p31 把这个量叫「差值**比率**」。
#   20% 远大于 ΔTOI 噪声（σ 0.07~2.42），所以不会凭空累积。
AUC_REL_DROP = 0.20


class AucAccumulator:
    """去饱和负荷：∫ max(警戒线 − TOI, 0) dt。内部单位 %O₂·s，另给 %·min。

    **自带参考点** `ref_toi`，与 BL 基线解耦（MOC400 的设计）：
    BL 随时可重抓、可手填，AUC 是整台手术的累计账，不该跟着被清掉。
    参考点只在 `reset(ref_toi)` 时设定；`ref_toi` 为 None 时不累计。

    ⚠️ 用实际时间差累加，不假设固定采样间隔（丢帧时仍正确）。
       两次之间间隔过大时跳过，避免把一段空洞算成持续去饱和。
    """

    MAX_GAP_S = 10.0

    def __init__(self, rel_drop: float = AUC_REL_DROP) -> None:
        self.rel_drop = float(rel_drop)
        self.ref_toi: float | None = None
        self.value = 0.0         # %O₂·s
        self.seconds = 0.0       # 低于警戒线的累计时长
        self._t_prev: float | None = None

    @property
    def threshold(self) -> float | None:
        """警戒线，%。没有参考点时为 None。"""
        if self.ref_toi is None:
            return None
        return self.ref_toi * (1.0 - self.rel_drop)

    @property
    def value_min(self) -> float:
        """%·min —— 相对口径文献的截断值（如 >150 min%）用这个单位。"""
        return self.value / 60.0

    def push(self, t: float, toi: float) -> float:
        """喂一个时刻的**绝对** TOI，返回当前累计值（%·s）。"""
        thr = self.threshold
        if self._t_prev is not None and thr is not None:
            dt = t - self._t_prev
            if 0.0 < dt <= self.MAX_GAP_S and np.isfinite(toi) and toi < thr:
                self.value += (thr - toi) * dt
                self.seconds += dt
        self._t_prev = float(t)
        return self.value

    def reset(self, ref_toi: float | None = None) -> None:
        """清零，并把参考点设为 `ref_toi`（None = 暂无参考点，不累计）。"""
        self.ref_toi = None if ref_toi is None else float(ref_toi)
        self.value = 0.0
        self.seconds = 0.0
        self._t_prev = None


# 攒基线时**挡掉**的 flag：物理上不可能的值。按 compute_toi 产出的措辞匹配
# （见上方 compute_toi 里的 flags.append）。措辞改了，t18 会报。
_BASELINE_BLOCKING = ("TOI_raw", "负值", "μeff 非正")
_LOW_SIGNAL_KEY = "计数不足"


def baseline_reject_reason(res: "SrsResult") -> str:
    """该点能否进基线：能 → 返回空串；不能 → 返回原因（给状态行显示）。

    ⚠️ 两类 flag **刻意不挡**：
       - tHb 越界：体模上恒触发（111030 实测 tHb 12.31 µM，低于 20 下限，
         237/237 点报警），挡了体模就永远设不上基线。
       - 最远探测器计数不足：前额透颅骨后 670/4cm 可能一直低于 MIN_COUNTS
         （**未测**），挡了在头上可能永远设不上。改为收下、把基线标 low_signal。
    """
    if not (np.isfinite(res.toi) and np.isfinite(res.thb_uM)):
        return "反演无效"
    for f in res.flags:
        if any(k in f for k in _BASELINE_BLOCKING):
            return f
    return ""


# 「贴稳了」的判据。**只做指示灯，不拦基线**（见 BaselineManager 的说明）。
# 依据（实测，8 份归档，与 online 同一条链路，N=20）：
#
#   记录      类型          770/3cm 60s 变化 p50   p90    第一个可判值
#   111030   A2 体模静止        0.1%            0.2%     0.2%
#   141147   体模低光           0.2%            0.4%     0.0%
#   164708   体模低光           0.3%            0.8%     1.4%
#   131237   在体刚贴上        10.0%           11.1%     6.2%
#   150943   手臂勒紧           8.1%           13.5%     1.1%
#
# 静止体模 p90 ≤ 0.8%，刚贴上 ≥ 6.2% → 阈值 2% 两边各约 3 倍余量。
# 只看**光强最高的一路**：「四路取最大」与「四路共模」都会混入低光 670/4cm
# 自身的噪声（低光体模上 p90 撑到 3~20%），和「没贴稳」分不开。
#
# **光强稳 ≠ TOI 稳**（2026-10-26 用真实归档截图时发现）：TOI 由两波长的
# **比值**决定，单一路光强平着，TOI 照样可以大变。103952 里 TOI 从 80 掉到
# 52 又爬回 75，光强判据在 t=129 s 就放行，基线设在爬升途中（65），此后整段
# ΔTOI 带 +10 的偏移。所以再加一条：过去 60 s TOI 线性总变化 < 5 个点。
#
#   记录      类型          TOI 60s 变化 p50   p90     关键时刻
#   111030   A2 体模静止       0.08       0.20
#   141147   体模低光          1.99       4.20
#   164708   体模低光          2.79       9.19   ← 纯噪声也会超 5，等得久些 / 走超时
#   150943   手臂勒紧          1.48       2.42
#   131237   在体刚贴上        1.54       1.81   （光强判据已挡）
#   103952   在体              5.45      24.78   t=129: 10.4  t=157: 19.9（爬升中）
#
# ⚠️ 全是手臂和体模，**前额未测**。
SETTLE_WIN_S = 60.0
SETTLE_MAX_REL = 0.02
SETTLE_MAX_TOI_PTS = 5.0
SETTLE_MIN_PTS = 10
# 2026-10-26 第二轮删掉 SETTLE_TIMEOUT_S：判据不再拦基线，没有「等到超时」这回事。

# 一次基线攒多少个点。20 个点 ≈ 30 秒（扫描率 0.657 周期/秒）。
# 基线的离散会加到此后**每一个**相对量上，所以它要明显小于单点噪声：
# 20 点把 ΔHbO 的基线不确定度压到 1.486/√20 ≈ 0.33 µM。
BASELINE_N = 20


class SettleGate:
    """两条都满足才算「稳」：

    ① 光强最高那一路，过去 `win_s` 秒 ln(I) 线性拟合的总变化 < `max_rel`（探头贴稳了）
    ② 过去 `win_s` 秒 TOI 线性拟合的总变化 < `max_toi_pts` 个点（生理状态平稳）

    `metric` 为 ①（0.02 = 2%），`metric_toi` 为 ②（点）。窗口没攒满时为 NaN。
    `push` 不给 `toi` 时只用 ①（单测与旧调用）。
    """

    def __init__(self, win_s: float = SETTLE_WIN_S,
                 max_rel: float = SETTLE_MAX_REL,
                 min_pts: int = SETTLE_MIN_PTS,
                 max_toi_pts: float = SETTLE_MAX_TOI_PTS) -> None:
        self.win_s = float(win_s)
        self.max_rel = float(max_rel)
        self.min_pts = int(min_pts)
        self.max_toi_pts = float(max_toi_pts)
        self.reset()

    def reset(self) -> None:
        self._t: deque = deque()
        self._I: deque = deque()
        self._y: deque = deque()
        self._t_first: float | None = None
        self._use_toi = False
        self.metric = float("nan")
        self.metric_toi = float("nan")

    def push(self, t: float, intensities, toi: float | None = None) -> float:
        I = np.asarray(intensities, dtype=float).ravel()
        if I.size == 0 or not np.all(np.isfinite(I)) or np.any(I <= 0):
            return self.metric          # 取不了对数的点不进窗，也不改判据
        if toi is not None:
            self._use_toi = True
        if self._t_first is None:
            self._t_first = float(t)
        self._t.append(float(t))
        self._I.append(I)
        self._y.append(float("nan") if toi is None else float(toi))
        while self._t and self._t[0] <= t - self.win_s:
            self._t.popleft()
            self._I.popleft()
            self._y.popleft()
        self.metric = self.metric_toi = float("nan")
        if t - self._t_first < self.win_s or len(self._t) < self.min_pts:
            return self.metric
        T = np.fromiter(self._t, float, len(self._t))
        M = np.vstack(self._I)
        j = int(np.argmax(M.mean(axis=0)))
        k = np.polyfit(T, np.log(M[:, j]), 1)[0]
        self.metric = float(abs(np.expm1(k * self.win_s)))
        Y = np.fromiter(self._y, float, len(self._y))
        ok = np.isfinite(Y)
        if ok.sum() >= self.min_pts:
            self.metric_toi = float(abs(np.polyfit(T[ok], Y[ok], 1)[0]
                                        * self.win_s))
        return self.metric

    @property
    def ready(self) -> bool:
        return bool(np.isfinite(self.metric))

    @property
    def stable(self) -> bool:
        if not (self.ready and self.metric < self.max_rel):
            return False
        if not self._use_toi:
            return True
        # 给过 TOI 却算不出（有效 TOI 点太少）→ 信号本身不行，不算稳
        return bool(np.isfinite(self.metric_toi)
                    and self.metric_toi < self.max_toi_pts)


@dataclass(frozen=True)
class BaselineState:
    """给界面显示用的快照。界面只读它，不做判断。"""

    phase: str               # collecting / set
    n_have: int              # 本轮已攒的点
    n_need: int
    n_skipped: int           # 本轮因 flag 跳过的点
    last_skip: str           # 最近一次跳过的原因
    gate_metric: float       # 光强判据当前值（0.02 = 2%）
    gate_metric_toi: float   # TOI 判据当前值（60 s 内变化几个点）
    collecting_mode: str     # 正在攒的这一轮的来源：auto / captured
    suggest_recapture: bool  # 基线标了未稳定，而现在信号稳了 → 建议重抓
    gate_ready: bool         # 判据窗攒满了没（没满就还判不了）
    gate_stable: bool        # **当前**信号稳不稳（实时，不是基线那一刻的）


class BaselineManager:
    """BL 基线与 AUC 的全部状态。界面只调方法、读 `state`，**不做任何判断** ——
    这样安卓端拿到的是同一份行为，自检和归档回放也能直接测它。

        reset()（连接 / 清屏）
           ▼
        采集中 n/N（只收 baseline_reject_reason 为空的点）
           ▼
        已设 ── 首次已设且 AUC 无参考点 → AUC 继承 BL 的 TOI
           ├ capture()         BL 立即作废 → 重新采集（captured）
           ├ set_manual_toi(x) 只改 BL 的 TOI，source=manual；AUC 不动
           └ reset_auc()       AUC 清零，参考点 = 当前 BL 的 TOI；BL 不动

    **AUC 在任何阶段都照常累计**（只认自己的参考点），重抓 BL 的那 30 秒不断档。

    ⚠️ **稳定判据是指示灯，不是闸门**（2026-10-26 第二轮改）。它不拦着基线，
       只在定基线那一刻被读一次，结果写进 `Baseline.unstable`。

       第一轮设计成「等稳定才设」，实测代价远大于收益：最快也要 120 s，
       8 份归档里 3 份全程设不上；而等待买到的精度差异 ≤0.5 点，
       在 ΔTOI 噪声底（1.2~1.9）以下。竞品也不等 —— BRS-100 说明书 p34
       「开始监测后，系统会自动计算基线」。详见 docs/online优化计划_第二轮.md。

       基线标了 unstable、而判据后来通过了 → `state.suggest_recapture` 为真，
       由界面提示操作者重抓。**不自动改基线** —— 监护仪的参考点自己移动是坏性质
       （实测自动精化在 8 份里只触发 4 份，改动 ≤1.7 点，精度 2 好 1 坏 1 平）。

    `t` 请传**数据时刻**（帧的 t_host），不要传反演那一刻的时钟 ——
    前者才是光被测到的时刻，回放与自检也才能复现。

    ⚠️ 只喂**满窗**的点：窗口没攒满时 `robust.aggregate` 因 MIN_N=4 不剔尖峰，
       实测用部分窗的点算基线，103952 差 −23.48 点。由调用方保证（online 在
       `n < need` 时不调本方法）。
    """

    COLLECTING, SET = "collecting", "set"

    def __init__(self, n: int = BASELINE_N,
                 gate: SettleGate | None = None,
                 auc: AucAccumulator | None = None) -> None:
        self.n = int(n)
        self.gate = gate if gate is not None else SettleGate()
        self.auc = auc if auc is not None else AucAccumulator()
        self.reset()

    # -- 操作 --------------------------------------------------------------
    def reset(self) -> None:
        """连接 / 清屏：基线、AUC、稳定判据全部从头来。"""
        self.baseline: Baseline | None = None
        self.last_derived: Derived | None = None
        self._t_last: float | None = None
        self.gate.reset()
        self.auc.reset(None)
        self._begin("auto")

    def capture(self) -> None:
        """「抓取当前」：立即重新攒一轮基线。

        旧基线立即作废 —— 攒的这 30 秒里不该继续用旧参考点出数。
        """
        self.baseline = None
        self._begin("captured")

    def set_manual_toi(self, toi: float) -> bool:
        """手填 TOI 基线。只改 TOI，浓度基线不动（浓度没法让人手填）。

        还没有浓度基线时拒绝并返回 False —— ΔHbO/ΔHbR/ΔtHb/THI 需要它。
        """
        if self.baseline is None or not np.isfinite(toi) or not 0.0 < toi <= 100.0:
            return False
        self.baseline = replace(self.baseline, toi=float(toi), source="manual")
        return True

    def reset_auc(self) -> None:
        """「AUC复位」：清零，参考点取当前 BL 的 TOI。BL 不动。"""
        self.auc.reset(self.baseline.toi if self.baseline is not None else None)

    # -- 数据 --------------------------------------------------------------
    def push(self, t: float, res: "SrsResult", intensities) -> Derived | None:
        """喂一次**满窗**反演结果。返回相对量；还没有基线时返回 None。"""
        t = float(t)
        self._t_last = t
        self.gate.push(t, intensities, res.toi)

        if self.phase == self.COLLECTING:
            self._collect(t, res)

        if np.isfinite(res.toi):
            self.auc.push(t, res.toi)
        d = derive(res, self.baseline) if self.phase == self.SET else None
        self.last_derived = d
        return d

    @property
    def state(self) -> BaselineState:
        return BaselineState(
            phase=self.phase, n_have=len(self._pending), n_need=self.n,
            n_skipped=self._n_skipped, last_skip=self._last_skip,
            gate_metric=self.gate.metric,
            gate_metric_toi=self.gate.metric_toi,
            collecting_mode=self._mode,
            suggest_recapture=bool(self.baseline is not None
                                   and self.baseline.unstable
                                   and self.gate.stable),
            gate_ready=self.gate.ready, gate_stable=self.gate.stable)

    # -- 内部 --------------------------------------------------------------
    def _begin(self, mode: str) -> None:
        self.phase = self.COLLECTING
        self._mode = mode
        self._pending: list = []
        self._low = False
        self._n_skipped = 0
        self._last_skip = ""

    def _collect(self, t: float, res: "SrsResult") -> None:
        why = baseline_reject_reason(res)
        if why:
            self._n_skipped += 1
            self._last_skip = why
            return
        self._pending.append(res)
        if any(_LOW_SIGNAL_KEY in f for f in res.flags):
            self._low = True
        if len(self._pending) < self.n:
            return
        bl = make_baseline(self._pending, t=t)
        if bl is None or not bl.ok:          # 收进来的点都有限，正常不会发生
            self._begin(self._mode)
            return
        # 判据在这一刻被读一次 —— 它是指示灯，不是闸门。
        #
        # ⚠️ 只有「判过且没通过」才标 unstable。**「还没判过」不标** ——
        #    基线在 ~59 s 设上，而判据窗是 60 s，此时必然 ready=False。
        #    若把没判过也算不稳，8 份归档会全部标红，就成了恒亮的告警灯
        #    （和 tHb 判据在体模上 237/237 报警是同一个毛病）。
        #    当前信号稳不稳由 `state.gate_*` 实时给，不冻在基线上。
        self.baseline = replace(bl, source=self._mode, low_signal=self._low,
                                unstable=self.gate.ready and not self.gate.stable)
        self.phase = self.SET
        self._pending = []
        if self.auc.ref_toi is None:
            self.auc.reset(self.baseline.toi)
