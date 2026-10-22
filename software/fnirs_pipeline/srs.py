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

from dataclasses import dataclass, field

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
