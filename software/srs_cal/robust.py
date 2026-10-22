# -*- coding: utf-8 -*-
"""稳健聚合 —— 把一组逐周期读数并成一个值，且不被尖峰带偏。

为什么不用均值：
    2026-10-22 的在体归档（COM16，104 秒，68 周期）实测，每个通道有
    **2~6% 的周期是尖峰**，幅度约正常值的 25 倍，且不按周期对齐、逐帧随机命中。
    `online.py` 默认每 10 个周期出一个 TOI 点，**10% 的窗口被尖峰污染**；
    一个 25 倍尖峰落进 10 周期的窗口，会把均值抬高约 2.5 倍，该点彻底作废。

为什么不用中位数：
    中位数在干净数据上要付效率代价。实测对比（同一份在体归档，窗口 N=10）：

        估计量      高斯对照代价   真实噪声代价   尖峰窗口中位偏差   最坏偏差
        mean          基准           基准           77.78%          2071%
        median       +18.1%         −4.1%            1.79%          33.8%
        trim10        +2.4%         −3.4%            1.30%        1208%
        trim20        +5.9%         −6.1%            1.17%          23.4%
        mad_mean      +5.6%         +0.4%            0.00%           9.1%

    （测法自验：纯高斯下 median/mean = 1.181，理论 1.253，N=10 时略低，合理。）

    trim10 最坏到 1208%，因为**一个窗口里有两个尖峰**时，两端各砍一个还剩一个。
    `mad_mean` 两项都最好：真实噪声上效率代价仅 0.4%，尖峰窗口偏差 0%。

判据取 5×MAD：
    高斯下 MAD×1.4826 ≈ σ，故 5×MAD ≈ 3.37σ，正常样本被误剔的概率约 0.08%；
    而实测尖峰是正常值的 25 倍，远在判据之外。两边都留足余量。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

MAD_K = 5.0          # 离群判据：|x − 中位| > MAD_K × MAD
MIN_N = 4            # 少于这么多样本时 MAD 不可靠，退回均值


@dataclass(frozen=True)
class Aggregate:
    """一次聚合的结果。`n_rejected` 供质量层使用，不要丢掉。"""

    value: float         # 稳健均值
    std: float           # **保留样本**的标准差（与 value 口径一致）
    n_used: int
    n_rejected: int

    @property
    def sem(self) -> float:
        """标准误。⚠️ 仍假定样本独立 —— 实测逐周期读数有 50~79% 的方差
        来自慢漂移，故此值偏乐观，只能当下界。"""
        return self.std / np.sqrt(self.n_used) if self.n_used > 1 else float("nan")

    @property
    def rel_std_pct(self) -> float:
        return 100.0 * self.std / abs(self.value) if self.value else float("nan")


def aggregate(values, k: float = MAD_K) -> Aggregate:
    """MAD 剔除离群后取均值。

    values : 一维序列（一个通道在若干周期上的读数）
    返回 `Aggregate`；全为非有限值时 value 为 nan。
    """
    v = np.asarray(values, dtype=float)
    v = v[np.isfinite(v)]
    n = v.size
    if n == 0:
        return Aggregate(float("nan"), float("nan"), 0, 0)
    if n < MIN_N:
        # 样本太少，MAD 估不准，强行剔除反而可能剔掉真值
        return Aggregate(float(v.mean()),
                         float(v.std(ddof=1)) if n > 1 else 0.0, n, 0)

    med = float(np.median(v))
    mad = float(np.median(np.abs(v - med)))
    # ⚠️ MAD 可能为 0（过半样本完全相同：饱和削顶、或信号远小于量化步长）。
    #    早先版本在此直接返回中位数并报告「剔除 0」—— 值是对的，但**报告不诚实**：
    #    它实际排除了尖峰却说没剔除，质量层因而看不见异常。
    #    改为给 MAD 一个**物理下限 1 个 ADC 计数**：读数是整数，MAD < 1
    #    本就意味着「样本几乎全同」，此时判据退化为「偏离中位 > 5 counts 即离群」，
    #    既能剔掉尖峰，又能如实计数。饱和削顶时所有样本都在 5 counts 内，
    #    n_rejected 仍为 0，不会误报。
    mad = max(mad, 1.0)

    keep = np.abs(v - med) <= k * mad
    kv = v[keep]
    if kv.size == 0:                       # 不该发生，兜底
        return Aggregate(med, float(v.std(ddof=1)), n, 0)
    return Aggregate(float(kv.mean()),
                     float(kv.std(ddof=1)) if kv.size > 1 else 0.0,
                     int(kv.size), int(n - kv.size))
