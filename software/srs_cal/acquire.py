# -*- coding: utf-8 -*-
"""体模标定数据采集 —— 12 块各一个按钮，结果存 xlsx 供 run_calibration 使用。

    python -m srs_cal.acquire                # 连真机
    python -m srs_cal.acquire --simulate     # 无硬件，用正演模型造数据排练
    python -m srs_cal.acquire --channel 2    # 指定采集通道（默认 1）

采集逻辑：点某块的按钮 → 连续收 N 帧（默认 200，约 20 个扫描周期）→
对每个「波长 × 距离」组合取均值、标准差、帧数。同一块可以反复采多次
（"take"），用来评估**重贴合变异** —— 这一项在实际中往往比块间差异更大，
而体模块本身的 holdout 完全测不到它。

⚠️ 950 nm 照常采集并存档，但不进 SRS 反演（组织侧水吸收占 ~40%，见
   hardware.py）。体模不含水，这一路在体模上是好的，存着备查。
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))

from srs_cal import hardware as hw           # noqa: E402
from srs_cal import phantom                  # noqa: E402
from srs_cal.robust import aggregate         # noqa: E402
from srs_cal.protocol import (                # noqa: E402
    BRIGHTNESS_MAX, BRIGHTNESS_MIN, CHANNEL_MAX, CHANNEL_MIN,
    CMD_START_CH2, CMD_STOP_CH1, DISTANCE_BY_CODE, EXPECTED_CHANNEL,
    BRIGHTNESS_DEFAULT, WAVELENGTH_BY_CODE, Frame, FrameStream,
    CMD_START, build_brightness, build_cmd, build_frame, format_frame,
    frames_per_cycle)

WLS = [WAVELENGTH_BY_CODE[c] for c in sorted(WAVELENGTH_BY_CODE)]     # 670..950
RHOS = [DISTANCE_BY_CODE[c] for c in sorted(DISTANCE_BY_CODE)]        # 3.0, 4.0
# 旧固件时代「人工遮死探头」采的那种暗，块名就叫 DARK。
# 新固件每周期自带暗时隙，软件不再**产生**这种记录，但仍要能**读**
# 旧 xlsx（run_srs_v1 会用到），所以常量保留。
DARK_CODE = "DARK"
# 「空测」：LED 正常亮，但探头下面**没有体模**（悬空，或放在黑面上）。
# 测的是光源经 PCB 板材/板面直接耦合到探测器的那条路 —— 它不经过介质，
# 而暗时隙是 LED 全灭时采的，看不见它，所以逐周期扣暗扣不掉这一项。
# 块名独立，且 run_srs_v1.load_measured 会跳过非体模块名，不会混进标定。
AIR_CODE = "AIR"
# 「手臂」：在体采集。和 AIR 一样不是体模，run_srs_v1 的加载器会跳过它。
#
# ⚠️ 体模的验收判据在这里**不适用**，所以 _add_row 对它走另一条分支：
#    · 白噪声度 r：体模上 r<0.6 判为漂移。在体时心搏（约 1 Hz）本身就让
#      相邻帧强相关，r 必然低 —— 那是真信号，不是故障。按体模标准会误报。
#    · 标准误限值 0.7%：那是按体模的静态噪声定的，在体时搏动一项就超了。
#    在体真正该看的是：4cm 够不够亮、**有没有心搏**。看得见心搏，才说明
#    探头确实耦合到了活体组织上 —— 这是目前最直接的一条在体有效性证据。
ARM_CODE = "ARM"
INVIVO_CODES = (ARM_CODE,)
# 心搏频带：40~180 bpm。搏动占比 = 该带内方差 / 总方差。
PULSE_BAND_HZ = (0.67, 3.0)
# 谱峰显著度（带内谱峰/全谱中位）的判定下限。自验：心搏 0.5% 振幅给
# 250~354，纯噪声给 4.8~5.7 —— 两者差约 50 倍，取 30 留足余量。
PULSE_PROM_MIN = 30.0
# 一个扫描周期的帧数：(1 暗槽 + 5 波长) × 2 距离。**单个通道**的。
# 多通道同采时串口上一周期是它的通道数倍，拿帧率换算周期率要用
# protocol.frames_per_cycle(活跃通道数)；而采集缓冲里的帧已按通道过滤过，
# 所以 _sync_frame_hint 的「每组合帧数」仍除这个常量，不乘通道数。
FRAMES_PER_CYCLE = (1 + len(WLS)) * len(RHOS)
# 实时表的配色阈值：扣暗后低于此值标红（srs.py 的 MIN_COUNTS 是 2000），
# 原始值高于此值标黄（疑似接近满量程）。
MIN_NET_COUNTS = 2000
# 满量程未经厂家确认，下面这个阈值是 2026-10-12 实测推出来的：
#
#   A2（最透的块）档位 130 → 150，950nm@3cm：
#     实测比 1.1473，等比应为 1.1538 —— 线性度 99.4%，无压缩迹象。
#     可用四路（670/770/850/950）线性度 97.6%~99.8%，而且**最亮的那路偏差
#     最小**；饱和压缩的形状应当相反，故判定未饱和。
#   该次逐帧 adc_raw 最大 2,883,734，最大值只出现 1 次、与次大值不相等
#   （削顶会出现大量重复的同一个值），进一步确认未撞顶。
#   暗读数为负（−56,230），故为有符号转换器；2,883,734 已超过 22 位有符号
#   上限 2,097,151，所以至少是 23 位有符号。
#
# 2026-10-13 更新：满量程**实测确认**了。手臂采集（标定采集_20261013_113257）
# 里出现削顶，削顶值恰为 8,388,607 = 2²³−1 —— **24 位有符号**。
# 原先按最保守的 23 位推断设成 3,750,000，只有真实满量程的 45%，会误报。
# 现取实测满量程的 90%。
#
# 注：A2 + 950nm + 3cm + 150档 是整个 12 块 × 5 波长 × 2 距离矩阵里最亮的
#     一格（峰值 2,883,734 = 满量程的 34%），所以 150 档对 12 块都安全。
ADC_FULL_SCALE = 8_388_607          # 实测削顶值，24 位有符号
SAT_WARN = int(ADC_FULL_SCALE * 0.9)
# 启动时自动连这个口、并下发这个档位。acquire 和 online 共用这两个常量
# （online 从这里 import），避免两处定义各改各的、悄悄跑偏。
# 串口不存在时**不自动连别的口** —— 见 autostart。
DEFAULT_PORT = "COM16"
DEFAULT_GEAR = BRIGHTNESS_MAX       # 255 = 单字节上限
# 暗时隙状态条：统计最近多少帧，以及暗自身 std 的报警线。
# 实测白噪声地板 474(3cm)/588(4cm)，留 3 倍余量。
DARK_STAT_FRAMES = 600
DARK_SD_WARN = 2000
# ⚠️ 本软件是【纯接收】，从不向设备下发任何字节（SerialWorker 只调 ser.read）。
#    所以这个按钮**不会关光源** —— LED 照常闪，必须先人工把探头遮死再点。
#    遮住探头而不是关光源，在诊断上反而更有用：LED 仍在工作但光进不到探测器，
#    于是测到的本底 = 暗电流 + 前端噪声 + LED 驱动电流经电源/地耦合进放大器的部分，
#    正好把「光学来源」和「电学来源」切开。要再往下分，需要 LED 断电（见文档）。
DEFAULT_FRAMES = 200
# 原始帧环形缓冲。⚠️ 真机帧率**未实测**；若按模拟器的 100 帧/s 算约
# 200 秒，实际秒数以界面连接栏显示的实测帧率为准。
RAW_BUFFER = 20000

# 模拟器的标度与噪声，全部由 2026-10-09 的 B3 实测反推，使排练贴近真机：
#   scale = LED功率 × PD响应 × 积分时间 × 增益（各格反推后取几何平均；
#           各格之间 14% 的离散就是仪器增益不齐，正是 G(λ) 要修的东西）
#   散粒系数 g 与加性本底由 fig_noise.py 的拟合给出
# 暗时隙的模拟。数值取自**真机带暗时隙的采集**（2026-10-10 的 B2 两份，
# 逐帧表 λ=0 行）：3cm 约 −44,500，4cm 约 +8,700。
#
# ⚠️ 曾按厂家给的一段 hex 示例设成 +855,936，那是错的 —— 实测不是那个量级。
#    示例帧只能用来核对**字段位置**，数值要以真机采集为准。
SIM_DARK = {3.0: -44_500.0, 4.0: 8_700.0}
SIM_DARK_DRIFT = 2_000.0        # 慢漂移幅度，实测 take 间差 1~3 千
SIM_DARK_NOISE = 500.0          # 暗样本自身白噪声，实测 474/588
SIM_SCALE = 1.119e10
SIM_SHOT_G = 22.5
SIM_FLOOR = {3.0: 8458.0, 4.0: 5156.0}
# 均值标准误上限（= 单帧 RSD / √帧数）。
# 依据：仿真实测「标定后 TOI 噪声贡献 ≈ 0.14 × 标准误(%)」，
# 取 0.7% → 噪声贡献约 0.1 %O₂，明显低于光谱插值的 0.21 %O₂ 地板，
# 两者平方和相加只比地板差 10%，再往下降收益很小。
# 标定后 TOI 的噪声贡献 ≈ 0.14 × 标准误(%)（仿真实测）。对照光谱插值
# 带来的误差地板 0.21 %O₂（认证不确定度 2%），分两档：
#   标准误 < 0.7%  → 噪声贡献 < 0.1 %O₂，相对地板可忽略           —— 合格
#   0.7% ~ 1.5%    → 噪声贡献 0.1~0.21 %O₂，与地板相当            —— 可用
#   > 1.5%         → 噪声开始主导，再采才有意义                   —— 不合格
#
# 为什么要有中间档：B3 在 2000 帧下标准误 1.16%，贡献 0.162 %O₂，仍低于地板；
# 按单一 0.7% 判会提示重采 5,640 帧 —— 多花 2.8 倍时间只把总误差从
# 0.265 降到 0.231（13%）。那是在浪费机时，不是在提高精度。
SEM_LIMIT = 0.007
SEM_USABLE = 0.015
# 白噪声度下限。r = std(diff)/(√2·std)：白噪声 r≈1，慢漂移/振荡 r→0。
# 2026-10-09 暗态实测的 3cm 振荡 r = 0.21 —— 远低于此，必须拦下。
DRIFT_LIMIT = 0.6
# 终端逐帧打印。默认开 —— 与厂家 jhw-adc.py 的行为一致，便于和它对拍。
# 行数 = 实测帧率（见连接栏）；长时间采集建议在界面上关掉或用 --no-print。
PRINT_FRAMES_DEFAULT = True
# 光源档位可调（厂家 2026-10-10 给出指令帧）。设备上电即发数据、不响应启停，
# 但接受档位指令。档位 0x00~0x0F，上电默认 0x0F。
BRIGHTNESS_ADJUSTABLE = True
# 档位连点的静默期（毫秒）：手停下来这么久才真正下发，把连点合并成一次。
BRIGHTNESS_SETTLE_MS = 400
# 逐帧值内嵌进 xlsx 的行数上限；超过则写同名 _逐帧.csv。
RAW_SHEET_MAX = 200_000
BAUDRATES = ["115200", "921600", "460800", "256000", "230400", "57600", "38400", "9600"]


# 档位一律按**十进制**显示和记录（厂家口径「150 个档位」，用户也这么想）。
# 16 进制只是帧里那个字节的写法，在档位框旁边、横幅里的完整帧、以及
# 原始帧窗口中对照即可。曾经做过一个 16 进制输入框，与这个口径拧着，已删。


def _crc_ok(f: Frame) -> bool:
    from srs_cal.protocol import FRAME_LEN, checksum
    return checksum(f.raw) == f.raw[FRAME_LEN - 1]


# ---------------------------------------------------------------------------
@dataclass
class Take:
    """一次采集。mean/std/n 的键是 (波长nm, 距离cm)。"""

    block: str
    index: int
    timestamp: str
    # 采这一次时的光源档位（0x00~0x0F，上电默认 0x0F）。固件不支持时为 None
    # —— 宁可空着，也不记一个没生效的数。
    # ⚠️ 2026-10-09/10 的五个文件记的「1」是界面默认值、从未下发过，
    #    实际是设备上电默认档位 0x0F（最亮）。那几份数据按 15 解读。
    brightness: int | None = None
    # 采这一次时的采集通道（指令帧 byte6 / 数据帧 byte10）。
    #
    # ⚠️ 这个值是**经回传校验过**的：on_frames 只把 byte10 与所选通道相符的帧
    #    放进缓冲，不符的当场丢弃。所以「这次采集的帧全部来自该通道」是
    #    由过滤保证的，不是由「我们下发过」推断的 —— 后者已被实测证伪
    #    （2026-10-28：下发通道 2，292 帧回来 byte10 全是 01）。
    #
    # ⚠️ 2026-10-28 之前采的文件没有这一列，不代表它们是通道 1 采的 ——
    #    只代表当时没记。按「未知」处理，别回填。
    channel: int | None = None
    mean: dict = field(default_factory=dict)
    # 稳健聚合剔除的样本数，键同 mean。供质量层使用，别丢。
    rejected: dict = field(default_factory=dict)
    std: dict = field(default_factory=dict)
    count: dict = field(default_factory=dict)
    # 暗时隙读数（距离 -> 本次采集的全部暗值）。信号已在采集时逐周期扣过暗，
    # 这里留档是为了能事后看暗电平自身漂了多少 —— 它是链路健康度指标。
    dark: dict = field(default_factory=dict)
    # 本次采集的**全部帧，按到达顺序**：
    #     (波长nm, 距离cm, ADC原始, ADC扣暗后)
    # 暗时隙行的「扣暗后」为 None —— 它自己就是被减的那个数。
    #
    # ⚠️ 两个值都留着，不是冗余：
    #   原始值  是唯一无法事后重建的东西。扣暗逻辑已经改过一次
    #           （旧固件静态暗 → 新固件逐周期暗），还可能再改。
    #   扣暗后  是判据和标定实际用的量；噪声指标必须在它上面算，
    #           在原始值上算会被暗电平的漂移淹没。
    #
    # 为什么要整条留着、并且写进 xlsx：
    #   ① 漂移/振荡只有在时间序列上才看得见。2026-10-09 实测到 3cm 通道有
    #      间歇慢振荡（峰峰 1.1e7），而它在均值/标准差里看不出来。
    #   ② 判据将来会改（标准误的定义已经改过一次）。只存均值的话，
    #      改了判据就得重新上机采，而不是重算。
    #   ③ 做噪声频谱（50Hz 工频 / 1/f 漂移 / 平坦本底）必须有逐帧值。
    frames: list = field(default_factory=list)

    def complete(self) -> bool:
        return all((w, r) in self.mean for w in WLS for r in RHOS)

    def dark_stats(self) -> dict:
        """距离 -> (暗均值, 暗自身的标准差, 峰峰)。空时返回 {}。"""
        out = {}
        for r, v in self.dark.items():
            a = np.asarray(v, float)
            if a.size:
                out[r] = (float(a.mean()),
                          float(a.std(ddof=1)) if a.size > 1 else 0.0,
                          float(np.ptp(a)))
        return out

    def pulsatility(self, cycle_hz: float) -> dict:
        """距离 -> (AC/DC 振幅比, 峰频Hz, 等效心率bpm, 谱峰显著度)。

        判据用**显著度** = 带内谱峰 / 全谱中位，不用「带内功率占比」：
        0.67~3 Hz 这个带本身就占频谱宽度的约 47%，白噪声天然落进去
        44~48%，拿占比当判据等于把纯噪声判成心搏（2026-10-13 自验到）。
        真心搏是一根窄谱线，显著度能到几十上百；白噪声只有个位数。

        AC/DC 是把带外谱清零后反变换得到的峰峰值除以均值 —— 就是灌注
        指数那个量，在体典型 1~3%，给人看比显著度直观。

        ⚠️ `cycle_hz` 要用**实测**的扫描周期率。采样率不足（< 2×3Hz）时
           返回空 —— 心搏会混叠到别的频率，报出来的数反而误导。
        """
        out: dict = {}
        if not np.isfinite(cycle_hz) or cycle_hz < 2 * PULSE_BAND_HZ[1]:
            return out
        for rho in RHOS:
            # 用该距离上最亮的那个波长，信噪最好
            best, series = None, None
            for w in WLS:
                v = [f[3] for f in self.frames
                     if f[0] == w and f[1] == rho and f[3] is not None]
                if len(v) >= 32 and (best is None or np.mean(v) > best):
                    best, series = float(np.mean(v)), np.asarray(v, float)
            if series is None:
                continue
            x = series - np.polyval(np.polyfit(np.arange(series.size),
                                               series, 1),
                                    np.arange(series.size))   # 去线性漂移
            f = np.fft.rfftfreq(x.size, d=1.0 / cycle_hz)
            X = np.fft.rfft(x * np.hanning(x.size))
            p = np.abs(X) ** 2
            p[0] = 0.0
            band = (f >= PULSE_BAND_HZ[0]) & (f <= PULSE_BAND_HZ[1])
            med = float(np.median(p[1:])) if p.size > 2 else 0.0
            if med <= 0 or not band.any():
                continue
            ib = int(np.argmax(p[band]))
            pk = float(f[band][ib])
            prom = float(p[band][ib] / med)
            # AC/DC：只留峰附近 ±1 bin 反变换，取峰峰 / 直流。
            # ⚠️ 这里用**不加窗**的变换。加窗后再按窗均值归一会把幅度放大
            #    一倍（窗中心是 1.0，均值才 0.5）—— 实测 1.5% 正弦被报成
            #    6.2%，正是这个原因。窗只用于上面算显著度（压旁瓣）。
            Xu = np.fft.rfft(x)
            keep = np.zeros_like(Xu)
            k = int(np.arange(f.size)[band][ib])
            keep[max(k - 1, 1):k + 2] = Xu[max(k - 1, 1):k + 2]
            ac = np.fft.irfft(keep, n=x.size)
            dc = float(np.mean(series))
            out[rho] = (float(np.ptp(ac) / dc) if dc else float("nan"),
                        pk, pk * 60.0, prom)
        return out

    def det_series(self) -> dict:
        """按【探测器】的到达序列（距离 -> 该探测器的全部读数，不分波长）。

        漂移是整个探测器的现象、与点亮哪颗 LED 无关（实测 3cm 读数跨波长
        连成一条光滑曲线）。按 (波长,距离) 分组后每组样本间隔 10 帧，
        慢振荡会被混叠成看似随机的噪声 —— 粒度不对就测不出来。
        """
        out: dict = {}
        for _w, r, _raw, net in self.frames:
            if net is None:            # 暗时隙本身不进信号的噪声统计
                continue
            out.setdefault(r, []).append(net)
        return out

    @staticmethod
    def _independence(x: np.ndarray) -> float:
        """白噪声度 r = std(diff)/(√2·std)。白噪声 r≈1，强漂移 r→0。

        对 AR(1)：var(diff)=2σ²(1−ρ) ⇒ r=√(1−ρ)，故 ρ = 1 − r²。
        """
        if x.size < 3 or x.std(ddof=1) == 0:
            return 1.0
        return float(np.diff(x).std(ddof=1) / (np.sqrt(2) * x.std(ddof=1)))

    def worst_noise(self) -> tuple[float, float, int, float]:
        """返回 (最差单帧 RSD, 最差均值标准误, 每组合帧数, 最低白噪声度)。

        ⚠️ 标准误**不是** RSD/√n。那个式子要求样本独立，而 2026-10-09 实测
           发现 3 cm 通道会出现间歇的慢振荡（峰峰 1.1e7，白噪声度仅 0.21），
           此时相邻帧高度相关，平均根本降不下噪声 —— 按 √n 算会把误差
           低估几十倍，判定会误判"合格"。

           故改用等效样本数：AR(1) 下均值方差放大 (1+ρ)/(1−ρ)，
           即 n_eff = n·(1−ρ)/(1+ρ) = n·r²/(2−r²)，并夹在 [1, n]。
           白噪声时 r=1 ⇒ n_eff=n，退化成原式。
        """
        worst_rsd = worst_sem = 0.0
        n = 0
        # 白噪声度按【探测器序列】判，不按单个波长组合判 —— 见 det_samples。
        worst_r = 1.0
        for seq in self.det_series().values():
            x = np.asarray(seq, dtype=float)
            if x.size >= 8:
                worst_r = min(worst_r, self._independence(x))
        for k, m in self.mean.items():
            if m == 0:
                continue
            c = max(1, self.count.get(k, 1))
            n = max(n, c)
            rsd = abs(self.std[k] / m)
            worst_rsd = max(worst_rsd, rsd)
            r = worst_r
            n_eff = min(c, max(1.0, c * r ** 2 / max(2 - r ** 2, 1e-9)))
            worst_sem = max(worst_sem, rsd / np.sqrt(n_eff))
        return worst_rsd, worst_sem, n, worst_r


# ---------------------------------------------------------------------------
class SerialWorker(QtCore.QThread):
    framesReady = QtCore.pyqtSignal(list)
    statusMsg = QtCore.pyqtSignal(str)
    failed = QtCore.pyqtSignal(str)

    def __init__(self, port: str, baud: int) -> None:
        super().__init__()
        self.port, self.baud = port, baud
        self._run = True
        self.stream = FrameStream()
        self._pending: list[bytes] = []
        self._lock = QtCore.QMutex()

    def send(self, payload: bytes) -> None:
        """线程安全地排队一条指令，由 run() 在读循环里发出。"""
        self._lock.lock()
        try:
            self._pending.append(payload)
        finally:
            self._lock.unlock()

    def stop(self) -> None:
        self._run = False
        self.wait(1500)

    def run(self) -> None:
        try:
            import serial
        except ImportError:
            self.failed.emit("未安装 pyserial：pip install pyserial")
            return
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.05)
        except Exception as e:                                  # noqa: BLE001
            self.failed.emit(f"打开 {self.port} 失败：{e}")
            return
        self.statusMsg.emit(f"已连接 {self.port} @ {self.baud}")
        try:
            while self._run:
                self._lock.lock()
                out, self._pending = self._pending, []
                self._lock.unlock()
                for pay in out:
                    ser.write(pay)
                # ⚠️ 2026-10-22：不要用 `ser.read(4096)` —— 它阻塞到**超时**
                #    才返回，帧的 t_host 因而晚于真实到达时刻，凭空制造抖动。
                #    真机实测（verify_timing，COM16 115200，2026-10-22）：
                #        read(4096) + timeout=0.2 → 每批 2 帧，净抖动 31%
                #        先等 1 字节再打戳        → 每批 1 帧，净抖动  7%
                #    设备以 2.3 ms 突发送完一帧，再静默约 125 ms，所以
                #    "等第一个字节"几乎就是帧的到达时刻。
                #    归档会把 t_host 永久记下来，事后补不回来，故在此修正。
                head = ser.read(1)
                if head:
                    t_arrive = time.perf_counter()
                    rest = ser.read(ser.in_waiting) if ser.in_waiting else b""
                    fr = self.stream.feed(head + rest, t_host=t_arrive)
                    if fr:
                        self.framesReady.emit(fr)
        except Exception as e:                                  # noqa: BLE001
            self.failed.emit(f"串口读取中断：{e}")
        finally:
            try:
                ser.close()
            except Exception:                                   # noqa: BLE001
                pass
            self.statusMsg.emit("已断开")


class SimWorker(QtCore.QThread):
    """模拟器：按体模正演模型造帧，让 UI 在没有硬件时可排练、可验证。"""

    framesReady = QtCore.pyqtSignal(list)
    statusMsg = QtCore.pyqtSignal(str)
    failed = QtCore.pyqtSignal(str)

    def __init__(self, block: str = "B3") -> None:
        super().__init__()
        self._run = True
        self.stream = FrameStream()
        self.block = block

    def set_block(self, block: str) -> None:
        self.block = block

    def send(self, payload: bytes) -> None:
        """模拟器忽略指令，只为与 SerialWorker 接口一致。

        ⚠️ 造帧时 `build_frame` 的通道走默认值 1，所以在模拟器下选通道 2~4
           会**一帧都收不到**（全被 on_frames 的 byte10 校验丢掉）。
           这不是 bug —— 真机实测也是这个表现（下发 byte6 不被接受）。
           想排练多通道，得先有一台真会切换的设备。
        """

    def stop(self) -> None:
        self._run = False
        self.wait(1500)

    def run(self) -> None:
        from srs_cal.forward import intensity_matrix
        self.statusMsg.emit("模拟器运行中（非真实数据）")
        rng = np.random.default_rng()
        wl = np.array(WLS)
        rho = np.array(RHOS)
        cache: dict[str, np.ndarray] = {}
        while self._run:
            blk = self.block
            if blk not in cache:
                if blk == DARK_CODE:
                    cache[blk] = np.full((len(WLS), len(RHOS)), 150.0)
                elif blk == ARM_CODE:
                    # 前臂量级：按头部组织 μeff≈2.1 的体模(D2/B4 之间)取，
                    # 比 D4 亮、比 C2 暗。**是量级，不是实测** ——
                    # 真机采到之前没有在体数据可依。搏动在下面逐帧叠加。
                    cache[blk] = np.array(
                        [[3.0e5, 1.1e4], [6.0e5, 2.6e4], [5.4e5, 2.4e4],
                         [8.5e5, 3.8e4], [1.4e6, 6.6e4]])[:len(WLS)]
                elif blk == AIR_CODE:
                    # 空测：没有体模，读数只剩板内串扰 —— 与波长弱相关、
                    # 与距离强相关。这里的量级是 2026-10-12 D4 数据拟
                    # I = A·R + X(ρ) 得到的 X，**是推测值，只为排练时
                    # 界面不至于空白，不可当实测**。
                    cache[blk] = np.array(
                        [[1.0e5, 2.8e3], [2.0e5, 6.3e3], [1.8e5, 6.0e3],
                         [2.7e5, 9.8e3], [4.3e5, 1.6e4]])[:len(WLS)]
                else:
                    i = phantom.BLOCK_CODES.index(blk)
                    mua, musp = phantom.interpolate_to(wl)
                    cache[blk] = intensity_matrix(mua[i], musp[i], rho,
                                                  phantom.REFRACTIVE_INDEX,
                                                  scale=SIM_SCALE)
            base = cache[blk]
            if blk == ARM_CODE:
                # 心搏：约 72 bpm，搏动幅度取总光强的 1.5%（在体典型 1~3%）。
                # 排练时能看到判定里的「搏动占比」亮起来，知道该期待什么。
                self._pulse = getattr(self, "_pulse", 0.0) + 2 * np.pi * 1.2 / 10.0
                base = base * (1.0 + 0.015 * np.sin(self._pulse))
            # 暗时隙：每周期最前，两个探测器各一帧。带慢漂移，量级取 SIM_DARK
            # （实测 3cm 约 −4.4 万、4cm 约 +0.9 万；两个探测器符号相反）。
            self._dark_phase = getattr(self, "_dark_phase", 0.0) + 0.05
            blob = b""
            for b_, r in enumerate(RHOS):
                dk = (SIM_DARK[r] + SIM_DARK_DRIFT * np.sin(self._dark_phase)
                      + rng.standard_normal() * SIM_DARK_NOISE)
                self._cur_dark = getattr(self, "_cur_dark", {})
                self._cur_dark[r] = dk
                blob += build_frame(0.0, r, int(dk))
            for a, w in enumerate(WLS):
                for b, r in enumerate(RHOS):
                    # 噪声按 B3 实测结构还原：散粒项 + 每探测器一个加性本底，
                    # 950 免除加性项（实测它落在散粒限上）。这样排练时看到的
                    # 「不合格」判定与真机一致，而不是乐观的假象。
                    sigma = np.sqrt(SIM_SHOT_G * base[a, b]
                                    + (0.0 if w in hw.WL_REFERENCE_ONLY_NM
                                       else SIM_FLOOR[r] ** 2))
                    v = (base[a, b] + sigma * rng.standard_normal()
                         + self._cur_dark[r])        # 真机读数里含暗偏置
                    blob += build_frame(w, r, int(np.clip(v, 0, 2 ** 31 - 1)))
            self.framesReady.emit(self.stream.feed(blob))
            # ⚠️ 10 个扫描周期/秒（100 帧/s）是**假设值，未经真机核实**。
            #    排练时界面上看到的刷新速度因此只能当参考 —— 真机的实测
            #    帧率显示在连接栏，以那个为准。
            self.msleep(100)


# ---------------------------------------------------------------------------
class MainWindow(QtWidgets.QWidget):
    def __init__(self, simulate: bool = False) -> None:
        super().__init__()
        self.simulate = simulate
        self.worker: QtCore.QThread | None = None
        self.takes: list[Take] = []
        self.capturing: str | None = None
        self.buffer: dict[tuple[float, float], list[int]] = {}
        self.order_buffer: list[tuple[float, float, int]] = []
        # 暗时隙：cycle_dark 是「本周期的暗」，随每个周期刷新；
        # dark_buffer 把全部暗读数留下来，用于事后看暗电平自己漂多少。
        self.dark_buffer: dict[float, list[int]] = {}
        self.cycle_dark: dict[float, int] = {}
        self.n_no_dark = 0
        self.n_captured = 0
        self.target_frames = DEFAULT_FRAMES
        self.latest: dict[tuple[float, float], int] = {}
        # 实时显示用的「本周期暗」，与采集缓冲独立 —— 空闲时也要更新
        self.live_dark: dict[float, int] = {}
        # 最后一次**成功下发**的档位。None = 从未发过，设备停在上电默认档。
        self._gear_sent: int | None = None
        # 通道同理。另外三个计数器用于**以回传为准**的校验（见 on_frames）。
        self._chan_sent: int | None = None
        self._n_chan_ok = 0
        self._n_chan_bad = 0
        self._chan_seen: set = set()
        self._frames_seen = 0
        self.raw_buf: deque = deque(maxlen=RAW_BUFFER)   # (序号, 相对秒, Frame)
        self._t0 = time.perf_counter()
        self._raw_win = None
        self._plot_win = None
        self._build_ui()
        self._rate_timer = QtCore.QTimer(self)
        self._rate_timer.timeout.connect(self._tick_rate)
        self._rate_timer.start(1000)
        self._last_seen = 0

    # -- UI ----------------------------------------------------------------
    def _build_ui(self) -> None:
        self.setWindowTitle("SRS 体模标定采集 —— P3137034"
                            + ("（模拟器）" if self.simulate else ""))
        self.resize(1140, 760)
        root = QtWidgets.QVBoxLayout(self)

        # 串口设置
        bar = QtWidgets.QHBoxLayout()
        self.cmb_port = QtWidgets.QComboBox(); self.cmb_port.setMinimumWidth(210)
        self.cmb_baud = QtWidgets.QComboBox(); self.cmb_baud.addItems(BAUDRATES)
        self.cmb_baud.setEditable(True)
        self.btn_refresh = QtWidgets.QPushButton("刷新")
        self.btn_conn = QtWidgets.QPushButton("连接")
        self.spn_frames = QtWidgets.QSpinBox()
        self.spn_frames.setRange(10, 20000); self.spn_frames.setValue(DEFAULT_FRAMES)
        # ⚠️ 这个数是**总帧数**，会被 5 波长 × 2 距离 = 10 个组合平分。
        #    判据看的是【每组合】帧数，所以后缀动态显示换算结果，免得看岔。
        self.spn_frames.setSuffix(" 帧总计")
        self.lbl_percombo = QtWidgets.QLabel()
        self.lbl_percombo.setStyleSheet("color:#b8860b;font-weight:bold")
        self.spn_frames.valueChanged.connect(self._sync_frame_hint)
        self._sync_frame_hint()          # 标签建好之后再刷一次初值
        # 十进制 0~150 —— 厂家口径是「150 个档位」，用户也按档位数思考。
        # 16 进制只是帧里那个字节的写法，放在旁边的 lbl_gearhex 上对照即可。
        self.spn_bright = QtWidgets.QSpinBox()
        self.spn_bright.setRange(BRIGHTNESS_MIN, BRIGHTNESS_MAX)
        self.spn_bright.setValue(BRIGHTNESS_DEFAULT)
        self.spn_bright.setSuffix(f"/{BRIGHTNESS_MAX}")
        self.lbl_gearhex = QtWidgets.QLabel()
        self.lbl_gearhex.setStyleSheet(
            "color:#2a78d6;font-family:Consolas,monospace")
        self.spn_bright.valueChanged.connect(
            lambda v: self.lbl_gearhex.setText(f"= 0x{v:02X}"))
        self.lbl_gearhex.setText(f"= 0x{BRIGHTNESS_DEFAULT:02X}")
        self.spn_bright.setEnabled(BRIGHTNESS_ADJUSTABLE)
        self.spn_bright.setToolTip(
            f"光源档位 0~{BRIGHTNESS_MAX} 共 {BRIGHTNESS_MAX + 1} 档，"
            f"上电默认 {BRIGHTNESS_DEFAULT}（= 上限的 "
            f"{BRIGHTNESS_DEFAULT / BRIGHTNESS_MAX * 100:.0f}%）。\n"
            "改动立即下发指令帧 55 AA 1A 01 01 <档位> <通道> …\n\n"
            "对 SRS 是安全的：光强整体乘一个因子时两个探测器同步变，空间斜率不变，\n"
            "所以不同体模块可以用不同档位迁就动态范围。同一块采集期间不要改。\n"
            "每次采集的档位会随数据存进 xlsx。\n\n"
            "⚠️ 前提未验证：五路 LED 是否等比例变化、档位是否改变峰值波长。\n"
            "   先用同一块体模在两个档位各采一次，比 G(λ)。")
        # 档位要下发，但**不能每动一下就发一帧**：从 0x00 点到 0x0F 会连发
        # 15 条，而且中间每一档都真的生效了一瞬间。所以
        #   ① 关掉 keyboardTracking —— 打字时按回车/失焦才算数，不是每敲一键；
        #   ② 再加一个静默期，把连点合并成最后那一次。
        self.spn_bright.setKeyboardTracking(False)
        self._bright_timer = QtCore.QTimer(self)
        self._bright_timer.setSingleShot(True)
        self._bright_timer.setInterval(BRIGHTNESS_SETTLE_MS)
        self._bright_timer.timeout.connect(self._send_brightness)
        self.spn_bright.valueChanged.connect(self._bright_timer.start)
        # 采集通道。与档位同走一条指令帧（byte5 档位、byte6 通道），
        # 所以改哪一个都调 _send_brightness。下拉框是离散选择、不会连点，
        # 不需要档位那样的静默期。
        self.cmb_chan = QtWidgets.QComboBox()
        for c in range(CHANNEL_MIN, CHANNEL_MAX + 1):
            self.cmb_chan.addItem(f"通道 {c}", c)
        i = self.cmb_chan.findData(EXPECTED_CHANNEL)
        self.cmb_chan.setCurrentIndex(max(i, 0))
        self.cmb_chan.setToolTip(
            f"采集通道 {CHANNEL_MIN}~{CHANNEL_MAX}，写进指令帧 byte6。\n"
            "改动立即下发 55 AA 1A 01 01 <档位> <通道> …\n\n"
            "⚠️ 以**回传**为准：只有数据帧 byte10 与此处相符的帧才会进缓冲，\n"
            "   不符的当场丢弃并在下方报出来。\n\n"
            "通道是**累加**的（厂家文档「累加依次」+ 2026-10-29 实测）：\n"
            "   开了 2 号之后 1 号仍在发，设备把帧率翻倍。\n"
            "   采集只收这里选中的那一个 —— 标定本来就是一次标一套探头。\n"
            "   要真正关掉某个通道，用右边的开关复选框。\n\n"
            "⚠️ 换通道 = 换一整套光学器件，G(λ) 不迁移。\n"
            "   一个 xlsx 里混多个通道，标定会被带歪。")
        self.cmb_chan.currentIndexChanged.connect(self._on_channel)
        # 每通道的开/关。与 online 同一套做法：勾 → 启动帧，取消 → 停止帧，
        # 都只作用在 byte6 指定的那个通道上。
        #
        # 采集侧为什么也要：通道累加意味着别的通道一直在占串口带宽，
        # 并且（2026-10-29 实测）开着 2 号会让 1 号的快噪声涨 5.3~5.9 倍。
        # 标定时要能把别的通道**真正关掉**，而不只是"不看它"。
        self.chk_ch: dict[int, QtWidgets.QCheckBox] = {}
        self._gear_by_ch: dict[int, int] = {}
        for c in range(CHANNEL_MIN, CHANNEL_MAX + 1):
            cb = QtWidgets.QCheckBox(str(c))
            cb.setToolTip(
                f"通道 {c} 的开/关。\n"
                f"  勾选 → 启动帧 byte4=01、byte6={c:02X}（档位用它自己的）\n"
                f"  取消 → 停止帧 byte4=00、byte6={c:02X}\n\n"
                "⚠️ 勾上不代表设备真开了 —— 以下方的回传状态为准。\n"
                "⚠️ 标定时建议**只开要标的那一个**：开着别的通道会给它\n"
                "   带来可观的电气噪声（实测快噪声涨 5.3~5.9 倍）。")
            cb.setChecked(c == EXPECTED_CHANNEL)
            cb.toggled.connect(lambda on, ch=c: self._on_chan_toggle(ch, on))
            self.chk_ch[c] = cb
            self._gear_by_ch[c] = BRIGHTNESS_DEFAULT
        self.chk_print = QtWidgets.QCheckBox("终端打印")
        self.chk_print.setChecked(PRINT_FRAMES_DEFAULT)
        self.chk_print.setToolTip(
            "把每一帧解析出的 ADC 打到终端，一行一个值。\n"
            "格式：序号 | 波长 | 距离 | ADC | 原始四字节 | 校验\n"
            "每秒行数 = 连接栏显示的帧率；嫌刷屏可以随时取消勾选。")
        self.btn_run = QtWidgets.QPushButton("▶ 开始采集")
        self.btn_run.setCheckable(True)
        self.btn_run.toggled.connect(self.toggle_run)
        self.btn_run.setEnabled(False)
        for w in (QtWidgets.QLabel("串口"), self.cmb_port, QtWidgets.QLabel("波特率"),
                  self.cmb_baud, self.btn_refresh, self.btn_conn,
                  QtWidgets.QLabel("  光源档位"), self.spn_bright,
                  self.lbl_gearhex,
                  QtWidgets.QLabel("  采集通道"), self.cmb_chan,
                  QtWidgets.QLabel(" 开"), *self.chk_ch.values(),
                  self.btn_run, self.chk_print,
                  QtWidgets.QLabel("  每次采集"), self.spn_frames,
                  self.lbl_percombo):
            bar.addWidget(w)
        bar.addStretch(1)
        self.lbl_link = QtWidgets.QLabel("未连接")
        self.lbl_link.setStyleSheet("color:#888")
        bar.addWidget(self.lbl_link)
        root.addLayout(bar)
        # 通道状态：单独一行而不是挤在工具栏里 —— 「收到 N 帧但都标着通道 M」
        # 这句话很长，塞进工具栏会把串口下拉框挤没。
        self.lbl_chan = QtWidgets.QLabel("")
        self.lbl_chan.setWordWrap(True)
        root.addWidget(self.lbl_chan)
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_conn.clicked.connect(self.toggle_conn)
        if self.simulate:
            self.cmb_port.setEnabled(False); self.cmb_baud.setEnabled(False)
            self.btn_refresh.setEnabled(False)

        mid = QtWidgets.QHBoxLayout(); root.addLayout(mid, 1)

        # 实时监视
        # 两组列：原始值 和 扣暗后。
        # 暗电平占原始读数 90%+，只看原始值时五个波长全挤在 85~95 万，
        # 看不出信号强弱、也判断不了该调哪个档位 —— 扣暗那两列才是能用的。
        gb_live = QtWidgets.QGroupBox("实时 ADC")
        gl = QtWidgets.QGridLayout(gb_live)
        gl.addWidget(QtWidgets.QLabel("<b>波长</b>"), 0, 0)
        for j, r in enumerate(RHOS):
            gl.addWidget(QtWidgets.QLabel(f"<span style='color:#999'>原始"
                                          f" {r:.0f}cm</span>"), 0, j + 1)
        for j, r in enumerate(RHOS):
            gl.addWidget(QtWidgets.QLabel(f"<b>扣暗 {r:.0f}cm</b>"),
                         0, j + 1 + len(RHOS))
        self.live_lbl: dict[tuple[float, float], QtWidgets.QLabel] = {}
        self.live_net: dict[tuple[float, float], QtWidgets.QLabel] = {}
        # 暗时隙单独一行，排在最前 —— 它在帧流里也排在每个周期最前，
        # 而且现在占原始读数的 90%+，是最该一眼看到的量。
        for i, w in enumerate([0.0] + WLS):
            if w == 0.0:
                lb = QtWidgets.QLabel("暗（LED 全灭）")
                lb.setStyleSheet("color:#2a78d6;font-weight:bold")
            else:
                tag = (f"{w:.0f} nm"
                       + ("  (参考)" if w in hw.WL_REFERENCE_ONLY_NM else ""))
                lb = QtWidgets.QLabel(tag)
                if w in hw.WL_REFERENCE_ONLY_NM:
                    lb.setStyleSheet("color:#999")
            gl.addWidget(lb, i + 1, 0)
            for j, r in enumerate(RHOS):
                v = QtWidgets.QLabel("—")
                v.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                v.setStyleSheet("font-family:Consolas,monospace;font-size:12px;"
                                "color:#999")
                gl.addWidget(v, i + 1, j + 1)
                self.live_lbl[(w, r)] = v
                if w == 0.0:
                    continue                 # 暗自己没有「扣暗后」
                n = QtWidgets.QLabel("—")
                n.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                n.setStyleSheet("font-family:Consolas,monospace;font-size:14px;"
                                "font-weight:bold")
                gl.addWidget(n, i + 1, j + 1 + len(RHOS))
                self.live_net[(w, r)] = n
        mid.addWidget(gb_live, 2)

        # 12 块按钮
        gb_blk = QtWidgets.QGroupBox("体模块（点击采集；可重复采，用于重贴合变异）")
        bl = QtWidgets.QGridLayout(gb_blk)
        for j, c in enumerate(("μa≈0.05", "μa≈0.085", "μa≈0.125")):
            bl.addWidget(QtWidgets.QLabel(f"<b>{c}</b>"), 0, j + 1,
                         alignment=QtCore.Qt.AlignCenter)
        self.blk_btn: dict[str, QtWidgets.QPushButton] = {}
        for i, (row, ms) in enumerate(zip("ABCD", (5, 12, 18, 25))):
            bl.addWidget(QtWidgets.QLabel(f"<b>{row}</b> μs′≈{ms}"), i + 1, 0)
            for j, col in enumerate("234"):
                code = f"{row}{col}"
                b = QtWidgets.QPushButton(code)
                b.setMinimumHeight(44)
                b.clicked.connect(lambda _, c=code: self.start_capture(c))
                bl.addWidget(b, i + 1, j + 1)
                self.blk_btn[code] = b
        # 原先这里有一个「暗电平（先人工遮死探头）」按钮。2026-10-10 的固件
        # 每个扫描周期自带一帧 LED 全灭的暗时隙，逐周期就近扣 —— 手动暗采
        # 既多余又误导（它当时的说明写着"本软件不关光源"）。已删除。
        # 取而代之的是下面这条常驻的暗时隙状态条。
        # 空测按钮：探头悬空或放黑面，测板内光串扰。放在网格最后一行，
        # 和 12 块分开 —— 它不是体模，不参与标定。
        b_air = QtWidgets.QPushButton("空测（探头悬空／放黑面，测板内串扰）")
        b_air.setMinimumHeight(40)
        b_air.setStyleSheet("font-weight:bold;color:#8a5a00")
        b_air.setToolTip(
            "探头下面不放体模，LED 照常亮。采到的就是不经介质、\n"
            "直接从板材耦合过来的那部分光。与同档位的体模读数相比，\n"
            "即可知道体模读数里有多大比例不是穿过体模的。\n"
            "不会进入标定。")
        b_air.clicked.connect(lambda: self.start_capture(AIR_CODE))
        bl.addWidget(b_air, 5, 0, 1, 2)
        # 在体采集。判定走另一条分支（见 ARM_CODE 的注释）。
        b_arm = QtWidgets.QPushButton("手臂（在体）")
        b_arm.setMinimumHeight(40)
        b_arm.setStyleSheet("font-weight:bold;color:#1a7f5a")
        b_arm.setToolTip(
            "贴在前臂上采集。体模的验收判据对它不适用：\n"
            "心搏会让白噪声度必然偏低，那是真信号不是故障。\n"
            "这里改看「有没有心搏」——看得见心搏，才说明探头\n"
            "确实耦合到了活体组织，而不是在读板内串扰。\n"
            "探头顺着手臂长轴放，不要横着绕。不进标定。")
        b_arm.clicked.connect(lambda: self.start_capture(ARM_CODE))
        bl.addWidget(b_arm, 5, 2, 1, 2)
        self.lbl_darkstat = QtWidgets.QLabel("暗时隙：等待数据…")
        self.lbl_darkstat.setStyleSheet(
            "padding:6px;border:1px solid #cfe0f5;border-radius:4px;"
            "background:#f2f7fd;color:#2a78d6;font-family:Consolas,monospace")
        bl.addWidget(self.lbl_darkstat, 6, 0, 1, 4)
        mid.addWidget(gb_blk, 3)

        # 进度 + 判定横幅
        self.bar = QtWidgets.QProgressBar(); self.bar.setValue(0)
        self.bar.setFormat("空闲")
        root.addWidget(self.bar)
        self.lbl_banner = QtWidgets.QLabel("")
        self.lbl_banner.setWordWrap(True)
        self.lbl_banner.setVisible(False)
        self.lbl_banner.setMinimumHeight(34)
        root.addWidget(self.lbl_banner)

        # 记录表
        self.table = QtWidgets.QTableWidget(0, 8)
        self.table.setHorizontalHeaderLabels(
            ["块", "第几次", "时间", "每组合帧数", "最差单帧 RSD",
             f"最差均值标准误（限 {SEM_LIMIT*100:.1f}%）",
             f"白噪声度（限 >{DRIFT_LIMIT}）",
             f"判定（合格<{SEM_LIMIT*100:.1f}% 可用<{SEM_USABLE*100:.1f}%）"])
        self.table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.Stretch)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        root.addWidget(self.table, 1)

        # 底部
        bot = QtWidgets.QHBoxLayout()
        self.btn_plot = QtWidgets.QPushButton("实时曲线…")
        self.btn_plot.setToolTip(
            "两个探测器各画 5 条波长，第三格画 SRS 真正用的空间斜率。\n"
            "串扰/泄漏只在斜率上现形 —— 单看探测器读数看不出来。")
        self.btn_plot.clicked.connect(self.open_live_plot)
        self.btn_raw = QtWidgets.QPushButton("原始帧…")
        self.btn_raw.setToolTip("按协议字段分段显示每帧 26 字节；可导出做噪声频谱")
        self.btn_raw.clicked.connect(self.open_raw_view)
        self.btn_del = QtWidgets.QPushButton("删除选中记录")
        self.btn_save = QtWidgets.QPushButton("保存 xlsx 到 srs_cal/")
        self.btn_save.setStyleSheet("font-weight:bold;padding:7px 18px")
        self.btn_del.clicked.connect(self.delete_selected)
        self.btn_save.clicked.connect(self.save_xlsx)
        # —— 两条固定的协议测试帧（2026-10-29，用户逐字节指定）——
        # 放在底排而不是顶排：顶排是常规控件，而「关闭通道一」会停数据流，
        # 误点一下代价不小（能不能用启动帧恢复还没验过）。
        self.btn_cmd_ch2 = QtWidgets.QPushButton("启动通道2")
        self.btn_cmd_ch2.setToolTip(
            "原样下发：55 AA 1A 01 01 64 02 … 82\n"
            "（启动 | 强度 100 | 二通道）\n\n"
            "⚠️ 2026-10-28 实测：下发通道 2 后回传的 byte10 仍是 01。\n"
            "   若这次仍无变化，看下方通道状态行会直接写出来。")
        self.btn_cmd_ch2.clicked.connect(self._on_cmd_start_ch2)
        self.btn_cmd_stop = QtWidgets.QPushButton("关闭通道一")
        self.btn_cmd_stop.setStyleSheet("color:#a33208;font-weight:bold")
        self.btn_cmd_stop.setToolTip(
            "原样下发：55 AA 1A 01 00 00 01 … 1C\n"
            "（停止 | 强度 0 | 一通道）\n\n"
            "⚠️ 停止帧**从未实测过**。设备一直是上电即连续发数据。\n"
            "   生效的标志是连接栏帧率掉到 0；恢复只能靠再发启动帧，\n"
            "   万一启动帧也不被接受，就得重新上电。先在体模上试。")
        self.btn_cmd_stop.clicked.connect(self._on_cmd_stop_ch1)
        self.lbl_prog = QtWidgets.QLabel("已采 0/12 块")
        bot.addWidget(self.lbl_prog); bot.addStretch(1)
        bot.addWidget(QtWidgets.QLabel("协议测试"))
        bot.addWidget(self.btn_cmd_ch2); bot.addWidget(self.btn_cmd_stop)
        bot.addWidget(QtWidgets.QLabel("｜"))
        bot.addWidget(self.btn_plot); bot.addWidget(self.btn_raw)
        bot.addWidget(self.btn_del)
        bot.addWidget(self.btn_save)
        root.addLayout(bot)

        self.refresh_ports()
        self._refresh_block_styles()

    # -- 串口 --------------------------------------------------------------
    def refresh_ports(self) -> None:
        """枚举串口，并把 DEFAULT_PORT 预选上（若在）。"""
        self.cmb_port.clear()
        try:
            from serial.tools import list_ports
            for p in list_ports.comports():
                self.cmb_port.addItem(f"{p.device} — {p.description}", p.device)
        except Exception:                                       # noqa: BLE001
            pass
        if self.cmb_port.count() == 0:
            self.cmb_port.addItem("未发现串口", None)
            return
        i = self.cmb_port.findData(DEFAULT_PORT)
        if i >= 0:
            self.cmb_port.setCurrentIndex(i)

    def autostart(self) -> None:
        """启动时自动连 DEFAULT_PORT 并下发 DEFAULT_GEAR。

        ⚠️ 只在该串口**确实存在**时才动。找不到就只提示，不去连别的口 ——
           自动连到另一台设备上还给它下发满档光强，比"没自动连"糟得多。
        """
        if self.simulate or self.worker is not None:
            return
        port = getattr(self, "auto_port", DEFAULT_PORT)
        if self.cmb_port.findData(port) < 0:
            self.lbl_link.setText(f"未发现 {port}，请手动选口")
            return
        self.cmb_port.setCurrentIndex(self.cmb_port.findData(port))
        self.toggle_conn()
        if self.worker is None:                 # 开口失败，on_failed 已提示
            return
        self.spn_bright.setValue(getattr(self, "auto_gear", DEFAULT_GEAR))
        self._bright_timer.start()              # 走既有防抖通道下发

    def toggle_conn(self) -> None:
        if self.worker is not None:
            if self.btn_run.isChecked():
                self.btn_run.setChecked(False)      # 断开前先发停止指令
            self.worker.stop(); self.worker = None
            self.btn_conn.setText("连接"); self.lbl_link.setText("未连接")
            self.btn_run.setEnabled(False)
            return
        if self.simulate:
            self.worker = SimWorker()
        else:
            port = self.cmb_port.currentData()
            if not port:
                QtWidgets.QMessageBox.warning(self, "没有串口", "未选择串口。")
                return
            try:
                baud = int(self.cmb_baud.currentText())
            except ValueError:
                QtWidgets.QMessageBox.warning(self, "波特率", "波特率必须是整数。")
                return
            self.worker = SerialWorker(port, baud)
        self.worker.framesReady.connect(self.on_frames)
        self.worker.statusMsg.connect(self.lbl_link.setText)
        self.worker.failed.connect(self.on_failed)
        self.worker.start()
        self.btn_conn.setText("断开")
        self.btn_run.setEnabled(True)
        # 连上就把界面上的档位补发一次，让设备状态与界面一致。
        # 否则「先改档位、后连接」时两边会一直对不上。
        if BRIGHTNESS_ADJUSTABLE:
            self._send_brightness()

    def gear_in_effect(self) -> int:
        """设备**实际**在用的档位。

        不能直接拿输入框的值 —— 未连接时改档位是发不出去的，设备仍停在
        上电默认档。拿输入框去填 xlsx，会记下一个从未生效过的数。
        """
        return BRIGHTNESS_DEFAULT if self._gear_sent is None else self._gear_sent

    def channel(self) -> int:
        """下拉框当前选中的通道。"""
        return int(self.cmb_chan.currentData())

    def channel_in_effect(self) -> int:
        """设备**实际**在用的通道。与 gear_in_effect 同理 ——
        未连接时改通道发不出去，设备仍停在上电默认通道。"""
        return EXPECTED_CHANNEL if self._chan_sent is None else self._chan_sent

    def _clear_channel_state(self) -> None:
        """换通道后把「上一个通道的痕迹」清掉。

        实时表里的旧数字必须清掉 —— 那是上一个通道的读数，留着会被当成
        新通道已经有数据。已采的 take **不动**：那是真实采到的数据，
        每一条自带 channel 列；真混了通道，由「汇总」表的通道列显形。
        """
        self._n_chan_ok = self._n_chan_bad = 0
        self._chan_seen = set()
        self.latest = {}
        self.live_dark = {}
        for lb in self.live_lbl.values():
            lb.setText("—")
            lb.setStyleSheet("font-family:Consolas,monospace;font-size:12px;"
                             "color:#999")
        for lb in self.live_net.values():
            lb.setText("—")
            lb.setStyleSheet("font-family:Consolas,monospace;font-size:14px;"
                             "font-weight:bold")

    def _warn_mixed_channel(self) -> None:
        """本次会话已采过别的通道时提醒 —— 一个 xlsx 混通道会把 G(λ) 带歪。"""
        done = {t.channel for t in self.takes if t.channel is not None}
        if done and self.channel() not in done:
            self._banner(
                f"⚠ 本次会话已有通道 {'、'.join(str(c) for c in sorted(done))} 的记录，"
                f"现在切到通道 {self.channel()}。"
                "一个 xlsx 里混多个通道会把 G(λ) 带歪 —— "
                "要么现在先保存再换，要么记得标定时只取一个通道。", ok=False)

    def _on_channel(self) -> None:
        """下拉框换通道：清状态、提醒混采、立即下发。"""
        self._clear_channel_state()
        self._warn_mixed_channel()
        self._send_brightness()
        self._refresh_chan_label()

    def _refresh_chan_label(self) -> None:
        """通道状态。先报「收没收到」，再报别的 —— 前者更要紧。"""
        ch = self.channel()
        if self._n_chan_bad and not self._n_chan_ok:
            seen = "、".join(str(c) for c in sorted(self._chan_seen))
            self.lbl_chan.setText(
                f"✗ 通道 {ch} 无数据：收到 {self._n_chan_bad} 帧，"
                f"都标着通道 {seen} —— 该通道没有回传。"
                "要么没发过它的启动帧、要么被停止帧关掉了、要么无硬件")
            self.lbl_chan.setStyleSheet("color:#c33;font-weight:bold")
            return
        if self._n_chan_bad:
            # 多通道同采是正常工况（通道累加），所以这里不是红色告警 ——
            # 别的通道的帧本来就该被丢掉，提示只是说明"你看到的是哪一路"。
            self.lbl_chan.setText(
                f"通道 {ch}：{self._n_chan_ok} 帧；"
                f"另有 {self._n_chan_bad} 帧属其他通道，未计入")
            self.lbl_chan.setStyleSheet("color:#2a6fb0")
            return
        self.lbl_chan.setText("")

    def _send_brightness(self) -> None:
        """把当前档位与通道下发给设备（经静默期合并后才会走到这里）。"""
        # ⚠️ 档位与通道共用**同一条指令帧**（byte5 / byte6），发不了一个就
        #    发不了另一个。所以 BRIGHTNESS_ADJUSTABLE 为假时通道也换不了 ——
        #    这不是耦合失误，是协议本身如此。
        if self.worker is None or not BRIGHTNESS_ADJUSTABLE:
            # 没连上发不出去。输入框和设备不一致，要让人看见。
            if self.worker is None and BRIGHTNESS_ADJUSTABLE:
                self._banner(
                    f"✗ 未连接，档位没发出去。设备仍在 "
                    f"0x{self.gear_in_effect():02X}（连接后会自动补发）",
                    ok=False)
            return
        if self.capturing is not None:
            # 采集中改档位/通道会让这一次的数据前后不一致。按说控件已禁用，
            # 这里是兜底 —— 真发生了要让人看见，而不是静默照发。
            self._banner("✗ 采集进行中，档位/通道未下发。要改请等本次采完。",
                         ok=False)
            return
        g, ch = self.spn_bright.value(), self.channel()
        self._send_cmd(build_brightness(g, ch), g, ch,
                       f"光源档位 0x{g:02X}、通道 {ch}")

    def _send_cmd(self, frame: bytes, gear: int, ch: int, what: str) -> bool:
        """把一条**已经造好**的指令帧发出去，并记下设备现在是什么状态。

        界面驱动的 `_send_brightness` 与两个固定帧按钮共用这一条路 ——
        否则「按钮发出去的帧」与「界面以为的状态」会分叉，
        之后随便动一下档位就会把按钮刚设的通道覆盖掉。
        """
        self.worker.send(frame)
        self._gear_sent, self._chan_sent = gear, ch
        # 记住这个通道的档位 —— 开关复选框重新勾上时要用它自己的那个，
        # 不是别人的。停止帧（档位写 0）不算"这个通道的档位"，不记。
        if frame[4] == CMD_START:
            self._gear_by_ch[ch] = gear
        # 复选框跟着这一帧走，不管这帧是谁造的（档位框、固定按钮、
        # 还是复选框自己）。否则界面上的勾选与真正发出去的字节会分叉。
        cb = self.chk_ch.get(ch)
        if cb is not None:
            cb.blockSignals(True)
            cb.setChecked(frame[4] == CMD_START)
            cb.blockSignals(False)
        self._banner(f"已下发 {what}　{frame.hex(' ').upper()}　"
                     "读数应随之变化；若没变，说明设备没接受这条指令。", ok=True)
        return True

    def _send_fixed(self, frame: bytes, gear: int, ch: int, name: str) -> None:
        """固定帧按钮：原样发出这 26 个字节，再把界面调到与之一致。

        ⚠️ 不走「改控件 → 触发下发」那条路：那条路会**重新造帧**，
           而重造出来的 byte4 恒为启动 —— 关闭帧会当场变回启动帧。
        """
        if self.worker is None:
            self._banner(f"✗ 未连接，「{name}」没发出去。", ok=False)
            return
        if self.capturing is not None:
            self._banner(f"✗ 采集进行中，「{name}」未下发。请等本次采完。",
                         ok=False)
            return
        prev_ch = self.channel()
        self._send_cmd(frame, gear, ch, name)
        # 界面跟上：**屏蔽信号**，否则 setValue/setCurrentIndex 会再触发一次
        # 下发，把刚发的固定帧覆盖掉。
        self.spn_bright.blockSignals(True)
        self.spn_bright.setValue(gear)
        self.spn_bright.blockSignals(False)
        self.lbl_gearhex.setText(f"= 0x{gear:02X}")   # 信号屏了，标签手动刷
        i = self.cmb_chan.findData(ch)
        if i >= 0:
            self.cmb_chan.blockSignals(True)
            self.cmb_chan.setCurrentIndex(i)
            self.cmb_chan.blockSignals(False)
        if ch != prev_ch:
            self._clear_channel_state()
            self._warn_mixed_channel()
        self._refresh_chan_label()

    def _on_chan_toggle(self, ch: int, on: bool) -> None:
        """某通道的开/关复选框。勾 → 启动帧；取消 → 停止帧。

        只作用在 byte6 指定的那一个通道上 —— 实测（2026-10-29，162704）：
        停 1 号时 2 号继续流，再发启动帧 1 号恢复。
        """
        cb = self.chk_ch[ch]

        def _revert(msg: str) -> None:
            # 发不出去就别让复选框假装生效 —— 它会被当成"设备状态"读。
            cb.blockSignals(True)
            cb.setChecked(not on)
            cb.blockSignals(False)
            self._banner(msg, ok=False)

        if self.worker is None:
            _revert(f"✗ 未连接，通道 {ch} 的"
                    f"{'启动' if on else '停止'}帧没发出去。")
            return
        if self.capturing is not None:
            _revert(f"✗ 采集进行中，通道 {ch} 的开关未下发。请等本次采完。")
            return
        g = self._gear_by_ch.get(ch, BRIGHTNESS_DEFAULT) if on else 0x00
        if not self._send_cmd(build_cmd(on, g, ch), g, ch,
                              f"通道 {ch} {'启动' if on else '停止'}"):
            _revert(f"✗ 通道 {ch} 的开关没发出去。")
            return
        self._refresh_chan_label()

    def _on_cmd_start_ch2(self) -> None:
        self._send_fixed(CMD_START_CH2, 0x64, 0x02, "启动通道2")

    def _on_cmd_stop_ch1(self) -> None:
        self._send_fixed(CMD_STOP_CH1, 0x00, 0x01, "关闭通道一")

    def toggle_run(self, on: bool) -> None:
        """保留的启停按钮 —— 当前固件上电即发数据，**不响应启停**。

        按下时只重发一次当前档位（那是唯一确认有效的指令）。
        """
        if self.worker is None:
            return
        self.btn_run.setText("■ 停止采集" if on else "▶ 开始采集")
        self.spn_bright.setEnabled(BRIGHTNESS_ADJUSTABLE and not on)
        self.cmb_chan.setEnabled(not on)
        if on:
            self._send_brightness()

    def on_failed(self, msg: str) -> None:
        QtWidgets.QMessageBox.critical(self, "串口", msg)
        if self.worker is not None:
            self.worker.stop(); self.worker = None
        self.btn_conn.setText("连接"); self.lbl_link.setText("未连接")
        self._abort_capture()

    def _n_active(self) -> int:
        """回传里还在发帧的通道数。**以回传为准**，不看我们下发过几个。"""
        st = getattr(self.worker, "stream", None)
        return getattr(st, "n_active", 1) if st is not None else 1

    def _cycle_hz(self) -> float:
        """实测扫描周期率（周期/秒）。判搏动必须用实测值 —— 按假设的帧率
        算会把心搏放到错误的频率上。拿不到返回 nan，上层据此跳过搏动判定。

        ⚠️ 必须除以活跃通道数。`_frames_seen` 在通道过滤**之前**就自增
           （见 on_frames），所以它数的是串口上的全部帧。多通道同采时
           设备把帧率成倍提高而每通道的周期率不变（2026-10-29 实测：
           双通道 15.6 帧/s，ch1 仍 0.656 周期/s）—— 只除 12 会成倍偏高。
        """
        t = time.perf_counter() - getattr(self, "_t0", time.perf_counter())
        n = getattr(self, "_frames_seen", 0)
        if t <= 0 or n <= 0:
            return float("nan")
        return n / t / frames_per_cycle(self._n_active())

    def _tick_rate(self) -> None:
        if self.worker is None:
            return
        fps = self._frames_seen - self._last_seen
        self._last_seen = self._frames_seen
        st = getattr(self.worker, "stream", None)
        extra = ""
        if st is not None and (st.n_dropped or st.n_bad_checksum):
            extra = f"  <span style='color:#c33'>丢弃 {st.n_dropped}B / 校验错 " \
                    f"{st.n_bad_checksum}</span>"
        na = self._n_active()
        self.lbl_link.setText(
            f"{fps} 帧/s（{fps/frames_per_cycle(na):.2f} 周期/s"
            + (f"，{na} 通道" if na > 1 else "") + f"）{extra}")
        self._tick_darkstat()
        self._refresh_chan_label()

    def _tick_darkstat(self) -> None:
        """暗时隙状态条 —— 取代原来的「尚无暗电平」提示。

        看三件事：暗帧还在不在、暗电平多少、它自己抖多少。
        第三项最关键：暗自身的 std 决定扣暗能扣多准，实测 474~588 是
        白噪声地板；明显更大就是链路在漂，这时候采的数据不能用。
        """
        recent = [f for _s, _t, f in list(self.raw_buf)[-DARK_STAT_FRAMES:]
                  if f.is_dark]
        if not recent:
            self.lbl_darkstat.setText(
                "暗时隙：<b style='color:#c33'>没收到</b> —— "
                "旧固件？本软件按每周期一帧 LED 全灭（波长码 0x00）解析")
            return
        parts = []
        worst_ok = True
        for rho in RHOS:
            v = np.array([f.adc for f in recent if f.distance_cm == rho], float)
            if v.size < 2:
                continue
            sd = float(v.std(ddof=1))
            worst_ok &= sd <= DARK_SD_WARN
            parts.append(f"{rho:.0f}cm {v.mean():>9,.0f} ±{sd:>5,.0f}")
        self.lbl_darkstat.setText(
            "暗时隙　" + "　｜　".join(parts)
            + ("　✓ 稳" if worst_ok else
               f"　<b style='color:#c33'>✗ 抖动 &gt;{DARK_SD_WARN:,}，"
               f"扣暗不准，先查干扰</b>"))

    # -- 采集 --------------------------------------------------------------
    def on_frames(self, frames: list) -> None:
        now = time.perf_counter() - self._t0
        do_print = self.chk_print.isChecked()
        for f in frames:
            self._frames_seen += 1
            self.raw_buf.append((self._frames_seen, now, f))
            if do_print:
                # 一个值一行，行尾附完整 26 字节（按协议字段分段）。
                # ADC 是有符号 int32，负值在遮光时是正常的。
                print(f"{self._frames_seen:>8d} | {f.wavelength_nm:>4.0f}nm |"
                      f" {f.distance_cm:.0f}cm | ADC={f.adc:>12,d} |"
                      f" {'OK ' if _crc_ok(f) else '校验错'} |"
                      f" {format_frame(f.raw)}", flush=True)
            # 通道校验：**以回传为准**（数据帧 byte10），不认我们下发了什么。
            #
            # 2026-10-28 实测：下发通道 2，292 帧回来 byte10 全是 01，
            # 十路读数与通道 1 相差都在 ±1% 内（就是两次采集的正常起伏）。
            # 若把它们当通道 2 收下，就会把一号通道的数据标成二号写进标定表 ——
            # 那比「没数据」糟得多：没数据看得见，标错的数据看不见。
            #
            # 原始帧照样进 raw_buf（上面已 append），丢的只是实时显示与入库，
            # 所以「原始帧…」窗口里仍能看到设备到底发了什么。
            self._chan_seen.add(f.channel)
            if f.channel != self.channel():
                self._n_chan_bad += 1
                continue
            self._n_chan_ok += 1
            self.latest[f.key] = f.adc
            self.live_lbl[f.key].setText(f"{f.adc:,}")
            # 实时扣暗显示 —— 与采集用的是同一个「本周期的暗」。
            # 这一路独立于 capturing，空闲时也要能看，否则没法判断该调哪个档位。
            if f.is_dark:
                self.live_dark[f.distance_cm] = f.adc
            else:
                d = self.live_dark.get(f.distance_cm)
                lb = self.live_net.get(f.key)
                if lb is not None:
                    if d is None:
                        lb.setText("等暗帧")
                        lb.setStyleSheet(lb.styleSheet().split(";color")[0]
                                         + ";color:#999")
                    else:
                        net = f.adc - d
                        lb.setText(f"{net:,}")
                        # 信号太弱或饱和一眼可见
                        col = ("#c33" if net < MIN_NET_COUNTS
                               else ("#d9a521" if f.adc > SAT_WARN else "#111"))
                        lb.setStyleSheet(
                            "font-family:Consolas,monospace;font-size:14px;"
                            f"font-weight:bold;color:{col}")
            if self.capturing is not None:
                # 暗时隙排在每个周期最前，先记下来，本周期后面的信号帧
                # 当场扣掉它。**必须按周期就近扣** —— 实测暗电平的漂移
                # 时间常数只有几个周期，隔几秒再扣等于没扣。
                if f.is_dark:
                    self.cycle_dark[f.distance_cm] = f.adc
                    self.dark_buffer.setdefault(f.distance_cm, []).append(f.adc)
                    self.order_buffer.append((0.0, f.distance_cm, f.adc, None))
                    continue
                d = self.cycle_dark.get(f.distance_cm)
                if d is None:
                    # 本周期还没收到暗帧（从周期中途开始接的），这一帧
                    # 扣不了暗，丢掉比带着未扣暗的值混进均值安全。
                    self.n_no_dark += 1
                    continue
                self.buffer.setdefault(f.key, []).append(f.adc - d)
                self.order_buffer.append((f.wavelength_nm, f.distance_cm,
                                          f.adc, f.adc - d))
                self.n_captured += 1
                if self.n_captured >= self.target_frames:
                    self._finish_capture()
                    break
        if self.capturing is not None:
            self.bar.setValue(int(100 * self.n_captured / self.target_frames))
            self.bar.setFormat(f"采集 {self.capturing} … "
                               f"{self.n_captured}/{self.target_frames} 帧")

    def start_capture(self, block: str) -> None:
        if self.worker is None:
            QtWidgets.QMessageBox.warning(self, "未连接", "请先连接串口。")
            return
        if self.capturing is not None:
            return
        # 本通道一帧都没收到时直接拦住。不拦的话进度条会停在 0% 不动
        # （on_frames 里这些帧根本不进缓冲），看着像死机，而真正的原因
        # ——「设备没切到这个通道」—— 反而被进度条盖住了。
        #
        # 用横幅而不是模态框 —— 与 _add_row 的判定提示同一个口径（见 _banner）。
        if self._n_chan_bad and not self._n_chan_ok:
            seen = "、".join(str(c) for c in sorted(self._chan_seen))
            self._banner(
                f"✗ 没开始采集：选的是通道 {self.channel()}，但收到的 "
                f"{self._n_chan_bad} 帧都标着通道 {seen}。"
                "设备没切到该通道，现在采只会采到空数据 —— "
                "先确认该通道有硬件、或切换指令是否被接受。", ok=False)
            return
        if self.simulate and isinstance(self.worker, SimWorker):
            self.worker.set_block(block)
            QtCore.QThread.msleep(250)          # 让模拟器切到该块
        self.capturing = block
        self.buffer = {}
        self.order_buffer = []
        self.dark_buffer = {}
        self.cycle_dark = {}
        self.n_no_dark = 0
        self.n_captured = 0
        self.target_frames = self.spn_frames.value()
        self._set_buttons_enabled(False)
        self.bar.setValue(0)

    def _abort_capture(self) -> None:
        self.capturing = None
        self.bar.setValue(0); self.bar.setFormat("空闲")
        self._set_buttons_enabled(True)

    def _finish_capture(self) -> None:
        block = self.capturing
        self.capturing = None
        self._set_buttons_enabled(True)
        self.bar.setValue(0); self.bar.setFormat("空闲")
        if block is None:
            return
        idx = sum(1 for t in self.takes if t.block == block) + 1
        tk = Take(block=block, index=idx,
                  timestamp=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                  brightness=(self.gear_in_effect() if BRIGHTNESS_ADJUSTABLE
                              else None),
                  # 记所选通道，不是 channel_in_effect() —— 进缓冲的每一帧
                  # 都经过 byte10 校验，所以这个值是**被数据证实过**的；
                  # 而「下发过什么」证实不了任何事。
                  channel=self.channel())
        tk.dark = {r: list(v) for r, v in self.dark_buffer.items()}
        # ⚠️ 2026-10-22：由算术均值改为**稳健聚合**（MAD 剔除离群后取均值）。
        #    实测在体归档每通道有 2~6% 的周期是尖峰、幅度约 25 倍；
        #    一个尖峰就能把该次采集的均值抬高数倍。选型依据见 robust.py。
        #    std 同样基于保留样本，与 mean 口径一致（否则 rel_std/sem 会虚高）。
        tk.rejected = {}
        for k, vals in self.buffer.items():
            ag = aggregate(vals)
            tk.mean[k] = ag.value
            tk.std[k] = ag.std
            tk.count[k] = ag.n_used
            tk.rejected[k] = ag.n_rejected
        tk.frames = list(self.order_buffer)
        # ⚠️ 不完整的采集**照样留下**。缺组合恰恰最需要原始帧来查原因
        #    （哪颗 LED 没亮？帧率够不够？），扔掉它就等于扔掉证据。
        #    标定时由「汇总」表把不完整的 take 排除在外，不会污染 G(λ)。
        if not tk.complete():
            miss = [f"{w:.0f}nm/{r:.0f}cm" for w in WLS for r in RHOS
                    if (w, r) not in tk.mean]
            self._banner(
                f"✗ {tk.block}：只收到 {len(tk.mean)}/{len(WLS)*len(RHOS)} 个组合，缺 "
                + "、".join(miss) +
                "。数据已留存（不进标定），可在原始帧窗口查原因："
                "帧数太少？某颗 LED 没亮？", ok=False)
        self.takes.append(tk)
        self._add_row(tk)
        self._refresh_block_styles()

    def _sync_frame_hint(self) -> None:
        """把「总帧数 → 每组合帧数」显式写在旁边。

        一个扫描周期是 (1 暗 + 5 波长) × 2 距离 = 12 帧，所以每组合只拿到
        总数的 1/12。标准误判据算的是每组合帧数，两者差 12 倍，
        不写出来必然看岔。
        """
        per = self.spn_frames.value() // FRAMES_PER_CYCLE
        self.lbl_percombo.setText(f"= 每组合 {per} 帧")

    def _banner(self, text: str, *, ok: bool) -> None:
        """非阻塞的判定提示。替代模态框 —— 见 _add_row 的注释。"""
        self.lbl_banner.setText(text)
        self.lbl_banner.setStyleSheet(
            "padding:7px;border-radius:4px;font-weight:bold;"
            + ("background:#e4f7ee;color:#0d6e4b;border:1px solid #1baf7a"
               if ok else
               "background:#fdeae3;color:#a33208;border:1px solid #eb6834"))
        self.lbl_banner.setVisible(True)

    def _set_buttons_enabled(self, on: bool) -> None:
        for b in self.blk_btn.values():
            b.setEnabled(on)
        self.btn_conn.setEnabled(on); self.btn_save.setEnabled(on)
        # 采集期间锁住档位：中途改亮度会让这一次的前后两段不是同一个条件，
        # 而 xlsx 里只记一个档位值 —— 数据看着正常，其实已经废了。
        self.spn_bright.setEnabled(BRIGHTNESS_ADJUSTABLE and on)
        # 通道同理，而且更严重：换通道是换一整套光学器件，
        # 中途换等于把两套器件的读数平均进同一个 take。
        self.cmb_chan.setEnabled(on)
        # btn_raw 故意不禁用 —— 采集过程中正是最该盯原始帧的时候

    def _add_row(self, tk: Take) -> None:
        rsd, sem, n, wr = tk.worst_noise()
        drift = wr < DRIFT_LIMIT
        need = 0
        if tk.block in INVIVO_CODES:
            # 在体：不套体模判据（心搏必然压低白噪声度、抬高标准误）。
            # 改判两件事：4cm 够不够亮、看不看得见心搏。
            cyc = self._cycle_hz()
            pul = tk.pulsatility(cyc)
            worst4 = min((m for (w, r), m in tk.mean.items() if r == max(RHOS)),
                         default=float("nan"))
            # 取显著度最高的那一路报（通常是 3cm，信噪最好）
            best = max(pul.values(), key=lambda v: v[3], default=None)
            acdc, bpm, prom = ((best[0], best[2], best[3]) if best
                               else (float("nan"),) * 3)
            if not tk.complete():
                verdict, color = f"不完整（{len(tk.mean)}/10 组合）", "#c33"
            elif np.isfinite(worst4) and worst4 < MIN_NET_COUNTS:
                verdict, color = f"4cm 最弱 {worst4:,.0f} ＜ {MIN_NET_COUNTS}", "#c33"
            elif not pul:
                verdict, color = "无法判搏动（周期率不足或帧太少）", "#b8860b"
            elif prom < PULSE_PROM_MIN:
                verdict, color = f"✗ 看不到心搏（显著度 {prom:.0f}）", "#c33"
            else:
                verdict, color = (f"✓ 在体（AC/DC {acdc:.2%}，{bpm:.0f} bpm）",
                                  "#1a8")
            self._arm_note = (worst4, acdc, bpm, prom, cyc)
        elif not tk.complete():
            verdict, color = f"不完整（{len(tk.mean)}/10 组合，不进标定）", "#c33"
        elif drift:
            verdict, color = "不合格 → 有漂移/振荡，加帧数无用", "#c33"
        elif np.isfinite(sem) and sem > SEM_USABLE:
            # 只补到「可用」档即可，不必追到 SEM_LIMIT —— 再往下收益很小
            need = int(np.ceil(n * (sem / SEM_USABLE) ** 2)) * len(WLS) * len(RHOS)
            verdict, color = f"不合格 → 重采 {need:,} 帧", "#c33"
        elif np.isfinite(sem) and sem > SEM_LIMIT:
            verdict, color = "可用（噪声与插值地板相当）", "#b8860b"
        else:
            verdict, color = "合格", "#1a8"
        r = self.table.rowCount(); self.table.insertRow(r)
        vals = [tk.block, str(tk.index), tk.timestamp, str(n),
                f"{rsd*100:.2f}%", f"{sem*100:.3f}%", f"{wr:.2f}", verdict]
        for c, v in enumerate(vals):
            it = QtWidgets.QTableWidgetItem(v)
            if c in (5, 6, 7):
                it.setForeground(QtGui.QBrush(QtGui.QColor(color)))
            self.table.setItem(r, c, it)
        self.table.scrollToBottom()
        # 判定用横幅，不用 QMessageBox。暗块本来就大概率不合格，
        # 每块弹一次模态框会把流程打断得没法做；而且模态框会阻塞事件循环，
        # 采集线程还在往里灌帧，无人点确定时整个界面就停在那儿。
        if tk.block in INVIVO_CODES:
            w4, acdc, bpm, prom, cyc = getattr(
                self, "_arm_note", (float("nan"),) * 5)
            if verdict.startswith("✓"):
                self._banner(
                    f"✓ {tk.block}：看到心搏（AC/DC {acdc:.2%}，{bpm:.0f} bpm，"
                    f"谱峰显著度 {prom:.0f}）"
                    f" —— 探头确实耦合在活体组织上。4cm 最弱 {w4:,.0f} 计数，"
                    f"周期率 {cyc:.1f} Hz。此次不进标定。", ok=True)
            else:
                self._banner(
                    f"{verdict}（{tk.block}）—— 在体采集不套体模判据。"
                    f"看不到心搏通常是贴得太松、位置不对，"
                    f"或者读到的根本不是穿过组织的光。", ok=False)
        elif drift:
            self._banner(
                f"✗ {tk.block}：白噪声度 {wr:.2f} ＜ {DRIFT_LIMIT} —— "
                "采集期间存在慢漂移/振荡，相邻帧高度相关，"
                "加帧数降不下来。先排除干扰再重采（原始帧窗口可看波形）", ok=False)
        elif verdict.startswith("可用"):
            self._banner(
                f"◐ {tk.block}：标准误 {sem*100:.3f}%，噪声贡献约 "
                f"{0.14*sem*100:.2f} %O₂ —— 与光谱插值的 0.21 %O₂ 地板相当。"
                "可以收下；想再降就加帧数，但收益有限。", ok=True)
        elif verdict.startswith("不合格"):
            self.spn_frames.setValue(min(self.spn_frames.maximum(), need))
            self._banner(
                f"✗ {tk.block}：最差组合标准误 {sem*100:.3f}% ＞ 限值 "
                f"{SEM_LIMIT*100:.1f}% —— 帧数已自动调到 {need:,}，"
                f"删掉这条重采即可", ok=False)
        else:
            self._banner(f"✓ {tk.block} 第 {tk.index} 次合格"
                         f"（标准误 {sem*100:.3f}%）", ok=True)

    def delete_selected(self) -> None:
        rows = sorted({i.row() for i in self.table.selectedIndexes()}, reverse=True)
        for r in rows:
            self.table.removeRow(r); del self.takes[r]
        self._refresh_block_styles()

    def _refresh_block_styles(self) -> None:
        done = 0
        for code, btn in self.blk_btn.items():
            n = sum(1 for t in self.takes if t.block == code)
            if n:
                done += 1
            btn.setText(f"{code}  ✓{n}" if n else code)
            btn.setStyleSheet("background:#d9f0e3;font-weight:bold" if n else "")
        n_rep = min((sum(1 for t in self.takes if t.block == c)
                     for c in phantom.BLOCK_CODES), default=0)
        self.lbl_prog.setText(
            f"已采 {done}/{len(phantom.BLOCK_CODES)} 块，"
            f"共 {len(self.takes)} 次记录"
            + (f"，每块至少 {n_rep} 次" if n_rep else
               "，<b>每块建议重贴 3 次</b>（分开贴合变异与器件变异）"))

    # -- 保存 --------------------------------------------------------------
    def save_xlsx(self) -> None:
        if not self.takes:
            QtWidgets.QMessageBox.information(self, "无数据", "还没有任何采集记录。")
            return
        default = _HERE / f"标定采集_{datetime.now():%Y%m%d_%H%M%S}.xlsx"
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "保存采集结果", str(default), "Excel (*.xlsx)")
        if not path:
            return
        try:
            n = write_workbook(Path(path), self.takes, simulate=self.simulate)
        except Exception as e:                                  # noqa: BLE001
            QtWidgets.QMessageBox.critical(self, "保存失败", str(e))
            return
        QtWidgets.QMessageBox.information(
            self, "已保存",
            f"{path}\n\n明细 {n} 行。\n\n接着跑标定：\n"
            f"  python -m srs_cal.run_calibration --xlsx \"{path}\"")

    def open_raw_view(self) -> None:
        """原始帧窗口。非模态，可以边采边看，也可以拖到第二块屏。"""
        from srs_cal.rawview import RawFrameViewer
        if self._raw_win is None:
            self._raw_win = RawFrameViewer(self, self.raw_buf)
            self._raw_win.finished.connect(lambda *_: setattr(self, "_raw_win", None))
        self._raw_win.show()
        self._raw_win.raise_()
        self._raw_win.activateWindow()

    def open_live_plot(self) -> None:
        """实时曲线窗口。同样非模态、同样只读 raw_buf，不影响采集。"""
        from srs_cal.liveplot import LivePlotWindow
        if self._plot_win is None:
            self._plot_win = LivePlotWindow(self, self.raw_buf)
            self._plot_win.finished.connect(
                lambda *_: setattr(self, "_plot_win", None))
        self._plot_win.show()
        self._plot_win.raise_()
        self._plot_win.activateWindow()

    def closeEvent(self, e) -> None:                            # noqa: N802
        if self._raw_win is not None:
            self._raw_win.close()
        if self._plot_win is not None:
            self._plot_win.close()
        if self.worker is not None:
            self.worker.stop()
        e.accept()


# ---------------------------------------------------------------------------
def _gear_key(g):
    """排序用：档位按数值升序，"固定" 这类字符串排最后。"""
    return (0, g) if isinstance(g, int) else (1, 0)


def _gear(tk: "Take"):
    """档位按**十进制**记（厂家口径是「150 个档位」，界面也是十进制）。

    写成数字而不是 "0x50" 字符串，是为了这一列能排序、能统计 ——
    不同块用不同档位时要能一眼看出高低。帧里那个字节的 16 进制写法
    在「元信息」里对照。
    """
    return "固定" if tk.brightness is None else int(tk.brightness)


def _chan(tk: "Take"):
    """通道号。None（2026-10-28 之前采的文件）记「未知」而不是填 1 ——
    那些数据当时没记通道，不能事后假设它是哪一个。"""
    return "未知" if tk.channel is None else int(tk.channel)


def write_workbook(path: Path, takes: list[Take], *, simulate: bool = False) -> int:
    """写 xlsx：明细 / 汇总 / 逐帧 / 元信息。返回明细行数。

    ⚠️ **信号值一律是扣暗后的** —— 采集时每个信号帧就减去了同一周期的
       暗时隙（波长码 0x00）。新固件的暗占原始读数 90%+，不扣算不了斜率。

       各表里暗的去向：
         明细   wavelength_nm = 0 的行 —— 暗时隙自身的均值/std/峰峰，
                不参与反演，是链路健康度指标
         汇总   不含暗（这张表是 run_calibration 的输入，只放信号）
         逐帧   wavelength_nm = 0 的行 —— 暗时隙的**原始**读数

    「汇总」表的列名与 `run_calibration` 的 CSV 格式一致，可直接被它读取；
    同一块有多次 take 时取各次均值，并额外给出 take 数与重贴合离散。
    """
    from openpyxl import Workbook
    from openpyxl.styles import Alignment, Font

    wb = Workbook()
    bold = Font(bold=True)

    ws = wb.active
    ws.title = "明细"
    ws.append(["block", "take", "timestamp", "brightness_gear", "channel",
               "wavelength_nm",
               "distance_cm", "adc_mean", "adc_std", "rel_std_pct", "sem_pct",
               "n_frames"])
    rows = 0
    for tk in takes:
        for w in WLS:
            for r in RHOS:
                if (w, r) not in tk.mean:
                    continue
                m, s = tk.mean[(w, r)], tk.std[(w, r)]
                nf = tk.count[(w, r)]
                ws.append([tk.block, tk.index, tk.timestamp, _gear(tk),
                           _chan(tk), w, r,
                           round(m, 2), round(s, 2),
                           round(100 * s / m, 4) if m > 0 else None,
                           round(100 * s / m / max(nf, 1) ** 0.5, 4) if m > 0 else None,
                           nf])
                rows += 1
        # 暗时隙单独记一行（wavelength_nm = 0）。信号列已经扣过暗，所以
        # 这一行不参与反演；它是**链路健康度**：暗电平自己的 std 和峰峰
        # 直接决定扣暗能扣得多准，出问题时第一个要看的就是它。
        for r, (m, s, pp) in sorted(tk.dark_stats().items()):
            nf = len(tk.dark.get(r, ()))
            ws.append([tk.block, tk.index, tk.timestamp, _gear(tk), _chan(tk),
                       0, r, round(m, 2), round(s, 2),
                       round(100 * s / m, 4) if m > 0 else None,
                       round(100 * s / m / max(nf, 1) ** 0.5, 4) if m > 0 else None,
                       nf])
            rows += 1

    ws2 = wb.create_sheet("汇总")
    # ⚠️ 「通道」列加在末尾：run_calibration 按**列名**找 block/wavelength_nm/
    #    I_3cm/I_4cm，末尾加列不影响它。
    #    这一列显示 "1/2" 就说明该块混了两个通道的 take —— G(λ) 已被带歪，
    #    不要用。本程序不替你拒绝（删哪一次是人的判断），但必须让它显形。
    ws2.append(["block", "wavelength_nm", "I_3cm", "I_4cm", "n_takes",
                "光源档位", "重贴合离散_3cm_%", "重贴合离散_4cm_%", "通道"])
    # 只有完整的 take 进「汇总」—— 这张表是 run_calibration 的输入，
    # 缺组合的数据进去会把 G(λ) 带歪。但它们在「明细」「逐帧」里都在。
    # 不含 DARK —— 新固件的暗是每周期的暗时隙，已在采集时扣掉；
    # 这张表只放信号，放进暗会被 run_calibration 当成第六个波长。
    for code in phantom.BLOCK_CODES:
        ts = [t for t in takes if t.block == code and t.complete()]
        if not ts:
            continue
        for w in WLS:
            v3 = [t.mean[(w, 3.0)] for t in ts if (w, 3.0) in t.mean]
            v4 = [t.mean[(w, 4.0)] for t in ts if (w, 4.0) in t.mean]
            if not v3 or not v4:
                continue
            a3, a4 = np.asarray(v3), np.asarray(v4)
            ws2.append([
                code, w, round(float(a3.mean()), 2), round(float(a4.mean()), 2),
                len(ts), "/".join(str(x) for x in sorted({_gear(t) for t in ts},
                                                         key=_gear_key)),
                round(float(a3.std(ddof=1) / a3.mean() * 100), 3) if a3.size > 1 else None,
                round(float(a4.std(ddof=1) / a4.mean() * 100), 3) if a4.size > 1 else None,
                "/".join(str(x) for x in sorted({_chan(t) for t in ts},
                                                key=_gear_key)),
            ])

    # ---- 逐帧原始值 ----
    # 标定本身只用均值，但逐帧值是事后一切复查的唯一依据：漂移、频谱、
    # 野点、重算判据。行数可控时直接进 xlsx；超了就落同名 CSV，避免
    # 把表撑得打不开。
    n_raw = sum(len(t.frames) for t in takes)
    raw_note = ""
    if n_raw <= RAW_SHEET_MAX:
        wsr = wb.create_sheet("逐帧")
        wsr.append(["block", "take", "seq", "wavelength_nm", "distance_cm",
                    "adc_raw", "adc_net"])
        for tk in takes:
            for i, (w, r, raw, net) in enumerate(tk.frames, 1):
                wsr.append([tk.block, tk.index, i, w, r, raw, net])
        raw_note = f"「逐帧」工作表，共 {n_raw:,} 行"
    else:
        side = path.with_name(path.stem + "_逐帧.csv")
        import csv as _csv
        with side.open("w", newline="", encoding="utf-8") as fh:
            wr = _csv.writer(fh)
            wr.writerow(["block", "take", "seq", "wavelength_nm",
                         "distance_cm", "adc_raw", "adc_net"])
            for tk in takes:
                for i, (w, r, raw, net) in enumerate(tk.frames, 1):
                    wr.writerow([tk.block, tk.index, i, w, r, raw, net])
        raw_note = f"{side.name}（{n_raw:,} 行，超过 {RAW_SHEET_MAX:,} 行未内嵌）"

    ws3 = wb.create_sheet("元信息")
    for k, v in [
        ("采集时间", datetime.now().strftime("%Y-%m-%d %H:%M:%S")),
        ("体模组", "MH Limited P3137034（12 块，认证 685/808/830/975 nm）"),
        ("体模折射率", f"{phantom.REFRACTIVE_INDEX}（厂家 2026-10 答复）"),
        ("器件波长 nm", "/".join(f"{w:.0f}" for w in WLS)),
        ("进反演波长 nm", "/".join(f"{w:.0f}" for w in hw.WAVELENGTHS_ACTIVE_NM)
            + "  —— 810 曾因暗电平未扣准被误判为泄漏，加暗时隙后恢复"),
        ("参考通道（不反演）", "/".join(f"{w:.0f}" for w in hw.WL_REFERENCE_ONLY_NM)
            + "  —— 组织侧水吸收占 ~40%"),
        ("源探距离 cm", "/".join(f"{r:.1f}" for r in RHOS)),
        ("扫描周期", f"(1 暗 + {len(WLS)} 波长) × {len(RHOS)} 距离 "
                     f"= {FRAMES_PER_CYCLE} 帧，暗时隙排在最前"),
        ("暗电平处理", "**逐周期就近扣暗** —— 每个信号帧减去同一周期的暗时隙。"
                       "「明细」「汇总」「逐帧」里的信号值都已扣暗；"
                       "「明细」中 wavelength_nm=0 的行是暗时隙本身（未扣）。"),
        ("光源档位", "本次使用 "
                 + "/".join(f"{g}（帧内 0x{g:02X}）" if isinstance(g, int)
                            else str(g)
                            for g in sorted({_gear(t) for t in takes}, key=_gear_key))
                 + f"　可调 0~{BRIGHTNESS_MAX}（厂家称 {BRIGHTNESS_MAX} 档，"
                   f"故 0 可能是关闭，待确认），上电默认 {BRIGHTNESS_DEFAULT}"
                 + ("" if BRIGHTNESS_ADJUSTABLE else "　当前固件不可调")),
        ("数据帧", "55 AA | 1A | 02 | 波长(00=暗时隙, 01~05=670/770/810/850/950) "
                   "| 距离 | ADC(大端有符号 int32) | 通道 | 14B 保留 "
                   "| 校验=sum(b[2:25])&0xFF"),
        ("指令帧", "55 AA | 1A | 01 | 启停(00停/01启) | 强度 | 通道 | 填充 | 校验　"
                   "（厂家协议文档：强度 0~100，通道 00=全部/01=一通道/02=二通道…；"
                   f"本软件档位上限仍放到 {BRIGHTNESS_MAX}，与文档不一致，待确认）"),
        ("采集通道", "本次使用 "
                 + "/".join(str(c) for c in sorted({_chan(t) for t in takes},
                                                   key=_gear_key))
                 + "　每一帧都按数据帧 byte10 校验过，不符的已丢弃未入库。"
                   "⚠️ 换通道 = 换一整套光学器件，G(λ) 不迁移；"
                   "「汇总」表通道列出现 \"1/2\" 即为混采，不可用于标定"),
        ("协议来源", "厂家 jhw-adc.py 参考实现 + 2026-10-10 答复 + "
                     "厂家通讯协议文档（指令帧数据位 0/1/2 = 启停/强度/通道）"),
        ("数据来源", "⚠️ 模拟器生成，非真实测量" if simulate else "真机串口采集"),
        ("逐帧值", raw_note + "　两列：adc_raw = 原始读数，"
                  "adc_net = 扣掉同周期暗时隙后的值（标定用的是这一列）；"
                  "波长 0 的行是暗时隙，没有 adc_net"),
        ("不完整的 take", f"{sum(1 for t in takes if not t.complete())} 次"
                          "（已留在「明细」「逐帧」，未进「汇总」）"),
    ]:
        ws3.append([k, v])

    for sheet in [x for x in (ws, ws2, ws3) if x is not None]:
        for c in sheet[1]:
            c.font = bold
            c.alignment = Alignment(horizontal="center")
        for col in sheet.columns:
            width = max(len(str(c.value)) if c.value is not None else 0 for c in col)
            sheet.column_dimensions[col[0].column_letter].width = min(60, width + 3)
    ws3.column_dimensions["A"].width = 22

    wb.save(path)
    return rows


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="SRS 体模标定数据采集")
    ap.add_argument("--simulate", action="store_true",
                    help="不连硬件，用体模正演模型造数据（排练 / 自测用）")
    ap.add_argument("--no-print", dest="print_frames", action="store_false",
                    default=PRINT_FRAMES_DEFAULT,
                    help="启动时不勾选「终端打印」（默认勾选）")
    ap.add_argument("--port", default=DEFAULT_PORT,
                    help=f"启动时自动连的串口，默认 {DEFAULT_PORT}")
    ap.add_argument("--gear", type=int, default=DEFAULT_GEAR,
                    help=f"启动时下发的光源档位，默认 {DEFAULT_GEAR}")
    ap.add_argument("--channel", type=int, default=EXPECTED_CHANNEL,
                    choices=range(CHANNEL_MIN, CHANNEL_MAX + 1),
                    help=f"启动时下发的采集通道，默认 {EXPECTED_CHANNEL}")
    ap.add_argument("--no-autostart", action="store_true",
                    help="不自动连口、不自动下发档位")
    args = ap.parse_args(argv)

    app = QtWidgets.QApplication(sys.argv[:1])
    win = MainWindow(simulate=args.simulate)
    win.auto_port, win.auto_gear = args.port, args.gear
    # 通道在连接**之前**就选好，这样 autostart 下发的那一帧里带的就是它，
    # 不必连上之后再发第二帧。
    i = win.cmb_chan.findData(args.channel)
    if i >= 0:
        win.cmb_chan.blockSignals(True)      # 还没连口，别触发下发
        win.cmb_chan.setCurrentIndex(i)
        win.cmb_chan.blockSignals(False)
    win.chk_print.setChecked(args.print_frames)
    if args.print_frames:
        print("  序号   |  波长  | 距离 |          ADC | 校验 |"
              " 帧头 长 型 波 距   ADC(4B)  通       保留(14B)        校验")
        print("-" * 118, flush=True)
    win.show()
    # 放在 show() 之后：开口失败要弹提示框，没有父窗口会弹不出来。
    if not args.no_autostart:
        QtCore.QTimer.singleShot(0, win.autostart)
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
