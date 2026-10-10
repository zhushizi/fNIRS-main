# -*- coding: utf-8 -*-
"""体模标定数据采集 —— 12 块各一个按钮，结果存 xlsx 供 run_calibration 使用。

    python -m srs_cal.acquire                # 连真机
    python -m srs_cal.acquire --simulate     # 无硬件，用正演模型造数据排练

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
from srs_cal.protocol import (                # noqa: E402
    BRIGHTNESS_MAX, BRIGHTNESS_MIN, DISTANCE_BY_CODE, EXPECTED_CHANNEL,
    BRIGHTNESS_DEFAULT, WAVELENGTH_BY_CODE, Frame, FrameStream,
    build_brightness, build_frame, format_frame)

WLS = [WAVELENGTH_BY_CODE[c] for c in sorted(WAVELENGTH_BY_CODE)]     # 670..950
RHOS = [DISTANCE_BY_CODE[c] for c in sorted(DISTANCE_BY_CODE)]        # 3.0, 4.0
# 旧固件时代「人工遮死探头」采的那种暗，块名就叫 DARK。
# 新固件每周期自带暗时隙，软件不再**产生**这种记录，但仍要能**读**
# 旧 xlsx（run_srs_v1 会用到），所以常量保留。
DARK_CODE = "DARK"
# 一个扫描周期的帧数：(1 暗槽 + 5 波长) × 2 距离。
FRAMES_PER_CYCLE = (1 + len(WLS)) * len(RHOS)
# 实时表的配色阈值：扣暗后低于此值标红（srs.py 的 MIN_COUNTS 是 2000），
# 原始值高于此值标黄（接近有符号 int32 上限的量级，疑似饱和）。
MIN_NET_COUNTS = 2000
SAT_WARN = 2_000_000_000
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
    mean: dict = field(default_factory=dict)
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
                data = ser.read(4096)
                if data:
                    fr = self.stream.feed(data)
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
        """模拟器忽略指令，只为与 SerialWorker 接口一致。"""

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
                else:
                    i = phantom.BLOCK_CODES.index(blk)
                    mua, musp = phantom.interpolate_to(wl)
                    cache[blk] = intensity_matrix(mua[i], musp[i], rho,
                                                  phantom.REFRACTIVE_INDEX,
                                                  scale=SIM_SCALE)
            base = cache[blk]
            # 暗时隙：每周期最前，两个探测器各一帧。模拟一个带慢漂移的
            # 大正偏置 —— 真机实测约 +85 万，占原始读数的 90%+。
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
        self.channel_warned = False
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
            "改动立即下发指令帧 55 AA 1A 01 01 <档位> 01 …\n\n"
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
                  self.lbl_gearhex, self.btn_run,
                  self.chk_print,
                  QtWidgets.QLabel("  每次采集"), self.spn_frames,
                  self.lbl_percombo):
            bar.addWidget(w)
        bar.addStretch(1)
        self.lbl_link = QtWidgets.QLabel("未连接")
        self.lbl_link.setStyleSheet("color:#888")
        bar.addWidget(self.lbl_link)
        root.addLayout(bar)
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
        self.lbl_darkstat = QtWidgets.QLabel("暗时隙：等待数据…")
        self.lbl_darkstat.setStyleSheet(
            "padding:6px;border:1px solid #cfe0f5;border-radius:4px;"
            "background:#f2f7fd;color:#2a78d6;font-family:Consolas,monospace")
        bl.addWidget(self.lbl_darkstat, 5, 0, 1, 4)
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
        self.lbl_prog = QtWidgets.QLabel("已采 0/12 块")
        bot.addWidget(self.lbl_prog); bot.addStretch(1)
        bot.addWidget(self.btn_plot); bot.addWidget(self.btn_raw)
        bot.addWidget(self.btn_del)
        bot.addWidget(self.btn_save)
        root.addLayout(bot)

        self.refresh_ports()
        self._refresh_block_styles()

    # -- 串口 --------------------------------------------------------------
    def refresh_ports(self) -> None:
        self.cmb_port.clear()
        try:
            from serial.tools import list_ports
            for p in list_ports.comports():
                self.cmb_port.addItem(f"{p.device} — {p.description}", p.device)
        except Exception:                                       # noqa: BLE001
            pass
        if self.cmb_port.count() == 0:
            self.cmb_port.addItem("未发现串口", None)

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

    def _send_brightness(self) -> None:
        """把当前档位下发给设备（经静默期合并后才会走到这里）。"""
        if self.worker is None or not BRIGHTNESS_ADJUSTABLE:
            # 没连上发不出去。输入框和设备不一致，要让人看见。
            if self.worker is None and BRIGHTNESS_ADJUSTABLE:
                self._banner(
                    f"✗ 未连接，档位没发出去。设备仍在 "
                    f"0x{self.gear_in_effect():02X}（连接后会自动补发）",
                    ok=False)
            return
        if self.capturing is not None:
            # 采集中改档位会让这一次的数据前后不一致。按说框已禁用，
            # 这里是兜底 —— 真发生了要让人看见，而不是静默照发。
            self._banner("✗ 采集进行中，档位未下发。换档位请等本次采完。",
                         ok=False)
            return
        g = self.spn_bright.value()
        frame = build_brightness(g)
        self.worker.send(frame)
        self._gear_sent = g
        self._banner(f"已下发光源档位 0x{g:02X}　{frame.hex(' ').upper()}　"
                     "读数应随之变化；若没变，说明设备没接受这条指令。", ok=True)

    def toggle_run(self, on: bool) -> None:
        """保留的启停按钮 —— 当前固件上电即发数据，**不响应启停**。

        按下时只重发一次当前档位（那是唯一确认有效的指令）。
        """
        if self.worker is None:
            return
        self.btn_run.setText("■ 停止采集" if on else "▶ 开始采集")
        self.spn_bright.setEnabled(BRIGHTNESS_ADJUSTABLE and not on)
        if on:
            self._send_brightness()

    def on_failed(self, msg: str) -> None:
        QtWidgets.QMessageBox.critical(self, "串口", msg)
        if self.worker is not None:
            self.worker.stop(); self.worker = None
        self.btn_conn.setText("连接"); self.lbl_link.setText("未连接")
        self._abort_capture()

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
        self.lbl_link.setText(f"{fps} 帧/s（{fps/FRAMES_PER_CYCLE:.1f} 周期/s）{extra}")
        self._tick_darkstat()

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
            if f.channel != EXPECTED_CHANNEL and not self.channel_warned:
                self.channel_warned = True
                QtWidgets.QMessageBox.warning(
                    self, "通道号异常",
                    f"收到通道号 {f.channel}，预期 {EXPECTED_CHANNEL}。\n"
                    "当前样机应只有一个采集通道，请确认硬件配置。")
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
                              else None))
        tk.dark = {r: list(v) for r, v in self.dark_buffer.items()}
        for k, vals in self.buffer.items():
            a = np.asarray(vals, dtype=float)
            tk.mean[k] = float(a.mean())
            tk.std[k] = float(a.std(ddof=1) if a.size > 1 else 0.0)
            tk.count[k] = int(a.size)
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
        # btn_raw 故意不禁用 —— 采集过程中正是最该盯原始帧的时候

    def _add_row(self, tk: Take) -> None:
        rsd, sem, n, wr = tk.worst_noise()
        drift = wr < DRIFT_LIMIT
        need = 0
        if not tk.complete():
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
        if drift:
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
    ws.append(["block", "take", "timestamp", "brightness_gear", "wavelength_nm",
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
                ws.append([tk.block, tk.index, tk.timestamp, _gear(tk), w, r,
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
            ws.append([tk.block, tk.index, tk.timestamp, _gear(tk),
                       0, r, round(m, 2), round(s, 2),
                       round(100 * s / m, 4) if m > 0 else None,
                       round(100 * s / m / max(nf, 1) ** 0.5, 4) if m > 0 else None,
                       nf])
            rows += 1

    ws2 = wb.create_sheet("汇总")
    ws2.append(["block", "wavelength_nm", "I_3cm", "I_4cm", "n_takes",
                "光源档位", "重贴合离散_3cm_%", "重贴合离散_4cm_%"])
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
        ("指令帧", f"55 AA | 1A | 01 | 01 | 档位(0x00~0x{BRIGHTNESS_MAX:02X}) "
                   "| 通道 01 | 填充 | 校验　"
                   "（设备上电即发数据，不响应启停；只有档位指令有效）"),
        ("协议来源", "厂家 jhw-adc.py 参考实现 + 2026-10-10 答复，逐字段核对一致"),
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
    args = ap.parse_args(argv)

    app = QtWidgets.QApplication(sys.argv[:1])
    win = MainWindow(simulate=args.simulate)
    win.chk_print.setChecked(args.print_frames)
    if args.print_frames:
        print("  序号   |  波长  | 距离 |          ADC | 校验 |"
              " 帧头 长 型 波 距   ADC(4B)  通       保留(14B)        校验")
        print("-" * 118, flush=True)
    win.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
