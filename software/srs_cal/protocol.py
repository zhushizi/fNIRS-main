# -*- coding: utf-8 -*-
"""样机串口协议 —— 26 字节定长数据帧的解析与流式拆包。

帧格式（由实机日志逐帧反推，7 帧全部对上，含校验）：

    偏移  长度  含义
    ----  ----  --------------------------------------------------
     0     2    帧头 0x55 0xAA
     2     1    包长，固定 0x1A = 26
     3     1    帧标识，0x02 = 数据帧
     4     1    波长码 0x00 = 暗时隙（LED 全灭）
                        0x01..0x05 → 670 / 770 / 810 / 850 / 950 nm
     5     1    距离码 0x01 = 3 cm，0x02 = 4 cm
     6     4    ADC 值，**大端有符号 int32**（补码）
    10     1    通道号，当前样机固定 0x01
    11    14    保留，实测全 0
    25     1    校验 = sum(bytes[2:25]) & 0xFF

校验的起点是**第 2 字节**（不含帧头 55 AA）—— 这一条是反推的：
按含帧头求和会恒差 0x55+0xAA=0xFF（即 −1），7 帧表现一致。

一个完整扫描周期 = (1 暗 + 5 波长) × 2 距离 = **12 帧**。
暗时隙排在最前（2026-10-10 固件新增；此前是 10 帧，无暗槽）。

⚠️ **ADC 是有符号数**（2026-10-09 实测确认）。遮住探头时 3 cm 通道读负值，
   原始字节形如 FF 9E 25 AA。按无符号解读会把 −6,879,318 变成 4,288,087,978，
   均值与标准差随之完全失真。判据不是猜的：实测序列
   +1,066,481 → −157,062 → −2,032,556 是**连续过零**的，
   只有有符号解读才连续；无符号下会从 106 万跳到 42.9 亿。
"""

from __future__ import annotations

from dataclasses import dataclass

HEADER = b"\x55\xAA"
FRAME_LEN = 26
FRAME_TYPE_DATA = 0x02

# 波长码 0x00 = **LED 全灭的暗时隙**（固件 2026-10-10 新增）。
# 每个扫描周期排在最前，两个探测器各一帧，所以一周期从 10 帧变 12 帧。
#
# ⚠️ 这一路不是波长，`WAVELENGTH_BY_CODE` 里刻意**不收录**它 ——
#    收录会让它被当成第六个波长混进反演。用 `Frame.is_dark` 判别。
#
# ⚠️ 新固件的暗电平是 **+85 万量级的正偏置**，占原始读数的 90~97%
#    （实测：暗 855,936，而 670/3cm 原始 885,624，扣暗后只剩 29,688）。
#    与旧固件的 −43,981 完全不同。**不扣暗算出的斜率是 0.016 而不是
#    0.843** —— 旧数据与新数据不能混用，必须按固件版本分开处理。
WL_DARK_CODE = 0x00

# 波长码 → 标称波长 nm
WAVELENGTH_BY_CODE: dict[int, float] = {
    0x01: 670.0, 0x02: 770.0, 0x03: 810.0, 0x04: 850.0, 0x05: 950.0,
}
CODE_BY_WAVELENGTH = {v: k for k, v in WAVELENGTH_BY_CODE.items()}

# 距离码 → 源探距离 cm
DISTANCE_BY_CODE: dict[int, float] = {0x01: 3.0, 0x02: 4.0}
CODE_BY_DISTANCE = {v: k for k, v in DISTANCE_BY_CODE.items()}

EXPECTED_CHANNEL = 0x01          # byte10。厂家参考实现不读这一字节，见下

# ---------------------------------------------------------------------------
# 指令帧（帧标识 0x01）—— 由厂家 jhw-adc.py 的 _build_cmd() 确认
# ---------------------------------------------------------------------------
#     [0:2] 55 AA     [2] 1A      [3] 01 指令帧
#     [4]   0x01      固定（厂家样例如此，含义未明，不要当"启停"用，见下）
#     [5]   光源档位  0x00 ~ 0x0F，**上电默认 0x0F**
#     [6]   通道      0x01
#     [7:25] 填充 0   [25] 校验（同数据帧）
#
# 厂家 2026-10-10 给的样例帧（控制光强）：
#     55 AA 1A 01 01 00 01 00 ... 00 1D      ← byte5=00
# 本模块 build_command(True, 0x00, 0x01) 逐字节复现该帧，校验 0x1D 对上。
#
# ⚠️ **设备上电即连续发数据，不需要、也不响应"开始"指令**（用户 2026-10-10
#    确认）。byte4 在样例里恒为 0x01，真实含义未知 —— 早先本文件把它注释成
#    "启停"是反推时的猜测，已证伪。现在一律照样例填 0x01。
#
# 档位范围（厂家 2026-10-10）：0x00 ~ **0x96**（十六进制，= 150），
# 上电默认 0x0F = 15，即上限的 **10%**。
#
# ⚠️ 由此**推翻**先前的判断：原以为 0x0F 是最亮（理由是默认档下 A2/B2 在
#    3cm 溢出），但默认 0x0F / 上限 0x96 说明设备只跑在 10%，**还能往上调
#    10 倍**。A2/B2 溢出是它们本身太亮，不是档位到顶。
#    直接后果：2026-10-09/10 的五次采集都在 10% 档位下完成，信号远未用满，
#    此前"4cm 太暗、670 的 SNR 只有 6"的结论大半源于此。
#
# ⚠️ 仍待确认：0x00 是「最暗」还是「关闭」。若是关闭，它就是现成的暗时隙，
#    不必等固件改 —— 可以先用软件切档位验证暗电平方案。
FRAME_TYPE_CMD = 0x01
CMD_START, CMD_STOP = 0x01, 0x00
BRIGHTNESS_MIN, BRIGHTNESS_MAX = 0x00, 0x96
BRIGHTNESS_DEFAULT = 0x0F        # 上电默认，= 上限的 10%

# ⚠️ **命名口径与厂家不同，对接时按字节位置说，别按名字说：**
#     byte4  厂家叫「通道号 01~05」 —— 实际是**波长** 670/770/810/850/950
#     byte5  厂家叫「编号 01~02」   —— 实际是**探测器距离** 3cm / 4cm
#     byte10 厂家参考实现**完全不读**；实测恒为 0x01，按采集通道处理


def checksum(frame: bytes) -> int:
    """协议校验：第 2 字节到第 24 字节求和取低 8 位。"""
    return sum(frame[2:FRAME_LEN - 1]) & 0xFF


@dataclass(frozen=True)
class Frame:
    wavelength_nm: float
    distance_cm: float
    adc: int
    channel: int
    raw: bytes

    @property
    def key(self) -> tuple[float, float]:
        return (self.wavelength_nm, self.distance_cm)

    @property
    def is_dark(self) -> bool:
        """LED 全灭时隙。`wavelength_nm` 为 0.0，不是真波长。"""
        return self.wavelength_nm == 0.0


def parse_frame(buf: bytes) -> Frame | None:
    """解析一个完整的 26 字节帧。任何一项不合法返回 None。

    暗时隙（波长码 0x00）解析为 `wavelength_nm = 0.0`，用 `is_dark` 判别。
    """
    if len(buf) != FRAME_LEN or not buf.startswith(HEADER):
        return None
    if buf[2] != FRAME_LEN or buf[3] != FRAME_TYPE_DATA:
        return None
    if checksum(buf) != buf[FRAME_LEN - 1]:
        return None
    wl = 0.0 if buf[4] == WL_DARK_CODE else WAVELENGTH_BY_CODE.get(buf[4])
    rho = DISTANCE_BY_CODE.get(buf[5])
    if wl is None or rho is None:
        return None
    return Frame(wavelength_nm=wl, distance_cm=rho,
                 adc=int.from_bytes(buf[6:10], "big", signed=True),
                 channel=buf[10], raw=bytes(buf))


class FrameStream:
    """字节流 → 帧。按帧头同步，容忍粘包/半包/垃圾字节。

    `feed()` 返回本次喂入后新解出的帧。丢弃的字节计入 `n_dropped`，
    校验失败计入 `n_bad_checksum` —— 这两个计数就是链路质量指标，
    标定时若不为零应当停下来查线，不要带病采数。
    """

    MAX_BUFFER = 64 * 1024

    def __init__(self) -> None:
        self._buf = bytearray()
        self.n_frames = 0
        self.n_dropped = 0
        self.n_bad_checksum = 0

    def reset(self) -> None:
        self._buf.clear()
        self.n_frames = self.n_dropped = self.n_bad_checksum = 0

    def feed(self, data: bytes) -> list[Frame]:
        self._buf.extend(data)
        if len(self._buf) > self.MAX_BUFFER:        # 异常流，别无限涨
            self.n_dropped += len(self._buf) - self.MAX_BUFFER
            del self._buf[:len(self._buf) - self.MAX_BUFFER]

        out: list[Frame] = []
        while True:
            i = self._buf.find(HEADER)
            if i < 0:
                # 保留末尾 1 字节，防止帧头被切在两次 feed 之间
                if len(self._buf) > 1:
                    self.n_dropped += len(self._buf) - 1
                    del self._buf[:-1]
                break
            if i > 0:
                self.n_dropped += i
                del self._buf[:i]
            if len(self._buf) < FRAME_LEN:
                break
            cand = bytes(self._buf[:FRAME_LEN])
            fr = parse_frame(cand)
            if fr is None:
                # 可能是数据里恰好出现 55 AA。只吃掉这一个帧头再找下一个。
                if cand[2] == FRAME_LEN and cand[3] == FRAME_TYPE_DATA:
                    self.n_bad_checksum += 1
                del self._buf[:2]
                self.n_dropped += 2
                continue
            del self._buf[:FRAME_LEN]
            self.n_frames += 1
            out.append(fr)
        return out


def split_fields(raw: bytes) -> list[str]:
    """按协议边界把 26 字节切成 9 段，便于肉眼核对。

    返回 [帧头, 长度, 类型, 波长, 距离, ADC(4B), byte10, 保留(14B), 校验]。
    分段显示比一整串 52 个十六进制字符好认得多 —— 字段错位一眼就能看出来。
    """
    return [raw[0:2].hex().upper(), f"{raw[2]:02X}", f"{raw[3]:02X}",
            f"{raw[4]:02X}", f"{raw[5]:02X}", raw[6:10].hex().upper(),
            f"{raw[10]:02X}", raw[11:FRAME_LEN - 1].hex().upper(),
            f"{raw[FRAME_LEN - 1]:02X}"]


def format_frame(raw: bytes) -> str:
    """分段后的一行十六进制。"""
    return " ".join(split_fields(raw))


def build_brightness(gear: int) -> bytes:
    """光源档位指令。gear = 0x00 ~ 0x0F，**上电默认 0x0F（最亮）**。

    这是目前唯一确认有效的指令。生成的帧与厂家 2026-10-10 给的样例一致：
        build_brightness(0x00) == 55 AA 1A 01 01 00 01 00 … 00 1D

    ⚠️ 对 SRS 而言改档位是**安全**的：光强整体乘一个因子 B 时，
       A = −log10(B·I) = −log10(B) − log10(I)，两个探测器同减一个常数，
       **空间斜率完全不变**。所以不同体模块可以用不同档位迁就动态范围，
       不影响 G(λ)。但同一块的一次采集期间不要改。

       前提有二，都**尚未验证**：
       ① 五路 LED 的光输出随档位**等比例**变化。若各 LED 的电流-光强曲线
          不同，改档位会引入波长相关的变化 —— 直接进 TOI。
       ② 档位不改变 LED 峰值波长。文献上 LED 波长随驱动电流和结温漂移
          （0.1 nm/K 量级，调电流比 PWM 漂得多），而波长漂移同样是波长
          相关误差。须问厂家档位是调电流还是调 PWM 占空比。

       验证办法：同一块体模、探头不动，在两三个档位各采一次，比 G(λ)。
       波长相关的差异应 < 0.005。
    """
    b = bytearray(FRAME_LEN)
    b[0:2] = HEADER
    b[2] = FRAME_LEN
    b[3] = FRAME_TYPE_CMD
    b[4] = 0x01                    # 厂家样例固定值，含义未明
    b[5] = max(BRIGHTNESS_MIN, min(BRIGHTNESS_MAX, int(gear)))
    b[6] = EXPECTED_CHANNEL
    b[FRAME_LEN - 1] = checksum(bytes(b))
    return bytes(b)


def build_command(start: bool, brightness: int = BRIGHTNESS_DEFAULT,
                  channel: int = EXPECTED_CHANNEL) -> bytes:
    """旧接口，保留给既有调用方。byte4 现在恒为 0x01。

    ⚠️ `start` 参数已**无作用** —— 设备上电即发数据、不响应启停（见文件头）。
       新代码请直接用 `build_brightness()`，意图更清楚。
    """
    return build_brightness(brightness)


def build_frame(wavelength_nm: float, distance_cm: float, adc: int,
                channel: int = EXPECTED_CHANNEL) -> bytes:
    """造一个合法帧 —— 给模拟器和单元测试用。

    `wavelength_nm = 0.0` 造暗时隙帧（波长码 0x00）。
    """
    b = bytearray(FRAME_LEN)
    b[0:2] = HEADER
    b[2] = FRAME_LEN
    b[3] = FRAME_TYPE_DATA
    b[4] = (WL_DARK_CODE if wavelength_nm == 0.0
            else CODE_BY_WAVELENGTH[wavelength_nm])
    b[5] = CODE_BY_DISTANCE[distance_cm]
    b[6:10] = int(adc).to_bytes(4, "big", signed=True)
    b[10] = channel
    b[FRAME_LEN - 1] = checksum(bytes(b))
    return bytes(b)
