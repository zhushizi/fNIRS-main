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

import time
from dataclasses import dataclass, replace

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

EXPECTED_CHANNEL = 0x01          # byte10。默认通道，也是单通道时的那一个

# 采集通道。指令帧 byte6 写它，数据帧 byte10 回它。
#
# 2026-10-29 实测确认（此前本处写「未经任何回传数据证实」，已不成立）：
#   · 下发 byte6=0x02 后，数据帧的 byte10 确实出现 0x02；
#   · 通道是**累加**的 —— 开了 2 号，1 号仍在跑，一个完整扫描周期
#     变成 24 帧（每通道 12 帧），按波长交错；
#   · 设备把帧率翻了倍（7.9 → 15.6 帧/s），每个通道自己的周期率不变
#     （恒 0.656 周期/s）—— 即不是分时间，是真翻倍。
#
# ⚠️ 仍未确认：byte6=0x03 是「1 号+2 号」（位或）还是「3 号通道」（序号）。
#    厂家文档只写「0x00 全部 / 0x01 一通道 / 0x02 二通道…累加依次」，
#    两种读法都讲得通。要问厂家。
#    *不影响逐个开关* —— 逐个下发 0x01/0x02 累加、`byte4=0x00` 按通道停，
#    这两条都已实测可行。
CHANNEL_MIN, CHANNEL_MAX = 0x01, 0x04
CHANNELS_ALL = tuple(range(CHANNEL_MIN, CHANNEL_MAX + 1))

# ---------------------------------------------------------------------------
# 扫描周期（2026-10-22 新增，为 t_host / cycle_id 服务）
# ---------------------------------------------------------------------------
# 一周期 = (1 暗 + 5 波长) × 2 距离。**由本文件的协议表自身推导**，
# 因为周期结构是固件定的，协议层才是它的权威出处。
#
# ⚠️ acquire.py:66 另有一个同名常量，由它自己的 WLS/RHOS 算出。两者**应当相等**；
#    若不等，说明采集侧期望的波长集与设备实际发送的不一致 —— 那本身就是个信号。
#    本模块不 import acquire（层次颠倒），故刻意不合并。
#
# ⚠️ 这是**单个通道**一周期的帧数。多通道同采时通道是累加的，串口上一个
#    完整周期是 `FRAMES_PER_CYCLE × 活跃通道数` 帧 —— 拿帧率换算周期率时
#    必须用下面的 `frames_per_cycle()`，直接除这个常量会**按通道数成倍偏高**。
FRAMES_PER_CYCLE = (1 + len(WAVELENGTH_BY_CODE)) * len(DISTANCE_BY_CODE)


def frames_per_cycle(n_active: int = 1) -> int:
    """串口上一个完整扫描周期的帧数 = 单通道帧数 × 活跃通道数。

    2026-10-29 实测：开通道 2 后帧率 7.9 → 15.6 帧/s，而每个通道自己的
    周期率不变（恒 0.656 周期/s）。所以「帧/s ÷ 12」在双通道下给出 1.3 周期/s，
    真值是 0.656 —— 偏高整整一倍。活跃通道数必须参与换算。

    `n_active` 应当取**回传里真的见过帧**的通道数（见 `FrameStream.n_active`），
    不是下发过几个通道 —— 下发了不等于设备开了。
    """
    return FRAMES_PER_CYCLE * max(1, int(n_active))

# 周期起点标志：暗时隙 + 最近的那个距离（实测帧序 (0,3cm) → (0,4cm) → (670,3cm) …）
CYCLE_START_DISTANCE_CM = min(DISTANCE_BY_CODE.values())

# ---------------------------------------------------------------------------
# 指令帧（帧标识 0x01）—— 由厂家 jhw-adc.py 的 _build_cmd() 确认
# ---------------------------------------------------------------------------
#     [0:2] 55 AA     [2] 1A      [3] 01 指令帧
#     [4]   启 / 停    0x01 启动 / 0x00 停止        ← 厂家通讯协议文档
#     [5]   光源强度  文档写 0~100（0x00~0x64），本模块上限仍 0xFF，见下
#     [6]   通道选择  0x00 全部 / 0x01 一通道 / 0x02 二通道…「累加依次」
#     [7:25] 填充 0   [25] 校验（同数据帧）
#
# 厂家 2026-10-10 给的样例帧（控制光强）：
#     55 AA 1A 01 01 00 01 00 ... 00 1D      ← byte5=00
# 本模块 build_cmd(True, 0x00, 0x01) 逐字节复现该帧，校验 0x1D 对上。
#
# ⚠️ **2026-10-29 改口：byte4 就是启停位**，由厂家通讯协议文档确认。
#    此前这里写「含义未明，早先猜是启停，已证伪」——「已证伪」是错的。
#    当时的证据是「设备上电即发数据，不发启动也有数」，这只说明启动
#    **不是必需的**，推不出「这一位不是启停位」。把"不必要"读成"无此功能"，
#    是这个项目里最贵的一类错误。
#    实测（2026-10-29，162704）：byte4=0x00 + byte6=0x01 只停 1 号通道，
#    2 号继续流；再发启动帧，1 号恢复。
#
# ⚠️ 档位上限口径不一致，**未解决**：
#      厂家 2026-10-10 口头说 0x00~0x96（150）；
#      厂家通讯协议文档写 0~100（0x64）；
#      本模块 BRIGHTNESS_MAX 仍是 0xFF（255）。
#    实测档位 255 的读数约为 100 的 2 倍 —— 设备**没有**在 100 截断，
#    所以也不能照文档直接把上限改成 0x64。要厂家给准话。
#
# ⚠️ 仍待确认：0x00 是「最暗」还是「关闭」。现在有了真正的停止位（byte4=0x00），
#    这条的紧迫性已降低 —— 停就发停止帧，不必靠把档位调到 0。
FRAME_TYPE_CMD = 0x01
CMD_START, CMD_STOP = 0x01, 0x00
# 档位占指令帧的**一个字节**（build_brightness 的 b[5]），所以物理上限就是
# 0xFF = 255。要再高必须改协议（换两字节或换字段），这是厂家的事。
# ⚠️ 2026-10-13 曾要求设到 300、256 —— 都越界：300 会抛 ValueError，
#    256 按模 256 变成 0（最暗），两种都不是想要的结果。
BRIGHTNESS_BYTE_MAX = 0xFF
BRIGHTNESS_MIN, BRIGHTNESS_MAX = 0x00, BRIGHTNESS_BYTE_MAX
BRIGHTNESS_DEFAULT = 0x0F        # 上电默认，= 上限的 10%

# ⚠️ **命名口径与厂家不同，对接时按字节位置说，别按名字说：**
#     byte4  厂家叫「通道号 01~05」 —— 实际是**波长** 670/770/810/850/950
#     byte5  厂家叫「编号 01~02」   —— 实际是**探测器距离** 3cm / 4cm
#     byte10 厂家参考实现**完全不读**；本模块按采集通道处理 ——
#            2026-10-29 实测开 2 号通道后它确实回 0x02，这条已被回传证实


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

    # --- 2026-10-22 新增。两者都由 **上位机** 补，设备不发这些 ---------------
    #
    # t_host：收到这一帧时的 `time.perf_counter()`。
    #   ⚠️ 设备侧 26 字节里**没有任何时间字段**（见文件头的帧格式表），
    #      而 FrameStream 是全链路唯一知道"字节何时到达"的地方 ——
    #      帧一旦交给上层，时间信息就永久丢失，事后无法补。
    #      没有它就没有采样率：2026-10-13~14 的 6 组在体采集，靠 take 时间戳
    #      倒推出 0.67 / 1.02 / 12.50 / 25.00 周期每秒，**跨度 37 倍**，
    #      任何按 Hz 指定的滤波都无从谈起（MBLL 因此完全做不了）。
    #   精度：串口按块读，同一批解出的帧共用一个时刻，批内误差在微秒级，
    #      对 ~12 周期/秒的扫描率足够。要更准需厂家在 byte11:25 的
    #      14 字节保留区里打设备侧时间戳。
    #
    # cycle_id：本帧属于第几个扫描周期，从 0 开始。
    #   用途一：让"逐周期扣暗"可验证。现在 online.py 用的是**最近一次**暗值
    #      （`self.cycle_dark[距离]`），不是**同周期**暗值 —— 不丢帧时两者
    #      等价，一旦丢帧（n_dropped 不为零是实际发生过的），信号帧会被
    #      上一周期的暗值扣掉，而且没有任何提示。
    #   用途二：五个波长是周期内**依次**点亮的，670 与 770 差几十毫秒。
    #      算慢变化无所谓，算心搏带时该进计算。
    #
    # 两者都有默认值，所有既有调用无需改动。
    t_host: float = 0.0
    cycle_id: int = -1

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

    2026-10-22 起还负责给每帧补 `t_host` 和 `cycle_id`（见 `Frame` 的说明）。
    周期计数**不假定每周期 12 帧**，而是实测计数后与 `FRAMES_PER_CYCLE`
    比对，不符的计入 `n_short_cycles`。这样固件改了帧结构也不会静默出错。

    ⚠️ 2026-10-29：周期跟踪改成**按通道各记一套**。
       多通道同采时通道是累加的、按波长交错，串口上一个周期是
       12 × 通道数 帧。原先用一个全局计数器与 12 相比，双通道下
       **几乎每个周期都被记成短周期**（实测 51/52、127/128、93/94、
       129/130），质量层因此持续报假警。
       对外的 `cycle_id` / `n_cycles` / `n_short_cycles` 语义保持不变 ——
       单通道时与改造前逐位相同。
    """

    MAX_BUFFER = 64 * 1024

    # 判「这个通道还活着」的静默上限（秒）。取 5 s ≈ 3 个扫描周期：
    # 比周期长到不会被一次抖动误判，又比操作者点开关的反应时间短。
    ACTIVE_WINDOW_S = 5.0

    def __init__(self, clock=time.perf_counter) -> None:
        self._buf = bytearray()
        self.n_frames = 0
        self.n_dropped = 0
        self.n_bad_checksum = 0
        # --- 周期跟踪（按通道各一套）----------------------------------------
        self._clock = clock          # 可注入，便于自检里用确定性时钟
        # 通道号 -> [cycle_id, n_cycles, n_short, in_cycle, 最后一帧的 t_host]
        self._cyc: dict[int, list] = {}
        self._t_last = 0.0           # 最近一帧的 t_host，供 n_active 判静默
        self._last_ch = EXPECTED_CHANNEL   # 最近一帧的通道，供 cycle_id 兜底

    # -- 周期计数的对外口径 --------------------------------------------------
    # 单通道时这三个与改造前完全一致（只有一个通道在 _cyc 里）。
    # 多通道时：cycle_id 取最近一帧所属通道的（逐帧的那个值在 Frame 上，
    # 这里只是兜底），n_cycles / n_short_cycles 取各通道之和。
    @property
    def cycle_id(self) -> int:
        return self._cyc[self._last_ch][0] if self._cyc else -1

    @property
    def n_cycles(self) -> int:
        return sum(v[1] for v in self._cyc.values())

    @property
    def n_short_cycles(self) -> int:
        return sum(v[2] for v in self._cyc.values())

    @property
    def channels_seen(self) -> tuple[int, ...]:
        """回传里出现过的通道号，升序。**以回传为准**，不含只下发过的。"""
        return tuple(sorted(self._cyc))

    def cycles_of(self, channel: int) -> int:
        """某个通道自己的周期数 —— 算该通道的周期率要用这个，不是 n_cycles。"""
        v = self._cyc.get(int(channel))
        return v[1] if v else 0

    @property
    def n_active(self) -> int:
        """最近 `ACTIVE_WINDOW_S` 秒内还在发帧的通道数，至少 1。

        用「静默多久」而不是「出现过没有」：关掉一个通道后它不再发帧，
        若按「出现过」算，周期率换算会一直按旧通道数偏低。
        """
        if not self._cyc:
            return 1
        n = sum(1 for v in self._cyc.values()
                if self._t_last - v[4] <= self.ACTIVE_WINDOW_S)
        return max(1, n)

    def reset(self) -> None:
        self._buf.clear()
        self.n_frames = self.n_dropped = self.n_bad_checksum = 0
        self._cyc.clear()
        self._t_last = 0.0
        self._last_ch = EXPECTED_CHANNEL

    def _stamp(self, fr: Frame, t_host: float) -> Frame:
        """补 t_host / cycle_id。周期起点 = 该通道的「暗时隙 + 最近距离」。"""
        ch = int(fr.channel)
        st = self._cyc.get(ch)
        if st is None:
            st = self._cyc[ch] = [-1, 0, 0, 0, t_host]
        if fr.is_dark and fr.distance_cm == CYCLE_START_DISTANCE_CM:
            # 上一周期收尾：帧数对不上就记一笔（丢帧 / 固件帧结构变了）。
            # 与 FRAMES_PER_CYCLE（单通道 12 帧）比 —— 这里数的是**本通道**
            # 的帧，所以不随通道数变。
            if st[0] >= 0 and st[3] != FRAMES_PER_CYCLE:
                st[2] += 1
            st[0] += 1
            st[1] += 1
            st[3] = 0
        st[3] += 1
        st[4] = t_host
        self._last_ch = ch
        self._t_last = t_host
        return replace(fr, t_host=t_host, cycle_id=st[0])

    def feed(self, data: bytes, t_host: float | None = None) -> list[Frame]:
        """`t_host` 留空则取 `self._clock()`；显式传入用于回放已归档的数据。"""
        t = self._clock() if t_host is None else float(t_host)
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
            out.append(self._stamp(fr, t))
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


def build_cmd(run: bool, gear: int, channel: int = EXPECTED_CHANNEL) -> bytes:
    """指令帧通用构造。byte4 启停、byte5 强度、byte6 通道。

    **byte4 是启/停位**（0x00 停止 / 0x01 启动）—— 2026-10-29 由厂家通讯协议
    文档确认。此前本文件把它注释成「固定值，含义未明；早先猜是启停，已证伪」，
    那条「已证伪」是错的：证据只是「不发启动也有数据」，
    这说明启动**不是必需的**，推不出「这一位不是启停位」。

    停止帧**已实测**（2026-10-29，162704）：`byte4=0x00` + `byte6=0x01`
    只停 1 号通道，2 号继续流；再发启动帧，1 号恢复。
    *（此处原先写「停止帧从未实测过，第一次试请在体模上做」—— 已经做了。）*

    ⚠️ 强度：厂家文档写 0~100（0x00~0x64），而本模块的 BRIGHTNESS_MAX
       仍是 0xFF（见上方注释）。实测 255 档约为 100 档的 2 倍，设备没在
       100 截断，所以两边都不能照搬。未解决；本函数按入参原样写入，
       不替调用方做范围判断之外的事。
    """
    b = bytearray(FRAME_LEN)
    b[0:2] = HEADER
    b[2] = FRAME_LEN
    b[3] = FRAME_TYPE_CMD
    b[4] = CMD_START if run else CMD_STOP
    b[5] = max(BRIGHTNESS_MIN, min(BRIGHTNESS_MAX, int(gear)))
    b[6] = max(CHANNEL_MIN, min(CHANNEL_MAX, int(channel)))
    b[FRAME_LEN - 1] = checksum(bytes(b))
    return bytes(b)


# 两条**固定**的协议测试帧（2026-10-29 由用户逐字节指定，界面上各挂一个按钮）。
# 写成常量而不是在界面里现造，是为了能在自检里逐字节对着用户给的那串验 ——
# 界面按钮送出去的必须就是这 26 个字节，不能是「我以为等价」的另一串。
CMD_START_CH2 = build_cmd(True, 0x64, 0x02)    # 55AA 1A 01 01 64 02 …00 82
CMD_STOP_CH1 = build_cmd(False, 0x00, 0x01)    # 55AA 1A 01 00 00 01 …00 1C


def build_brightness(gear: int, channel: int = EXPECTED_CHANNEL) -> bytes:
    """光源档位指令（byte4 恒为**启动**）。gear = 0x00 ~ 0xFF，上电默认 0x0F。

    `channel` 写进 byte6，指定这条指令作用在哪个采集通道上（1~4）。

    2026-10-29 实测确认（此前这里写「无法从回传数据上证实」，已不成立）：
      · 下发 byte6=0x02 后数据帧的 byte10 确实回 0x02；
      · 通道**累加**不切换 —— 开了 2 号，1 号仍在跑；
      · **强度按通道分别生效**：对 2 号下发档位 100，1 号纹丝不动
        （693,722 → 674,276）；对 1 号下发 100，它才掉到 342,014。
        8 份文件口径一致。

    ⚠️ 档位是**一个字节**，传 >255 会被 bytearray 拒绝。上限经历过
       0x0F → 0x96(150) → 0xFF(255) 三次上调；而厂家通讯协议文档写的是
       0~100（0x64）—— 口径不一致，未解决，见文件头。

    这是目前唯一确认有效的指令。生成的帧与厂家 2026-10-10 给的样例一致：
        build_brightness(0x00) == 55 AA 1A 01 01 00 01 00 … 00 1D

    ⚠️ 对 SRS 而言改档位是**安全**的：光强整体乘一个因子 B 时，
       A = −log10(B·I) = −log10(B) − log10(I)，两个探测器同减一个常数，
       **空间斜率完全不变**。所以不同体模块可以用不同档位迁就动态范围，
       不影响 G(λ)。但同一块的一次采集期间不要改。

       ① 五路 LED **不等比例** —— 2026-10-29 实测（同体模、探头不动，
          档位 255 与 100 各采一次）：十路的 100/255 比值在 0.384~0.507，
          极差 28.4%。等比例的话应当全都一样，所以这条**已被证伪**。
          *但它在 SRS 里抵消*：同一波长的两个距离上因子几乎相同
          （670: 0.507/0.505；770: 0.414/0.414），所以 I3/I4 基本不动，
          TOI 从 83.91 变到 83.67，只差 0.24 点。
          → 结论不变：**改档位对 SRS 安全**，但理由不是"等比例"，
            而是"同一波长的两个距离同比例"。
       ② 档位不改变 LED 峰值波长 —— **仍未验证**。文献上 LED 波长随驱动
          电流和结温漂移（0.1 nm/K 量级，调电流比 PWM 漂得多）。
          厂家通讯协议文档写「光源电流可调」，即是调电流 —— 漂得更多的那种。
          这条要上光谱仪才能验，不是软件能解决的。
    """
    return build_cmd(True, gear, channel)


def build_command(start: bool, brightness: int = BRIGHTNESS_DEFAULT,
                  channel: int = EXPECTED_CHANNEL) -> bytes:
    """旧接口，保留给既有调用方。

    ⚠️ 2026-10-29 修正：此前这里写「`start` 已无作用」并且**把 channel 也丢了**
       （函数体是 `build_brightness(brightness)`，两个参数都没传下去）。
       byte4 既然是启停位，`start` 就该生效。本仓库内无调用方，改之无碍。
    """
    return build_cmd(start, brightness, channel)


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
