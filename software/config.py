"""
单通道 fNIRS 上位机的集中配置文件。

这里统一放置：
1. 串口参数
2. 26B 协议常量
3. 单通道几何与波长表（改波长 / 增波长只改 WAVELENGTH_CHANNELS）
4. 输出文件名
"""

from __future__ import annotations

from dataclasses import dataclass

# 串口端口号。Windows 例如 "COM5"，Linux/macOS 可写成 /dev/ttyUSB0 之类。
SERIAL_PORT = "COM6"

# 串口基础参数
BAUD_RATE = 115200
TIMEOUT = 0.05
# 设备标称采样率（Hz），用于重建等间隔时间戳。
SAMPLING_RATE_HZ = 250.0

# 安卓 Host TCP Protocol 参数。解码端作为 TCP Server，安卓主动连接。
HOST_TCP_LISTEN_HOST = "0.0.0.0"
HOST_TCP_DEFAULT_PORT = 9000
HOST_TCP_PROTOCOL_VERSION = 1
HOST_TCP_MAX_BODY_BYTES = 4 * 1024 * 1024

# 26B 协议固定帧格式
FRAME_HEADER = bytes((0x55, 0xAA))
FRAME_LENGTH = 0x1A
FRAME_PAYLOAD_SIZE = 21

# 帧类型定义
FRAME_TYPE_COMMAND = 0x01
FRAME_TYPE_DATA = 0x02
FRAME_TYPE_ACK = 0x03

# ACK 等待与重发策略。
ACK_TIMEOUT_SECONDS = 0.05
MAX_RETRIES = 2

# 默认下发的命令参数
DEFAULT_INTENSITY_MA = 200
DEFAULT_STREAM_ENABLED = True

# 当前软件按单物理通道处理：S1_D1
CHANNEL_NAME = "S1_D1"
SOURCE_DETECTOR_DISTANCE_CM = 3.0

# 数据帧 payload byte0：两路均未点亮
WAVELENGTH_OFF_CODE = 0x00


@dataclass(frozen=True)
class WavelengthChannel:
    """
    单路活跃波长。

    - code: 协议 Wavelength 列取值（如 0x01、0x02）
    - emitter_nm: 光源标称波长，用于 CSV 列名后缀（如 S1_D1_940）
    - mbll_nm: MBLL / DPF 反演用的光学波长（可与 emitter 不同，如 940→860）
    """

    code: int
    emitter_nm: float
    mbll_nm: float

    @property
    def intensity_column(self) -> str:
        return intensity_column_name(self.emitter_nm)


def intensity_column_name(emitter_nm: float) -> str:
    """由光源标称波长生成光强列名，如 S1_D1_940。"""
    if emitter_nm == int(emitter_nm):
        suffix = str(int(emitter_nm))
    else:
        suffix = str(emitter_nm).replace(".", "_")
    return f"{CHANNEL_NAME}_{suffix}"


# ---------------------------------------------------------------------------
# 活跃波长表（按 MBLL samples 矩阵的行顺序排列）
# 增删波长：只改此元组；协议码不可重复，emitter_nm 不可重复。
# ---------------------------------------------------------------------------
WAVELENGTH_CHANNELS: tuple[WavelengthChannel, ...] = (
    WavelengthChannel(code=0x01, emitter_nm=940.0, mbll_nm=860.0),
    WavelengthChannel(code=0x02, emitter_nm=660.0, mbll_nm=660.0),
    # 示例：第三波长
    # WavelengthChannel(code=0x03, emitter_nm=850.0, mbll_nm=850.0),
)

# 兼容旧代码的别名（由 WAVELENGTH_CHANNELS 派生）
WAVELENGTH_940_CODE = WAVELENGTH_CHANNELS[0].code
WAVELENGTH_660_CODE = WAVELENGTH_CHANNELS[1].code
MBLL_WAVELENGTH_WL1_NM = WAVELENGTH_CHANNELS[0].mbll_nm
MBLL_WAVELENGTH_WL2_NM = WAVELENGTH_CHANNELS[1].mbll_nm

WAVELENGTH_BY_CODE = {ch.code: ch.emitter_nm for ch in WAVELENGTH_CHANNELS}
CODE_BY_WAVELENGTH = {ch.emitter_nm: ch.code for ch in WAVELENGTH_CHANNELS}
ACTIVE_WAVELENGTH_CODES = frozenset(ch.code for ch in WAVELENGTH_CHANNELS)
INTENSITY_COLUMNS: tuple[str, ...] = tuple(ch.intensity_column for ch in WAVELENGTH_CHANNELS)

MBLL_DEFAULT_AGE = 27

# OD 带通
BP_LOW_HZ = 0.01
BP_HIGH_HZ = 0.1
BP_ORDER = 4
BP_TARGET_FS_HZ = 20.0

DEFAULT_SENSOR_ID = 0x00

RAW_OUTPUT_CSV = "all_groups.csv"
INTERLEAVED_OUTPUT_CSV = "interleaved_output.csv"
PROCESSED_OUTPUT_CSV = "processed_output.csv"
# 采集中发往安卓的 HbO/HbR 曲线（与 processed_output.csv 同列格式，便于对比）
ANDROID_LIVE_OUTPUT_CSV = "android_live_output.csv"


def wavelength_channel_by_code(code: int) -> WavelengthChannel | None:
    for ch in WAVELENGTH_CHANNELS:
        if ch.code == code:
            return ch
    return None


def mbll_wavelengths_nm() -> list[float]:
    return [ch.mbll_nm for ch in WAVELENGTH_CHANNELS]


def _validate_wavelength_channels() -> None:
    if len(WAVELENGTH_CHANNELS) < 2:
        raise ValueError("config.WAVELENGTH_CHANNELS 至少需要 2 路波长才能运行 MBLL。")
    codes = [ch.code for ch in WAVELENGTH_CHANNELS]
    if len(codes) != len(set(codes)):
        raise ValueError("config.WAVELENGTH_CHANNELS 中存在重复的协议 code。")
    emitters = [ch.emitter_nm for ch in WAVELENGTH_CHANNELS]
    if len(emitters) != len(set(emitters)):
        raise ValueError("config.WAVELENGTH_CHANNELS 中存在重复的 emitter_nm（列名会冲突）。")
    if WAVELENGTH_OFF_CODE in codes:
        raise ValueError("WAVELENGTH_OFF_CODE 不能出现在 WAVELENGTH_CHANNELS 中。")


_validate_wavelength_channels()
