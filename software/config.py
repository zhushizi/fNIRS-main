"""
双接收源五波长 fNIRS 上位机的集中配置文件。

这里统一放置：
1. 串口参数
2. 26B 协议常量
3. S1-D1 / S1-D2 几何与五波长表
4. HbO / HbR / Cyt 反演、输出通道选择与输出文件名
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

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
DEFAULT_INTENSITY_MA = 0x18
DEFAULT_STREAM_ENABLED = True
DEFAULT_COMMAND_CHANNEL_CODE = 0x01

# 默认通道名保留给旧脚本兜底；主处理链路使用 DETECTOR_CHANNELS。
CHANNEL_NAME = "S1_D1"
SOURCE_DETECTOR_DISTANCE_CM = 3.0

# 数据帧 payload byte0：两路均未点亮
WAVELENGTH_OFF_CODE = 0x00
DETECTOR_OFF_CODE = 0x00

# ---------------------------------------------------------------------------
# 输出通道选择（离线 processed_output、在线回传安卓、live 曲线均使用此通道）
# 可选: "S1_D1" | "S1_D2" | "S1_D1_ssr"
# ---------------------------------------------------------------------------
OutputChannelName = Literal["S1_D1", "S1_D2", "S1_D1_ssr"]
OUTPUT_CHANNEL: OutputChannelName = "S1_D1_ssr"
VALID_OUTPUT_CHANNELS: tuple[OutputChannelName, ...] = ("S1_D1", "S1_D2", "S1_D1_ssr")

# ---------------------------------------------------------------------------
# 在线回传安卓的模式与显示单位
# ---------------------------------------------------------------------------
# "full_replace"       : 旧逻辑，每 tick 对全会话非因果重算并整段重发（历史会变）。
# "causal_incremental" : 路线 B，因果 IIR 滤波 + 仅追加新样本（历史不再变，量纲固定）。
OnlineModeName = Literal["full_replace", "causal_incremental"]
ONLINE_MODE: OnlineModeName = "causal_incremental"

# 回传安卓前对浓度做显示换算：摩尔(M) × LIVE_CONC_SCALE，并附带单位标签。
# 1.0 → 保持原始摩尔(M)，与离线 processed_output.csv 同量纲；1e6 → 微摩尔(μM)。
LIVE_CONC_SCALE = 1.0
LIVE_CONC_UNIT = "M"
# 是否减去基线窗均值做锚定（让每条曲线从 0 起算，run 之间幅度可比）。
LIVE_BASELINE_ANCHOR = False
# 是否向安卓 live_analysis_batch 附带 delta_thb（HbO+HbR）。安卓未升级解析前可设 False。
LIVE_SEND_DELTA_THB = True


def _nm_suffix(nm: float) -> str:
    if nm == int(nm):
        return str(int(nm))
    return str(nm).replace(".", "_")


@dataclass(frozen=True)
class WavelengthChannel:
    """
    单路活跃波长。

    - code: 协议 Wavelength 列取值（LED1..LED5）
    - emitter_nm: 光源标称波长，用于 CSV 列名后缀（如 S1_D1_850）
    - mbll_nm: MBLL / DPF 反演用的光学波长
    """

    code: int
    emitter_nm: float
    mbll_nm: float

    @property
    def intensity_column(self) -> str:
        return intensity_column_name(self.emitter_nm)


@dataclass(frozen=True)
class DetectorChannel:
    """
    单个接收源形成的物理通道。

    - code: 协议 DetectorId / SensorId 取值（PD1=0x01，PD2=0x02）
    - name: 输出列名前缀（S1_D1 / S1_D2）
    - distance_cm: 源探距离，进入 MBLL pathlength
    """

    code: int
    name: str
    distance_cm: float


def intensity_column_name(emitter_nm: float, channel_name: str = CHANNEL_NAME) -> str:
    """由通道名和光源标称波长生成光强列名，如 S1_D1_850。"""
    return f"{channel_name}_{_nm_suffix(emitter_nm)}"


def detector_channel_by_code(code: int) -> DetectorChannel | None:
    for ch in DETECTOR_CHANNELS:
        if ch.code == code:
            return ch
    return None


def intensity_column_for(channel_name: str, emitter_nm: float) -> str:
    return intensity_column_name(emitter_nm, channel_name)


# ---------------------------------------------------------------------------
# 接收源与活跃波长表（按 samples 矩阵的行顺序排列）
# ---------------------------------------------------------------------------
DETECTOR_CHANNELS: tuple[DetectorChannel, ...] = (
    DetectorChannel(code=0x01, name="S1_D1", distance_cm=3.0),
    DetectorChannel(code=0x02, name="S1_D2", distance_cm=1.0),
)

WAVELENGTH_CHANNELS: tuple[WavelengthChannel, ...] = (
    WavelengthChannel(code=0x01, emitter_nm=850.0, mbll_nm=850.0),
    WavelengthChannel(code=0x02, emitter_nm=810.0, mbll_nm=810.0),
    WavelengthChannel(code=0x03, emitter_nm=770.0, mbll_nm=770.0),
    WavelengthChannel(code=0x04, emitter_nm=730.0, mbll_nm=730.0),
    WavelengthChannel(code=0x05, emitter_nm=700.0, mbll_nm=700.0),
)

# 兼容旧代码的别名
WAVELENGTH_850_CODE = WAVELENGTH_CHANNELS[0].code
WAVELENGTH_810_CODE = WAVELENGTH_CHANNELS[1].code
WAVELENGTH_770_CODE = WAVELENGTH_CHANNELS[2].code
WAVELENGTH_730_CODE = WAVELENGTH_CHANNELS[3].code
WAVELENGTH_700_CODE = WAVELENGTH_CHANNELS[4].code
WAVELENGTH_940_CODE = WAVELENGTH_850_CODE
WAVELENGTH_660_CODE = WAVELENGTH_700_CODE
MBLL_WAVELENGTH_WL1_NM = WAVELENGTH_CHANNELS[0].mbll_nm
MBLL_WAVELENGTH_WL2_NM = WAVELENGTH_CHANNELS[1].mbll_nm

WAVELENGTH_BY_CODE = {ch.code: ch.emitter_nm for ch in WAVELENGTH_CHANNELS}
CODE_BY_WAVELENGTH = {ch.emitter_nm: ch.code for ch in WAVELENGTH_CHANNELS}
ACTIVE_WAVELENGTH_CODES = frozenset(ch.code for ch in WAVELENGTH_CHANNELS)
DETECTOR_BY_CODE = {ch.code: ch.name for ch in DETECTOR_CHANNELS}
ACTIVE_DETECTOR_CODES = frozenset(ch.code for ch in DETECTOR_CHANNELS)
CHANNEL_NAMES: tuple[str, ...] = tuple(ch.name for ch in DETECTOR_CHANNELS)
INTENSITY_COLUMNS: tuple[str, ...] = tuple(
    intensity_column_for(det.name, wl.emitter_nm)
    for det in DETECTOR_CHANNELS
    for wl in WAVELENGTH_CHANNELS
)

# Cyt / oxCCO 差分消光系数，单位 OD / cm / mM（UCL-NIR-Spectra）。
CYT_DIFFERENCE_EXTINCTION = {
    700.0: 2.4148076,
    730.0: 1.8012364,
    770.0: 1.9650957,
    810.0: 2.3166136,
    850.0: 2.2899045,
}
CYT_RELATIVE_EXTINCTION = CYT_DIFFERENCE_EXTINCTION
CHROMOPHORE_TYPES: tuple[str, ...] = ("hbo", "hbr", "cyt")

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
ANDROID_LIVE_OUTPUT_CSV = "android_live_output.csv"


def wavelength_channel_by_code(code: int) -> WavelengthChannel | None:
    for ch in WAVELENGTH_CHANNELS:
        if ch.code == code:
            return ch
    return None


def mbll_wavelengths_nm() -> list[float]:
    return [ch.mbll_nm for ch in WAVELENGTH_CHANNELS]


def intensity_columns_for_detector(detector: DetectorChannel) -> tuple[str, ...]:
    return tuple(intensity_column_for(detector.name, ch.emitter_nm) for ch in WAVELENGTH_CHANNELS)


def processed_column_names(channel: str | None = None) -> tuple[str, ...]:
    """生成 processed_output 列名，如 S1_D1_ssr_hbo。"""
    prefix = channel if channel is not None else OUTPUT_CHANNEL
    return tuple(f"{prefix}_{chrom}" for chrom in CHROMOPHORE_TYPES)


def _validate_output_channel() -> None:
    if OUTPUT_CHANNEL not in VALID_OUTPUT_CHANNELS:
        raise ValueError(
            f"config.OUTPUT_CHANNEL 必须为 {VALID_OUTPUT_CHANNELS} 之一，当前为 {OUTPUT_CHANNEL!r}。"
        )


def _validate_wavelength_channels() -> None:
    if len(WAVELENGTH_CHANNELS) < len(CHROMOPHORE_TYPES):
        raise ValueError("config.WAVELENGTH_CHANNELS 至少需要 3 路波长才能反演 HbO/HbR/Cyt。")
    codes = [ch.code for ch in WAVELENGTH_CHANNELS]
    if len(codes) != len(set(codes)):
        raise ValueError("config.WAVELENGTH_CHANNELS 中存在重复的协议 code。")
    emitters = [ch.emitter_nm for ch in WAVELENGTH_CHANNELS]
    if len(emitters) != len(set(emitters)):
        raise ValueError("config.WAVELENGTH_CHANNELS 中存在重复的 emitter_nm（列名会冲突）。")
    if WAVELENGTH_OFF_CODE in codes:
        raise ValueError("WAVELENGTH_OFF_CODE 不能出现在 WAVELENGTH_CHANNELS 中。")
    detector_codes = [ch.code for ch in DETECTOR_CHANNELS]
    if len(detector_codes) != len(set(detector_codes)):
        raise ValueError("config.DETECTOR_CHANNELS 中存在重复的协议 code。")
    detector_names = [ch.name for ch in DETECTOR_CHANNELS]
    if len(detector_names) != len(set(detector_names)):
        raise ValueError("config.DETECTOR_CHANNELS 中存在重复的 name。")
    if DETECTOR_OFF_CODE in detector_codes:
        raise ValueError("DETECTOR_OFF_CODE 不能出现在 DETECTOR_CHANNELS 中。")
    missing_cyt = [ch.mbll_nm for ch in WAVELENGTH_CHANNELS if ch.mbll_nm not in CYT_RELATIVE_EXTINCTION]
    if missing_cyt:
        raise ValueError(f"CYT_RELATIVE_EXTINCTION 缺少波长: {missing_cyt}")


_validate_wavelength_channels()
_validate_output_channel()
