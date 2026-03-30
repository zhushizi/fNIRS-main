"""
单通道 fNIRS 上位机的集中配置文件。

这里统一放置：
1. 串口参数
2. 26B 协议常量
3. 单通道几何配置
4. 输出文件名
"""

# 串口端口号。Windows 例如 "COM5"，Linux/macOS 可写成 /dev/ttyUSB0 之类。
SERIAL_PORT = "COM5"

# 串口基础参数
BAUD_RATE = 115200
TIMEOUT = 0.05

# 26B 协议固定帧格式
FRAME_HEADER = bytes((0x55, 0xAA))
FRAME_LENGTH = 0x1A
FRAME_PAYLOAD_SIZE = 21

# 帧类型定义
FRAME_TYPE_COMMAND = 0x01
FRAME_TYPE_DATA = 0x02
FRAME_TYPE_ACK = 0x03

# ACK 等待与重发策略。
# 这里是“通用默认值”，某些入口（如 visualizer）会再做更宽松的等待。
ACK_TIMEOUT_SECONDS = 0.05
MAX_RETRIES = 2

# 默认下发的命令参数
DEFAULT_INTENSITY_MA = 300
DEFAULT_STREAM_ENABLED = True

# 当前软件按单物理通道处理：S1_D1
CHANNEL_NAME = "S1_D1"
SOURCE_DETECTOR_DISTANCE_CM = 3.0

# 波长编码映射（数据帧 payload byte0）。
# - 0x00：两路均未点亮（本样本不代表某一工作波长）
# - 0x01：940nm
# - 0x02：660nm
WAVELENGTH_OFF_CODE = 0x00
WAVELENGTH_940_CODE = 0x01
WAVELENGTH_660_CODE = 0x02
WAVELENGTH_BY_CODE = {
    WAVELENGTH_660_CODE: 660.0,
    WAVELENGTH_940_CODE: 940.0,
}
CODE_BY_WAVELENGTH = {
    660.0: WAVELENGTH_660_CODE,
    940.0: WAVELENGTH_940_CODE,
}

# 当前只有一个传感器模块，因此编号固定为 0x00。
# 后续如果扩展到多传感器，可从这里统一调整。
DEFAULT_SENSOR_ID = 0x00

# 处理链路中的三个主要输出文件
RAW_OUTPUT_CSV = "all_groups.csv"
INTERLEAVED_OUTPUT_CSV = "interleaved_output.csv"
PROCESSED_OUTPUT_CSV = "processed_output.csv"
