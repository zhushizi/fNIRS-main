"""
双接收源五波长 26B 协议的公共辅助模块。

这个文件负责两类事情：
1. 组帧：把命令/ACK 拼成完整 26B 帧
2. 拆帧：从串口字节流中同步帧头、校验长度和 checksum

上层脚本（adc_live / visualizer / fNIRS_processing）都通过这里来收发协议帧。
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Optional

from config import (
    ACK_TIMEOUT_SECONDS,
    DETECTOR_BY_CODE,
    DEFAULT_COMMAND_CHANNEL_CODE,
    DEFAULT_INTENSITY_MA,
    FRAME_HEADER,
    FRAME_LENGTH,
    FRAME_PAYLOAD_SIZE,
    FRAME_TYPE_ACK,
    FRAME_TYPE_COMMAND,
    FRAME_TYPE_DATA,
    MAX_RETRIES,
    WAVELENGTH_BY_CODE,
    WAVELENGTH_OFF_CODE,
    normalize_channel_code,
)


@dataclass
class ParsedFrame:
    """一帧已经通过校验后的抽象表示。"""
    frame_type: int
    payload: bytes


@dataclass
class DataSample:
    """把 0x02 数据帧再解释成更高层的单个采样点。"""
    wavelength_code: int
    wavelength_nm: Optional[float]
    sensor_id: int
    detector_code: int
    channel_name: Optional[str]
    value: int
    # 采集通道（硬件通道，payload byte6 归一化后）：通道1=0x01，通道2=0x02。
    acq_channel_code: int = 0x01


def calculate_checksum(length_byte: int, frame_type: int, payload: bytes) -> int:
    """按照协议定义计算 checksum：从长度字节开始累加到 payload 结束。"""
    checksum_source = bytes((length_byte, frame_type)) + payload
    return sum(checksum_source) & 0xFF


def build_frame(frame_type: int, payload: bytes) -> bytes:
    """将帧类型和 21B payload 封装成完整 26B 协议帧。"""
    if len(payload) != FRAME_PAYLOAD_SIZE:
        raise ValueError(f"Payload must be {FRAME_PAYLOAD_SIZE} bytes.")

    checksum = calculate_checksum(FRAME_LENGTH, frame_type, payload)
    return FRAME_HEADER + bytes((FRAME_LENGTH, frame_type)) + payload + bytes((checksum,))


def build_command_frame(stream_enabled: bool, intensity_ma: int = DEFAULT_INTENSITY_MA) -> bytes:
    """构造 0x01 命令帧：payload[0]=启停, payload[1]=光强, payload[2]=通道选择。"""
    intensity = max(0, min(intensity_ma, 0xFF))
    payload = bytearray(FRAME_PAYLOAD_SIZE)
    payload[0] = 0x01 if stream_enabled else 0x00
    payload[1] = intensity
    payload[2] = DEFAULT_COMMAND_CHANNEL_CODE
    return build_frame(FRAME_TYPE_COMMAND, bytes(payload))


def build_ack_frame() -> bytes:
    """构造 0x03 ACK 帧，payload 当前全 0。"""
    return build_frame(FRAME_TYPE_ACK, bytes(FRAME_PAYLOAD_SIZE))


def parse_data_frame(frame: ParsedFrame) -> DataSample:
    """
    解析 0x02 数据帧。

    当前协议约定：
    - payload[0] = 光源波长编号（0x01..0x05）
    - payload[1] = 接收源编号（PD1=0x01，PD2=0x02）
    - payload[2:6] = 采样值（高字节在前，4 字节）
    - payload[6] = 采集通道编号（通道1=0x01，通道2=0x02；0x00 归入默认通道）

    sensor_id 字段保留为 detector_code 的兼容别名。
    """
    if frame.frame_type != FRAME_TYPE_DATA:
        raise ValueError("Expected a data frame.")

    payload = frame.payload
    wavelength_code = payload[0]
    detector_code = payload[1]
    value = int.from_bytes(payload[2:6], byteorder="big", signed=False)
    acq_channel_code = normalize_channel_code(payload[6])
    if wavelength_code == WAVELENGTH_OFF_CODE:
        wavelength_nm: Optional[float] = None
    else:
        wavelength_nm = WAVELENGTH_BY_CODE.get(wavelength_code)
    channel_name = DETECTOR_BY_CODE.get(detector_code)
    return DataSample(
        wavelength_code=wavelength_code,
        wavelength_nm=wavelength_nm,
        sensor_id=detector_code,
        detector_code=detector_code,
        channel_name=channel_name,
        value=value,
        acq_channel_code=acq_channel_code,
    )


class FrameReader:
    def __init__(self, ser: Any):
        self.ser = ser
        self.buffer = bytearray()

    def _read_more(self, timeout_seconds: Optional[float]) -> bool:
        """尽量多从串口读一点字节，补充到内部缓冲区。"""
        if timeout_seconds is None:
            chunk = self.ser.read(1)
            if not chunk:
                return False
            self.buffer.extend(chunk)
            return True

        end_time = time.monotonic() + timeout_seconds
        while time.monotonic() < end_time:
            waiting = getattr(self.ser, "in_waiting", 0) or 1
            chunk = self.ser.read(waiting)
            if chunk:
                self.buffer.extend(chunk)
                return True
            time.sleep(0.001)
        return False

    def read_frame(self, timeout_seconds: Optional[float] = None) -> Optional[ParsedFrame]:
        """从内部缓冲区中同步出一帧完整 26B 数据。"""
        start = time.monotonic()
        total_size = len(FRAME_HEADER) + 1 + 1 + FRAME_PAYLOAD_SIZE + 1

        while True:
            header_pos = self.buffer.find(FRAME_HEADER)
            if header_pos > 0:
                del self.buffer[:header_pos]

            if len(self.buffer) >= total_size and self.buffer[:2] == FRAME_HEADER:
                length_byte = self.buffer[2]
                frame_type = self.buffer[3]
                payload = bytes(self.buffer[4:4 + FRAME_PAYLOAD_SIZE])
                checksum = self.buffer[4 + FRAME_PAYLOAD_SIZE]
                del self.buffer[:total_size]

                if length_byte != FRAME_LENGTH:
                    continue

                expected = calculate_checksum(length_byte, frame_type, payload)
                if checksum != expected:
                    continue

                return ParsedFrame(frame_type=frame_type, payload=payload)

            if timeout_seconds is None:
                self._read_more(None)
                continue

            elapsed = time.monotonic() - start
            remaining = timeout_seconds - elapsed
            if remaining <= 0:
                return None
            if not self._read_more(remaining):
                return None


def wait_for_ack(ser: Any, reader: FrameReader, timeout_seconds: float = ACK_TIMEOUT_SECONDS) -> bool:
    """等待 ACK。"""
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        frame = reader.read_frame(timeout_seconds=deadline - time.monotonic())
        if frame is None:
            return False
        if frame.frame_type == FRAME_TYPE_ACK:
            return True
    return False


def send_frame_with_ack(
    ser: Any,
    reader: FrameReader,
    frame: bytes,
    retries: int = MAX_RETRIES,
) -> bool:
    """发送一帧（当前固件不回 ACK，临时跳过 ACK 判断）。"""
    ser.write(frame)
    return True
