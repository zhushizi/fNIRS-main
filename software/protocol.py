"""
单通道 26B 协议的公共辅助模块。

这个文件负责两类事情：
1. 组帧：把命令/ACK 拼成完整 26B 帧
2. 拆帧：从串口字节流中同步帧头、校验长度和 checksum

上层脚本（adc_live / visualizer / fNIRS_processing）都通过这里来收发协议帧。
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import serial

from config import (
    ACK_TIMEOUT_SECONDS,
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
    value: int


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
    """构造 0x01 命令帧。当前使用 payload[0]=启停, payload[1]=单字节光强。"""
    intensity = max(0, min(intensity_ma, 0xFF))
    payload = bytearray(FRAME_PAYLOAD_SIZE)
    payload[0] = 0x01 if stream_enabled else 0x00
    payload[1] = intensity
    return build_frame(FRAME_TYPE_COMMAND, bytes(payload))


def build_ack_frame() -> bytes:
    """构造 0x03 ACK 帧，payload 当前全 0。"""
    return build_frame(FRAME_TYPE_ACK, bytes(FRAME_PAYLOAD_SIZE))


def parse_data_frame(frame: ParsedFrame) -> DataSample:
    """
    解析 0x02 数据帧。

    当前协议约定（payload byte0）：
    - 0x00：未点亮
    - 0x01：940nm
    - 0x02：660nm

    其余字段：
    - payload[1] = 传感器编号
    - payload[2:4] = 采样值（低字节在前）
    """
    if frame.frame_type != FRAME_TYPE_DATA:
        raise ValueError("Expected a data frame.")

    payload = frame.payload
    wavelength_code = payload[0]
    sensor_id = payload[1]
    value = int.from_bytes(payload[2:4], byteorder="little", signed=False)
    if wavelength_code == WAVELENGTH_OFF_CODE:
        wavelength_nm: Optional[float] = None
    else:
        wavelength_nm = WAVELENGTH_BY_CODE.get(wavelength_code)
    return DataSample(
        wavelength_code=wavelength_code,
        wavelength_nm=wavelength_nm,
        sensor_id=sensor_id,
        value=value,
    )


class FrameReader:
    def __init__(self, ser: serial.Serial):
        # buffer 用来承接“串口是字节流而不是天然按帧到来”这个事实。
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
        """
        从内部缓冲区中同步出一帧完整 26B 数据。

        流程：
        1. 找帧头 55 AA
        2. 检查长度字节是否等于 0x1A
        3. 检查 checksum
        4. 通过后返回 ParsedFrame
        """
        start = time.monotonic()
        total_size = len(FRAME_HEADER) + 1 + 1 + FRAME_PAYLOAD_SIZE + 1

        while True:
            header_pos = self.buffer.find(FRAME_HEADER)
            if header_pos > 0:
                # 丢弃帧头之前的噪声或残帧字节。
                del self.buffer[:header_pos]

            if len(self.buffer) >= total_size and self.buffer[:2] == FRAME_HEADER:
                length_byte = self.buffer[2]
                frame_type = self.buffer[3]
                payload = bytes(self.buffer[4:4 + FRAME_PAYLOAD_SIZE])
                checksum = self.buffer[4 + FRAME_PAYLOAD_SIZE]
                del self.buffer[:total_size]

                if length_byte != FRAME_LENGTH:
                    # 长度不对，继续向后找下一帧头。
                    continue

                expected = calculate_checksum(length_byte, frame_type, payload)
                if checksum != expected:
                    # checksum 不对，说明这一帧损坏，直接丢弃。
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


def wait_for_ack(ser: serial.Serial, reader: FrameReader, timeout_seconds: float = ACK_TIMEOUT_SECONDS) -> bool:
    """
    等待 ACK。

    等待过程中若收到数据帧则直接丢弃，不再对数据帧回 ACK。
    """
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        frame = reader.read_frame(timeout_seconds=deadline - time.monotonic())
        if frame is None:
            return False
        if frame.frame_type == FRAME_TYPE_ACK:
            return True
        # 收到数据帧不应答，继续等 ACK
    return False


def send_frame_with_ack(
    ser: serial.Serial,
    reader: FrameReader,
    frame: bytes,
    retries: int = MAX_RETRIES,
) -> bool:
    """发送一帧（当前固件不回 ACK，临时跳过 ACK 判断）。"""
    # 兼容当前下位机版本：命令发送后不等待 0x03 ACK。
    # 保留 reader/retries 参数是为了不破坏现有调用签名。
    ser.write(frame)
    return True
