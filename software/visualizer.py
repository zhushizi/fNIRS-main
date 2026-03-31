"""
轻量级单通道控制面板后端。

它做三件事：
1. 提供网页接口（Flask）
2. 后台持续收串口数据并缓存最近若干帧
3. 接受前端控制请求，发启动/停止命令并等待 ACK
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque

import serial
from flask import Flask, jsonify, request, send_from_directory

from config import (
    ACK_TIMEOUT_SECONDS,
    BAUD_RATE,
    DEFAULT_INTENSITY_MA,
    MAX_RETRIES,
    SERIAL_PORT,
    TIMEOUT,
    FRAME_TYPE_ACK,
    FRAME_TYPE_DATA,
    WAVELENGTH_660_CODE,
    WAVELENGTH_940_CODE,
)
from protocol import (
    FrameReader,
    build_command_frame,
    build_frame,
    parse_data_frame,
)


demo_mode = False
app = Flask(__name__)

# 串口相关共享状态。
# 之所以要用锁，是因为“后台读线程”和“前端触发的命令发送”会共用同一个串口对象。
serial_lock = threading.Lock()
reader_thread = None
stop_reader = threading.Event()
ack_event = threading.Event()
recent_packets = deque(maxlen=50)
latest_packet = None
# 用于计算实时采样率：最近若干帧的接收时间戳（仅数据帧）
frame_timestamps = deque(maxlen=3000)
control_state = {
    "stream_enabled": False,
    "intensity_ma": DEFAULT_INTENSITY_MA,
    "last_command_ok": None,
}
COMMAND_ACK_TIMEOUT_SECONDS = max(ACK_TIMEOUT_SECONDS, 0.2)


def open_serial() -> serial.Serial:
    """按配置打开串口。"""
    return serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=TIMEOUT)


ser = None
frame_reader = None


def _append_packet(packet: dict) -> None:
    """维护页面展示用的最近一帧和最近 50 帧历史，并记录时间戳用于采样率计算。"""
    global latest_packet
    latest_packet = packet
    recent_packets.append(packet)
    frame_timestamps.append(time.time())


def frame_to_hex(frame_bytes: bytes) -> str:
    """把二进制帧转成十六进制字符串，便于联调打印。"""
    return " ".join(f"{b:02X}" for b in frame_bytes)


def get_sampling_rate_hz() -> float | None:
    """
    根据最近数据帧的时间戳计算实时采样率（Hz）。
    需要至少 0.2 秒的时间跨度才返回数值，否则返回 None。
    """
    if len(frame_timestamps) < 2:
        return None
    t0 = frame_timestamps[0]
    t1 = frame_timestamps[-1]
    span = t1 - t0
    if span < 0.2:
        return None
    return (len(frame_timestamps) - 1) / span


def serial_reader_loop() -> None:
    """
    后台串口读线程。

    注意这里统一负责“读串口”：
    - 收到 ACK：只负责 set ack_event
    - 收到 0x02：仅缓存数据，不对数据帧回 ACK
    这样可以避免多个线程同时 read 同一串口导致 ACK 竞争。
    """
    while not stop_reader.is_set():
        if demo_mode:
            t = time.time()
            wave_code = WAVELENGTH_660_CODE if int(t * 2) % 2 == 0 else WAVELENGTH_940_CODE
            value = int(2000 + 300 * math.sin(t * 2))
            # demo 模式下直接伪造一条交替波长的数据，方便前端调试界面。
            _append_packet(
                {
                    "timestamp": round(t, 3),
                "sensor_id": 0,
                    "wavelength_code": wave_code,
                    "value": value,
                }
            )
            time.sleep(0.1)
            continue

        with serial_lock:
            # 短超时便于快速排空串口缓冲，提高可达到的采样率显示
            frame = frame_reader.read_frame(timeout_seconds=0.002)
            if frame is None:
                continue
            raw_frame = build_frame(frame.frame_type, frame.payload)
            if frame.frame_type == FRAME_TYPE_ACK:
                print(f"[visualizer] RX frame type=0x03 (ACK) raw={frame_to_hex(raw_frame)}")
                ack_event.set()
                continue
            if frame.frame_type != FRAME_TYPE_DATA:
                print(f"[visualizer] RX frame type=0x{frame.frame_type:02X} raw={frame_to_hex(raw_frame)}")
                continue
            sample = parse_data_frame(frame)

        _append_packet(
            {
                "timestamp": round(time.time(), 3),
                "sensor_id": int(sample.sensor_id),
                "wavelength_code": int(sample.wavelength_code),
                "value": int(sample.value),
            }
        )


def initialize_serial() -> None:
    """初始化串口和公共 FrameReader。"""
    global ser, frame_reader
    if demo_mode:
        return
    ser = open_serial()
    ser.reset_input_buffer()
    frame_reader = FrameReader(ser)


def send_command_with_ack(stream_enabled: bool, intensity_ma: int) -> bool:
    """
    向下位机发送启动/停止命令。

    当前固件版本不返回 ACK，这里仅发送命令，不再等待 ack_event。
    """
    frame = build_command_frame(stream_enabled=stream_enabled, intensity_ma=intensity_ma)
    with serial_lock:
        print(f"[visualizer] TX command raw={frame_to_hex(frame)}")
        ser.write(frame)
    print("[visualizer] ACK check skipped for current firmware.")
    return True


@app.route("/")
def serve_index():
    """返回控制面板首页。"""
    return send_from_directory(".", "index.html")


@app.route("/api/status")
def api_status():
    """前端轮询接口：返回当前控制状态、最近帧缓存和实时采样率。"""
    rate_hz = get_sampling_rate_hz()
    return jsonify(
        {
            "demo_mode": demo_mode,
            "serial_port": SERIAL_PORT,
            "baud_rate": BAUD_RATE,
            "control_state": control_state,
            "latest_packet": latest_packet,
            "recent_packets": list(recent_packets),
            "sampling_rate_hz": round(rate_hz, 1) if rate_hz is not None else None,
        }
    )


@app.route("/update_control_data", methods=["POST"])
def update_control_data():
    """前端发控制命令时调用：更新状态并向下位机发命令帧。"""
    data = request.get_json(force=True)
    stream_enabled = bool(data.get("stream_enabled", control_state["stream_enabled"]))
    intensity_ma = int(data.get("intensity_ma", control_state["intensity_ma"]))
    intensity_ma = max(0, min(intensity_ma, 0xFF))

    control_state["stream_enabled"] = stream_enabled
    control_state["intensity_ma"] = intensity_ma

    if demo_mode:
        control_state["last_command_ok"] = True
        return jsonify({"status": "success", "demo_mode": True, "control_state": control_state})

    ok = send_command_with_ack(stream_enabled=stream_enabled, intensity_ma=intensity_ma)

    control_state["last_command_ok"] = ok
    status = "success" if ok else "ack_timeout"
    return jsonify({"status": status, "control_state": control_state})


def main():
    """程序入口：初始化串口、启动后台线程、跑 Flask 服务。"""
    global demo_mode, reader_thread
    import sys

    logging.getLogger("werkzeug").setLevel(logging.ERROR)
    app.logger.disabled = True
    demo_mode = any(arg.lower() == "demo" for arg in sys.argv[1:])
    initialize_serial()

    reader_thread = threading.Thread(target=serial_reader_loop, daemon=True)
    reader_thread.start()

    try:
        print(" * Running on http://127.0.0.1:8050")
        app.run(host="127.0.0.1", port=8050, debug=False)
    finally:
        stop_reader.set()
        if reader_thread is not None:
            reader_thread.join(timeout=1.0)
        if ser is not None:
            ser.close()


if __name__ == "__main__":
    main()
