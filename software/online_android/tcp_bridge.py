"""
Host TCP Protocol：安卓上位机与 PC 解码端之间的 JSON/TCP 传输层。

Android 独占 UART 启停；PC 被动接收 serial_data，并可回传分析结果。
"""

from __future__ import annotations

import base64
import binascii
import json
import math
import socket
import threading
import time
from typing import Any

from config import (
    HOST_TCP_LISTEN_HOST,
    HOST_TCP_MAX_BODY_BYTES,
    HOST_TCP_PROTOCOL_VERSION,
    TIMEOUT,
)


class HostTcpProtocolError(RuntimeError):
    """Raised when the Host TCP Protocol stream cannot be decoded."""


class HostTcpSerialBridge:
    """A serial-like transport backed by the Android Host TCP Protocol."""

    def __init__(
        self,
        host: str = HOST_TCP_LISTEN_HOST,
        port: int = 9000,
        timeout: float = TIMEOUT,
        server_id: str = "fNIRS-Decoder-1.0",
        debug: bool = False,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self.server_id = server_id
        self.debug = debug

        self._seq = 0
        self._closed = False
        self._capture_finished = False
        self._buffer = bytearray()
        self._buffer_cond = threading.Condition()
        self._send_lock = threading.Lock()
        self._client_sock: socket.socket | None = None

        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind((self.host, self.port))
        self._server_sock.listen(1)
        self._server_sock.settimeout(0.5)

        print(f"Waiting for Android TCP client on {self.host}:{self.port} ...")
        try:
            self._client_sock, client_addr = self._accept_client()
        except KeyboardInterrupt:
            self._server_sock.close()
            raise
        self._client_sock.settimeout(None)
        print(f"Android TCP client connected from {client_addr[0]}:{client_addr[1]}")

        self._rx_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._rx_thread.start()

    @property
    def in_waiting(self) -> int:
        with self._buffer_cond:
            return len(self._buffer)

    @property
    def closed(self) -> bool:
        return self._closed

    @property
    def is_open(self) -> bool:
        return not self._closed

    @property
    def capture_finished(self) -> bool:
        return self._capture_finished

    def reset_input_buffer(self) -> None:
        with self._buffer_cond:
            self._buffer.clear()

    def read(self, size: int = 1) -> bytes:
        """Read up to size bytes from serial_data payloads."""
        if size <= 0:
            return b""

        deadline = None if self.timeout is None else time.monotonic() + self.timeout
        with self._buffer_cond:
            while not self._buffer and not self._closed:
                if deadline is None:
                    self._buffer_cond.wait()
                    continue
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return b""
                self._buffer_cond.wait(timeout=remaining)

            if not self._buffer:
                return b""

            n = min(size, len(self._buffer))
            data = bytes(self._buffer[:n])
            del self._buffer[:n]
            return data

    def write(self, data: bytes) -> int:
        """PC does not send UART commands in the Android-driven TCP flow."""
        raise HostTcpProtocolError("serial_cmd is disabled in passive TCP mode.")

    def close(self) -> None:
        self._mark_closed()
        for sock in (getattr(self, "_client_sock", None), self._server_sock):
            try:
                if sock is not None:
                    sock.close()
            except Exception:
                pass

    def send_analysis_result(
        self,
        ok: bool,
        message: str | None = None,
    ) -> None:
        body: dict[str, Any] = {"ok": ok}
        if message:
            body["message"] = message
        self._send_message("analysis_result", body)

    def send_live_analysis_batch(
        self,
        times: list[float],
        hbo: list[float],
        hbr: list[float],
        window_start_s: float,
        window_end_s: float,
        ok: bool = True,
        message: str | None = None,
        *,
        rso2: list[float] | None = None,
        baseline_ready: bool = False,
        latest_rso2_pct: float | None = None,
        baseline_rso2_pct: float | None = None,
        replace_full_series: bool = False,
    ) -> None:
        """为安卓端绘图发送一组在线 HbO/HbR/rSO2 批次数据。"""
        body: dict[str, Any] = {
            "ok": ok,
            "sample_count": min(len(times), len(hbo), len(hbr)),
            "unit": "a.u.",
            "baseline_ready": baseline_ready,
            "replace_full_series": replace_full_series,
        }
        if ok:
            body.update(
                {
                    "times": times,
                    "hbo": hbo,
                    "hbr": hbr,
                    "window_start_s": window_start_s,
                    "window_end_s": window_end_s,
                }
            )
            if rso2 is not None:
                body["rso2"] = rso2
                body["sample_count"] = min(
                    len(times), len(hbo), len(hbr), len(rso2)
                )
            if latest_rso2_pct is not None and math.isfinite(latest_rso2_pct):
                body["latest_rso2_pct"] = float(latest_rso2_pct)
            if baseline_rso2_pct is not None:
                body["baseline_rso2_pct"] = float(baseline_rso2_pct)
        if message:
            body["message"] = message
        self._send_message("live_analysis_batch", body)

    def _next_seq(self) -> int:
        self._seq += 1
        return self._seq

    def _accept_client(self) -> tuple[socket.socket, tuple[str, int]]:
        while True:
            try:
                return self._server_sock.accept()
            except socket.timeout:
                continue

    def _send_message(self, msg_type: str, body: dict[str, Any]) -> None:
        if self._client_sock is None:
            raise HostTcpProtocolError("TCP client is not connected.")
        message = {
            "ver": HOST_TCP_PROTOCOL_VERSION,
            "type": msg_type,
            "seq": self._next_seq(),
            "ts_ms": int(time.time() * 1000),
            "body": body,
        }
        raw = json.dumps(message, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        if len(raw) > HOST_TCP_MAX_BODY_BYTES:
            raise HostTcpProtocolError(f"Message too large: {len(raw)} bytes.")
        packet = len(raw).to_bytes(4, byteorder="big", signed=False) + raw
        with self._send_lock:
            self._client_sock.sendall(packet)
        if self.debug:
            print(
                f"[TCP->Android] type={msg_type} seq={message['seq']} "
                f"byte_len={body.get('byte_len', '-')}"
            )

    def _send_error(self, code: str, message: str, ref_seq: int | None = None) -> None:
        body: dict[str, Any] = {"code": code, "message": message}
        if ref_seq is not None:
            body["ref_seq"] = ref_seq
        try:
            self._send_message("error", body)
        except Exception:
            self._mark_closed()

    def _recv_loop(self) -> None:
        try:
            while not self._closed:
                message = self._recv_message()
                self._handle_message(message)
        except (OSError, HostTcpProtocolError) as exc:
            if not self._closed:
                print(f"Android TCP connection closed: {exc}")
        finally:
            self._mark_closed()

    def _recv_exact(self, size: int) -> bytes:
        chunks = bytearray()
        while len(chunks) < size:
            chunk = self._client_sock.recv(size - len(chunks))
            if not chunk:
                raise HostTcpProtocolError("peer closed connection")
            chunks.extend(chunk)
        return bytes(chunks)

    def _recv_message(self) -> dict[str, Any]:
        header = self._recv_exact(4)
        body_len = int.from_bytes(header, byteorder="big", signed=False)
        if body_len < 1 or body_len > HOST_TCP_MAX_BODY_BYTES:
            raise HostTcpProtocolError(f"invalid body_length: {body_len}")

        body = self._recv_exact(body_len)
        try:
            message = json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise HostTcpProtocolError(f"invalid JSON body: {exc}") from exc
        if not isinstance(message, dict):
            raise HostTcpProtocolError("message root must be a JSON object")
        return message

    def _handle_message(self, message: dict[str, Any]) -> None:
        seq = message.get("seq")
        msg_type = message.get("type")
        body = message.get("body") or {}
        if self.debug:
            payload_len = "-"
            if isinstance(body, dict):
                payload_len = body.get("byte_len", "-")
            print(f"[Android->TCP] type={msg_type} seq={seq} byte_len={payload_len}")

        if message.get("ver") != HOST_TCP_PROTOCOL_VERSION:
            self._send_error("VERSION_MISMATCH", "Unsupported protocol version.", seq)
            return
        if not isinstance(body, dict):
            self._send_error("INVALID_JSON", "body must be an object.", seq)
            return

        if msg_type == "hello":
            self._send_message("hello_ack", {"server_id": self.server_id, "ok": True})
        elif msg_type == "serial_data":
            self._handle_serial_data(body, seq)
        elif msg_type == "heartbeat":
            return
        elif msg_type == "bye":
            print(f"Android TCP client sent bye: {body.get('reason', '')}")
            self._send_message("bye_ack", {"ref_seq": seq, "ok": True})
            with self._buffer_cond:
                self._capture_finished = True
                self._buffer_cond.notify_all()
        elif msg_type == "error":
            print(f"Android TCP error: {body}")
        else:
            self._send_error("UNSUPPORTED_TYPE", f"Unsupported message type: {msg_type}", seq)

    def _handle_serial_data(self, body: dict[str, Any], seq: int | None) -> None:
        payload_b64 = body.get("payload_b64", "")
        if not isinstance(payload_b64, str):
            self._send_error("INVALID_BASE64", "payload_b64 must be a string.", seq)
            return

        try:
            payload = base64.b64decode(payload_b64, validate=True)
        except (binascii.Error, ValueError):
            self._send_error("INVALID_BASE64", "payload_b64 is not valid standard Base64.", seq)
            return

        expected_len = body.get("byte_len")
        if expected_len is not None and expected_len != len(payload):
            self._send_error("LENGTH_MISMATCH", "byte_len does not match payload length.", seq)
            return

        if not payload:
            return
        with self._buffer_cond:
            self._buffer.extend(payload)
            self._buffer_cond.notify_all()

    def _mark_closed(self) -> None:
        with self._buffer_cond:
            self._closed = True
            self._buffer_cond.notify_all()
