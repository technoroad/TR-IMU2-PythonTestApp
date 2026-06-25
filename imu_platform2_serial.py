from __future__ import annotations

"""USB virtual COM serial helpers for the IMU Platform2 tools."""

import argparse
import dataclasses
import struct
import sys
import time
from typing import Callable, Optional

from imu_platform2_frames import (
    MOTION_STRUCT,
    SETTINGS_STRUCT,
    MotionFrame,
    SettingsFrame,
    parse_motion_frame,
    parse_settings_frame,
)

try:
    import serial
except ImportError as exc:  # pragma: no cover - depends on target environment
    serial = None
    SERIAL_IMPORT_ERROR = exc
else:
    SERIAL_IMPORT_ERROR = None

HEADER = b"\xAA\xAA"
SERIAL_BAUDRATE = 921600

CMD_GET_MOTION_ONCE = 0x30
CMD_START_PERIODIC = 0x31
CMD_STOP_PERIODIC = 0x32
CMD_RESET_ATTITUDE = 0x33
CMD_RESET_FILTER = CMD_RESET_ATTITUDE
CMD_GET_SETTINGS = 0x70

RSP_PERIODIC_MOTION = 0x20
RSP_MOTION_MIN = 0x30
RSP_MOTION_MAX = 0x33
RSP_SETTINGS_MIN = 0x70
RSP_SETTINGS_MAX = 0x77

PacketHandler = Callable[["SerialPacket"], None]
ResponseMatcher = Callable[[int], bool]
ResponseValidator = Callable[["SerialPacket"], bool]


@dataclasses.dataclass(frozen=True)
class SerialPacket:
    response_id: int
    data: bytes
    checksum: int
    raw: bytes

    @property
    def length(self) -> int:
        return len(self.data)


def add_serial_common_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--port",
        required=True,
        help="USB serial TTY/COM port, e.g. /dev/ttyACM0 or COM3.",
    )
    parser.add_argument(
        "--startup-drain-ms",
        type=int,
        default=300,
        help="Discard pending receive frames before the first command.",
    )
    parser.add_argument(
        "--response-timeout",
        type=float,
        default=5.0,
        help="Timeout in seconds per attempt while waiting for a specific response.",
    )
    parser.add_argument(
        "--response-retries",
        type=int,
        default=2,
        help="Additional retries after a response timeout.",
    )
    parser.add_argument(
        "--tx-timeout",
        type=float,
        default=0.2,
        help="Transmit timeout in seconds. 0 disables the timeout.",
    )
    parser.add_argument(
        "--no-stop-on-exit",
        action="store_true",
        help="Do not send 0x32 when the script exits.",
    )


def _fold_ones_complement(total: int) -> int:
    while total > 0xFFFF:
        total = (total & 0xFFFF) + (total >> 16)
    return total


def compute_checksum(frame_id: int, data: bytes) -> int:
    total = frame_id + len(data) + sum(data)
    total = _fold_ones_complement(total)
    return (~total) & 0xFFFF


def verify_checksum(frame_id: int, data: bytes, checksum: int) -> bool:
    total = frame_id + len(data) + sum(data) + checksum
    total = _fold_ones_complement(total)
    return total == 0xFFFF


def encode_command_frame(command_id: int, data: bytes) -> bytes:
    payload = bytes((command_id, len(data))) + data
    checksum = compute_checksum(command_id, data)
    return HEADER + payload + struct.pack("<H", checksum)


def open_serial_port(
    port_name: str,
    *,
    tx_timeout_s: Optional[float],
):
    if serial is None:
        raise RuntimeError(str(SERIAL_IMPORT_ERROR))
    write_timeout = None if tx_timeout_s is None or tx_timeout_s <= 0.0 else tx_timeout_s
    return serial.Serial(
        port=port_name,
        baudrate=SERIAL_BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.0,
        write_timeout=write_timeout,
    )


def is_motion_response_id(response_id: int) -> bool:
    return response_id == RSP_PERIODIC_MOTION or RSP_MOTION_MIN <= response_id <= RSP_MOTION_MAX


def is_settings_response_id(response_id: int) -> bool:
    return RSP_SETTINGS_MIN <= response_id <= RSP_SETTINGS_MAX


def expected_response_length(response_id: int) -> Optional[int]:
    if is_motion_response_id(response_id) or is_settings_response_id(response_id):
        return 64
    return None


def print_unknown_packet(packet: SerialPacket) -> None:
    print(
        f"[RX] id=0x{packet.response_id:02X} len={packet.length} "
        f"data={packet.data.hex(' ')} checksum=0x{packet.checksum:04X}"
    )


def has_expected_payload_length(
    packet: SerialPacket,
    expected_length: int,
    *,
    log_unknown: bool = False,
) -> bool:
    actual_length = packet.length
    if actual_length != expected_length:
        print(
            f"[WARN] id=0x{packet.response_id:02X} expected {expected_length} bytes but got {actual_length}"
        )
        if log_unknown:
            print_unknown_packet(packet)
        return False
    return True


def is_valid_motion_response_packet(packet: SerialPacket) -> bool:
    return has_expected_payload_length(packet, MOTION_STRUCT.size)


def is_valid_settings_response_packet(packet: SerialPacket) -> bool:
    return has_expected_payload_length(packet, SETTINGS_STRUCT.size)


def parse_motion_packet(
    packet: SerialPacket,
    *,
    log_unknown_on_error: bool = False,
) -> Optional[MotionFrame]:
    if not is_motion_response_id(packet.response_id):
        return None
    if not has_expected_payload_length(
        packet,
        MOTION_STRUCT.size,
        log_unknown=log_unknown_on_error,
    ):
        return None
    return parse_motion_frame(packet.data)


def parse_settings_packet(
    packet: SerialPacket,
    *,
    log_unknown_on_error: bool = False,
) -> Optional[SettingsFrame]:
    if not is_settings_response_id(packet.response_id):
        return None
    if not has_expected_payload_length(
        packet,
        SETTINGS_STRUCT.size,
        log_unknown=log_unknown_on_error,
    ):
        return None
    return parse_settings_frame(packet.data)


class SerialLink:
    def __init__(
        self,
        port,
        *,
        tx_timeout_s: Optional[float] = None,
    ) -> None:
        self.port = port
        self.tx_timeout_s = tx_timeout_s
        self._rx_buffer = bytearray()

    def send_command(self, command_id: int, data: bytes = b"\x00") -> None:
        frame = encode_command_frame(command_id, data)
        self.port.write(frame)
        self.port.flush()
        print(
            f"[TX] id=0x{command_id:02X} len={len(data)} "
            f"data={data.hex(' ')} checksum=0x{compute_checksum(command_id, data):04X}"
        )

    def _read_available_bytes(self) -> bytes:
        waiting = getattr(self.port, "in_waiting", 0)
        chunk_size = waiting if waiting > 0 else 1
        return self.port.read(chunk_size)

    def _extract_packet(self) -> Optional[SerialPacket]:
        while True:
            if len(self._rx_buffer) < 4:
                return None

            header_index = self._rx_buffer.find(HEADER)
            if header_index < 0:
                if self._rx_buffer[-1:] == HEADER[:1]:
                    del self._rx_buffer[:-1]
                else:
                    self._rx_buffer.clear()
                return None
            if header_index > 0:
                del self._rx_buffer[:header_index]

            if len(self._rx_buffer) < 4:
                return None

            response_id = self._rx_buffer[2]
            payload_length = self._rx_buffer[3]
            expected_length = expected_response_length(response_id)
            if expected_length is None:
                del self._rx_buffer[0]
                continue
            if payload_length != expected_length:
                print(
                    f"[WARN] Dropping invalid serial frame header: "
                    f"id=0x{response_id:02X} len={payload_length} expected={expected_length}",
                    file=sys.stderr,
                )
                del self._rx_buffer[0]
                continue

            total_length = 2 + 1 + 1 + expected_length + 2
            if len(self._rx_buffer) < total_length:
                return None

            data_offset = 4
            checksum_offset = data_offset + expected_length
            data = bytes(self._rx_buffer[data_offset:checksum_offset])
            checksum = struct.unpack_from("<H", self._rx_buffer, checksum_offset)[0]

            if not verify_checksum(response_id, data, checksum):
                print(
                    f"[WARN] Dropping serial frame with invalid checksum: "
                    f"id=0x{response_id:02X} len={expected_length} checksum=0x{checksum:04X}",
                    file=sys.stderr,
                )
                # Slide by one byte so we can quickly recover sync after a dropped byte.
                del self._rx_buffer[0]
                continue

            raw = bytes(self._rx_buffer[:total_length])
            del self._rx_buffer[:total_length]

            return SerialPacket(
                response_id=response_id,
                data=data,
                checksum=checksum,
                raw=raw,
            )

    def recv_packet(self, timeout_s: float) -> Optional[SerialPacket]:
        deadline = time.monotonic() + max(0.0, timeout_s)
        allow_nonblocking_probe = timeout_s <= 0.0

        while True:
            packet = self._extract_packet()
            if packet is not None:
                return packet

            chunk = b""
            if allow_nonblocking_probe or time.monotonic() <= deadline:
                chunk = self._read_available_bytes()
                allow_nonblocking_probe = False
            if chunk:
                self._rx_buffer.extend(chunk)
                continue

            remaining = deadline - time.monotonic()
            if timeout_s <= 0.0:
                return None
            if remaining < 0.0:
                return None

            time.sleep(min(0.005, max(0.0, remaining)))

    def drain_pending_frames(self, duration_ms: int) -> int:
        deadline = time.monotonic() + max(0, duration_ms) / 1000.0
        drained = 0
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            packet = self.recv_packet(min(0.01, remaining))
            if packet is None:
                continue
            drained += 1
        return drained

    def poll_pending_packets(
        self,
        max_batch: int = 256,
        *,
        handler: Optional[PacketHandler] = None,
    ) -> int:
        handled = 0
        for _ in range(max_batch):
            packet = self.recv_packet(0.0)
            if packet is None:
                break
            handled += 1
            if handler is not None:
                handler(packet)
        return handled

    def wait_for_response(
        self,
        response_matches: ResponseMatcher,
        timeout_s: float,
        *,
        response_validator: Optional[ResponseValidator] = None,
        background_handler: Optional[PacketHandler] = None,
    ) -> Optional[SerialPacket]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            packet = self.recv_packet(min(0.1, remaining))
            if packet is None:
                continue
            if response_matches(packet.response_id):
                if response_validator is None or response_validator(packet):
                    return packet
                continue
            if background_handler is not None:
                background_handler(packet)
        return None

    def send_command_and_wait(
        self,
        command_id: int,
        response_matches: ResponseMatcher,
        response_description: str,
        *,
        data: bytes = b"\x00",
        timeout_s: float,
        retries: int,
        response_validator: Optional[ResponseValidator] = None,
        background_handler: Optional[PacketHandler] = None,
    ) -> Optional[SerialPacket]:
        attempt_count = max(1, retries + 1)

        for attempt_index in range(attempt_count):
            self.send_command(command_id, data=data)
            response = self.wait_for_response(
                response_matches,
                timeout_s,
                response_validator=response_validator,
                background_handler=background_handler,
            )
            if response is not None:
                return response
            if attempt_index + 1 < attempt_count:
                print(
                    f"[WARN] Timed out waiting for {response_description} after {timeout_s:.1f}s "
                    f"(attempt {attempt_index + 1}/{attempt_count}). Retrying."
                )
                self.drain_pending_frames(50)

        return None

    def shutdown(self) -> None:
        self.port.close()
