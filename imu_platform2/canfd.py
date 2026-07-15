from __future__ import annotations

"""CAN FD transport/protocol helpers for the IMU Platform2 tools."""

import argparse
import os
import subprocess
import sys
import time
from typing import Callable, Optional

import can

from .frames import (
    MOTION_STRUCT,
    SETTINGS_STRUCT,
    MotionFrame,
    SettingsFrame,
    parse_motion_frame,
    parse_settings_frame,
)

ARBITRATION_BITRATE = 500000
DATA_BITRATE = 2000000
RESTART_MS = 1000
SAMPLE_POINT = 0.8
DATA_SAMPLE_POINT = 0.8

CMD_GET_MOTION_ONCE = 0x030
CMD_START_PERIODIC = 0x031
CMD_STOP_PERIODIC = 0x032
CMD_RESET_ATTITUDE = 0x033
CMD_RESET_FILTER = CMD_RESET_ATTITUDE
CMD_GET_SETTINGS = 0x070

RSP_PERIODIC_MOTION = 0x120
RSP_MOTION_MIN = 0x130
RSP_MOTION_MAX = 0x133
RSP_SETTINGS_MIN = 0x170
RSP_SETTINGS_MAX = 0x177

MessageHandler = Callable[[can.Message], None]
ResponseMatcher = Callable[[int], bool]
ResponseValidator = Callable[[can.Message], bool]


def add_canfd_common_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--channel", default="can0", help="SocketCAN channel name.")
    parser.set_defaults(setup_link=True)
    parser.add_argument(
        "--setup-link",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--no-setup-link",
        dest="setup_link",
        action="store_false",
        help="Skip automatic SocketCAN interface configuration before opening the bus.",
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        default=ARBITRATION_BITRATE,
        help="Arbitration bitrate in bps.",
    )
    parser.add_argument(
        "--dbitrate",
        type=int,
        default=DATA_BITRATE,
        help="Data phase bitrate in bps.",
    )
    parser.add_argument(
        "--restart-ms",
        type=int,
        default=RESTART_MS,
        help="Automatic CAN controller restart time in ms.",
    )
    parser.add_argument(
        "--sample-point",
        type=float,
        default=SAMPLE_POINT,
        help="Arbitration sample point.",
    )
    parser.add_argument(
        "--dsample-point",
        type=float,
        default=DATA_SAMPLE_POINT,
        help="Data phase sample point.",
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
        default=3.0,
        help="Timeout in seconds per attempt while waiting for a specific response.",
    )
    parser.add_argument(
        "--response-retries",
        type=int,
        default=5,
        help="Additional retries after a response timeout.",
    )
    parser.add_argument(
        "--tx-timeout",
        type=float,
        default=0.2,
        help="Transmit timeout in seconds. 0 disables the timeout.",
    )
    parser.add_argument(
        "--no-brs",
        action="store_true",
        help="Disable Bit Rate Switch on transmitted CAN FD frames.",
    )
    parser.add_argument(
        "--no-stop-on-exit",
        action="store_true",
        help="Do not send 0x032 when the script exits.",
    )


def run_ip_link(args: argparse.Namespace) -> None:
    command_prefix = []
    if hasattr(os, "geteuid") and os.geteuid() != 0:
        command_prefix = ["sudo"]

    down_cmd = command_prefix + ["ip", "link", "set", args.channel, "down"]
    up_cmd = command_prefix + [
        "ip",
        "link",
        "set",
        args.channel,
        "up",
        "type",
        "can",
        "bitrate",
        str(args.bitrate),
        "dbitrate",
        str(args.dbitrate),
        "restart-ms",
        str(args.restart_ms),
        "berr-reporting",
        "on",
        "sample-point",
        f"{args.sample_point:.3f}",
        "dsample-point",
        f"{args.dsample_point:.3f}",
        "fd",
        "on",
    ]

    subprocess.run(down_cmd, check=False)
    subprocess.run(up_cmd, check=True)


def open_bus(channel: str) -> can.BusABC:
    return can.interface.Bus(channel=channel, interface="socketcan", fd=True)


def print_can_error_hints(args: argparse.Namespace, exc: Exception) -> None:
    message = str(exc)
    if "Invalid argument" in message:
        print(
            "[HINT] SocketCAN bus must be opened with CAN FD enabled and the Linux link must also be in FD mode.",
            file=sys.stderr,
        )
    if "Network is down" in message:
        print(
            f"[HINT] Check that automatic setup for {args.channel} succeeded, or configure it manually.",
            file=sys.stderr,
        )
        print(
            f"[HINT] Example: sudo ip link set {args.channel} up type can bitrate {args.bitrate} dbitrate {args.dbitrate} restart-ms {args.restart_ms} berr-reporting on sample-point {args.sample_point:.3f} dsample-point {args.dsample_point:.3f} fd on",
            file=sys.stderr,
        )


def is_fd_frame(message: can.Message) -> bool:
    return bool(getattr(message, "is_fd", False))


def is_motion_response_id(arbitration_id: int) -> bool:
    return arbitration_id == RSP_PERIODIC_MOTION or RSP_MOTION_MIN <= arbitration_id <= RSP_MOTION_MAX


def is_settings_response_id(arbitration_id: int) -> bool:
    return RSP_SETTINGS_MIN <= arbitration_id <= RSP_SETTINGS_MAX


def print_unknown_frame(message: can.Message) -> None:
    print(
        f"[RX] id=0x{message.arbitration_id:03X} dlc={message.dlc} "
        f"fd={bool(getattr(message, 'is_fd', False))} data={message.data.hex(' ')}"
    )


def has_expected_payload_length(
    message: can.Message,
    expected_length: int,
    *,
    log_unknown: bool = False,
) -> bool:
    actual_length = len(message.data)
    if actual_length != expected_length:
        print(
            f"[WARN] id=0x{message.arbitration_id:03X} expected {expected_length} bytes but got {actual_length}"
        )
        if log_unknown:
            print_unknown_frame(message)
        return False
    return True


def is_valid_motion_response_message(message: can.Message) -> bool:
    return has_expected_payload_length(message, MOTION_STRUCT.size)


def is_valid_settings_response_message(message: can.Message) -> bool:
    return has_expected_payload_length(message, SETTINGS_STRUCT.size)


def parse_motion_message(
    message: can.Message,
    *,
    log_unknown_on_error: bool = False,
) -> Optional[MotionFrame]:
    if not is_fd_frame(message):
        return None
    if not is_motion_response_id(message.arbitration_id):
        return None
    if not has_expected_payload_length(
        message,
        MOTION_STRUCT.size,
        log_unknown=log_unknown_on_error,
    ):
        return None
    return parse_motion_frame(bytes(message.data))


def parse_settings_message(
    message: can.Message,
    *,
    log_unknown_on_error: bool = False,
) -> Optional[SettingsFrame]:
    if not is_fd_frame(message):
        return None
    if not is_settings_response_id(message.arbitration_id):
        return None
    if not has_expected_payload_length(
        message,
        SETTINGS_STRUCT.size,
        log_unknown=log_unknown_on_error,
    ):
        return None
    return parse_settings_frame(bytes(message.data))


class CanFdLink:
    def __init__(
        self,
        bus: can.BusABC,
        *,
        bitrate_switch: bool = True,
        tx_timeout_s: Optional[float] = None,
    ) -> None:
        self.bus = bus
        self.bitrate_switch = bitrate_switch
        self.tx_timeout_s = tx_timeout_s

    def send_command(
        self,
        arbitration_id: int,
        data: bytes = b"\x00",
        *,
        bitrate_switch: Optional[bool] = None,
        timeout_s: Optional[float] = None,
    ) -> None:
        message = can.Message(
            arbitration_id=arbitration_id,
            is_extended_id=False,
            is_fd=True,
            bitrate_switch=self.bitrate_switch if bitrate_switch is None else bitrate_switch,
            data=data,
        )
        timeout = self.tx_timeout_s if timeout_s is None else timeout_s
        send_timeout = None if timeout is None or timeout <= 0.0 else timeout
        self.bus.send(message, timeout=send_timeout)
        print(
            f"[TX] id=0x{arbitration_id:03X} dlc={message.dlc} data={message.data.hex(' ')}"
        )

    def recv_message(self, timeout_s: float) -> Optional[can.Message]:
        return self.bus.recv(timeout=timeout_s)

    def drain_pending_frames(self, duration_ms: int) -> int:
        deadline = time.monotonic() + max(0, duration_ms) / 1000.0
        drained = 0
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            message = self.bus.recv(timeout=min(0.01, remaining))
            if message is None:
                continue
            drained += 1
        return drained

    def poll_pending_messages(
        self,
        max_batch: int = 256,
        *,
        handler: Optional[MessageHandler] = None,
    ) -> int:
        handled = 0
        for _ in range(max_batch):
            message = self.bus.recv(timeout=0.0)
            if message is None:
                break
            handled += 1
            if handler is not None:
                handler(message)
        return handled

    def wait_for_response(
        self,
        response_matches: ResponseMatcher,
        timeout_s: float,
        *,
        response_validator: Optional[ResponseValidator] = None,
        background_handler: Optional[MessageHandler] = None,
    ) -> Optional[can.Message]:
        deadline = time.monotonic() + max(0.0, timeout_s)
        allow_nonblocking_probe = timeout_s <= 0.0
        while allow_nonblocking_probe or time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            recv_timeout = 0.0 if allow_nonblocking_probe else min(0.1, remaining)
            message = self.bus.recv(timeout=recv_timeout)
            allow_nonblocking_probe = False
            if message is None:
                continue
            if not is_fd_frame(message):
                continue
            if response_matches(message.arbitration_id):
                if response_validator is None or response_validator(message):
                    return message
                continue
            if background_handler is not None:
                background_handler(message)
        return None

    def send_command_and_wait(
        self,
        arbitration_id: int,
        response_matches: ResponseMatcher,
        response_description: str,
        *,
        data: bytes = b"\x00",
        timeout_s: float,
        retries: int,
        response_validator: Optional[ResponseValidator] = None,
        background_handler: Optional[MessageHandler] = None,
    ) -> Optional[can.Message]:
        attempt_count = max(1, retries + 1)

        for attempt_index in range(attempt_count):
            self.send_command(arbitration_id, data=data)
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
        self.bus.shutdown()
