from __future__ import annotations

import argparse
import signal
import subprocess
import sys
from typing import Optional

import can

from imu_platform2_canfd import (
    CMD_GET_SETTINGS,
    CMD_START_PERIODIC,
    CMD_STOP_PERIODIC,
    RSP_PERIODIC_MOTION,
    CanFdLink,
    add_canfd_common_arguments,
    is_fd_frame,
    is_motion_response_id,
    is_settings_response_id,
    is_valid_motion_response_message,
    is_valid_settings_response_message,
    open_bus,
    parse_motion_message,
    parse_settings_message,
    print_can_error_hints,
    print_unknown_frame,
    run_ip_link,
)
from imu_platform2_frames import (
    MotionFrame,
    SettingsFrame,
    format_motion_frame,
    format_settings_frame,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="IMU_Platform2 CAN FD test app for Jetson Orin Nano."
    )
    add_canfd_common_arguments(parser)
    parser.add_argument(
        "--skip-settings",
        action="store_true",
        help="Skip the initial 0x070 settings request.",
    )
    parser.add_argument(
        "--rx-timeout",
        type=float,
        default=0.2,
        help="Receive timeout in seconds.",
    )
    parser.add_argument(
        "--print-every",
        type=int,
        default=20,
        help="Print every Nth periodic frame. 1 prints all frames.",
    )
    parser.add_argument(
        "--print-raw-hex",
        action="store_true",
        help="Also print the raw payload bytes for decoded frames.",
    )
    return parser.parse_args()


def handle_decoded_frame(
    message: can.Message,
    *,
    print_raw_hex: bool,
) -> Optional[int]:
    motion_frame: Optional[MotionFrame] = parse_motion_message(
        message,
        log_unknown_on_error=True,
    )
    if motion_frame is not None:
        line = format_motion_frame(message.arbitration_id, motion_frame)
        if print_raw_hex:
            line += f" raw={bytes(message.data).hex(' ')}"
        print(line)
        return motion_frame.imu_counter
    if is_motion_response_id(message.arbitration_id):
        return None

    settings_frame: Optional[SettingsFrame] = parse_settings_message(
        message,
        log_unknown_on_error=True,
    )
    if settings_frame is not None:
        line = format_settings_frame(message.arbitration_id, settings_frame)
        if print_raw_hex:
            line += f" raw={bytes(message.data).hex(' ')}"
        print(line)
        if not settings_frame.fdcan_enabled:
            print("[WARN] FDCAN bit is disabled in peripheral_enable.")
        return None
    if is_settings_response_id(message.arbitration_id):
        return None

    if is_fd_frame(message):
        print_unknown_frame(message)
    return None


def main() -> int:
    args = parse_args()

    if args.print_every < 1:
        print("--print-every must be 1 or larger.", file=sys.stderr)
        return 2
    if args.response_retries < 0:
        print("--response-retries must be 0 or larger.", file=sys.stderr)
        return 2
    if args.tx_timeout < 0.0:
        print("--tx-timeout must be 0 or larger.", file=sys.stderr)
        return 2

    if args.setup_link:
        try:
            run_ip_link(args)
            print(
                f"[INFO] Configured {args.channel}: bitrate={args.bitrate} dbitrate={args.dbitrate}"
            )
        except FileNotFoundError:
            print("[ERR] 'ip' command was not found. Run this on Jetson Linux.", file=sys.stderr)
            return 1
        except subprocess.CalledProcessError as exc:
            print(f"[ERR] Failed to configure {args.channel}: {exc}", file=sys.stderr)
            return 1

    try:
        bus = open_bus(args.channel)
    except OSError as exc:
        print(f"[ERR] Failed to open SocketCAN channel '{args.channel}': {exc}", file=sys.stderr)
        return 1
    except can.CanError as exc:
        print(f"[ERR] Failed to open CAN bus: {exc}", file=sys.stderr)
        return 1

    link = CanFdLink(
        bus,
        bitrate_switch=not args.no_brs,
        tx_timeout_s=args.tx_timeout,
    )
    try:
        drained_at_startup = link.drain_pending_frames(args.startup_drain_ms)
    except can.CanError as exc:
        print(f"[ERR] CAN error: {exc}", file=sys.stderr)
        print_can_error_hints(args, exc)
        try:
            link.shutdown()
        except Exception:
            pass
        return 1
    if drained_at_startup > 0:
        print(f"[INFO] Discarded {drained_at_startup} pending frame(s) before start.")

    stop_requested = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    periodic_frame_count = 0

    try:
        if not args.skip_settings:
            settings_message = link.send_command_and_wait(
                CMD_GET_SETTINGS,
                is_settings_response_id,
                "settings response 0x170-0x177",
                timeout_s=args.response_timeout,
                retries=args.response_retries,
                response_validator=is_valid_settings_response_message,
                background_handler=lambda message: handle_decoded_frame(
                    message,
                    print_raw_hex=False,
                ),
            )
            if settings_message is None:
                print(
                    f"[WARN] No settings response received after {max(1, args.response_retries + 1)} attempt(s). "
                    "Continuing without it."
                )
            else:
                handle_decoded_frame(
                    settings_message,
                    print_raw_hex=args.print_raw_hex,
                )

        motion_message = link.send_command_and_wait(
            CMD_START_PERIODIC,
            is_motion_response_id,
            "periodic motion response 0x120/0x130-0x133",
            timeout_s=args.response_timeout,
            retries=args.response_retries,
            response_validator=is_valid_motion_response_message,
            background_handler=lambda message: handle_decoded_frame(
                message,
                print_raw_hex=False,
            ),
        )
        if motion_message is None:
            print(
                f"[WARN] No periodic motion response received after {max(1, args.response_retries + 1)} attempt(s). "
                "Continuing to wait for late frames."
            )
        elif motion_message.arbitration_id == RSP_PERIODIC_MOTION:
            motion_frame = parse_motion_message(
                motion_message,
                log_unknown_on_error=True,
            )
            if motion_frame is not None:
                periodic_frame_count += 1
                if periodic_frame_count % args.print_every == 0:
                    line = format_motion_frame(motion_message.arbitration_id, motion_frame)
                    if args.print_raw_hex:
                        line += f" raw={bytes(motion_message.data).hex(' ')}"
                    print(line)
        else:
            handle_decoded_frame(motion_message, print_raw_hex=args.print_raw_hex)
        print("[INFO] Waiting for CAN FD frames. Press Ctrl+C to stop.")

        while not stop_requested:
            message = link.recv_message(args.rx_timeout)
            if message is None:
                continue
            if not is_fd_frame(message):
                continue

            if message.arbitration_id == RSP_PERIODIC_MOTION:
                motion_frame = parse_motion_message(
                    message,
                    log_unknown_on_error=True,
                )
                if motion_frame is None:
                    continue
                periodic_frame_count += 1
                if periodic_frame_count % args.print_every != 0:
                    continue
                line = format_motion_frame(message.arbitration_id, motion_frame)
                if args.print_raw_hex:
                    line += f" raw={bytes(message.data).hex(' ')}"
                print(line)
                continue

            handle_decoded_frame(message, print_raw_hex=args.print_raw_hex)

    except KeyboardInterrupt:
        stop_requested = True
    except can.CanError as exc:
        print(f"[ERR] CAN error: {exc}", file=sys.stderr)
        print_can_error_hints(args, exc)
        return 1
    finally:
        if not args.no_stop_on_exit:
            try:
                link.send_command(CMD_STOP_PERIODIC)
                link.drain_pending_frames(200)
            except can.CanError as exc:
                print(f"[WARN] Failed to send stop command: {exc}", file=sys.stderr)

        try:
            link.shutdown()
        except Exception as exc:  # pragma: no cover - defensive cleanup
            print(f"[WARN] Bus shutdown failed: {exc}", file=sys.stderr)

        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
