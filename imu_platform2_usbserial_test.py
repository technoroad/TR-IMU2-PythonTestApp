from __future__ import annotations

import argparse
import signal
import sys
from typing import Optional

from imu_platform2_frames import (
    MotionFrame,
    SettingsFrame,
    format_motion_frame,
    format_settings_frame,
)
from imu_platform2_serial import (
    CMD_GET_SETTINGS,
    CMD_START_PERIODIC,
    CMD_STOP_PERIODIC,
    RSP_PERIODIC_MOTION,
    SERIAL_IMPORT_ERROR,
    SerialLink,
    add_serial_common_arguments,
    is_motion_response_id,
    is_settings_response_id,
    is_valid_motion_response_packet,
    is_valid_settings_response_packet,
    open_serial_port,
    parse_motion_packet,
    parse_settings_packet,
    print_unknown_packet,
    serial,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="IMU_Platform2 USB serial test app."
    )
    add_serial_common_arguments(parser)
    parser.add_argument(
        "--skip-settings",
        action="store_true",
        help="Skip the initial 0x70 settings request.",
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


def handle_decoded_packet(
    packet,
    *,
    print_raw_hex: bool,
) -> Optional[int]:
    motion_frame: Optional[MotionFrame] = parse_motion_packet(
        packet,
        log_unknown_on_error=True,
    )
    if motion_frame is not None:
        line = format_motion_frame(packet.response_id, motion_frame)
        if print_raw_hex:
            line += f" raw={packet.data.hex(' ')}"
        print(line)
        return motion_frame.imu_counter
    if is_motion_response_id(packet.response_id):
        return None

    settings_frame: Optional[SettingsFrame] = parse_settings_packet(
        packet,
        log_unknown_on_error=True,
    )
    if settings_frame is not None:
        line = format_settings_frame(packet.response_id, settings_frame)
        if print_raw_hex:
            line += f" raw={packet.data.hex(' ')}"
        print(line)
        return None
    if is_settings_response_id(packet.response_id):
        return None

    print_unknown_packet(packet)
    return None


def main() -> int:
    args = parse_args()

    if serial is None:
        print("[ERR] pyserial is required. Install it with: pip install pyserial", file=sys.stderr)
        print(f"[ERR] Import detail: {SERIAL_IMPORT_ERROR}", file=sys.stderr)
        return 1

    if args.print_every < 1:
        print("--print-every must be 1 or larger.", file=sys.stderr)
        return 2
    if args.response_retries < 0:
        print("--response-retries must be 0 or larger.", file=sys.stderr)
        return 2
    if args.tx_timeout < 0.0:
        print("--tx-timeout must be 0 or larger.", file=sys.stderr)
        return 2

    try:
        port = open_serial_port(
            args.port,
            tx_timeout_s=args.tx_timeout,
        )
    except Exception as exc:
        print(f"[ERR] Failed to open serial port '{args.port}': {exc}", file=sys.stderr)
        return 1

    link = SerialLink(port, tx_timeout_s=args.tx_timeout)
    drained_at_startup = link.drain_pending_frames(args.startup_drain_ms)
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
            settings_packet = link.send_command_and_wait(
                CMD_GET_SETTINGS,
                is_settings_response_id,
                "settings response 0x70-0x77",
                timeout_s=args.response_timeout,
                retries=args.response_retries,
                response_validator=is_valid_settings_response_packet,
                background_handler=lambda packet: handle_decoded_packet(
                    packet,
                    print_raw_hex=False,
                ),
            )
            if settings_packet is None:
                print(
                    f"[WARN] No settings response received after {max(1, args.response_retries + 1)} attempt(s). "
                    "Continuing without it."
                )
            else:
                handle_decoded_packet(
                    settings_packet,
                    print_raw_hex=args.print_raw_hex,
                )

        motion_packet = link.send_command_and_wait(
            CMD_START_PERIODIC,
            is_motion_response_id,
            "periodic motion response 0x20/0x30-0x33",
            timeout_s=args.response_timeout,
            retries=args.response_retries,
            response_validator=is_valid_motion_response_packet,
            background_handler=lambda packet: handle_decoded_packet(
                packet,
                print_raw_hex=False,
            ),
        )
        if motion_packet is None:
            print(
                f"[WARN] No periodic motion response received after {max(1, args.response_retries + 1)} attempt(s). "
                "Continuing to wait for late frames."
            )
        elif motion_packet.response_id == RSP_PERIODIC_MOTION:
            motion_frame = parse_motion_packet(
                motion_packet,
                log_unknown_on_error=True,
            )
            if motion_frame is not None:
                periodic_frame_count += 1
                if periodic_frame_count % args.print_every == 0:
                    line = format_motion_frame(motion_packet.response_id, motion_frame)
                    if args.print_raw_hex:
                        line += f" raw={motion_packet.data.hex(' ')}"
                    print(line)
        else:
            handle_decoded_packet(motion_packet, print_raw_hex=args.print_raw_hex)
        print("[INFO] Waiting for USB serial frames. Press Ctrl+C to stop.")

        while not stop_requested:
            packet = link.recv_packet(args.rx_timeout)
            if packet is None:
                continue

            if packet.response_id == RSP_PERIODIC_MOTION:
                motion_frame = parse_motion_packet(
                    packet,
                    log_unknown_on_error=True,
                )
                if motion_frame is None:
                    continue
                periodic_frame_count += 1
                if periodic_frame_count % args.print_every != 0:
                    continue
                line = format_motion_frame(packet.response_id, motion_frame)
                if args.print_raw_hex:
                    line += f" raw={packet.data.hex(' ')}"
                print(line)
                continue

            handle_decoded_packet(packet, print_raw_hex=args.print_raw_hex)

    except KeyboardInterrupt:
        stop_requested = True
    except Exception as exc:
        print(f"[ERR] Serial error: {exc}", file=sys.stderr)
        return 1
    finally:
        if not args.no_stop_on_exit:
            try:
                link.send_command(CMD_STOP_PERIODIC)
                link.drain_pending_frames(200)
            except Exception as exc:
                print(f"[WARN] Failed to send stop command: {exc}", file=sys.stderr)

        try:
            link.shutdown()
        except Exception as exc:  # pragma: no cover - defensive cleanup
            print(f"[WARN] Serial shutdown failed: {exc}", file=sys.stderr)

        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
