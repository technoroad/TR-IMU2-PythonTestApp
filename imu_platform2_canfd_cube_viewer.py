from __future__ import annotations

import argparse
import signal
import subprocess
import sys
from typing import Optional

import can

from imu_platform2_canfd import (
    CMD_GET_SETTINGS,
    CMD_RESET_FILTER,
    CMD_START_PERIODIC,
    CMD_STOP_PERIODIC,
    CanFdLink,
    add_canfd_common_arguments,
    is_motion_response_id,
    is_settings_response_id,
    is_valid_motion_response_message,
    is_valid_settings_response_message,
    open_bus,
    parse_motion_message,
    parse_settings_message,
    print_can_error_hints,
    run_ip_link,
)
from imu_platform2_frames import MotionFrame, format_settings_frame
from imu_platform2_viewer import CubeViewer, TK_IMPORT_ERROR, tk


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render IMU quaternion data as a rotating cuboid."
    )
    add_canfd_common_arguments(parser)
    parser.add_argument("--width", type=int, default=960, help="Window width.")
    parser.add_argument("--height", type=int, default=720, help="Window height.")
    parser.add_argument("--fps", type=int, default=60, help="Display refresh cap.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if tk is None:
        print(
            "[ERR] tkinter is required. On Windows, install the official Python with Tcl/Tk support. On Ubuntu/Jetson, run: sudo apt install python3-tk",
            file=sys.stderr,
        )
        print(f"[ERR] Import detail: {TK_IMPORT_ERROR}", file=sys.stderr)
        return 1

    if args.response_timeout < 0.0:
        print("--response-timeout must be 0 or larger.", file=sys.stderr)
        return 2
    if args.response_retries < 0:
        print("--response-retries must be 0 or larger.", file=sys.stderr)
        return 2
    if args.tx_timeout < 0.0:
        print("--tx-timeout must be 0 or larger.", file=sys.stderr)
        return 2
    if args.width < 320 or args.height < 240:
        print("Window size is too small.", file=sys.stderr)
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
        drained = link.drain_pending_frames(args.startup_drain_ms)
    except can.CanError as exc:
        print(f"[ERR] CAN error: {exc}", file=sys.stderr)
        print_can_error_hints(args, exc)
        try:
            link.shutdown()
        except Exception:
            pass
        return 1
    if drained > 0:
        print(f"[INFO] Discarded {drained} pending frame(s) before start.")

    def shutdown_link(*, send_stop_command: bool) -> None:
        if send_stop_command and not args.no_stop_on_exit:
            try:
                link.send_command(CMD_STOP_PERIODIC)
                link.drain_pending_frames(200)
            except can.CanError as exc:
                print(f"[WARN] Failed to send stop command: {exc}", file=sys.stderr)

        try:
            link.shutdown()
        except Exception as exc:  # pragma: no cover - defensive cleanup
            print(f"[WARN] Bus shutdown failed: {exc}", file=sys.stderr)

    initial_motion_frame: Optional[MotionFrame] = None
    viewer: Optional[CubeViewer] = None

    def handle_background_message(message: can.Message) -> None:
        nonlocal viewer

        motion_frame = parse_motion_message(message)
        if motion_frame is not None:
            if viewer is not None:
                viewer.apply_motion_frame(motion_frame)
            return
        if is_motion_response_id(message.arbitration_id):
            return

        settings_frame = parse_settings_message(message)
        if settings_frame is not None:
            print(format_settings_frame(message.arbitration_id, settings_frame))
            return
        if is_settings_response_id(message.arbitration_id):
            return

    try:
        settings_message = link.send_command_and_wait(
            CMD_GET_SETTINGS,
            is_settings_response_id,
            "settings response 0x170-0x177",
            timeout_s=args.response_timeout,
            retries=args.response_retries,
            response_validator=is_valid_settings_response_message,
            background_handler=handle_background_message,
        )
        if settings_message is None:
            print(
                f"[WARN] No settings response received after {max(1, args.response_retries + 1)} attempt(s). "
                "Continuing without it."
            )
        else:
            settings_frame = parse_settings_message(settings_message)
            if settings_frame is not None:
                print(format_settings_frame(settings_message.arbitration_id, settings_frame))

        motion_message = link.send_command_and_wait(
            CMD_START_PERIODIC,
            is_motion_response_id,
            "periodic motion response 0x120/0x130-0x133",
            timeout_s=args.response_timeout,
            retries=args.response_retries,
            response_validator=is_valid_motion_response_message,
            background_handler=handle_background_message,
        )
        if motion_message is None:
            print(
                f"[WARN] No periodic motion response received after {max(1, args.response_retries + 1)} attempt(s). "
                "Opening the viewer and continuing to poll."
            )
        else:
            initial_motion_frame = parse_motion_message(motion_message)
        print("[INFO] Starting cube viewer. Close the window or press Ctrl+C to stop.")
    except can.CanError as exc:
        print(f"[ERR] CAN error while starting viewer: {exc}", file=sys.stderr)
        print_can_error_hints(args, exc)
        shutdown_link(send_stop_command=True)
        return 1

    def send_reset_command() -> None:
        try:
            link.send_command(CMD_RESET_FILTER)
            print("[INFO] Sent CMD_RESET_FILTER due to canvas click.")
        except can.CanError as exc:
            print(f"[WARN] Failed to send CMD_RESET_FILTER on canvas click: {exc}", file=sys.stderr)

    def poll_messages() -> None:
        link.poll_pending_messages(handler=handle_background_message)

    try:
        viewer = CubeViewer(
            width=args.width,
            height=args.height,
            fps=args.fps,
            poll_callback=poll_messages,
            reset_callback=send_reset_command,
        )
    except tk.TclError as exc:
        print(f"[ERR] Failed to open viewer window: {exc}", file=sys.stderr)
        print(
            "[HINT] Do not launch the Tk viewer with sudo. Run it as your normal user.",
            file=sys.stderr,
        )
        print(
            "[HINT] This script can invoke sudo only for ip link. Run the viewer itself as your normal user.",
            file=sys.stderr,
        )
        print(
            f"[HINT] Example: python {__file__.split('/')[-1]} --channel {args.channel}",
            file=sys.stderr,
        )
        shutdown_link(send_stop_command=True)
        return 1
    if initial_motion_frame is not None:
        viewer.apply_motion_frame(initial_motion_frame)

    def request_stop(_signum: int, _frame: object) -> None:
        viewer.request_stop()

    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        viewer.run()
    finally:
        shutdown_link(send_stop_command=True)
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
