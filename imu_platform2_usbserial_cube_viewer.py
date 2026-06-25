from __future__ import annotations

import argparse
import signal
import sys
import threading
from typing import Optional

from imu_platform2_frames import MotionFrame, format_settings_frame
from imu_platform2_serial import (
    CMD_GET_SETTINGS,
    CMD_RESET_FILTER,
    CMD_START_PERIODIC,
    CMD_STOP_PERIODIC,
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
    serial,
)
from imu_platform2_viewer import CubeViewer, TK_IMPORT_ERROR, tk


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render IMU quaternion data from USB serial as a rotating cuboid."
    )
    add_serial_common_arguments(parser)
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

    if serial is None:
        print("[ERR] pyserial is required. Install it with: pip install pyserial", file=sys.stderr)
        print(f"[ERR] Import detail: {SERIAL_IMPORT_ERROR}", file=sys.stderr)
        return 1

    if args.response_retries < 0:
        print("--response-retries must be 0 or larger.", file=sys.stderr)
        return 2
    if args.tx_timeout < 0.0:
        print("--tx-timeout must be 0 or larger.", file=sys.stderr)
        return 2
    if args.width < 320 or args.height < 240:
        print("Window size is too small.", file=sys.stderr)
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
    drained = link.drain_pending_frames(args.startup_drain_ms)
    if drained > 0:
        print(f"[INFO] Discarded {drained} pending frame(s) before start.")

    initial_motion_frame: Optional[MotionFrame] = None
    latest_motion_frame: Optional[MotionFrame] = None
    motion_lock = threading.Lock()
    reader_stop_event = threading.Event()

    def handle_background_packet(packet) -> None:
        motion_frame = parse_motion_packet(packet)
        if motion_frame is not None:
            nonlocal latest_motion_frame
            with motion_lock:
                latest_motion_frame = motion_frame
            return
        if is_motion_response_id(packet.response_id):
            return

        settings_frame = parse_settings_packet(packet)
        if settings_frame is not None:
            print(format_settings_frame(packet.response_id, settings_frame))
            return
        if is_settings_response_id(packet.response_id):
            return

    try:
        settings_packet = link.send_command_and_wait(
            CMD_GET_SETTINGS,
            is_settings_response_id,
            "settings response 0x70-0x77",
            timeout_s=args.response_timeout,
            retries=args.response_retries,
            response_validator=is_valid_settings_response_packet,
            background_handler=handle_background_packet,
        )
        if settings_packet is None:
            print(
                f"[WARN] No settings response received after {max(1, args.response_retries + 1)} attempt(s). "
                "Continuing without it."
            )
        else:
            settings_frame = parse_settings_packet(settings_packet)
            if settings_frame is not None:
                print(format_settings_frame(settings_packet.response_id, settings_frame))

        motion_packet = link.send_command_and_wait(
            CMD_START_PERIODIC,
            is_motion_response_id,
            "periodic motion response 0x20/0x30-0x33",
            timeout_s=args.response_timeout,
            retries=args.response_retries,
            response_validator=is_valid_motion_response_packet,
            background_handler=handle_background_packet,
        )
        if motion_packet is None:
            print(
                f"[WARN] No periodic motion response received after {max(1, args.response_retries + 1)} attempt(s). "
                "Opening the viewer and continuing to poll."
            )
        else:
            initial_motion_frame = parse_motion_packet(motion_packet)
        print("[INFO] Starting USB serial cube viewer. Close the window or press Ctrl+C to stop.")
    except Exception as exc:
        print(f"[ERR] Serial error while starting viewer: {exc}", file=sys.stderr)
        try:
            link.shutdown()
        except Exception:
            pass
        return 1

    def send_reset_command() -> None:
        try:
            link.send_command(CMD_RESET_FILTER)
            print("[INFO] Sent CMD_RESET_FILTER due to canvas click.")
        except Exception as exc:
            print(f"[WARN] Failed to send CMD_RESET_FILTER on canvas click: {exc}", file=sys.stderr)

    def apply_pending_motion() -> None:
        nonlocal latest_motion_frame
        with motion_lock:
            frame = latest_motion_frame
            latest_motion_frame = None
        if frame is not None:
            viewer.apply_motion_frame(frame)

    viewer = CubeViewer(
        width=args.width,
        height=args.height,
        fps=args.fps,
        poll_callback=apply_pending_motion,
        reset_callback=send_reset_command,
        window_title="IMU Platform2 USB Serial Cube Viewer",
    )
    if initial_motion_frame is not None:
        viewer.apply_motion_frame(initial_motion_frame)

    def serial_reader_loop() -> None:
        while not reader_stop_event.is_set():
            try:
                packet = link.recv_packet(0.05)
            except Exception as exc:
                if not reader_stop_event.is_set():
                    print(f"[WARN] Serial reader stopped: {exc}", file=sys.stderr)
                return
            if packet is None:
                continue
            handle_background_packet(packet)

    reader_thread = threading.Thread(
        target=serial_reader_loop,
        name="imu-platform2-usbserial-reader",
        daemon=True,
    )
    reader_thread.start()

    def request_stop(_signum: int, _frame: object) -> None:
        viewer.request_stop()

    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        viewer.run()
    finally:
        reader_stop_event.set()
        reader_thread.join(timeout=0.2)

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
