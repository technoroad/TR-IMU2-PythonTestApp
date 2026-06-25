from __future__ import annotations

"""Shared motion/settings payloads reused by CAN FD and USB serial variants."""

import dataclasses
import struct

MOTION_STRUCT = struct.Struct("<BIhhhhiiiiiihHHHHBQ8x")
SETTINGS_STRUCT = struct.Struct("<BIIBBBQQHBHBBBBB26x")

BOARD_NAMES = {
    0: "TR-IMU16607",
    1: "TR-IMU-Platform2",
}

MODEL_NAMES = {
    0x00: "Undefined",
    0x03: "ADIS16XXX-1",
    0x07: "ADIS16XXX-2",
    0x0F: "ADIS16XXX-3",
}

IMU_MAKER_NAMES = {
    0: "Undefined",
    1: "Analog Devices",
}

FILTER_NAMES = {
    0: "Disabled",
    1: "MKAE",
    2: "VQF (double)",
    3: "VQF (double+float)",
    4: "VQF (float)",
    5: "Madgwick",
}


@dataclasses.dataclass(frozen=True)
class MotionFrame:
    mcu_error: int
    send_counter: int
    quat_w_raw: int
    quat_x_raw: int
    quat_y_raw: int
    quat_z_raw: int
    acc_x_raw: int
    acc_y_raw: int
    acc_z_raw: int
    gyro_x_raw: int
    gyro_y_raw: int
    gyro_z_raw: int
    temperature_raw: int
    imu_counter: int
    imu_dropped: int
    computation_time_us: int
    spi_transaction_time_us: int
    pps_in_state: int
    timestamp_us: int

    @property
    def raw_quaternion(self) -> tuple[float, float, float, float]:
        scale = 1.0 / 32767.0
        return (
            self.quat_w_raw * scale,
            self.quat_x_raw * scale,
            self.quat_y_raw * scale,
            self.quat_z_raw * scale,
        )

    @property
    def viewer_quaternion(self) -> tuple[float, float, float, float]:
        w, x, y, z = self.raw_quaternion
        # Map device axes to the front-facing viewer coordinate system.
        return (w, x, z, -y)

    @property
    def temperature_c(self) -> float:
        return self.temperature_raw * 0.1


@dataclasses.dataclass(frozen=True)
class SettingsFrame:
    mcu_error: int
    send_counter: int
    build_date: int
    peripheral_enable: int
    read_32bit: int
    filter_select: int
    accl_sensitivity_raw: int
    gyro_sensitivity_raw: int
    sample_rate_hz: int
    imu_maker: int
    product_id: int
    model: int
    board: int
    grav_corr_en: int
    pps_in_pull: int
    pps_in_edge: int

    @property
    def accl_sensitivity(self) -> float:
        return self.accl_sensitivity_raw * 1e-6

    @property
    def gyro_sensitivity(self) -> float:
        return self.gyro_sensitivity_raw * 1e-6

    @property
    def fdcan_enabled(self) -> bool:
        return bool(self.peripheral_enable & 0x02)


def parse_motion_frame(data: bytes) -> MotionFrame:
    return MotionFrame(*MOTION_STRUCT.unpack(data))


def parse_settings_frame(data: bytes) -> SettingsFrame:
    return SettingsFrame(*SETTINGS_STRUCT.unpack(data))


def decode_peripheral_enable(value: int) -> str:
    names = []
    if value & 0x01:
        names.append("USB")
    if value & 0x02:
        names.append("FDCAN")
    if value & 0x04:
        names.append("UART")
    return ",".join(names) if names else "none"


def format_motion_frame(frame_id: int, frame: MotionFrame) -> str:
    quat_w, quat_x, quat_y, quat_z = frame.raw_quaternion
    return (
        f"[RX] id=0x{frame_id:03X} err=0x{frame.mcu_error:02X} "
        f"send={frame.send_counter} imu={frame.imu_counter} dropped={frame.imu_dropped} "
        f"quat=({quat_w:+.4f}, {quat_x:+.4f}, {quat_y:+.4f}, {quat_z:+.4f}) "
        f"temp={frame.temperature_c:+.1f}C ts_us={frame.timestamp_us}"
    )


def format_settings_frame(frame_id: int, frame: SettingsFrame) -> str:
    return (
        f"[RX] id=0x{frame_id:03X} err=0x{frame.mcu_error:02X} "
        f"build={frame.build_date} board={BOARD_NAMES.get(frame.board, 'Unknown')} "
        f"model={MODEL_NAMES.get(frame.model, 'Unknown')} "
        f"imu_maker={IMU_MAKER_NAMES.get(frame.imu_maker, 'Unknown')} "
        f"product_id={frame.product_id} if={decode_peripheral_enable(frame.peripheral_enable)} "
        f"read_32bit={frame.read_32bit} filter={FILTER_NAMES.get(frame.filter_select, 'Unknown')} "
        f"sample_rate={frame.sample_rate_hz}Hz grav_corr={frame.grav_corr_en} "
        f"acc_sens={frame.accl_sensitivity:.6f} gyro_sens={frame.gyro_sensitivity:.6f}"
    )
