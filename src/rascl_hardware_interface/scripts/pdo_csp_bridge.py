#!/usr/bin/env python3

"""Cyclic PDO bridge for the four-axis RASCL robot.

The FAULHABER MC 5004 P ET standard PDO set is used:

RxPDO2 / 0x1601, assigned to SyncManager 2:
    0x6040:00 Controlword      U16
    0x607A:00 Target Position S32

TxPDO2 / 0x1A01, assigned to SyncManager 3:
    0x6041:00 Statusword            U16
    0x6064:00 Position Actual Value S32

ROS commands and feedback are exchanged with the C++ ros2_control hardware
interface through a local Unix datagram socket. SDO is used only during startup
configuration. Position targets and feedback are transferred cyclically by PDO.
"""

from __future__ import annotations

import errno
import gc
import math
import os
import socket
import struct
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pysoem

try:
    import yaml
except ImportError as exc:  # pragma: no cover - only relevant in incomplete containers
    raise RuntimeError("PyYAML is required for ethercat_pdo.yaml") from exc


EXPECTED_SLAVE_COUNT = 4
FAULHABER_VENDOR_ID = 0x00000147
JOINT_NAMES = [
    "shoulder_joint",
    "upperarm_joint",
    "lowerarm_joint",
    "end_effector_joint",
]

# Physical transmission and expected sensor resolution from the supplied hardware data.
# Axes 1-3: IEF3-4096 evaluated in quadrature -> 4 * 4096 increments/rev.
# End effector: analogue Hall position resolution -> 4096 increments/rev.
#
# 0x607A and 0x6064 are expressed in the drive's configured Factor Group units,
# not necessarily raw encoder counts. The bridge therefore reads 0x608F, 0x6091
# and 0x6092 from every drive and derives the actual PDO units per physical output
# revolution at runtime. This avoids multiplying the physical gear ratio twice when
# it is already configured in the Motion Controller.
EXPECTED_ENCODER_INCREMENTS_PER_MOTOR_REVOLUTION = [16384, 16384, 16384, 4096]
PHYSICAL_GEAR_RATIOS = [196.0, 196.0, 196.0, 323.0]

# CiA 402 / FAULHABER objects.
CONTROL_WORD = 0x6040
STATUS_WORD = 0x6041
MODES_OF_OPERATION = 0x6060
MODES_OF_OPERATION_DISPLAY = 0x6061
TARGET_POSITION = 0x607A
ACTUAL_POSITION = 0x6064
ABORT_CONNECTION_OPTION = 0x6007
QUICK_STOP_OPTION = 0x605A
HALT_OPTION = 0x605D
CYCLIC_INTERPOLATION_RATE = 0x2332
POSITION_ENCODER_RESOLUTION = 0x608F
GEAR_RATIO = 0x6091
FEED_CONSTANT = 0x6092
POLARITY = 0x607E
MAXIMUM_MOTOR_SPEED = 0x6080

# PDO mapping and SyncManager assignment objects.
RXPDO2_MAPPING = 0x1601
TXPDO2_MAPPING = 0x1A01
SM2_RXPDO_ASSIGNMENT = 0x1C12
SM3_TXPDO_ASSIGNMENT = 0x1C13

# Mapping entries: index (16 bits), subindex (8 bits), bit length (8 bits).
MAP_CONTROLWORD_U16 = 0x60400010
MAP_TARGET_POSITION_S32 = 0x607A0020
MAP_STATUSWORD_U16 = 0x60410010
MAP_ACTUAL_POSITION_S32 = 0x60640020

CSP_MODE = 8
CONTROLWORD_SHUTDOWN = 0x0006
CONTROLWORD_SWITCH_ON = 0x0007
CONTROLWORD_ENABLE_OPERATION = 0x000F
CONTROLWORD_HALT = 0x010F
CONTROLWORD_DISABLE_VOLTAGE = 0x0000

STATUS_STATE_MASK = 0x006F
STATUS_OPERATION_ENABLED = 0x0027
STATUS_BIT_FAULT = 1 << 3
STATUS_BIT_WARNING = 1 << 7
STATUS_BIT_FAULHABER_ERROR = 1 << 8
STATUS_BIT_INTERNAL_LIMIT = 1 << 11
STATUS_BIT_FOLLOWS_COMMAND = 1 << 12
STATUS_BIT_FOLLOWING_ERROR = 1 << 13

# IPC protocol. Both processes run on the same little-endian Linux host.
IPC_VERSION = 1
COMMAND_MAGIC = b"RCMD"
FEEDBACK_MAGIC = b"RFDB"
COMMAND_STRUCT = struct.Struct("<4sIIQQ4d")
FEEDBACK_STRUCT = struct.Struct("<4sIQQIi4d4HI")

COMMAND_FLAG_REGISTER = 1 << 0
COMMAND_FLAG_POSITION_VALID = 1 << 1
COMMAND_FLAG_HALT = 1 << 2

BRIDGE_STATE_STARTING = 0
BRIDGE_STATE_READY = 1
BRIDGE_STATE_HOLD = 2
BRIDGE_STATE_FAULT = 3
BRIDGE_STATE_STOPPED = 4

ERROR_WKC = 1 << 0
ERROR_DRIVE_FAULT = 1 << 1
ERROR_NOT_FOLLOWING = 1 << 2
ERROR_FOLLOWING_ERROR = 1 << 3
ERROR_INTERNAL_LIMIT = 1 << 4
ERROR_WARNING = 1 << 5
ERROR_COMMAND_WATCHDOG = 1 << 6
ERROR_MOTION_DISABLED = 1 << 7
ERROR_INVALID_COMMAND = 1 << 8
ERROR_ETHERCAT_STATE = 1 << 9
ERROR_TRACKING = 1 << 10
ERROR_TIMING = 1 << 11

FOLLOWING_ERROR_WINDOW = 0x6065
FOLLOWING_ERROR_TIMEOUT = 0x6066

END_EFFECTOR_INDEX = 3


@dataclass(frozen=True)
class PdoConfig:
    interface: str
    command_socket: str
    ready_file: str
    cycle_time_us: int
    processdata_timeout_us: int
    use_distributed_clocks: bool
    dc_shift_ns: int
    realtime_priority: int
    cpu_affinity: int | None
    command_watchdog_ms: int
    feedback_log_period_s: float
    wkc_fault_cycles: int
    state_fault_cycles: int
    timing_fault_cycles: int
    max_cycle_lateness_us: int
    abort_connection_option_code: int
    quick_stop_option_code: int
    halt_option_code: int
    startup_mode: str
    allow_motion: bool
    allow_unhomed_motion: bool
    gripper_reference_valid: bool
    position_min_rad: list[float]
    position_max_rad: list[float]
    max_velocity_rad_s: list[float]
    max_tracking_error_rad: list[float]
    drive_following_error_window_rad: list[float]
    drive_following_error_timeout: list[int]
    minimum_motor_speed_rpm: list[int]
    first_command_tolerance_rad: list[float]
    initial_sync_limit_tolerance_rad: list[float]
    model_to_drive_sign: list[float]
    drive_rad_at_model_zero: list[float]

    @property
    def cycle_time_s(self) -> float:
        return self.cycle_time_us / 1_000_000.0

    @property
    def cycle_time_ns(self) -> int:
        return self.cycle_time_us * 1000

    @property
    def interpolation_rate_100us(self) -> int:
        if self.cycle_time_us % 100 != 0:
            raise ValueError("cycle_time_us must be an integer multiple of 100 us")
        value = self.cycle_time_us // 100
        if not 1 <= value <= 0xFFFF:
            raise ValueError("0x2332 interpolation rate does not fit in U16")
        return value


def _require_joint_vector(value: Any, field: str) -> list[float]:
    if not isinstance(value, list) or len(value) != EXPECTED_SLAVE_COUNT:
        raise ValueError(f"{field} must contain exactly four values")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{field} contains a non-finite value")
    return result


def _require_joint_int_vector(value: Any, field: str) -> list[int]:
    if not isinstance(value, list) or len(value) != EXPECTED_SLAVE_COUNT:
        raise ValueError(f"{field} must contain exactly four values")
    result = [int(item) for item in value]
    return result


def find_pdo_config_path() -> Path:
    candidates: list[Path] = []
    env_path = os.environ.get("RASCL_PDO_CONFIG")
    if env_path:
        candidates.append(Path(env_path))

    candidates.extend(
        [
            Path(
                "/root/ws/install/rascl_hardware_interface/share/"
                "rascl_hardware_interface/config/ethercat_pdo.yaml"
            ),
            Path(
                "/root/ws/src/rascl_hardware_interface/config/ethercat_pdo.yaml"
            ),
            Path.cwd()
            / "src/rascl_hardware_interface/config/ethercat_pdo.yaml",
        ]
    )

    for path in candidates:
        if path.is_file():
            return path
    raise FileNotFoundError(
        "ethercat_pdo.yaml was not found; set RASCL_PDO_CONFIG explicitly"
    )


def load_pdo_config(path: Path | None = None) -> PdoConfig:
    selected = path or find_pdo_config_path()
    with selected.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}

    ethercat = raw.get("ethercat", {}) or {}
    startup = raw.get("startup", {}) or {}
    safety = raw.get("safety", {}) or {}
    runtime = raw.get("runtime", {}) or {}
    kinematics = raw.get("kinematics", {}) or {}

    startup_mode = os.environ.get(
        "RASCL_STARTUP_MODE",
        str(startup.get("mode", "hold_current")),
    ).strip().lower()
    cycle_time_us = int(ethercat.get("cycle_time_us", 10000))
    config = PdoConfig(
        interface=os.environ.get(
            "RASCL_ETHERCAT_INTERFACE",
            str(ethercat.get("interface", "enx3c18a026482c")),
        ),
        command_socket=os.environ.get(
            "RASCL_CSP_SOCKET",
            str(runtime.get("command_socket", "/tmp/rascl_csp.sock")),
        ),
        ready_file=os.environ.get(
            "RASCL_PDO_READY_FILE",
            str(runtime.get("ready_file", "/tmp/rascl_pdo_ready")),
        ),
        cycle_time_us=cycle_time_us,
        processdata_timeout_us=int(
            ethercat.get("processdata_timeout_us", max(1000, cycle_time_us // 2))
        ),
        use_distributed_clocks=bool(
            ethercat.get("use_distributed_clocks", True)
        ),
        dc_shift_ns=int(ethercat.get("dc_shift_ns", 0)),
        realtime_priority=int(runtime.get("realtime_priority", 50)),
        cpu_affinity=(
            None
            if runtime.get("cpu_affinity", None) is None
            else int(runtime["cpu_affinity"])
        ),
        command_watchdog_ms=int(runtime.get("command_watchdog_ms", 250)),
        feedback_log_period_s=float(runtime.get("feedback_log_period_s", 1.0)),
        wkc_fault_cycles=int(runtime.get("wkc_fault_cycles", 3)),
        state_fault_cycles=int(runtime.get("state_fault_cycles", 5)),
        timing_fault_cycles=int(runtime.get("timing_fault_cycles", 3)),
        max_cycle_lateness_us=int(runtime.get("max_cycle_lateness_us", 2000)),
        abort_connection_option_code=int(
            safety.get("abort_connection_option_code", 3)
        ),
        quick_stop_option_code=int(safety.get("quick_stop_option_code", 5)),
        halt_option_code=int(safety.get("halt_option_code", 1)),
        startup_mode=startup_mode,
        allow_motion=bool(safety.get("allow_motion", False)),
        allow_unhomed_motion=bool(
            safety.get("allow_unhomed_motion", False)
        ),
        gripper_reference_valid=bool(
            safety.get("gripper_reference_valid", False)
        ),
        position_min_rad=_require_joint_vector(
            safety.get(
                "position_min_rad", [-1.570796327, -1.5708, -1.571000, 0.0]
            ),
            "position_min_rad",
        ),
        position_max_rad=_require_joint_vector(
            safety.get(
                "position_max_rad", [1.570796327, 1.5708, 1.571000, 1.571000]
            ),
            "position_max_rad",
        ),
        max_velocity_rad_s=_require_joint_vector(
            safety.get("max_velocity_rad_s", [6.0, 6.0, 6.0, 0.8]),
            "max_velocity_rad_s",
        ),
        max_tracking_error_rad=_require_joint_vector(
            safety.get("max_tracking_error_rad", [0.08, 0.08, 0.08, 0.10]),
            "max_tracking_error_rad",
        ),
        drive_following_error_window_rad=_require_joint_vector(
            safety.get(
                "drive_following_error_window_rad", [0.08, 0.08, 0.06, 0.02]
            ),
            "drive_following_error_window_rad",
        ),
        drive_following_error_timeout=_require_joint_int_vector(
            safety.get("drive_following_error_timeout", [500, 500, 500, 100]),
            "drive_following_error_timeout",
        ),
        minimum_motor_speed_rpm=_require_joint_int_vector(
            safety.get("minimum_motor_speed_rpm", [0, 0, 0, 500]),
            "minimum_motor_speed_rpm",
        ),
        first_command_tolerance_rad=_require_joint_vector(
            safety.get(
                "first_command_tolerance_rad", [0.03, 0.03, 0.03, 0.05]
            ),
            "first_command_tolerance_rad",
        ),
        initial_sync_limit_tolerance_rad=_require_joint_vector(
            safety.get(
                "initial_sync_limit_tolerance_rad", [0.001, 0.001, 0.001, 0.001],
            ),
            "initial_sync_limit_tolerance_rad",
        ),
        model_to_drive_sign=_require_joint_vector(
            kinematics.get("model_to_drive_sign", [1.0, 1.0, 1.0, 1.0]),
            "model_to_drive_sign",
        ),
        drive_rad_at_model_zero=_require_joint_vector(
            kinematics.get("drive_rad_at_model_zero", [0.0, 0.0, 0.0, 0.0]),
            "drive_rad_at_model_zero",
        ),
    )

    allowed_startup_modes = {
        "home_then_csp",
        "home_then_pick_ready",
        "pick_ready_only",
        "hold_current",
    }
    if config.startup_mode not in allowed_startup_modes:
        raise ValueError(
            "startup.mode must be one of "
            + ", ".join(sorted(allowed_startup_modes))
        )
    if config.gripper_reference_valid:
        raise ValueError(
            "safety.gripper_reference_valid is no longer a manual bypass. "
            "Keep it false; runtime readiness is set only after successful "
            "end-effector homing and switch verification."
        )

    if (
        config.allow_motion
        and config.startup_mode not in {"home_then_csp", "home_then_pick_ready"}
        and not config.allow_unhomed_motion
    ):
        raise ValueError(
            "Trajectory motion is blocked because startup homing is skipped. "
            "Keep safety.allow_motion=false for PDO communication tests, or set "
            "safety.allow_unhomed_motion=true only after explicitly accepting "
            "the unreferenced-motion risk."
        )

    if config.cycle_time_us <= 0:
        raise ValueError("cycle_time_us must be positive")
    _ = config.interpolation_rate_100us
    if config.processdata_timeout_us <= 0:
        raise ValueError("processdata_timeout_us must be positive")
    if config.command_watchdog_ms <= config.cycle_time_us / 1000:
        raise ValueError("command_watchdog_ms must exceed one PDO cycle")
    if config.timing_fault_cycles <= 0:
        raise ValueError("timing_fault_cycles must be positive")
    if config.max_cycle_lateness_us < 0:
        raise ValueError("max_cycle_lateness_us must not be negative")

    for index, name in enumerate(JOINT_NAMES):
        if config.position_min_rad[index] >= config.position_max_rad[index]:
            raise ValueError(f"Invalid position limits for {name}")
        if config.max_velocity_rad_s[index] <= 0.0:
            raise ValueError(f"Invalid velocity limit for {name}")
        if config.max_tracking_error_rad[index] <= 0.0:
            raise ValueError(f"Invalid tracking-error limit for {name}")
        if config.drive_following_error_window_rad[index] <= 0.0:
            raise ValueError(f"Invalid drive following-error window for {name}")
        if not 0 <= config.drive_following_error_timeout[index] <= 0xFFFF:
            raise ValueError(f"Invalid drive following-error timeout for {name}")
        if not 0 <= config.minimum_motor_speed_rpm[index] <= 0xFFFFFFFF:
            raise ValueError(f"Invalid minimum motor speed for {name}")
        if config.initial_sync_limit_tolerance_rad[index] < 0.0:
            raise ValueError(
                f"Invalid initial synchronization tolerance for {name}"
            )
        if config.model_to_drive_sign[index] not in (-1.0, 1.0):
            raise ValueError(
                f"model_to_drive_sign for {name} must be exactly -1 or +1"
            )

    print(f"Loaded PDO configuration: {selected}", flush=True)
    return config


def pack_u8(value: int) -> bytes:
    return struct.pack("<B", int(value))


def pack_s8(value: int) -> bytes:
    return struct.pack("<b", int(value))


def pack_u16(value: int) -> bytes:
    return struct.pack("<H", int(value))


def pack_s16(value: int) -> bytes:
    return struct.pack("<h", int(value))


def pack_u32(value: int) -> bytes:
    return struct.pack("<I", int(value))


def pack_s32(value: int) -> bytes:
    return struct.pack("<i", int(value))


def unpack_s8(data: bytes) -> int:
    return struct.unpack("<b", data)[0]


def unpack_u8(data: bytes) -> int:
    return struct.unpack("<B", data)[0]


def unpack_u16(data: bytes) -> int:
    return struct.unpack("<H", data)[0]


def unpack_u32(data: bytes) -> int:
    return struct.unpack("<I", data)[0]


def unpack_s32(data: bytes) -> int:
    return struct.unpack("<i", data)[0]


def decode_drive_state(statusword: int) -> str:
    masked = statusword & STATUS_STATE_MASK
    names = {
        0x0000: "not-ready-to-switch-on",
        0x0040: "switch-on-disabled",
        0x0021: "ready-to-switch-on",
        0x0023: "switched-on",
        0x0027: "operation-enabled",
        0x0007: "quick-stop-active",
        0x000F: "fault-reaction-active",
        0x0008: "fault",
    }
    return names.get(masked, f"unknown-0x{masked:04X}")


class PdoCspBridge:
    """FAULHABER CSP bridge using RxPDO2 and TxPDO2."""

    def __init__(
        self,
        config: PdoConfig,
        master: Any | None = None,
        master_already_initialized: bool = False,
        arm_reference_valid: bool = False,
        gripper_reference_valid: bool = False,
        reference_scope: str = "none",
    ):
        self.config = config
        self.run_id = os.environ.get("RASCL_BRIDGE_RUN_ID", "")
        self.master = master if master is not None else pysoem.Master()
        self.master_already_initialized = bool(master_already_initialized)
        self.arm_reference_valid = bool(arm_reference_valid)
        self.gripper_reference_valid = bool(gripper_reference_valid)
        self.reference_scope = str(reference_scope)
        self.slaves: list[Any] = []
        self.socket: socket.socket | None = None
        self.client_address: str | None = None
        self.expected_wkc = 0
        self.pdo_mapped = False
        # The S32 PDO values are user-defined Factor Group position units.
        # Variable names retain ``counts`` for protocol continuity, but conversion
        # uses the runtime-derived units-per-physical-output-revolution values.
        self.position_units_per_output_revolution = [0.0] * EXPECTED_SLAVE_COUNT
        self.actual_counts = [0] * EXPECTED_SLAVE_COUNT
        self.actual_positions = [0.0] * EXPECTED_SLAVE_COUNT
        self.target_positions = [0.0] * EXPECTED_SLAVE_COUNT
        self.applied_target_positions = [0.0] * EXPECTED_SLAVE_COUNT
        self.target_counts = [0] * EXPECTED_SLAVE_COUNT
        self.statuswords = [0] * EXPECTED_SLAVE_COUNT
        self.latest_command_sequence = 0
        self.last_valid_command_time: float | None = None
        self.last_changed_command_time: float | None = None
        self.last_received_command = [0.0] * EXPECTED_SLAVE_COUNT
        self.have_valid_command = False
        self.halt_requested = False
        self.bridge_state = BRIDGE_STATE_STARTING
        self.error_flags = 0
        self.bad_wkc_cycles = 0
        self.bad_state_cycles = 0
        self.bad_timing_cycles = 0
        self.last_cycle_lateness_us = 0.0
        self.cycle_count = 0
        self._last_log_time = 0.0
        self._running = False

    def _read_u32(self, slave: Any, index: int, subindex: int) -> int:
        return unpack_u32(slave.sdo_read(index, subindex))

    def _configure_position_scaling(self, slave: Any, joint_index: int) -> None:
        """Derive 0x607A/0x6064 units per physical output revolution.

        FAULHABER's Factor Group converts internal encoder increments to the
        user-defined position values exposed at 0x607A and 0x6064. For one
        physical output revolution:

            units = physical_gear_ratio / configured_gear_ratio * feed_constant

        where configured_gear_ratio is 0x6091.01 / 0x6091.02 and
        feed_constant is 0x6092.01 / 0x6092.02. This covers both common setups:
        a default 1:1 software gear ratio and a drive already configured with the
        physical gearbox ratio.
        """
        encoder_increments = self._read_u32(
            slave, POSITION_ENCODER_RESOLUTION, 0x01
        )
        encoder_motor_revolutions = self._read_u32(
            slave, POSITION_ENCODER_RESOLUTION, 0x02
        )
        gear_motor_revolutions = self._read_u32(slave, GEAR_RATIO, 0x01)
        gear_output_revolutions = self._read_u32(slave, GEAR_RATIO, 0x02)
        feed_units = self._read_u32(slave, FEED_CONSTANT, 0x01)
        feed_output_revolutions = self._read_u32(slave, FEED_CONSTANT, 0x02)
        polarity = unpack_u8(slave.sdo_read(POLARITY, 0x00))

        values = {
            "encoder_motor_revolutions": encoder_motor_revolutions,
            "gear_motor_revolutions": gear_motor_revolutions,
            "gear_output_revolutions": gear_output_revolutions,
            "feed_units": feed_units,
            "feed_output_revolutions": feed_output_revolutions,
        }
        for name, value in values.items():
            if value <= 0:
                raise RuntimeError(
                    f"Slave {joint_index} has invalid Factor Group {name}={value}"
                )

        encoder_resolution = (
            float(encoder_increments) / float(encoder_motor_revolutions)
        )
        expected_encoder_resolution = float(
            EXPECTED_ENCODER_INCREMENTS_PER_MOTOR_REVOLUTION[joint_index]
        )
        if not math.isclose(
            encoder_resolution, expected_encoder_resolution, rel_tol=0.0, abs_tol=0.5
        ):
            raise RuntimeError(
                f"{JOINT_NAMES[joint_index]} encoder resolution mismatch: "
                f"0x608F={encoder_increments}/{encoder_motor_revolutions} "
                f"({encoder_resolution:g}), expected {expected_encoder_resolution:g}"
            )

        configured_gear_ratio = (
            float(gear_motor_revolutions) / float(gear_output_revolutions)
        )
        feed_constant = float(feed_units) / float(feed_output_revolutions)
        units_per_output_revolution = (
            PHYSICAL_GEAR_RATIOS[joint_index]
            / configured_gear_ratio
            * feed_constant
        )
        if not math.isfinite(units_per_output_revolution) or (
            units_per_output_revolution <= 0.0
        ):
            raise RuntimeError(
                f"Invalid position scaling for {JOINT_NAMES[joint_index]}: "
                f"{units_per_output_revolution}"
            )

        drive_limit_positions = [
            self._model_to_drive_radians(
                self.config.position_min_rad[joint_index], joint_index
            ),
            self._model_to_drive_radians(
                self.config.position_max_rad[joint_index], joint_index
            ),
        ]
        maximum_abs_position = max(abs(value) for value in drive_limit_positions)
        maximum_abs_units = (
            maximum_abs_position
            / (2.0 * math.pi)
            * units_per_output_revolution
        )
        if maximum_abs_units > (2**31 - 1):
            raise RuntimeError(
                f"Configured range for {JOINT_NAMES[joint_index]} exceeds S32 "
                f"with the drive Factor Group ({maximum_abs_units:.0f} units)"
            )

        self.position_units_per_output_revolution[joint_index] = (
            units_per_output_revolution
        )
        print(
            f"{JOINT_NAMES[joint_index]} scaling: "
            f"0x608F={encoder_increments}/{encoder_motor_revolutions}, "
            f"0x6091={gear_motor_revolutions}/{gear_output_revolutions} "
            f"({configured_gear_ratio:g}:1), "
            f"0x6092={feed_units}/{feed_output_revolutions}, "
            f"0x607E=0x{polarity:02X}, "
            f"PDO_units/output_rev={units_per_output_revolution:g}, "
            f"model_to_drive_sign={self.config.model_to_drive_sign[joint_index]:+.0f}, "
            f"drive_zero={self.config.drive_rad_at_model_zero[joint_index]:+.6f} rad",
            flush=True,
        )

    def _model_to_drive_radians(self, model_radians: float, joint_index: int) -> float:
        """Map a ROS/URDF joint angle to the post-homing drive coordinate.

        This mapping is used only by the CSP runtime. The proven homing sequence
        and method-37 zero assignment remain completely unchanged.
        """
        return (
            self.config.drive_rad_at_model_zero[joint_index]
            + self.config.model_to_drive_sign[joint_index] * float(model_radians)
        )

    def _drive_to_model_radians(self, drive_radians: float, joint_index: int) -> float:
        sign = self.config.model_to_drive_sign[joint_index]
        return sign * (
            float(drive_radians)
            - self.config.drive_rad_at_model_zero[joint_index]
        )

    def _radians_to_position_units(self, model_radians: float, joint_index: int) -> int:
        units_per_revolution = self.position_units_per_output_revolution[joint_index]
        if units_per_revolution <= 0.0:
            raise RuntimeError(
                f"Position scaling for {JOINT_NAMES[joint_index]} is unavailable"
            )
        drive_radians = self._model_to_drive_radians(model_radians, joint_index)
        units = round(
            drive_radians / (2.0 * math.pi) * units_per_revolution
        )
        if not -(2**31) <= units <= 2**31 - 1:
            raise OverflowError(
                f"Target for {JOINT_NAMES[joint_index]} does not fit in S32: {units}"
            )
        return int(units)

    def _position_units_to_radians(self, units: int, joint_index: int) -> float:
        units_per_revolution = self.position_units_per_output_revolution[joint_index]
        if units_per_revolution <= 0.0:
            raise RuntimeError(
                f"Position scaling for {JOINT_NAMES[joint_index]} is unavailable"
            )
        drive_radians = float(units) / units_per_revolution * 2.0 * math.pi
        return self._drive_to_model_radians(drive_radians, joint_index)

    def _configure_runtime_scheduling(self) -> None:
        if self.config.cpu_affinity is not None:
            try:
                os.sched_setaffinity(0, {self.config.cpu_affinity})
                print(
                    f"Pinned PDO process to CPU {self.config.cpu_affinity}",
                    flush=True,
                )
            except (AttributeError, PermissionError, OSError) as exc:
                print(f"Could not set CPU affinity: {exc}", flush=True)

        if self.config.realtime_priority > 0:
            try:
                os.sched_setscheduler(
                    0,
                    os.SCHED_FIFO,
                    os.sched_param(self.config.realtime_priority),
                )
                print(
                    f"Enabled SCHED_FIFO priority {self.config.realtime_priority}",
                    flush=True,
                )
            except (AttributeError, PermissionError, OSError) as exc:
                print(
                    "Could not enable real-time scheduling; PDO timing must be "
                    f"reviewed before motion: {exc}",
                    flush=True,
                )

    def _open_ipc_socket(self) -> None:
        try:
            os.unlink(self.config.command_socket)
        except FileNotFoundError:
            pass

        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.bind(self.config.command_socket)
        self.socket.setblocking(False)
        os.chmod(self.config.command_socket, 0o660)
        print(f"CSP IPC socket: {self.config.command_socket}", flush=True)

    def _remove_ready_file(self) -> None:
        try:
            os.unlink(self.config.ready_file)
        except FileNotFoundError:
            pass

    def _create_ready_file(self) -> None:
        path = Path(self.config.ready_file)
        temporary_path = path.with_name(f"{path.name}.{os.getpid()}.tmp")
        positions = ",".join(f"{value:.9f}" for value in self.actual_positions)
        temporary_path.write_text(
            "PDO_READY\n"
            f"run_id={self.run_id}\n"
            f"cycle_time_us={self.config.cycle_time_us}\n"
            f"expected_wkc={self.expected_wkc}\n"
            f"startup_mode={self.config.startup_mode}\n"
            f"reference_valid={str(self.arm_reference_valid).lower()}\n"
            f"reference_scope={self.reference_scope}\n"
            f"gripper_reference_valid={str(self.gripper_reference_valid).lower()}\n"
            f"allow_motion={str(self.config.allow_motion).lower()}\n"
            f"actual_positions_rad={positions}\n",
            encoding="utf-8",
        )
        os.replace(temporary_path, path)
        print(f"PDO bridge ready file created: {path}", flush=True)

    def _write_mapping_entry(
        self, slave: Any, mapping_index: int, subindex: int, value: int
    ) -> None:
        slave.sdo_write(mapping_index, subindex, pack_u32(value))

    def _read_statusword_sdo(self, slave: Any) -> int:
        return unpack_u16(slave.sdo_read(STATUS_WORD, 0x00))

    def _wait_for_drive_state(
        self,
        slave: Any,
        expected_state: int,
        label: str,
        timeout_s: float = 1.0,
    ) -> int:
        deadline = time.monotonic() + timeout_s
        last_status = 0
        while time.monotonic() < deadline:
            last_status = self._read_statusword_sdo(slave)
            if (last_status & STATUS_STATE_MASK) == expected_state:
                return last_status
            time.sleep(0.005)
        raise RuntimeError(
            f"Drive did not reach {label}: status=0x{last_status:04X} "
            f"({decode_drive_state(last_status)})"
        )

    def _transition_drive_to_operation_enabled(self, slave: Any) -> int:
        statusword = self._read_statusword_sdo(slave)
        if statusword & STATUS_BIT_FAULT:
            # Fault Reset is edge triggered. Clear the command first, then pulse bit 7.
            slave.sdo_write(CONTROL_WORD, 0x00, pack_u16(CONTROLWORD_DISABLE_VOLTAGE))
            slave.sdo_write(CONTROL_WORD, 0x00, pack_u16(0x0080))
            self._wait_for_drive_state(
                slave, 0x0040, "Switch On Disabled after Fault Reset"
            )

        slave.sdo_write(CONTROL_WORD, 0x00, pack_u16(CONTROLWORD_SHUTDOWN))
        self._wait_for_drive_state(slave, 0x0021, "Ready to Switch On")

        slave.sdo_write(CONTROL_WORD, 0x00, pack_u16(CONTROLWORD_SWITCH_ON))
        self._wait_for_drive_state(slave, 0x0023, "Switched On")

        slave.sdo_write(
            CONTROL_WORD, 0x00, pack_u16(CONTROLWORD_ENABLE_OPERATION)
        )
        return self._wait_for_drive_state(
            slave, STATUS_OPERATION_ENABLED, "Operation Enabled"
        )

    def _configure_slave_pdos(self, slave_position: int) -> None:
        slave = self.master.slaves[slave_position]

        # Disable assignments before changing mapping.
        slave.sdo_write(SM2_RXPDO_ASSIGNMENT, 0x00, pack_u8(0))
        slave.sdo_write(SM3_TXPDO_ASSIGNMENT, 0x00, pack_u8(0))

        # Explicitly define standard RxPDO2: Controlword + Target Position.
        slave.sdo_write(RXPDO2_MAPPING, 0x00, pack_u8(0))
        self._write_mapping_entry(slave, RXPDO2_MAPPING, 0x01, MAP_CONTROLWORD_U16)
        self._write_mapping_entry(
            slave, RXPDO2_MAPPING, 0x02, MAP_TARGET_POSITION_S32
        )
        slave.sdo_write(RXPDO2_MAPPING, 0x00, pack_u8(2))

        # Explicitly define standard TxPDO2: Statusword + Actual Position.
        slave.sdo_write(TXPDO2_MAPPING, 0x00, pack_u8(0))
        self._write_mapping_entry(slave, TXPDO2_MAPPING, 0x01, MAP_STATUSWORD_U16)
        self._write_mapping_entry(
            slave, TXPDO2_MAPPING, 0x02, MAP_ACTUAL_POSITION_S32
        )
        slave.sdo_write(TXPDO2_MAPPING, 0x00, pack_u8(2))

        # Assign only PDO2 in each process-data direction.
        slave.sdo_write(SM2_RXPDO_ASSIGNMENT, 0x01, pack_u16(RXPDO2_MAPPING))
        slave.sdo_write(SM2_RXPDO_ASSIGNMENT, 0x00, pack_u8(1))
        slave.sdo_write(SM3_TXPDO_ASSIGNMENT, 0x01, pack_u16(TXPDO2_MAPPING))
        slave.sdo_write(SM3_TXPDO_ASSIGNMENT, 0x00, pack_u8(1))

        # Drive-side cyclic timing and communication-loss behavior.
        slave.sdo_write(
            CYCLIC_INTERPOLATION_RATE,
            0x00,
            pack_u16(self.config.interpolation_rate_100us),
        )
        slave.sdo_write(
            ABORT_CONNECTION_OPTION,
            0x00,
            pack_s16(self.config.abort_connection_option_code),
        )
        slave.sdo_write(
            QUICK_STOP_OPTION,
            0x00,
            pack_s16(self.config.quick_stop_option_code),
        )
        slave.sdo_write(
            HALT_OPTION,
            0x00,
            pack_s16(self.config.halt_option_code),
        )

        minimum_motor_speed_rpm = self.config.minimum_motor_speed_rpm[slave_position]
        if minimum_motor_speed_rpm > 0:
            previous_motor_speed_rpm = unpack_u32(
                slave.sdo_read(MAXIMUM_MOTOR_SPEED, 0x00)
            )
            configured_motor_speed_rpm = max(
                previous_motor_speed_rpm, minimum_motor_speed_rpm
            )
            if configured_motor_speed_rpm != previous_motor_speed_rpm:
                slave.sdo_write(
                    MAXIMUM_MOTOR_SPEED,
                    0x00,
                    pack_u32(configured_motor_speed_rpm),
                )
            readback_motor_speed_rpm = unpack_u32(
                slave.sdo_read(MAXIMUM_MOTOR_SPEED, 0x00)
            )
            if readback_motor_speed_rpm != configured_motor_speed_rpm:
                raise RuntimeError(
                    f"{JOINT_NAMES[slave_position]} maximum motor-speed write failed: "
                    f"requested={configured_motor_speed_rpm}, "
                    f"readback={readback_motor_speed_rpm}"
                )
            print(
                f"{JOINT_NAMES[slave_position]} maximum motor speed: "
                f"previous={previous_motor_speed_rpm} rpm, "
                f"configured={readback_motor_speed_rpm} rpm",
                flush=True,
            )

        # Configure drive-side following-error supervision for every axis. Leaving
        # axes at device defaults previously made behavior depend on stale drive
        # settings rather than the repository configuration.
        window_rad = self.config.drive_following_error_window_rad[slave_position]
        timeout = self.config.drive_following_error_timeout[slave_position]
        window_units = max(
            1,
            int(
                round(
                    window_rad
                    * self.position_units_per_output_revolution[slave_position]
                    / (2.0 * math.pi)
                )
            ),
        )
        slave.sdo_write(FOLLOWING_ERROR_WINDOW, 0x00, pack_u32(window_units))
        slave.sdo_write(FOLLOWING_ERROR_TIMEOUT, 0x00, pack_u16(timeout))
        configured_window = unpack_u32(
            slave.sdo_read(FOLLOWING_ERROR_WINDOW, 0x00)
        )
        configured_timeout = unpack_u16(
            slave.sdo_read(FOLLOWING_ERROR_TIMEOUT, 0x00)
        )
        if configured_window != window_units or configured_timeout != timeout:
            raise RuntimeError(
                f"{JOINT_NAMES[slave_position]} following-error configuration "
                f"readback mismatch: window={configured_window}/{window_units}, "
                f"timeout={configured_timeout}/{timeout}"
            )
        print(
            f"{JOINT_NAMES[slave_position]} following-error monitoring: "
            f"window={configured_window} units ({window_rad:.3f} rad), "
            f"timeout={configured_timeout}",
            flush=True,
        )

        # Initialize the CSP set-point to the measured position before enabling.
        actual = unpack_s32(slave.sdo_read(ACTUAL_POSITION, 0x00))
        slave.sdo_write(TARGET_POSITION, 0x00, pack_s32(actual))
        slave.sdo_write(MODES_OF_OPERATION, 0x00, pack_s8(CSP_MODE))
        statusword = self._transition_drive_to_operation_enabled(slave)

        mode_deadline = time.monotonic() + 1.0
        mode_display = 0
        while time.monotonic() < mode_deadline:
            mode_display = unpack_s8(
                slave.sdo_read(MODES_OF_OPERATION_DISPLAY, 0x00)
            )
            if mode_display == CSP_MODE:
                break
            time.sleep(0.005)
        if mode_display != CSP_MODE:
            raise RuntimeError(
                f"Slave {slave_position} did not confirm CSP mode: {mode_display}"
            )
        if (statusword & STATUS_STATE_MASK) != STATUS_OPERATION_ENABLED:
            raise RuntimeError(
                f"Slave {slave_position} did not reach Operation Enabled: "
                f"0x{statusword:04X} ({decode_drive_state(statusword)})"
            )

    def _read_identity_string(self, slave: Any, index: int) -> str:
        try:
            raw = slave.sdo_read(index, 0x00)
        except Exception as exc:  # noqa: BLE001 - identity is diagnostic.
            return f"<read failed: {exc}>"
        return raw.split(b"\x00", 1)[0].decode("utf-8", errors="replace")

    def _connect_ethercat(self) -> None:
        if self.master_already_initialized:
            found = len(self.master.slaves)
            print(
                "Reusing the initialized EtherCAT master from startup homing; "
                "no communication gap is introduced before PDO mapping",
                flush=True,
            )
        else:
            print(f"Opening EtherCAT interface: {self.config.interface}", flush=True)
            self.master.open(self.config.interface)
            found = self.master.config_init()
        if found != EXPECTED_SLAVE_COUNT:
            raise RuntimeError(
                f"Expected exactly {EXPECTED_SLAVE_COUNT} slaves, found {found}"
            )

        self.slaves = list(self.master.slaves)
        for index, slave in enumerate(self.slaves):
            device_name = self._read_identity_string(slave, 0x1008)
            hardware_version = self._read_identity_string(slave, 0x1009)
            software_version = self._read_identity_string(slave, 0x100A)
            print(
                f"Slave {index}: name={slave.name!r}, "
                f"manufacturer=0x{int(slave.man):08X}, "
                f"product=0x{int(slave.id):08X}, revision=0x{int(slave.rev):08X}, "
                f"device={device_name!r}, hw={hardware_version!r}, "
                f"sw={software_version!r}",
                flush=True,
            )
            if int(slave.man) != FAULHABER_VENDOR_ID:
                raise RuntimeError(
                    f"Slave {index} is not a FAULHABER device: "
                    f"vendor=0x{int(slave.man):08X}"
                )
            self._configure_position_scaling(slave, index)
            slave.config_func = self._configure_slave_pdos

        io_size = self.master.config_map()
        print(f"Mapped EtherCAT process image: {io_size} bytes", flush=True)

        for index, slave in enumerate(self.slaves):
            if len(slave.output) != 6 or len(slave.input) != 6:
                raise RuntimeError(
                    f"Slave {index} PDO length mismatch: "
                    f"output={len(slave.output)}, input={len(slave.input)}, expected 6/6"
                )
        self.pdo_mapped = True

        dc_found = self.master.config_dc()
        print(f"Distributed clocks found: {bool(dc_found)}", flush=True)
        if self.config.use_distributed_clocks:
            if not dc_found:
                raise RuntimeError("Distributed Clocks requested but no DC slave was found")
            for slave in self.slaves:
                slave.dc_sync(
                    True,
                    self.config.cycle_time_ns,
                    self.config.dc_shift_ns,
                )

        # Read actual position once more, initialize every cyclic output, then
        # exchange valid data before requesting OP.
        for index, slave in enumerate(self.slaves):
            actual = unpack_s32(slave.sdo_read(ACTUAL_POSITION, 0x00))
            self.actual_counts[index] = actual
            self.actual_positions[index] = self._position_units_to_radians(actual, index)
            self.target_counts[index] = actual
            self.target_positions[index] = self.actual_positions[index]
            self.applied_target_positions[index] = self.actual_positions[index]
            slave.output = struct.pack(
                "<Hi", CONTROLWORD_ENABLE_OPERATION, actual
            )

        self.master.state_check(pysoem.SAFEOP_STATE, 50_000)
        self.master.send_processdata()
        self.master.receive_processdata(self.config.processdata_timeout_us)

        self.master.state = pysoem.OP_STATE
        self.master.write_state()

        reached_op = False
        for _ in range(200):
            self.master.send_processdata()
            self.master.receive_processdata(self.config.processdata_timeout_us)
            if self.master.state_check(pysoem.OP_STATE, 5_000) == pysoem.OP_STATE:
                reached_op = True
                break
            time.sleep(0.001)

        if not reached_op:
            self.master.read_state()
            states = [
                (
                    index,
                    int(slave.state),
                    int(slave.al_status),
                )
                for index, slave in enumerate(self.slaves)
            ]
            raise RuntimeError(f"EtherCAT slaves did not reach OP: {states}")

        self.expected_wkc = int(self.master.expected_wkc)
        if self.expected_wkc <= 0:
            raise RuntimeError(f"Invalid expected working counter: {self.expected_wkc}")
        print(
            f"EtherCAT OP, expected WKC={self.expected_wkc}, "
            f"PDO cycle={self.config.cycle_time_us} us, "
            f"0x2332={self.config.interpolation_rate_100us}",
            flush=True,
        )

    def _prime_pdo_exchange(self, cycles: int = 20) -> None:
        """Verify stable WKC and drive status before publishing readiness."""
        self.bridge_state = BRIDGE_STATE_HOLD
        for _ in range(max(1, int(cycles))):
            now = time.monotonic()
            self._update_outputs(now)
            self.master.send_processdata()
            wkc = int(
                self.master.receive_processdata(
                    self.config.processdata_timeout_us
                )
            )
            self._read_inputs_and_validate(wkc)
            if self.bridge_state == BRIDGE_STATE_FAULT:
                raise RuntimeError(
                    "PDO validation failed before readiness: "
                    f"WKC={wkc}/{self.expected_wkc}, "
                    f"errors=0x{self.error_flags:08X}, "
                    f"statuswords={[f'0x{word:04X}' for word in self.statuswords]}"
                )
            time.sleep(self.config.cycle_time_s)

        print(
            "Initial PDO hold exchange is stable: "
            f"WKC={self.expected_wkc}, "
            f"statuswords={[f'0x{word:04X}' for word in self.statuswords]}",
            flush=True,
        )

    def _drain_commands(self, now: float) -> None:
        if self.socket is None:
            return

        while True:
            try:
                data, address = self.socket.recvfrom(512)
            except BlockingIOError:
                break
            except OSError as exc:
                self.error_flags |= ERROR_INVALID_COMMAND
                print(f"IPC receive failed: {exc}", flush=True)
                break

            self.client_address = address
            if len(data) != COMMAND_STRUCT.size:
                self.error_flags |= ERROR_INVALID_COMMAND
                print(
                    f"Ignored IPC packet with size {len(data)}; "
                    f"expected {COMMAND_STRUCT.size}",
                    flush=True,
                )
                continue

            magic, version, flags, sequence, _timestamp_ns, *positions = (
                COMMAND_STRUCT.unpack(data)
            )
            if magic != COMMAND_MAGIC or version != IPC_VERSION:
                self.error_flags |= ERROR_INVALID_COMMAND
                print("Ignored IPC packet with invalid magic/version", flush=True)
                continue

            self.latest_command_sequence = int(sequence)

            if flags & COMMAND_FLAG_REGISTER:
                # Registration only. Position values are deliberately ignored.
                continue

            if flags & COMMAND_FLAG_HALT:
                self.halt_requested = True
                self.target_positions = list(self.actual_positions)
                self.target_counts = list(self.actual_counts)
                continue

            if not flags & COMMAND_FLAG_POSITION_VALID:
                continue

            candidate = [float(value) for value in positions]

            candidate, reason = self._normalize_initial_command(candidate)

            if reason is None:
                reason = self._validate_command(candidate, now)
                
            if reason is not None:
                self.error_flags |= ERROR_INVALID_COMMAND
                self.halt_requested = True
                print(f"Rejected CSP command: {reason}", flush=True)
                continue

            if not self.config.allow_motion:
                self.error_flags |= ERROR_MOTION_DISABLED
                self.halt_requested = True
                continue

            changed = any(
                abs(candidate[index] - self.last_received_command[index]) > 1e-9
                for index in range(EXPECTED_SLAVE_COUNT)
            )
            self.last_received_command = candidate
            self.last_valid_command_time = now
            if changed:
                self.last_changed_command_time = now

            self.target_positions = candidate
            self.target_counts = [
                self._radians_to_position_units(value, index)
                for index, value in enumerate(candidate)
            ]
            self.have_valid_command = True
            self.halt_requested = False
            self.error_flags &= ~(
                ERROR_COMMAND_WATCHDOG
                | ERROR_INVALID_COMMAND
                | ERROR_MOTION_DISABLED
            )

    def _validate_command(self, candidate: list[float], now: float) -> str | None:
        if len(candidate) != EXPECTED_SLAVE_COUNT:
            return "wrong joint count"
        if not all(math.isfinite(value) for value in candidate):
            return "non-finite position"

        for index, value in enumerate(candidate):
            if not (
                self.config.position_min_rad[index]
                <= value
                <= self.config.position_max_rad[index]
            ):
                return (
                    f"{JOINT_NAMES[index]}={value:.6f} rad outside "
                    f"[{self.config.position_min_rad[index]:.6f}, "
                    f"{self.config.position_max_rad[index]:.6f}]"
                )

        if not self.have_valid_command:
            for index, value in enumerate(candidate):
                difference = abs(value - self.actual_positions[index])
                if difference > self.config.first_command_tolerance_rad[index]:
                    return (
                        f"first command for {JOINT_NAMES[index]} differs from actual "
                        f"position by {difference:.6f} rad; tolerance is "
                        f"{self.config.first_command_tolerance_rad[index]:.6f} rad"
                    )
            return None

        # Validate against the target used in the previous PDO cycle, not against
        # packet-arrival timing. If several Unix datagrams arrive in one cycle,
        # this prevents the latest packet from creating a multi-sample position jump.
        del now
        for index, value in enumerate(candidate):
            position_step = abs(value - self.applied_target_positions[index])
            maximum_step = (
                self.config.max_velocity_rad_s[index]
                * self.config.cycle_time_s
                * 1.05
            )
            if position_step > maximum_step:
                equivalent_velocity = position_step / self.config.cycle_time_s
                return (
                    f"{JOINT_NAMES[index]} PDO step {position_step:.6f} rad "
                    f"({equivalent_velocity:.3f} rad/s equivalent) exceeds "
                    f"{maximum_step:.6f} rad per cycle"
                )
        return None

    def _update_outputs(self, now: float) -> None:
        watchdog_expired = (
            self.have_valid_command
            and self.last_valid_command_time is not None
            and (now - self.last_valid_command_time)
            > self.config.command_watchdog_ms / 1000.0
        )

        if watchdog_expired:
            self.error_flags |= ERROR_COMMAND_WATCHDOG
            self.halt_requested = True

        if self.bridge_state == BRIDGE_STATE_FAULT:
            controlword = CONTROLWORD_HALT
            commanded_counts = list(self.actual_counts)
        elif self.halt_requested or not self.config.allow_motion:
            controlword = CONTROLWORD_HALT
            commanded_counts = list(self.actual_counts)
            self.bridge_state = BRIDGE_STATE_HOLD
        else:
            controlword = CONTROLWORD_ENABLE_OPERATION
            commanded_counts = list(self.target_counts)
            self.applied_target_positions = list(self.target_positions)
            self.bridge_state = BRIDGE_STATE_READY

        for index, slave in enumerate(self.slaves):
            slave.output = struct.pack(
                "<Hi", controlword, int(commanded_counts[index])
            )

    def _read_inputs_and_validate(self, wkc: int) -> None:
        self.cycle_count += 1
        if wkc != self.expected_wkc:
            self.bad_wkc_cycles += 1
            self.error_flags |= ERROR_WKC
        else:
            self.bad_wkc_cycles = 0
            self.error_flags &= ~ERROR_WKC

        current_cycle_state_bad = False
        current_cycle_warning = False
        for index, slave in enumerate(self.slaves):
            data = bytes(slave.input)
            if len(data) != 6:
                self.error_flags |= ERROR_ETHERCAT_STATE
                current_cycle_state_bad = True
                continue

            statusword, actual_counts = struct.unpack("<Hi", data)
            self.statuswords[index] = int(statusword)
            self.actual_counts[index] = int(actual_counts)
            self.actual_positions[index] = self._position_units_to_radians(actual_counts, index)

            if statusword & (STATUS_BIT_FAULT | STATUS_BIT_FAULHABER_ERROR):
                self.error_flags |= ERROR_DRIVE_FAULT
                current_cycle_state_bad = True
            if statusword & STATUS_BIT_WARNING:
                current_cycle_warning = True
            if statusword & STATUS_BIT_INTERNAL_LIMIT:
                self.error_flags |= ERROR_INTERNAL_LIMIT
                current_cycle_state_bad = True
            if statusword & STATUS_BIT_FOLLOWING_ERROR:
                commanded_rad = self.applied_target_positions[index]
                actual_rad = self.actual_positions[index]
                error_rad = commanded_rad - actual_rad

                print(
                    "FOLLOWING ERROR: "
                    f"joint={JOINT_NAMES[index]}, "
                    f"command={commanded_rad:.6f} rad, "
                    f"actual={actual_rad:.6f} rad, "
                    f"error={error_rad:+.6f} rad, "
                    f"target_counts={self.target_counts[index]}, "
                    f"actual_counts={self.actual_counts[index]}, "
                    f"statusword=0x{statusword:04X}, "
                    f"cycle={self.cycle_count}",
                    flush=True,
                )

                self.error_flags |= ERROR_FOLLOWING_ERROR
                current_cycle_state_bad = True
            # Bit 12 may lag during the first PDO frames after switching to OP.
            # Enforce it after a short, deterministic startup grace interval.
            if (
                self.cycle_count > 20
                and self.config.allow_motion
                and self.have_valid_command
                and not self.halt_requested
                and not statusword & STATUS_BIT_FOLLOWS_COMMAND
            ):
                self.error_flags |= ERROR_NOT_FOLLOWING
                current_cycle_state_bad = True
            if (statusword & STATUS_STATE_MASK) != STATUS_OPERATION_ENABLED:
                self.error_flags |= ERROR_ETHERCAT_STATE
                current_cycle_state_bad = True

            if self.have_valid_command and not self.halt_requested:
                tracking_error = abs(
                    self.target_positions[index] - self.actual_positions[index]
                )
                if tracking_error > self.config.max_tracking_error_rad[index]:
                    self.error_flags |= ERROR_TRACKING
                    current_cycle_state_bad = True

        if current_cycle_warning:
            self.error_flags |= ERROR_WARNING
        else:
            self.error_flags &= ~ERROR_WARNING

        if current_cycle_state_bad:
            self.bad_state_cycles += 1
        else:
            self.bad_state_cycles = 0
            self.error_flags &= ~(
                ERROR_DRIVE_FAULT
                | ERROR_NOT_FOLLOWING
                | ERROR_FOLLOWING_ERROR
                | ERROR_INTERNAL_LIMIT
                | ERROR_ETHERCAT_STATE
                | ERROR_TRACKING
            )

        if self.bad_wkc_cycles >= self.config.wkc_fault_cycles:
            self.halt_requested = True
            self.bridge_state = BRIDGE_STATE_FAULT
        if self.bad_state_cycles >= self.config.state_fault_cycles:
            self.halt_requested = True
            self.bridge_state = BRIDGE_STATE_FAULT

    def _record_cycle_timing(self, remaining_s: float) -> None:
        self.last_cycle_lateness_us = max(0.0, -remaining_s * 1_000_000.0)
        if self.last_cycle_lateness_us > self.config.max_cycle_lateness_us:
            self.bad_timing_cycles += 1
        else:
            self.bad_timing_cycles = 0

        if self.bad_timing_cycles >= self.config.timing_fault_cycles:
            self.error_flags |= ERROR_TIMING
            self.halt_requested = True
            self.bridge_state = BRIDGE_STATE_FAULT
        else:
            self.error_flags &= ~ERROR_TIMING

    def _send_feedback(self, now_ns: int, wkc: int) -> None:
        if self.socket is None or self.client_address is None:
            return

        packet = FEEDBACK_STRUCT.pack(
            FEEDBACK_MAGIC,
            IPC_VERSION,
            self.latest_command_sequence,
            int(now_ns),
            int(self.bridge_state),
            int(wkc),
            *self.actual_positions,
            *self.statuswords,
            int(self.error_flags),
        )
        try:
            self.socket.sendto(packet, self.client_address)
        except (FileNotFoundError, ConnectionRefusedError):
            self.client_address = None
        except OSError as exc:
            # Feedback is latest-value telemetry. If the nonblocking client
            # receive queue is full, dropping this older datagram is safer than
            # blocking the EtherCAT cycle or flooding the terminal; the next
            # cycle will carry a newer sample.
            if exc.errno in {errno.EAGAIN, errno.EWOULDBLOCK, errno.ENOBUFS}:
                return
            print(f"IPC feedback send failed: {exc}", flush=True)

    def _periodic_log(self, now: float, wkc: int) -> None:
        if now - self._last_log_time < self.config.feedback_log_period_s:
            return
        self._last_log_time = now
        status = ", ".join(
            f"{JOINT_NAMES[index]}=0x{word:04X}/"
            f"{decode_drive_state(word)}"
            for index, word in enumerate(self.statuswords)
        )
        upper_index = JOINT_NAMES.index("upperarm_joint")
        lower_index = JOINT_NAMES.index("lowerarm_joint")
        gripper_index = JOINT_NAMES.index("end_effector_joint")

        lower_drive_target = self._model_to_drive_radians(
            self.target_positions[lower_index], lower_index
        )
        lower_drive_actual = self._model_to_drive_radians(
            self.actual_positions[lower_index], lower_index
        )
        # From the current URDF fixed rotations.  This is the lower-arm link's
        # estimated absolute elevation angle relative to base horizontal.
        lower_absolute_angle = (
            -math.pi / 4.0
            - self.actual_positions[upper_index]
            - self.actual_positions[lower_index]
        )

        print(
            f"PDO wkc={wkc}/{self.expected_wkc}, state={self.bridge_state}, "
            f"errors=0x{self.error_flags:08X}, motion={self.config.allow_motion}, "
            f"startup={self.config.startup_mode}, "
            f"lateness_us={self.last_cycle_lateness_us:.1f}; {status}; "
            f"lowerarm_model_target={self.target_positions[lower_index]:+.6f} rad, "
            f"lowerarm_model_actual={self.actual_positions[lower_index]:+.6f} rad, "
            f"lowerarm_drive_target={lower_drive_target:+.6f} rad, "
            f"lowerarm_drive_actual={lower_drive_actual:+.6f} rad, "
            f"lowerarm_abs_estimate={lower_absolute_angle:+.6f} rad; "
            f"gripper_model_target={self.target_positions[gripper_index]:+.6f} rad, "
            f"gripper_model_actual={self.actual_positions[gripper_index]:+.6f} rad, "
            f"gripper_target_counts={self.target_counts[gripper_index]}, "
            f"gripper_actual_counts={self.actual_counts[gripper_index]}",
            flush=True,
        )

    def run(self) -> None:
        self._remove_ready_file()
        gc_was_disabled = False
        try:
            self._configure_runtime_scheduling()
            self._open_ipc_socket()
            self._connect_ethercat()
            self._prime_pdo_exchange()
            self._running = True
            self.bridge_state = BRIDGE_STATE_HOLD
            self._create_ready_file()

            if not self.config.allow_motion:
                print(
                    "PDO motion is DISABLED by ethercat_pdo.yaml. The bridge will "
                    "exchange real PDO feedback and hold actual position only.",
                    flush=True,
                )

            gc.disable()
            gc_was_disabled = True
            next_deadline = time.monotonic()
            while self._running:
                next_deadline += self.config.cycle_time_s
                now = time.monotonic()

                self._drain_commands(now)
                self._update_outputs(now)
                self.master.send_processdata()
                wkc = self.master.receive_processdata(
                    self.config.processdata_timeout_us
                )
                self._read_inputs_and_validate(int(wkc))

                remaining = next_deadline - time.monotonic()
                self._record_cycle_timing(remaining)
                self._send_feedback(time.monotonic_ns(), int(wkc))
                self._periodic_log(now, int(wkc))

                if remaining > 0.0:
                    time.sleep(remaining)
                else:
                    # Do not accumulate missed deadlines indefinitely.
                    next_deadline = time.monotonic()
        except KeyboardInterrupt:
            print("PDO bridge interrupted", flush=True)
        finally:
            if gc_was_disabled:
                gc.enable()
            self.close()

    def close(self) -> None:
        self._running = False
        self.bridge_state = BRIDGE_STATE_STOPPED
        self._remove_ready_file()

        # Request a controlled halt before relinquishing EtherCAT ownership.
        try:
            if self.slaves and self.pdo_mapped:
                for _ in range(5):
                    for index, slave in enumerate(self.slaves):
                        slave.output = struct.pack(
                            "<Hi", CONTROLWORD_HALT, self.actual_counts[index]
                        )
                    self.master.send_processdata()
                    self.master.receive_processdata(
                        self.config.processdata_timeout_us
                    )
                    time.sleep(self.config.cycle_time_s)

                if self.config.use_distributed_clocks:
                    for slave in self.slaves:
                        slave.dc_sync(False, 0, 0)

                self.master.state = pysoem.SAFEOP_STATE
                self.master.write_state()
            elif self.slaves:
                # Startup failed before process data was mapped. SDO is still
                # available in Pre-Operational, so request the configured halt
                # instead of merely closing the master on enabled drives.
                for slave in self.slaves:
                    slave.sdo_write(
                        CONTROL_WORD, 0x00, pack_u16(CONTROLWORD_HALT)
                    )
                time.sleep(0.1)
        except Exception as exc:  # noqa: BLE001 - shutdown must continue.
            print(f"Controlled PDO shutdown encountered an error: {exc}", flush=True)

        try:
            self.master.close()
        except Exception:
            pass

        if self.socket is not None:
            try:
                self.socket.close()
            finally:
                self.socket = None
        try:
            os.unlink(self.config.command_socket)
        except FileNotFoundError:
            pass
        print("PDO CSP bridge closed", flush=True)

    def _normalize_initial_command(
        self,
        candidate: list[float],
    ) -> tuple[list[float], str | None]:
        """Clamp only the first feedback-synchronized command near a limit."""

        if self.have_valid_command:
            return candidate, None

        normalized = list(candidate)

        for index, value in enumerate(candidate):
            minimum = self.config.position_min_rad[index]
            maximum = self.config.position_max_rad[index]
            tolerance = self.config.initial_sync_limit_tolerance_rad[index]

            if value < minimum:
                excess = minimum - value

                if excess > tolerance:
                    return candidate, (
                        f"first command for {JOINT_NAMES[index]}={value:.6f} rad "
                        f"is below {minimum:.6f} rad by {excess:.6f} rad; "
                        f"initial tolerance is {tolerance:.6f} rad"
                    )

                normalized[index] = minimum
                print(
                    f"Clamped initial {JOINT_NAMES[index]} command "
                    f"from {value:.6f} to {minimum:.6f} rad",
                    flush=True,
                )

            elif value > maximum:
                excess = value - maximum

                if excess > tolerance:
                    return candidate, (
                        f"first command for {JOINT_NAMES[index]}={value:.6f} rad "
                        f"is above {maximum:.6f} rad by {excess:.6f} rad; "
                        f"initial tolerance is {tolerance:.6f} rad"
                    )

                normalized[index] = maximum
                print(
                    f"Clamped initial {JOINT_NAMES[index]} command "
                    f"from {value:.6f} to {maximum:.6f} rad",
                    flush=True,
                )

        return normalized, None


def main() -> None:
    config = load_pdo_config()
    bridge = PdoCspBridge(config)
    bridge.run()


if __name__ == "__main__":
    main()
