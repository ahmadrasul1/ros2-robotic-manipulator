#!/usr/bin/env python3

"""
@file pysoem_bridge.py
@brief EtherCAT communication bridge for the four-joint RASCL robot.

This bridge receives high-level commands through a temporary command file and
communicates with the FAULHABER motion controllers using PySOEM.

Supported commands:

@code
MOVE_RAD q1 q2 q3 q4      # Profile Position move, absolute joint radians
HOME_ALL                  # Run configured physical homing for all enabled joints
HOME_JOINT i              # Run configured physical homing for joint index i, 0-based
PARK_AT_A i               # Legacy diagnostic: move one A/B/C joint to A
PARK_AT_NEGATIVE_BOUNDARY i # Move to A or the negative software limit and hold
HOME_FROM_A_TO_B i         # Legacy diagnostic: move a parked joint from A to B
HOME_DIRECT_TO_B i          # Find B directly from either side and set B as zero
MOVE_PICK_READY           # Move to configured pick_ready_pose after homing
READ_INPUTS i             # Print logical/physical digital inputs for joint index i
@endcode

Homing configuration is read from config/homing.yaml. The finalized arm sequence
is dependency-aware: shoulder A-to-B, lower arm held at its negative clearance
boundary, upper arm C-to-B, then lower arm direct-to-B. The gripper is
intentionally excluded because it has no validated reference sensor.
"""

from __future__ import annotations

import math
import os
import struct
import time
from pathlib import Path
from typing import Any

import pysoem

try:
    import yaml
except ImportError:  # pragma: no cover - only relevant in incomplete containers
    yaml = None


INTERFACE_NAME = os.environ.get("RASCL_ETHERCAT_INTERFACE", "enx3c18a026482c")
COMMAND_FILE = os.environ.get("RASCL_COMMAND_FILE", "/tmp/rascl_robot_command.txt")
POLL_PERIOD = float(os.environ.get("RASCL_COMMAND_POLL_PERIOD", "0.01"))

EXPECTED_SLAVE_COUNT = 4
JOINT_NAMES = ["shoulder_joint", "upperarm_joint", "lowerarm_joint", "end_effector_joint"]

# CiA 402 object dictionary indices.
CONTROL_WORD = 0x6040
STATUS_WORD = 0x6041
MODES_OF_OPERATION = 0x6060
MODES_OF_OPERATION_DISPLAY = 0x6061
TARGET_POSITION = 0x607A
ACTUAL_POSITION = 0x6064
SOFTWARE_POSITION_LIMIT = 0x607D
PROFILE_VELOCITY = 0x6081
PROFILE_ACCELERATION = 0x6083
PROFILE_DECELERATION = 0x6084
POSITION_ENCODER_RESOLUTION = 0x608F
GEAR_RATIO = 0x6091
FEED_CONSTANT = 0x6092
POLARITY = 0x607E

# Digital input and limit switch objects.
LOWER_LIMIT_SWITCH_INPUTS = 0x2310  # subindex 0x01, bit mask DigIn1..DigIn8
UPPER_LIMIT_SWITCH_INPUTS = 0x2310  # subindex 0x02, bit mask DigIn1..DigIn8
LIMIT_SWITCH_OPTION_CODE = 0x2310   # subindex 0x03
DIGITAL_INPUT_STATUS = 0x2311       # subindex 0x01 logical, 0x02 physical

# Homing objects.
HOMING_METHOD = 0x6098
HOMING_OFFSET = 0x607C
HOMING_SPEED = 0x6099
HOMING_ACCELERATION = 0x609A
REFERENCE_SWITCH_INPUT = 0x2310  # subindex 0x04
DIGITAL_INPUT_POLARITY = 0x2310  # subindex 0x10
DIGITAL_INPUT_FILTER = 0x2310    # subindex 0x12
POSITIVE_TORQUE_LIMIT_HOMING = 0x2350
NEGATIVE_TORQUE_LIMIT_HOMING = 0x2351
DEVICE_STATUS_WORD = 0x2324       # subindex 0x01, detailed drive/limit status
LIMIT_CHECK_DELAY_TIME = 0x2324  # subindex 0x02

DEVICE_STATUS_POSITIVE_LIMIT_SWITCH = 1 << 6
DEVICE_STATUS_NEGATIVE_LIMIT_SWITCH = 1 << 7
DEVICE_STATUS_POSITIVE_SOFTWARE_LIMIT = 1 << 8
DEVICE_STATUS_NEGATIVE_SOFTWARE_LIMIT = 1 << 9

CONTROL_WORD_SHUTDOWN = 0x0006
CONTROL_WORD_SWITCH_ON = 0x0007
CONTROL_WORD_ENABLE_OPERATION = 0x000F
CONTROL_WORD_DISABLE_VOLTAGE = 0x0000
CONTROL_WORD_START_MOTION = 0x001F
CONTROL_WORD_START_MOTION_RELATIVE = CONTROL_WORD_START_MOTION | 0x0040
CONTROL_WORD_HALT = CONTROL_WORD_ENABLE_OPERATION | 0x0100
CONTROL_WORD_START_HOMING = CONTROL_WORD_ENABLE_OPERATION | 0x0010

PROFILE_POSITION_MODE = 1
HOMING_MODE = 6

EXPECTED_ENCODER_INCREMENTS_PER_MOTOR_REVOLUTION = [16384, 16384, 16384, 4096]
PHYSICAL_GEAR_RATIOS = [196.0, 196.0, 196.0, 323.0]


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


def find_homing_config_path() -> Path | None:
    """Return the first existing homing.yaml path."""
    candidates = []

    env_path = os.environ.get("RASCL_HOMING_CONFIG")
    if env_path:
        candidates.append(Path(env_path))

    candidates.extend(
        [
            Path("/root/ws/install/rascl_wp3_ss26_group11/share/rascl_wp3_ss26_group11/config/homing.yaml"),
            Path("/root/ws/src/rascl_wp3_ss26_group11/config/homing.yaml"),
            Path.cwd() / "src/rascl_wp3_ss26_group11/config/homing.yaml",
        ]
    )

    for path in candidates:
        if path.is_file():
            return path
    return None


def load_homing_config() -> dict[str, Any]:
    """Load config/homing.yaml. Missing config returns a disabled setup."""
    path = find_homing_config_path()
    if path is None:
        print("No homing.yaml found. Homing disabled.", flush=True)
        return {"homing": {"enabled": False}}

    if yaml is None:
        raise RuntimeError("PyYAML is not installed, but homing.yaml is required")

    with path.open("r", encoding="utf-8") as file:
        data = yaml.safe_load(file) or {}

    print(f"Loaded homing config: {path}", flush=True)
    return data


class RasclRobotController:
    """Low-level EtherCAT controller for the four-joint RASCL robot."""

    def __init__(self, interface: str, homing_config: dict[str, Any] | None = None):
        self.homing_config = homing_config or {"homing": {"enabled": False}}

        self.master = pysoem.Master()
        self.master.open(interface)

        slaves_found = self.master.config_init()
        if slaves_found != EXPECTED_SLAVE_COUNT:
            self.master.close()
            raise RuntimeError(
                f"Expected exactly {EXPECTED_SLAVE_COUNT} EtherCAT slaves, "
                f"found {slaves_found}"
            )

        print(f"{slaves_found} EtherCAT slave(s) found", flush=True)
        for index, slave in enumerate(self.master.slaves):
            print(f"Slave {index}: {slave.name}", flush=True)

        self.slaves = self.master.slaves[:EXPECTED_SLAVE_COUNT]
        self.position_units_per_output_revolution = [0.0] * EXPECTED_SLAVE_COUNT

        # SDO access uses the EtherCAT mailbox and is only valid once every slave
        # has reached PRE-OP. Give the mailbox communication time to settle before
        # reading the Factor Group.
        self.master.sdo_read_timeout = 2_000_000
        state = self.master.state_check(pysoem.PREOP_STATE, 2_000_000)
        self.master.read_state()
        if (state & 0x0F) != pysoem.PREOP_STATE:
            states = [
                f"slave {index}: state=0x{slave.state:02X}"
                for index, slave in enumerate(self.slaves)
            ]
            self.master.close()
            raise RuntimeError(
                "Not all EtherCAT slaves reached PRE-OP before SDO access: "
                + ", ".join(states)
            )

        time.sleep(2.0)
        for index, slave in enumerate(self.slaves):
            self._configure_position_scaling(slave, index)

    def _configure_position_scaling(self, slave, joint_index: int) -> None:
        """Read the FAULHABER Factor Group used by 0x607A and 0x6064."""
        encoder_increments = self.read_object_unsigned(
            slave, POSITION_ENCODER_RESOLUTION, 0x01
        )
        encoder_motor_revolutions = self.read_object_unsigned(
            slave, POSITION_ENCODER_RESOLUTION, 0x02
        )
        gear_motor_revolutions = self.read_object_unsigned(slave, GEAR_RATIO, 0x01)
        gear_output_revolutions = self.read_object_unsigned(slave, GEAR_RATIO, 0x02)
        feed_units = self.read_object_unsigned(slave, FEED_CONSTANT, 0x01)
        feed_output_revolutions = self.read_object_unsigned(slave, FEED_CONSTANT, 0x02)
        polarity = self.read_object_unsigned(slave, POLARITY, 0x00)

        divisors = {
            "encoder_motor_revolutions": encoder_motor_revolutions,
            "gear_motor_revolutions": gear_motor_revolutions,
            "gear_output_revolutions": gear_output_revolutions,
            "feed_units": feed_units,
            "feed_output_revolutions": feed_output_revolutions,
        }
        for name, value in divisors.items():
            if value <= 0:
                raise RuntimeError(
                    f"Invalid Factor Group value for {JOINT_NAMES[joint_index]}: "
                    f"{name}={value}"
                )

        encoder_resolution = (
            float(encoder_increments) / float(encoder_motor_revolutions)
        )
        expected_resolution = float(
            EXPECTED_ENCODER_INCREMENTS_PER_MOTOR_REVOLUTION[joint_index]
        )
        if not math.isclose(
            encoder_resolution, expected_resolution, rel_tol=0.0, abs_tol=0.5
        ):
            raise RuntimeError(
                f"{JOINT_NAMES[joint_index]} encoder resolution mismatch: "
                f"0x608F={encoder_increments}/{encoder_motor_revolutions} "
                f"({encoder_resolution:g}), expected {expected_resolution:g}"
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
                f"Invalid Factor Group scaling for {JOINT_NAMES[joint_index]}"
            )

        self.position_units_per_output_revolution[joint_index] = (
            units_per_output_revolution
        )
        print(
            f"{JOINT_NAMES[joint_index]} scaling: "
            f"0x608F={encoder_increments}/{encoder_motor_revolutions}, "
            f"0x6091={gear_motor_revolutions}/{gear_output_revolutions}, "
            f"0x6092={feed_units}/{feed_output_revolutions}, "
            f"0x607E=0x{polarity:02X}, "
            f"PDO_units/output_rev={units_per_output_revolution:g}",
            flush=True,
        )

    def set_control_word(self, slave, value: int) -> None:
        slave.sdo_write(CONTROL_WORD, 0x00, pack_u16(value))

    def get_status_word(self, slave) -> int:
        status = slave.sdo_read(STATUS_WORD, 0x00)
        return struct.unpack("<H", status)[0]

    def get_actual_position_counts(self, slave) -> int:
        data = slave.sdo_read(ACTUAL_POSITION, 0x00)
        return struct.unpack("<i", data)[0]

    def get_software_position_limits_counts(self, slave) -> tuple[int, int]:
        """Read CiA 402 minimum/maximum software-position limits."""
        minimum = struct.unpack(
            "<i", slave.sdo_read(SOFTWARE_POSITION_LIMIT, 0x01)
        )[0]
        maximum = struct.unpack(
            "<i", slave.sdo_read(SOFTWARE_POSITION_LIMIT, 0x02)
        )[0]
        return minimum, maximum

    def set_software_position_limit_counts(
        self,
        slave,
        *,
        minimum_counts: int | None = None,
        maximum_counts: int | None = None,
    ) -> tuple[int, int]:
        """Write selected software-position limits and verify the readback."""
        if minimum_counts is not None:
            slave.sdo_write(
                SOFTWARE_POSITION_LIMIT,
                0x01,
                pack_s32(minimum_counts),
            )
        if maximum_counts is not None:
            slave.sdo_write(
                SOFTWARE_POSITION_LIMIT,
                0x02,
                pack_s32(maximum_counts),
            )

        readback_minimum, readback_maximum = (
            self.get_software_position_limits_counts(slave)
        )
        if (
            minimum_counts is not None
            and readback_minimum != int(minimum_counts)
        ):
            raise RuntimeError(
                "Software-position minimum limit readback mismatch: "
                f"requested={minimum_counts}, readback={readback_minimum}"
            )
        if (
            maximum_counts is not None
            and readback_maximum != int(maximum_counts)
        ):
            raise RuntimeError(
                "Software-position maximum limit readback mismatch: "
                f"requested={maximum_counts}, readback={readback_maximum}"
            )
        return readback_minimum, readback_maximum

    def read_object_unsigned(
        self,
        slave,
        index: int,
        subindex: int,
        attempts: int = 3,
    ) -> int:
        """Read an unsigned SDO object with bounded WKC retries."""
        slave_index = next(
            (candidate_index for candidate_index, candidate in enumerate(self.slaves)
             if candidate is slave),
            -1,
        )

        for attempt in range(1, attempts + 1):
            try:
                data = slave.sdo_read(index, subindex)
                return int.from_bytes(data, byteorder="little", signed=False)
            except pysoem.WkcError as exc:
                self.master.read_state()
                print(
                    "SDO Working Counter failure: "
                    f"slave={slave_index}, name={slave.name!r}, "
                    f"state=0x{slave.state:02X}, "
                    f"object=0x{index:04X}:{subindex:02X}, "
                    f"attempt={attempt}/{attempts}, error={exc}",
                    flush=True,
                )
                if attempt == attempts:
                    raise RuntimeError(
                        "SDO communication failed after retries: "
                        f"slave={slave_index}, "
                        f"object=0x{index:04X}:{subindex:02X}"
                    ) from exc
                time.sleep(0.2)

        raise RuntimeError("Unreachable SDO read state")

    def read_digital_inputs(self, slave, physical: bool = True) -> int:
        """Return digital input bit mask. Bit 0 = DigIn1, bit 1 = DigIn2, etc."""
        subindex = 0x02 if physical else 0x01
        return self.read_object_unsigned(slave, DIGITAL_INPUT_STATUS, subindex)

    def print_digital_inputs(self, joint_index: int) -> None:
        """Diagnostic helper for identifying A/B/C switch wiring."""
        slave = self.slaves[joint_index]
        physical = self.read_digital_inputs(slave, physical=True)
        logical = self.read_digital_inputs(slave, physical=False)
        print(
            f"{JOINT_NAMES[joint_index]} inputs: "
            f"physical 0x2311.02 = 0b{physical:08b} (0x{physical:02X}), "
            f"logical 0x2311.01 = 0b{logical:08b} (0x{logical:02X})",
            flush=True,
        )

    def is_input_active(self, slave, input_number: int, active_low: bool = False) -> bool:
        """Return whether one physical digital input is active."""
        if input_number is None:
            raise ValueError("input_number must not be None")
        if input_number < 1 or input_number > 8:
            raise ValueError(f"Digital input must be in range 1..8, got {input_number}")
        state = self.read_digital_inputs(slave, physical=True)
        raw_high = bool(state & (1 << (input_number - 1)))
        return (not raw_high) if active_low else raw_high

    def set_operation_mode(self, slave, mode: int) -> None:
        slave.sdo_write(MODES_OF_OPERATION, 0x00, pack_s8(mode))
        time.sleep(0.1)

        data = slave.sdo_read(MODES_OF_OPERATION_DISPLAY, 0x00)
        mode_display = struct.unpack("<b", data)[0]

        if mode_display != mode:
            raise RuntimeError(
                f"Operation mode was not confirmed. Requested {mode}, got {mode_display}"
            )

    def enable_drive(self, slave, joint_index: int, mode: int = PROFILE_POSITION_MODE) -> None:
        """Select the mode and synchronize its target before enabling motion.

        Enabling Operation Enabled before selecting Profile Position mode can
        reactivate a stale target retained by the drive. Configure the mode while
        the drive is disabled and seed position modes from 0x6064 first.
        """
        self.set_control_word(slave, CONTROL_WORD_SHUTDOWN)
        time.sleep(0.2)

        self.set_operation_mode(slave, mode)
        if mode == PROFILE_POSITION_MODE:
            actual_counts = self.get_actual_position_counts(slave)
            slave.sdo_write(TARGET_POSITION, 0x00, pack_s32(actual_counts))
        time.sleep(0.1)

        self.set_control_word(slave, CONTROL_WORD_SWITCH_ON)
        time.sleep(0.2)

        self.set_control_word(slave, CONTROL_WORD_ENABLE_OPERATION)
        time.sleep(0.2)

        status = self.get_status_word(slave)
        print(
            f"Joint {joint_index + 1} enabled in mode {mode}. Status: 0x{status:04X}",
            flush=True,
        )

    def enable_all_drives(self, mode: int = PROFILE_POSITION_MODE) -> None:
        for index, slave in enumerate(self.slaves):
            self.enable_drive(slave, index, mode)

    def disable_all_drives(self) -> None:
        for index, slave in enumerate(self.slaves):
            try:
                self.set_control_word(slave, CONTROL_WORD_DISABLE_VOLTAGE)
                print(f"Joint {index + 1} voltage disabled", flush=True)
            except Exception as exc:
                print(f"Could not disable joint {index + 1}: {exc}", flush=True)

    def radians_to_counts(self, radians: float, joint_index: int) -> int:
        units_per_output_revolution = (
            self.position_units_per_output_revolution[joint_index]
        )
        value = round(
            float(radians) / (2.0 * math.pi) * units_per_output_revolution
        )
        if not -(2**31) <= value <= 2**31 - 1:
            raise OverflowError(
                f"Position for {JOINT_NAMES[joint_index]} does not fit in S32: {value}"
            )
        return int(value)

    def counts_to_radians(self, counts: int, joint_index: int) -> float:
        units_per_output_revolution = (
            self.position_units_per_output_revolution[joint_index]
        )
        return float(counts) / units_per_output_revolution * 2.0 * math.pi

    def write_target_radians(self, slave, radians: float, joint_index: int) -> None:
        target_position = self.radians_to_counts(radians, joint_index)
        slave.sdo_write(TARGET_POSITION, 0x00, pack_s32(target_position))
        print(
            f"Joint {joint_index + 1}: {radians:.4f} rad -> {target_position} counts",
            flush=True,
        )

    def get_device_status_word(self, slave) -> int:
        """Read FAULHABER device statusword 0x2324.01."""
        return self.read_object_unsigned(slave, DEVICE_STATUS_WORD, 0x01)

    def _check_profile_position_status(
        self,
        status: int,
        joint_index: int,
        context: str,
        *,
        allow_internal_limit_escape: bool = False,
    ) -> None:
        """Raise for unsafe PP states, except a verified move away from a limit."""
        if status & (1 << 3):
            raise RuntimeError(
                f"{JOINT_NAMES[joint_index]}: drive fault during {context}, "
                f"status=0x{status:04X}"
            )
        if status & (1 << 11) and not allow_internal_limit_escape:
            raise RuntimeError(
                f"{JOINT_NAMES[joint_index]}: internal limit active during {context}, "
                f"status=0x{status:04X}"
            )
        if status & (1 << 13):
            raise RuntimeError(
                f"{JOINT_NAMES[joint_index]}: following error during {context}, "
                f"status=0x{status:04X}"
            )

    def _verified_limit_escape(
        self,
        slave,
        joint_index: int,
        initial_counts: int,
        target_counts: int,
    ) -> bool:
        """Permit only a command that moves away from the active limit side.

        The limit mapping remains enabled. This does not suppress the drive's limit
        protection; it only prevents the bridge from rejecting the command before
        the drive has a chance to leave an already-active limit switch/range.
        """
        status = self.get_status_word(slave)
        if not status & (1 << 11):
            return False

        device_status = self.get_device_status_word(slave)
        positive_active = bool(
            device_status
            & (
                DEVICE_STATUS_POSITIVE_LIMIT_SWITCH
                | DEVICE_STATUS_POSITIVE_SOFTWARE_LIMIT
            )
        )
        negative_active = bool(
            device_status
            & (
                DEVICE_STATUS_NEGATIVE_LIMIT_SWITCH
                | DEVICE_STATUS_NEGATIVE_SOFTWARE_LIMIT
            )
        )
        delta_counts = target_counts - initial_counts

        escaping_negative = negative_active and not positive_active and delta_counts > 0
        escaping_positive = positive_active and not negative_active and delta_counts < 0
        allowed = escaping_negative or escaping_positive

        if allowed:
            side = "negative" if escaping_negative else "positive"
            print(
                f"{JOINT_NAMES[joint_index]} starts on the {side} limit; "
                f"allowing only the verified move away from it "
                f"(0x2324.01=0x{device_status:08X})",
                flush=True,
            )
        else:
            print(
                f"{JOINT_NAMES[joint_index]} limit escape rejected: "
                f"initial={initial_counts}, target={target_counts}, "
                f"0x2324.01=0x{device_status:08X}",
                flush=True,
            )

        return allowed

    def _start_profile_position_motion(
        self,
        slave,
        joint_index: int,
        *,
        relative: bool,
        acknowledge_timeout_s: float = 2.0,
        allow_internal_limit_escape: bool = False,
    ) -> None:
        """Start one PP command using the CiA 402 bit-4/bit-12 handshake."""
        start_controlword = (
            CONTROL_WORD_START_MOTION_RELATIVE
            if relative
            else CONTROL_WORD_START_MOTION
        )

        # Bit 4 must be low before the new rising edge. Wait until the previous
        # set-point acknowledgement has also returned low.
        self.set_control_word(slave, CONTROL_WORD_ENABLE_OPERATION)
        deadline = time.monotonic() + acknowledge_timeout_s
        while True:
            status = self.get_status_word(slave)
            self._check_profile_position_status(
                status,
                joint_index,
                "waiting for previous acknowledgement to clear",
                allow_internal_limit_escape=allow_internal_limit_escape,
            )
            if not status & (1 << 12):
                break
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"{JOINT_NAMES[joint_index]}: previous set-point "
                    f"acknowledgement did not clear, status=0x{status:04X}"
                )
            time.sleep(0.01)

        # The rising edge on bit 4 loads the target and profile parameters.
        self.set_control_word(slave, start_controlword)
        deadline = time.monotonic() + acknowledge_timeout_s
        while True:
            status = self.get_status_word(slave)
            self._check_profile_position_status(
                status,
                joint_index,
                "waiting for new set-point acknowledgement",
                allow_internal_limit_escape=allow_internal_limit_escape,
            )
            if status & (1 << 12):
                break
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"{JOINT_NAMES[joint_index]}: new target was not acknowledged, "
                    f"status=0x{status:04X}"
                )
            time.sleep(0.01)

        # FAULHABER specifies that bit 4 may only be reset after bit 12 is set.
        self.set_control_word(slave, CONTROL_WORD_ENABLE_OPERATION)
        deadline = time.monotonic() + acknowledge_timeout_s
        while True:
            status = self.get_status_word(slave)
            self._check_profile_position_status(
                status,
                joint_index,
                "waiting for acknowledgement reset",
                allow_internal_limit_escape=allow_internal_limit_escape,
            )
            if not status & (1 << 12):
                break
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"{JOINT_NAMES[joint_index]}: set-point acknowledgement "
                    f"did not reset, status=0x{status:04X}"
                )
            time.sleep(0.01)

        print(
            f"{JOINT_NAMES[joint_index]} Profile Position target acknowledged",
            flush=True,
        )

    def start_motion(
        self,
        slave,
        joint_index: int,
        acknowledge_timeout_s: float = 2.0,
        allow_internal_limit_escape: bool = False,
    ) -> None:
        self._start_profile_position_motion(
            slave,
            joint_index,
            relative=False,
            acknowledge_timeout_s=acknowledge_timeout_s,
            allow_internal_limit_escape=allow_internal_limit_escape,
        )

    def start_relative_motion(
        self,
        slave,
        joint_index: int,
        acknowledge_timeout_s: float = 2.0,
        allow_internal_limit_escape: bool = False,
    ) -> None:
        """Start a PP relative move via controlword bits 4 and 6."""
        self._start_profile_position_motion(
            slave,
            joint_index,
            relative=True,
            acknowledge_timeout_s=acknowledge_timeout_s,
            allow_internal_limit_escape=allow_internal_limit_escape,
        )

    def soft_halt_joint(self, slave) -> None:
        """Request a controlled halt without disabling the whole bridge."""
        self.set_control_word(slave, CONTROL_WORD_HALT)
        time.sleep(0.1)
        self.set_control_word(slave, CONTROL_WORD_ENABLE_OPERATION)
        time.sleep(0.05)

    def configure_profile_position_motion(self, slave, velocity: int, acceleration: int, deceleration: int | None = None) -> None:
        """Set conservative Profile Position parameters for pre-homing moves."""
        if deceleration is None:
            deceleration = acceleration
        slave.sdo_write(PROFILE_VELOCITY, 0x00, pack_u32(int(velocity)))
        slave.sdo_write(PROFILE_ACCELERATION, 0x00, pack_u32(int(acceleration)))
        slave.sdo_write(PROFILE_DECELERATION, 0x00, pack_u32(int(deceleration)))

    def wait_for_profile_target_reached(
        self,
        slave,
        joint_index: int,
        target_counts: int,
        timeout_s: float,
        tolerance_rad: float = 0.01,
        allow_internal_limit_escape: bool = False,
        limit_release_timeout_s: float = 3.0,
    ) -> None:
        """Require Target Reached, real position agreement, and limit release."""
        start_time = time.monotonic()
        limit_was_active = allow_internal_limit_escape
        next_log_time = start_time
        tolerance_counts = max(
            1, abs(self.radians_to_counts(tolerance_rad, joint_index))
        )

        while True:
            status = self.get_status_word(slave)
            internal_limit_active = bool(status & (1 << 11))
            self._check_profile_position_status(
                status,
                joint_index,
                "Profile Position movement",
                allow_internal_limit_escape=(
                    allow_internal_limit_escape and internal_limit_active
                ),
            )
            actual_counts = self.get_actual_position_counts(slave)
            error_counts = abs(target_counts - actual_counts)
            target_reached = bool(status & (1 << 10))

            if (
                target_reached
                and error_counts <= tolerance_counts
                and not internal_limit_active
            ):
                final_rad = self.counts_to_radians(actual_counts, joint_index)
                target_rad = self.counts_to_radians(target_counts, joint_index)
                print(
                    f"{JOINT_NAMES[joint_index]} reached target: "
                    f"target={target_rad:.4f} rad, actual={final_rad:.4f} rad, "
                    f"status=0x{status:04X}",
                    flush=True,
                )
                return

            now = time.monotonic()
            if allow_internal_limit_escape and internal_limit_active:
                if now - start_time > limit_release_timeout_s:
                    self.soft_halt_joint(slave)
                    device_status = self.get_device_status_word(slave)
                    raise RuntimeError(
                        f"{JOINT_NAMES[joint_index]} did not leave the active limit "
                        f"within {limit_release_timeout_s:.1f} s; "
                        f"actual={self.counts_to_radians(actual_counts, joint_index):.4f} rad, "
                        f"0x2324.01=0x{device_status:08X}"
                    )
            elif limit_was_active:
                print(
                    f"{JOINT_NAMES[joint_index]} active limit released; "
                    "continuing to pick-ready",
                    flush=True,
                )
                limit_was_active = False
                allow_internal_limit_escape = False

            if now >= next_log_time:
                actual_rad = self.counts_to_radians(actual_counts, joint_index)
                target_rad = self.counts_to_radians(target_counts, joint_index)
                print(
                    f"Waiting for {JOINT_NAMES[joint_index]}: "
                    f"target={target_rad:.4f} rad, actual={actual_rad:.4f} rad, "
                    f"status=0x{status:04X}",
                    flush=True,
                )
                next_log_time = now + 1.0

            if now - start_time > timeout_s:
                actual_rad = self.counts_to_radians(actual_counts, joint_index)
                target_rad = self.counts_to_radians(target_counts, joint_index)
                raise TimeoutError(
                    f"{JOINT_NAMES[joint_index]} did not reach target: "
                    f"target={target_rad:.4f} rad, actual={actual_rad:.4f} rad, "
                    f"status=0x{status:04X}"
                )
            time.sleep(0.05)

    def move_joint_relative_radians(
        self,
        joint_index: int,
        delta_rad: float,
        velocity: int,
        acceleration: int,
        timeout_s: float,
        wait_target: bool = True,
    ) -> None:
        """Move one joint by a relative amount in Profile Position mode.

        If the joint begins on a configured limit switch, only a command that is
        verified to move away from that active limit is permitted.
        """
        slave = self.slaves[joint_index]
        self.enable_drive(slave, joint_index, PROFILE_POSITION_MODE)
        self.configure_profile_position_motion(slave, velocity, acceleration)
        initial_counts = self.get_actual_position_counts(slave)
        delta_counts = self.radians_to_counts(delta_rad, joint_index)
        expected_target_counts = initial_counts + delta_counts

        initial_status = self.get_status_word(slave)
        allow_internal_limit_escape = False
        if initial_status & (1 << 11):
            allow_internal_limit_escape = self._verified_limit_escape(
                slave,
                joint_index,
                initial_counts,
                expected_target_counts,
            )
            if not allow_internal_limit_escape:
                self._check_profile_position_status(
                    initial_status,
                    joint_index,
                    "relative Profile Position command",
                )

        slave.sdo_write(TARGET_POSITION, 0x00, pack_s32(delta_counts))
        print(
            f"Relative move {JOINT_NAMES[joint_index]}: "
            f"{delta_rad:.4f} rad -> {delta_counts} counts",
            flush=True,
        )
        self.start_relative_motion(
            slave,
            joint_index,
            allow_internal_limit_escape=allow_internal_limit_escape,
        )
        if wait_target:
            self.wait_for_profile_target_reached(
                slave,
                joint_index,
                expected_target_counts,
                timeout_s,
                allow_internal_limit_escape=allow_internal_limit_escape,
            )

    def input_mask(self, input_number: int | None) -> int:
        if input_number is None:
            return 0
        if int(input_number) < 1 or int(input_number) > 8:
            raise ValueError(f"Digital input must be in range 1..8, got {input_number}")
        return 1 << (int(input_number) - 1)

    def configure_input_polarity_and_filters(self, slave, switch_specs: list[tuple[int | None, bool, bool]]) -> None:
        """Write combined polarity/filter masks for all switches used by this slave."""
        polarity_mask = 0
        filter_mask = 0
        for input_number, active_low, filter_enabled in switch_specs:
            if input_number is None:
                continue
            bit = self.input_mask(int(input_number))
            if active_low:
                polarity_mask |= bit
            if filter_enabled:
                filter_mask |= bit
        slave.sdo_write(DIGITAL_INPUT_POLARITY, 0x10, pack_u8(polarity_mask))
        slave.sdo_write(DIGITAL_INPUT_FILTER, 0x12, pack_u8(filter_mask))
        print(
            f"Digital input masks: polarity=0b{polarity_mask:08b}, filter=0b{filter_mask:08b}",
            flush=True,
        )

    def configure_limit_switches_for_joint(self, slave, cfg: dict[str, Any]) -> None:
        """Configure A/C as lower/upper limit switches for the current slave."""
        negative_input = cfg.get("negative_limit_switch_input")
        positive_input = cfg.get("positive_limit_switch_input")
        lower_mask = self.input_mask(negative_input)
        upper_mask = self.input_mask(positive_input)
        slave.sdo_write(LOWER_LIMIT_SWITCH_INPUTS, 0x01, pack_u8(lower_mask))
        slave.sdo_write(UPPER_LIMIT_SWITCH_INPUTS, 0x02, pack_u8(upper_mask))
        slave.sdo_write(LIMIT_SWITCH_OPTION_CODE, 0x03, pack_s16(int(cfg.get("limit_switch_option_code", 1))))
        print(
            f"Limit switches configured: lower(A)=0b{lower_mask:08b}, upper(C)=0b{upper_mask:08b}",
            flush=True,
        )

    def wait_for_input_active(
        self,
        slave,
        joint_index: int,
        input_number: int,
        active_low: bool,
        timeout_s: float,
        label: str,
    ) -> None:
        """Wait until a selected physical input becomes active."""
        start_time = time.monotonic()
        last_log = 0.0
        while True:
            if self.is_input_active(slave, int(input_number), active_low=bool(active_low)):
                state = self.read_digital_inputs(slave, physical=True)
                print(
                    f"{label} detected on {JOINT_NAMES[joint_index]}: physical inputs=0b{state:08b}",
                    flush=True,
                )
                return

            elapsed = time.monotonic() - start_time
            if elapsed - last_log > 1.0:
                state = self.read_digital_inputs(slave, physical=True)
                print(
                    f"Waiting for {label} on {JOINT_NAMES[joint_index]}... physical inputs=0b{state:08b}",
                    flush=True,
                )
                last_log = elapsed

            if elapsed > timeout_s:
                raise TimeoutError(
                    f"Timeout waiting for {label} on {JOINT_NAMES[joint_index]}"
                )
            time.sleep(0.05)

    def wait_for_input_inactive(
        self,
        slave,
        joint_index: int,
        input_number: int,
        active_low: bool,
        timeout_s: float,
        label: str,
    ) -> None:
        start_time = time.monotonic()
        while True:
            if not self.is_input_active(slave, int(input_number), active_low=bool(active_low)):
                return
            if time.monotonic() - start_time > timeout_s:
                raise TimeoutError(
                    f"Timeout waiting for {label} to clear on {JOINT_NAMES[joint_index]}"
                )
            time.sleep(0.05)

    def _abc_homing_parameters(
        self,
        joint_index: int,
        cfg: dict[str, Any],
    ) -> dict[str, Any]:
        """Validate and return the shared A/B/C homing parameters."""
        negative_input = cfg.get("negative_limit_switch_input")
        reference_input = cfg.get("reference_switch_input")
        positive_input = cfg.get("positive_limit_switch_input")
        if negative_input is None or reference_input is None or positive_input is None:
            raise RuntimeError(
                f"A/B/C switch inputs are required for {JOINT_NAMES[joint_index]}: "
                "negative_limit_switch_input, reference_switch_input, "
                "positive_limit_switch_input"
            )

        search_distance_rad = float(cfg.get("prehome_search_distance_rad", -6.0))
        reference_search_distance_rad = float(
            cfg.get("reference_search_distance_rad", 2.2)
        )
        if abs(search_distance_rad) < 1e-9:
            raise RuntimeError("prehome_search_distance_rad must not be zero")
        if abs(reference_search_distance_rad) < 1e-9:
            raise RuntimeError("reference_search_distance_rad must not be zero")

        return {
            "negative_input": int(negative_input),
            "reference_input": int(reference_input),
            "positive_input": int(positive_input),
            "negative_active_low": bool(
                cfg.get("negative_limit_active_low", cfg.get("active_low", False))
            ),
            "reference_active_low": bool(
                cfg.get("reference_switch_active_low", cfg.get("active_low", False))
            ),
            "positive_active_low": bool(
                cfg.get("positive_limit_active_low", cfg.get("active_low", False))
            ),
            "filter_enabled": bool(cfg.get("filter_enabled", True)),
            "timeout_s": float(
                cfg.get("timeout_s", self._homing_section().get("timeout_s", 45.0))
            ),
            "prehome_velocity": int(
                cfg.get("prehome_velocity", cfg.get("seek_velocity", 50))
            ),
            "prehome_acceleration": int(
                cfg.get("prehome_acceleration", cfg.get("acceleration", 20))
            ),
            "search_distance_rad": search_distance_rad,
            "backoff_distance_rad": float(
                cfg.get("reference_backoff_rad", cfg.get("prehome_backoff_rad", 0.10))
            ),
            "reference_search_distance_rad": reference_search_distance_rad,
            "reference_search_velocity": int(
                cfg.get("reference_search_velocity", cfg.get("homing_velocity", 30))
            ),
            "reference_search_acceleration": int(
                cfg.get("reference_search_acceleration", cfg.get("acceleration", 20))
            ),
        }

    def _configure_abc_switches(
        self,
        slave,
        cfg: dict[str, Any],
        params: dict[str, Any],
    ) -> None:
        self.configure_input_polarity_and_filters(
            slave,
            [
                (
                    params["negative_input"],
                    params["negative_active_low"],
                    params["filter_enabled"],
                ),
                (
                    params["reference_input"],
                    params["reference_active_low"],
                    params["filter_enabled"],
                ),
                (
                    params["positive_input"],
                    params["positive_active_low"],
                    params["filter_enabled"],
                ),
            ],
        )
        self.configure_limit_switches_for_joint(slave, cfg)

    def halt_joint_at_current_position(
        self,
        slave,
        joint_index: int,
        reason: str,
    ) -> None:
        """Set the CiA 402 halt bit and leave it asserted.

        The joint remains operation-enabled but is not allowed to resume its old
        Profile Position target. A later enable_drive() call clears the halt when
        the next controlled movement starts.
        """
        self.set_control_word(slave, CONTROL_WORD_HALT)
        time.sleep(0.15)
        status = self.get_status_word(slave)
        if status & (1 << 3):
            raise RuntimeError(
                f"{JOINT_NAMES[joint_index]} fault while halting at {reason}: "
                f"status=0x{status:04X}"
            )
        actual_counts = self.get_actual_position_counts(slave)
        actual_rad = self.counts_to_radians(actual_counts, joint_index)
        print(
            f"{JOINT_NAMES[joint_index]} halted at {reason}: "
            f"actual={actual_counts} counts ({actual_rad:.6f} rad), "
            f"status=0x{status:04X}",
            flush=True,
        )

    def wait_for_input_active_with_abort(
        self,
        slave,
        joint_index: int,
        wanted_input: int,
        wanted_active_low: bool,
        wanted_label: str,
        abort_input: int,
        abort_active_low: bool,
        abort_label: str,
        timeout_s: float,
    ) -> None:
        """Wait for one switch while aborting if the opposite switch is reached."""
        start_time = time.monotonic()
        next_log_time = start_time

        while True:
            if self.is_input_active(
                slave, wanted_input, active_low=wanted_active_low
            ):
                state = self.read_digital_inputs(slave, physical=True)
                print(
                    f"{wanted_label} detected on {JOINT_NAMES[joint_index]}: "
                    f"physical inputs=0b{state:08b}",
                    flush=True,
                )
                return

            if self.is_input_active(
                slave, abort_input, active_low=abort_active_low
            ):
                self.halt_joint_at_current_position(
                    slave, joint_index, abort_label
                )
                state = self.read_digital_inputs(slave, physical=True)
                raise RuntimeError(
                    f"{abort_label} became active before {wanted_label} on "
                    f"{JOINT_NAMES[joint_index]}; physical inputs=0b{state:08b}"
                )

            status = self.get_status_word(slave)
            device_status = self.get_device_status_word(slave)
            now = time.monotonic()

            if status & (1 << 11):
                physical = self.read_digital_inputs(
                    slave,
                    physical=True,
                )
                actual_counts = self.get_actual_position_counts(slave)
                actual_rad = self.counts_to_radians(
                    actual_counts,
                    joint_index,
                )

                causes = []
                if device_status & DEVICE_STATUS_POSITIVE_LIMIT_SWITCH:
                    causes.append("positive physical limit")
                if device_status & DEVICE_STATUS_NEGATIVE_LIMIT_SWITCH:
                    causes.append("negative physical limit")
                if device_status & DEVICE_STATUS_POSITIVE_SOFTWARE_LIMIT:
                    causes.append("positive software limit")
                if device_status & DEVICE_STATUS_NEGATIVE_SOFTWARE_LIMIT:
                    causes.append("negative software limit")

                cause_text = ", ".join(causes) or "undecoded internal limit"

                self.halt_joint_at_current_position(
                    slave,
                    joint_index,
                    "internal limit before sensor",
                )

                raise RuntimeError(
                    f"{JOINT_NAMES[joint_index]} stopped before "
                    f"{wanted_label}: {cause_text}; "
                    f"actual={actual_rad:.6f} rad, "
                    f"inputs=0b{physical:08b}, "
                    f"status=0x{status:04X}, "
                    f"0x2324.01=0x{device_status:08X}"
                )

            if (
                now - start_time > 0.5
                and status & (1 << 10)
            ):
                physical = self.read_digital_inputs(
                    slave,
                    physical=True,
                )
                actual_counts = self.get_actual_position_counts(slave)
                actual_rad = self.counts_to_radians(
                    actual_counts,
                    joint_index,
                )

                self.halt_joint_at_current_position(
                    slave,
                    joint_index,
                    "Profile Position target reached before sensor",
                )

                raise RuntimeError(
                    f"{JOINT_NAMES[joint_index]} completed its "
                    f"Profile Position command before reaching "
                    f"{wanted_label}; actual={actual_rad:.6f} rad, "
                    f"inputs=0b{physical:08b}, "
                    f"status=0x{status:04X}"
                )

            if status & (1 << 3):
                self.halt_joint_at_current_position(
                    slave, joint_index, "drive fault"
                )
                raise RuntimeError(
                    f"Drive fault while searching for {wanted_label} on "
                    f"{JOINT_NAMES[joint_index]}: status=0x{status:04X}"
                )
            if status & (1 << 13):
                self.halt_joint_at_current_position(
                    slave, joint_index, "following error"
                )
                raise RuntimeError(
                    f"Following error while searching for {wanted_label} on "
                    f"{JOINT_NAMES[joint_index]}: status=0x{status:04X}"
                )

            now = time.monotonic()
            if now >= next_log_time:
                state = self.read_digital_inputs(slave, physical=True)
                actual_counts = self.get_actual_position_counts(slave)
                actual_rad = self.counts_to_radians(actual_counts, joint_index)
                print(
                    f"Searching for {wanted_label} on {JOINT_NAMES[joint_index]}: "
                    f"actual={actual_rad:.4f} rad, inputs=0b{state:08b}, "
                    f"status=0x{status:04X}",
                    flush=True,
                )
                next_log_time = now + 1.0

            if now - start_time > timeout_s:
                self.halt_joint_at_current_position(
                    slave, joint_index, f"timeout searching for {wanted_label}"
                )
                raise TimeoutError(
                    f"Timeout searching for {wanted_label} on "
                    f"{JOINT_NAMES[joint_index]}"
                )

            time.sleep(0.02)

    def set_current_position_as_home(
        self,
        joint_index: int,
        cfg: dict[str, Any],
    ) -> None:
        """Use homing method 37 after B has been found in software.

        Method 37 performs no reference movement; it zeroes the position counter at
        the current location. The caller must therefore verify that B is active.
        """
        slave = self.slaves[joint_index]
        params = self._abc_homing_parameters(joint_index, cfg)
        if not self.is_input_active(
            slave,
            params["reference_input"],
            active_low=params["reference_active_low"],
        ):
            raise RuntimeError(
                f"Cannot zero {JOINT_NAMES[joint_index]}: reference switch B is not active"
            )

        homing_cfg = dict(cfg)
        homing_cfg["method"] = int(cfg.get("set_zero_method", 37))
        homing_cfg["active_low"] = params["reference_active_low"]
        homing_cfg["_skip_digital_input_mask_config"] = True

        print(
            f"Setting {JOINT_NAMES[joint_index]} B position as zero "
            f"with homing method {homing_cfg['method']}",
            flush=True,
        )
        # The joint is currently stopped at B in Profile Position mode.
        #
        # Replace the previous search target with the measured current
        # position before clearing Halt. If the mode switch is delayed, the
        # PP interpretation of controlword bit 4 can only command this same
        # position and cannot resume the old search movement.
        current_counts = self.get_actual_position_counts(slave)

        slave.sdo_write(
            TARGET_POSITION,
            0x00,
            pack_s32(current_counts),
        )

        # Clear the old PP handshake and command a zero-distance absolute
        # positioning operation at the current position.
        self.set_control_word(
            slave,
            CONTROL_WORD_ENABLE_OPERATION,
        )
        time.sleep(0.02)

        self.set_control_word(
            slave,
            CONTROL_WORD_START_MOTION,
        )
        time.sleep(0.05)

        self.set_control_word(
            slave,
            CONTROL_WORD_ENABLE_OPERATION,
        )

        hold_deadline = time.monotonic() + 0.50

        while time.monotonic() < hold_deadline:
            if not self.is_input_active(
                slave,
                params["reference_input"],
                active_low=params["reference_active_low"],
            ):
                physical = self.read_digital_inputs(
                    slave,
                    physical=True,
                )
                raise RuntimeError(
                    f"{JOINT_NAMES[joint_index]} left reference "
                    f"switch B while replacing the PP target; "
                    f"physical inputs=0b{physical:08b}"
                )

            time.sleep(0.01)

        # Request Homing mode directly. Do not use set_operation_mode()
        # here because that helper checks 0x6061 immediately. This drive may
        # update the displayed mode only after the mode-specific start bit
        # has been processed.
        slave.sdo_write(
            MODES_OF_OPERATION,
            0x00,
            pack_s8(HOMING_MODE),
        )

        self.configure_homing_for_joint(
            slave,
            joint_index,
            homing_cfg,
        )

        # Start method 37. If the drive has not changed modes yet, the PP
        # target is still the measured current position, so this remains a
        # zero-distance command rather than resuming the old B-search move.
        self.set_control_word(
            slave,
            CONTROL_WORD_ENABLE_OPERATION,
        )
        time.sleep(0.02)

        self.set_control_word(
            slave,
            CONTROL_WORD_START_HOMING,
        )

        mode_deadline = time.monotonic() + 1.0
        mode_confirmed = False
        last_mode = None

        while time.monotonic() < mode_deadline:
            data = slave.sdo_read(
                MODES_OF_OPERATION_DISPLAY,
                0x00,
            )
            last_mode = struct.unpack("<b", data)[0]

            if last_mode == HOMING_MODE:
                mode_confirmed = True
                break

            if not self.is_input_active(
                slave,
                params["reference_input"],
                active_low=params["reference_active_low"],
            ):
                physical = self.read_digital_inputs(
                    slave,
                    physical=True,
                )
                raise RuntimeError(
                    f"{JOINT_NAMES[joint_index]} left reference "
                    f"switch B while waiting for Homing mode; "
                    f"last displayed mode={last_mode}, "
                    f"physical inputs=0b{physical:08b}"
                )

            time.sleep(0.02)

        if not mode_confirmed:
            # Remove bit 4. The neutral PP target prevents additional motion
            # even if the drive remained in Profile Position mode.
            self.set_control_word(
                slave,
                CONTROL_WORD_ENABLE_OPERATION,
            )

            raise RuntimeError(
                f"{JOINT_NAMES[joint_index]} did not enter Homing "
                f"mode after the start request; requested "
                f"{HOMING_MODE}, displayed {last_mode}"
            )

        print(
            f"{JOINT_NAMES[joint_index]} confirmed in Homing "
            f"mode after start request",
            flush=True,
        )
        self.wait_for_homing_finished(slave, joint_index, params["timeout_s"])
        self.set_control_word(slave, CONTROL_WORD_ENABLE_OPERATION)
        time.sleep(0.05)

        actual_counts = self.get_actual_position_counts(slave)
        actual_rad = self.counts_to_radians(actual_counts, joint_index)
        self._validate_homed_position(
            joint_index,
            cfg,
            actual_counts,
            "B reference coordinate does not match the configured home offset",
        )
        if not self.is_input_active(
            slave,
            params["reference_input"],
            active_low=params["reference_active_low"],
        ):
            raise RuntimeError(
                f"{JOINT_NAMES[joint_index]} left reference switch B while zeroing"
            )

        print(
            f"{JOINT_NAMES[joint_index]} reference established at B: "
            f"actual={actual_counts} steps ({actual_rad:.6f} rad), "
            f"configured home offset={self._home_offset_steps(cfg)} steps",
            flush=True,
        )

    def park_joint_at_a(
        self,
        joint_index: int,
        cfg: dict[str, Any] | None = None,
    ) -> None:
        """Move one A/B/C joint to A and leave it halted there."""
        if cfg is None:
            cfg = self._joint_homing_config(joint_index)
        slave = self.slaves[joint_index]
        params = self._abc_homing_parameters(joint_index, cfg)
        self._configure_abc_switches(slave, cfg, params)

        print(f"Parking {JOINT_NAMES[joint_index]} at switch A", flush=True)

        a_active = self.is_input_active(
            slave,
            params["negative_input"],
            active_low=params["negative_active_low"],
        )

        if not a_active:
            status = self.get_status_word(slave)

            # A previous emergency stop or voltage disable can leave a
            # gravity-loaded joint sitting exactly on a software limit.
            #
            # When that limit lies in the same direction as the A search,
            # first move a small distance in the opposite direction. The
            # existing verified-limit-escape logic permits only movement away
            # from the active limit.
            if status & (1 << 11):
                device_status = self.get_device_status_word(slave)

                negative_limit_active = bool(
                    device_status
                    & (
                        DEVICE_STATUS_NEGATIVE_LIMIT_SWITCH
                        | DEVICE_STATUS_NEGATIVE_SOFTWARE_LIMIT
                    )
                )
                positive_limit_active = bool(
                    device_status
                    & (
                        DEVICE_STATUS_POSITIVE_LIMIT_SWITCH
                        | DEVICE_STATUS_POSITIVE_SOFTWARE_LIMIT
                    )
                )

                search_direction = (
                    1.0
                    if params["search_distance_rad"] > 0.0
                    else -1.0
                )

                pinned_in_search_direction = (
                    search_direction < 0.0
                    and negative_limit_active
                    and not positive_limit_active
                ) or (
                    search_direction > 0.0
                    and positive_limit_active
                    and not negative_limit_active
                )

                if pinned_in_search_direction:
                    escape_magnitude_rad = abs(
                        float(
                            cfg.get(
                                "prehome_limit_escape_rad",
                                0.15,
                            )
                        )
                    )

                    if escape_magnitude_rad <= 0.0:
                        raise RuntimeError(
                            f"{JOINT_NAMES[joint_index]} has an "
                            "active limit in the A-search direction, "
                            "but prehome_limit_escape_rad is not positive"
                        )

                    escape_distance_rad = (
                        -search_direction
                        * escape_magnitude_rad
                    )

                    escape_velocity = int(
                        cfg.get(
                            "prehome_limit_escape_velocity",
                            params["prehome_velocity"],
                        )
                    )
                    escape_acceleration = int(
                        cfg.get(
                            "prehome_limit_escape_acceleration",
                            params["prehome_acceleration"],
                        )
                    )

                    print(
                        f"{JOINT_NAMES[joint_index]} starts on a "
                        f"software limit before switch A. "
                        f"Escaping by "
                        f"{escape_distance_rad:+.4f} rad before "
                        "restarting the A search.",
                        flush=True,
                    )

                    self.move_joint_relative_radians(
                        joint_index,
                        escape_distance_rad,
                        velocity=escape_velocity,
                        acceleration=escape_acceleration,
                        timeout_s=params["timeout_s"],
                        wait_target=True,
                    )

                    status_after_escape = self.get_status_word(
                        slave
                    )

                    if status_after_escape & (1 << 11):
                        device_status_after = (
                            self.get_device_status_word(slave)
                        )
                        raise RuntimeError(
                            f"{JOINT_NAMES[joint_index]} remained "
                            "on an internal limit after the "
                            f"{escape_distance_rad:+.4f} rad escape; "
                            f"status=0x{status_after_escape:04X}, "
                            f"0x2324.01="
                            f"0x{device_status_after:08X}"
                        )

                    actual_counts = (
                        self.get_actual_position_counts(slave)
                    )
                    actual_rad = self.counts_to_radians(
                        actual_counts,
                        joint_index,
                    )

                    print(
                        f"{JOINT_NAMES[joint_index]} limit "
                        f"escape completed at "
                        f"{actual_rad:.6f} rad",
                        flush=True,
                    )

                    a_active = self.is_input_active(
                        slave,
                        params["negative_input"],
                        active_low=params[
                            "negative_active_low"
                        ],
                    )

        if not a_active:
            self.move_joint_relative_radians(
                joint_index,
                params["search_distance_rad"],
                velocity=params["prehome_velocity"],
                acceleration=params["prehome_acceleration"],
                timeout_s=params["timeout_s"],
                wait_target=False,
            )
            self.wait_for_input_active_with_abort(
                slave,
                joint_index,
                wanted_input=params["negative_input"],
                wanted_active_low=params["negative_active_low"],
                wanted_label="switch A",
                abort_input=params["positive_input"],
                abort_active_low=params["positive_active_low"],
                abort_label="switch C",
                timeout_s=params["timeout_s"],
            )
        else:
            print(
                f"{JOINT_NAMES[joint_index]} is already on switch A",
                flush=True,
            )

        self.halt_joint_at_current_position(slave, joint_index, "switch A")
        if not self.is_input_active(
            slave,
            params["negative_input"],
            active_low=params["negative_active_low"],
        ):
            raise RuntimeError(
                f"{JOINT_NAMES[joint_index]} did not remain on switch A after halt"
            )

    def park_joint_at_c(
        self,
        joint_index: int,
        cfg: dict[str, Any] | None = None,
    ) -> None:
        """Move one A/B/C joint positively to physical switch C and hold.

        Switch B is deliberately ignored during this move. This is required for
        homing approaches that must return from the positive side of B. A
        negative limit active at startup is tolerated only while the joint makes
        measurable progress away from it.
        """
        if cfg is None:
            cfg = self._joint_homing_config(joint_index)
        slave = self.slaves[joint_index]
        params = self._abc_homing_parameters(joint_index, cfg)
        self._configure_abc_switches(slave, cfg, params)

        print(f"Parking {JOINT_NAMES[joint_index]} at switch C", flush=True)

        if self.is_input_active(
            slave,
            params["positive_input"],
            active_low=params["positive_active_low"],
        ):
            self.halt_joint_at_current_position(slave, joint_index, "switch C")
        else:
            search_distance_rad = abs(
                float(
                    cfg.get(
                        "positive_search_distance_rad",
                        abs(params["search_distance_rad"]),
                    )
                )
            )
            if search_distance_rad <= 0.0:
                raise RuntimeError(
                    f"positive_search_distance_rad for "
                    f"{JOINT_NAMES[joint_index]} must be positive"
                )

            velocity = int(
                cfg.get("positive_search_velocity", params["prehome_velocity"])
            )
            acceleration = int(
                cfg.get(
                    "positive_search_acceleration",
                    params["prehome_acceleration"],
                )
            )
            timeout_s = float(
                cfg.get("positive_search_timeout_s", params["timeout_s"])
            )
            escape_timeout_s = float(cfg.get("limit_escape_timeout_s", 2.0))
            minimum_escape_progress_rad = float(
                cfg.get("minimum_escape_progress_rad", 0.005)
            )

            start_counts = self.get_actual_position_counts(slave)
            start_rad = self.counts_to_radians(start_counts, joint_index)
            search_start = time.monotonic()
            next_log_time = search_start

            print(
                f"Searching for C on {JOINT_NAMES[joint_index]} with monitored "
                f"positive move {search_distance_rad:+.3f} rad; switch B is "
                "intentionally ignored",
                flush=True,
            )
            self.move_joint_relative_radians(
                joint_index,
                search_distance_rad,
                velocity=velocity,
                acceleration=acceleration,
                timeout_s=timeout_s,
                wait_target=False,
            )

            while True:
                now = time.monotonic()
                actual_counts = self.get_actual_position_counts(slave)
                actual_rad = self.counts_to_radians(actual_counts, joint_index)
                progress_rad = actual_rad - start_rad
                physical = self.read_digital_inputs(slave, physical=True)
                status = self.get_status_word(slave)
                device_status = self.get_device_status_word(slave)

                if self.is_input_active(
                    slave,
                    params["positive_input"],
                    active_low=params["positive_active_low"],
                ):
                    self.halt_joint_at_current_position(
                        slave, joint_index, "switch C"
                    )
                    break

                if status & (1 << 3):
                    self.halt_joint_at_current_position(
                        slave, joint_index, "drive fault while searching for C"
                    )
                    raise RuntimeError(
                        f"Drive fault while searching for C on "
                        f"{JOINT_NAMES[joint_index]}: status=0x{status:04X}"
                    )
                if status & (1 << 13):
                    self.halt_joint_at_current_position(
                        slave,
                        joint_index,
                        "following error while searching for C",
                    )
                    raise RuntimeError(
                        f"Following error while searching for C on "
                        f"{JOINT_NAMES[joint_index]}: status=0x{status:04X}"
                    )

                internal_limit_active = bool(status & (1 << 11))
                negative_limit_active = bool(
                    device_status
                    & (
                        DEVICE_STATUS_NEGATIVE_LIMIT_SWITCH
                        | DEVICE_STATUS_NEGATIVE_SOFTWARE_LIMIT
                    )
                )
                positive_limit_active = bool(
                    device_status
                    & (
                        DEVICE_STATUS_POSITIVE_LIMIT_SWITCH
                        | DEVICE_STATUS_POSITIVE_SOFTWARE_LIMIT
                    )
                )

                if internal_limit_active:
                    escaping_negative_limit = (
                        negative_limit_active and not positive_limit_active
                    )
                    if not escaping_negative_limit:
                        self.halt_joint_at_current_position(
                            slave, joint_index, "positive boundary before switch C"
                        )
                        raise RuntimeError(
                            f"{JOINT_NAMES[joint_index]} reached an internal "
                            "positive boundary before physical switch C; "
                            f"actual={actual_rad:.6f} rad, "
                            f"inputs=0b{physical:08b}, status=0x{status:04X}, "
                            f"0x2324.01=0x{device_status:08X}"
                        )
                    if (
                        now - search_start > escape_timeout_s
                        and progress_rad < minimum_escape_progress_rad
                    ):
                        self.halt_joint_at_current_position(
                            slave, joint_index, "failed negative-limit escape"
                        )
                        raise RuntimeError(
                            f"{JOINT_NAMES[joint_index]} did not move away from "
                            "the negative limit while searching for C"
                        )

                if now - search_start > 0.5 and status & (1 << 10):
                    self.halt_joint_at_current_position(
                        slave, joint_index, "PP target reached before switch C"
                    )
                    raise RuntimeError(
                        f"{JOINT_NAMES[joint_index]} completed its positive "
                        "search move before reaching physical switch C; "
                        f"actual={actual_rad:.6f} rad, "
                        f"inputs=0b{physical:08b}"
                    )

                if now >= next_log_time:
                    print(
                        f"Searching for C on {JOINT_NAMES[joint_index]}: "
                        f"actual={actual_rad:.6f} rad, "
                        f"progress={progress_rad:+.6f} rad, "
                        f"inputs=0b{physical:08b}, status=0x{status:04X}, "
                        f"device=0x{device_status:08X}",
                        flush=True,
                    )
                    next_log_time = now + 0.5

                if now - search_start > timeout_s:
                    self.halt_joint_at_current_position(
                        slave, joint_index, "timeout searching for switch C"
                    )
                    raise TimeoutError(
                        f"Timeout searching for C on "
                        f"{JOINT_NAMES[joint_index]}"
                    )

                time.sleep(0.02)

        stable_time_s = float(cfg.get("positive_boundary_stable_time_s", 0.25))
        deadline = time.monotonic() + max(stable_time_s, 0.0)
        while time.monotonic() < deadline:
            if not self.is_input_active(
                slave,
                params["positive_input"],
                active_low=params["positive_active_low"],
            ):
                physical = self.read_digital_inputs(slave, physical=True)
                raise RuntimeError(
                    f"{JOINT_NAMES[joint_index]} left switch C after halt; "
                    f"inputs=0b{physical:08b}"
                )
            time.sleep(0.01)

        actual_counts = self.get_actual_position_counts(slave)
        actual_rad = self.counts_to_radians(actual_counts, joint_index)
        physical = self.read_digital_inputs(slave, physical=True)
        print(
            f"{JOINT_NAMES[joint_index]} parked at switch C: "
            f"actual={actual_rad:.6f} rad, inputs=0b{physical:08b}",
            flush=True,
        )

    def _read_joint_boundary_state(
        self,
        joint_index: int,
        cfg: dict[str, Any],
    ) -> dict[str, Any]:
        """Read the physical and software boundary state for one A/B/C joint."""
        slave = self.slaves[joint_index]
        params = self._abc_homing_parameters(joint_index, cfg)
        status = self.get_status_word(slave)
        device_status = self.get_device_status_word(slave)
        physical = self.read_digital_inputs(slave, physical=True)

        negative_input_active = self.is_input_active(
            slave,
            params["negative_input"],
            active_low=params["negative_active_low"],
        )
        positive_input_active = self.is_input_active(
            slave,
            params["positive_input"],
            active_low=params["positive_active_low"],
        )
        internal_limit_active = bool(status & (1 << 11))
        negative_device_limit = bool(
            device_status
            & (
                DEVICE_STATUS_NEGATIVE_LIMIT_SWITCH
                | DEVICE_STATUS_NEGATIVE_SOFTWARE_LIMIT
            )
        )
        positive_device_limit = bool(
            device_status
            & (
                DEVICE_STATUS_POSITIVE_LIMIT_SWITCH
                | DEVICE_STATUS_POSITIVE_SOFTWARE_LIMIT
            )
        )

        return {
            "status": status,
            "device_status": device_status,
            "physical": physical,
            "negative_input_active": negative_input_active,
            "positive_input_active": positive_input_active,
            "negative_active": negative_input_active
            or (internal_limit_active and negative_device_limit),
            "positive_active": positive_input_active
            or (internal_limit_active and positive_device_limit),
        }

    def hold_joint_at_negative_boundary(
        self,
        joint_index: int,
        cfg: dict[str, Any] | None = None,
        reason: str = "negative clearance boundary",
    ) -> None:
        """Verify a negative boundary and leave the operation-enabled joint halted."""
        if cfg is None:
            cfg = self._joint_homing_config(joint_index)
        slave = self.slaves[joint_index]
        state = self._read_joint_boundary_state(joint_index, cfg)

        if not state["negative_active"] or state["positive_active"]:
            actual_counts = self.get_actual_position_counts(slave)
            actual_rad = self.counts_to_radians(actual_counts, joint_index)
            raise RuntimeError(
                f"{JOINT_NAMES[joint_index]} is not exclusively at its negative "
                f"boundary: actual={actual_rad:.6f} rad, "
                f"inputs=0b{state['physical']:08b}, "
                f"status=0x{state['status']:04X}, "
                f"0x2324.01=0x{state['device_status']:08X}"
            )

        # The normal unified sequence reaches this point operation-enabled. Keep
        # the drive enabled and assert Halt; do not cycle the CiA 402 state.
        if (state["status"] & 0x006F) != 0x0027:
            self.enable_drive(slave, joint_index, PROFILE_POSITION_MODE)

        self.halt_joint_at_current_position(slave, joint_index, reason)

        stable_time_s = float(cfg.get("clearance_boundary_stable_time_s", 0.25))
        deadline = time.monotonic() + max(stable_time_s, 0.0)
        while time.monotonic() < deadline:
            state = self._read_joint_boundary_state(joint_index, cfg)
            if not state["negative_active"] or state["positive_active"]:
                actual_counts = self.get_actual_position_counts(slave)
                actual_rad = self.counts_to_radians(actual_counts, joint_index)
                raise RuntimeError(
                    f"{JOINT_NAMES[joint_index]} left its negative clearance "
                    f"boundary while being held: actual={actual_rad:.6f} rad, "
                    f"inputs=0b{state['physical']:08b}, "
                    f"status=0x{state['status']:04X}, "
                    f"0x2324.01=0x{state['device_status']:08X}"
                )
            time.sleep(0.01)

        actual_counts = self.get_actual_position_counts(slave)
        actual_rad = self.counts_to_radians(actual_counts, joint_index)
        state = self._read_joint_boundary_state(joint_index, cfg)
        print(
            f"{JOINT_NAMES[joint_index]} held at negative clearance boundary: "
            f"actual={actual_rad:.6f} rad, inputs=0b{state['physical']:08b}, "
            f"status=0x{state['status']:04X}, "
            f"0x2324.01=0x{state['device_status']:08X}",
            flush=True,
        )

    def park_joint_at_negative_boundary(
        self,
        joint_index: int,
        cfg: dict[str, Any] | None = None,
    ) -> None:
        """Move toward A and accept either A or the negative software limit.

        The lower-arm A sensor is not reached before the configured negative
        software boundary on the tested robot. For clearance, that boundary is
        therefore the repeatable parking condition. Switch B is deliberately
        ignored while travelling toward the boundary.
        """
        if cfg is None:
            cfg = self._joint_homing_config(joint_index)
        slave = self.slaves[joint_index]
        params = self._abc_homing_parameters(joint_index, cfg)
        self._configure_abc_switches(slave, cfg, params)

        direction = int(cfg.get("clearance_search_direction", -1))
        if direction != -1:
            raise RuntimeError(
                f"clearance_search_direction for {JOINT_NAMES[joint_index]} "
                "must be -1 for a negative-boundary park"
            )

        distance_rad = abs(float(cfg.get("clearance_search_distance_rad", 8.0)))
        if distance_rad <= 0.0:
            raise RuntimeError("clearance_search_distance_rad must be positive")
        velocity = int(cfg.get("clearance_search_velocity", params["prehome_velocity"]))
        acceleration = int(
            cfg.get("clearance_search_acceleration", params["prehome_acceleration"])
        )
        timeout_s = float(cfg.get("clearance_search_timeout_s", params["timeout_s"]))

        initial_state = self._read_joint_boundary_state(joint_index, cfg)
        if initial_state["negative_active"] and not initial_state["positive_active"]:
            print(
                f"{JOINT_NAMES[joint_index]} is already at its negative "
                "clearance boundary",
                flush=True,
            )
            self.hold_joint_at_negative_boundary(joint_index, cfg)
            return

        print(
            f"Parking {JOINT_NAMES[joint_index]} at negative clearance "
            f"boundary with move {-distance_rad:+.3f} rad",
            flush=True,
        )
        self.move_joint_relative_radians(
            joint_index,
            -distance_rad,
            velocity=velocity,
            acceleration=acceleration,
            timeout_s=timeout_s,
            wait_target=False,
        )

        start_time = time.monotonic()
        next_log_time = start_time
        while True:
            now = time.monotonic()
            state = self._read_joint_boundary_state(joint_index, cfg)
            actual_counts = self.get_actual_position_counts(slave)
            actual_rad = self.counts_to_radians(actual_counts, joint_index)

            if state["negative_active"] and not state["positive_active"]:
                boundary_kind = (
                    "switch A"
                    if state["negative_input_active"]
                    else "negative software/drive boundary"
                )
                self.halt_joint_at_current_position(
                    slave, joint_index, boundary_kind
                )
                self.hold_joint_at_negative_boundary(
                    joint_index, cfg, reason=boundary_kind
                )
                return

            if state["positive_active"]:
                self.halt_joint_at_current_position(
                    slave, joint_index, "unexpected positive boundary"
                )
                raise RuntimeError(
                    f"{JOINT_NAMES[joint_index]} reached its positive boundary "
                    "while commanded toward the negative clearance boundary"
                )

            if state["status"] & (1 << 3):
                self.halt_joint_at_current_position(slave, joint_index, "drive fault")
                raise RuntimeError(
                    f"Drive fault while parking {JOINT_NAMES[joint_index]} at "
                    f"the negative boundary: status=0x{state['status']:04X}"
                )
            if state["status"] & (1 << 13):
                self.halt_joint_at_current_position(
                    slave, joint_index, "following error"
                )
                raise RuntimeError(
                    f"Following error while parking {JOINT_NAMES[joint_index]} "
                    f"at the negative boundary: status=0x{state['status']:04X}"
                )

            if now - start_time > 0.5 and state["status"] & (1 << 10):
                self.halt_joint_at_current_position(
                    slave, joint_index, "PP target reached before boundary"
                )
                raise RuntimeError(
                    f"{JOINT_NAMES[joint_index]} completed its clearance move "
                    "without reaching A or the negative software boundary; "
                    f"actual={actual_rad:.6f} rad, "
                    f"inputs=0b{state['physical']:08b}"
                )

            if now >= next_log_time:
                print(
                    f"Parking {JOINT_NAMES[joint_index]}: "
                    f"actual={actual_rad:.6f} rad, "
                    f"inputs=0b{state['physical']:08b}, "
                    f"status=0x{state['status']:04X}, "
                    f"device=0x{state['device_status']:08X}",
                    flush=True,
                )
                next_log_time = now + 0.5

            if now - start_time > timeout_s:
                self.halt_joint_at_current_position(
                    slave, joint_index, "negative-boundary parking timeout"
                )
                raise TimeoutError(
                    f"Timeout parking {JOINT_NAMES[joint_index]} at its "
                    "negative clearance boundary"
                )

            time.sleep(0.02)

    def home_joint_from_a_to_b(
        self,
        joint_index: int,
        cfg: dict[str, Any] | None = None,
    ) -> None:
        """Back away from A, find B in monitored PP motion, then zero at B."""
        if cfg is None:
            cfg = self._joint_homing_config(joint_index)
        slave = self.slaves[joint_index]
        params = self._abc_homing_parameters(joint_index, cfg)
        self._configure_abc_switches(slave, cfg, params)

        if not self.is_input_active(
            slave,
            params["negative_input"],
            active_low=params["negative_active_low"],
        ):
            raise RuntimeError(
                f"{JOINT_NAMES[joint_index]} must be physically on A before "
                "HOME_FROM_A_TO_B"
            )

        backoff_distance_rad = params["backoff_distance_rad"]
        reference_search_distance_rad = params["reference_search_distance_rad"]
        if backoff_distance_rad * reference_search_distance_rad <= 0.0:
            raise RuntimeError(
                f"{JOINT_NAMES[joint_index]} reference_backoff_rad and "
                "reference_search_distance_rad must have the same A-to-B sign"
            )

        print(
            f"Backing {JOINT_NAMES[joint_index]} away from A toward B by "
            f"{backoff_distance_rad:+.3f} rad",
            flush=True,
        )
        self.move_joint_relative_radians(
            joint_index,
            backoff_distance_rad,
            velocity=params["prehome_velocity"],
            acceleration=params["prehome_acceleration"],
            timeout_s=params["timeout_s"],
            wait_target=True,
        )
        self.wait_for_input_inactive(
            slave,
            joint_index,
            params["negative_input"],
            params["negative_active_low"],
            timeout_s=params["timeout_s"],
            label="switch A",
        )

        if not self.is_input_active(
            slave,
            params["reference_input"],
            active_low=params["reference_active_low"],
        ):
            print(
                f"Searching for B on {JOINT_NAMES[joint_index]} with monitored "
                f"relative move {reference_search_distance_rad:+.3f} rad",
                flush=True,
            )
            self.move_joint_relative_radians(
                joint_index,
                reference_search_distance_rad,
                velocity=params["reference_search_velocity"],
                acceleration=params["reference_search_acceleration"],
                timeout_s=params["timeout_s"],
                wait_target=False,
            )
            self.wait_for_input_active_with_abort(
                slave,
                joint_index,
                wanted_input=params["reference_input"],
                wanted_active_low=params["reference_active_low"],
                wanted_label="switch B",
                abort_input=params["positive_input"],
                abort_active_low=params["positive_active_low"],
                abort_label="switch C",
                timeout_s=params["timeout_s"],
            )

        self.halt_joint_at_current_position(slave, joint_index, "switch B")
        self.set_current_position_as_home(joint_index, cfg)

    def _search_reference_switch_one_direction(
        self,
        joint_index: int,
        cfg: dict[str, Any],
        direction: int,
        distance_rad: float,
    ) -> str:
        """Search for B in one direction.

        Returns ``"reference"`` when B is detected and ``"boundary"`` when
        the end switch/software limit in the commanded direction is reached.
        The initial software-limit state behind the motion is tolerated while
        the joint escapes from it.
        """
        if direction not in (-1, 1):
            raise ValueError("direction must be -1 or +1")

        slave = self.slaves[joint_index]
        params = self._abc_homing_parameters(joint_index, cfg)
        signed_distance = abs(float(distance_rad)) * float(direction)
        velocity = int(cfg.get("direct_search_velocity", params["reference_search_velocity"]))
        acceleration = int(
            cfg.get("direct_search_acceleration", params["reference_search_acceleration"])
        )
        timeout_s = float(cfg.get("direct_search_timeout_s", params["timeout_s"]))
        escape_timeout_s = float(cfg.get("limit_escape_timeout_s", 2.0))
        minimum_escape_progress_rad = float(
            cfg.get("minimum_escape_progress_rad", 0.005)
        )

        start_counts = self.get_actual_position_counts(slave)
        start_rad = self.counts_to_radians(start_counts, joint_index)
        search_start = time.monotonic()
        next_log_time = search_start

        print(
            f"Searching directly for B on {JOINT_NAMES[joint_index]}: "
            f"direction={direction:+d}, move={signed_distance:+.3f} rad",
            flush=True,
        )

        self.move_joint_relative_radians(
            joint_index,
            signed_distance,
            velocity=velocity,
            acceleration=acceleration,
            timeout_s=timeout_s,
            wait_target=False,
        )

        boundary_input = (
            params["positive_input"] if direction > 0 else params["negative_input"]
        )
        boundary_active_low = (
            params["positive_active_low"]
            if direction > 0
            else params["negative_active_low"]
        )
        boundary_label = "switch C" if direction > 0 else "switch A"
        trailing_software_limit = (
            DEVICE_STATUS_NEGATIVE_SOFTWARE_LIMIT
            if direction > 0
            else DEVICE_STATUS_POSITIVE_SOFTWARE_LIMIT
        )
        leading_software_limit = (
            DEVICE_STATUS_POSITIVE_SOFTWARE_LIMIT
            if direction > 0
            else DEVICE_STATUS_NEGATIVE_SOFTWARE_LIMIT
        )

        while True:
            now = time.monotonic()
            actual_counts = self.get_actual_position_counts(slave)
            actual_rad = self.counts_to_radians(actual_counts, joint_index)
            progress_rad = float(direction) * (actual_rad - start_rad)
            physical = self.read_digital_inputs(slave, physical=True)
            status = self.get_status_word(slave)
            device_status = self.get_device_status_word(slave)

            if self.is_input_active(
                slave,
                params["reference_input"],
                active_low=params["reference_active_low"],
            ):
                self.halt_joint_at_current_position(
                    slave, joint_index, "reference switch B"
                )
                print(
                    f"B detected on {JOINT_NAMES[joint_index]} at "
                    f"{actual_rad:.6f} rad; inputs=0b{physical:08b}",
                    flush=True,
                )
                return "reference"

            if self.is_input_active(
                slave, boundary_input, active_low=boundary_active_low
            ):
                self.halt_joint_at_current_position(
                    slave, joint_index, boundary_label
                )
                print(
                    f"{boundary_label} reached before B on "
                    f"{JOINT_NAMES[joint_index]}; reversing is required",
                    flush=True,
                )
                return "boundary"

            if status & (1 << 3):
                self.halt_joint_at_current_position(
                    slave, joint_index, "drive fault"
                )
                raise RuntimeError(
                    f"Drive fault while searching directly for B on "
                    f"{JOINT_NAMES[joint_index]}: status=0x{status:04X}"
                )
            if status & (1 << 13):
                self.halt_joint_at_current_position(
                    slave, joint_index, "following error"
                )
                raise RuntimeError(
                    f"Following error while searching directly for B on "
                    f"{JOINT_NAMES[joint_index]}: status=0x{status:04X}"
                )

            if device_status & leading_software_limit:
                self.halt_joint_at_current_position(
                    slave, joint_index, "software limit before B"
                )
                print(
                    f"Software limit reached before B on "
                    f"{JOINT_NAMES[joint_index]}; reversing is required",
                    flush=True,
                )
                return "boundary"

            if (
                device_status & trailing_software_limit
                and now - search_start > escape_timeout_s
                and progress_rad < minimum_escape_progress_rad
            ):
                self.halt_joint_at_current_position(
                    slave, joint_index, "failed software-limit escape"
                )
                raise RuntimeError(
                    f"{JOINT_NAMES[joint_index]} did not move away from the "
                    "software limit behind the direct B search"
                )

            if now - search_start > 0.5 and status & (1 << 10):
                self.halt_joint_at_current_position(
                    slave, joint_index, "PP target reached before B"
                )
                return "boundary"

            if now >= next_log_time:
                print(
                    f"Direct B search {JOINT_NAMES[joint_index]}: "
                    f"actual={actual_rad:.6f} rad, progress={progress_rad:+.6f} rad, "
                    f"inputs=0b{physical:08b}, status=0x{status:04X}, "
                    f"device=0x{device_status:08X}",
                    flush=True,
                )
                next_log_time = now + 0.5

            if now - search_start > timeout_s:
                self.halt_joint_at_current_position(
                    slave, joint_index, "timeout searching directly for B"
                )
                raise TimeoutError(
                    f"Timeout searching directly for B on "
                    f"{JOINT_NAMES[joint_index]}"
                )

            time.sleep(0.02)

    def set_current_single_switch_as_zero(
        self,
        joint_index: int,
        cfg: dict[str, Any],
    ) -> None:
        """Assign the active single-switch position its calibrated home coordinate."""
        slave = self.slaves[joint_index]
        input_number = int(cfg["reference_switch_input"])
        active_low = bool(cfg.get("active_low", False))
        timeout_s = float(cfg.get("timeout_s", 45.0))

        if not self.is_input_active(slave, input_number, active_low=active_low):
            raise RuntimeError(
                f"Cannot zero {JOINT_NAMES[joint_index]}: reference switch "
                f"DigIn{input_number} is not active"
            )

        current_counts = self.get_actual_position_counts(slave)
        slave.sdo_write(TARGET_POSITION, 0x00, pack_s32(current_counts))
        self.set_control_word(slave, CONTROL_WORD_ENABLE_OPERATION)
        time.sleep(0.02)

        homing_cfg = dict(cfg)
        homing_cfg["method"] = 37
        homing_cfg["switch_type"] = "home"
        homing_cfg["_skip_digital_input_mask_config"] = True

        slave.sdo_write(MODES_OF_OPERATION, 0x00, pack_s8(HOMING_MODE))
        self.configure_homing_for_joint(slave, joint_index, homing_cfg)
        self.set_control_word(slave, CONTROL_WORD_ENABLE_OPERATION)
        time.sleep(0.02)
        self.set_control_word(slave, CONTROL_WORD_START_HOMING)

        self.wait_for_homing_finished(slave, joint_index, timeout_s)
        self.set_control_word(slave, CONTROL_WORD_ENABLE_OPERATION)
        time.sleep(0.05)

        actual_counts = self.get_actual_position_counts(slave)
        actual_rad = self.counts_to_radians(actual_counts, joint_index)
        self._validate_homed_position(
            joint_index,
            cfg,
            actual_counts,
            "single-switch coordinate does not match the configured home offset",
        )
        if not self.is_input_active(slave, input_number, active_low=active_low):
            raise RuntimeError(
                f"{JOINT_NAMES[joint_index]} left DigIn{input_number} while zeroing"
            )

        print(
            f"{JOINT_NAMES[joint_index]} homed at DigIn{input_number}: "
            f"actual={actual_counts} steps ({actual_rad:.6f} rad), "
            f"configured home offset={self._home_offset_steps(cfg)} steps",
            flush=True,
        )

    def home_end_effector_toward_zero(
        self,
        joint_index: int,
        cfg: dict[str, Any] | None = None,
    ) -> None:
        """Move the end effector from its current coordinate toward zero.

        This strategy is intentionally limited to the end-effector axis. It
        reads the current position, chooses the sign that reduces its absolute
        value, and continues slightly beyond the calculated zero until the
        single physical reference switch becomes active. The switch location is
        then assigned to exactly zero with homing method 37.
        """
        if JOINT_NAMES[joint_index] != "end_effector_joint":
            raise RuntimeError(
                "toward_zero_single_switch is only valid for end_effector_joint"
            )
        if cfg is None:
            cfg = self._joint_homing_config(joint_index)

        slave = self.slaves[joint_index]
        input_number = int(cfg["reference_switch_input"])
        active_low = bool(cfg.get("active_low", False))
        filter_enabled = bool(cfg.get("filter_enabled", True))
        timeout_s = float(cfg.get("timeout_s", 45.0))
        velocity = int(cfg.get("seek_velocity", 30))
        acceleration = int(cfg.get("acceleration", 20))
        margin_rad = abs(float(cfg.get("zero_search_margin_rad", 0.20)))
        zero_epsilon_rad = abs(float(cfg.get("zero_direction_epsilon_rad", 0.0005)))
        zero_fallback_direction = int(cfg.get("zero_fallback_direction", 1))
        if zero_fallback_direction not in (-1, 1):
            raise RuntimeError(
                "end_effector_joint zero_fallback_direction must be -1 or 1"
            )

        self.configure_input_polarity_and_filters(
            slave,
            [(input_number, active_low, filter_enabled)],
        )

        if self.is_input_active(slave, input_number, active_low=active_low):
            self.halt_joint_at_current_position(
                slave, joint_index, f"DigIn{input_number} already active"
            )
            self.set_current_single_switch_as_zero(joint_index, cfg)
            return

        start_counts = self.get_actual_position_counts(slave)
        start_rad = self.counts_to_radians(start_counts, joint_index)

        # The switch is defined as q=0. Move according to the sign of the
        # measured position so that |q| decreases. Only an effectively-zero
        # reading is ambiguous; in that case use the configured physical search
        # direction instead of refusing to home.
        # if start_rad > zero_epsilon_rad:
        #     direction = -1
        #     direction_reason = "positive position -> negative motion"
        # elif start_rad < -zero_epsilon_rad:
        #     direction = 1
        #     direction_reason = "negative position -> positive motion"
        # else:
        #     direction = zero_fallback_direction
        #     direction_reason = (
        #         "position approximately zero -> configured fallback direction"
        #     )
        direction = int(cfg.get("search_direction", -1))

        if direction not in (-1, 1):
            raise RuntimeError(
                f"Invalid search_direction for end_effector_joint: {direction}. "
                "Expected -1 or 1."
            )

        # delta_rad = -start_rad + direction * margin_rad

        search_distance_rad = float(
            cfg.get("zero_search_margin_rad", 0.84)
        )

        delta_rad = direction * search_distance_rad
        print(
            f"Homing {JOINT_NAMES[joint_index]} toward zero: "
            f"current={start_rad:.6f} rad, direction={direction:+d} "
            f"move={delta_rad:+.6f} rad, "
            f"switch=DigIn{input_number}",
            flush=True,
        )

        self.move_joint_relative_radians(
            joint_index,
            delta_rad,
            velocity=velocity,
            acceleration=acceleration,
            timeout_s=timeout_s,
            wait_target=False,
        )

        started = time.monotonic()
        next_log = started
        while True:
            now = time.monotonic()
            actual_counts = self.get_actual_position_counts(slave)
            actual_rad = self.counts_to_radians(actual_counts, joint_index)
            status = self.get_status_word(slave)
            physical = self.read_digital_inputs(slave, physical=True)

            if self.is_input_active(slave, input_number, active_low=active_low):
                self.halt_joint_at_current_position(
                    slave, joint_index, f"reference switch DigIn{input_number}"
                )
                self.set_current_single_switch_as_zero(joint_index, cfg)
                return

            if status & (1 << 3):
                self.halt_joint_at_current_position(slave, joint_index, "drive fault")
                raise RuntimeError(
                    f"Drive fault while homing {JOINT_NAMES[joint_index]}: "
                    f"status=0x{status:04X}"
                )
            if status & (1 << 13):
                self.halt_joint_at_current_position(
                    slave, joint_index, "following error"
                )
                raise RuntimeError(
                    f"Following error while homing {JOINT_NAMES[joint_index]}: "
                    f"status=0x{status:04X}"
                )
            if now - started > 0.25 and status & (1 << 10):
                self.halt_joint_at_current_position(
                    slave, joint_index, "target reached before reference switch"
                )
                raise RuntimeError(
                    f"{JOINT_NAMES[joint_index]} reached the calculated zero "
                    f"search target without activating DigIn{input_number}; "
                    f"actual={actual_rad:.6f} rad, inputs=0b{physical:08b}"
                )
            if now >= next_log:
                print(
                    f"End-effector homing: actual={actual_rad:.6f} rad, "
                    f"inputs=0b{physical:08b}, status=0x{status:04X}",
                    flush=True,
                )
                next_log = now + 0.5
            if now - started > timeout_s:
                self.halt_joint_at_current_position(
                    slave, joint_index, "single-switch homing timeout"
                )
                raise TimeoutError(
                    f"Timeout homing {JOINT_NAMES[joint_index]} toward zero"
                )
            time.sleep(0.02)

    def home_joint_direct_to_reference(
        self,
        joint_index: int,
        cfg: dict[str, Any] | None = None,
    ) -> None:
        """Find B directly from either side, then establish B as zero.

        The lower arm uses this directly from its negative clearance boundary.
        It also remains available for manual diagnostics. Normal upper-arm
        homing uses the dedicated one-way ``home_joint_from_c_to_b()`` routine.
        """
        if cfg is None:
            cfg = self._joint_homing_config(joint_index)
        slave = self.slaves[joint_index]
        params = self._abc_homing_parameters(joint_index, cfg)
        self._configure_abc_switches(slave, cfg, params)

        prerequisite_at_a = cfg.get("requires_joint_at_a")
        prerequisite_at_boundary = cfg.get("requires_joint_at_negative_boundary")
        if prerequisite_at_a is not None and prerequisite_at_boundary is not None:
            raise RuntimeError(
                f"{JOINT_NAMES[joint_index]} cannot require both A and a "
                "negative-boundary clearance prerequisite"
            )

        if prerequisite_at_boundary is not None:
            prerequisite_name = str(prerequisite_at_boundary)
            if prerequisite_name not in JOINT_NAMES:
                raise RuntimeError(
                    f"Unknown requires_joint_at_negative_boundary value for "
                    f"{JOINT_NAMES[joint_index]}: {prerequisite_name}"
                )
            prerequisite_index = JOINT_NAMES.index(prerequisite_name)
            prerequisite_cfg = self._joint_homing_config(prerequisite_index)
            self._configure_abc_switches(
                self.slaves[prerequisite_index],
                prerequisite_cfg,
                self._abc_homing_parameters(prerequisite_index, prerequisite_cfg),
            )
            self.hold_joint_at_negative_boundary(
                prerequisite_index,
                prerequisite_cfg,
                reason=f"required clearance for {JOINT_NAMES[joint_index]}",
            )

        if prerequisite_at_a is not None:
            prerequisite_name = str(prerequisite_at_a)
            if prerequisite_name not in JOINT_NAMES:
                raise RuntimeError(
                    f"Unknown requires_joint_at_a value for "
                    f"{JOINT_NAMES[joint_index]}: {prerequisite_name}"
                )
            prerequisite_index = JOINT_NAMES.index(prerequisite_name)
            prerequisite_cfg = self._joint_homing_config(prerequisite_index)
            prerequisite_params = self._abc_homing_parameters(
                prerequisite_index, prerequisite_cfg
            )
            prerequisite_slave = self.slaves[prerequisite_index]
            self._configure_abc_switches(
                prerequisite_slave, prerequisite_cfg, prerequisite_params
            )
            if not self.is_input_active(
                prerequisite_slave,
                prerequisite_params["negative_input"],
                active_low=prerequisite_params["negative_active_low"],
            ):
                physical = self.read_digital_inputs(
                    prerequisite_slave, physical=True
                )
                raise RuntimeError(
                    f"{JOINT_NAMES[joint_index]} direct B homing requires "
                    f"{prerequisite_name} at A; inputs=0b{physical:08b}"
                )
            self.halt_joint_at_current_position(
                prerequisite_slave,
                prerequisite_index,
                f"required A clearance for {JOINT_NAMES[joint_index]}",
            )

        if self.is_input_active(
            slave,
            params["reference_input"],
            active_low=params["reference_active_low"],
        ):
            self.halt_joint_at_current_position(slave, joint_index, "switch B")
            self.set_current_position_as_home(joint_index, cfg)
            return

        preferred_direction = int(cfg.get("direct_search_preferred_direction", 1))
        if preferred_direction not in (-1, 1):
            raise RuntimeError(
                f"direct_search_preferred_direction for "
                f"{JOINT_NAMES[joint_index]} must be -1 or +1"
            )

        # Starting on a known physical or software boundary removes ambiguity.
        # Otherwise use the configured preferred direction; if the far boundary
        # is reached, the second pass reverses and must cross B.
        boundary_state = self._read_joint_boundary_state(joint_index, cfg)
        if boundary_state["negative_active"] and not boundary_state["positive_active"]:
            first_direction = 1
        elif boundary_state["positive_active"] and not boundary_state["negative_active"]:
            first_direction = -1
        else:
            first_direction = preferred_direction

        search_distance_rad = float(
            cfg.get(
                "direct_search_distance_rad",
                abs(params["reference_search_distance_rad"]),
            )
        )
        reverse_distance_rad = float(
            cfg.get("direct_reverse_search_distance_rad", search_distance_rad)
        )

        result = self._search_reference_switch_one_direction(
            joint_index, cfg, first_direction, search_distance_rad
        )
        if result != "reference":
            result = self._search_reference_switch_one_direction(
                joint_index, cfg, -first_direction, reverse_distance_rad
            )
        if result != "reference":
            raise RuntimeError(
                f"{JOINT_NAMES[joint_index]} could not find B in either direction"
            )

        self.set_current_position_as_home(joint_index, cfg)

    def home_joint_from_c_to_b(
        self,
        joint_index: int,
        cfg: dict[str, Any] | None = None,
    ) -> None:
        """Search only from physical switch C toward B, then zero at B.

        Before homing, the drive's position coordinate may place B just beyond
        the normal negative software-position limit. For this dedicated C-to-B
        move, extend only that negative limit by a small configured amount. The
        physical A switch remains active as the hard monitored abort boundary.
        The original software limits are restored after method 37 establishes B
        as zero, or immediately after any failure.

        Unlike ``home_joint_direct_to_reference()``, this routine never reverses
        back toward C when B is not found. Reversal is unsafe and contradicts
        the required C-to-B reference approach.
        """
        if cfg is None:
            cfg = self._joint_homing_config(joint_index)

        slave = self.slaves[joint_index]
        params = self._abc_homing_parameters(joint_index, cfg)
        self._configure_abc_switches(slave, cfg, params)

        if not self.is_input_active(
            slave,
            params["positive_input"],
            active_low=params["positive_active_low"],
        ):
            physical = self.read_digital_inputs(slave, physical=True)
            raise RuntimeError(
                f"{JOINT_NAMES[joint_index]} must be physically on C before "
                f"C-to-B homing; inputs=0b{physical:08b}"
            )

        search_distance_rad = float(
            cfg.get(
                "c_to_b_search_distance_rad",
                cfg.get("direct_search_distance_rad", 2.20),
            )
        )
        if search_distance_rad <= 0.0:
            raise RuntimeError(
                f"c_to_b_search_distance_rad for "
                f"{JOINT_NAMES[joint_index]} must be positive"
            )

        extension_rad = float(
            cfg.get("c_to_b_negative_software_limit_extension_rad", 0.0)
        )
        if extension_rad < 0.0:
            raise RuntimeError(
                f"c_to_b_negative_software_limit_extension_rad for "
                f"{JOINT_NAMES[joint_index]} must be non-negative"
            )

        original_minimum, original_maximum = (
            self.get_software_position_limits_counts(slave)
        )
        extension_counts = self.radians_to_counts(extension_rad, joint_index)
        temporary_minimum = original_minimum - extension_counts
        if temporary_minimum < -(2**31):
            raise OverflowError(
                f"Temporary negative software limit for "
                f"{JOINT_NAMES[joint_index]} does not fit in S32"
            )
        if temporary_minimum >= original_maximum:
            raise RuntimeError(
                f"Invalid temporary software-position range for "
                f"{JOINT_NAMES[joint_index]}: min={temporary_minimum}, "
                f"max={original_maximum}"
            )

        original_minimum_rad = self.counts_to_radians(
            original_minimum, joint_index
        )
        temporary_minimum_rad = self.counts_to_radians(
            temporary_minimum, joint_index
        )
        print(
            f"Preparing dedicated C-to-B search for "
            f"{JOINT_NAMES[joint_index]}: negative software limit "
            f"{original_minimum} counts ({original_minimum_rad:.6f} rad) -> "
            f"{temporary_minimum} counts ({temporary_minimum_rad:.6f} rad); "
            "physical switch A remains active",
            flush=True,
        )

        limit_was_changed = temporary_minimum != original_minimum
        try:
            if limit_was_changed:
                self.set_software_position_limit_counts(
                    slave,
                    minimum_counts=temporary_minimum,
                )

            result = self._search_reference_switch_one_direction(
                joint_index,
                cfg,
                direction=-1,
                distance_rad=search_distance_rad,
            )
            if result != "reference":
                raise RuntimeError(
                    f"{JOINT_NAMES[joint_index]} did not reach B while moving "
                    "from C in the negative direction. The search was stopped "
                    "at A, the temporary negative software limit, or its PP "
                    "target; it will not reverse toward C."
                )

            self.set_current_position_as_home(joint_index, cfg)
        finally:
            if limit_was_changed:
                restored_minimum, restored_maximum = (
                    self.set_software_position_limit_counts(
                        slave,
                        minimum_counts=original_minimum,
                        maximum_counts=original_maximum,
                    )
                )
                print(
                    f"Restored {JOINT_NAMES[joint_index]} software-position "
                    f"limits: min={restored_minimum}, max={restored_maximum}",
                    flush=True,
                )

    def validate_homing_references(self) -> set[str]:
        """Verify each required joint using its configured homing strategy.

        The three arm axes use their A/B/C reference-switch configuration. The
        end effector uses one dedicated reference switch and therefore must not
        be passed through the arm-specific A/B/C validator.
        """
        homing = self._homing_section()
        names = homing.get(
            "required_reference_joints",
            [
                "shoulder_joint",
                "upperarm_joint",
                "lowerarm_joint",
                "end_effector_joint",
            ],
        )
        failures: list[str] = []
        validated: set[str] = set()

        print("Final homing reference verification:", flush=True)
        for name in names:
            joint_failures: list[str] = []
            if name not in JOINT_NAMES:
                failures.append(f"unknown joint {name}")
                continue

            index = JOINT_NAMES.index(name)
            cfg = self._joint_homing_config(index)
            strategy = str(cfg.get("strategy", "faulhaber_home_switch"))
            slave = self.slaves[index]
            physical = self.read_digital_inputs(slave, physical=True)
            actual_counts = self.get_actual_position_counts(slave)
            actual_rad = self.counts_to_radians(actual_counts, index)
            tolerance_rad = float(cfg.get("home_zero_tolerance_rad", 0.02))

            if strategy == "toward_zero_single_switch":
                input_number = int(cfg["reference_switch_input"])
                active_low = bool(cfg.get("active_low", False))
                reference_active = self.is_input_active(
                    slave,
                    input_number,
                    active_low=active_low,
                )
                print(
                    f"  {name}: DigIn{input_number}_active={reference_active}, "
                    f"actual={actual_counts} steps ({actual_rad:.6f} rad), "
                    f"expected_offset={self._home_offset_steps(cfg)} steps, "
                    f"inputs=0b{physical:08b}",
                    flush=True,
                )
                if not reference_active:
                    joint_failures.append(
                        f"{name} DigIn{input_number} reference is not active"
                    )
            else:
                params = self._abc_homing_parameters(index, cfg)
                reference_active = self.is_input_active(
                    slave,
                    params["reference_input"],
                    active_low=params["reference_active_low"],
                )
                print(
                    f"  {name}: B_active={reference_active}, "
                    f"actual={actual_counts} steps ({actual_rad:.6f} rad), "
                    f"expected_offset={self._home_offset_steps(cfg)} steps, "
                    f"inputs=0b{physical:08b}",
                    flush=True,
                )
                if not reference_active:
                    joint_failures.append(f"{name} B is not active")

            expected_counts = self._home_offset_steps(cfg)
            error_counts = actual_counts - expected_counts
            error_rad = self.counts_to_radians(error_counts, index)
            if abs(error_rad) > tolerance_rad:
                expected_rad = self.counts_to_radians(expected_counts, index)
                joint_failures.append(
                    f"{name} actual={actual_counts} steps ({actual_rad:.6f} rad), "
                    f"expected={expected_counts} steps ({expected_rad:.6f} rad), "
                    f"error={error_counts} steps ({error_rad:.6f} rad) exceeds "
                    f"tolerance={tolerance_rad:.6f} rad"
                )

            if joint_failures:
                failures.extend(joint_failures)
            else:
                validated.add(name)

        if failures:
            raise RuntimeError(
                "Unified homing verification failed: " + "; ".join(failures)
            )

        print(
            "Unified homing verified for: " + ", ".join(sorted(validated)),
            flush=True,
        )
        return validated

    def home_joint_via_positive_limit_then_reference(
        self,
        joint_index: int,
        cfg: dict[str, Any],
    ) -> None:
        """Run the monitored C -> B homing strategy.

        The configured clearance prerequisite is re-verified before the joint
        travels toward C. After C is reached, the dedicated one-way C-to-B
        routine searches negatively for B without a reverse fallback and
        preserves the already-tested method-37 zeroing procedure.
        """
        print(
            f"Starting software-monitored C-to-B homing for "
            f"{JOINT_NAMES[joint_index]}",
            flush=True,
        )

        prerequisite_name_value = cfg.get(
            "requires_joint_at_negative_boundary"
        )
        if prerequisite_name_value is not None:
            prerequisite_name = str(prerequisite_name_value)
            if prerequisite_name not in JOINT_NAMES:
                raise RuntimeError(
                    f"Unknown requires_joint_at_negative_boundary value for "
                    f"{JOINT_NAMES[joint_index]}: {prerequisite_name}"
                )
            prerequisite_index = JOINT_NAMES.index(prerequisite_name)
            prerequisite_cfg = self._joint_homing_config(prerequisite_index)
            prerequisite_params = self._abc_homing_parameters(
                prerequisite_index, prerequisite_cfg
            )
            self._configure_abc_switches(
                self.slaves[prerequisite_index],
                prerequisite_cfg,
                prerequisite_params,
            )
            self.hold_joint_at_negative_boundary(
                prerequisite_index,
                prerequisite_cfg,
                reason=(
                    f"required clearance for {JOINT_NAMES[joint_index]} "
                    "C-to-B homing"
                ),
            )

        self.park_joint_at_c(joint_index, cfg)
        self.home_joint_from_c_to_b(joint_index, cfg)

    def home_joint_via_negative_limit_then_reference(
        self,
        joint_index: int,
        cfg: dict[str, Any],
    ) -> None:
        """Run the software-monitored A -> B homing strategy."""
        print(
            f"Starting software-monitored A/B/C homing for "
            f"{JOINT_NAMES[joint_index]}",
            flush=True,
        )
        self.park_joint_at_a(joint_index, cfg)
        self.home_joint_from_a_to_b(joint_index, cfg)

    def move_joints(
        self,
        joint_positions_rad: list[float],
        velocity: int = 80,
        acceleration: int = 20,
        timeout_s: float = 90.0,
        tolerance_rad: float = 0.01,
        acknowledge_timeout_s: float = 2.0,
        joint_indices: list[int] | None = None,
    ) -> None:
        """Move selected joints sequentially in PP mode with verified feedback.

        Sequential execution is intentional during the current diagnostic phase:
        each selected drive must acknowledge its command and reach the requested
        0x6064 position before the next joint can move. Unselected joints receive
        no Profile Position target. Joints already within tolerance remain still.
        """
        if len(joint_positions_rad) != EXPECTED_SLAVE_COUNT:
            raise ValueError(
                f"Expected {EXPECTED_SLAVE_COUNT} joint commands, got {len(joint_positions_rad)}"
            )

        selected_indices = (
            list(range(EXPECTED_SLAVE_COUNT))
            if joint_indices is None
            else list(joint_indices)
        )
        if not selected_indices:
            raise ValueError("joint_indices must contain at least one joint")
        if len(set(selected_indices)) != len(selected_indices):
            raise ValueError("joint_indices must not contain duplicates")
        invalid_indices = [
            index
            for index in selected_indices
            if index < 0 or index >= EXPECTED_SLAVE_COUNT
        ]
        if invalid_indices:
            raise ValueError(f"Invalid joint indices: {invalid_indices}")

        for index in selected_indices:
            target_rad = joint_positions_rad[index]
            slave = self.slaves[index]
            self.enable_drive(slave, index, PROFILE_POSITION_MODE)
            self.configure_profile_position_motion(
                slave, velocity, acceleration
            )

            initial_counts = self.get_actual_position_counts(slave)
            initial_rad = self.counts_to_radians(initial_counts, index)
            target_counts = self.radians_to_counts(target_rad, index)
            initial_status = self.get_status_word(slave)
            allow_internal_limit_escape = False
            if initial_status & (1 << 11):
                allow_internal_limit_escape = self._verified_limit_escape(
                    slave, index, initial_counts, target_counts
                )
                if not allow_internal_limit_escape:
                    self._check_profile_position_status(
                        initial_status, index, "pick-ready position check"
                    )
            print(
                f"{JOINT_NAMES[index]} pick-ready move: "
                f"initial={initial_rad:.4f} rad, target={target_rad:.4f} rad",
                flush=True,
            )

            if abs(target_rad - initial_rad) <= tolerance_rad:
                status = self.get_status_word(slave)
                self._check_profile_position_status(
                    status, index, "pick-ready position check"
                )
                print(
                    f"{JOINT_NAMES[index]} already within "
                    f"{tolerance_rad:.4f} rad tolerance; no move commanded",
                    flush=True,
                )
                continue

            if allow_internal_limit_escape:
                escape_distance_rad = float(
                    self._homing_section().get(
                        "pick_ready_limit_escape_distance_rad", 0.05
                    )
                )
                if escape_distance_rad <= 0.0:
                    raise ValueError(
                        "pick_ready_limit_escape_distance_rad must be positive"
                    )

                escape_delta_counts = max(
                    1, abs(self.radians_to_counts(escape_distance_rad, index))
                )
                if target_counts > initial_counts:
                    escape_target_counts = min(
                        target_counts, initial_counts + escape_delta_counts
                    )
                else:
                    escape_target_counts = max(
                        target_counts, initial_counts - escape_delta_counts
                    )

                print(
                    f"{JOINT_NAMES[index]} first performs a controlled "
                    f"{escape_distance_rad:.4f} rad limit-release step",
                    flush=True,
                )
                slave.sdo_write(
                    TARGET_POSITION, 0x00, pack_s32(escape_target_counts)
                )
                self.start_motion(
                    slave,
                    index,
                    acknowledge_timeout_s=acknowledge_timeout_s,
                    allow_internal_limit_escape=True,
                )
                self.wait_for_profile_target_reached(
                    slave,
                    index,
                    escape_target_counts,
                    timeout_s,
                    tolerance_rad=tolerance_rad,
                    allow_internal_limit_escape=True,
                )

                initial_counts = self.get_actual_position_counts(slave)
                initial_rad = self.counts_to_radians(initial_counts, index)
                if abs(target_rad - initial_rad) <= tolerance_rad:
                    print(
                        f"{JOINT_NAMES[index]} reached pick-ready during the "
                        "limit-release step",
                        flush=True,
                    )
                    continue

            slave.sdo_write(TARGET_POSITION, 0x00, pack_s32(target_counts))
            self.start_motion(
                slave,
                index,
                acknowledge_timeout_s=acknowledge_timeout_s,
            )
            self.wait_for_profile_target_reached(
                slave,
                index,
                target_counts,
                timeout_s,
                tolerance_rad=tolerance_rad,
            )

        moved_names = ", ".join(JOINT_NAMES[index] for index in selected_indices)
        print(
            "Selected joints are within tolerance of the requested Profile Position "
            f"target: {moved_names}",
            flush=True,
        )

    def _homing_section(self) -> dict[str, Any]:
        return self.homing_config.get("homing", {}) or {}

    def _joint_homing_config(self, joint_index: int) -> dict[str, Any]:
        homing = self._homing_section()
        joints = homing.get("joints", {}) or {}
        name = JOINT_NAMES[joint_index]
        cfg = joints.get(name)
        if cfg is None:
            raise RuntimeError(f"No homing configuration found for {name}")
        return cfg

    def _home_offset_steps(self, cfg: dict[str, Any]) -> int:
        """Return the signed CiA-402 home offset in drive position steps.

        ``home_offset_steps`` is the user-facing calibration setting. It is
        written one-to-one to object 0x607C before homing starts; no radians or
        degrees conversion is applied. ``offset_counts`` remains accepted only
        for backward compatibility with older configurations.
        """
        new_value = cfg.get("home_offset_steps")
        legacy_value = cfg.get("offset_counts")

        if new_value is not None and legacy_value is not None:
            if int(new_value) != int(legacy_value):
                raise RuntimeError(
                    "Conflicting homing offsets: home_offset_steps="
                    f"{new_value}, offset_counts={legacy_value}"
                )

        raw_value = (
            new_value
            if new_value is not None
            else legacy_value
            if legacy_value is not None
            else 0
        )

        if isinstance(raw_value, bool):
            raise RuntimeError("home_offset_steps must be a signed integer")

        try:
            value = int(raw_value)
        except (TypeError, ValueError) as exc:
            raise RuntimeError(
                f"home_offset_steps must be a signed integer, got {raw_value!r}"
            ) from exc

        # Reject fractional YAML values instead of silently truncating them.
        try:
            if float(raw_value) != float(value):
                raise RuntimeError(
                    f"home_offset_steps must be a whole number, got {raw_value!r}"
                )
        except (TypeError, ValueError) as exc:
            raise RuntimeError(
                f"home_offset_steps must be a signed integer, got {raw_value!r}"
            ) from exc

        if not -(2**31) <= value <= 2**31 - 1:
            raise RuntimeError(
                f"home_offset_steps={value} does not fit in the signed 32-bit "
                "CiA-402 object 0x607C"
            )
        return value

    def _validate_homed_position(
        self,
        joint_index: int,
        cfg: dict[str, Any],
        actual_counts: int,
        context: str,
    ) -> None:
        """Check the post-homing coordinate against the configured offset."""
        expected_counts = self._home_offset_steps(cfg)
        error_counts = int(actual_counts) - expected_counts
        error_rad = self.counts_to_radians(error_counts, joint_index)
        tolerance_rad = float(cfg.get("home_zero_tolerance_rad", 0.02))

        if abs(error_rad) > tolerance_rad:
            actual_rad = self.counts_to_radians(actual_counts, joint_index)
            expected_rad = self.counts_to_radians(expected_counts, joint_index)
            raise RuntimeError(
                f"{JOINT_NAMES[joint_index]} {context}: "
                f"actual={actual_counts} steps ({actual_rad:.6f} rad), "
                f"expected home offset={expected_counts} steps "
                f"({expected_rad:.6f} rad), error={error_counts} steps "
                f"({error_rad:.6f} rad), tolerance={tolerance_rad:.6f} rad"
            )

    def configure_homing_for_joint(self, slave, joint_index: int, cfg: dict[str, Any]) -> None:
        """Configure the FAULHABER homing objects for one joint."""
        method = cfg.get("method")
        if method is None:
            raise RuntimeError(
                f"Homing method is missing for {JOINT_NAMES[joint_index]}. "
                "Set it in config/homing.yaml after confirming switch direction."
            )

        reference_switch_input = cfg.get("reference_switch_input")
        switch_type = cfg.get("switch_type", "home")

        if switch_type == "home":
            if reference_switch_input is None:
                raise RuntimeError(
                    f"reference_switch_input is missing for {JOINT_NAMES[joint_index]}"
                )
            slave.sdo_write(REFERENCE_SWITCH_INPUT, 0x04, pack_u8(reference_switch_input))
        else:
            raise RuntimeError(
                "Only switch_type='home' is implemented in this bridge. "
                "Limit-switch and mechanical-stop homing require explicit review first."
            )

        # Optional digital input polarity/filter masks. For A/B/C custom homing these
        # are written as a combined mask before entering this function.
        if not bool(cfg.get("_skip_digital_input_mask_config", False)):
            active_low = cfg.get("active_low")
            filter_enabled = cfg.get("filter_enabled", False)
            if active_low is not None or filter_enabled is not None:
                self.configure_input_polarity_and_filters(
                    slave,
                    [(int(reference_switch_input), bool(active_low), bool(filter_enabled))],
                )

        home_offset_steps = self._home_offset_steps(cfg)

        slave.sdo_write(HOMING_METHOD, 0x00, pack_s8(int(method)))
        # CiA-402 0x607C is configured before the homing start edge. The value
        # is deliberately accepted in raw drive steps so the physical
        # calibration can be fine-tuned without an implicit unit conversion.
        slave.sdo_write(HOMING_OFFSET, 0x00, pack_s32(home_offset_steps))
        slave.sdo_write(HOMING_SPEED, 0x01, pack_u32(int(cfg.get("seek_velocity", 100))))
        slave.sdo_write(HOMING_SPEED, 0x02, pack_u32(int(cfg.get("homing_velocity", 50))))
        slave.sdo_write(HOMING_ACCELERATION, 0x00, pack_u32(int(cfg.get("acceleration", 50))))

        # Optional homing torque limits. Useful especially for mechanical-stop homing,
        # but kept configurable for safety even with switch homing.
        if cfg.get("positive_torque_limit") is not None:
            slave.sdo_write(POSITIVE_TORQUE_LIMIT_HOMING, 0x00, pack_u16(int(cfg["positive_torque_limit"])))
        if cfg.get("negative_torque_limit") is not None:
            slave.sdo_write(NEGATIVE_TORQUE_LIMIT_HOMING, 0x00, pack_u16(int(cfg["negative_torque_limit"])))
        if cfg.get("limit_check_delay_ms") is not None:
            slave.sdo_write(LIMIT_CHECK_DELAY_TIME, 0x02, pack_u16(int(cfg["limit_check_delay_ms"])))

        print(
            f"Configured homing for {JOINT_NAMES[joint_index]}: "
            f"method={method}, ref_input={reference_switch_input}, "
            f"home_offset_steps={home_offset_steps}",
            flush=True,
        )

    def wait_for_homing_finished(self, slave, joint_index: int, timeout_s: float) -> None:
        start_time = time.monotonic()

        while True:
            status = self.get_status_word(slave)
            target_reached = bool(status & (1 << 10))
            homing_attained = bool(status & (1 << 12))
            homing_error = bool(status & (1 << 13))

            if homing_error:
                raise RuntimeError(
                    f"Homing error on {JOINT_NAMES[joint_index]}. Status: 0x{status:04X}"
                )

            if homing_attained and target_reached:
                actual_counts = self.get_actual_position_counts(slave)
                actual_rad = self.counts_to_radians(actual_counts, joint_index)
                print(
                    f"Homing completed for {JOINT_NAMES[joint_index]}: "
                    f"status=0x{status:04X}, actual={actual_counts} counts ({actual_rad:.4f} rad)",
                    flush=True,
                )
                return

            if time.monotonic() - start_time > timeout_s:
                raise TimeoutError(
                    f"Timeout during homing of {JOINT_NAMES[joint_index]}. "
                    f"Last status: 0x{status:04X}"
                )

            time.sleep(0.05)

    def home_joint(self, joint_index: int) -> None:
        if joint_index < 0 or joint_index >= EXPECTED_SLAVE_COUNT:
            raise ValueError(f"Invalid joint index: {joint_index}")

        homing = self._homing_section()
        cfg = self._joint_homing_config(joint_index)
        if not bool(cfg.get("enabled", True)):
            print(f"Skipping homing for disabled joint: {JOINT_NAMES[joint_index]}", flush=True)
            return

        timeout_s = float(cfg.get("timeout_s", homing.get("timeout_s", 45.0)))
        slave = self.slaves[joint_index]

        strategy = str(cfg.get("strategy", "faulhaber_home_switch"))
        if strategy == "negative_limit_then_reference_switch":
            self.home_joint_via_negative_limit_then_reference(joint_index, cfg)
            return
        if strategy == "positive_limit_then_reference_switch":
            self.home_joint_via_positive_limit_then_reference(joint_index, cfg)
            return
        if strategy == "direct_reference_switch":
            self.home_joint_direct_to_reference(joint_index, cfg)
            return
        if strategy == "toward_zero_single_switch":
            self.home_end_effector_toward_zero(joint_index, cfg)
            return

        print(f"Starting homing for {JOINT_NAMES[joint_index]}", flush=True)

        self.enable_drive(slave, joint_index, HOMING_MODE)
        self.configure_homing_for_joint(slave, joint_index, cfg)

        # Reset homing start bit, then create the required rising edge on bit 4.
        self.set_control_word(slave, CONTROL_WORD_ENABLE_OPERATION)
        time.sleep(0.05)
        self.set_control_word(slave, CONTROL_WORD_START_HOMING)

        self.wait_for_homing_finished(slave, joint_index, timeout_s)

        # Reset bit 4 so another reference run can be started later if needed.
        self.set_control_word(slave, CONTROL_WORD_ENABLE_OPERATION)
        time.sleep(0.05)

    def home_all_joints(self) -> set[str]:
        homing = self._homing_section()
        if not bool(homing.get("enabled", False)):
            print("Homing requested, but homing.enabled=false. Nothing executed.", flush=True)
            return set()

        name_to_index = {name: index for index, name in enumerate(JOINT_NAMES)}
        sequence = homing.get("sequence")

        if sequence:
            for step_number, step in enumerate(sequence, start=1):
                if not isinstance(step, dict):
                    raise RuntimeError(
                        f"homing.sequence step {step_number} must be a mapping"
                    )
                action = str(step.get("action", "home"))
                joint_name = step.get("joint")
                if joint_name not in name_to_index:
                    raise RuntimeError(
                        f"Unknown joint in homing.sequence step {step_number}: "
                        f"{joint_name}"
                    )
                joint_index = name_to_index[joint_name]
                cfg = self._joint_homing_config(joint_index)
                if not bool(cfg.get("enabled", True)):
                    print(
                        f"Skipping disabled homing sequence step: "
                        f"{action} {joint_name}",
                        flush=True,
                    )
                    continue

                print(
                    f"Homing sequence step {step_number}/{len(sequence)}: "
                    f"{action} {joint_name}",
                    flush=True,
                )
                if action == "home":
                    self.home_joint(joint_index)
                elif action == "park_at_a":
                    self.park_joint_at_a(joint_index, cfg)
                elif action == "park_at_negative_boundary":
                    self.park_joint_at_negative_boundary(joint_index, cfg)
                elif action == "home_from_a_to_b":
                    self.home_joint_from_a_to_b(joint_index, cfg)
                elif action == "home_direct_to_b":
                    self.home_joint_direct_to_reference(joint_index, cfg)
                else:
                    raise RuntimeError(
                        f"Unknown homing.sequence action: {action}"
                    )
        else:
            order = homing.get("order", JOINT_NAMES)
            for joint_name in order:
                if joint_name not in name_to_index:
                    raise RuntimeError(f"Unknown joint in homing.order: {joint_name}")
                self.home_joint(name_to_index[joint_name])

        validated_references = self.validate_homing_references()
        print("All configured homing procedures completed", flush=True)

        if bool(homing.get("move_to_pick_ready_after_homing", False)):
            print(
                "WARNING: homing.move_to_pick_ready_after_homing is deprecated "
                "and ignored. Select startup.mode=home_then_pick_ready instead.",
                flush=True,
            )

        return validated_references

    def move_to_pick_ready(self, include_end_effector: bool = True) -> None:
        homing = self._homing_section()
        pose = homing.get("pick_ready_pose", {}) or {}
        missing = [name for name in JOINT_NAMES if name not in pose]
        if missing:
            raise RuntimeError(f"pick_ready_pose is missing joints: {missing}")

        joint_positions = [float(pose[name]) for name in JOINT_NAMES]
        velocity = int(homing.get("pick_ready_velocity", 80))
        acceleration = int(homing.get("pick_ready_acceleration", 20))
        timeout_s = float(
            homing.get("pick_ready_timeout_s", homing.get("timeout_s", 90.0))
        )
        tolerance_rad = float(homing.get("pick_ready_tolerance_rad", 0.01))
        acknowledge_timeout_s = float(
            homing.get("pick_ready_acknowledge_timeout_s", 2.0)
        )
        joint_indices = list(range(EXPECTED_SLAVE_COUNT))
        if not include_end_effector:
            joint_indices = list(range(EXPECTED_SLAVE_COUNT - 1))
            print(
                "End-effector reference is not validated; the pick-ready move will "
                "command only shoulder, upper arm, and lower arm. The gripper drive "
                "remains at its measured position.",
                flush=True,
            )
        print(
            f"Moving selected joints to pick_ready_pose: {joint_positions}, "
            f"velocity={velocity}, acceleration={acceleration}, "
            f"tolerance={tolerance_rad}",
            flush=True,
        )
        self.move_joints(
            joint_positions,
            velocity=velocity,
            acceleration=acceleration,
            timeout_s=timeout_s,
            tolerance_rad=tolerance_rad,
            acknowledge_timeout_s=acknowledge_timeout_s,
            joint_indices=joint_indices,
        )

    def close(self) -> None:
        self.master.close()
        print("EtherCAT master closed", flush=True)


def read_command_file(path: str) -> str | None:
    try:
        with open(path, "r", encoding="utf-8") as file:
            command = file.read().strip()
        os.remove(path)
        return command
    except FileNotFoundError:
        return None


def parse_move_command(command: str) -> list[float]:
    parts = command.split()
    if len(parts) != EXPECTED_SLAVE_COUNT + 1:
        raise ValueError(
            f"MOVE_RAD requires {EXPECTED_SLAVE_COUNT} values. Received: {command}"
        )
    return [float(value) for value in parts[1:]]


def execute_command(controller: RasclRobotController, command: str) -> None:
    parts = command.split()
    if not parts:
        return

    verb = parts[0]

    if verb == "MOVE_RAD":
        joint_positions = parse_move_command(command)
        print(f"Received command: {command}", flush=True)
        controller.move_joints(joint_positions)
        return

    if verb in {"CSP_RAD", "CSP_HOLD"}:
        raise RuntimeError(
            "CSP through SDO/command-file transport has been removed. "
            "Use hardware_bridge.py, which streams CSP by PDO."
        )

    if verb == "HOME_ALL":
        controller.home_all_joints()
        return

    if verb == "HOME_JOINT":
        if len(parts) != 2:
            raise ValueError("HOME_JOINT requires one 0-based joint index")
        controller.home_joint(int(parts[1]))
        return

    if verb == "PARK_AT_A":
        if len(parts) != 2:
            raise ValueError("PARK_AT_A requires one 0-based joint index")
        controller.park_joint_at_a(int(parts[1]))
        return

    if verb == "PARK_AT_NEGATIVE_BOUNDARY":
        if len(parts) != 2:
            raise ValueError(
                "PARK_AT_NEGATIVE_BOUNDARY requires one 0-based joint index"
            )
        controller.park_joint_at_negative_boundary(int(parts[1]))
        return

    if verb == "HOME_FROM_A_TO_B":
        if len(parts) != 2:
            raise ValueError("HOME_FROM_A_TO_B requires one 0-based joint index")
        controller.home_joint_from_a_to_b(int(parts[1]))
        return

    if verb == "HOME_DIRECT_TO_B":
        if len(parts) != 2:
            raise ValueError("HOME_DIRECT_TO_B requires one 0-based joint index")
        controller.home_joint_direct_to_reference(int(parts[1]))
        return

    if verb == "MOVE_PICK_READY":
        controller.move_to_pick_ready()
        return

    if verb == "READ_INPUTS":
        if len(parts) != 2:
            raise ValueError("READ_INPUTS requires one 0-based joint index")
        controller.print_digital_inputs(int(parts[1]))
        return

    raise ValueError(f"Unknown command: {command}")


def main() -> None:
    print("Starting PySOEM RASCL robot bridge", flush=True)
    print(f"Interface: {INTERFACE_NAME}", flush=True)
    print(f"Command file: {COMMAND_FILE}", flush=True)

    try:
        os.remove(COMMAND_FILE)
        print("Removed stale command file before startup", flush=True)
    except FileNotFoundError:
        pass

    homing_config = load_homing_config()
    controller = RasclRobotController(INTERFACE_NAME, homing_config=homing_config)

    try:
        controller.enable_all_drives(PROFILE_POSITION_MODE)

        homing = controller._homing_section()
        if bool(homing.get("enabled", False)) and bool(homing.get("run_on_startup", False)):
            controller.home_all_joints()

        while True:
            command = read_command_file(COMMAND_FILE)

            if command is None or command == "":
                time.sleep(POLL_PERIOD)
                continue

            try:
                execute_command(controller, command)
            except Exception as exc:
                print(f"Command failed: {command}", flush=True)
                print(f"Error: {exc}", flush=True)

            time.sleep(POLL_PERIOD)

    except KeyboardInterrupt:
        print("Bridge interrupted", flush=True)

    finally:
        # Do not automatically move to zero on shutdown. That was not homing and can
        # be unsafe after a partially completed WP3 task. Stop by disabling voltage.
        controller.disable_all_drives()
        controller.close()


if __name__ == "__main__":
    main()