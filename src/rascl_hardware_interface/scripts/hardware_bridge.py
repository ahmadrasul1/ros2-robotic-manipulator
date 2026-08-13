#!/usr/bin/env python3

"""Run the non-cyclic startup phase, then hand one open master to PDO CSP."""

from __future__ import annotations

import copy
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from pdo_csp_bridge import PdoConfig, PdoCspBridge, load_pdo_config
from pysoem_bridge import (
    PROFILE_POSITION_MODE,
    RasclRobotController,
    load_homing_config,
)


@dataclass(frozen=True)
class StartupResult:
    """Facts proven by the startup phase, not inferred from its mode name."""

    master: Any
    arm_reference_valid: bool
    gripper_reference_valid: bool
    reference_scope: str


def _homing_config_with_drive_pick_ready(
    homing_config: dict[str, Any],
    config: PdoConfig,
) -> dict[str, Any]:
    """Return a copy whose post-homing pick-ready pose is in drive coordinates.

    homing.yaml stores pick_ready_pose in ROS/URDF model coordinates.  Homing
    itself still assigns the physical reference switches to drive coordinate zero.
    After homing, the Profile Position pick-ready move must therefore apply the
    same model->drive signs and zero offsets that CSP uses later:

        q_drive = drive_rad_at_model_zero + model_to_drive_sign * q_model

    Only the post-homing pick-ready values are transformed.  Search directions,
    switches, method 37, and every homing parameter remain untouched.
    """

    mapped = copy.deepcopy(homing_config)
    homing = mapped.get("homing", {}) or {}
    pose = homing.get("pick_ready_pose", {}) or {}

    joint_names = (
        "shoulder_joint",
        "upperarm_joint",
        "lowerarm_joint",
        "end_effector_joint",
    )
    missing = [name for name in joint_names if name not in pose]
    if missing:
        raise RuntimeError(f"pick_ready_pose is missing joints: {missing}")

    model_pose = [float(pose[name]) for name in joint_names]
    drive_pose = [
        config.drive_rad_at_model_zero[index]
        + config.model_to_drive_sign[index] * model_pose[index]
        for index in range(len(joint_names))
    ]

    homing["pick_ready_pose"] = {
        name: drive_pose[index] for index, name in enumerate(joint_names)
    }
    mapped["homing"] = homing

    model_text = ", ".join(
        f"{name}={model_pose[index]:+.6f}"
        for index, name in enumerate(joint_names)
    )
    drive_text = ", ".join(
        f"{name}={drive_pose[index]:+.6f}"
        for index, name in enumerate(joint_names)
    )
    print(
        "Post-homing pick-ready mapping: "
        f"model=[{model_text}] -> drive=[{drive_text}]",
        flush=True,
    )
    return mapped


def run_startup_phase(config: PdoConfig) -> StartupResult:
    """Run the selected startup mode and retain the initialized PySOEM master."""
    raw_homing_config = load_homing_config()
    homing_config = _homing_config_with_drive_pick_ready(
        raw_homing_config, config
    )
    homing = homing_config.get("homing", {}) or {}
    mode = config.startup_mode

    controller = RasclRobotController(
        config.interface,
        homing_config=homing_config,
    )
    arm_reference_valid = False
    gripper_reference_valid = False
    reference_scope = "none"

    try:
        if mode in {"home_then_csp", "home_then_pick_ready", "pick_ready_only"}:
            controller.enable_all_drives(PROFILE_POSITION_MODE)

        if mode in {"home_then_csp", "home_then_pick_ready"}:
            if not bool(homing.get("enabled", False)):
                raise RuntimeError(f"startup.mode={mode} requires homing.enabled=true")

            print("Running unified four-axis homing", flush=True)
            validated_references = controller.home_all_joints()
            # home_all_joints() returns only after every configured reference has
            # passed its strategy-specific final switch/zero verification.
            required_arm_references = {
                "shoulder_joint",
                "upperarm_joint",
                "lowerarm_joint",
            }
            arm_reference_valid = required_arm_references.issubset(
                validated_references
            )
            gripper_reference_valid = (
                "end_effector_joint" in validated_references
            )
            reference_labels = (
                ("shoulder_joint", "shoulder"),
                ("upperarm_joint", "upperarm"),
                ("lowerarm_joint", "lowerarm"),
                ("end_effector_joint", "end_effector"),
            )
            reference_scope = "_".join(
                label
                for joint_name, label in reference_labels
                if joint_name in validated_references
            ) or "none"

            if not arm_reference_valid:
                raise RuntimeError("Arm homing returned without all arm references")
            if not gripper_reference_valid:
                raise RuntimeError(
                    "End-effector homing returned without a validated reference"
                )

            if mode == "home_then_pick_ready":
                print(
                    "Homing verified; moving explicitly to the Task 1 pick-ready "
                    "pose in Profile Position before entering CSP",
                    flush=True,
                )
                controller.move_to_pick_ready(
                    include_end_effector=gripper_reference_valid
                )
            else:
                print(
                    "Homing verified; entering CSP while holding the measured "
                    "homed pose (no pick-ready move)",
                    flush=True,
                )

        elif mode == "pick_ready_only":
            print(
                "WARNING: homing is skipped. Moving to pick-ready using the "
                "existing drive reference; arm_reference_valid remains false.",
                flush=True,
            )
            controller.move_to_pick_ready(include_end_effector=False)

        elif mode == "hold_current":
            print(
                "Homing and startup motion are skipped. CSP will hold measured "
                "positions; arm_reference_valid remains false.",
                flush=True,
            )

        else:  # load_pdo_config validates this too.
            raise RuntimeError(f"Unsupported startup mode: {mode}")

    except Exception:
        controller.disable_all_drives()
        controller.close()
        raise

    print(
        f"Startup complete (mode={mode}); reusing the open EtherCAT master for PDO",
        flush=True,
    )
    return StartupResult(
        master=controller.master,
        arm_reference_valid=arm_reference_valid,
        gripper_reference_valid=gripper_reference_valid,
        reference_scope=reference_scope,
    )


def _write_startup_failure(config: PdoConfig, exc: BaseException) -> None:
    """Publish a non-ready failure marker for this launch run."""
    path = Path(config.ready_file)
    temporary_path = path.with_name(f"{path.name}.{os.getpid()}.failed.tmp")
    reason = str(exc).replace("\n", " ").strip() or type(exc).__name__
    temporary_path.write_text(
        "PDO_FAILED\n"
        f"run_id={os.environ.get('RASCL_BRIDGE_RUN_ID', '')}\n"
        f"reason={reason}\n",
        encoding="utf-8",
    )
    os.replace(temporary_path, path)
    print(f"PDO startup failure file created: {path}", flush=True)


def main() -> None:
    config = load_pdo_config()
    Path(config.ready_file).unlink(missing_ok=True)
    print(
        "Starting RASCL hardware bridge: "
        f"startup_mode={config.startup_mode} -> PDO CSP",
        flush=True,
    )

    try:
        startup = run_startup_phase(config)
        bridge = PdoCspBridge(
            config,
            master=startup.master,
            master_already_initialized=True,
            arm_reference_valid=startup.arm_reference_valid,
            gripper_reference_valid=startup.gripper_reference_valid,
            reference_scope=startup.reference_scope,
        )
        bridge.run()
    except Exception as exc:
        _write_startup_failure(config, exc)
        raise


if __name__ == "__main__":
    main()
