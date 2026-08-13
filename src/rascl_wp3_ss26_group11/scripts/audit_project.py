#!/usr/bin/env python3
"""Cross-file consistency audit for the Group 11 WP3 repository."""

from __future__ import annotations

import argparse
import csv
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import yaml

JOINTS = [
    "shoulder_joint",
    "upperarm_joint",
    "lowerarm_joint",
    "end_effector_joint",
]


class Audit:
    def __init__(self) -> None:
        self.failures: list[str] = []
        self.notes: list[str] = []

    def require(self, condition: bool, message: str) -> None:
        if not condition:
            self.failures.append(message)

    def equal(self, actual: Any, expected: Any, message: str) -> None:
        if actual != expected:
            self.failures.append(f"{message}: got {actual!r}, expected {expected!r}")

    def close(self, actual: float, expected: float, message: str, tol: float = 1e-9) -> None:
        if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=tol):
            self.failures.append(
                f"{message}: got {actual:.12g}, expected {expected:.12g}"
            )


def read_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Expected mapping in {path}")
    return data


def read_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(
            row for row in handle if row.strip() and not row.lstrip().startswith("#")
        )
        if reader.fieldnames is None:
            raise ValueError(f"No CSV header: {path}")
        return list(reader.fieldnames), list(reader)


def audit_repository(root: Path) -> Audit:
    audit = Audit()
    description = root / "src/rascl_description"
    hardware = root / "src/rascl_hardware_interface"
    wp3 = root / "src/rascl_wp3_ss26_group11"

    urdf_path = description / "urdf/rascl.urdf.xacro"
    limits_path = wp3 / "config/robot_limits.yaml"
    homing_path = wp3 / "config/homing.yaml"
    controllers_path = description / "config/controllers.yaml"
    safe_pdo_path = hardware / "config/ethercat_pdo.yaml"
    homing_pdo_path = hardware / "config/ethercat_pdo_homing.yaml"
    task_pdo_path = hardware / "config/ethercat_pdo_task1.yaml"
    csv_path = wp3 / "trajectories/task1/task1_full_hardware.csv"
    waypoints_path = wp3 / "trajectories/task1/input_waypoints_ik_hardware.yaml"

    required = [
        urdf_path,
        limits_path,
        homing_path,
        controllers_path,
        safe_pdo_path,
        homing_pdo_path,
        task_pdo_path,
        csv_path,
        waypoints_path,
    ]
    for path in required:
        audit.require(path.is_file(), f"Required file missing: {path.relative_to(root)}")
    if audit.failures:
        return audit

    tree = ET.parse(urdf_path)
    robot = tree.getroot()
    urdf_joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    control = robot.find("ros2_control")
    audit.require(control is not None, "URDF has no ros2_control block")
    control_joints = [] if control is None else [j.attrib["name"] for j in control.findall("joint")]
    audit.equal(control_joints, JOINTS, "ros2_control joint order conflict")
    if control is not None:
        audit.require(
            not control.findall(".//param[@name='initial_value']"),
            "URDF contains stale initial_value parameters; feedback must seed commands",
        )

    limits = read_yaml(limits_path)
    audit.equal(list(limits.get("joint_names", [])), JOINTS, "robot_limits joint order")
    for name in JOINTS:
        joint = urdf_joints.get(name)
        audit.require(joint is not None, f"URDF missing {name}")
        if joint is None:
            continue
        limit = joint.find("limit")
        audit.require(limit is not None, f"URDF {name} has no limit")
        if limit is None:
            continue
        expected_min, expected_max = [float(v) for v in limits["position_limits"][name]]
        audit.close(float(limit.attrib["lower"]), expected_min, f"{name} lower limit")
        audit.close(float(limit.attrib["upper"]), expected_max, f"{name} upper limit")
        audit.close(
            float(limit.attrib["velocity"]),
            float(limits["velocity_limits"][name]),
            f"{name} velocity limit",
        )

    rail = urdf_joints.get("rail_finger_joint")
    end = urdf_joints.get("end_effector_joint")
    if rail is not None and end is not None:
        mimic = rail.find("mimic")
        rail_limit = rail.find("limit")
        end_limit = end.find("limit")
        audit.require(mimic is not None, "rail_finger_joint does not mimic end_effector_joint")
        if mimic is not None and rail_limit is not None and end_limit is not None:
            multiplier = float(mimic.attrib.get("multiplier", "1"))
            offset = float(mimic.attrib.get("offset", "0"))
            mapped = [
                multiplier * float(end_limit.attrib[bound]) + offset
                for bound in ("lower", "upper")
            ]
            rail_min = float(rail_limit.attrib["lower"])
            rail_max = float(rail_limit.attrib["upper"])
            audit.require(
                min(mapped) >= rail_min - 1e-12 and max(mapped) <= rail_max + 1e-12,
                f"Mimic range {mapped} exceeds rail limits [{rail_min}, {rail_max}]",
            )

    controllers = read_yaml(controllers_path)
    manager = controllers["controller_manager"]["ros__parameters"]
    update_rate = int(manager["update_rate"])
    configured_joints = controllers["joint_position_controller"]["ros__parameters"]["joints"]
    audit.equal(configured_joints, JOINTS, "controller joint order")

    safe_pdo = read_yaml(safe_pdo_path)
    homing_pdo = read_yaml(homing_pdo_path)
    task_pdo = read_yaml(task_pdo_path)
    for label, pdo in (("safe", safe_pdo), ("homing", homing_pdo), ("task", task_pdo)):
        cycle_us = int(pdo["ethercat"]["cycle_time_us"])
        audit.close(update_rate, 1_000_000.0 / cycle_us, f"{label} PDO/controller rate")
        expected_startup = "hold_current" if label == "safe" else "home_then_pick_ready"
        audit.equal(pdo["startup"]["mode"], expected_startup, f"{label} startup mode")
        audit.equal(
            [float(v) for v in pdo["safety"]["position_min_rad"]],
            [float(limits["position_limits"][name][0]) for name in JOINTS],
            f"{label} PDO lower limits",
        )
        audit.equal(
            [float(v) for v in pdo["safety"]["position_max_rad"]],
            [float(limits["position_limits"][name][1]) for name in JOINTS],
            f"{label} PDO upper limits",
        )
        audit.equal(
            [float(v) for v in pdo["safety"]["max_velocity_rad_s"]],
            [float(limits["velocity_limits"][name]) for name in JOINTS],
            f"{label} PDO velocity limits",
        )
        audit.require(
            not bool(pdo["safety"].get("allow_unhomed_motion", False)),
            f"{label} PDO allows unhomed motion",
        )
    audit.require(
        not bool(safe_pdo["safety"].get("allow_motion", False)),
        "Default PDO config must be hold-only",
    )
    audit.require(
        not bool(homing_pdo["safety"].get("allow_motion", False)),
        "Homing-only PDO config must block CSP trajectory motion",
    )
    audit.require(
        bool(task_pdo["safety"].get("allow_motion", False)),
        "Task PDO config does not enable motion",
    )
    for label, pdo in (("safe", safe_pdo), ("homing", homing_pdo), ("task", task_pdo)):
        audit.require(
            not bool(pdo["safety"].get("gripper_reference_valid", False)),
            f"{label} PDO falsely claims a validated gripper reference",
        )

    hardware_bridge_text = (hardware / "scripts/hardware_bridge.py").read_text(encoding="utf-8")
    pysoem_bridge_text = (hardware / "scripts/pysoem_bridge.py").read_text(encoding="utf-8")
    audit.require(
        "include_end_effector=gripper_reference_valid" in hardware_bridge_text,
        "Startup pick-ready movement is not gated by validated gripper reference",
    )
    audit.require(
        "joint_indices=joint_indices" in pysoem_bridge_text,
        "Profile Position startup cannot restrict movement to referenced axes",
    )
    audit.require(
        "slave.sdo_write(TARGET_POSITION, 0x00, pack_s32(actual_counts))" in pysoem_bridge_text,
        "Profile Position is enabled without synchronizing its retained target",
    )
    audit.require(
        'if mode in {"home_then_csp", "home_then_pick_ready", "pick_ready_only"}'
        in hardware_bridge_text,
        "hold_current still enables Profile Position during startup",
    )

    prepare_launch_text = (wp3 / "launch/wp3_prepare_task1.launch.py").read_text(
        encoding="utf-8"
    )
    homing_launch_text = (wp3 / "launch/wp3_homing_hardware.launch.py").read_text(
        encoding="utf-8"
    )
    task_launch_text = (wp3 / "launch/wp3_tsk1_hardware.launch.py").read_text(
        encoding="utf-8"
    )
    audit.require(
        "regenerate_task1_trajectory.py" in prepare_launch_text,
        "Task 1 preparation launch does not regenerate and validate the trajectory",
    )
    audit.require(
        "ros2_control.launch.py" not in prepare_launch_text
        and 'executable="wp3_tsk1"' not in prepare_launch_text,
        "Task 1 preparation launch still starts hardware or the task player",
    )
    audit.require(
        "ros2_control.launch.py" in homing_launch_text
        and "ethercat_pdo_task1.yaml" in homing_launch_text,
        "Homing launch does not start the persistent homed Task 1 hardware stack",
    )
    audit.require(
        "regenerate_task1_trajectory.py" not in homing_launch_text
        and 'executable="wp3_tsk1"' not in homing_launch_text,
        "Homing launch still generates or executes Task 1",
    )
    audit.require(
        'executable="wp3_tsk1"' in task_launch_text,
        "Task 1 hardware launch does not start the task player",
    )
    audit.require(
        "regenerate_task1_trajectory.py" not in task_launch_text
        and "ros2_control.launch.py" not in task_launch_text
        and "ethercat_pdo_task1.yaml" not in task_launch_text,
        "Task 1 hardware launch still owns generation, EtherCAT, or homing",
    )

    homing = read_yaml(homing_path)["homing"]
    audit.require(bool(homing.get("enabled", False)), "Homing is disabled")
    audit.require(
        not bool(homing.get("move_to_pick_ready_after_homing", False)),
        "home_all_joints still hides a pick-ready move",
    )
    audit.equal(
        list(homing.get("required_reference_joints", [])),
        JOINTS,
        "Required homing-reference joints",
    )

    header, rows = read_csv(csv_path)
    audit.equal([name for name in header if name in JOINTS], JOINTS, "active CSV joint order")
    audit.require(bool(rows), "Active CSV contains no rows")
    if rows:
        audit.close(float(rows[0]["time"]), 0.0, "Active CSV start time")
        for name in JOINTS:
            audit.close(
                float(rows[0][name]),
                float(homing["pick_ready_pose"][name]),
                f"Active CSV/pick-ready mismatch for {name}",
                tol=float(homing.get("pick_ready_tolerance_rad", 0.01)),
            )

        hold_rows = [row for row in rows if row.get("segment") == "final_tower_hold_3_seconds"]
        audit.require(bool(hold_rows), "Active CSV has no final 3-second tower hold segment")
        if hold_rows:
            sample_dt = float(rows[1]["time"]) - float(rows[0]["time"])
            represented_duration = (
                float(hold_rows[-1]["time"]) - float(hold_rows[0]["time"]) + sample_dt
            )
            audit.require(
                represented_duration >= 3.0 - sample_dt - 1e-9,
                f"Tower hold is too short: {represented_duration:.6f}s",
            )

    waypoints = read_yaml(waypoints_path)
    audit.equal(
        waypoints.get("metadata", {}).get("gripper_mode"),
        "hardware",
        "Active waypoint gripper mode",
    )
    trajectories = waypoints.get("trajectories", {})
    trajectory = trajectories.get("task1_full")
    audit.require(isinstance(trajectory, list), "Waypoint YAML lacks the full task sequence")
    if isinstance(trajectory, list):
        names = [segment.get("name") for segment in trajectory]
        expected_order = [
            "transfer_cube3_to_buffer",
            "transfer_cube1_to_goal",
            "transfer_cube2_to_goal",
            "transfer_cube3_to_goal",
            "final_tower_hold_3_seconds",
        ]
        positions = [names.index(name) if name in names else -1 for name in expected_order]
        audit.require(
            all(position >= 0 for position in positions) and positions == sorted(positions),
            "Task sequence does not implement buffer -> cube1 -> cube2 -> cube3 -> hold",
        )

    return audit


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parents[3],
        help="Workspace root containing src/.",
    )
    args = parser.parse_args()
    root = args.root.resolve()
    audit = audit_repository(root)
    if audit.failures:
        for failure in audit.failures:
            print(f"[FAIL] {failure}")
        print(f"Audit failed with {len(audit.failures)} conflict(s).")
        raise SystemExit(1)
    print("[OK] WP3 cross-file audit passed.")


if __name__ == "__main__":
    main()
