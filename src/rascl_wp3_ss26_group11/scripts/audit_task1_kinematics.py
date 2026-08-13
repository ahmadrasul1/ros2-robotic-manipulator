#!/usr/bin/env python3
"""Audit the complete Task 1 kinematic and trajectory contract.

This check deliberately does not import Robotics Toolbox. It independently parses
exactly the URDF transforms used by the IK generator, evaluates forward kinematics,
and verifies the generated YAML/CSV files. It therefore catches a trajectory that
is numerically self-consistent with a stale or malformed waypoint file.
"""

from __future__ import annotations

import argparse
import csv
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import yaml

ARM_JOINTS = ["shoulder_joint", "upperarm_joint", "lowerarm_joint"]
ALL_JOINTS = [*ARM_JOINTS, "end_effector_joint"]
TCP_CHAIN = [*ARM_JOINTS, "gripper_tcp_joint"]
CARTESIAN_TOLERANCE_M = 0.0005
GEOMETRY_TOLERANCE_M = 0.001


@dataclass(frozen=True)
class ChainElement:
    fixed: np.ndarray
    axis: np.ndarray | None


class AuditFailure(RuntimeError):
    pass


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise AuditFailure(f"YAML root must be a mapping: {path}")
    return data


def parse_vector(text: str | None, length: int) -> np.ndarray:
    values = [float(value) for value in (text or " ".join(["0"] * length)).split()]
    if len(values) != length:
        raise AuditFailure(f"Expected {length} values, received {values}")
    return np.asarray(values, dtype=float)


def rpy_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    normalized = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(normalized))
    if norm <= 1e-12:
        raise AuditFailure("A revolute joint axis is zero")
    normalized /= norm
    x, y, z = normalized
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return (
        np.eye(3)
        + math.sin(angle) * skew
        + (1.0 - math.cos(angle)) * (skew @ skew)
    )


def homogeneous(rotation: np.ndarray, translation: Iterable[float]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(list(translation), dtype=float)
    return transform


class UrdfPositionModel:
    def __init__(self, urdf_path: Path):
        self.urdf_path = urdf_path
        self.root = ET.parse(urdf_path).getroot()
        self.joints = {
            joint.attrib["name"]: joint for joint in self.root.findall("joint")
        }
        self.chain: list[ChainElement] = []
        self.position_limits: list[tuple[float, float]] = []

        for joint_name in TCP_CHAIN:
            joint = self.joints.get(joint_name)
            if joint is None:
                raise AuditFailure(f"URDF is missing {joint_name}")
            origin = joint.find("origin")
            xyz = parse_vector(origin.attrib.get("xyz") if origin is not None else None, 3)
            rpy = parse_vector(origin.attrib.get("rpy") if origin is not None else None, 3)
            fixed = homogeneous(rpy_rotation(*rpy), xyz)

            joint_type = joint.attrib.get("type", "")
            if joint_type == "revolute":
                axis_element = joint.find("axis")
                axis = parse_vector(
                    axis_element.attrib.get("xyz") if axis_element is not None else "1 0 0",
                    3,
                )
                limit = joint.find("limit")
                if limit is None:
                    raise AuditFailure(f"URDF joint {joint_name} has no limits")
                self.position_limits.append(
                    (float(limit.attrib["lower"]), float(limit.attrib["upper"]))
                )
                self.chain.append(ChainElement(fixed=fixed, axis=axis))
            elif joint_type == "fixed":
                self.chain.append(ChainElement(fixed=fixed, axis=None))
            else:
                raise AuditFailure(
                    f"Unexpected joint type {joint_type!r} in TCP chain at {joint_name}"
                )

    def origin_xyz(self, joint_name: str) -> np.ndarray:
        joint = self.joints[joint_name]
        origin = joint.find("origin")
        return parse_vector(origin.attrib.get("xyz") if origin is not None else None, 3)

    def visual_origin_xyz(self, link_name: str) -> np.ndarray:
        link = self.root.find(f".//link[@name='{link_name}']")
        if link is None:
            raise AuditFailure(f"URDF is missing link {link_name}")
        origin = link.find("visual/origin")
        return parse_vector(origin.attrib.get("xyz") if origin is not None else None, 3)

    def fk(self, q_arm: Iterable[float]) -> np.ndarray:
        q = np.asarray(list(q_arm), dtype=float)
        if q.shape != (len(ARM_JOINTS),):
            raise AuditFailure(f"Expected three arm coordinates, received {q.tolist()}")
        transform = np.eye(4)
        q_index = 0
        for element in self.chain:
            transform = transform @ element.fixed
            if element.axis is not None:
                transform = transform @ homogeneous(
                    axis_rotation(element.axis, float(q[q_index])),
                    (0.0, 0.0, 0.0),
                )
                q_index += 1
        return transform[:3, 3].copy()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(
            row for row in handle if row.strip() and not row.lstrip().startswith("#")
        )
        if reader.fieldnames is None:
            raise AuditFailure(f"CSV has no header: {path}")
        if [name for name in reader.fieldnames if name in ALL_JOINTS] != ALL_JOINTS:
            raise AuditFailure(f"CSV joint order does not match {ALL_JOINTS}: {path}")
        rows = list(reader)
    if not rows:
        raise AuditFailure(f"CSV contains no samples: {path}")
    return rows


def waypoint_base_name(name: str) -> str:
    for suffix in ("_open", "_hold"):
        if name.endswith(suffix):
            return name[: -len(suffix)]
    raise AuditFailure(f"Cartesian waypoint has no _open/_hold suffix: {name}")


def distance_to_line(point: np.ndarray, start: np.ndarray, end: np.ndarray) -> float:
    direction = end - start
    denominator = float(np.dot(direction, direction))
    if denominator <= 1e-18:
        return float(np.linalg.norm(point - start))
    fraction = float(np.dot(point - start, direction) / denominator)
    projection = start + fraction * direction
    return float(np.linalg.norm(point - projection))


def require_close_vector(
    actual: np.ndarray,
    expected: np.ndarray,
    tolerance: float,
    label: str,
) -> None:
    error = float(np.linalg.norm(actual - expected))
    if error > tolerance:
        raise AuditFailure(
            f"{label}: {actual.tolist()} differs from {expected.tolist()} by "
            f"{error * 1000.0:.3f} mm"
        )


def audit(root: Path) -> dict[str, float]:
    description = root / "src/rascl_description"
    wp3 = root / "src/rascl_wp3_ss26_group11"
    urdf_path = description / "urdf/rascl.urdf.xacro"
    poses_path = wp3 / "config/task1_cube_poses.yaml"
    calibration_path = wp3 / "config/kinematics_calibration.yaml"
    sim_waypoints_path = wp3 / "trajectories/task1/input_waypoints_ik_sim.yaml"
    hardware_waypoints_path = wp3 / "trajectories/task1/input_waypoints_ik_hardware.yaml"
    sim_csv_path = wp3 / "trajectories/task1/task1_full_simulation_ik.csv"
    hardware_csv_path = wp3 / "trajectories/task1/task1_full_hardware.csv"

    for path in (
        urdf_path,
        poses_path,
        calibration_path,
        sim_waypoints_path,
        hardware_waypoints_path,
        sim_csv_path,
        hardware_csv_path,
    ):
        if not path.is_file():
            raise AuditFailure(f"Required kinematics file is missing: {path}")

    model = UrdfPositionModel(urdf_path)
    poses = load_yaml(poses_path)
    calibration = load_yaml(calibration_path)
    sim_waypoints = load_yaml(sim_waypoints_path)
    hardware_waypoints = load_yaml(hardware_waypoints_path)

    physical = calibration["physical_geometry"]
    geometry_tolerance = float(
        physical.get("geometry_tolerance_m", GEOMETRY_TOLERANCE_M)
    )
    urdf_reference = calibration.get("urdf_reference", {})

    require_close_vector(
        model.origin_xyz("lowerarm_joint"),
        np.asarray(urdf_reference["lowerarm_joint_origin_xyz_m"], dtype=float),
        geometry_tolerance,
        "lowerarm_joint calibrated URDF translation",
    )
    require_close_vector(
        model.origin_xyz("gripper_tcp_joint"),
        np.asarray(urdf_reference["gripper_tcp_joint_origin_xyz_m"], dtype=float),
        geometry_tolerance,
        "gripper_tcp calibrated URDF translation",
    )
    require_close_vector(
        model.visual_origin_xyz("upperarm"),
        np.asarray(urdf_reference["upperarm_visual_origin_xyz_m"], dtype=float),
        1e-6,
        "upperarm calibrated visual origin",
    )

    if not math.isclose(
        float(poses["planning"]["approach_clearance_m"]),
        0.060,
        abs_tol=1e-12,
    ):
        raise AuditFailure("Task 1 approach clearance is not the confirmed 60 mm")
    if not math.isclose(float(poses["cube"]["height_m"]), 0.040, abs_tol=1e-12):
        raise AuditFailure("Task 1 cube stack height is not 40 mm")

    expected_revision = calibration["calibration_id"]
    for label, data in (("simulation", sim_waypoints), ("hardware", hardware_waypoints)):
        metadata = data.get("metadata", {})
        if metadata.get("gripper_mode") != label:
            raise AuditFailure(f"{label} waypoint YAML has wrong gripper_mode")
        if metadata.get("kinematic_model_revision") != expected_revision:
            raise AuditFailure(
                f"{label} waypoint YAML was not generated from {expected_revision}"
            )

    sim_targets = sim_waypoints["cartesian_targets_base_m"]
    hardware_targets = hardware_waypoints["cartesian_targets_base_m"]
    if sim_targets != hardware_targets:
        raise AuditFailure("Simulation and hardware Cartesian targets differ")

    max_endpoint_error = 0.0
    sim_q = sim_waypoints["waypoints"]
    hardware_q = hardware_waypoints["waypoints"]
    for target_name, raw_target in sim_targets.items():
        target = np.asarray(raw_target, dtype=float)
        for label, waypoint_map in (("simulation", sim_q), ("hardware", hardware_q)):
            q = np.asarray(waypoint_map[f"{target_name}_open"][:3], dtype=float)
            actual = model.fk(q)
            error = float(np.linalg.norm(actual - target))
            max_endpoint_error = max(max_endpoint_error, error)
            if error > CARTESIAN_TOLERANCE_M:
                raise AuditFailure(
                    f"{label} waypoint {target_name} FK error is "
                    f"{error * 1000.0:.3f} mm"
                )
        if not np.allclose(
            np.asarray(sim_q[f"{target_name}_open"][:3], dtype=float),
            np.asarray(hardware_q[f"{target_name}_open"][:3], dtype=float),
            atol=1e-12,
            rtol=0.0,
        ):
            raise AuditFailure(
                f"Simulation/hardware arm waypoint differs at {target_name}"
            )

    trajectory = sim_waypoints["trajectories"]["task1_full"]
    cartesian_names = {
        str(segment["name"])
        for segment in trajectory
        if str(segment.get("interpolation", "joint")) == "cartesian_linear"
    }
    expected_cartesian_names = {
        str(segment["name"])
        for segment in trajectory
        if str(segment["name"]).startswith(("descend", "lift", "retreat"))
    }
    if cartesian_names != expected_cartesian_names:
        raise AuditFailure(
            "Every descend/lift/retreat segment must use cartesian_linear and no "
            "unrelated segment may be tagged"
        )

    sim_rows = read_csv(sim_csv_path)
    hardware_rows = read_csv(hardware_csv_path)
    if len(sim_rows) != len(hardware_rows):
        raise AuditFailure("Simulation and hardware CSV sample counts differ")

    max_arm_csv_difference = 0.0
    for sim_row, hardware_row in zip(sim_rows, hardware_rows):
        if sim_row["time"] != hardware_row["time"] or sim_row["segment"] != hardware_row["segment"]:
            raise AuditFailure("Simulation/hardware CSV time or segment sequence differs")
        for joint in ARM_JOINTS:
            difference = abs(float(sim_row[joint]) - float(hardware_row[joint]))
            max_arm_csv_difference = max(max_arm_csv_difference, difference)
    if max_arm_csv_difference > 1e-12:
        raise AuditFailure(
            f"Simulation/hardware arm CSVs differ by {max_arm_csv_difference:.12g} rad"
        )

    rows_by_segment: dict[str, list[dict[str, str]]] = {}
    for row in sim_rows:
        rows_by_segment.setdefault(row["segment"], []).append(row)

    max_line_deviation = 0.0
    for segment in trajectory:
        if str(segment.get("interpolation", "joint")) != "cartesian_linear":
            continue
        name = str(segment["name"])
        rows = rows_by_segment.get(name, [])
        if len(rows) < 2:
            raise AuditFailure(f"Cartesian segment has fewer than two samples: {name}")
        positions = [
            model.fk([float(row[joint]) for joint in ARM_JOINTS]) for row in rows
        ]
        start = positions[0]
        end = positions[-1]
        deviation = max(distance_to_line(point, start, end) for point in positions)
        max_line_deviation = max(max_line_deviation, deviation)
        if deviation > CARTESIAN_TOLERANCE_M:
            raise AuditFailure(
                f"Cartesian segment {name} deviates {deviation * 1000.0:.3f} mm "
                "from its TCP line"
            )

        source_key = waypoint_base_name(str(segment["from"]))
        destination_key = waypoint_base_name(str(segment["to"]))
        require_close_vector(
            start,
            np.asarray(sim_targets[source_key], dtype=float),
            CARTESIAN_TOLERANCE_M,
            f"{name} source target",
        )
        require_close_vector(
            end,
            np.asarray(sim_targets[destination_key], dtype=float),
            CARTESIAN_TOLERANCE_M,
            f"{name} destination target",
        )


    return {
        "upper_joint_distance_m": float(np.linalg.norm(model.origin_xyz("lowerarm_joint"))),
        "lower_joint_to_tcp_m": float(model.origin_xyz("gripper_tcp_joint")[0]),
        "max_ik_endpoint_error_m": max_endpoint_error,
        "max_cartesian_line_deviation_m": max_line_deviation,
        "max_sim_hardware_arm_difference_rad": max_arm_csv_difference,
        "sample_count": float(len(sim_rows)),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parents[3],
        help="Workspace root containing src/.",
    )
    args = parser.parse_args()

    try:
        metrics = audit(args.root.resolve())
    except (AuditFailure, KeyError, ValueError, OSError, ET.ParseError) as exc:
        print(f"[FAIL] Task 1 kinematics audit: {exc}")
        raise SystemExit(1) from exc

    print("[OK] Task 1 physical-kinematics audit passed")
    print(f"     upper joint-axis distance: {metrics['upper_joint_distance_m'] * 1000.0:.3f} mm")
    print(f"     lower joint to TCP:        {metrics['lower_joint_to_tcp_m'] * 1000.0:.3f} mm")
    print(f"     maximum IK endpoint error: {metrics['max_ik_endpoint_error_m'] * 1000.0:.6f} mm")
    print(
        "     maximum vertical-line deviation: "
        f"{metrics['max_cartesian_line_deviation_m'] * 1000.0:.6f} mm"
    )
    print(
        "     simulation/hardware arm delta: "
        f"{metrics['max_sim_hardware_arm_difference_rad']:.12g} rad"
    )
    print(f"     samples per trajectory:    {int(metrics['sample_count'])}")


if __name__ == "__main__":
    main()
