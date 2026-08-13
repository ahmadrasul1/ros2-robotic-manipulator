#!/usr/bin/env python3
"""Sample Task 1 with minimum-jerk timing.

Transfer and gripper-only segments remain joint-space minimum-jerk motions.
Segments tagged ``interpolation: cartesian_linear`` are sampled as straight TCP
lines and solved continuously with Robotics Toolbox. This prevents a nominal
vertical descend/lift from bowing sideways merely because joint coordinates were
interpolated independently.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml

# Reuse exactly the same URDF parser and RTB conventions as endpoint generation.
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from generate_task1_waypoints_rtb import (  # noqa: E402
    ARM_JOINTS,
    build_robot_from_urdf,
    call_ik,
    solve_position,
)


JOINT_NAMES = [*ARM_JOINTS, "end_effector_joint"]


def min_jerk_scalar(tau: np.ndarray) -> np.ndarray:
    """Minimum-jerk blend: s(0)=0, s(1)=1, zero velocity/acceleration at ends."""
    return 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5


def sample_times(duration: float, dt: float) -> tuple[np.ndarray, np.ndarray]:
    if duration <= 0.0:
        raise ValueError("Segment duration must be positive")
    if dt <= 0.0:
        raise ValueError("dt must be positive")
    n_steps = max(2, int(np.ceil(duration / dt)) + 1)
    local_times = np.linspace(0.0, duration, n_steps)
    return local_times, min_jerk_scalar(local_times / duration)


def make_row(
    q: np.ndarray,
    time_value: float,
    segment_name: str,
    gripper_marker: float | None,
) -> dict[str, Any]:
    row: dict[str, Any] = {
        "time": float(time_value),
        "segment": segment_name,
    }
    for joint_name, value in zip(JOINT_NAMES, q):
        row[joint_name] = float(value)
    row["gripper"] = "" if gripper_marker is None else float(gripper_marker)
    return row


def interpolate_joint_segment(
    q0: np.ndarray,
    q1: np.ndarray,
    duration: float,
    dt: float,
    start_time: float,
    segment_name: str,
    gripper_marker: float | None,
) -> list[dict[str, Any]]:
    local_times, blend_values = sample_times(duration, dt)
    return [
        make_row(
            q0 + blend * (q1 - q0),
            start_time + float(t_local),
            segment_name,
            gripper_marker,
        )
        for t_local, blend in zip(local_times, blend_values)
    ]


def cartesian_key(waypoint_name: str) -> str:
    for suffix in ("_open", "_hold"):
        if waypoint_name.endswith(suffix):
            return waypoint_name[: -len(suffix)]
    raise ValueError(
        f"Cartesian segment waypoint {waypoint_name!r} has no _open/_hold suffix"
    )


def _continuous_ik_sample(
    robot,
    qlim: np.ndarray,
    target: np.ndarray,
    previous: np.ndarray,
    nominal: np.ndarray,
    tolerance_m: float,
    random_seed: int,
) -> np.ndarray:
    """Solve one dense Cartesian sample without changing IK branches."""

    candidates: list[np.ndarray] = []
    for seed in (previous, nominal):
        try:
            q, _success, _residual = call_ik(robot, target, seed, random_seed)
        except (ValueError, np.linalg.LinAlgError):
            continue
        if q.shape != (len(ARM_JOINTS),) or not np.all(np.isfinite(q)):
            continue
        if np.any(q < qlim[0] - 1e-9) or np.any(q > qlim[1] + 1e-9):
            continue
        actual = np.asarray(robot.fkine(q).t, dtype=float).reshape(3)
        if float(np.linalg.norm(actual - target)) <= tolerance_m:
            candidates.append(q)

    if not candidates:
        fallback = solve_position(
            robot=robot,
            qlim=qlim,
            target=target,
            preferred=nominal,
            previous=previous,
            tolerance_m=tolerance_m,
            random_seed=random_seed,
        )
        if fallback is None:
            raise RuntimeError(
                "Continuous Cartesian IK failed for target "
                f"[{target[0]:.6f}, {target[1]:.6f}, {target[2]:.6f}] m"
            )
        candidates.append(fallback.q)

    return min(candidates, key=lambda q: float(np.linalg.norm(q - previous)))


def interpolate_cartesian_segment(
    *,
    robot,
    qlim: np.ndarray,
    q0: np.ndarray,
    q1: np.ndarray,
    p0: np.ndarray,
    p1: np.ndarray,
    duration: float,
    dt: float,
    start_time: float,
    segment_name: str,
    gripper_marker: float | None,
    tolerance_m: float,
    random_seed: int,
) -> list[dict[str, Any]]:
    local_times, blend_values = sample_times(duration, dt)
    rows: list[dict[str, Any]] = []
    previous = q0[: len(ARM_JOINTS)].copy()

    for sample_index, (t_local, blend) in enumerate(
        zip(local_times, blend_values)
    ):
        if sample_index == 0:
            q_arm = q0[: len(ARM_JOINTS)].copy()
        elif sample_index == len(local_times) - 1:
            q_arm = q1[: len(ARM_JOINTS)].copy()
        else:
            target = p0 + float(blend) * (p1 - p0)
            nominal = q0[: len(ARM_JOINTS)] + float(blend) * (
                q1[: len(ARM_JOINTS)] - q0[: len(ARM_JOINTS)]
            )
            q_arm = _continuous_ik_sample(
                robot,
                qlim,
                target,
                previous,
                nominal,
                tolerance_m,
                random_seed + sample_index,
            )

        # Dense IK must remain on one continuous branch. A large one-sample jump
        # indicates branch switching or a bad model and is rejected offline.
        if sample_index > 0:
            maximum_step = float(np.max(np.abs(q_arm - previous)))
            if maximum_step > 0.08:
                raise RuntimeError(
                    f"Cartesian IK branch jump in {segment_name}: "
                    f"{maximum_step:.6f} rad in one {dt:.3f}s sample"
                )
        previous = q_arm

        gripper_q = q0[3] + float(blend) * (q1[3] - q0[3])
        q = np.concatenate([q_arm, np.array([gripper_q], dtype=float)])
        rows.append(
            make_row(
                q,
                start_time + float(t_local),
                segment_name,
                gripper_marker,
            )
        )

    return rows


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return data


def generate_trajectory(
    data: dict[str, Any],
    trajectory_name: str,
    urdf_path: Path | None,
) -> list[dict[str, Any]]:
    settings = data.get("settings", {}) or {}
    dt = float(settings.get("dt", 0.02))
    tolerance_m = float(settings.get("ik_position_tolerance_m", 0.0005))

    waypoints = data.get("waypoints", {}) or {}
    trajectories = data.get("trajectories", {}) or {}
    cartesian_targets = data.get("cartesian_targets_base_m", {}) or {}
    if trajectory_name not in trajectories:
        raise KeyError(f"Trajectory {trajectory_name!r} not found in input file")

    segment_definitions = trajectories[trajectory_name]
    needs_cartesian = any(
        str(segment.get("interpolation", "joint")) == "cartesian_linear"
        for segment in segment_definitions
    )
    robot = None
    qlim = None
    if needs_cartesian:
        if urdf_path is None:
            raise ValueError("--urdf is required for cartesian_linear segments")
        robot, qlim = build_robot_from_urdf(urdf_path)

    rows: list[dict[str, Any]] = []
    current_time = 0.0

    for segment_index, segment in enumerate(segment_definitions):
        name = str(segment["name"])
        q0_name = str(segment["from"])
        q1_name = str(segment["to"])
        duration = float(segment.get("duration", settings.get("default_duration", 2.0)))
        gripper_marker = segment.get("gripper", None)
        interpolation = str(segment.get("interpolation", "joint"))

        if q0_name not in waypoints or q1_name not in waypoints:
            raise KeyError(f"Unknown waypoint in segment {name}: {q0_name} -> {q1_name}")

        q0 = np.asarray(waypoints[q0_name], dtype=float)
        q1 = np.asarray(waypoints[q1_name], dtype=float)
        if q0.shape != (len(JOINT_NAMES),) or q1.shape != (len(JOINT_NAMES),):
            raise ValueError(f"Waypoints must contain {len(JOINT_NAMES)} joint values")

        if interpolation == "joint":
            segment_rows = interpolate_joint_segment(
                q0,
                q1,
                duration,
                dt,
                current_time,
                name,
                gripper_marker,
            )
        elif interpolation == "cartesian_linear":
            assert robot is not None and qlim is not None
            p0_key = cartesian_key(q0_name)
            p1_key = cartesian_key(q1_name)
            if p0_key not in cartesian_targets or p1_key not in cartesian_targets:
                raise KeyError(
                    f"Cartesian targets missing for {name}: {p0_key} -> {p1_key}"
                )
            p0 = np.asarray(cartesian_targets[p0_key], dtype=float)
            p1 = np.asarray(cartesian_targets[p1_key], dtype=float)
            if p0.shape != (3,) or p1.shape != (3,):
                raise ValueError(f"Cartesian target for {name} must contain x/y/z")
            segment_rows = interpolate_cartesian_segment(
                robot=robot,
                qlim=qlim,
                q0=q0,
                q1=q1,
                p0=p0,
                p1=p1,
                duration=duration,
                dt=dt,
                start_time=current_time,
                segment_name=name,
                gripper_marker=gripper_marker,
                tolerance_m=tolerance_m,
                random_seed=11000 + segment_index * 1000,
            )
        else:
            raise ValueError(f"Unsupported interpolation {interpolation!r} in {name}")

        if rows and segment_rows:
            segment_rows = segment_rows[1:]
        rows.extend(segment_rows)
        current_time += duration

    return rows


def write_csv(
    path: Path,
    rows: list[dict[str, Any]],
    source_data: dict[str, Any],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = ["time", *JOINT_NAMES, "gripper", "segment"]
    metadata = source_data.get("metadata", {}) or {}
    settings = source_data.get("settings", {}) or {}
    location_z = settings.get("location_z_correction_m", {}) or {}

    with path.open("w", encoding="utf-8", newline="") as handle:
        # These comments are ignored by the normal CSV reader but are checked
        # before hardware execution. They prevent an old known-good trajectory
        # from being run after a newer pose/URDF regeneration attempt failed.
        comments = {
            "rascl_task1_format": "2",
            "poses_sha256": str(metadata.get("poses_sha256", "")),
            "urdf_sha256": str(metadata.get("urdf_sha256", "")),
            "gripper_mode": str(metadata.get("gripper_mode", "")),
            "target_z_correction_m": str(settings.get("target_z_correction_m", 0.0)),
            "cube1_start_z_correction_m": str(
                location_z.get("cube1_start", location_z.get("cube_locations", 0.0))
            ),
            "cube2_3_start_z_correction_m": str(
                location_z.get("cube2_3_start", location_z.get("cube_locations", 0.0))
            ),
            "cube3_buffer_z_correction_m": str(
                location_z.get("cube3_buffer", location_z.get("cube_locations", 0.0))
            ),
            "goal_z_correction_m": str(location_z.get("goal", 0.0)),
        }
        for key, value in comments.items():
            handle.write(f"# {key}={value}\n")

        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input",
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / "trajectories"
        / "task1"
        / "input_waypoints_ik_sim.yaml",
    )
    parser.add_argument("--trajectory", default="task1_full")
    parser.add_argument("--urdf", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    data = load_yaml(args.input)
    rows = generate_trajectory(data, args.trajectory, args.urdf)
    output = args.output or args.input.parent / f"{args.trajectory}.csv"
    write_csv(output, rows, data)
    print(f"Wrote {len(rows)} samples to {output}")


if __name__ == "__main__":
    main()
