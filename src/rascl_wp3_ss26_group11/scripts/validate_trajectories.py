#!/usr/bin/env python3
"""Strict offline validation for RASCL Task 1 joint-space CSV files."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Any

import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from rascl_wp3_ss26_group11.trajectory_loader import load_joint_trajectory  # noqa: E402


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Expected a YAML mapping: {path}")
    return data


def _validate_limit_sections(limits: dict[str, Any]) -> list[str]:
    joint_names = list(limits.get("joint_names", []))
    if not joint_names or len(set(joint_names)) != len(joint_names):
        raise ValueError("robot_limits.yaml joint_names must be non-empty and unique")
    for section in ("position_limits", "velocity_limits", "acceleration_limits"):
        values = limits.get(section)
        if not isinstance(values, dict):
            raise ValueError(f"robot_limits.yaml is missing {section}")
        missing = [name for name in joint_names if name not in values]
        if missing:
            raise ValueError(f"{section} is missing joints: {missing}")
    return joint_names


def _validate_pick_ready(
    trajectory,
    joint_names: list[str],
    homing: dict[str, Any],
) -> list[str]:
    section = homing.get("homing", {}) or {}
    pose = section.get("pick_ready_pose", {}) or {}
    tolerance = float(section.get("pick_ready_tolerance_rad", 0.01))
    errors: list[str] = []
    missing = [name for name in joint_names if name not in pose]
    if missing:
        return [f"homing pick_ready_pose is missing joints: {missing}"]
    for index, name in enumerate(joint_names):
        expected = float(pose[name])
        actual = trajectory.samples[0].positions[index]
        difference = abs(actual - expected)
        if difference > tolerance + 1e-12:
            errors.append(
                f"first sample {name}={actual:.6f} does not match pick_ready "
                f"{expected:.6f} (error {difference:.6f} > {tolerance:.6f} rad)"
            )
    return errors


def validate_file(
    csv_path: Path,
    limits: dict[str, Any],
    homing: dict[str, Any] | None,
    verbose: bool,
) -> int:
    joint_names = _validate_limit_sections(limits)
    position_limits = limits["position_limits"]
    velocity_limits = limits["velocity_limits"]
    acceleration_limits = limits["acceleration_limits"]

    try:
        trajectory = load_joint_trajectory(csv_path, joint_names)
    except Exception as exc:  # noqa: BLE001 - command-line validator reports all context.
        print(f"[FAIL] {csv_path}: {exc}")
        return 1

    failures: list[str] = []
    max_velocity = {name: 0.0 for name in joint_names}
    max_acceleration = {name: 0.0 for name in joint_names}

    for sample_index, sample in enumerate(trajectory.samples):
        for joint_index, name in enumerate(joint_names):
            value = sample.positions[joint_index]
            lower, upper = [float(v) for v in position_limits[name]]
            if not lower <= value <= upper:
                failures.append(
                    f"t={sample.time_from_start:.6f}: {name}={value:.6f} outside "
                    f"[{lower:.6f}, {upper:.6f}]"
                )

    interval_velocities: list[tuple[float, list[float]]] = []
    for index in range(1, len(trajectory.samples)):
        previous = trajectory.samples[index - 1]
        current = trajectory.samples[index]
        dt = current.time_from_start - previous.time_from_start
        velocities = [
            (current.positions[j] - previous.positions[j]) / dt
            for j in range(len(joint_names))
        ]
        interval_velocities.append((dt, velocities))
        for joint_index, name in enumerate(joint_names):
            speed = abs(velocities[joint_index])
            max_velocity[name] = max(max_velocity[name], speed)
            limit = float(velocity_limits[name])
            if speed > limit + 1e-9:
                failures.append(
                    f"t={current.time_from_start:.6f}: {name} speed {speed:.6f} "
                    f"> {limit:.6f} rad/s"
                )

    for index in range(1, len(interval_velocities)):
        previous_dt, previous_velocity = interval_velocities[index - 1]
        current_dt, current_velocity = interval_velocities[index]
        acceleration_dt = 0.5 * (previous_dt + current_dt)
        sample_time = trajectory.samples[index + 1].time_from_start
        for joint_index, name in enumerate(joint_names):
            acceleration = abs(
                (current_velocity[joint_index] - previous_velocity[joint_index])
                / acceleration_dt
            )
            max_acceleration[name] = max(max_acceleration[name], acceleration)
            limit = float(acceleration_limits[name])
            if acceleration > limit + 1e-8:
                failures.append(
                    f"t={sample_time:.6f}: {name} acceleration {acceleration:.6f} "
                    f"> {limit:.6f} rad/s^2"
                )

    if homing is not None:
        failures.extend(_validate_pick_ready(trajectory, joint_names, homing))

    # Avoid flooding the terminal while preserving the total count.
    for failure in failures[:30]:
        print(f"[FAIL] {csv_path}: {failure}")
    if len(failures) > 30:
        print(f"[FAIL] {csv_path}: {len(failures) - 30} additional violations omitted")

    if failures:
        return len(failures)

    print(
        f"[OK] {csv_path}: {len(trajectory.samples)} samples, "
        f"duration {trajectory.duration:.3f}s"
    )
    if verbose:
        for name in joint_names:
            print(
                f"     {name}: max_speed={max_velocity[name]:.6f} rad/s, "
                f"max_acceleration={max_acceleration[name]:.6f} rad/s^2"
            )
    return 0


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="+", type=Path)
    parser.add_argument(
        "--limits",
        type=Path,
        default=PACKAGE_ROOT / "config" / "robot_limits.yaml",
    )
    parser.add_argument(
        "--homing",
        type=Path,
        default=PACKAGE_ROOT / "config" / "homing.yaml",
        help="Use an empty string to disable first-sample/pick-ready validation.",
    )
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    limits = load_yaml(args.limits)
    homing = load_yaml(args.homing) if args.homing else None
    total_errors = sum(
        validate_file(csv_path, limits, homing, args.verbose)
        for csv_path in args.csv
    )
    raise SystemExit(1 if total_errors else 0)


if __name__ == "__main__":
    main()
