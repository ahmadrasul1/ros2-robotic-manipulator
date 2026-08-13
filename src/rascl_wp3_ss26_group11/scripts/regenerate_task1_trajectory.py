#!/usr/bin/env python3
"""Regenerate and validate the active Task 1 IK trajectory atomically.

This wrapper is used by wp3_prepare_task1.launch.py. Hardware startup and
trajectory execution are intentionally separate and are never started here. It preserves the last known-good generated files
unless all three stages succeed:

1. Cartesian task configuration -> IK waypoint YAML.
2. IK waypoint YAML -> sampled minimum-jerk CSV.
3. CSV validation against robot_limits.yaml.

The output targets are resolved through symlinks before writing. This matters
for ROS 2 workspaces built with ``colcon build --symlink-install``: updating an
installed share-file symlink should update the corresponding source file rather
than replacing the symlink inside ``install/``.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path


def writable_target(path: Path) -> Path:
    """Return the real file that should be updated for an output path."""

    expanded = path.expanduser()
    if expanded.is_symlink():
        return expanded.resolve(strict=False)
    return expanded


def temporary_sibling(target: Path, suffix: str) -> Path:
    """Create a unique temporary filename on the target filesystem."""

    target.parent.mkdir(parents=True, exist_ok=True)
    return target.with_name(f".{target.name}.{os.getpid()}.{suffix}.tmp")


def run_stage(command: list[str], label: str) -> None:
    print(f"\n[trajectory-regeneration] {label}", flush=True)
    print("  " + " ".join(command), flush=True)
    subprocess.run(command, check=True)


def main() -> None:
    script_dir = Path(__file__).resolve().parent

    parser = argparse.ArgumentParser(
        description="Regenerate and validate the generated Task 1 IK CSV."
    )
    parser.add_argument("--poses", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--waypoints-output", type=Path, required=True)
    parser.add_argument("--trajectory-name", default="task1_full")
    parser.add_argument("--csv-output", type=Path, required=True)
    parser.add_argument("--limits", type=Path, required=True)
    parser.add_argument(
        "--gripper-mode",
        choices=["simulation", "hardware"],
        default="simulation",
    )
    args = parser.parse_args()

    waypoint_target = writable_target(args.waypoints_output)
    csv_target = writable_target(args.csv_output)
    waypoint_temp = temporary_sibling(waypoint_target, "waypoints")
    csv_temp = temporary_sibling(csv_target, "trajectory")

    generator = script_dir / "generate_task1_waypoints_rtb.py"
    min_jerk = script_dir / "generate_min_jerk_task1.py"
    validator = script_dir / "validate_trajectories.py"

    for required in (
        args.poses,
        args.urdf,
        args.limits,
        generator,
        min_jerk,
        validator,
    ):
        if not required.exists():
            raise FileNotFoundError(f"Required trajectory input does not exist: {required}")

    try:
        run_stage(
            [
                sys.executable,
                str(generator),
                "--poses",
                str(args.poses),
                "--urdf",
                str(args.urdf),
                "--output",
                str(waypoint_temp),
                "--gripper-mode",
                args.gripper_mode,
            ],
            "Solving inverse kinematics",
        )

        run_stage(
            [
                sys.executable,
                str(min_jerk),
                "--input",
                str(waypoint_temp),
                "--trajectory",
                args.trajectory_name,
                "--urdf",
                str(args.urdf),
                "--output",
                str(csv_temp),
            ],
            "Sampling the minimum-jerk trajectory",
        )

        run_stage(
            [
                sys.executable,
                str(validator),
                str(csv_temp),
                "--limits",
                str(args.limits),
                "--verbose",
            ],
            "Validating the generated trajectory",
        )

        # Both generated files are known-good at this point. Replace each target
        # atomically on its own filesystem.
        os.replace(waypoint_temp, waypoint_target)
        os.replace(csv_temp, csv_target)

        print("\n[trajectory-regeneration] Generation completed successfully.")
        print(f"  Waypoints: {waypoint_target}")
        print(f"  CSV:       {csv_target}")
    finally:
        for temporary in (waypoint_temp, csv_temp):
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as exc:
        print(
            f"\n[trajectory-regeneration] FAILED during command with exit code "
            f"{exc.returncode}.",
            file=sys.stderr,
        )
        raise SystemExit(exc.returncode or 1) from exc
    except Exception as exc:  # noqa: BLE001 - launch must receive a non-zero exit.
        print(f"\n[trajectory-regeneration] FAILED: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
