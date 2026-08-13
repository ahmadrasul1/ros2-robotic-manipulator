#!/usr/bin/env python3
"""Plot joint positions from a generated WP3 CSV trajectory."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


JOINT_NAMES = [
    "shoulder_joint",
    "upperarm_joint",
    "lowerarm_joint",
    "end_effector_joint",
]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path, help="Trajectory CSV file.")
    parser.add_argument("--output", type=Path, default=None, help="Optional PNG output path.")
    args = parser.parse_args()

    times: list[float] = []
    values = {joint: [] for joint in JOINT_NAMES}

    with args.csv.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(
            row for row in handle if row.strip() and not row.lstrip().startswith("#")
        )
        for row in reader:
            times.append(float(row["time"]))
            for joint in JOINT_NAMES:
                values[joint].append(float(row[joint]))

    fig = plt.figure()
    for joint in JOINT_NAMES:
        plt.plot(times, values[joint], label=joint)

    plt.xlabel("time [s]")
    plt.ylabel("joint position [rad]")
    plt.title(args.csv.name)
    plt.legend()
    plt.grid(True)
    fig.tight_layout()

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.output, dpi=150)
        print(f"Saved {args.output}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
