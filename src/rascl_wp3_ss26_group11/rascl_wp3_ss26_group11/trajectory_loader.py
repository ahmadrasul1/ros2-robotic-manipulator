from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class TrajectorySample:
    """One sampled joint-space trajectory point."""

    time_from_start: float
    positions: list[float]
    gripper: float | None = None
    segment: str = ""


@dataclass(frozen=True)
class JointTrajectory:
    """Loaded CSV trajectory and its optional generation metadata."""

    joint_names: list[str]
    samples: list[TrajectorySample]
    metadata: dict[str, str] = field(default_factory=dict)

    @property
    def duration(self) -> float:
        return self.samples[-1].time_from_start if self.samples else 0.0


def _find_column(fieldnames: Iterable[str], *candidates: str) -> str | None:
    fields = list(fieldnames)
    return next((candidate for candidate in candidates if candidate in fields), None)


def _finite_float(raw: str | None, *, field: str, row_number: int) -> float:
    if raw is None or not raw.strip():
        raise ValueError(f"Missing {field!r} at CSV row {row_number}")
    try:
        value = float(raw)
    except ValueError as exc:
        raise ValueError(
            f"Invalid numeric value for {field!r} at CSV row {row_number}: {raw!r}"
        ) from exc
    if not math.isfinite(value):
        raise ValueError(f"Non-finite {field!r} at CSV row {row_number}: {raw!r}")
    return value


def load_joint_trajectory(csv_path: str | Path, joint_names: list[str]) -> JointTrajectory:
    """Load and strictly validate the offline joint-space CSV contract."""
    path = Path(csv_path)
    if not path.is_file():
        raise FileNotFoundError(f"Trajectory file not found: {path}")
    if not joint_names or len(set(joint_names)) != len(joint_names):
        raise ValueError("Configured joint_names must be non-empty and unique")

    samples: list[TrajectorySample] = []
    metadata: dict[str, str] = {}
    with path.open("r", encoding="utf-8", newline="") as handle:
        raw_lines = handle.readlines()

    data_lines: list[str] = []
    for raw_line in raw_lines:
        stripped = raw_line.strip()
        if not stripped:
            continue
        if stripped.startswith("#"):
            payload = stripped[1:].strip()
            if "=" in payload:
                key, value = payload.split("=", 1)
                metadata[key.strip()] = value.strip()
            continue
        data_lines.append(raw_line)

    reader = csv.DictReader(data_lines)
    if reader.fieldnames is None:
        raise ValueError(f"Trajectory file has no header: {path}")

    fieldnames = [field.strip() for field in reader.fieldnames]
    if len(fieldnames) != len(set(fieldnames)):
        raise ValueError(f"Trajectory header contains duplicate columns: {path}")

    time_column = _find_column(fieldnames, "time", "t", "time_from_start")
    if time_column is None:
        raise ValueError(f"Trajectory file must contain a time column: {path}")

    missing = [name for name in joint_names if name not in fieldnames]
    if missing:
        raise ValueError(f"Trajectory file {path} is missing joint columns: {missing}")

    csv_joint_order = [field for field in fieldnames if field in joint_names]
    if csv_joint_order != joint_names:
        raise ValueError(
            f"Trajectory joint-column order {csv_joint_order} does not match "
            f"configured order {joint_names}: {path}"
        )

    for row_number, row in enumerate(reader, start=2):
        t = _finite_float(row.get(time_column), field=time_column, row_number=row_number)
        positions = [
            _finite_float(row.get(name), field=name, row_number=row_number)
            for name in joint_names
        ]
        gripper = None
        if row.get("gripper", "") not in ("", None):
            gripper = _finite_float(
                row.get("gripper"), field="gripper", row_number=row_number
            )

        samples.append(
            TrajectorySample(
                time_from_start=t,
                positions=positions,
                gripper=gripper,
                segment=(row.get("segment") or "").strip(),
            )
        )
    if not samples:
        raise ValueError(f"Trajectory file contains no samples: {path}")
    if abs(samples[0].time_from_start) > 1e-9:
        raise ValueError(
            f"Trajectory must start at time 0.0, got {samples[0].time_from_start}: {path}"
        )

    previous_time = samples[0].time_from_start
    for index, sample in enumerate(samples[1:], start=3):
        if sample.time_from_start <= previous_time:
            raise ValueError(
                f"Trajectory timestamps must be strictly increasing; row {index} "
                f"has {sample.time_from_start} after {previous_time}: {path}"
            )
        previous_time = sample.time_from_start

    return JointTrajectory(
        joint_names=list(joint_names),
        samples=samples,
        metadata=metadata,
    )
