from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .task2_kinematics import ARM_JOINTS, ALL_JOINTS, Task2Kinematics
from .trajectory_loader import JointTrajectory, TrajectorySample


@dataclass(frozen=True)
class JointLimits:
    joint_names: list[str]
    lower: np.ndarray
    upper: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray


@dataclass(frozen=True)
class TrajectoryMetrics:
    max_velocity: np.ndarray
    max_acceleration: np.ndarray
    required_time_scale: float


def load_joint_limits(path: str | Path) -> JointLimits:
    limit_path = Path(path)
    with limit_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"Robot limits YAML root must be a mapping: {limit_path}")

    joint_names = list(data.get("joint_names", []))
    if joint_names != ALL_JOINTS:
        raise ValueError(
            f"robot_limits.yaml joint order {joint_names} does not match {ALL_JOINTS}"
        )

    position = data.get("position_limits", {}) or {}
    velocity = data.get("velocity_limits", {}) or {}
    acceleration = data.get("acceleration_limits", {}) or {}

    lower: list[float] = []
    upper: list[float] = []
    velocity_values: list[float] = []
    acceleration_values: list[float] = []
    for name in joint_names:
        if name not in position or name not in velocity or name not in acceleration:
            raise ValueError(f"robot_limits.yaml is missing limits for {name}")
        pair = np.asarray(position[name], dtype=float)
        if pair.shape != (2,) or not np.all(np.isfinite(pair)) or pair[0] >= pair[1]:
            raise ValueError(f"Invalid position limit for {name}: {position[name]}")
        speed = float(velocity[name])
        accel = float(acceleration[name])
        if not math.isfinite(speed) or speed <= 0.0:
            raise ValueError(f"Invalid velocity limit for {name}")
        if not math.isfinite(accel) or accel <= 0.0:
            raise ValueError(f"Invalid acceleration limit for {name}")
        lower.append(float(pair[0]))
        upper.append(float(pair[1]))
        velocity_values.append(speed)
        acceleration_values.append(accel)

    return JointLimits(
        joint_names=joint_names,
        lower=np.asarray(lower, dtype=float),
        upper=np.asarray(upper, dtype=float),
        velocity=np.asarray(velocity_values, dtype=float),
        acceleration=np.asarray(acceleration_values, dtype=float),
    )


def min_jerk_scalar(tau: np.ndarray | float) -> np.ndarray | float:
    return 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5


def _sample_times(duration_s: float, sample_period_s: float) -> tuple[np.ndarray, np.ndarray]:
    if duration_s <= 0.0 or sample_period_s <= 0.0:
        raise ValueError("Trajectory duration and sample period must be positive")
    count = max(2, int(np.ceil(duration_s / sample_period_s)) + 1)
    times = np.linspace(0.0, duration_s, count)
    return times, np.asarray(min_jerk_scalar(times / duration_s), dtype=float)


def _sample(
    time_s: float,
    positions: np.ndarray,
    segment: str,
    gripper_marker: float | None,
) -> TrajectorySample:
    q = np.asarray(positions, dtype=float)
    if q.shape != (len(ALL_JOINTS),) or not np.all(np.isfinite(q)):
        raise ValueError(f"Invalid Task 2 joint sample in segment {segment}")
    return TrajectorySample(
        time_from_start=float(time_s),
        positions=[float(value) for value in q],
        gripper=gripper_marker,
        segment=segment,
    )


def joint_segment(
    *,
    q0: np.ndarray,
    q1: np.ndarray,
    duration_s: float,
    sample_period_s: float,
    start_time_s: float,
    name: str,
    gripper_marker: float | None,
) -> list[TrajectorySample]:
    start = np.asarray(q0, dtype=float)
    end = np.asarray(q1, dtype=float)
    if start.shape != (len(ALL_JOINTS),) or end.shape != (len(ALL_JOINTS),):
        raise ValueError("Task 2 joint segment endpoints must contain four values")
    local_times, blends = _sample_times(duration_s, sample_period_s)
    return [
        _sample(
            start_time_s + float(local_time),
            start + float(blend) * (end - start),
            name,
            gripper_marker,
        )
        for local_time, blend in zip(local_times, blends)
    ]


def cartesian_segment(
    *,
    kinematics: Task2Kinematics,
    q0: np.ndarray,
    q1: np.ndarray,
    p0_m: np.ndarray,
    p1_m: np.ndarray,
    duration_s: float,
    sample_period_s: float,
    start_time_s: float,
    name: str,
    gripper_marker: float | None,
    tolerance_m: float,
    maximum_joint_step_rad: float,
    random_seed: int,
) -> list[TrajectorySample]:
    start = np.asarray(q0, dtype=float)
    end = np.asarray(q1, dtype=float)
    p0 = np.asarray(p0_m, dtype=float)
    p1 = np.asarray(p1_m, dtype=float)
    if start.shape != (len(ALL_JOINTS),) or end.shape != (len(ALL_JOINTS),):
        raise ValueError("Task 2 Cartesian segment endpoints must contain four joints")
    if p0.shape != (3,) or p1.shape != (3,):
        raise ValueError("Task 2 Cartesian segment endpoints must contain XYZ")

    local_times, blends = _sample_times(duration_s, sample_period_s)
    result: list[TrajectorySample] = []
    previous = start[: len(ARM_JOINTS)].copy()

    for index, (local_time, blend) in enumerate(zip(local_times, blends)):
        blend_value = float(blend)
        if index == 0:
            q_arm = start[: len(ARM_JOINTS)].copy()
        elif index == len(local_times) - 1:
            q_arm = end[: len(ARM_JOINTS)].copy()
        else:
            target = p0 + blend_value * (p1 - p0)
            nominal = start[: len(ARM_JOINTS)] + blend_value * (
                end[: len(ARM_JOINTS)] - start[: len(ARM_JOINTS)]
            )
            q_arm = kinematics.continuous_sample(
                target_m=target,
                previous=previous,
                nominal=nominal,
                tolerance_m=tolerance_m,
                random_seed=random_seed + index,
            )

        if index > 0:
            maximum_step = float(np.max(np.abs(q_arm - previous)))
            if maximum_step > maximum_joint_step_rad:
                raise RuntimeError(
                    f"Cartesian IK branch jump in {name}: {maximum_step:.6f} rad "
                    f"> {maximum_joint_step_rad:.6f} rad"
                )
        previous = q_arm

        gripper_q = start[3] + blend_value * (end[3] - start[3])
        q = np.concatenate([q_arm, np.asarray([gripper_q], dtype=float)])
        result.append(
            _sample(
                start_time_s + float(local_time),
                q,
                name,
                gripper_marker,
            )
        )
    return result


def append_segment(
    destination: list[TrajectorySample],
    segment: list[TrajectorySample],
) -> None:
    if not segment:
        raise ValueError("Cannot append an empty trajectory segment")
    if destination:
        segment = segment[1:]
    destination.extend(segment)


def scale_trajectory_time(
    trajectory: JointTrajectory,
    scale: float,
) -> JointTrajectory:
    if not math.isfinite(scale) or scale < 1.0:
        raise ValueError("Trajectory time scale must be finite and at least one")
    return JointTrajectory(
        joint_names=list(trajectory.joint_names),
        samples=[
            TrajectorySample(
                time_from_start=float(sample.time_from_start * scale),
                positions=list(sample.positions),
                gripper=sample.gripper,
                segment=sample.segment,
            )
            for sample in trajectory.samples
        ],
        metadata=dict(trajectory.metadata),
    )


def resample_trajectory(
    trajectory: JointTrajectory,
    publish_rate_hz: float,
) -> JointTrajectory:
    if publish_rate_hz <= 0.0:
        raise ValueError("publish_rate_hz must be positive")
    if not trajectory.samples:
        raise ValueError("Cannot resample an empty trajectory")

    step = 1.0 / publish_rate_hz
    duration = trajectory.duration
    result: list[TrajectorySample] = []
    source_index = 0
    stream_index = 0

    while True:
        target_time = min(stream_index * step, duration)
        while (
            source_index + 1 < len(trajectory.samples)
            and trajectory.samples[source_index + 1].time_from_start
            < target_time - 1e-12
        ):
            source_index += 1

        if source_index + 1 >= len(trajectory.samples):
            source = trajectory.samples[-1]
            result.append(
                TrajectorySample(
                    time_from_start=duration,
                    positions=list(source.positions),
                    gripper=source.gripper,
                    segment=source.segment,
                )
            )
        else:
            left = trajectory.samples[source_index]
            right = trajectory.samples[source_index + 1]
            interval = right.time_from_start - left.time_from_start
            if interval <= 0.0:
                raise ValueError("Trajectory timestamps must be strictly increasing")
            alpha = min(max((target_time - left.time_from_start) / interval, 0.0), 1.0)
            result.append(
                TrajectorySample(
                    time_from_start=float(target_time),
                    positions=[
                        left.positions[index]
                        + alpha * (right.positions[index] - left.positions[index])
                        for index in range(len(left.positions))
                    ],
                    gripper=left.gripper if alpha < 1.0 else right.gripper,
                    segment=left.segment if alpha < 1.0 else right.segment,
                )
            )

        if target_time >= duration - 1e-12:
            break
        stream_index += 1

    return JointTrajectory(
        joint_names=list(trajectory.joint_names),
        samples=result,
        metadata=dict(trajectory.metadata),
    )


def trajectory_metrics(
    trajectory: JointTrajectory,
    limits: JointLimits,
) -> TrajectoryMetrics:
    if trajectory.joint_names != limits.joint_names:
        raise ValueError("Trajectory joint order does not match robot limits")
    if len(trajectory.samples) < 2:
        raise ValueError("Task 2 trajectory must contain at least two samples")

    positions = np.asarray(
        [sample.positions for sample in trajectory.samples],
        dtype=float,
    )
    times = np.asarray(
        [sample.time_from_start for sample in trajectory.samples],
        dtype=float,
    )
    if positions.shape[1] != len(limits.joint_names) or not np.all(np.isfinite(positions)):
        raise ValueError("Task 2 trajectory contains invalid joint positions")
    if not np.all(np.isfinite(times)) or abs(times[0]) > 1e-12:
        raise ValueError("Task 2 trajectory must start at t=0 with finite times")

    deltas = np.diff(times)
    if np.any(deltas <= 0.0):
        raise ValueError("Task 2 trajectory timestamps must be strictly increasing")
    if np.any(positions < limits.lower - 1e-9) or np.any(
        positions > limits.upper + 1e-9
    ):
        violations: list[str] = []
        for sample_index, q in enumerate(positions):
            for joint_index, name in enumerate(limits.joint_names):
                if q[joint_index] < limits.lower[joint_index] - 1e-9 or q[
                    joint_index
                ] > limits.upper[joint_index] + 1e-9:
                    violations.append(
                        f"{name}={q[joint_index]:.6f} rad at "
                        f"t={times[sample_index]:.3f}s"
                    )
        raise RuntimeError(
            "Task 2 trajectory violates position limits: " + "; ".join(violations[:8])
        )

    velocities = np.diff(positions, axis=0) / deltas[:, None]
    max_velocity = np.max(np.abs(velocities), axis=0)

    if len(velocities) >= 2:
        acceleration_dt = 0.5 * (deltas[:-1] + deltas[1:])
        accelerations = np.diff(velocities, axis=0) / acceleration_dt[:, None]
        max_acceleration = np.max(np.abs(accelerations), axis=0)
    else:
        max_acceleration = np.zeros(len(limits.joint_names), dtype=float)

    velocity_scale = float(np.max(max_velocity / limits.velocity))
    acceleration_scale = float(
        np.max(np.sqrt(max_acceleration / limits.acceleration))
    )
    required_scale = max(1.0, velocity_scale, acceleration_scale)
    return TrajectoryMetrics(
        max_velocity=max_velocity,
        max_acceleration=max_acceleration,
        required_time_scale=required_scale,
    )


def validate_and_time_scale(
    trajectory: JointTrajectory,
    limits: JointLimits,
    publish_rate_hz: float,
) -> tuple[JointTrajectory, TrajectoryMetrics, float]:
    """Resample at the controller rate and stretch time only when limits require it."""
    scale_total = 1.0
    candidate = trajectory
    metrics: TrajectoryMetrics | None = None

    for _ in range(4):
        streamed = resample_trajectory(candidate, publish_rate_hz)
        metrics = trajectory_metrics(streamed, limits)
        if metrics.required_time_scale <= 1.0 + 1e-9:
            return streamed, metrics, scale_total
        scale = metrics.required_time_scale
        scale_total *= scale
        candidate = scale_trajectory_time(candidate, scale)

    assert metrics is not None
    streamed = resample_trajectory(candidate, publish_rate_hz)
    metrics = trajectory_metrics(streamed, limits)
    if metrics.required_time_scale > 1.0 + 1e-6:
        raise RuntimeError(
            "Task 2 automatic time scaling could not satisfy velocity/acceleration limits"
        )
    return streamed, metrics, scale_total
