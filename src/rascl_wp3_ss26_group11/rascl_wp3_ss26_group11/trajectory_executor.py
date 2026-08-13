from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .trajectory_loader import JointTrajectory, TrajectorySample


@dataclass(frozen=True)
class ExecutionConfig:
    mode: str
    joint_names: list[str]
    controller_topic: str
    joint_states_topic: str
    publish_rate_hz: float
    hold_last_sample: bool = True


class TrajectoryExecutor:
    """Non-blocking, timer-driven trajectory streamer.

    The old implementation slept for the complete trajectory inside a ROS timer
    callback. That blocked the single-threaded executor for roughly 223 seconds,
    delayed shutdown, and prevented other callbacks from running. This version
    publishes one fixed-rate sample per ROS timer callback and returns immediately.
    """

    def __init__(
        self,
        node,
        config: ExecutionConfig,
        on_gripper_command: Callable[[float], None] | None = None,
    ) -> None:
        if config.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        if not config.joint_names:
            raise ValueError("joint_names must not be empty")

        self._node = node
        self._config = config
        self._on_gripper_command = on_gripper_command
        self._command_pub = None
        self._joint_state_pub = None

        if config.mode == "controller":
            self._command_pub = node.create_publisher(
                Float64MultiArray, config.controller_topic, 10
            )
        elif config.mode == "joint_states":
            self._joint_state_pub = node.create_publisher(
                JointState, config.joint_states_topic, 10
            )
        else:
            raise ValueError("execution mode must be 'controller' or 'joint_states'")

        self._stream_samples: list[TrajectorySample] = []
        self._next_index = 0
        self._running = False
        self._finished = False
        self._held_sample: TrajectorySample | None = None
        self._previous_gripper: float | None = None
        self._wall_start: float | None = None
        self._timer = node.create_timer(
            1.0 / config.publish_rate_hz,
            self._on_timer,
        )

    @property
    def running(self) -> bool:
        return self._running

    @property
    def finished(self) -> bool:
        return self._finished

    @property
    def controller_subscription_count(self) -> int:
        if self._command_pub is None:
            return 0
        return int(self._command_pub.get_subscription_count())

    def hold_sample(self, sample: TrajectorySample) -> None:
        """Publish and retain a stationary sample before/after execution."""
        self._held_sample = sample
        self._publish(sample)

    def start(self, trajectory: JointTrajectory) -> None:
        """Arm execution and return immediately; the ROS timer performs streaming."""
        if self._running:
            raise RuntimeError("Trajectory execution is already running")
        if trajectory.joint_names != self._config.joint_names:
            raise ValueError(
                f"Trajectory joints {trajectory.joint_names} do not match configured "
                f"joints {self._config.joint_names}"
            )

        self._stream_samples = self._resample_trajectory(trajectory)
        if not self._stream_samples:
            raise ValueError("Trajectory has no streamable samples")

        self._next_index = 0
        self._finished = False
        self._running = True
        self._previous_gripper = None
        self._wall_start = time.monotonic()
        self._node.get_logger().info(
            f"Starting non-blocking trajectory: {len(trajectory.samples)} CSV samples, "
            f"{len(self._stream_samples)} commands at {self._config.publish_rate_hz:.1f} Hz, "
            f"duration {trajectory.duration:.3f}s, mode={self._config.mode}"
        )

        # Publish t=0 immediately. Subsequent samples are one-per-timer-cycle.
        self._publish_stream_sample(self._stream_samples[0])
        self._next_index = 1
        if len(self._stream_samples) == 1:
            self._finish()

    # Backward-compatible name used by older code; now intentionally non-blocking.
    execute = start

    def _on_timer(self) -> None:
        if not self._running:
            if (
                self._config.mode == "joint_states"
                and self._config.hold_last_sample
                and self._held_sample is not None
            ):
                self._publish_joint_state(self._held_sample)
            return

        if self._next_index >= len(self._stream_samples):
            self._finish()
            return

        sample = self._stream_samples[self._next_index]
        self._publish_stream_sample(sample)
        self._next_index += 1

        if self._next_index >= len(self._stream_samples):
            self._finish()

    def _finish(self) -> None:
        self._running = False
        self._finished = True
        if self._stream_samples:
            self._held_sample = self._stream_samples[-1]
        elapsed = (
            time.monotonic() - self._wall_start if self._wall_start is not None else 0.0
        )
        self._node.get_logger().info(
            f"Trajectory execution finished (wall time {elapsed:.3f}s)"
        )

    def _publish_stream_sample(self, sample: TrajectorySample) -> None:
        self._held_sample = sample
        self._publish(sample)
        if sample.gripper is not None and sample.gripper != self._previous_gripper:
            self._previous_gripper = sample.gripper
            if self._on_gripper_command is not None:
                self._on_gripper_command(sample.gripper)
            else:
                self._node.get_logger().info(
                    f"Gripper semantic marker: {sample.gripper:.3f}"
                )

    def _publish(self, sample: TrajectorySample) -> None:
        if self._config.mode == "controller":
            self._publish_controller_command(sample)
        else:
            self._publish_joint_state(sample)

    def _resample_trajectory(self, trajectory: JointTrajectory) -> list[TrajectorySample]:
        if not trajectory.samples:
            return []

        step = 1.0 / self._config.publish_rate_hz
        duration = trajectory.duration
        result: list[TrajectorySample] = []
        source_index = 0
        stream_index = 0

        while True:
            time_from_start = min(stream_index * step, duration)
            while (
                source_index + 1 < len(trajectory.samples)
                and trajectory.samples[source_index + 1].time_from_start
                < time_from_start - 1e-12
            ):
                source_index += 1

            if source_index + 1 >= len(trajectory.samples):
                source = trajectory.samples[-1]
                sample = TrajectorySample(
                    time_from_start=duration,
                    positions=list(source.positions),
                    gripper=source.gripper,
                    segment=source.segment,
                )
            else:
                left = trajectory.samples[source_index]
                right = trajectory.samples[source_index + 1]
                interval = right.time_from_start - left.time_from_start
                alpha = min(
                    max((time_from_start - left.time_from_start) / interval, 0.0),
                    1.0,
                )
                sample = TrajectorySample(
                    time_from_start=time_from_start,
                    positions=[
                        left.positions[index]
                        + alpha * (right.positions[index] - left.positions[index])
                        for index in range(len(left.positions))
                    ],
                    gripper=left.gripper if alpha < 1.0 else right.gripper,
                    segment=left.segment if alpha < 1.0 else right.segment,
                )

            result.append(sample)
            if time_from_start >= duration - 1e-12:
                break
            stream_index += 1

        return result

    def _publish_controller_command(self, sample: TrajectorySample) -> None:
        assert self._command_pub is not None
        msg = Float64MultiArray()
        msg.data = list(sample.positions)
        self._command_pub.publish(msg)

    def _publish_joint_state(self, sample: TrajectorySample) -> None:
        assert self._joint_state_pub is not None
        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = list(self._config.joint_names)
        msg.position = list(sample.positions)
        self._joint_state_pub.publish(msg)
