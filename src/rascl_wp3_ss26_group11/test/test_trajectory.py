from __future__ import annotations

import math
import sys
import tempfile
import types
import unittest
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))


class _Header:
    def __init__(self):
        self.stamp = None


class JointState:
    def __init__(self):
        self.header = _Header()
        self.name = []
        self.position = []


class Float64MultiArray:
    def __init__(self):
        self.data = []


sensor_msgs = types.ModuleType("sensor_msgs")
sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
sensor_msgs_msg.JointState = JointState
sensor_msgs.msg = sensor_msgs_msg
std_msgs = types.ModuleType("std_msgs")
std_msgs_msg = types.ModuleType("std_msgs.msg")
std_msgs_msg.Float64MultiArray = Float64MultiArray
std_msgs.msg = std_msgs_msg
sys.modules.setdefault("sensor_msgs", sensor_msgs)
sys.modules.setdefault("sensor_msgs.msg", sensor_msgs_msg)
sys.modules.setdefault("std_msgs", std_msgs)
sys.modules.setdefault("std_msgs.msg", std_msgs_msg)

from rascl_wp3_ss26_group11.trajectory_executor import (  # noqa: E402
    ExecutionConfig,
    TrajectoryExecutor,
)
from rascl_wp3_ss26_group11.trajectory_loader import (  # noqa: E402
    JointTrajectory,
    TrajectorySample,
    load_joint_trajectory,
)


JOINTS = ["j1", "j2", "j3", "j4"]


class FakePublisher:
    def __init__(self, subscriptions=1):
        self.messages = []
        self.subscriptions = subscriptions

    def publish(self, message):
        self.messages.append(message)

    def get_subscription_count(self):
        return self.subscriptions


class FakeTimer:
    def __init__(self, callback):
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(("info", message))

    def warning(self, message):
        self.messages.append(("warning", message))


class FakeNow:
    def to_msg(self):
        return object()


class FakeClock:
    def now(self):
        return FakeNow()


class FakeNode:
    def __init__(self):
        self.publisher = None
        self.timer = None
        self.logger = FakeLogger()

    def create_publisher(self, _msg_type, _topic, _queue):
        self.publisher = FakePublisher()
        return self.publisher

    def create_timer(self, _period, callback):
        self.timer = FakeTimer(callback)
        return self.timer

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return FakeClock()


class LoaderTests(unittest.TestCase):
    def _write(self, text: str) -> Path:
        handle = tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False)
        handle.write(text)
        handle.close()
        self.addCleanup(lambda: Path(handle.name).unlink(missing_ok=True))
        return Path(handle.name)

    def test_loads_strict_valid_csv(self):
        path = self._write(
            "time,j1,j2,j3,j4,gripper,segment\n"
            "0,0,0,0,0,0,start\n"
            "0.01,1,2,3,4,1,end\n"
        )
        trajectory = load_joint_trajectory(path, JOINTS)
        self.assertEqual(len(trajectory.samples), 2)
        self.assertAlmostEqual(trajectory.duration, 0.01)

    def test_rejects_equal_timestamps(self):
        path = self._write(
            "time,j1,j2,j3,j4\n0,0,0,0,0\n0,1,1,1,1\n"
        )
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            load_joint_trajectory(path, JOINTS)

    def test_rejects_non_finite_values(self):
        path = self._write(
            "time,j1,j2,j3,j4\n0,0,0,nan,0\n"
        )
        with self.assertRaisesRegex(ValueError, "Non-finite"):
            load_joint_trajectory(path, JOINTS)

    def test_rejects_joint_order_conflict(self):
        path = self._write(
            "time,j2,j1,j3,j4\n0,0,0,0,0\n"
        )
        with self.assertRaisesRegex(ValueError, "joint-column order"):
            load_joint_trajectory(path, JOINTS)


class ExecutorTests(unittest.TestCase):
    def test_timer_driven_controller_stream(self):
        node = FakeNode()
        executor = TrajectoryExecutor(
            node,
            ExecutionConfig(
                mode="controller",
                joint_names=JOINTS,
                controller_topic="/commands",
                joint_states_topic="/joint_states",
                publish_rate_hz=100.0,
            ),
        )
        trajectory = JointTrajectory(
            JOINTS,
            [
                TrajectorySample(0.0, [0.0] * 4, gripper=0.0),
                TrajectorySample(0.02, [1.0] * 4, gripper=1.0),
            ],
        )

        executor.start(trajectory)
        self.assertTrue(executor.running)
        self.assertEqual(len(node.publisher.messages), 1)  # t=0 is immediate
        node.timer.callback()
        node.timer.callback()
        self.assertTrue(executor.finished)
        self.assertEqual(len(node.publisher.messages), 3)
        self.assertEqual(node.publisher.messages[-1].data, [1.0] * 4)
        self.assertEqual(executor.controller_subscription_count, 1)

    def test_resampling_preserves_finite_positions(self):
        node = FakeNode()
        executor = TrajectoryExecutor(
            node,
            ExecutionConfig(
                mode="joint_states",
                joint_names=JOINTS,
                controller_topic="/commands",
                joint_states_topic="/joint_states",
                publish_rate_hz=100.0,
            ),
        )
        trajectory = JointTrajectory(
            JOINTS,
            [
                TrajectorySample(0.0, [0.0, 1.0, 2.0, 3.0]),
                TrajectorySample(0.015, [1.0, 2.0, 3.0, 4.0]),
            ],
        )
        samples = executor._resample_trajectory(trajectory)
        self.assertAlmostEqual(samples[-1].time_from_start, 0.015)
        self.assertTrue(all(math.isfinite(v) for s in samples for v in s.positions))


if __name__ == "__main__":
    unittest.main()
