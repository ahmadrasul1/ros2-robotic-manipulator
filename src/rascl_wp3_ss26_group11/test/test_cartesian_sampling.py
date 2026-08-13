from __future__ import annotations

import importlib.util
import sys
import types
import unittest
from pathlib import Path

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
MODULE_PATH = SCRIPTS / "generate_min_jerk_task1.py"


class _Pose:
    def __init__(self, translation):
        self.t = np.asarray(translation, dtype=float)


class _IdentityPositionRobot:
    def fkine(self, q):
        return _Pose(q)


def _call_ik(_robot, target, _seed, _random_seed):
    return np.asarray(target, dtype=float), True, 0.0


_stub = types.ModuleType("generate_task1_waypoints_rtb")
_stub.ARM_JOINTS = ["shoulder_joint", "upperarm_joint", "lowerarm_joint"]
_stub.build_robot_from_urdf = lambda _path: (_IdentityPositionRobot(), np.array([[-1.0] * 3, [1.0] * 3]))
_stub.call_ik = _call_ik
_stub.solve_position = lambda **_kwargs: None
_previous = sys.modules.get("generate_task1_waypoints_rtb")
sys.modules["generate_task1_waypoints_rtb"] = _stub
try:
    spec = importlib.util.spec_from_file_location("generate_min_jerk_task1_test", MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not import {MODULE_PATH}")
    min_jerk = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(min_jerk)
finally:
    if _previous is None:
        sys.modules.pop("generate_task1_waypoints_rtb", None)
    else:
        sys.modules["generate_task1_waypoints_rtb"] = _previous


class CartesianSamplingTests(unittest.TestCase):
    def test_dense_cartesian_sampler_calls_continuous_ik_contract(self):
        q0 = np.array([0.0, 0.0, 0.0, 0.0])
        q1 = np.array([0.03, 0.04, 0.05, 0.0])
        rows = min_jerk.interpolate_cartesian_segment(
            robot=_IdentityPositionRobot(),
            qlim=np.array([[-1.0] * 3, [1.0] * 3]),
            q0=q0,
            q1=q1,
            p0=q0[:3],
            p1=q1[:3],
            duration=0.1,
            dt=0.05,
            start_time=0.0,
            segment_name="test_cartesian",
            gripper_marker=0.0,
            tolerance_m=1e-9,
            random_seed=7,
        )
        self.assertEqual(len(rows), 3)
        self.assertTrue(
            np.allclose(
                [rows[1][name] for name in min_jerk.ARM_JOINTS],
                0.5 * q1[:3],
                atol=1e-12,
            )
        )


if __name__ == "__main__":
    unittest.main()
