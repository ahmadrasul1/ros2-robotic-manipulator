from __future__ import annotations

import sys
import unittest
from copy import deepcopy
from pathlib import Path

import numpy as np
import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from rascl_wp3_ss26_group11.task2_config import load_task2_config  # noqa: E402


class Task2ConfigurationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.task2_path = PACKAGE_ROOT / "config" / "task2_online_planning.yaml"
        cls.task1_path = PACKAGE_ROOT / "config" / "task1_cube_poses.yaml"
        cls.config = load_task2_config(cls.task2_path)
        cls.task2_raw = yaml.safe_load(cls.task2_path.read_text(encoding="utf-8"))
        cls.task1_raw = yaml.safe_load(cls.task1_path.read_text(encoding="utf-8"))

    def test_z_is_computed_from_board_and_half_cube(self):
        expected = self.config.board_surface_z_m + 0.5 * self.config.cube_height_m
        self.assertAlmostEqual(self.config.cube_center_z_m, expected)
        self.assertTrue(self.task2_raw["input"]["ignore_point_z"])

    def test_workspace_placeholder_units_and_values(self):
        self.assertEqual(self.task2_raw["workspace"]["units"], "m")
        self.assertAlmostEqual(
            self.config.workspace.min_radius_m,
            float(self.task2_raw["workspace"]["min_radius_m"]),
        )
        self.assertAlmostEqual(
            self.config.workspace.max_radius_m,
            float(self.task2_raw["workspace"]["max_radius_m"]),
        )
        self.assertEqual(
            self.config.workspace.values_are_placeholders,
            bool(self.task2_raw["workspace"]["values_are_placeholders"]),
        )

    def test_task2_reuses_exact_task1_goal_and_xy_correction(self):
        self.assertEqual(
            self.task2_raw["goal"]["board_xy_m"],
            self.task1_raw["poses_board_xy_m"]["goal"],
        )
        self.assertEqual(
            self.task2_raw["board"]["target_xy_correction_base_m"],
            self.task1_raw["board"]["target_xy_correction_base_m"],
        )
        goal_radius, _ = self.config.validate_workspace_xy(
            self.config.goal_base_xy_m,
            label="Task 2 goal",
        )
        self.assertLess(goal_radius, self.config.workspace.max_radius_m)

    def test_runtime_input_is_board_frame_and_uses_configured_mapping(self):
        self.assertEqual(self.config.input_xy_frame, "board")
        transformed = self.config.board_xy_to_base([0.20, -0.05])

        nominal = np.asarray([-0.05, 0.20], dtype=float)
        yaw = float(self.config.board_to_base_xy.get("yaw_correction_rad", 0.0))
        rotation = np.asarray(
            [
                [np.cos(yaw), -np.sin(yaw)],
                [np.sin(yaw), np.cos(yaw)],
            ],
            dtype=float,
        )
        correction = np.asarray(self.config.xy_correction_base_m, dtype=float)
        expected = rotation @ nominal + correction

        np.testing.assert_allclose(transformed, expected, atol=1e-12)

    def test_yaw_is_applied_after_nominal_axis_mapping(self):
        mapping = deepcopy(self.config.board_to_base_xy)
        mapping["yaw_correction_rad"] = 0.5 * np.pi
        rotated = self.config.__class__(
            **{
                **self.config.__dict__,
                "board_to_base_xy": mapping,
            }
        ).board_xy_to_base([0.20, -0.05])
        np.testing.assert_allclose(
            rotated,
            np.asarray([-0.20, -0.05], dtype=float),
            atol=1e-12,
        )

    def test_task1_and_task2_share_the_same_transform_configuration(self):
        self.assertEqual(
            self.task2_raw["board"]["board_to_base_xy"],
            self.task1_raw["board"]["board_to_base_xy"],
        )


class Task2IsolationAndSequenceTests(unittest.TestCase):
    def test_task2_planner_closes_only_after_cube_center_descent(self):
        source = (
            PACKAGE_ROOT / "rascl_wp3_ss26_group11" / "task2_planner.py"
        ).read_text(encoding="utf-8")
        descend = source.index('name="descend_to_cube_center_open"')
        close = source.index('name="close_gripper_at_cube_center"')
        lift = source.index('name="lift_cube_vertically"')
        self.assertLess(descend, close)
        self.assertLess(close, lift)
        self.assertIn("q0=q_cube_open", source[close - 300 : close + 300])
        self.assertIn("q1=q_cube_hold", source[close - 300 : close + 300])

    def test_task2_launch_does_not_own_homing_or_task1_generation(self):
        source = (PACKAGE_ROOT / "launch" / "wp3_tsk2.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("ros2_control.launch.py", source)
        self.assertNotIn("regenerate_task1_trajectory.py", source)
        self.assertNotIn("ethercat_pdo_task1.yaml", source)
        self.assertIn('executable="wp3_tsk2"', source)

    def test_task2_code_does_not_import_frozen_task1_generator(self):
        task2_files = sorted(
            (PACKAGE_ROOT / "rascl_wp3_ss26_group11").glob("task2_*.py")
        ) + [PACKAGE_ROOT / "rascl_wp3_ss26_group11" / "wp3_tsk2.py"]
        combined = "\n".join(path.read_text(encoding="utf-8") for path in task2_files)
        self.assertNotIn("generate_task1_waypoints_rtb", combined)
        self.assertNotIn("task1_cube_poses.yaml", combined)

    def test_task2_callback_does_not_swap_board_x_and_y(self):
        source = (
            PACKAGE_ROOT / "rascl_wp3_ss26_group11" / "wp3_tsk2.py"
        ).read_text(encoding="utf-8")
        self.assertIn("cube_x_m=float(message.x)", source)
        self.assertIn("cube_y_m=float(message.y)", source)
        self.assertNotIn("cube_x_m=float(message.y)", source)


if __name__ == "__main__":
    unittest.main()
