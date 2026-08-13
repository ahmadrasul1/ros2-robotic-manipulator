from __future__ import annotations

import importlib.util
import math
import unittest

import yaml
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
AUDIT_SCRIPT = (
    WORKSPACE_ROOT
    / "src/rascl_wp3_ss26_group11/scripts/audit_task1_kinematics.py"
)

spec = importlib.util.spec_from_file_location("audit_task1_kinematics", AUDIT_SCRIPT)
if spec is None or spec.loader is None:
    raise RuntimeError(f"Could not import {AUDIT_SCRIPT}")
audit_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit_module)


class KinematicsContractTests(unittest.TestCase):
    def test_generated_trajectory_matches_physical_urdf(self):
        metrics = audit_module.audit(WORKSPACE_ROOT)

        calibration_path = (
            WORKSPACE_ROOT
            / "src/rascl_wp3_ss26_group11/config/kinematics_calibration.yaml"
        )
        calibration = yaml.safe_load(calibration_path.read_text(encoding="utf-8"))
        urdf_reference = calibration["urdf_reference"]
        expected_upper_distance = math.sqrt(
            sum(
                float(value) ** 2
                for value in urdf_reference["lowerarm_joint_origin_xyz_m"]
            )
        )
        expected_tcp_distance = float(
            urdf_reference["gripper_tcp_joint_origin_xyz_m"][0]
        )

        self.assertAlmostEqual(
            metrics["upper_joint_distance_m"], expected_upper_distance, places=9
        )
        self.assertAlmostEqual(
            metrics["lower_joint_to_tcp_m"], expected_tcp_distance, places=9
        )
        self.assertLessEqual(metrics["max_ik_endpoint_error_m"], 0.0005)
        self.assertLessEqual(metrics["max_cartesian_line_deviation_m"], 0.0005)
        self.assertLessEqual(metrics["max_sim_hardware_arm_difference_rad"], 1e-12)
        self.assertGreater(int(metrics["sample_count"]), 0)


if __name__ == "__main__":
    unittest.main()
