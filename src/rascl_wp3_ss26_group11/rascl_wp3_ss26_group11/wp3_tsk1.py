from __future__ import annotations

import hashlib
import math
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from .trajectory_executor import ExecutionConfig, TrajectoryExecutor
from .trajectory_loader import JointTrajectory, load_joint_trajectory


class Wp3Task1Node(Node):
    """Play one prevalidated offline Task 1 trajectory."""

    def __init__(self) -> None:
        super().__init__("wp3_tsk1")
        package_share = Path(get_package_share_directory("rascl_wp3_ss26_group11"))

        self.declare_parameter(
            "trajectory_file",
            str(
                package_share
                / "trajectories"
                / "task1"
                / "task1_full_simulation_ik.csv"
            ),
        )
        self.declare_parameter("poses_file", "")
        self.declare_parameter("robot_urdf_file", "")
        self.declare_parameter("require_fresh_trajectory", False)
        self.declare_parameter("execution_mode", "joint_states")
        self.declare_parameter("trajectory_gripper_mode", "simulation")
        self.declare_parameter("controller_topic", "/joint_position_controller/commands")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter(
            "joint_names",
            [
                "shoulder_joint",
                "upperarm_joint",
                "lowerarm_joint",
                "end_effector_joint",
            ],
        )
        self.declare_parameter("auto_start", True)
        self.declare_parameter("hold_last_sample", True)
        self.declare_parameter("hardware_ready_file", "/tmp/rascl_pdo_ready")
        self.declare_parameter("require_arm_reference", True)
        self.declare_parameter("require_gripper_reference", True)
        self.declare_parameter(
            "start_pose_tolerance_rad", [0.03, 0.03, 0.03, 0.05]
        )

        self._trajectory_file = Path(str(self.get_parameter("trajectory_file").value))
        self._poses_file = Path(str(self.get_parameter("poses_file").value))
        self._robot_urdf_file = Path(
            str(self.get_parameter("robot_urdf_file").value)
        )
        self._require_fresh_trajectory = bool(
            self.get_parameter("require_fresh_trajectory").value
        )
        self._execution_mode = str(self.get_parameter("execution_mode").value)
        self._trajectory_gripper_mode = str(
            self.get_parameter("trajectory_gripper_mode").value
        ).strip().lower()
        self._controller_topic = str(self.get_parameter("controller_topic").value)
        self._joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._joint_names = list(self.get_parameter("joint_names").value)
        self._auto_start = bool(self.get_parameter("auto_start").value)
        self._hold_last_sample = bool(self.get_parameter("hold_last_sample").value)
        self._hardware_ready_file = Path(
            str(self.get_parameter("hardware_ready_file").value)
        )
        self._require_arm_reference = bool(
            self.get_parameter("require_arm_reference").value
        )
        self._require_gripper_reference = bool(
            self.get_parameter("require_gripper_reference").value
        )
        self._start_pose_tolerance = [
            float(value)
            for value in self.get_parameter("start_pose_tolerance_rad").value
        ]

        if self._execution_mode not in {"joint_states", "controller"}:
            raise ValueError("execution_mode must be 'joint_states' or 'controller'")
        if self._trajectory_gripper_mode not in {"simulation", "hardware"}:
            raise ValueError("trajectory_gripper_mode must be 'simulation' or 'hardware'")
        if (
            self._execution_mode == "controller"
            and self._trajectory_gripper_mode != "hardware"
        ):
            raise RuntimeError(
                "Controller execution refuses a simulation-gripper trajectory. "
                "Generate/select a hardware trajectory and set "
                "trajectory_gripper_mode=hardware."
            )
        if len(self._start_pose_tolerance) != len(self._joint_names):
            raise ValueError("start_pose_tolerance_rad must contain one value per joint")
        if any(not math.isfinite(value) or value <= 0.0 for value in self._start_pose_tolerance):
            raise ValueError("start_pose_tolerance_rad values must be finite and positive")

        self._started = False
        self._last_hardware_wait_log = 0.0

        self.get_logger().info(f"Task 1 trajectory: {self._trajectory_file}")
        self.get_logger().info(f"Execution mode: {self._execution_mode}")

        # Fail the process immediately on malformed/missing trajectories. Keeping a
        # broken node alive previously made launch appear successful while doing nothing.
        self._trajectory: JointTrajectory = load_joint_trajectory(
            self._trajectory_file, self._joint_names
        )
        self._validate_trajectory_inputs()
        self._executor = TrajectoryExecutor(
            self,
            ExecutionConfig(
                mode=self._execution_mode,
                joint_names=self._joint_names,
                controller_topic=self._controller_topic,
                joint_states_topic=self._joint_states_topic,
                publish_rate_hz=self._publish_rate_hz,
                hold_last_sample=self._hold_last_sample,
            ),
        )

        if self._execution_mode == "joint_states":
            self._executor.hold_sample(self._trajectory.samples[0])
            self.get_logger().info("Initial visualization pose published")
        else:
            self.get_logger().info(
                "Controller commands remain blocked until PDO readiness, references, "
                "and the measured start pose are verified"
            )

        self._start_timer = None
        if self._auto_start:
            self._start_timer = self.create_timer(0.25, self._start_once)

    @staticmethod
    def _sha256(path: Path) -> str:
        if not path.is_file():
            raise FileNotFoundError(f"Trajectory input file not found: {path}")
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def _validate_trajectory_inputs(self) -> None:
        metadata = self._trajectory.metadata
        if not self._require_fresh_trajectory:
            return

        if metadata.get("rascl_task1_format") != "2":
            raise RuntimeError(
                "Hardware execution refuses this CSV because it has no current "
                "Task 1 source fingerprint. Run wp3_prepare_task1.launch.py."
            )

        checks = (
            ("poses_sha256", self._poses_file, "task1_cube_poses.yaml"),
            ("urdf_sha256", self._robot_urdf_file, "rascl.urdf.xacro"),
        )
        mismatches: list[str] = []
        for metadata_key, source_path, label in checks:
            expected = metadata.get(metadata_key, "")
            actual = self._sha256(source_path)
            if not expected or expected != actual:
                mismatches.append(label)

        csv_gripper_mode = metadata.get("gripper_mode", "")
        if csv_gripper_mode != self._trajectory_gripper_mode:
            mismatches.append(
                f"gripper mode CSV={csv_gripper_mode!r}, "
                f"requested={self._trajectory_gripper_mode!r}"
            )

        if mismatches:
            raise RuntimeError(
                "Hardware execution refuses a stale Task 1 trajectory. Changed "
                "or mismatched inputs: "
                + ", ".join(mismatches)
                + ". Run: ros2 launch rascl_wp3_ss26_group11 "
                "wp3_prepare_task1.launch.py"
            )

        self.get_logger().info(
            "Trajectory source fingerprints match the current pose YAML and URDF"
        )
        self.get_logger().info(
            "Trajectory Z calibration: global=%s m, cube1=%s m, "
            "cube2/3=%s m, buffer=%s m, goal=%s m"
            % (
                metadata.get("target_z_correction_m", "unknown"),
                metadata.get("cube1_start_z_correction_m", "unknown"),
                metadata.get("cube2_3_start_z_correction_m", "unknown"),
                metadata.get("cube3_buffer_z_correction_m", "unknown"),
                metadata.get("goal_z_correction_m", "unknown"),
            )
        )

    def _read_hardware_ready_fields(self) -> dict[str, str]:
        content = self._hardware_ready_file.read_text(encoding="utf-8")
        lines = [line.strip() for line in content.splitlines() if line.strip()]
        if not lines or lines[0] != "PDO_READY":
            reason = next(
                (line.split("=", 1)[1] for line in lines if line.startswith("reason=")),
                "marker is missing or invalid",
            )
            raise RuntimeError(f"PDO bridge is not ready: {reason}")
        fields: dict[str, str] = {}
        for line in lines[1:]:
            if "=" in line:
                key, value = line.split("=", 1)
                fields[key.strip()] = value.strip()
        return fields

    def _parse_actual_positions(self, fields: dict[str, str]) -> list[float]:
        raw = fields.get("actual_positions_rad")
        if not raw:
            raise RuntimeError("PDO ready marker has no measured actual_positions_rad")
        try:
            positions = [float(value) for value in raw.split(",")]
        except ValueError as exc:
            raise RuntimeError("PDO ready marker contains invalid actual positions") from exc
        if len(positions) != len(self._joint_names) or not all(
            math.isfinite(value) for value in positions
        ):
            raise RuntimeError("PDO ready marker actual position vector is invalid")
        return positions

    def _hardware_execution_ready(self) -> tuple[bool, str]:
        if self._executor.controller_subscription_count < 1:
            return False, "waiting for the active joint_position_controller subscriber"
        if not self._hardware_ready_file.is_file():
            return False, f"waiting for {self._hardware_ready_file}"
        try:
            fields = self._read_hardware_ready_fields()
        except Exception as exc:  # noqa: BLE001 - converted into a blocking reason.
            return False, str(exc)

        if fields.get("allow_motion", "false").lower() != "true":
            return False, "PDO bridge has allow_motion=false"
        if (
            self._require_arm_reference
            and fields.get("reference_valid", "false").lower() != "true"
        ):
            return False, "shoulder/upper-arm/lower-arm homing was not validated"
        if (
            self._require_gripper_reference
            and fields.get("gripper_reference_valid", "false").lower() != "true"
        ):
            return False, "end-effector reference has not been physically validated"

        try:
            actual = self._parse_actual_positions(fields)
        except RuntimeError as exc:
            return False, str(exc)

        first = self._trajectory.samples[0].positions
        errors = [abs(command - measured) for command, measured in zip(first, actual)]
        checked_indices = list(range(len(self._joint_names)))
        if not self._require_gripper_reference:
            checked_indices = checked_indices[:-1]
        violations = [
            f"{self._joint_names[index]}: measured={actual[index]:.4f}, "
            f"csv={first[index]:.4f}, error={errors[index]:.4f} > "
            f"{self._start_pose_tolerance[index]:.4f} rad"
            for index in checked_indices
            if errors[index] > self._start_pose_tolerance[index]
        ]
        if violations:
            return False, "start-pose mismatch; " + "; ".join(violations)
        return True, "hardware ready and start pose matched"

    def _log_hardware_wait(self, reason: str) -> None:
        now = time.monotonic()
        if now - self._last_hardware_wait_log >= 2.0:
            self.get_logger().warning(f"Controller execution blocked: {reason}")
            self._last_hardware_wait_log = now

    def _start_once(self) -> None:
        if self._started:
            return

        if self._execution_mode == "controller":
            ready, reason = self._hardware_execution_ready()
            if not ready:
                self._log_hardware_wait(reason)
                return
            self.get_logger().info(reason)

        # start() validates all remaining executor contracts before returning.
        self._executor.start(self._trajectory)
        self._started = True
        if self._start_timer is not None:
            self._start_timer.cancel()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = Wp3Task1Node()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
