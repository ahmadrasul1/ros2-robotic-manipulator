from __future__ import annotations

import math
import time
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .task2_config import Task2Config, load_task2_config
from .task2_kinematics import ALL_JOINTS
from .task2_planner import Task2Plan, Task2Planner
from .task2_trajectory import load_joint_limits
from .trajectory_executor import ExecutionConfig, TrajectoryExecutor
from .trajectory_loader import TrajectorySample


class Wp3Task2Node(Node):
    """Online Task 2 planner for one runtime cube position at a time."""

    WAITING = "WAITING_FOR_CUBE"
    PLANNING = "PLANNING"
    EXECUTING = "EXECUTING"

    def __init__(self) -> None:
        super().__init__("wp3_tsk2")
        wp3_share = Path(get_package_share_directory("rascl_wp3_ss26_group11"))
        description_share = Path(get_package_share_directory("rascl_description"))

        self.declare_parameter(
            "config_file",
            str(wp3_share / "config" / "task2_online_planning.yaml"),
        )
        self.declare_parameter(
            "robot_urdf_file",
            str(description_share / "urdf" / "rascl.urdf.xacro"),
        )
        self.declare_parameter(
            "robot_limits_file",
            str(wp3_share / "config" / "robot_limits.yaml"),
        )
        self.declare_parameter("execution_mode", "joint_states")
        self.declare_parameter("gripper_mode", "simulation")
        self.declare_parameter("controller_topic", "/joint_position_controller/commands")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("status_topic", "/wp3_tsk2/status")

        self._config_file = Path(str(self.get_parameter("config_file").value))
        self._urdf_file = Path(str(self.get_parameter("robot_urdf_file").value))
        self._limits_file = Path(str(self.get_parameter("robot_limits_file").value))
        self._execution_mode = str(self.get_parameter("execution_mode").value).strip()
        self._gripper_mode = str(self.get_parameter("gripper_mode").value).strip()
        self._controller_topic = str(self.get_parameter("controller_topic").value)
        self._joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self._status_topic = str(self.get_parameter("status_topic").value)

        if self._execution_mode not in {"joint_states", "controller"}:
            raise ValueError("execution_mode must be 'joint_states' or 'controller'")
        if self._gripper_mode not in {"simulation", "hardware"}:
            raise ValueError("gripper_mode must be 'simulation' or 'hardware'")
        if self._execution_mode == "controller" and self._gripper_mode != "hardware":
            raise RuntimeError("Controller execution requires gripper_mode=hardware")

        self._config: Task2Config = load_task2_config(self._config_file)
        if (
            self._execution_mode == "controller"
            and self._config.workspace.values_are_placeholders
        ):
            raise RuntimeError(
                "Task 2 hardware execution is blocked because the configured "
                f"workspace radii ({self._config.workspace.min_radius_m:.6f} m, "
                f"{self._config.workspace.max_radius_m:.6f} m) are still marked "
                "as placeholders. Enter the verified radii and set "
                "workspace.values_are_placeholders=false."
            )

        limits = load_joint_limits(self._limits_file)
        self._planner = Task2Planner(
            config=self._config,
            urdf_path=str(self._urdf_file),
            limits=limits,
        )

        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._joint_state_sub = self.create_subscription(
            JointState,
            self._joint_states_topic,
            self._on_joint_state,
            20,
        )
        self._point_sub = self.create_subscription(
            Point,
            self._config.input_topic,
            self._on_cube_point,
            10,
        )
        self._executor = TrajectoryExecutor(
            self,
            ExecutionConfig(
                mode=self._execution_mode,
                joint_names=list(ALL_JOINTS),
                controller_topic=self._controller_topic,
                joint_states_topic=self._joint_states_topic,
                publish_rate_hz=self._config.publish_rate_hz,
                hold_last_sample=True,
            ),
        )

        self._state = self.WAITING
        self._current_positions: np.ndarray | None = None
        self._last_joint_state_monotonic: float | None = None
        self._planning_pool = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="wp3_task2_planner",
        )
        self._planning_future: Future[Task2Plan] | None = None
        self._plan_start_positions: np.ndarray | None = None
        self._state_timer = self.create_timer(0.05, self._on_state_timer)
        self._last_wait_log = 0.0

        if self._execution_mode == "joint_states":
            initial = np.concatenate(
                [
                    self._config.pick_ready_arm_rad,
                    np.asarray([self._config.gripper_open_rad], dtype=float),
                ]
            )
            self._current_positions = initial.copy()
            self._last_joint_state_monotonic = time.monotonic()
            self._executor.hold_sample(
                TrajectorySample(
                    time_from_start=0.0,
                    positions=[float(value) for value in initial],
                    gripper=0.0,
                    segment="task2_pick_ready",
                )
            )

        goal = self._config.goal_base_xyz_m
        self.get_logger().info(f"Task 2 configuration: {self._config_file}")
        self.get_logger().info(
            "Task 2 uses only Point.x and Point.y in base_link metres; Point.z is ignored"
        )
        self.get_logger().info(
            f"Fixed cube/TCP center Z = board {self._config.board_surface_z_m:.3f} m "
            f"+ half cube {0.5 * self._config.cube_height_m:.3f} m "
            f"= {self._config.cube_center_z_m:.3f} m"
        )
        self.get_logger().info(
            "Empirical XY correction in base_link: "
            f"[{self._config.xy_correction_base_m[0]:+.4f}, "
            f"{self._config.xy_correction_base_m[1]:+.4f}] m"
        )
        self.get_logger().info(
            f"Fixed Task 1 goal reused by Task 2: "
            f"[{goal[0]:.4f}, {goal[1]:.4f}, {goal[2]:.4f}] m"
        )
        self.get_logger().info(
            f"Configured workspace: {self._config.workspace.min_radius_m:.3f} to "
            f"{self._config.workspace.max_radius_m:.3f} m"
        )
        if self._config.workspace.values_are_placeholders:
            self.get_logger().warning(
                "Workspace radii are placeholders (0.11 m, 0.30 m). "
                "Simulation is allowed; hardware mode is blocked."
            )
        self._publish_status(self.WAITING)

    def destroy_node(self) -> bool:
        self._planning_pool.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()

    def _publish_status(self, text: str) -> None:
        message = String()
        message.data = text
        self._status_pub.publish(message)

    def _set_state(self, state: str, detail: str | None = None) -> None:
        self._state = state
        payload = state if not detail else f"{state}: {detail}"
        self._publish_status(payload)

    def _on_joint_state(self, message: JointState) -> None:
        index = {name: position for name, position in zip(message.name, message.position)}
        if any(name not in index for name in ALL_JOINTS):
            return
        values = np.asarray([index[name] for name in ALL_JOINTS], dtype=float)
        if values.shape != (len(ALL_JOINTS),) or not np.all(np.isfinite(values)):
            return
        self._current_positions = values
        self._last_joint_state_monotonic = time.monotonic()

    def _current_state(self) -> tuple[np.ndarray | None, str]:
        if self._current_positions is None or self._last_joint_state_monotonic is None:
            return None, "waiting for a complete /joint_states sample"
        age = time.monotonic() - self._last_joint_state_monotonic
        if age > self._config.joint_state_timeout_s:
            return None, (
                f"latest /joint_states sample is stale ({age:.3f}s > "
                f"{self._config.joint_state_timeout_s:.3f}s)"
            )
        return self._current_positions.copy(), "joint state ready"

    def _read_hardware_ready_fields(self) -> dict[str, str]:
        path = self._config.hardware_ready_file
        content = path.read_text(encoding="utf-8")
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

    def _hardware_ready(self) -> tuple[bool, str]:
        if self._execution_mode != "controller":
            return True, "simulation execution"
        if self._executor.controller_subscription_count < 1:
            return False, "waiting for the active joint_position_controller subscriber"
        if not self._config.hardware_ready_file.is_file():
            return False, f"waiting for {self._config.hardware_ready_file}"
        try:
            fields = self._read_hardware_ready_fields()
        except Exception as exc:  # noqa: BLE001 - converted into a blocking reason.
            return False, str(exc)

        if (
            self._config.require_allow_motion
            and fields.get("allow_motion", "false").lower() != "true"
        ):
            return False, "PDO bridge has allow_motion=false"
        if (
            self._config.require_arm_reference
            and fields.get("reference_valid", "false").lower() != "true"
        ):
            return False, "arm homing/reference is not valid"
        if (
            self._config.require_gripper_reference
            and fields.get("gripper_reference_valid", "false").lower() != "true"
        ):
            return False, "end-effector reference is not valid"
        return True, "hardware ready"

    def _pick_ready_check(self, current: np.ndarray) -> tuple[bool, str]:
        if not self._config.require_pick_ready_before_request:
            return True, "pick-ready check disabled"
        errors = np.abs(current[:3] - self._config.pick_ready_arm_rad)
        violations = [
            f"{ALL_JOINTS[index]} error {errors[index]:.4f} rad > "
            f"{self._config.start_pose_tolerance_rad[index]:.4f} rad"
            for index in range(3)
            if errors[index] > self._config.start_pose_tolerance_rad[index]
        ]
        if violations:
            return False, "robot is not at pick-ready; " + "; ".join(violations)
        return True, "pick-ready verified"

    def _on_cube_point(self, message: Point) -> None:
        if self._state != self.WAITING:
            self.get_logger().warning(
                f"Task 2 is {self._state}; ignoring cube point x={message.x:.4f}, "
                f"y={message.y:.4f} m"
            )
            return

        if not math.isfinite(message.x) or not math.isfinite(message.y):
            self.get_logger().error("Rejected Task 2 point: x and y must be finite")
            self._publish_status("REJECTED_INPUT: non-finite x/y")
            return

        hardware_ready, hardware_reason = self._hardware_ready()
        if not hardware_ready:
            self.get_logger().warning(f"Rejected Task 2 point: {hardware_reason}")
            self._publish_status(f"REJECTED_INPUT: {hardware_reason}")
            return

        current, current_reason = self._current_state()
        if current is None:
            self.get_logger().warning(f"Rejected Task 2 point: {current_reason}")
            self._publish_status(f"REJECTED_INPUT: {current_reason}")
            return

        ready, ready_reason = self._pick_ready_check(current)
        if not ready:
            self.get_logger().warning(f"Rejected Task 2 point: {ready_reason}")
            self._publish_status(f"REJECTED_INPUT: {ready_reason}")
            return

        self.get_logger().info(
            f"Received cube board coordinates x_board={message.x:.6f} m, "
            f"y_board={message.y:.6f} m; "
            f"point.z={message.z:.6f} is ignored"
        )
        self._plan_start_positions = current.copy()
        self._set_state(self.PLANNING)
        self._planning_future = self._planning_pool.submit(
            self._planner.plan,
            cube_x_m=float(message.x),
            cube_y_m=float(message.y),
            current_positions_rad=current,
            gripper_mode=self._gripper_mode,
        )

    def _start_completed_plan(self, plan: Task2Plan) -> None:
        hardware_ready, reason = self._hardware_ready()
        if not hardware_ready:
            raise RuntimeError(f"Hardware became unavailable during planning: {reason}")

        current, current_reason = self._current_state()
        if current is None:
            raise RuntimeError(current_reason)
        planned_start = np.asarray(plan.trajectory.samples[0].positions, dtype=float)
        tolerances = np.concatenate(
            [
                self._config.start_pose_tolerance_rad,
                np.asarray([self._config.start_gripper_tolerance_rad], dtype=float),
            ]
        )
        errors = np.abs(current - planned_start)
        violations = [
            f"{name} changed by {errors[index]:.4f} rad > {tolerances[index]:.4f} rad"
            for index, name in enumerate(ALL_JOINTS)
            if errors[index] > tolerances[index]
        ]
        if violations:
            raise RuntimeError(
                "Robot moved while Task 2 was planning; " + "; ".join(violations)
            )

        endpoint_text = ", ".join(
            f"{name}={error * 1000.0:.3f} mm"
            for name, error in plan.endpoint_errors_m.items()
        )
        self.get_logger().info(
            f"Task 2 plan valid: cube board="
            f"[{plan.cube_board_xy_m[0]:.4f}, {plan.cube_board_xy_m[1]:.4f}] m, "
            f"cube base="
            f"[{plan.cube_base_xy_m[0]:.4f}, {plan.cube_base_xy_m[1]:.4f}, "
            f"{plan.cube_center_z_m:.4f}] m, radius={plan.cube_radius_m:.4f} m, "
            f"angle={plan.cube_angle_rad:.4f} rad"
        )
        self.get_logger().info(
            f"Goal=[{plan.goal_xyz_m[0]:.4f}, {plan.goal_xyz_m[1]:.4f}, "
            f"{plan.goal_xyz_m[2]:.4f}] m; IK errors: {endpoint_text}"
        )
        if plan.time_scale > 1.0 + 1e-9:
            self.get_logger().warning(
                f"Trajectory time was stretched by {plan.time_scale:.3f} to satisfy "
                "the existing robot velocity/acceleration limits"
            )
        self._executor.start(plan.trajectory)
        self._set_state(self.EXECUTING)

    def _on_state_timer(self) -> None:
        if self._state == self.PLANNING and self._planning_future is not None:
            if self._planning_future.done():
                future = self._planning_future
                self._planning_future = None
                try:
                    plan = future.result()
                    self._start_completed_plan(plan)
                except Exception as exc:  # noqa: BLE001 - planning failure is recoverable.
                    self.get_logger().error(f"Task 2 planning rejected: {exc}")
                    self._publish_status(f"PLANNING_FAILED: {exc}")
                    self._set_state(self.WAITING, "ready for a corrected cube point")

        if self._state == self.EXECUTING and self._executor.finished:
            self.get_logger().info(
                "Task 2 cycle finished at pick-ready/open. Waiting for the next cube."
            )
            self._set_state(self.WAITING)

        if self._state == self.WAITING and self._execution_mode == "controller":
            ready, reason = self._hardware_ready()
            now = time.monotonic()
            if not ready and now - self._last_wait_log >= 2.0:
                self.get_logger().warning(f"Task 2 waiting: {reason}")
                self._last_wait_log = now


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: Wp3Task2Node | None = None
    try:
        node = Wp3Task2Node()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
