from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .task2_config import Task2Config
from .task2_kinematics import ALL_JOINTS, IKSolution, Task2Kinematics
from .task2_trajectory import (
    JointLimits,
    append_segment,
    cartesian_segment,
    joint_segment,
    validate_and_time_scale,
)
from .trajectory_loader import JointTrajectory, TrajectorySample


@dataclass(frozen=True)
class Task2Plan:
    trajectory: JointTrajectory
    cube_board_xy_m: np.ndarray
    cube_base_xy_m: np.ndarray
    cube_center_z_m: float
    cube_radius_m: float
    cube_angle_rad: float
    goal_xyz_m: np.ndarray
    endpoint_errors_m: dict[str, float]
    time_scale: float
    max_velocity_rad_s: np.ndarray
    max_acceleration_rad_s2: np.ndarray


class Task2Planner:
    """Generate one complete online pick-and-place cycle in memory."""

    def __init__(
        self,
        *,
        config: Task2Config,
        urdf_path: str,
        limits: JointLimits,
    ) -> None:
        self.config = config
        self.kinematics = Task2Kinematics(urdf_path)
        self.limits = limits

    def _solve_required(
        self,
        *,
        name: str,
        target_m: np.ndarray,
        previous: np.ndarray,
        random_seed: int,
    ) -> IKSolution:
        solution = self.kinematics.solve_position(
            target_m=target_m,
            preferred=self.config.pick_ready_arm_rad,
            previous=previous,
            tolerance_m=self.config.ik_position_tolerance_m,
            random_seed=random_seed,
        )
        if solution is None:
            target = np.asarray(target_m, dtype=float)
            raise RuntimeError(
                f"Task 2 IK failed for {name} at "
                f"[{target[0]:.6f}, {target[1]:.6f}, {target[2]:.6f}] m"
            )
        return solution

    @staticmethod
    def _with_gripper(q_arm: np.ndarray, gripper_rad: float) -> np.ndarray:
        return np.concatenate(
            [np.asarray(q_arm, dtype=float), np.asarray([gripper_rad], dtype=float)]
        )

    def plan(
        self,
        *,
        cube_x_m: float,
        cube_y_m: float,
        current_positions_rad: np.ndarray,
        gripper_mode: str,
    ) -> Task2Plan:
        if gripper_mode not in {"simulation", "hardware"}:
            raise ValueError("gripper_mode must be 'simulation' or 'hardware'")

        current = np.asarray(current_positions_rad, dtype=float)
        if current.shape != (len(ALL_JOINTS),) or not np.all(np.isfinite(current)):
            raise ValueError("Current Task 2 joint state must contain four finite values")

        cube_board_xy = np.asarray([cube_x_m, cube_y_m], dtype=float)
        if cube_board_xy.shape != (2,) or not np.all(np.isfinite(cube_board_xy)):
            raise ValueError(
                "Task 2 cube board coordinates must contain two finite metre values"
            )

        cube_base_xy = self.config.board_xy_to_base(cube_board_xy)
        cube_radius, cube_angle = self.config.validate_workspace_xy(
            cube_base_xy,
            label="Cube",
        )
        cube_center = np.asarray(
            [
                cube_base_xy[0],
                cube_base_xy[1],
                self.config.cube_center_z_m,
            ],
            dtype=float,
        )
        above_cube = cube_center.copy()
        above_cube[2] = self.config.approach_z_m

        goal_center = self.config.goal_base_xyz_m
        above_goal = goal_center.copy()
        above_goal[2] = self.config.approach_z_m

        seeds = self.config.random_seed
        current_arm = current[:3]
        above_cube_solution = self._solve_required(
            name="above_cube",
            target_m=above_cube,
            previous=current_arm,
            random_seed=seeds,
        )
        cube_solution = self._solve_required(
            name="cube_center",
            target_m=cube_center,
            previous=above_cube_solution.q,
            random_seed=seeds + 1,
        )
        above_goal_solution = self._solve_required(
            name="above_goal",
            target_m=above_goal,
            previous=above_cube_solution.q,
            random_seed=seeds + 2,
        )
        goal_solution = self._solve_required(
            name="goal_center",
            target_m=goal_center,
            previous=above_goal_solution.q,
            random_seed=seeds + 3,
        )

        # The professor constrains shoulder_joint itself, not atan2(y, x).
        for endpoint_name, solution in (
            ("above_cube", above_cube_solution),
            ("cube_center", cube_solution),
            ("above_goal", above_goal_solution),
            ("goal_center", goal_solution),
        ):
            self.config.validate_shoulder_joint(
                float(solution.q[0]),
                label=endpoint_name,
            )

        open_rad = self.config.gripper_open_rad
        hold_rad = (
            self.config.hardware_hold_rad
            if gripper_mode == "hardware"
            else self.config.simulation_hold_rad
        )

        q_above_cube_open = self._with_gripper(above_cube_solution.q, open_rad)
        q_cube_open = self._with_gripper(cube_solution.q, open_rad)
        q_cube_hold = self._with_gripper(cube_solution.q, hold_rad)
        q_above_cube_hold = self._with_gripper(above_cube_solution.q, hold_rad)
        q_above_goal_hold = self._with_gripper(above_goal_solution.q, hold_rad)
        q_goal_hold = self._with_gripper(goal_solution.q, hold_rad)
        q_goal_open = self._with_gripper(goal_solution.q, open_rad)
        q_above_goal_open = self._with_gripper(above_goal_solution.q, open_rad)
        q_pick_ready_open = self._with_gripper(
            self.config.pick_ready_arm_rad,
            open_rad,
        )

        samples: list[TrajectorySample] = []
        current_time = 0.0
        dt = self.config.planning_sample_period_s

        # Never combine an unexpected gripper opening with arm travel.
        if abs(current[3] - open_rad) > 1e-9:
            q_current_open = current.copy()
            q_current_open[3] = open_rad
            segment = joint_segment(
                q0=current,
                q1=q_current_open,
                duration_s=self.config.gripper_action_duration_s,
                sample_period_s=dt,
                start_time_s=current_time,
                name="ensure_gripper_open_before_approach",
                gripper_marker=0.0,
            )
            append_segment(samples, segment)
            current_time += self.config.gripper_action_duration_s
            current = q_current_open

        segment = joint_segment(
            q0=current,
            q1=q_above_cube_open,
            duration_s=self.config.transfer_duration_s,
            sample_period_s=dt,
            start_time_s=current_time,
            name="move_to_above_cube_open",
            gripper_marker=0.0,
        )
        append_segment(samples, segment)
        current_time += self.config.transfer_duration_s

        segment = cartesian_segment(
            kinematics=self.kinematics,
            q0=q_above_cube_open,
            q1=q_cube_open,
            p0_m=above_cube,
            p1_m=cube_center,
            duration_s=self.config.vertical_duration_s,
            sample_period_s=dt,
            start_time_s=current_time,
            name="descend_to_cube_center_open",
            gripper_marker=0.0,
            tolerance_m=self.config.ik_position_tolerance_m,
            maximum_joint_step_rad=self.config.cartesian_max_joint_step_rad,
            random_seed=seeds + 1000,
        )
        append_segment(samples, segment)
        current_time += self.config.vertical_duration_s

        # This is deliberately the first closing segment. Both endpoints use the
        # cube-center arm configuration, so the gripper cannot close early.
        segment = joint_segment(
            q0=q_cube_open,
            q1=q_cube_hold,
            duration_s=self.config.gripper_action_duration_s,
            sample_period_s=dt,
            start_time_s=current_time,
            name="close_gripper_at_cube_center",
            gripper_marker=1.0,
        )
        append_segment(samples, segment)
        current_time += self.config.gripper_action_duration_s

        segment = cartesian_segment(
            kinematics=self.kinematics,
            q0=q_cube_hold,
            q1=q_above_cube_hold,
            p0_m=cube_center,
            p1_m=above_cube,
            duration_s=self.config.vertical_duration_s,
            sample_period_s=dt,
            start_time_s=current_time,
            name="lift_cube_vertically",
            gripper_marker=1.0,
            tolerance_m=self.config.ik_position_tolerance_m,
            maximum_joint_step_rad=self.config.cartesian_max_joint_step_rad,
            random_seed=seeds + 2000,
        )
        append_segment(samples, segment)
        current_time += self.config.vertical_duration_s

        segment = joint_segment(
            q0=q_above_cube_hold,
            q1=q_above_goal_hold,
            duration_s=self.config.transfer_duration_s,
            sample_period_s=dt,
            start_time_s=current_time,
            name="transfer_cube_to_fixed_task1_goal",
            gripper_marker=1.0,
        )
        append_segment(samples, segment)
        current_time += self.config.transfer_duration_s

        segment = cartesian_segment(
            kinematics=self.kinematics,
            q0=q_above_goal_hold,
            q1=q_goal_hold,
            p0_m=above_goal,
            p1_m=goal_center,
            duration_s=self.config.vertical_duration_s,
            sample_period_s=dt,
            start_time_s=current_time,
            name="descend_to_goal_center_hold",
            gripper_marker=1.0,
            tolerance_m=self.config.ik_position_tolerance_m,
            maximum_joint_step_rad=self.config.cartesian_max_joint_step_rad,
            random_seed=seeds + 3000,
        )
        append_segment(samples, segment)
        current_time += self.config.vertical_duration_s

        segment = joint_segment(
            q0=q_goal_hold,
            q1=q_goal_open,
            duration_s=self.config.gripper_action_duration_s,
            sample_period_s=dt,
            start_time_s=current_time,
            name="open_gripper_at_goal_center",
            gripper_marker=0.0,
        )
        append_segment(samples, segment)
        current_time += self.config.gripper_action_duration_s

        segment = cartesian_segment(
            kinematics=self.kinematics,
            q0=q_goal_open,
            q1=q_above_goal_open,
            p0_m=goal_center,
            p1_m=above_goal,
            duration_s=self.config.vertical_duration_s,
            sample_period_s=dt,
            start_time_s=current_time,
            name="retreat_from_goal_open",
            gripper_marker=0.0,
            tolerance_m=self.config.ik_position_tolerance_m,
            maximum_joint_step_rad=self.config.cartesian_max_joint_step_rad,
            random_seed=seeds + 4000,
        )
        append_segment(samples, segment)
        current_time += self.config.vertical_duration_s

        segment = joint_segment(
            q0=q_above_goal_open,
            q1=q_pick_ready_open,
            duration_s=self.config.return_duration_s,
            sample_period_s=dt,
            start_time_s=current_time,
            name="return_to_pick_ready_open",
            gripper_marker=0.0,
        )
        append_segment(samples, segment)

        raw = JointTrajectory(
            joint_names=list(ALL_JOINTS),
            samples=samples,
            metadata={
                "rascl_task2_format": "1",
                "point_z_policy": "ignored; fixed from board height plus half cube height",
                "input_xy_frame": self.config.input_xy_frame,
                "cube_x_board_m": f"{cube_board_xy[0]:.9f}",
                "cube_y_board_m": f"{cube_board_xy[1]:.9f}",
                "cube_x_base_m": f"{cube_base_xy[0]:.9f}",
                "cube_y_base_m": f"{cube_base_xy[1]:.9f}",
                "cube_center_z_m": f"{self.config.cube_center_z_m:.9f}",
                "goal_x_m": f"{goal_center[0]:.9f}",
                "goal_y_m": f"{goal_center[1]:.9f}",
                "goal_z_m": f"{goal_center[2]:.9f}",
                "gripper_mode": gripper_mode,
            },
        )

        streamed, metrics, time_scale = validate_and_time_scale(
            raw,
            self.limits,
            self.config.publish_rate_hz,
        )
        if not np.allclose(
            np.asarray(streamed.samples[0].positions, dtype=float),
            np.asarray(current_positions_rad, dtype=float),
            atol=1e-12,
            rtol=0.0,
        ):
            raise RuntimeError("Task 2 trajectory does not start at the measured state")
        if not np.allclose(
            np.asarray(streamed.samples[-1].positions, dtype=float),
            q_pick_ready_open,
            atol=1e-9,
            rtol=0.0,
        ):
            raise RuntimeError("Task 2 trajectory does not finish at pick-ready/open")

        return Task2Plan(
            trajectory=streamed,
            cube_board_xy_m=cube_board_xy,
            cube_base_xy_m=cube_base_xy,
            cube_center_z_m=self.config.cube_center_z_m,
            cube_radius_m=cube_radius,
            cube_angle_rad=cube_angle,
            goal_xyz_m=goal_center,
            endpoint_errors_m={
                "above_cube": above_cube_solution.position_error_m,
                "cube_center": cube_solution.position_error_m,
                "above_goal": above_goal_solution.position_error_m,
                "goal_center": goal_solution.position_error_m,
            },
            time_scale=time_scale,
            max_velocity_rad_s=metrics.max_velocity,
            max_acceleration_rad_s2=metrics.max_acceleration,
        )
