from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import yaml

from .board_transform import transform_board_xy_to_base


@dataclass(frozen=True)
class WorkspaceConfig:
    min_radius_m: float
    max_radius_m: float
    min_shoulder_joint_rad: float
    max_shoulder_joint_rad: float
    values_are_placeholders: bool


@dataclass(frozen=True)
class Task2Config:
    input_topic: str
    input_xy_frame: str
    board_surface_z_m: float
    cube_height_m: float
    xy_correction_base_m: np.ndarray
    goal_board_xy_m: np.ndarray
    workspace: WorkspaceConfig
    approach_clearance_m: float
    ik_position_tolerance_m: float
    pick_ready_arm_rad: np.ndarray
    random_seed: int
    cartesian_max_joint_step_rad: float
    transfer_duration_s: float
    vertical_duration_s: float
    gripper_action_duration_s: float
    return_duration_s: float
    planning_sample_period_s: float
    publish_rate_hz: float
    require_pick_ready_before_request: bool
    start_pose_tolerance_rad: np.ndarray
    start_gripper_tolerance_rad: float
    joint_state_timeout_s: float
    gripper_open_rad: float
    simulation_hold_rad: float
    hardware_hold_rad: float
    hardware_ready_file: Path
    require_arm_reference: bool
    require_gripper_reference: bool
    require_allow_motion: bool
    board_to_base_xy: dict[str, Any]

    @property
    def cube_center_z_m(self) -> float:
        """Fixed TCP height for one cube standing directly on the board."""
        return self.board_surface_z_m + 0.5 * self.cube_height_m

    @property
    def approach_z_m(self) -> float:
        return self.cube_center_z_m + self.approach_clearance_m

    def board_xy_to_base(self, board_xy_m: Iterable[float]) -> np.ndarray:
        return transform_board_xy_to_base(
            board_xy_m,
            mapping=self.board_to_base_xy,
            correction_base_m=self.xy_correction_base_m,
        )

    @property
    def goal_base_xy_m(self) -> np.ndarray:
        return self.board_xy_to_base(self.goal_board_xy_m)

    @property
    def goal_base_xyz_m(self) -> np.ndarray:
        xy = self.goal_base_xy_m
        return np.asarray([xy[0], xy[1], self.cube_center_z_m], dtype=float)

    def validate_workspace_xy(self, xy_m: np.ndarray, *, label: str) -> tuple[float, float]:
        xy = np.asarray(xy_m, dtype=float)
        if xy.shape != (2,) or not np.all(np.isfinite(xy)):
            raise ValueError(f"{label} XY must contain two finite metre values")

        radius = float(np.linalg.norm(xy))
        angle = float(math.atan2(float(xy[1]), float(xy[0])))
        workspace = self.workspace
        if radius < workspace.min_radius_m - 1e-12:
            raise ValueError(
                f"{label} radius {radius:.6f} m is below the configured minimum "
                f"{workspace.min_radius_m:.6f} m"
            )
        if radius > workspace.max_radius_m + 1e-12:
            raise ValueError(
                f"{label} radius {radius:.6f} m exceeds the configured maximum "
                f"{workspace.max_radius_m:.6f} m"
            )
        # atan2(y, x) is reported for diagnostics only. The task sheet limits
        # shoulder_joint, not the Cartesian polar angle. The IK result is checked
        # separately because the URDF chain has fixed XY offsets.
        return radius, angle

    def validate_shoulder_joint(self, shoulder_rad: float, *, label: str) -> None:
        value = float(shoulder_rad)
        if not math.isfinite(value):
            raise ValueError(f"{label} shoulder_joint must be finite")
        workspace = self.workspace
        if (
            value < workspace.min_shoulder_joint_rad - 1e-12
            or value > workspace.max_shoulder_joint_rad + 1e-12
        ):
            raise ValueError(
                f"{label} shoulder_joint {value:.6f} rad is outside "
                f"[{workspace.min_shoulder_joint_rad:.6f}, "
                f"{workspace.max_shoulder_joint_rad:.6f}] rad"
            )


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"Task 2 YAML root must be a mapping: {path}")
    return data


def _finite(value: Any, label: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def _positive(value: Any, label: str) -> float:
    result = _finite(value, label)
    if result <= 0.0:
        raise ValueError(f"{label} must be positive")
    return result


def _finite_vector(value: Any, length: int, label: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must contain {length} finite values")
    return result


def load_task2_config(path: str | Path) -> Task2Config:
    config_path = Path(path)
    data = _load_yaml(config_path)

    input_cfg = data.get("input", {}) or {}
    board = data.get("board", {}) or {}
    cube = data.get("cube", {}) or {}
    workspace_cfg = data.get("workspace", {}) or {}
    goal = data.get("goal", {}) or {}
    planning = data.get("planning", {}) or {}
    gripper = data.get("gripper", {}) or {}
    hardware = data.get("hardware", {}) or {}
    hardware_ready_file = str(hardware.get("ready_file", "")).strip()

    if str(input_cfg.get("message_type", "")) != "geometry_msgs/msg/Point":
        raise ValueError("input.message_type must remain geometry_msgs/msg/Point")
    if not bool(input_cfg.get("use_point_x", False)) or not bool(
        input_cfg.get("use_point_y", False)
    ):
        raise ValueError("Task 2 must use point.x and point.y")
    if not bool(input_cfg.get("ignore_point_z", False)):
        raise ValueError(
            "input.ignore_point_z must be true; Z is fixed from board and cube height"
        )
    if str(input_cfg.get("units", "")) != "m":
        raise ValueError("input.units must be 'm'")
    if not str(input_cfg.get("topic", "")).strip():
        raise ValueError("input.topic must not be empty")
    if str(workspace_cfg.get("units", "")) != "m":
        raise ValueError("workspace.units must be 'm'")
    if str(goal.get("source", "")) != "task1_goal":
        raise ValueError(
            "goal.source must be 'task1_goal'; Task 2 reuses the frozen Task 1 goal"
        )
    if not hardware_ready_file:
        raise ValueError("hardware.ready_file must not be empty")

    workspace = WorkspaceConfig(
        min_radius_m=_positive(workspace_cfg.get("min_radius_m"), "workspace.min_radius_m"),
        max_radius_m=_positive(workspace_cfg.get("max_radius_m"), "workspace.max_radius_m"),
        min_shoulder_joint_rad=_finite(
            workspace_cfg.get("min_shoulder_joint_rad"),
            "workspace.min_shoulder_joint_rad",
        ),
        max_shoulder_joint_rad=_finite(
            workspace_cfg.get("max_shoulder_joint_rad"),
            "workspace.max_shoulder_joint_rad",
        ),
        values_are_placeholders=bool(
            workspace_cfg.get("values_are_placeholders", True)
        ),
    )
    if workspace.min_radius_m >= workspace.max_radius_m:
        raise ValueError("workspace.min_radius_m must be smaller than max_radius_m")
    if workspace.min_shoulder_joint_rad >= workspace.max_shoulder_joint_rad:
        raise ValueError(
            "workspace.min_shoulder_joint_rad must be smaller than "
            "max_shoulder_joint_rad"
        )

    result = Task2Config(
        input_topic=str(input_cfg.get("topic", "/goal_poses")),
        input_xy_frame=str(input_cfg.get("xy_frame", "board")),
        board_surface_z_m=_finite(board.get("surface_z_m"), "board.surface_z_m"),
        cube_height_m=_positive(cube.get("height_m"), "cube.height_m"),
        xy_correction_base_m=_finite_vector(
            board.get("target_xy_correction_base_m"),
            2,
            "board.target_xy_correction_base_m",
        ),
        goal_board_xy_m=_finite_vector(goal.get("board_xy_m"), 2, "goal.board_xy_m"),
        workspace=workspace,
        approach_clearance_m=_positive(
            planning.get("approach_clearance_m"), "planning.approach_clearance_m"
        ),
        ik_position_tolerance_m=_positive(
            planning.get("ik_position_tolerance_m"),
            "planning.ik_position_tolerance_m",
        ),
        pick_ready_arm_rad=_finite_vector(
            planning.get("pick_ready_arm_rad"), 3, "planning.pick_ready_arm_rad"
        ),
        random_seed=int(planning.get("random_seed")),
        cartesian_max_joint_step_rad=_positive(
            planning.get("cartesian_max_joint_step_rad"),
            "planning.cartesian_max_joint_step_rad",
        ),
        transfer_duration_s=_positive(
            planning.get("transfer_duration_s"), "planning.transfer_duration_s"
        ),
        vertical_duration_s=_positive(
            planning.get("vertical_duration_s"), "planning.vertical_duration_s"
        ),
        gripper_action_duration_s=_positive(
            planning.get("gripper_action_duration_s"),
            "planning.gripper_action_duration_s",
        ),
        return_duration_s=_positive(
            planning.get("return_duration_s"), "planning.return_duration_s"
        ),
        planning_sample_period_s=_positive(
            planning.get("planning_sample_period_s"),
            "planning.planning_sample_period_s",
        ),
        publish_rate_hz=_positive(
            planning.get("publish_rate_hz"), "planning.publish_rate_hz"
        ),
        require_pick_ready_before_request=bool(
            planning.get("require_pick_ready_before_request", True)
        ),
        start_pose_tolerance_rad=_finite_vector(
            planning.get("start_pose_tolerance_rad"),
            3,
            "planning.start_pose_tolerance_rad",
        ),
        start_gripper_tolerance_rad=_positive(
            planning.get("start_gripper_tolerance_rad"),
            "planning.start_gripper_tolerance_rad",
        ),
        joint_state_timeout_s=_positive(
            planning.get("joint_state_timeout_s"), "planning.joint_state_timeout_s"
        ),
        gripper_open_rad=_finite(gripper.get("open_rad"), "gripper.open_rad"),
        simulation_hold_rad=_finite(
            gripper.get("simulation_hold_rad"), "gripper.simulation_hold_rad"
        ),
        hardware_hold_rad=_finite(
            gripper.get("hardware_hold_rad"), "gripper.hardware_hold_rad"
        ),
        hardware_ready_file=Path(hardware_ready_file),
        require_arm_reference=bool(hardware.get("require_arm_reference", True)),
        require_gripper_reference=bool(
            hardware.get("require_gripper_reference", True)
        ),
        require_allow_motion=bool(hardware.get("require_allow_motion", True)),
        board_to_base_xy=dict(board.get("board_to_base_xy", {}) or {}),
    )

    if np.any(result.start_pose_tolerance_rad <= 0.0):
        raise ValueError("planning.start_pose_tolerance_rad values must be positive")
    if result.input_xy_frame != "board":
        raise ValueError("Task 2 point.x/y must be expressed in board coordinates")
    # The fixed Task 1 goal must remain inside the configured feasible region.
    result.validate_workspace_xy(result.goal_base_xy_m, label="Task 2 goal")
    return result
