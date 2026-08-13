#!/usr/bin/env python3
"""Generate WP3 Task 1 joint waypoints with Robotics Toolbox for Python.

The script deliberately separates the two planning stages:

1. Cartesian cube/goal poses -> arm joint waypoints using Robotics Toolbox IK.
2. Joint waypoints -> sampled minimum-jerk CSV using generate_min_jerk_task1.py.

Only the three positioning joints are included in IK:
    shoulder_joint, upperarm_joint, lowerarm_joint

The gear-driven end_effector_joint controls jaw opening and is appended after IK.
The IK mask therefore constrains TCP x/y/z only. This matches the robot's three
positioning degrees of freedom; the gripper orientation is not independently
controllable.

No joint limit is widened by this script. Limits are read directly from the
latest URDF/Xacro. If a requested pose is unreachable, no waypoint file is
written.
"""

from __future__ import annotations

import argparse
import hashlib
import math
import sys
import xml.etree.ElementTree as ET_XML
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from rascl_wp3_ss26_group11.board_transform import (  # noqa: E402
    transform_board_xy_to_base,
)

try:
    import roboticstoolbox as rtb
    from roboticstoolbox import ET
    from spatialmath import SE3
except ImportError as exc:  # pragma: no cover - depends on container setup.
    raise SystemExit(
        "Robotics Toolbox is not installed. Rebuild the Docker image after adding "
        "'roboticstoolbox-python==1.3.1', or run:\n"
        "  python3 -m pip install --break-system-packages "
        "roboticstoolbox-python==1.3.1\n"
        f"Original import error: {exc}"
    ) from exc


ARM_JOINTS = ["shoulder_joint", "upperarm_joint", "lowerarm_joint"]
ALL_JOINTS = [*ARM_JOINTS, "end_effector_joint"]
POSITION_MASK = np.array([1.0, 1.0, 1.0, 0.0, 0.0, 0.0])
KINEMATIC_MODEL_REVISION = "group11_physical_geometry_z_calibration_v2"


@dataclass(frozen=True)
class IKSolution:
    q: np.ndarray
    position_error_m: float
    solver_success: bool
    solver_residual: float


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return data


def parse_float_list(text: str | None, length: int, default: float = 0.0) -> list[float]:
    if text is None:
        return [default] * length
    values = [float(value) for value in text.split()]
    if len(values) != length:
        raise ValueError(f"Expected {length} values, got {values}")
    return values


def static_origin_ets(xyz: Iterable[float], rpy: Iterable[float]):
    """Return an ETS matching a URDF origin transform.

    URDF fixed-axis RPY convention:
        T = Trans(x, y, z) * Rz(yaw) * Ry(pitch) * Rx(roll)
    """

    x, y, z = xyz
    roll, pitch, yaw = rpy
    return (
        ET.tx(x)
        * ET.ty(y)
        * ET.tz(z)
        * ET.Rz(yaw)
        * ET.Ry(pitch)
        * ET.Rx(roll)
    )


def variable_axis_et(axis: Iterable[float], jindex: int):
    axis_array = np.asarray(list(axis), dtype=float)
    norm = np.linalg.norm(axis_array)
    if norm <= 1e-12:
        raise ValueError("Revolute joint axis may not be zero")
    axis_array /= norm

    principal_axes = {
        (1.0, 0.0, 0.0): ET.Rx,
        (0.0, 1.0, 0.0): ET.Ry,
        (0.0, 0.0, 1.0): ET.Rz,
    }
    for expected, constructor in principal_axes.items():
        if np.allclose(axis_array, expected, atol=1e-9):
            return constructor(jindex=jindex)

    negative_axes = {
        (-1.0, 0.0, 0.0): ET.Rx,
        (0.0, -1.0, 0.0): ET.Ry,
        (0.0, 0.0, -1.0): ET.Rz,
    }
    for expected, constructor in negative_axes.items():
        if np.allclose(axis_array, expected, atol=1e-9):
            # RTB's flip flag reverses the joint variable sign.
            return constructor(jindex=jindex, flip=True)

    raise ValueError(
        "This generator currently supports only principal-axis URDF joints. "
        f"Unsupported axis: {axis_array.tolist()}"
    )


def read_joint(root: ET_XML.Element, name: str) -> ET_XML.Element:
    joint = root.find(f".//joint[@name='{name}']")
    if joint is None:
        raise KeyError(f"Joint '{name}' not found in URDF/Xacro")
    return joint


def build_robot_from_urdf(urdf_path: Path, qlim_override: np.ndarray | None = None):
    """Build the base_link -> gripper_tcp ETS directly from the current URDF."""

    root = ET_XML.parse(urdf_path).getroot()
    chain_joint_names = [*ARM_JOINTS, "gripper_tcp_joint"]

    ets = None
    limits: list[list[float]] = []
    joint_index = 0

    for joint_name in chain_joint_names:
        joint = read_joint(root, joint_name)
        joint_type = joint.attrib.get("type", "")

        origin = joint.find("origin")
        xyz = parse_float_list(origin.attrib.get("xyz") if origin is not None else None, 3)
        rpy = parse_float_list(origin.attrib.get("rpy") if origin is not None else None, 3)
        joint_ets = static_origin_ets(xyz, rpy)

        if joint_type in {"revolute", "continuous"}:
            axis_element = joint.find("axis")
            axis = parse_float_list(
                axis_element.attrib.get("xyz") if axis_element is not None else "1 0 0",
                3,
            )
            joint_ets = joint_ets * variable_axis_et(axis, joint_index)
            joint_index += 1

            limit_element = joint.find("limit")
            if joint_type == "continuous":
                limits.append([-math.pi, math.pi])
            elif limit_element is None:
                raise ValueError(f"Joint '{joint_name}' has no <limit> element")
            else:
                limits.append(
                    [
                        float(limit_element.attrib["lower"]),
                        float(limit_element.attrib["upper"]),
                    ]
                )
        elif joint_type != "fixed":
            raise ValueError(
                f"Unexpected joint type '{joint_type}' in TCP chain at '{joint_name}'"
            )

        ets = joint_ets if ets is None else ets * joint_ets

    if joint_index != len(ARM_JOINTS):
        raise ValueError(
            f"Expected {len(ARM_JOINTS)} arm joints, built {joint_index}"
        )

    joint_indices = [joint.jindex for joint in ets.joints()]
    expected_indices = list(range(len(ARM_JOINTS)))

    if joint_indices != expected_indices:
        raise ValueError(
            f"Invalid ETS joint indices: {joint_indices}; "
            f"expected {expected_indices}"
        )

    print(f"ETS joint indices: {joint_indices}")

    robot = rtb.Robot(ets, name="RASCL_Group11_Task1")
    qlim = np.asarray(limits, dtype=float).T
    if qlim.shape != (2, len(ARM_JOINTS)):
        raise ValueError(f"Unexpected joint-limit shape: {qlim.shape}")
    if qlim_override is not None:
        qlim = np.asarray(qlim_override, dtype=float)
    robot.qlim = qlim
    return robot, qlim


def unpack_rtb_solution(result: Any) -> tuple[np.ndarray, bool, float]:
    """Support both current ik_LM tuples and older IKSolution objects."""

    if hasattr(result, "q"):
        q = np.asarray(result.q, dtype=float)
        success = bool(result.success)
        residual = float(getattr(result, "residual", math.inf))
        return q, success, residual

    if isinstance(result, tuple) and len(result) >= 2:
        q = np.asarray(result[0], dtype=float)
        success = bool(result[1])
        residual = float(result[4]) if len(result) > 4 else math.inf
        return q, success, residual

    raise TypeError(f"Unrecognized Robotics Toolbox IK result: {type(result)!r}")


def call_ik(
    robot,
    target: np.ndarray,
    seed: np.ndarray,
    random_seed: int,
):
    """Run position-only LM IK from one deterministic starting pose.

    The outer solve_position() function already tries many different seeds.
    Therefore each call performs one LM search from the supplied q0.
    """

    target_pose = SE3.Trans(*target.tolist())

    result = robot.ikine_LM(
        target_pose,
        q0=seed,
        mask=POSITION_MASK,
        joint_limits=True,
        ilimit=1000,
        slimit=1,
        tol=1e-8,
        seed=random_seed,
        method="chan",
        k=0.01,
    )

    return unpack_rtb_solution(result)

def deterministic_seeds(
    qlim: np.ndarray,
    preferred: np.ndarray,
    previous: np.ndarray | None,
    rng: np.random.Generator,
) -> list[np.ndarray]:
    lower = qlim[0]
    upper = qlim[1]
    centre = 0.5 * (lower + upper)

    seeds: list[np.ndarray] = []
    if previous is not None:
        seeds.append(np.clip(previous, lower, upper))
    seeds.extend([np.clip(preferred, lower, upper), centre])

    # Add deterministic spread across the actual current limits.
    for shoulder_fraction in (0.1, 0.5, 0.9):
        for upper_fraction in (0.15, 0.5, 0.85):
            for lower_fraction in (0.15, 0.5, 0.85):
                fractions = np.array(
                    [shoulder_fraction, upper_fraction, lower_fraction], dtype=float
                )
                seeds.append(lower + fractions * (upper - lower))

    for _ in range(20):
        seeds.append(rng.uniform(lower, upper))

    # Remove near-duplicates while preserving order.
    unique: list[np.ndarray] = []
    for seed in seeds:
        if not any(np.allclose(seed, existing, atol=1e-10) for existing in unique):
            unique.append(seed)
    return unique


def solve_position(
    robot,
    qlim: np.ndarray,
    target: np.ndarray,
    preferred: np.ndarray,
    previous: np.ndarray | None,
    tolerance_m: float,
    random_seed: int,
) -> IKSolution | None:
    rng = np.random.default_rng(random_seed)
    candidates: list[IKSolution] = []

    for seed in deterministic_seeds(qlim, preferred, previous, rng):
        try:
            q, solver_success, solver_residual = call_ik(
                robot, target, seed, random_seed
            )
        except (ValueError, np.linalg.LinAlgError):
            continue

        if q.shape != (len(ARM_JOINTS),) or not np.all(np.isfinite(q)):
            continue
        if np.any(q < qlim[0] - 1e-9) or np.any(q > qlim[1] + 1e-9):
            continue

        actual = np.asarray(robot.fkine(q).t, dtype=float).reshape(3)
        error = float(np.linalg.norm(actual - target))
        if error <= tolerance_m:
            candidates.append(
                IKSolution(
                    q=q,
                    position_error_m=error,
                    solver_success=solver_success,
                    solver_residual=solver_residual,
                )
            )

    if not candidates:
        return None

    continuity_reference = previous if previous is not None else preferred
    return min(
        candidates,
        key=lambda item: (
            float(np.linalg.norm(item.q - continuity_reference)),
            item.position_error_m,
        ),
    )


def board_xy_to_base(config: dict[str, Any], board_xy: Iterable[float]) -> np.ndarray:
    board_config = config["board"]
    return transform_board_xy_to_base(
        board_xy,
        mapping=board_config["board_to_base_xy"],
        correction_base_m=board_config.get(
            "target_xy_correction_base_m",
            [0.0, 0.0],
        ),
    )


def make_cartesian_targets(config: dict[str, Any]) -> dict[str, np.ndarray]:
    board_z_raw = config["board"].get("surface_z_m")
    if board_z_raw is None:
        raise ValueError(
            "board.surface_z_m is null. Measure the cardboard thickness and put "
            "the measured value in config/task1_cube_poses.yaml before generating IK."
        )

    board_z = float(board_z_raw)
    target_z_correction = float(
        config["board"].get("target_z_correction_m", 0.0)
    )
    if not math.isfinite(target_z_correction):
        raise ValueError("board.target_z_correction_m must be finite")

    location_z_raw = config["board"].get("location_z_correction_m", {}) or {}
    if not isinstance(location_z_raw, dict):
        raise ValueError("board.location_z_correction_m must be a mapping")

    # Backward compatible: older files used one `cube_locations` correction.
    # New files can calibrate each physical station independently because the
    # far cube and the near cubes do not have the same reachable Z envelope.
    legacy_cube_correction = float(location_z_raw.get("cube_locations", 0.0))
    location_z_corrections = {
        "cube1_start": float(
            location_z_raw.get("cube1_start", legacy_cube_correction)
        ),
        "cube2_3_start": float(
            location_z_raw.get("cube2_3_start", legacy_cube_correction)
        ),
        "cube3_buffer": float(
            location_z_raw.get("cube3_buffer", legacy_cube_correction)
        ),
        "goal": float(location_z_raw.get("goal", 0.0)),
    }
    if not all(math.isfinite(value) for value in location_z_corrections.values()):
        raise ValueError("board.location_z_correction_m values must be finite")

    cube_height = float(config["cube"]["height_m"])
    clearance = float(config["planning"]["approach_clearance_m"])
    xy = config["poses_board_xy_m"]

    level_board = board_z + 0.5 * cube_height
    level_second = board_z + 1.5 * cube_height
    level_third = board_z + 2.5 * cube_height

    def target(board_key: str, z: float) -> np.ndarray:
        base_xy = board_xy_to_base(config, xy[board_key])
        return np.array(
            [
                base_xy[0],
                base_xy[1],
                z + target_z_correction + location_z_corrections[board_key],
            ],
            dtype=float,
        )

    return {
        "above_cube3_start": target("cube2_3_start", level_second + clearance),
        "grasp_cube3_start": target("cube2_3_start", level_second),
        "above_cube3_buffer": target("cube3_buffer", level_board + clearance),
        "grasp_cube3_buffer": target("cube3_buffer", level_board),
        "above_cube1": target("cube1_start", level_board + clearance),
        "grasp_cube1": target("cube1_start", level_board),
        "above_goal_bottom": target("goal", level_board + clearance),
        "place_goal_bottom": target("goal", level_board),
        "above_cube2_start": target("cube2_3_start", level_board + clearance),
        "grasp_cube2_start": target("cube2_3_start", level_board),
        "above_goal_middle": target("goal", level_second + clearance),
        "place_goal_middle": target("goal", level_second),
        "above_goal_top": target("goal", level_third + clearance),
        "place_goal_top": target("goal", level_third),
    }


def add_gripper(q_arm: np.ndarray, gripper_rad: float) -> list[float]:
    return [float(value) for value in q_arm] + [float(gripper_rad)]


def create_output_yaml(
    config: dict[str, Any],
    cartesian_targets: dict[str, np.ndarray],
    arm_solutions: dict[str, IKSolution],
    gripper_mode: str,
    poses_sha256: str,
    urdf_sha256: str,
) -> dict[str, Any]:
    gripper = config["gripper"]
    open_rad = float(gripper["open_rad"])

    if gripper_mode == "simulation":
        hold_rad = float(gripper["geometric_contact_rad"])
    else:
        hold_raw = gripper.get("hardware_hold_rad")
        if hold_raw is None:
            raise ValueError(
                "gripper.hardware_hold_rad is null. Calibrate it physically before "
                "generating hardware waypoints."
            )
        hold_rad = float(hold_raw)

    preferred = np.asarray(config["planning"]["pick_ready_arm_rad"], dtype=float)

    waypoints: dict[str, list[float]] = {
        "pick_ready": add_gripper(preferred, open_rad),
    }

    for base_name, solution in arm_solutions.items():
        waypoints[f"{base_name}_open"] = add_gripper(solution.q, open_rad)
        waypoints[f"{base_name}_hold"] = add_gripper(solution.q, hold_rad)

    trajectory = [
        # Move Cube 3 off Cube 2 and into the temporary buffer.
        {"name": "pick_ready_to_above_cube3_start_open", "from": "pick_ready", "to": "above_cube3_start_open", "duration": 5.0, "gripper": 0.0},
        {"name": "descend_cube3_start_open", "from": "above_cube3_start_open", "to": "grasp_cube3_start_open", "duration": 3.0, "gripper": 0.0, "interpolation": "cartesian_linear"},
        {"name": "close_cube3_start", "from": "grasp_cube3_start_open", "to": "grasp_cube3_start_hold", "duration": 6.0, "gripper": 1.0},
        {"name": "lift_cube3_start", "from": "grasp_cube3_start_hold", "to": "above_cube3_start_hold", "duration": 3.0, "gripper": 1.0, "interpolation": "cartesian_linear"},
        {"name": "transfer_cube3_to_buffer", "from": "above_cube3_start_hold", "to": "above_cube3_buffer_hold", "duration": 5.0, "gripper": 1.0},
        {"name": "descend_cube3_buffer", "from": "above_cube3_buffer_hold", "to": "grasp_cube3_buffer_hold", "duration": 3.0, "gripper": 1.0, "interpolation": "cartesian_linear"},
        {"name": "open_cube3_buffer", "from": "grasp_cube3_buffer_hold", "to": "grasp_cube3_buffer_open", "duration": 6.0, "gripper": 0.0},
        {"name": "retreat_cube3_buffer", "from": "grasp_cube3_buffer_open", "to": "above_cube3_buffer_open", "duration": 3.0, "gripper": 0.0, "interpolation": "cartesian_linear"},

        # Move Cube 1 to the bottom of the goal stack.
        {"name": "buffer_to_above_cube1", "from": "above_cube3_buffer_open", "to": "above_cube1_open", "duration": 5.0, "gripper": 0.0},
        {"name": "descend_cube1", "from": "above_cube1_open", "to": "grasp_cube1_open", "duration": 3.0, "gripper": 0.0, "interpolation": "cartesian_linear"},
        {"name": "close_cube1", "from": "grasp_cube1_open", "to": "grasp_cube1_hold", "duration": 6.0, "gripper": 1.0},
        {"name": "lift_cube1", "from": "grasp_cube1_hold", "to": "above_cube1_hold", "duration": 3.0, "gripper": 1.0, "interpolation": "cartesian_linear"},
        {"name": "transfer_cube1_to_goal", "from": "above_cube1_hold", "to": "above_goal_bottom_hold", "duration": 5.0, "gripper": 1.0},
        {"name": "descend_goal_bottom", "from": "above_goal_bottom_hold", "to": "place_goal_bottom_hold", "duration": 3.0, "gripper": 1.0, "interpolation": "cartesian_linear"},
        {"name": "open_goal_bottom", "from": "place_goal_bottom_hold", "to": "place_goal_bottom_open", "duration": 6.0, "gripper": 0.0},
        {"name": "retreat_goal_bottom", "from": "place_goal_bottom_open", "to": "above_goal_bottom_open", "duration": 3.0, "gripper": 0.0, "interpolation": "cartesian_linear"},

        # Move Cube 2 onto Cube 1.
        {"name": "goal_to_above_cube2", "from": "above_goal_bottom_open", "to": "above_cube2_start_open", "duration": 5.0, "gripper": 0.0},
        {"name": "descend_cube2", "from": "above_cube2_start_open", "to": "grasp_cube2_start_open", "duration": 3.0, "gripper": 0.0, "interpolation": "cartesian_linear"},
        {"name": "close_cube2", "from": "grasp_cube2_start_open", "to": "grasp_cube2_start_hold", "duration": 6.0, "gripper": 1.0},
        {"name": "lift_cube2", "from": "grasp_cube2_start_hold", "to": "above_cube2_start_hold", "duration": 3.0, "gripper": 1.0, "interpolation": "cartesian_linear"},
        {"name": "transfer_cube2_to_goal", "from": "above_cube2_start_hold", "to": "above_goal_middle_hold", "duration": 5.0, "gripper": 1.0},
        {"name": "descend_goal_middle", "from": "above_goal_middle_hold", "to": "place_goal_middle_hold", "duration": 3.0, "gripper": 1.0, "interpolation": "cartesian_linear"},
        {"name": "open_goal_middle", "from": "place_goal_middle_hold", "to": "place_goal_middle_open", "duration": 8.0, "gripper": 0.0},
        {"name": "retreat_goal_middle", "from": "place_goal_middle_open", "to": "above_goal_middle_open", "duration": 3.0, "gripper": 0.0, "interpolation": "cartesian_linear"},

        # Pick Cube 3 from the buffer and place it on top.
        {"name": "goal_to_above_cube3_buffer", "from": "above_goal_middle_open", "to": "above_cube3_buffer_open", "duration": 5.0, "gripper": 0.0},
        {"name": "descend_cube3_buffer_for_goal", "from": "above_cube3_buffer_open", "to": "grasp_cube3_buffer_open", "duration": 3.0, "gripper": 0.0, "interpolation": "cartesian_linear"},
        {"name": "close_cube3_buffer", "from": "grasp_cube3_buffer_open", "to": "grasp_cube3_buffer_hold", "duration": 6.0, "gripper": 1.0},
        {"name": "lift_cube3_buffer", "from": "grasp_cube3_buffer_hold", "to": "above_cube3_buffer_hold", "duration": 3.0, "gripper": 1.0, "interpolation": "cartesian_linear"},
        {"name": "transfer_cube3_to_goal", "from": "above_cube3_buffer_hold", "to": "above_goal_top_hold", "duration": 5.0, "gripper": 1.0},
        {"name": "descend_goal_top", "from": "above_goal_top_hold", "to": "place_goal_top_hold", "duration": 3.0, "gripper": 1.0, "interpolation": "cartesian_linear"},
        {"name": "open_goal_top", "from": "place_goal_top_hold", "to": "place_goal_top_open", "duration": 6.0, "gripper": 0.0},
        {"name": "retreat_goal_top", "from": "place_goal_top_open", "to": "above_goal_top_open", "duration": 3.0, "gripper": 0.0, "interpolation": "cartesian_linear"},
        {"name": "final_tower_hold_3_seconds", "from": "above_goal_top_open", "to": "above_goal_top_open", "duration": 3.0, "gripper": 0.0},
        {"name": "return_to_pick_ready", "from": "above_goal_top_open", "to": "pick_ready", "duration": 5.0, "gripper": 0.0},
    ]

    return {
        "metadata": {
            "generated_by": "generate_task1_waypoints_rtb.py",
            "ik_library": "Robotics Toolbox for Python",
            "ik_method": "Levenberg-Marquardt, position-only mask",
            "kinematic_model_revision": KINEMATIC_MODEL_REVISION,
            "gripper_mode": gripper_mode,
            "poses_sha256": poses_sha256,
            "urdf_sha256": urdf_sha256,
            "joint_order": ALL_JOINTS,
        },
        "settings": {
            "dt": 0.05,
            "default_duration": 5.0,
            "default_gripper_action_duration": 6.0,
            "ik_position_tolerance_m": float(config["planning"]["position_tolerance_m"]),
            "target_z_correction_m": float(
                config["board"].get("target_z_correction_m", 0.0)
            ),
            "location_z_correction_m": {
                key: float(value)
                for key, value in (
                    config["board"].get("location_z_correction_m", {}) or {}
                ).items()
            },
            "target_xy_correction_base_m": [
                float(value)
                for value in config["board"].get(
                    "target_xy_correction_base_m", [0.0, 0.0]
                )
            ],
        },
        "cartesian_targets_base_m": {
            name: [float(value) for value in target]
            for name, target in cartesian_targets.items()
        },
        "ik_position_error_m": {
            name: float(solution.position_error_m)
            for name, solution in arm_solutions.items()
        },
        "waypoints": waypoints,
        "trajectories": {"task1_full": trajectory},
    }


def diagnose_relaxed_limits(
    urdf_path: Path,
    target_name: str,
    target: np.ndarray,
    preferred: np.ndarray,
    tolerance_m: float,
    random_seed: int,
) -> str:
    """Estimate a solution without changing the real URDF limits.

    This is diagnostic only. It helps identify which existing limits block a
    target; the returned values must never be copied into hardware limits without
    physical validation.
    """

    relaxed = np.array(
        [
            [-math.pi / 2.0, -math.pi, -math.pi],
            [math.pi / 2.0, math.pi, math.pi],
        ],
        dtype=float,
    )
    robot, qlim = build_robot_from_urdf(urdf_path, qlim_override=relaxed)
    solution = solve_position(
        robot=robot,
        qlim=qlim,
        target=target,
        preferred=preferred,
        previous=None,
        tolerance_m=tolerance_m,
        random_seed=random_seed,
    )
    if solution is None:
        return f"{target_name}: no relaxed-limit position solution found"
    q_text = ", ".join(
        f"{name}={value:.6f}" for name, value in zip(ARM_JOINTS, solution.q)
    )
    return f"{target_name}: relaxed diagnostic solution [{q_text}]"


def main() -> None:
    script_path = Path(__file__).resolve()
    package_root = script_path.parents[1]
    workspace_src = package_root.parent

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--poses",
        type=Path,
        default=package_root / "config" / "task1_cube_poses.yaml",
    )
    parser.add_argument(
        "--urdf",
        type=Path,
        default=workspace_src / "rascl_description" / "urdf" / "rascl.urdf.xacro",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=package_root / "trajectories" / "task1" / "input_waypoints_ik_sim.yaml",
    )
    parser.add_argument(
        "--gripper-mode",
        choices=["simulation", "hardware"],
        default="simulation",
    )
    parser.add_argument(
        "--diagnose-relaxed-limits",
        action="store_true",
        help="For unreachable targets, report a diagnostic solution with relaxed arm limits. Never changes the URDF.",
    )
    args = parser.parse_args()

    config = load_yaml(args.poses)
    cartesian_targets = make_cartesian_targets(config)
    location_z = config["board"].get("location_z_correction_m", {}) or {}
    legacy_cube = float(location_z.get("cube_locations", 0.0))
    print(
        "Active Task 1 Z corrections: "
        f"global={float(config['board'].get('target_z_correction_m', 0.0)):+.3f} m, "
        f"cube1_start={float(location_z.get('cube1_start', legacy_cube)):+.3f} m, "
        f"cube2_3_start={float(location_z.get('cube2_3_start', legacy_cube)):+.3f} m, "
        f"cube3_buffer={float(location_z.get('cube3_buffer', legacy_cube)):+.3f} m, "
        f"goal={float(location_z.get('goal', 0.0)):+.3f} m",
        flush=True,
    )
    preferred = np.asarray(config["planning"]["pick_ready_arm_rad"], dtype=float)
    tolerance_m = float(config["planning"]["position_tolerance_m"])
    random_seed = int(config["planning"].get("random_seed", 11))

    if preferred.shape != (len(ARM_JOINTS),):
        raise ValueError(
            f"planning.pick_ready_arm_rad must contain {len(ARM_JOINTS)} values"
        )

    robot, qlim = build_robot_from_urdf(args.urdf)
    print("Robotics Toolbox model built from:", args.urdf)
    print("Current URDF arm limits:")
    for index, name in enumerate(ARM_JOINTS):
        print(f"  {name}: [{qlim[0, index]:.6f}, {qlim[1, index]:.6f}] rad")

    arm_solutions: dict[str, IKSolution] = {}
    unreachable: list[tuple[str, np.ndarray]] = []
    previous: np.ndarray | None = preferred

    for index, (name, target) in enumerate(cartesian_targets.items()):
        solution = solve_position(
            robot=robot,
            qlim=qlim,
            target=target,
            preferred=preferred,
            previous=previous,
            tolerance_m=tolerance_m,
            random_seed=random_seed + index,
        )
        if solution is None:
            unreachable.append((name, target))
            print(
                f"[UNREACHABLE] {name}: target="
                f"[{target[0]:.6f}, {target[1]:.6f}, {target[2]:.6f}] m"
            )
            continue

        arm_solutions[name] = solution
        previous = solution.q
        q_text = ", ".join(f"{value:.6f}" for value in solution.q)
        print(
            f"[OK] {name}: q=[{q_text}], "
            f"position_error={solution.position_error_m * 1000.0:.3f} mm"
        )

    if unreachable:
        print("\nNo waypoint file was written because one or more poses violate the current model/limits.")
        if args.diagnose_relaxed_limits:
            print("\nRelaxed-limit diagnostics only (do not use as hardware limits):")
            for name, target in unreachable:
                print(
                    "  "
                    + diagnose_relaxed_limits(
                        urdf_path=args.urdf,
                        target_name=name,
                        target=target,
                        preferred=preferred,
                        tolerance_m=tolerance_m,
                        random_seed=random_seed,
                    )
                )
        raise SystemExit(2)

    output_data = create_output_yaml(
        config=config,
        cartesian_targets=cartesian_targets,
        arm_solutions=arm_solutions,
        gripper_mode=args.gripper_mode,
        poses_sha256=hashlib.sha256(args.poses.read_bytes()).hexdigest(),
        urdf_sha256=hashlib.sha256(args.urdf.read_bytes()).hexdigest(),
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(output_data, handle, sort_keys=False, width=120)

    print(f"\nWrote verified joint waypoints to: {args.output}")
    print("Next generate the sampled minimum-jerk CSV with:")
    print(
        "  python3 src/rascl_wp3_ss26_group11/scripts/generate_min_jerk_task1.py "
        "--input src/rascl_wp3_ss26_group11/trajectories/task1/input_waypoints_ik_sim.yaml "
        "--trajectory task1_full"
    )


if __name__ == "__main__":
    main()
