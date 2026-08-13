from __future__ import annotations

import math
import xml.etree.ElementTree as ET_XML
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np

try:
    import roboticstoolbox as rtb
    from roboticstoolbox import ET
    from spatialmath import SE3
except ImportError as exc:  # pragma: no cover - depends on the ROS container.
    raise ImportError(
        "Task 2 requires roboticstoolbox-python==1.3.1 and spatialmath-python. "
        "Rebuild the supplied Docker image before running wp3_tsk2. "
        f"Original import error: {exc}"
    ) from exc


ARM_JOINTS = ["shoulder_joint", "upperarm_joint", "lowerarm_joint"]
ALL_JOINTS = [*ARM_JOINTS, "end_effector_joint"]
POSITION_MASK = np.asarray([1.0, 1.0, 1.0, 0.0, 0.0, 0.0], dtype=float)


@dataclass(frozen=True)
class IKSolution:
    q: np.ndarray
    position_error_m: float
    solver_success: bool
    solver_residual: float


def _parse_float_list(
    text: str | None,
    length: int,
    default: float = 0.0,
) -> list[float]:
    if text is None:
        return [default] * length
    values = [float(value) for value in text.split()]
    if len(values) != length:
        raise ValueError(f"Expected {length} values, got {values}")
    return values


def _static_origin_ets(xyz: Iterable[float], rpy: Iterable[float]):
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


def _variable_axis_et(axis: Iterable[float], joint_index: int):
    axis_array = np.asarray(list(axis), dtype=float)
    norm = float(np.linalg.norm(axis_array))
    if norm <= 1e-12:
        raise ValueError("Revolute joint axis may not be zero")
    axis_array /= norm

    positive = {
        (1.0, 0.0, 0.0): ET.Rx,
        (0.0, 1.0, 0.0): ET.Ry,
        (0.0, 0.0, 1.0): ET.Rz,
    }
    for expected, constructor in positive.items():
        if np.allclose(axis_array, expected, atol=1e-9):
            return constructor(jindex=joint_index)

    negative = {
        (-1.0, 0.0, 0.0): ET.Rx,
        (0.0, -1.0, 0.0): ET.Ry,
        (0.0, 0.0, -1.0): ET.Rz,
    }
    for expected, constructor in negative.items():
        if np.allclose(axis_array, expected, atol=1e-9):
            return constructor(jindex=joint_index, flip=True)

    raise ValueError(
        "Task 2 supports only principal-axis URDF joints. "
        f"Unsupported axis: {axis_array.tolist()}"
    )


def _read_joint(root: ET_XML.Element, name: str) -> ET_XML.Element:
    joint = root.find(f".//joint[@name='{name}']")
    if joint is None:
        raise KeyError(f"Joint {name!r} not found in URDF/Xacro")
    return joint


def build_robot_from_urdf(urdf_path: str | Path):
    """Build the unchanged base_link -> gripper_tcp arm model from the URDF."""
    path = Path(urdf_path)
    root = ET_XML.parse(path).getroot()
    chain_joint_names = [*ARM_JOINTS, "gripper_tcp_joint"]

    ets = None
    limits: list[list[float]] = []
    joint_index = 0

    for joint_name in chain_joint_names:
        joint = _read_joint(root, joint_name)
        joint_type = joint.attrib.get("type", "")
        origin = joint.find("origin")
        xyz = _parse_float_list(
            origin.attrib.get("xyz") if origin is not None else None,
            3,
        )
        rpy = _parse_float_list(
            origin.attrib.get("rpy") if origin is not None else None,
            3,
        )
        joint_ets = _static_origin_ets(xyz, rpy)

        if joint_type in {"revolute", "continuous"}:
            axis_element = joint.find("axis")
            axis = _parse_float_list(
                axis_element.attrib.get("xyz")
                if axis_element is not None
                else "1 0 0",
                3,
            )
            joint_ets = joint_ets * _variable_axis_et(axis, joint_index)
            joint_index += 1

            limit_element = joint.find("limit")
            if joint_type == "continuous":
                limits.append([-math.pi, math.pi])
            elif limit_element is None:
                raise ValueError(f"Joint {joint_name!r} has no <limit> element")
            else:
                limits.append(
                    [
                        float(limit_element.attrib["lower"]),
                        float(limit_element.attrib["upper"]),
                    ]
                )
        elif joint_type != "fixed":
            raise ValueError(
                f"Unexpected joint type {joint_type!r} in TCP chain at {joint_name!r}"
            )

        ets = joint_ets if ets is None else ets * joint_ets

    if joint_index != len(ARM_JOINTS):
        raise ValueError(
            f"Expected {len(ARM_JOINTS)} arm joints, built {joint_index}"
        )
    assert ets is not None
    indices = [joint.jindex for joint in ets.joints()]
    expected = list(range(len(ARM_JOINTS)))
    if indices != expected:
        raise ValueError(f"Invalid ETS joint indices {indices}; expected {expected}")

    robot = rtb.Robot(ets, name="RASCL_Group11_Task2")
    qlim = np.asarray(limits, dtype=float).T
    if qlim.shape != (2, len(ARM_JOINTS)):
        raise ValueError(f"Unexpected arm joint-limit shape: {qlim.shape}")
    robot.qlim = qlim
    return robot, qlim


def _unpack_solution(result: Any) -> tuple[np.ndarray, bool, float]:
    if hasattr(result, "q"):
        return (
            np.asarray(result.q, dtype=float),
            bool(result.success),
            float(getattr(result, "residual", math.inf)),
        )
    if isinstance(result, tuple) and len(result) >= 2:
        return (
            np.asarray(result[0], dtype=float),
            bool(result[1]),
            float(result[4]) if len(result) > 4 else math.inf,
        )
    raise TypeError(f"Unrecognized Robotics Toolbox IK result: {type(result)!r}")


def _call_ik(robot, target_m: np.ndarray, seed: np.ndarray, random_seed: int):
    result = robot.ikine_LM(
        SE3.Trans(*target_m.tolist()),
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
    return _unpack_solution(result)


def _deterministic_seeds(
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

    for shoulder_fraction in (0.1, 0.5, 0.9):
        for upper_fraction in (0.15, 0.5, 0.85):
            for lower_fraction in (0.15, 0.5, 0.85):
                fractions = np.asarray(
                    [shoulder_fraction, upper_fraction, lower_fraction],
                    dtype=float,
                )
                seeds.append(lower + fractions * (upper - lower))

    for _ in range(20):
        seeds.append(rng.uniform(lower, upper))

    unique: list[np.ndarray] = []
    for seed in seeds:
        if not any(np.allclose(seed, item, atol=1e-10) for item in unique):
            unique.append(seed)
    return unique


class Task2Kinematics:
    """Task 2-only IK wrapper; the frozen Task 1 generator is not imported."""

    def __init__(self, urdf_path: str | Path) -> None:
        self.robot, self.qlim = build_robot_from_urdf(urdf_path)

    def forward_position(self, q_arm: np.ndarray) -> np.ndarray:
        q = np.asarray(q_arm, dtype=float)
        if q.shape != (len(ARM_JOINTS),):
            raise ValueError("Arm configuration must contain three values")
        return np.asarray(self.robot.fkine(q).t, dtype=float).reshape(3)

    def solve_position(
        self,
        *,
        target_m: np.ndarray,
        preferred: np.ndarray,
        previous: np.ndarray | None,
        tolerance_m: float,
        random_seed: int,
    ) -> IKSolution | None:
        target = np.asarray(target_m, dtype=float)
        preferred_q = np.asarray(preferred, dtype=float)
        previous_q = None if previous is None else np.asarray(previous, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError("IK target must contain three finite metre values")
        if preferred_q.shape != (len(ARM_JOINTS),):
            raise ValueError("IK preferred seed must contain three joint values")

        rng = np.random.default_rng(random_seed)
        candidates: list[IKSolution] = []
        for seed in _deterministic_seeds(
            self.qlim,
            preferred_q,
            previous_q,
            rng,
        ):
            try:
                q, success, residual = _call_ik(
                    self.robot,
                    target,
                    seed,
                    random_seed,
                )
            except (ValueError, np.linalg.LinAlgError):
                continue

            if q.shape != (len(ARM_JOINTS),) or not np.all(np.isfinite(q)):
                continue
            if np.any(q < self.qlim[0] - 1e-9) or np.any(q > self.qlim[1] + 1e-9):
                continue

            error = float(np.linalg.norm(self.forward_position(q) - target))
            if error <= tolerance_m:
                candidates.append(
                    IKSolution(
                        q=q,
                        position_error_m=error,
                        solver_success=success,
                        solver_residual=residual,
                    )
                )

        if not candidates:
            return None
        reference = previous_q if previous_q is not None else preferred_q
        return min(
            candidates,
            key=lambda candidate: (
                float(np.linalg.norm(candidate.q - reference)),
                candidate.position_error_m,
            ),
        )

    def continuous_sample(
        self,
        *,
        target_m: np.ndarray,
        previous: np.ndarray,
        nominal: np.ndarray,
        tolerance_m: float,
        random_seed: int,
    ) -> np.ndarray:
        """Solve one dense Cartesian sample while preferring the same IK branch."""
        target = np.asarray(target_m, dtype=float)
        previous_q = np.asarray(previous, dtype=float)
        nominal_q = np.asarray(nominal, dtype=float)
        candidates: list[np.ndarray] = []

        for seed in (previous_q, nominal_q):
            try:
                q, _success, _residual = _call_ik(
                    self.robot,
                    target,
                    seed,
                    random_seed,
                )
            except (ValueError, np.linalg.LinAlgError):
                continue
            if q.shape != (len(ARM_JOINTS),) or not np.all(np.isfinite(q)):
                continue
            if np.any(q < self.qlim[0] - 1e-9) or np.any(q > self.qlim[1] + 1e-9):
                continue
            error = float(np.linalg.norm(self.forward_position(q) - target))
            if error <= tolerance_m:
                candidates.append(q)

        if not candidates:
            fallback = self.solve_position(
                target_m=target,
                preferred=nominal_q,
                previous=previous_q,
                tolerance_m=tolerance_m,
                random_seed=random_seed,
            )
            if fallback is None:
                raise RuntimeError(
                    "Continuous Cartesian IK failed for target "
                    f"[{target[0]:.6f}, {target[1]:.6f}, {target[2]:.6f}] m"
                )
            candidates.append(fallback.q)

        return min(
            candidates,
            key=lambda q: float(np.linalg.norm(q - previous_q)),
        )
