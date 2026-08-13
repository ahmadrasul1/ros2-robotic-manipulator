from __future__ import annotations

import math
from typing import Any, Iterable

import numpy as np


def transform_board_xy_to_base(
    board_xy_m: Iterable[float],
    *,
    mapping: dict[str, Any],
    correction_base_m: Iterable[float],
) -> np.ndarray:
    """Convert one board-frame XY point into the URDF base_link frame.

    Transformation order:
      1. Apply the configured nominal board-axis mapping/signs.
      2. Rotate the mapped point by yaw_correction_rad around +Z_base.
      3. Add the residual translation correction in base_link axes.
    """

    board_xy = np.asarray(list(board_xy_m), dtype=float)
    if board_xy.shape != (2,) or not np.all(np.isfinite(board_xy)):
        raise ValueError("board_xy_m must contain two finite metre values")

    source = {
        "x_board": float(board_xy[0]),
        "y_board": float(board_xy[1]),
    }

    try:
        x_source = str(mapping["x_base_from"])
        y_source = str(mapping["y_base_from"])
        x_sign = float(mapping["x_base_sign"])
        y_sign = float(mapping["y_base_sign"])
        yaw_rad = float(mapping.get("yaw_correction_rad", 0.0))
        nominal_base_xy = np.asarray(
            [x_sign * source[x_source], y_sign * source[y_source]],
            dtype=float,
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Invalid board.board_to_base_xy mapping") from exc

    if x_source not in source or y_source not in source:
        raise ValueError(
            "board_to_base_xy sources must be 'x_board' or 'y_board'"
        )
    if not np.all(np.isfinite(nominal_base_xy)):
        raise ValueError("board_to-base signs must be finite")
    if not math.isfinite(yaw_rad):
        raise ValueError("board_to_base_xy.yaw_correction_rad must be finite")

    correction = np.asarray(list(correction_base_m), dtype=float)
    if correction.shape != (2,) or not np.all(np.isfinite(correction)):
        raise ValueError(
            "board.target_xy_correction_base_m must contain two finite values"
        )

    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    rotation = np.asarray(
        [
            [cos_yaw, -sin_yaw],
            [sin_yaw, cos_yaw],
        ],
        dtype=float,
    )
    return rotation @ nominal_base_xy + correction
