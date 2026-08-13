from __future__ import annotations

import argparse
import math
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node

from .task2_config import load_task2_config


class OneShotCubePublisher(Node):
    def __init__(self, *, topic: str, x_m: float, y_m: float) -> None:
        super().__init__("task2_cube_publisher")
        self._publisher = self.create_publisher(Point, topic, 10)
        self._message = Point(x=float(x_m), y=float(y_m), z=0.0)
        self._sent = False
        self._timer = self.create_timer(0.25, self._publish_once)

    def _publish_once(self) -> None:
        if self._sent:
            return
        self._publisher.publish(self._message)
        self.get_logger().info(
            f"Published Task 2 cube board coordinates "
            f"x_board={self._message.x:.6f} m, "
            f"y_board={self._message.y:.6f} m; z=0.0 is unused"
        )
        self._sent = True
        self._timer.cancel()


def main(args: list[str] | None = None) -> None:
    default_config = (
        Path(get_package_share_directory("rascl_wp3_ss26_group11"))
        / "config"
        / "task2_online_planning.yaml"
    )
    parser = argparse.ArgumentParser(
        description=(
            "Publish one Task 2 cube position. Only X/Y are used; the Point.z "
            "field is sent as 0.0 and ignored by wp3_tsk2."
        )
    )
    parser.add_argument(
        "--x",
        type=float,
        required=True,
        help="Cube X coordinate in the board convention, in m",
    )
    parser.add_argument(
        "--y",
        type=float,
        required=True,
        help="Cube Y coordinate in the board convention, in m",
    )
    parser.add_argument("--config", type=Path, default=default_config)
    parsed, ros_args = parser.parse_known_args(args)

    if not math.isfinite(parsed.x) or not math.isfinite(parsed.y):
        raise SystemExit("--x and --y must be finite metre values")

    config = load_task2_config(parsed.config)
    board_xy = [parsed.x, parsed.y]
    base_xy = config.board_xy_to_base(board_xy)
    radius, angle = config.validate_workspace_xy(base_xy, label="Published cube")
    yaw_rad = float(config.board_to_base_xy.get("yaw_correction_rad", 0.0))
    print(
        "Task 2 cube preview:\n"
        f"  board XY:               [{parsed.x:.6f}, {parsed.y:.6f}] m\n"
        f"  board yaw correction:   {yaw_rad:+.6f} rad\n"
        f"  base XY correction:     [{config.xy_correction_base_m[0]:+.6f}, "
        f"{config.xy_correction_base_m[1]:+.6f}] m\n"
        f"  transformed base XY:    [{base_xy[0]:.6f}, {base_xy[1]:.6f}] m\n"
        f"  fixed center Z:         {config.cube_center_z_m:.6f} m\n"
        f"  radius / angle:         {radius:.6f} m / {angle:.6f} rad"
    )

    rclpy.init(args=ros_args)
    node = OneShotCubePublisher(topic=config.input_topic, x_m=parsed.x, y_m=parsed.y)
    try:
        while rclpy.ok() and not node._sent:  # one-shot helper, no persistent API.
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
