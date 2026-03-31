#!/usr/bin/env python3
"""Clean starter node for the pel_ros2 package."""

from __future__ import annotations

import rclpy
from rclpy.node import Node


class PelRosNode(Node):
    """Minimal node used as a clean starting point for new work."""

    def __init__(self) -> None:
        super().__init__("pel_ros2")
        self.create_timer(5.0, self._heartbeat)
        self.get_logger().info("pel_ros2 is running as a clean starter package.")

    def _heartbeat(self) -> None:
        """Log a low-frequency heartbeat while the node is alive."""
        self.get_logger().debug("pel_ros2 heartbeat")


def main(args: list[str] | None = None) -> None:
    """Run the starter node."""
    rclpy.init(args=args)
    node = PelRosNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
