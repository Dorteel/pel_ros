#!/usr/bin/env python3

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from pel_ros2.srv import Observe


class PelRosNode(Node):

    def __init__(self) -> None:
        super().__init__("pel_ros2")

        self.client = self.create_client(
            Observe,
            "observe",
        )

        self.get_logger().info("Waiting for /observe service...")

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("/observe service not available yet...")

        self.get_logger().info("/observe service connected.")

        # Call observation periodically
        self.create_timer(5.0, self.call_observe_service)

    def call_observe_service(self) -> None:
        request = Observe.Request()

        # Empty list means: observe all sensors
        request.sensors = []

        self.get_logger().info("Calling observation service...")

        start_time = time.perf_counter()

        future = self.client.call_async(request)

        future.add_done_callback(
            lambda f: self.observe_response_callback(f, start_time)
        )

    def observe_response_callback(self, future, start_time) -> None:
        elapsed = time.perf_counter() - start_time

        try:
            response = future.result()

            self.get_logger().info(
                f"Observation service completed in "
                f"{elapsed:.3f} seconds"
            )

            if response.success:
                self.get_logger().info(response.message)

                for sensor, path in zip(
                    response.sensor_names,
                    response.file_paths,
                ):
                    self.get_logger().info(
                        f"{sensor} -> {path}"
                    )

            else:
                self.get_logger().warn(response.message)

        except Exception as e:
            self.get_logger().error(
                f"Service call failed: {e}"
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    node = PelRosNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()