"""Launch the starter node and ORKA base-graph service."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Create a minimal launch description for pel_ros2."""
    return LaunchDescription(
        [
            Node(
                package="pel_ros2",
                executable="base_graph_service",
                name="base_graph_service",
                output="screen",
            ),
            Node(
                package="pel_ros2",
                executable="observer",
                name="observer",
                output="screen",
            ),
            Node(
                package="pel_ros2",
                executable="observation_graph_manager",
                name="observation_graph_manager",
                output="screen",
            ),
            Node(
                package="pel_ros2",
                executable="main",
                name="pel_ros2",
                output="screen",
            )
        ]
    )
