"""Launch the clean pel_ros2 starter node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Create a minimal launch description for pel_ros2."""
    return LaunchDescription(
        [
            Node(
                package="pel_ros2",
                executable="main",
                name="pel_ros2",
                output="screen",
            )
        ]
    )
