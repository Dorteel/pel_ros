"""Launch the observer stack and optionally start the simulation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create the observer stack launch description."""
    run_simulation = LaunchConfiguration("run_simulation")

    simulation_enabled = PythonExpression(
        [
            "'",
            run_simulation,
            "'.lower() in ['yes', 'true', '1']",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "run_simulation",
                default_value="yes",
                description="Start general_robot_planner simulation.py when yes/true/1.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("general_robot_planner"),
                            "launch",
                            "simulation.py",
                        ]
                    )
                ),
                condition=IfCondition(simulation_enabled),
            ),
            Node(
                package="pel_ros2",
                executable="observer",
                name="observer",
                output="screen",
            ),
            Node(
                package="pel_ros2",
                executable="main",
                name="pel_ros2",
                output="screen",
            ),
        ]
    )
