"""Optional Gazebo Sim scene alongside ROS2Drive's native runtime.

This is a visualization / integration scaffold, NOT a coupled Gazebo vehicle
dynamics backend. Gazebo runs its own world; ROS2Drive keeps its native physics.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = Path(get_package_share_directory("self_driving_car_demo"))
    world = pkg / "gazebo" / "worlds" / "empty.sdf"
    if not world.is_file():
        raise RuntimeError(f"Missing installed Gazebo world: {world}")
    # Keep ros_gz_sim optional: importing the package only when this launch runs.
    gazebo_pkg = Path(get_package_share_directory("ros_gz_sim"))
    return LaunchDescription([
        DeclareLaunchArgument("run_native", default_value="false",
                              description="Also launch the existing native ROS2Drive runtime"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_pkg / "launch" / "gz_sim.launch.py")),
            launch_arguments={"gz_args": "-r " + str(world)}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pkg / "launch" / "ring_road.launch.py")),
            condition=IfCondition(LaunchConfiguration("run_native")),
            launch_arguments={"use_rviz": LaunchConfiguration("use_rviz")}.items(),
        ),
    ])
