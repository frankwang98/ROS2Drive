"""
启动环形道路仿真（RViz2 可视化）

用法：
  ros2 launch self_driving_car_demo ring_road.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("self_driving_car_demo")

    # 默认 RViz 配置文件
    default_rviz = os.path.join(pkg_dir, "rviz", "ring_road.rviz")

    return LaunchDescription(
        [
            # ---------- 可选参数 ----------
            DeclareLaunchArgument("use_rviz", default_value="true", description="是否启动 RViz2"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),

            # ---------- 环形道路仿真节点 ----------
            Node(
                package="self_driving_car_demo",
                executable="ring_road_sim",
                name="ring_road_sim",
                output="screen",
                parameters=[],
            ),

            # ---------- RViz2 ----------
            Node(
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                output="screen",
            ),
        ]
    )
