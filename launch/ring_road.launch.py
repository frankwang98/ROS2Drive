"""
启动自动驾驶仿真（RViz2 可视化，双地图切换 + 科目二考试）

用法：
  ros2 launch self_driving_car_demo ring_road.launch.py
  ros2 launch self_driving_car_demo ring_road.launch.py map:=1   # 科目二综合赛道

map 取值：
  0 = 环形道路（Map 1，HUD 提供「开始 / 暂停」，保留随机障碍避障）
  1 = 科目二综合赛道（Map 2，倒车入库 / 侧方停车 / 直角转弯 在同一条道路上，
      HUD 提供「开始考试」，无障碍物）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
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
            DeclareLaunchArgument("map", default_value="0",
                                  description="初始地图: 0=环形道路 1=科目二综合赛道"),

            # ---------- 自动驾驶仿真节点 ----------
            Node(
                package="self_driving_car_demo",
                executable="ring_road_sim",
                name="ring_road_sim",
                output="screen",
                parameters=[{"initial_map": LaunchConfiguration("map")}],
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
