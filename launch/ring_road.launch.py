"""
启动自动驾驶仿真（RViz2 可视化，环形道路 + 自动/手动驾驶）

用法：
  ros2 launch self_driving_car_demo ring_road.launch.py
  ros2 launch self_driving_car_demo ring_road.launch.py mode:=1   # 以手动驾驶模式启动

mode 取值：
  0 = 自动驾驶（默认，VehicleRuntime 统一轨迹闭环）
  1 = 手动驾驶（WASD 键盘控制：W=前进 S=倒车 A=左转 D=右转）
也可以在 RViz 右侧 HUD 面板中随时切换。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("self_driving_car_demo")

    # 默认 RViz 配置文件
    default_rviz = os.path.join(pkg_dir, "rviz", "ring_road.rviz")
    default_params = os.path.join(pkg_dir, "config", "runtime.yaml")
    default_bt = os.path.join(pkg_dir, "config", "behavior_trees", "scene_driving.xml")

    required_resources = {
        "RViz config": default_rviz,
        "Runtime parameters": default_params,
        "BehaviorTree XML": default_bt,
    }
    missing = [f"{label}: {path}" for label, path in required_resources.items()
               if not os.path.isfile(path)]
    if missing:
        raise RuntimeError(
            "self_driving_car_demo package resources are not installed correctly; "
            "clean and rebuild the package. Missing: " + "; ".join(missing)
        )

    return LaunchDescription(
        [
            # ---------- 可选参数 ----------
            DeclareLaunchArgument("use_rviz", default_value="true", description="是否启动 RViz2"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("mode", default_value="0",
                                  description="初始驾驶模式: 0=自动 1=手动(WASD)"),
            DeclareLaunchArgument("robot_namespace", default_value="",
                                  description="机器人 ROS namespace，例如 car01"),
            DeclareLaunchArgument("params_file", default_value=default_params,
                                  description="Runtime 参数 YAML"),
            DeclareLaunchArgument("robot_id", default_value="car01"),
            DeclareLaunchArgument("bt_xml", default_value=default_bt,
                                  description="场景 BehaviorTree XML"),
            DeclareLaunchArgument("bt_tree_id", default_value="RingDemo",
                                  description="BehaviorTree ID: RingDemo/MiningHaul/PortTransport/AgricultureRoute"),

            # ---------- 自动驾驶仿真节点 ----------
            Node(
                package="self_driving_car_demo",
                executable="ring_road_sim",
                name="ring_road_sim",
                namespace=LaunchConfiguration("robot_namespace"),
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "initial_mode": LaunchConfiguration("mode"),
                        "robot_id": LaunchConfiguration("robot_id"),
                        "bt_xml": LaunchConfiguration("bt_xml"),
                        "bt_tree_id": LaunchConfiguration("bt_tree_id"),
                    },
                ],
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
