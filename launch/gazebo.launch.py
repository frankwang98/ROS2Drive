"""Optional Gazebo Ackermann command-forwarding experiment; NOT closed loop."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg = Path(get_package_share_directory("self_driving_car_demo"))
    gz = Path(get_package_share_directory("ros_gz_sim"))
    world = pkg / "gazebo" / "worlds" / "ackermann.sdf"
    return LaunchDescription([
        DeclareLaunchArgument("run_native", default_value="false"),
        DeclareLaunchArgument("bridge_commands", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gz / "launch" / "gz_sim.launch.py")),
            launch_arguments={"gz_args": "-r " + str(world)}.items(),
        ),
        Node(
            package="self_driving_car_demo", executable="gazebo_command_bridge.py",
            condition=IfCondition(LaunchConfiguration("bridge_commands")),
            output="screen",
        ),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            arguments=["/model/ackermann_car/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"],
            condition=IfCondition(LaunchConfiguration("bridge_commands")),
            output="screen",
        ),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            arguments=["/model/ackermann_car/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"],
            remappings=[("/model/ackermann_car/odometry", "/gazebo/odometry")],
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pkg / "launch" / "ring_road.launch.py")),
            condition=IfCondition(LaunchConfiguration("run_native")),
            launch_arguments={"use_rviz": LaunchConfiguration("use_rviz")}.items(),
        ),
    ])
