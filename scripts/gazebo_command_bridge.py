#!/usr/bin/env python3
"""Optional ROS2Drive command to Gazebo Twist adapter (command-only)."""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from self_driving_car_demo.msg import ControlCommand

class Bridge(Node):
    def __init__(self):
        super().__init__("gazebo_command_bridge")
        self.last = 0.0
        self.pub = self.create_publisher(Twist, "/model/ackermann_car/cmd_vel", 10)
        self.create_subscription(ControlCommand, "control/command", self.on_command, 10)
        self.create_timer(0.05, self.watchdog)

    def on_command(self, msg):
        self.last = time.monotonic()
        cmd = Twist()
        if not msg.emergency_stop and msg.brake < 0.5 and all(
            math.isfinite(v) for v in (msg.target_speed, msg.steering_angle, msg.brake)
        ):
            cmd.linear.x = max(-3.0, min(3.0, msg.target_speed))
            steer = max(-0.55, min(0.55, msg.steering_angle))
            cmd.angular.z = cmd.linear.x * math.tan(steer) / 2.0
        self.pub.publish(cmd)

    def watchdog(self):
        if time.monotonic() - self.last > 0.3:
            self.pub.publish(Twist())

def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
