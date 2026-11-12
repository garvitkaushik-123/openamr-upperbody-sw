#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Mock profile only (issue #44): publish the base's odometry on /odom, so the lift guard's "base stopped" condition has an input.

    ros2 run openamrobot_upperbody_control lift_guard_odom_stub.py [--ros-args -p linear_x:=0.0 -p angular_z:=0.0 -p rate_hz:=20.0]

The mock profile has no base (the base is not under ros2_control, ADR-003), so nothing publishes /odom there, and the guard treats missing
odometry as unsafe. This node stands in for it. Accepted on #44:
  - node /lift_guard_odom_stub, publishing nav_msgs/Odometry on /odom at rate_hz;
  - its twist is linear.x = linear_x (m/s) and angular.z = angular_z (rad/s), parameters read on every message, so a test can make the
    base "move" by setting them (the pose is not integrated: the guard reads the twist only);
  - std_srvs/SetBool on /lift_guard_odom_stub/enable: false stops publishing (odometry loss), true resumes it.
Not for gazebo, where the DiffDrive plugin publishes /odom, and never for the real profile.
"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import SetBool

from openamrobot_upperbody_control.ros_exit import exit_process, spin_until_shutdown

NODE_NAME = "lift_guard_odom_stub"
TOPIC = "/odom"


class OdomStub(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.declare_parameter("linear_x", 0.0)
        self.declare_parameter("angular_z", 0.0)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_footprint")
        rate_hz = float(self.get_parameter("rate_hz").value)
        if not rate_hz > 0:
            raise SystemExit(f"{NODE_NAME}: rate_hz must be > 0, got {rate_hz!r}")
        self.enabled = True
        self.publisher = self.create_publisher(Odometry, TOPIC, 10)
        self.create_service(SetBool, "~/enable", self.on_enable)
        self.create_timer(1.0 / rate_hz, self.publish)
        self.get_logger().info(f"publishing {TOPIC} at {rate_hz:g} Hz (mock profile: there is no base); ~/enable switches it")

    def on_enable(self, request, response):
        self.enabled = bool(request.data)
        response.success = True
        response.message = f"{TOPIC} {'enabled' if self.enabled else 'disabled'}"
        self.get_logger().info(response.message)
        return response

    def publish(self):
        if not self.enabled:
            return
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("frame_id").value
        msg.child_frame_id = self.get_parameter("child_frame_id").value
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = float(self.get_parameter("linear_x").value)
        msg.twist.twist.angular.z = float(self.get_parameter("angular_z").value)
        self.publisher.publish(msg)


def main(argv=None):
    rclpy.init(args=argv)
    node = OdomStub()
    return spin_until_shutdown(node)              # 0 on SIGINT/SIGTERM (P24); no destroy_node: see ros_exit.exit_process


if __name__ == "__main__":
    exit_process(main())
