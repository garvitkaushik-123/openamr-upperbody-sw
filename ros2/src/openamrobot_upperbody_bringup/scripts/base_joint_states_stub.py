#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Publish the joints no ros2_control owns on /joint_states, for the MOCK stack only: the base's wheels at rest.

The base is not under ros2_control (ADR-003), so nothing in the mock bringup reports `left_wheel_joint` or `right_wheel_joint`, and
MoveIt, whose robot model includes them, warns every second that "the complete state of the robot is not yet known". The base does not
move during manipulation (AD-30), so reporting the wheels at zero is the true state of the thing being modelled.
`lift_joint` is no longer published here (issue #17): in the mock and gazebo profiles it has its own ros2_control interface and
joint_state_broadcaster reports it (its starting position is mock.launch.py's `lift_position`). In Gazebo the wheel states come from the
simulator and this node is not started.

    ros2 run openamrobot_upperbody_bringup base_joint_states_stub.py [--ros-args -p rate:=10.0]

How it ends: a SIGINT or a SIGTERM ends the spin, and the process leaves with os._exit(0) WITHOUT destroying the node. `destroy_node()` can
hang for good under rmw_fastrtps: rmw_destroy_node -> destroy_participant -> DomainParticipant::set_listener loops with the GIL held while a
Fast DDS discovery thread (PDP::remove_remote_participant) waits on a lock the main thread holds. SIGTERM cannot interrupt it, launch escalates
to SIGKILL, the process exits -9 and M01's exit-code check fails (1 in 6 mock launches; about 1 in 160 to 400 starts of 16 stubs at a time in
one domain under CPU load, teardown_stress.py). The same pattern as the lift guard's exit (issue #44): the OS reclaims the participant.
"""
import os
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

WHEELS = ("left_wheel_joint", "right_wheel_joint")


class BaseJointStatesStub(Node):
    def __init__(self):
        super().__init__("base_joint_states_stub")
        self.declare_parameter("rate", 10.0)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(1.0 / float(self.get_parameter("rate").value), self.publish)

    def publish(self):
        names = list(WHEELS)
        msg = JointState(name=names, position=[0.0] * len(names), velocity=[0.0] * len(names))
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def exit_process(code):
    """Leave now with `code`: flush what was printed, then os._exit, which runs no destructor, so no rmw teardown that can hang."""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.flush()
        except (OSError, ValueError):
            pass
    os._exit(code)


def main():
    rclpy.init()
    node = BaseJointStatesStub()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):     # SIGINT and SIGTERM both end the spin here: a clean stop
        pass
    exit_process(0)              # not node.destroy_node() + rclpy.try_shutdown(): see the module docstring


if __name__ == "__main__":
    main()
