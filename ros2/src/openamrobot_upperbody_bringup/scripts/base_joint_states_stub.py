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

Exit (issue #63): 0 on SIGINT and SIGTERM, as every process of a launch must at teardown (P24). It used to exit 1 on every SIGTERM
(ExternalShutdownException was not caught), and it could hang in Fast DDS while destroying its node and be SIGKILLed (M01, exit -9).
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


def main(argv=None):
    """0 when a shutdown (SIGINT or SIGTERM) ends the spin; anything else raises. The RuntimeError is the executor's take racing the signal,
    a shutdown only when the context is closed (the same rule as joint_state_clamp.run)."""
    rclpy.init(args=argv)
    node = BaseJointStatesStub()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    return 0                                      # no destroy_node(): see exit_process


def exit_process(code):
    """End the process with `code` now, without destroying the node (issue #63). With rmw_fastrtps, destroying the last node destroys the DDS
    participant, and at a launch teardown that can deadlock: the main thread waits in DomainParticipant::set_listener for a listener
    callback that a discovery thread runs and that waits on a lock the main thread holds, with the GIL held, so SIGTERM cannot end it and
    launch SIGKILLs the process (exit -9). The process is ending: os._exit leaves the participant to the kernel, peers drop it at its lease."""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.flush()
        except (OSError, ValueError):
            pass
    os._exit(code)


if __name__ == "__main__":
    exit_process(main())
