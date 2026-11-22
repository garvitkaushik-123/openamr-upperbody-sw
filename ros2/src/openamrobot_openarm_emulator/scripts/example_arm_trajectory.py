#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Example arm trajectory for the software-in-the-loop launch.

Sends one FollowJointTrajectory goal to each arm controller of the real driver's controller manager
(left_arm_controller, right_arm_controller): out to a bent pose and back to zero. The path is
JointTrajectoryController -> OpenArmHW -> MIT frames on vcan -> emulated motors -> joint effort in Gazebo.

    ros2 run openamrobot_openarm_emulator example_arm_trajectory.py [--seconds 4.0] [--once]

The pose is small and symmetric. It is an example for the simulation, not a motion that was checked for the robot.
"""
import argparse
import sys

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

# rad, joint1..joint7. joint4 bends the elbow; the right arm mirrors joints 1 to 3.
LEFT_POSE = [0.3, 0.25, 0.0, 0.6, 0.0, 0.3, 0.0]
RIGHT_POSE = [-0.3, -0.25, 0.0, 0.6, 0.0, 0.3, 0.0]


def duration(seconds):
    return Duration(sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9))


def goal_for(side, pose, seconds, back):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = [f"openarm_{side}_joint{i}" for i in range(1, 8)]
    goal.trajectory.points.append(JointTrajectoryPoint(positions=pose, velocities=[0.0] * 7, time_from_start=duration(seconds)))
    if back:
        goal.trajectory.points.append(
            JointTrajectoryPoint(positions=[0.0] * 7, velocities=[0.0] * 7, time_from_start=duration(2 * seconds)))
    return goal


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seconds", type=float, default=4.0, help="time to reach the pose, and again to return")
    parser.add_argument("--once", action="store_true", help="go to the pose and stay there")
    parser.add_argument("--wait", type=float, default=120.0, help="seconds to wait for the action servers")
    args, _ = parser.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Node("example_arm_trajectory")
    clients = {side: ActionClient(node, FollowJointTrajectory, f"/{side}_arm_controller/follow_joint_trajectory")
               for side in ("left", "right")}
    for side, client in clients.items():
        if not client.wait_for_server(timeout_sec=args.wait):
            node.get_logger().error(f"{side}_arm_controller has no action server after {args.wait} s")
            return 1
    sends = {side: clients[side].send_goal_async(goal_for(side, pose, args.seconds, not args.once))
             for side, pose in (("left", LEFT_POSE), ("right", RIGHT_POSE))}
    results = {}
    for side, future in sends.items():
        rclpy.spin_until_future_complete(node, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            node.get_logger().error(f"{side}_arm_controller rejected the goal")
            return 1
        results[side] = handle.get_result_async()
    status = 0
    for side, future in results.items():
        rclpy.spin_until_future_complete(node, future)
        code = future.result().result.error_code
        node.get_logger().info(f"{side} arm trajectory finished, error_code {code}")
        status = status or (0 if code == FollowJointTrajectory.Result.SUCCESSFUL else 1)
    node.destroy_node()
    rclpy.shutdown()
    return status


if __name__ == "__main__":
    sys.exit(main())
