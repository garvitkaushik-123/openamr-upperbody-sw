#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Start-up move of both arms to a stowed SRDF group state (default `park`), so the lift guard can allow the lift (issue #44).

    ros2 run openamrobot_upperbody_control park_arms.py --ros-args -p robot_description_semantic:="$(cat openamrobot.srdf)"
        [-p state:=park] [-p duration_s:=3.0] [-p timeout_s:=120.0] [-p use_sim_time:=true]

Accepted on #44: upstream's ros2_control macro hardcodes every arm joint's initial position to 0 (home), and park differs (joint4 0.1 rad), so
in mock and in gazebo the launch moves both arm controllers to park once at start-up. The URDF and the configuration IDs are unchanged.

One FollowJointTrajectory goal per arm controller (left_arm_controller, right_arm_controller), joints openarm_{side}_joint1..7, a single point
at `duration_s`, positions from the SRDF and zero velocities. A goal rejected because the controller is not active yet is sent again until
`timeout_s`. Exits 0 when both goals succeed, 1 otherwise (the guard then keeps the lift blocked: the arms are not stowed).

Not failures (exit 0, logged): a goal the controller ended because a newer goal on the same controller replaced it (someone commanded the
arm during the start-up move, e.g. a test or MoveIt), or because the controller was deactivated (a teardown). JointTrajectoryController
reports both with the error strings in SUPERSEDED (a preempted goal: aborted, INVALID_GOAL) (Jazzy 4.x); the guard decides "stowed" from the joint
states either way, so exiting 1 here only failed every launch's exit-code check (P24; seen when L01's goal arrived 0.1 s before park's
ended).
"""
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from openamrobot_upperbody_control import lift_guard as lg
from openamrobot_upperbody_control.ros_exit import exit_process, is_clean_stop

ARM_CONTROLLERS = {"left": "left_arm_controller", "right": "right_arm_controller"}
RETRY_S = 0.5
# JointTrajectoryController's error_string when it ends an accepted goal for a reason other than the goal itself (joint_trajectory_controller,
# Jazzy 4.x): a newer goal preempted it, or the controller was deactivated.
SUPERSEDED = ("preempted by new incoming action", "cancelled during deactivate transition")


def result_outcome(result):
    """(ok, detail) for a FollowJointTrajectory result: success, superseded (ok: not park's failure) or failure."""
    if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
        return True, f"error_code {result.error_code}"
    if any(s in result.error_string for s in SUPERSEDED):
        return True, f"superseded, not parked by park_arms: {result.error_string!r}"
    return False, f"error_code {result.error_code} {result.error_string!r}"


def goal_for(side, pose, duration_s):
    joints = list(lg.ARM_JOINTS[side])
    seconds = int(duration_s)
    point = JointTrajectoryPoint(positions=[pose[j] for j in joints], velocities=[0.0] * len(joints),
                                 time_from_start=Duration(sec=seconds, nanosec=int(round((duration_s - seconds) * 1e9))))
    return FollowJointTrajectory.Goal(trajectory=JointTrajectory(joint_names=joints, points=[point]))


def park(node, poses, duration_s, timeout_s):
    """{side: (ok, detail)}: send each arm its goal, resending a rejected one until the deadline, then wait for the results."""
    deadline = time.monotonic() + timeout_s
    clients = {side: ActionClient(node, FollowJointTrajectory, f"/{controller}/follow_joint_trajectory")
               for side, controller in ARM_CONTROLLERS.items()}
    handles, outcome = {}, {}
    for side, client in clients.items():
        while side not in handles and time.monotonic() < deadline:
            if not client.wait_for_server(timeout_sec=min(1.0, max(0.0, deadline - time.monotonic()))):
                continue
            future = client.send_goal_async(goal_for(side, poses[side], duration_s))
            rclpy.spin_until_future_complete(node, future, timeout_sec=max(0.0, deadline - time.monotonic()))
            handle = future.result() if future.done() else None
            if handle is not None and handle.accepted:
                handles[side] = handle
            else:
                time.sleep(RETRY_S)              # most likely the controller is configured but not active yet
        if side not in handles:
            outcome[side] = (False, f"{ARM_CONTROLLERS[side]} did not accept the goal within {timeout_s:g} s")
    for side, handle in handles.items():
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, future, timeout_sec=max(0.0, deadline - time.monotonic()))
        if not future.done():
            outcome[side] = (False, f"{ARM_CONTROLLERS[side]}: no result within {timeout_s:g} s")
            continue
        ok, detail = result_outcome(future.result().result)
        outcome[side] = (ok, f"{ARM_CONTROLLERS[side]}: {detail}")
    return outcome


def main(argv=None):
    rclpy.init(args=argv)
    node = Node("park_arms")
    node.declare_parameter("robot_description_semantic", "")
    node.declare_parameter("state", "park")
    node.declare_parameter("duration_s", 3.0)
    node.declare_parameter("timeout_s", 120.0)
    try:
        state = node.get_parameter("state").value
        poses = {side: pose_list[0] for side, pose_list in
                 lg.stowed_poses_from_srdf(node.get_parameter("robot_description_semantic").value, [state]).items()}
        duration_s, timeout_s = float(node.get_parameter("duration_s").value), float(node.get_parameter("timeout_s").value)
        if not (duration_s > 0 and timeout_s > 0):
            raise lg.GuardConfigError("duration_s and timeout_s must be > 0")
        outcome = park(node, poses, duration_s, timeout_s)
    except lg.GuardConfigError as err:
        print(f"park_arms: {err}", file=sys.stderr, flush=True)
        return 2
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError) as error:
        if not is_clean_stop(error):
            raise
        # A teardown before the arms reached park: a clean stop (P24). The guard never saw them stowed, so the lift stayed blocked.
        print("park_arms: stopped by a shutdown before the arms were parked", flush=True)
        return 0
    for side in lg.SIDES:
        print(f"park_arms: {state} {side}: {'ok' if outcome[side][0] else 'FAILED'} ({outcome[side][1]})", flush=True)
    return 0 if all(ok for ok, _ in outcome.values()) else 1


if __name__ == "__main__":
    exit_process(main())                          # no destroy_node: see ros_exit.exit_process
