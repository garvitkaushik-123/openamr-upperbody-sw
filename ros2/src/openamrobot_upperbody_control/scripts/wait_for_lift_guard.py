#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Wait until the lift guard reports OK on /lift_guard/status; with lift_position, then send the lift there (issue #44).

    ros2 run openamrobot_upperbody_control wait_for_lift_guard.py [--ros-args -p timeout_s:=120.0]
        [-p lift_position:=0.175 -p duration_s:=20.0]

lift_controller is spawned inactive and rejects goals until the guard activates it, so the start-up lift goal of moveit.launch.py
(lift_position:=X) is sent by this process, after the first OK: one FollowJointTrajectory goal to lift_controller, lift_joint to
`lift_position` (m) at `duration_s`, then it waits for the result.

Exit: 0 when the guard reported OK (and, with lift_position, the goal succeeded); 1 when it did not within timeout_s, or the goal was
rejected or failed; 0 when a shutdown stops it first (every process of a launch must exit 0 at teardown, P24). This replaces
`ros2 action send_goal`, which a teardown during the goal made exit 1 ("Executor is already spinning"; #60's L05 in the mock stack).
"""
import math
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from openamrobot_upperbody_control.ros_exit import exit_process, is_clean_stop

STATUS_TOPIC = "/lift_guard/status"
LIFT_ACTION = "/lift_controller/follow_joint_trajectory"
LIFT_JOINT = "lift_joint"
# The guard publishes with this QoS (scripts/lift_guard.py): transient local delivers the latest status on subscription.
LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def spin_until(node, done, deadline):
    while time.monotonic() < deadline and not done():
        rclpy.spin_once(node, timeout_sec=0.1)
    return done()


def lift_goal(position, duration_s):
    seconds = int(duration_s)
    point = JointTrajectoryPoint(positions=[position], velocities=[0.0],
                                 time_from_start=Duration(sec=seconds, nanosec=int(round((duration_s - seconds) * 1e9))))
    return FollowJointTrajectory.Goal(trajectory=JointTrajectory(joint_names=[LIFT_JOINT], points=[point]))


def run(node, timeout_s, position, duration_s):
    """0 or 1, as the module says. Raises on a shutdown; main turns that into 0."""
    last = {"msg": None}
    node.create_subscription(DiagnosticStatus, STATUS_TOPIC, lambda msg: last.__setitem__("msg", msg), LATCHED)
    deadline = time.monotonic() + timeout_s
    allowed = spin_until(node, lambda: last["msg"] is not None and last["msg"].level == DiagnosticStatus.OK, deadline)
    seen = "nothing" if last["msg"] is None else repr(last["msg"].message)
    if not allowed:
        print(f"wait_for_lift_guard: not OK after {timeout_s:g} s, last status {seen}", flush=True)
        return 1
    print("wait_for_lift_guard: OK", flush=True)
    if math.isnan(position):
        return 0
    client = ActionClient(node, FollowJointTrajectory, LIFT_ACTION)
    if not client.wait_for_server(timeout_sec=max(0.0, deadline - time.monotonic())):
        print(f"wait_for_lift_guard: no {LIFT_ACTION} action server", flush=True)
        return 1
    sent = client.send_goal_async(lift_goal(position, duration_s))
    if not spin_until(node, sent.done, deadline) or not sent.result().accepted:
        print(f"wait_for_lift_guard: the lift goal to {position:g} m was not accepted", flush=True)
        return 1
    result = sent.result().get_result_async()
    if not spin_until(node, result.done, time.monotonic() + duration_s + timeout_s):
        print(f"wait_for_lift_guard: the lift goal to {position:g} m did not finish", flush=True)
        return 1
    code = result.result().result.error_code
    print(f"wait_for_lift_guard: lift goal to {position:g} m: error_code {code}", flush=True)
    return 0 if code == FollowJointTrajectory.Result.SUCCESSFUL else 1


def main(argv=None):
    rclpy.init(args=argv)
    node = Node("wait_for_lift_guard")
    node.declare_parameter("timeout_s", 120.0)
    node.declare_parameter("lift_position", math.nan)     # NaN: only wait for the guard's OK
    node.declare_parameter("duration_s", 10.0)
    try:
        return run(node, float(node.get_parameter("timeout_s").value), float(node.get_parameter("lift_position").value),
                   float(node.get_parameter("duration_s").value))
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError) as error:
        if not is_clean_stop(error):
            raise
        print("wait_for_lift_guard: stopped by a shutdown", flush=True)      # a clean stop (P24)
        return 0


if __name__ == "__main__":
    exit_process(main())                          # no destroy_node: see ros_exit.exit_process
