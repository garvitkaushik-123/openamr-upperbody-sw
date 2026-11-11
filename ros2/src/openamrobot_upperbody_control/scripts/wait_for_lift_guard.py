#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Wait until the lift guard reports OK on /lift_guard/status, then exit 0; exit 1 after timeout_s (issue #44).

    ros2 run openamrobot_upperbody_control wait_for_lift_guard.py [--ros-args -p timeout_s:=120.0]

lift_controller is spawned inactive and rejects goals until the guard activates it, so a launch that sends a lift goal at start-up
(moveit.launch.py lift_position:=X) runs this first and sends the goal only when it exits 0.
"""
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

STATUS_TOPIC = "/lift_guard/status"
# The guard publishes with this QoS (scripts/lift_guard.py): transient local delivers the latest status on subscription.
LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def main(argv=None):
    rclpy.init(args=argv)
    node = Node("wait_for_lift_guard")
    node.declare_parameter("timeout_s", 120.0)
    timeout_s = float(node.get_parameter("timeout_s").value)
    last = {"msg": None}
    node.create_subscription(DiagnosticStatus, STATUS_TOPIC, lambda msg: last.__setitem__("msg", msg), LATCHED)
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline and not (last["msg"] is not None and last["msg"].level == DiagnosticStatus.OK):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    ok = last["msg"] is not None and last["msg"].level == DiagnosticStatus.OK
    seen = "nothing" if last["msg"] is None else f"{last['msg'].message!r}"
    print(f"wait_for_lift_guard: {'OK' if ok else f'not OK after {timeout_s:g} s, last status {seen}'}", flush=True)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
