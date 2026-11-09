#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Keep lift_controller inactive unless the arms are stowed and the base is stopped: the simulated mirror of the robot's lift interlock.

    ros2 run openamrobot_upperbody_control lift_guard.py --ros-args -p robot_description_semantic:="$(cat openamrobot.srdf)"
        [-p arm_state_topic:=/arm/joint_states] [-p odom_topic:=/odom] [-p use_sim_time:=true] [-p <setting>:=<value> ...]

Issue #44. Simulated profiles only (mock, gazebo). lift_controller is spawned inactive; this node activates it through the controller manager's
switch_controller service only while openamrobot_upperbody_control/lift_guard.py finds nothing blocking, and deactivates it as soon as
something does. An inactive JointTrajectoryController rejects action goals and ignores its joint_trajectory topic, so every path to the lift
is covered. Deactivation aborts a running goal; re-activation does not replay it.

Inputs (the guard's clock at receipt, never header stamps):
  <arm_state_topic>                                     sensor_msgs/JointState          arm joint positions (/arm/joint_states in gazebo)
  /{left,right}_arm_controller/controller_state         control_msgs/JointTrajectoryControllerState   the controllers' reference
  /{left,right}_arm_controller/follow_joint_trajectory/_action/status   action_msgs/GoalStatusArray   goals in progress
  <odom_topic>                                          nav_msgs/Odometry               the base's twist
  /controller_manager/list_controllers                  polled at status_rate_hz         lift_controller's actual state
Output:
  /lift_guard/status   diagnostic_msgs/DiagnosticStatus, transient local: OK only when nothing blocks and lift_controller is listed active.

Not here: the deactivation when THIS process exits. The launch that starts the guard does that (OnProcessExit), because a process that has
died cannot. Nothing covers a guard that hangs, and this node claims nothing about it.
"""
import sys

import rclpy
from action_msgs.msg import GoalStatusArray
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState

from openamrobot_upperbody_control import lift_guard as lg

STATUS_TOPIC = "/lift_guard/status"
# The arm controllers of arm_controllers.yaml (and moveit_controllers.yaml, action_ns follow_joint_trajectory).
ARM_CONTROLLERS = {"left": "left_arm_controller", "right": "right_arm_controller"}
LEVELS = {lg.LEVEL_OK: DiagnosticStatus.OK, lg.LEVEL_ERROR: DiagnosticStatus.ERROR}
# The action status topic's own QoS (rcl_action_qos_profile_status_default): keep last 1, reliable, transient local. Matching it delivers the
# last status on subscription, so a goal that started before the guard did is seen.
LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class LiftGuard(Node):
    def __init__(self):
        super().__init__("lift_guard")
        self.declare_parameter("robot_description_semantic", "")
        self.declare_parameter("stowed_states", list(lg.DEFAULTS["stowed_states"]))
        for key, value in lg.DEFAULTS.items():
            if key != "stowed_states":
                self.declare_parameter(key, float(value))
        self.declare_parameter("arm_state_topic", "/joint_states")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("status_rate_hz", 10.0)
        self.declare_parameter("controller_name", "lift_controller")
        self.declare_parameter("controller_manager", "/controller_manager")
        self.declare_parameter("switch_timeout_s", 2.0)

        config = {key: self.get_parameter(key).value for key in lg.DEFAULTS}
        srdf = self.get_parameter("robot_description_semantic").value
        if not srdf:
            raise lg.GuardConfigError("the robot_description_semantic parameter is empty: the stowed poses come from the SRDF and from "
                                      "nowhere else")
        self.logic = lg.LiftGuardLogic(lg.stowed_poses_from_srdf(srdf, config["stowed_states"]), config)
        self.controller_name = self.get_parameter("controller_name").value
        self.switch_timeout_s = float(self.get_parameter("switch_timeout_s").value)
        status_rate_hz = float(self.get_parameter("status_rate_hz").value)
        if not (self.switch_timeout_s > 0 and status_rate_hz > 0):
            raise lg.GuardConfigError("switch_timeout_s and status_rate_hz must be > 0")

        self.controller_state = None           # lift_controller's state as last listed (or as a successful switch set it)
        self.list_future = None
        self.switch_future = None              # (future, target state, sent at)
        self.last_status = None
        self.last_codes = None

        manager = self.get_parameter("controller_manager").value.rstrip("/")
        self.list_client = self.create_client(ListControllers, manager + "/list_controllers")
        self.switch_client = self.create_client(SwitchController, manager + "/switch_controller")
        self.status_publisher = self.create_publisher(DiagnosticStatus, STATUS_TOPIC, LATCHED)

        # Best effort subscriptions match reliable and best-effort publishers alike.
        self.create_subscription(JointState, self.get_parameter("arm_state_topic").value, self.on_joint_state, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.on_odometry, qos_profile_sensor_data)
        for side, controller in ARM_CONTROLLERS.items():
            self.create_subscription(JointTrajectoryControllerState, f"/{controller}/controller_state",
                                     lambda msg, side=side: self.on_controller_state(side, msg), qos_profile_sensor_data)
            self.create_subscription(GoalStatusArray, f"/{controller}/follow_joint_trajectory/_action/status",
                                     lambda msg, side=side: self.on_goal_status(side, msg), LATCHED)
        self.create_timer(1.0 / status_rate_hz, self.on_timer)
        self.get_logger().info(f"guarding {self.controller_name}: stowed states {list(config['stowed_states'])}, "
                               f"arm state on {self.get_parameter('arm_state_topic').value}, odometry on "
                               f"{self.get_parameter('odom_topic').value}; it stays inactive until nothing blocks")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # --- inputs ---------------------------------------------------------------------------------------------------------------------------
    def on_joint_state(self, msg):
        self.logic.on_joint_state(msg.name, msg.position, self.now())
        self.step()

    def on_controller_state(self, side, msg):
        self.logic.on_controller_state(side, msg.joint_names, msg.reference.positions, msg.reference.velocities, self.now())
        self.step()

    def on_goal_status(self, side, msg):
        self.logic.on_goal_statuses(side, [s.status for s in msg.status_list])
        self.step()

    def on_odometry(self, msg):
        twist = msg.twist.twist
        self.logic.on_odometry((twist.linear.x, twist.linear.y, twist.linear.z),
                               (twist.angular.x, twist.angular.y, twist.angular.z), self.now())
        self.step()

    def on_timer(self):
        self.poll_controllers()
        self.step(periodic=True)

    # --- the controller manager -----------------------------------------------------------------------------------------------------------
    def poll_controllers(self):
        if self.list_future is not None or not self.list_client.service_is_ready():
            return
        self.list_future = self.list_client.call_async(ListControllers.Request())
        self.list_future.add_done_callback(self.on_list)

    def on_list(self, future):
        self.list_future = None
        try:
            controllers = future.result().controller
        except Exception as err:             # noqa: BLE001 - any failure leaves the state unknown, which never activates anything
            self.get_logger().error(f"list_controllers failed: {err}")
            self.controller_state = None
            return
        states = [c.state for c in controllers if c.name == self.controller_name]
        self.controller_state = states[0] if states else None

    def switch(self, target):
        request = SwitchController.Request()
        if target == "active":
            request.activate_controllers = [self.controller_name]
        else:
            request.deactivate_controllers = [self.controller_name]
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = False
        seconds = int(self.switch_timeout_s)
        request.timeout = Duration(sec=seconds, nanosec=int(round((self.switch_timeout_s - seconds) * 1e9)))
        future = self.switch_client.call_async(request)
        self.switch_future = (future, target, self.now())
        future.add_done_callback(lambda f, target=target: self.on_switch(f, target))
        self.get_logger().info(f"{'activating' if target == 'active' else 'deactivating'} {self.controller_name}")

    def on_switch(self, future, target):
        if self.switch_future is None or self.switch_future[0] is not future:
            return                           # superseded by a later request
        self.switch_future = None
        try:
            response = future.result()
            ok, message = response.ok, response.message
        except Exception as err:             # noqa: BLE001
            ok, message = False, str(err)
        if ok:
            self.logic.switch_failed = False
            self.controller_state = target
        else:
            self.logic.switch_failed = True
            self.get_logger().error(f"switching {self.controller_name} to {target} failed{': ' + message if message else ''}")
        self.step()

    def drive(self, codes):
        """Ask for the switch the decision wants (lift_guard.switch_to_request). A request with no answer after switch_timeout_s + 1 s is
        dropped and counts as failed, so it can be asked again."""
        if self.switch_future is not None and self.now() - self.switch_future[2] > self.switch_timeout_s + 1.0:
            self.get_logger().error(f"switching {self.controller_name} to {self.switch_future[1]} got no answer")
            self.switch_future = None
            self.logic.switch_failed = True
        if not self.switch_client.service_is_ready():
            return
        pending = self.switch_future[1] if self.switch_future is not None else None
        target = lg.switch_to_request(codes, self.controller_state, pending)
        if target is not None:
            self.switch(target)

    # --- the decision and the status ------------------------------------------------------------------------------------------------------
    def step(self, periodic=False):
        codes = self.logic.evaluate(self.now())
        self.drive(codes)
        status = lg.build_status(codes, self.controller_state)
        if codes != self.last_codes:
            self.get_logger().info(f"blocking: {', '.join(codes) or 'nothing'}")
            self.last_codes = codes
        if periodic or status != self.last_status:
            self.last_status = status
            level, name, message, values = status
            self.status_publisher.publish(DiagnosticStatus(level=LEVELS[level], name=name, message=message, hardware_id="",
                                                           values=[KeyValue(key=k, value=v) for k, v in values]))


def run(node, spin=rclpy.spin, context_ok=rclpy.ok):
    """Spin until the context is shut down; 0 when a shutdown ended the spin (the same contract as joint_state_clamp.py's run)."""
    try:
        spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if context_ok():
            raise
    return 0


def main(argv=None):
    rclpy.init(args=argv)
    try:
        node = LiftGuard()
    except lg.GuardConfigError as err:
        # Exits non-zero: the launch deactivates lift_controller on this exit, and it was spawned inactive, so the lift stays blocked.
        print(f"lift_guard: refusing to start: {err}", file=sys.stderr, flush=True)
        rclpy.try_shutdown()
        return 2
    try:
        return run(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
