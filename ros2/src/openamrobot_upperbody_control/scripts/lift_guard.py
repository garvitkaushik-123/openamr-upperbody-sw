#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Keep lift_controller inactive unless the arms are stowed and the base is stopped: the simulated mirror of the robot's lift interlock.

    ros2 run openamrobot_upperbody_control lift_guard.py --ros-args -p robot_description_semantic:="$(cat openamrobot.srdf)"
        [-p arm_state_topic:=/arm/joint_states] [-p odom_topic:=/odom] [-p use_sim_time:=true] [-p <setting>:=<value> ...]

Issue #44. Simulated profiles only (mock, gazebo). lift_controller is spawned inactive; this node activates it through the controller manager's
switch_controller service only while openamrobot_upperbody_control/lift_guard.py finds nothing blocking, and deactivates it as soon as
something does, including when something else activated it (list_controllers is polled at status_rate_hz). An inactive
JointTrajectoryController rejects action goals and ignores its joint_trajectory topic, so every path to the lift is covered. Deactivation
aborts a running goal; re-activation does not replay it.

Inputs (the guard's clock at receipt, never header stamps):
  <arm_state_topic>                                     sensor_msgs/JointState          arm joint positions (/arm/joint_states in gazebo)
  /{left,right}_arm_controller/controller_state         control_msgs/JointTrajectoryControllerState   the controllers' reference
  /{left,right}_arm_controller/follow_joint_trajectory/_action/status   action_msgs/GoalStatusArray   goals in progress
  <odom_topic>                                          nav_msgs/Odometry               the base's twist
  /controller_manager/list_controllers                  polled at status_rate_hz         lift_controller's actual state
Output:
  /lift_guard/status   diagnostic_msgs/DiagnosticStatus, transient local. OK: the lift may move. WARN (blocking empty, lift_controller
                       "activating"): switching on, for at most switch_timeout_s, then SWITCH_FAILED. ERROR: blocked, with the codes.

Exit. On SIGINT or SIGTERM the node stops deciding (no activation can be asked from then on), deactivates lift_controller itself (best
effort, so an already inactive controller is not an error) and confirms with list_controllers that it is not active, before it exits 0:
the controller is inactive no later than the guard's exit (accepted on #44). If the controller manager has already left the ROS graph (a
normal teardown stops both, often the manager first), lift_controller stopped with it: exit 0, logged as UNVERIFIED by the guard. If the
manager is still there but never confirms: exit 1.
A death without a signal it can handle (SIGKILL, a crash) is covered by the launch's OnProcessExit handler. Nothing covers a guard that
hangs, and this node claims nothing about it.
"""
import signal
import sys
import time

import rclpy
from action_msgs.msg import GoalStatusArray
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

from openamrobot_upperbody_control import lift_guard as lg
from openamrobot_upperbody_control.ros_exit import exit_process

STATUS_TOPIC = "/lift_guard/status"
# The arm controllers of arm_controllers.yaml (and moveit_controllers.yaml, action_ns follow_joint_trajectory).
ARM_CONTROLLERS = {"left": "left_arm_controller", "right": "right_arm_controller"}
LEVELS = {lg.LEVEL_OK: DiagnosticStatus.OK, lg.LEVEL_WARN: DiagnosticStatus.WARN, lg.LEVEL_ERROR: DiagnosticStatus.ERROR}
# The action status topic's own QoS (rcl_action_qos_profile_status_default): keep last 1, reliable, transient local. Matching it delivers the
# last status on subscription, so a goal that started before the guard did is seen.
LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
SPIN_PERIOD_S = 0.02        # how often the main loop looks at the stop flag between callbacks
EXIT_ATTEMPTS = 3           # deactivate-and-confirm rounds at exit before giving up (the launch handler is the fallback)
EXIT_SERVICE_WAIT_S = 0.5   # how long one exit round waits for switch_controller before it looks at the graph again
EXIT_POLL_S = 0.1           # while an exit request is unanswered, how often the graph is checked for the manager
# The whole exit, every round included: below the launch's 5 s before it escalates SIGINT to SIGTERM. A manager that crashed can stay in
# the graph (its participant never said goodbye), so "gone" is not seen and the rounds would otherwise run on (seen with a crashed gz server).
EXIT_BUDGET_S = 4.0
EXIT_OUTCOMES = ("confirmed", "manager_gone", "unanswered", "failed")
EXIT_CODES = {"confirmed": 0, "manager_gone": 0, "unanswered": 0, "failed": 1}


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
        self.switches_sent = 0                 # list answers requested before the latest switch are dropped (lift_guard.list_answer_is_current)
        self.activating_since = None           # start of the WARN "activating" state (lift_guard.activation_overdue)
        self.exiting = False                   # set by deactivate_for_exit: from then on nothing polls, decides or switches
        self.last_status = None
        self.last_codes = None

        manager = self.get_parameter("controller_manager").value.rstrip("/")
        self.manager = manager if manager.startswith("/") else "/" + manager
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
        self.status_timer = self.create_timer(1.0 / status_rate_hz, self.on_timer)
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
        if self.exiting or self.list_future is not None or not self.list_client.service_is_ready():
            return
        requested_after, requested_while = self.switches_sent, self.switch_future is not None
        self.list_future = self.list_client.call_async(ListControllers.Request())
        self.list_future.add_done_callback(lambda f: self.on_list(f, requested_after, requested_while))

    def on_list(self, future, requested_after, requested_while):
        self.list_future = None
        if self.exiting:
            return                           # deactivate_for_exit confirms the state itself
        if not lg.list_answer_is_current(requested_after, requested_while, self.switches_sent, self.switch_future is not None):
            return                           # may predate a switch: the next poll asks again
        try:
            controllers = future.result().controller
        except Exception as err:             # noqa: BLE001 - any failure leaves the state unknown, which never activates anything
            self.get_logger().error(f"list_controllers failed: {err}")
            self.controller_state = None
            return
        states = [c.state for c in controllers if c.name == self.controller_name]
        self.controller_state = states[0] if states else None
        self.step()

    def switch_request(self, target, strictness):
        request = SwitchController.Request()
        if target == "active":
            request.activate_controllers = [self.controller_name]
        else:
            request.deactivate_controllers = [self.controller_name]
        request.strictness = strictness
        request.activate_asap = False
        seconds = int(self.switch_timeout_s)
        request.timeout = Duration(sec=seconds, nanosec=int(round((self.switch_timeout_s - seconds) * 1e9)))
        return request

    def switch(self, target):
        future = self.switch_client.call_async(self.switch_request(target, SwitchController.Request.STRICT))
        self.switches_sent += 1
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
        dropped and counts as failed, so it can be asked again. Never while exiting."""
        if self.exiting:
            return
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

    def call_and_wait(self, executor, client, request, deadline=None):
        """The answer to `request`, or None after switch_timeout_s + 1 s, or as soon as the controller manager has left the graph (a
        teardown that stopped it first leaves nothing to answer, and waiting out the timeout would push the guard toward the launch's
        SIGTERM escalation)."""
        future = client.call_async(request)
        end = time.monotonic() + self.switch_timeout_s + 1.0
        if deadline is not None:
            end = min(end, deadline)
        while not future.done() and time.monotonic() < end:
            executor.spin_until_future_complete(future, timeout_sec=EXIT_POLL_S)
            if not future.done() and self.manager_gone():
                break
        return future.result() if future.done() else None

    def manager_gone(self):
        """True when no node of the controller manager's name is in the ROS graph any more (it has exited)."""
        namespace, _, name = self.manager.rpartition("/")
        return (name, namespace or "/") not in set(self.get_node_names_and_namespaces())

    def deactivate_for_exit(self, executor):
        """Leave lift_controller inactive before the process exits (accepted on #44; review of #53). Returns one of EXIT_OUTCOMES:

          "confirmed"     the controller manager answered the deactivation ok and lists lift_controller as not active;
          "manager_gone"  the controller manager has left the ROS graph (a normal teardown stops it with the guard, and often first). A
                          controller cannot run without its manager, and the manager deactivates every controller when it shuts down
                          (controller_manager shutdown_controllers, seen in the mock stack's log). The guard did not see it: it says so,
                          as UNVERIFIED by the guard;
          "unanswered"    the manager is still in the graph but answered none of the exit's requests. At a launch teardown it gets the
                          same signal: in gazebo the manager runs inside gz sim, its ROS interface stops at the signal while the process
                          lives on for seconds (M07 on ROS 2 Jazzy: "Deactivated controllers: [ lift_controller ]" in gz's log, the reply
                          never arrived, gz was SIGTERMed 5 s later). A manager that answers nobody cannot move the lift for anyone
                          either. UNVERIFIED by the guard, exit 0;
          "failed"        the manager answered, but never confirmed lift_controller inactive (a refused deactivation, or still listed
                          active) within EXIT_ATTEMPTS rounds and EXIT_BUDGET_S. Exit 1.

        First the shutdown state: from here on nothing polls, decides or switches (the timer is cancelled and step, drive, poll and on_list
        return at once), so no callback that runs while this waits can ask for an activation; a switch already sent is no longer tracked and
        list answers requested before now are dropped. Then, each round: stop if the manager has gone; ask to deactivate (best effort, so an
        already inactive controller is not an error); confirm with list_controllers that lift_controller is not active. An activation sent
        before the shutdown is answered by the manager before this deactivation (same client, in order); if one still shows active, the
        round repeats."""
        self.exiting = True
        self.status_timer.cancel()
        self.switches_sent += 1
        self.switch_future = None
        deadline = time.monotonic() + EXIT_BUDGET_S
        answered = False
        for attempt in range(1, EXIT_ATTEMPTS + 1):
            if self.manager_gone():
                return self.exit_report("manager_gone")
            if time.monotonic() >= deadline:
                self.exit_note(f"the {EXIT_BUDGET_S:g} s exit budget is spent")
                break
            if not self.switch_client.wait_for_service(timeout_sec=min(EXIT_SERVICE_WAIT_S, max(0.0, deadline - time.monotonic()))):
                self.exit_note(f"round {attempt}: switch_controller not available")
                continue
            response = self.call_and_wait(executor, self.switch_client,
                                          self.switch_request("inactive", SwitchController.Request.BEST_EFFORT), deadline)
            answered = answered or response is not None
            if response is None or not response.ok:
                self.exit_note(f"round {attempt}: no ok answer to the deactivation")
                continue
            listed = self.call_and_wait(executor, self.list_client, ListControllers.Request(), deadline)
            answered = answered or listed is not None
            if listed is None:
                self.exit_note(f"round {attempt}: list_controllers did not answer")
                continue
            states = [c.state for c in listed.controller if c.name == self.controller_name]
            if not states or states[0] != "active":
                return self.exit_report("confirmed", states[0] if states else "absent")
            self.exit_note(f"round {attempt}: {self.controller_name} still active")
        if self.manager_gone():
            return self.exit_report("manager_gone")
        return self.exit_report("failed" if answered else "unanswered")

    def exit_note(self, text):
        print(f"lift_guard: exit deactivation, {text}", file=sys.stderr, flush=True)

    def exit_report(self, outcome, state=None):
        messages = {
            "confirmed": f"{self.controller_name} confirmed {state} at exit",
            "manager_gone": f"the controller manager {self.manager} has gone: {self.controller_name} stopped with it (the manager "
                            "deactivates its controllers when it shuts down). UNVERIFIED by the guard",
            "unanswered": f"the controller manager {self.manager} is in the graph but answered none of the exit's requests (it is "
                          f"stopping too): {self.controller_name} UNVERIFIED by the guard",
            "failed": f"{self.controller_name} NOT confirmed inactive at exit: the controller manager answered but did not confirm it",
        }
        print(f"lift_guard: {messages[outcome]}", file=sys.stderr, flush=True)
        return outcome

    # --- the decision and the status ------------------------------------------------------------------------------------------------------
    def step(self, periodic=False):
        if self.exiting:
            return
        now = self.now()
        codes = self.logic.evaluate(now)
        self.activating_since, overdue = lg.activation_overdue(codes, self.controller_state, self.activating_since, now,
                                                               self.switch_timeout_s)
        if overdue:
            self.get_logger().error(f"{self.controller_name} not active {self.switch_timeout_s:g} s after nothing blocked: SWITCH_FAILED")
            self.logic.switch_failed = True
            codes = self.logic.evaluate(now)
            self.activating_since = None
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


class StopFlag:
    """Set by SIGINT or SIGTERM. rclpy's own handlers are not installed (they would shut the context down before the exit deactivation)."""

    def __init__(self):
        self.signal = None

    def __call__(self, signum, frame):
        self.signal = signum


def main(argv=None):
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    stop = StopFlag()
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        node = LiftGuard()
    except lg.GuardConfigError as err:
        # Exits non-zero: the launch deactivates lift_controller on this exit, and it was spawned inactive, so the lift stays blocked.
        print(f"lift_guard: refusing to start: {err}", file=sys.stderr, flush=True)
        rclpy.try_shutdown()
        return 2
    outcome = "failed"
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    while stop.signal is None and rclpy.ok():
        executor.spin_once(timeout_sec=SPIN_PERIOD_S)
    outcome = node.deactivate_for_exit(executor)
    return EXIT_CODES[outcome]                    # no destroy_node or shutdown: see ros_exit.exit_process


if __name__ == "__main__":
    exit_process(main())
