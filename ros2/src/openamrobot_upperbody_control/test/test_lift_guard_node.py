# SPDX-License-Identifier: MIT
"""The lift guard NODE (scripts/lift_guard.py, issue #44), with rclpy: what the decision's unit tests cannot show (review of #53).

  - Callback order (in process, deterministic): the guard's controller-manager clients are replaced by clients whose futures this test
    completes in the order it chooses, so an old list_controllers answer arrives AFTER a switch's answer, both ways round. The node's own
    callbacks then run as rclpy would run them. No timing is involved.
  - Exit (a real process): lift_guard.py runs as a child, this test serves the controller manager's switch_controller and
    list_controllers, and sends SIGTERM (and SIGINT): the guard must ask to deactivate lift_controller, best effort, and exit 0 only after
    the answer.
Not here: what the lift does (no controller runs). The black-box launch tests of #44 cover the stack.
"""
import importlib.util
import os
import signal
import subprocess
import sys
import time

import pytest
import rclpy
import yaml
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

HERE = os.path.dirname(os.path.abspath(__file__))
NODE_SCRIPT = os.path.join(HERE, os.pardir, "scripts", "lift_guard.py")
# The repository's SRDF, read from the source tree (as test_lift_guard_logic.py does): park is the stowed state.
REPO_SRDF = os.path.join(HERE, os.pardir, os.pardir, "openamrobot_upperbody_moveit", "config", "openamrobot.srdf")


def load_node_module():
    spec = importlib.util.spec_from_file_location("lift_guard_node_under_test", NODE_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def params_file(tmp_path, **extra):
    with open(REPO_SRDF) as f:
        srdf = f.read()
    path = tmp_path / "lift_guard.yaml"
    path.write_text(yaml.safe_dump({"lift_guard": {"ros__parameters": {"robot_description_semantic": srdf, **extra}}}))
    return str(path)


# --- in process: the order of the controller manager's answers ---------------------------------------------------------------------------
class HeldClient:
    """Stands in for a service client: every call returns a future that the test completes, when it chooses."""

    def __init__(self):
        self.calls = []

    def service_is_ready(self):
        return True

    def wait_for_service(self, timeout_sec=None):
        return True

    def call_async(self, request):
        future = Future()
        self.calls.append((request, future))
        return future


class Recorder:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


@pytest.fixture(scope="module")
def node_module():
    return load_node_module()


@pytest.fixture
def guard(node_module, tmp_path):
    rclpy.init(args=["--ros-args", "--params-file", params_file(tmp_path)])
    node = node_module.LiftGuard()
    node.list_client, node.switch_client, node.status_publisher = HeldClient(), HeldClient(), Recorder()
    clock = {"t": 100.0}
    node.now = lambda: clock["t"]
    node.clock = clock
    node.gone = {"value": False}                    # no real controller manager in this graph: the test says whether it has gone
    node.manager_gone = lambda: node.gone["value"]
    yield node
    node.destroy_node()
    rclpy.shutdown()


def park(side):
    return [0.1 if n == 4 else 0.0 for n in range(1, 8)]


def feed(node, t, linear_x=0.0):
    """Every input, fresh at `t`: both arms in park and still, the base at `linear_x`."""
    node.clock["t"] = t
    for side in ("left", "right"):
        joints = [f"openarm_{side}_joint{n}" for n in range(1, 8)]
        node.on_joint_state(JointState(name=joints, position=park(side)))
        node.on_controller_state(side, JointTrajectoryControllerState(
            joint_names=joints, reference=JointTrajectoryPoint(positions=park(side), velocities=[0.0] * 7)))
    odom = Odometry()
    odom.twist.twist.linear.x = linear_x
    node.on_odometry(odom)


def poll(node):
    """A list_controllers request, left pending (its future is returned)."""
    node.on_timer()
    return node.list_client.calls[-1][1]


def answer_list(future, state):
    future.set_result(ListControllers.Response(controller=[ControllerState(name="lift_controller", state=state)]))


def answer_switch(node, ok=True):
    request, future = node.switch_client.calls[-1]
    future.set_result(SwitchController.Response(ok=ok, message="" if ok else "refused"))
    return request


def level_and_values(msg):
    return msg.level, {kv.key: kv.value for kv in msg.values}


def last_status(node):
    return level_and_values(node.status_publisher.msgs[-1])


def make_active(node):
    """From start to lift_controller active: listed inactive, nothing blocking, activation asked and answered ok."""
    answer_list(poll(node), "inactive")
    feed(node, 100.0)
    feed(node, 101.0)                               # the base has been stopped for the hold time
    request = answer_switch(node)
    assert list(request.activate_controllers) == ["lift_controller"]
    assert last_status(node) == (DiagnosticStatus.OK, {"lift_controller": "active", "blocking": ""})


def test_an_old_active_snapshot_after_a_deactivation_never_brings_back_ok(guard):
    """#53 review (P2): a list requested before a deactivation, answered after it, must not make the guard believe the controller active."""
    make_active(guard)
    stale = poll(guard)                             # requested while lift_controller is active
    feed(guard, 101.1, linear_x=0.2)                # the base moves: a deactivation is asked at once
    request = answer_switch(guard)
    assert list(request.deactivate_controllers) == ["lift_controller"]
    answer_list(stale, "active")                    # the old snapshot arrives last
    assert guard.controller_state == "inactive"
    published_from = len(guard.status_publisher.msgs)
    for k in range(1, 9):                           # the base stops again; the guard may switch on, never claim OK before that
        feed(guard, 101.1 + 0.1 * k)
    after = [level_and_values(m) for m in guard.status_publisher.msgs[published_from:]]
    assert after and all(level != DiagnosticStatus.OK for level, _ in after), after
    assert list(guard.switch_client.calls[-1][0].activate_controllers) == ["lift_controller"], "it asks to switch on again"


def test_an_old_inactive_snapshot_after_an_activation_is_dropped(guard):
    """The inverse order: a list requested before an activation, answered after it, must not undo the newer state."""
    answer_list(poll(guard), "inactive")
    feed(guard, 100.0)
    stale = poll(guard)                             # requested before the activation is asked
    feed(guard, 101.0)                              # the activation is asked
    answer_switch(guard)
    answer_list(stale, "inactive")
    assert guard.controller_state == "active"
    feed(guard, 101.05)
    assert last_status(guard) == (DiagnosticStatus.OK, {"lift_controller": "active", "blocking": ""})


def test_a_current_list_answer_is_still_applied(guard):
    make_active(guard)
    answer_list(poll(guard), "inactive")            # requested after the last switch, answered with nothing pending
    assert guard.controller_state == "inactive"


def test_switching_on_is_warn_and_turns_into_switch_failed_without_an_answer(guard):
    answer_list(poll(guard), "inactive")
    feed(guard, 100.0)
    feed(guard, 101.0)                              # activation asked, never answered
    assert last_status(guard) == (DiagnosticStatus.WARN, {"lift_controller": "activating", "blocking": ""})
    timeout = guard.switch_timeout_s
    t = 101.0
    while t < 101.0 + timeout + 0.25:
        t += 0.25
        feed(guard, t)
    assert last_status(guard) == (DiagnosticStatus.ERROR, {"lift_controller": "inactive", "blocking": "SWITCH_FAILED"})


def test_an_outside_activation_while_blocked_is_undone_and_the_cause_named(guard):
    answer_list(poll(guard), "inactive")
    feed(guard, 100.0, linear_x=0.5)                # blocked: the base moves
    answer_list(poll(guard), "active")              # something else activated lift_controller
    assert list(guard.switch_client.calls[-1][0].deactivate_controllers) == ["lift_controller"]
    assert last_status(guard) == (DiagnosticStatus.ERROR, {"lift_controller": "active", "blocking": "BASE_MOVING"})


# --- in process: the exit (review of #53, P1: re-activation while waiting for the exit deactivation) ---------------------------------------
class ScriptedExecutor:
    """Stands in for the executor while the guard waits at exit. Every wait first runs what can interleave with it: a timer tick that was
    already due, the answer to any list request still outstanding (reporting `between`), `before` (e.g. answering an activation sent
    before the exit) and fresh SAFE inputs, so the decision on its own would ask for an activation. Then it completes the awaited future:
    a switch with `switch_ok`, a list with the next state of `confirm` (the last one repeats)."""

    def __init__(self, node, between="inactive", confirm=("inactive",), switch_ok=True, before=None):
        self.node, self.between, self.confirm, self.switch_ok, self.before = node, between, list(confirm), switch_ok, before

    def spin_until_future_complete(self, future, timeout_sec=None):
        node = self.node
        node.clock["t"] += 0.05
        node.on_timer()
        for _, pending in list(node.list_client.calls):
            if pending is not future and not pending.done():
                answer_list(pending, self.between)
        if self.before is not None:
            self.before()
        feed(node, node.clock["t"])
        for _, awaited in node.switch_client.calls:
            if awaited is future:
                awaited.set_result(SwitchController.Response(ok=self.switch_ok, message=""))
                return
        for _, awaited in node.list_client.calls:
            if awaited is future:
                answer_list(awaited, self.confirm.pop(0) if len(self.confirm) > 1 else self.confirm[0])
                return


def switches_since(node, start):
    return [request for request, _ in node.switch_client.calls[start:]]


def test_no_activation_is_asked_once_the_exit_begins(guard):
    """The reproduction from the review: active, safe inputs, a list answer 'inactive' arriving while the exit waits. Before the fix this
    asked for an activation and still reported success."""
    make_active(guard)
    start = len(guard.switch_client.calls)
    assert guard.deactivate_for_exit(ScriptedExecutor(guard, between="inactive", confirm=("inactive",))) == "confirmed"
    sent = switches_since(guard, start)
    assert sent and all(not list(r.activate_controllers) for r in sent), [list(r.activate_controllers) for r in sent]
    assert list(sent[0].deactivate_controllers) == ["lift_controller"]
    assert sent[0].strictness == SwitchController.Request.BEST_EFFORT


def test_an_activation_in_flight_at_exit_cannot_leave_the_controller_active(guard):
    """An activation sent before the signal, answered while the exit waits: the exit still ends with lift_controller confirmed inactive,
    and asks for nothing but deactivation."""
    answer_list(poll(guard), "inactive")
    feed(guard, 100.0)
    feed(guard, 101.0)                              # the activation is asked and left unanswered
    in_flight = guard.switch_client.calls[-1]
    assert list(in_flight[0].activate_controllers) == ["lift_controller"]
    start = len(guard.switch_client.calls)

    def manager_answers_the_activation():
        if not in_flight[1].done():
            in_flight[1].set_result(SwitchController.Response(ok=True, message=""))

    assert guard.deactivate_for_exit(ScriptedExecutor(guard, before=manager_answers_the_activation, confirm=("inactive",))) == "confirmed"
    assert all(not list(r.activate_controllers) for r in switches_since(guard, start))


def test_still_active_after_the_exit_deactivation_repeats_it(guard):
    make_active(guard)
    start = len(guard.switch_client.calls)
    assert guard.deactivate_for_exit(ScriptedExecutor(guard, confirm=("active", "inactive"))) == "confirmed"
    sent = switches_since(guard, start)
    assert [list(r.deactivate_controllers) for r in sent] == [["lift_controller"], ["lift_controller"]]


def test_never_confirmed_inactive_is_not_a_clean_exit(guard, node_module):
    make_active(guard)
    start = len(guard.switch_client.calls)
    assert guard.deactivate_for_exit(ScriptedExecutor(guard, confirm=("active",))) == "failed"
    assert len(switches_since(guard, start)) == node_module.EXIT_ATTEMPTS
    assert node_module.EXIT_CODES["failed"] != 0


def test_a_manager_that_answers_but_refuses_is_a_failed_exit(guard, node_module):
    """An answer that is not ok is not silence: the manager is up and did not deactivate. That stays exit 1."""
    make_active(guard)
    assert guard.deactivate_for_exit(ScriptedExecutor(guard, switch_ok=False)) == "failed"
    assert node_module.EXIT_CODES["failed"] != 0


def test_a_silent_manager_still_in_the_graph_ends_the_exit_within_its_budget(guard, node_module):
    """A manager that crashed can stay in the graph and never answer (seen with a crashed gz server). The exit must not outlast the
    launch's 5 s before SIGTERM: it ends within EXIT_BUDGET_S as "unanswered" (exit 0, logged UNVERIFIED). The gazebo teardown case: the
    manager inside gz sim stops answering at the signal and gz lives on (M07 on ROS 2 Jazzy)."""
    make_active(guard)

    class Silent(ScriptedExecutor):
        def spin_until_future_complete(self, future, timeout_sec=None):
            time.sleep(timeout_sec or 0.0)          # nothing ever answers

    started = time.monotonic()
    assert guard.deactivate_for_exit(Silent(guard)) == "unanswered"
    assert node_module.EXIT_CODES["unanswered"] == 0
    took = time.monotonic() - started
    assert took <= node_module.EXIT_BUDGET_S + 0.5 < 5.0, f"the exit took {took:.2f} s"


def test_a_manager_already_gone_ends_the_exit_at_once_as_unverified(guard, node_module, capsys):
    """A normal teardown stops the controller manager with the guard, often first (#60's run, the mock stack's log): the controller stopped
    with its manager. Nothing is asked of a manager that is not there; exit 0, and the log says UNVERIFIED by the guard."""
    make_active(guard)
    start = len(guard.switch_client.calls)
    guard.gone["value"] = True
    assert guard.deactivate_for_exit(ScriptedExecutor(guard)) == "manager_gone"
    assert switches_since(guard, start) == []
    assert node_module.EXIT_CODES["manager_gone"] == 0
    assert "UNVERIFIED by the guard" in capsys.readouterr().err


def test_a_manager_that_leaves_during_the_exit_ends_it_as_unverified(guard):
    """The deactivation goes unanswered (the manager is shutting down) and the manager then leaves the graph."""
    make_active(guard)

    class ManagerLeaves(ScriptedExecutor):
        def spin_until_future_complete(self, future, timeout_sec=None):
            guard.gone["value"] = True              # nothing answers; by the next round the manager has gone

    assert guard.deactivate_for_exit(ManagerLeaves(guard)) == "manager_gone"
    assert len(guard.switch_client.calls) and list(guard.switch_client.calls[-1][0].deactivate_controllers) == ["lift_controller"]


def test_nothing_is_decided_or_published_once_exiting(guard):
    make_active(guard)
    guard.deactivate_for_exit(ScriptedExecutor(guard))
    published, switches = len(guard.status_publisher.msgs), len(guard.switch_client.calls)
    feed(guard, guard.clock["t"] + 1.0, linear_x=0.5)
    feed(guard, guard.clock["t"] + 2.0)
    guard.on_timer()
    assert (len(guard.status_publisher.msgs), len(guard.switch_client.calls)) == (published, switches)


# --- a real process: the exit -----------------------------------------------------------------------------------------------------------
class FakeControllerManager(Node):
    """Serves /controller_manager/{switch_controller,list_controllers}: lift_controller starts inactive and follows the switches. With
    `safe_inputs` it also publishes, at 20 Hz, both arms in park and still (/joint_states and both controller_state topics) and a stopped
    base (/odom): everything the guard needs to activate lift_controller, and to try again, if it could, while it exits."""

    def __init__(self, safe_inputs=False):
        super().__init__("controller_manager")      # the real manager's name: the guard looks for it in the graph at exit
        self.state = "inactive"
        self.switches = []                          # (request, time answered)
        self.create_service(SwitchController, "/controller_manager/switch_controller", self.on_switch)
        self.create_service(ListControllers, "/controller_manager/list_controllers", self.on_list)
        if safe_inputs:
            self.joints = self.create_publisher(JointState, "/joint_states", 10)
            self.references = {side: self.create_publisher(JointTrajectoryControllerState, f"/{side}_arm_controller/controller_state", 10)
                               for side in ("left", "right")}
            self.odom = self.create_publisher(Odometry, "/odom", 10)
            self.create_timer(0.05, self.publish_safe_inputs)

    def publish_safe_inputs(self):
        for side, publisher in self.references.items():
            joints = [f"openarm_{side}_joint{n}" for n in range(1, 8)]
            self.joints.publish(JointState(name=joints, position=park(side)))
            publisher.publish(JointTrajectoryControllerState(
                joint_names=joints, reference=JointTrajectoryPoint(positions=park(side), velocities=[0.0] * 7)))
        self.odom.publish(Odometry())

    def on_switch(self, request, response):
        if "lift_controller" in request.activate_controllers:
            self.state = "active"
        if "lift_controller" in request.deactivate_controllers:
            self.state = "inactive"
        self.switches.append((request, time.monotonic()))
        response.ok = True
        return response

    def on_list(self, request, response):
        response.controller = [ControllerState(name="lift_controller", state=self.state)]
        return response


@pytest.fixture(params=[False, True], ids=["blocked", "safe_inputs"])
def manager(request):
    rclpy.init()
    node = FakeControllerManager(safe_inputs=request.param)
    node.safe_inputs = request.param
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_when_the_manager_stops_first_the_guard_exits_0_and_says_unverified(tmp_path):
    """The teardown order seen in #60 and in the mock stack: the controller manager exits before the guard. The guard must not wait out
    every round against a manager that has left the graph, nor exit 1 (every process's exit code is checked, P24)."""
    rclpy.init()
    manager = FakeControllerManager(safe_inputs=True)
    child = subprocess.Popen([sys.executable, NODE_SCRIPT, "--ros-args", "--params-file", params_file(tmp_path)],
                             stderr=subprocess.PIPE, text=True)
    try:
        spin_for(manager, 30.0, until=lambda: manager.state == "active")
        assert manager.state == "active", "the guard never activated lift_controller with safe inputs"
        manager.destroy_node()                      # the manager stops first
        observer = Node("observer")
        end = time.monotonic() + 15.0
        while time.monotonic() < end and ("controller_manager", "/") in observer.get_node_names_and_namespaces():
            rclpy.spin_once(observer, timeout_sec=0.1)
        child.send_signal(signal.SIGTERM)
        started = time.monotonic()
        _, err = child.communicate(timeout=20.0)
        took = time.monotonic() - started
    finally:
        if child.poll() is None:
            child.kill()
            child.wait()
        rclpy.shutdown()
    assert child.returncode == 0, err
    assert "UNVERIFIED by the guard" in err, err
    assert took < 5.0, f"the guard took {took:.1f} s to exit with the manager gone (launch escalates to SIGTERM after 5 s)"


def spin_for(node, seconds, until=lambda: False):
    end = time.monotonic() + seconds
    while time.monotonic() < end and not until():
        rclpy.spin_once(node, timeout_sec=0.05)


@pytest.mark.parametrize("sig", [signal.SIGTERM, signal.SIGINT])
def test_on_a_signal_the_guard_deactivates_lift_controller_before_it_exits(manager, tmp_path, sig):
    """Accepted on #44: the node deactivates lift_controller itself on SIGTERM before exiting (the launch handler stays for SIGKILL)."""
    child = subprocess.Popen([sys.executable, NODE_SCRIPT, "--ros-args", "--params-file", params_file(tmp_path, switch_timeout_s=5.0)])
    try:
        spin_for(manager, 30.0, until=lambda: "lift_guard" in manager.get_node_names())
        assert "lift_guard" in manager.get_node_names(), "the guard did not come up"
        spin_for(manager, 1.0)                      # let it poll list_controllers once
        if manager.safe_inputs:                     # the guard sees safe inputs: it must have switched lift_controller on
            spin_for(manager, 15.0, until=lambda: manager.state == "active")
            assert manager.state == "active", "the guard never activated lift_controller with safe inputs"
        before = len(manager.switches)
        child.send_signal(sig)
        spin_for(manager, 15.0, until=lambda: child.poll() is not None)
        exited_at = time.monotonic()
        assert child.poll() == 0, f"the guard exited {child.poll()} after {sig.name}"
    finally:
        if child.poll() is None:
            child.kill()
            child.wait()
    exit_switches = manager.switches[before:]
    assert exit_switches, "no switch was asked for at exit"
    assert all(not list(r.activate_controllers) for r, _ in exit_switches), "an activation was asked after the signal"
    request, answered_at = exit_switches[-1]
    assert list(request.deactivate_controllers) == ["lift_controller"] and not list(request.activate_controllers)
    assert request.strictness == SwitchController.Request.BEST_EFFORT
    assert answered_at <= exited_at and manager.state == "inactive"


# --- park_arms.py: a goal another command replaced is not a failure (full colcon run, L01 and G01) ---------------------------------------
def _park_arms():
    spec = importlib.util.spec_from_file_location("park_arms_under_test", os.path.join(HERE, os.pardir, "scripts", "park_arms.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("code, text, ok", [
    (0, "", True),
    # joint_trajectory_controller's own strings (Jazzy 4.x), seen when L01's goal arrived 0.1 s before park's ended:
    (-1, "Current goal preempted by new incoming action.", True),
    (-1, "Current goal cancelled during deactivate transition.", True),
    (-1, "Joints on incoming goal don't match the controller joints.", False),
    (-4, "Aborted due to path tolerance violation", False),
    (-5, "Aborted due to goal tolerance violation", False),
])
def test_park_arms_counts_a_superseded_goal_as_not_failed(code, text, ok):
    from control_msgs.action import FollowJointTrajectory
    result = FollowJointTrajectory.Result(error_code=code, error_string=text)
    assert _park_arms().result_outcome(result)[0] is ok


# --- ros_exit.exit_process: the helpers end without destroying the DDS participant (Fast DDS deadlock at destroy_participant) -----------
@pytest.mark.parametrize("code", [0, 1, 2])
def test_exit_process_exits_with_the_code_and_flushes_what_was_printed(code):
    script = ("from openamrobot_upperbody_control.ros_exit import exit_process\n"
              "import rclpy\nfrom rclpy.node import Node\n"
              "rclpy.init()\nnode = Node('exit_process_probe')\n"      # a live node and participant, as in the helpers
              "print('before exit', end='')\n"
              f"exit_process({code})\n"
              "print('not reached')\n")
    done = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, timeout=30)
    assert done.returncode == code, done.stderr
    assert done.stdout == "before exit"
