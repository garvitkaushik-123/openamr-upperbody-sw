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


# --- a real process: the exit -----------------------------------------------------------------------------------------------------------
class FakeControllerManager(Node):
    """Serves /controller_manager/{switch_controller,list_controllers}: lift_controller starts inactive and follows the switches."""

    def __init__(self):
        super().__init__("fake_controller_manager")
        self.state = "inactive"
        self.switches = []                          # (request, time answered)
        self.create_service(SwitchController, "/controller_manager/switch_controller", self.on_switch)
        self.create_service(ListControllers, "/controller_manager/list_controllers", self.on_list)

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


@pytest.fixture
def manager():
    rclpy.init()
    node = FakeControllerManager()
    yield node
    node.destroy_node()
    rclpy.shutdown()


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
    request, answered_at = exit_switches[-1]
    assert list(request.deactivate_controllers) == ["lift_controller"] and not list(request.activate_controllers)
    assert request.strictness == SwitchController.Request.BEST_EFFORT
    assert answered_at <= exited_at and manager.state == "inactive"
