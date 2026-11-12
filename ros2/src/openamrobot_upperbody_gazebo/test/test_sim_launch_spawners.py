# SPDX-License-Identifier: MIT
"""How sim.launch.py starts the controllers, on the launch description itself. No Gazebo, no ROS graph.

What is pinned, against the failure seen on 16 Oct (a 4-core machine with CPU-only rendering): the joint_state_broadcaster's first switch
timed out after the spawner's default 5 s, its spawner exited 1, and the arm and gripper spawners were started anyway (an OnProcessExit with
a fixed action list), so the stack ran with four of five controllers active.
  - the broadcaster's spawner is given --switch-timeout 30, and a service-call timeout longer than that (with the default 10 s the call was
    abandoned and retried and the spawner exited 1 although the switch completed);
  - the arm and gripper spawners are started only when that spawner exits 0, and not at all when it exits non-zero; since issue #17 the
    lift_controller spawner is part of the same chain.
"""
import importlib.util
import os
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_LAUNCH = os.path.join(HERE, os.pardir, "launch", "sim.launch.py")
ARM_AND_GRIPPER = ["left_arm_controller", "right_arm_controller", "left_gripper_controller", "right_gripper_controller"]
LIFT = ["lift_controller"]      # issue #17: lift_joint has a gz_ros2_control interface in this profile


@pytest.fixture(scope="module")
def sim():
    spec = importlib.util.spec_from_file_location("sim_launch_spawners_under_test", SIM_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def spawner_arguments(node):
    """The literal arguments of a spawner Node, after the executable (every argument is a plain string here)."""
    return ["".join(getattr(part, "text", "") for part in argument) for argument in node.cmd[1:]]


def controller_of(node):
    return spawner_arguments(node)[0]


def is_spawner(entity):
    return type(entity).__name__ == "Node" and entity.node_executable == "spawner"


def exit_event(action, returncode):
    from launch.events.process import ProcessExited
    return ProcessExited(action=action, name="process-6", cmd=["process"], cwd=None, env=None, pid=1, returncode=returncode)


def exit_handlers(description):
    from launch.actions import RegisterEventHandler
    from launch.event_handlers import OnProcessExit
    return [e.event_handler for e in description.entities
            if isinstance(e, RegisterEventHandler) and isinstance(e.event_handler, OnProcessExit)]


def broadcaster_and_chain(sim):
    description = sim.generate_launch_description()
    broadcasters = [e for e in description.entities if is_spawner(e) and controller_of(e) == "joint_state_broadcaster"]
    assert len(broadcasters) == 1, f"expected one joint_state_broadcaster spawner, found {len(broadcasters)}"
    # Since issue #44 there is a second OnProcessExit, the lift guard's: only one may handle the broadcaster's exit.
    handlers = [h for h in exit_handlers(description) if h.matches(exit_event(broadcasters[0], 0))]
    assert len(handlers) == 1, f"expected one OnProcessExit on the broadcaster's spawner (-> the rest), found {len(handlers)}"
    return broadcasters[0], handlers[0]


def started_after_exit(sim, returncode, kind=is_spawner):
    """The actions of `kind` (spawners by default) the launch would start when the broadcaster's spawner exits with `returncode`."""
    from launch import LaunchContext
    broadcaster, handler = broadcaster_and_chain(sim)
    return [a for a in (handler.handle(exit_event(broadcaster, returncode), LaunchContext()) or []) if kind(a)]


def test_the_broadcaster_spawner_has_a_30_s_switch_timeout(sim):
    broadcaster, _ = broadcaster_and_chain(sim)
    arguments = spawner_arguments(broadcaster)
    assert arguments[0] == "joint_state_broadcaster"
    assert "--switch-timeout" in arguments, f"no --switch-timeout on the broadcaster spawner: {arguments}"
    assert float(arguments[arguments.index("--switch-timeout") + 1]) == 30.0
    # the spawner's service-call timeout (default 10 s) must not end the wait first: it did, on 16 Oct, and the spawner exited 1
    assert "--service-call-timeout" in arguments, f"no --service-call-timeout on the broadcaster spawner: {arguments}"
    assert float(arguments[arguments.index("--service-call-timeout") + 1]) > 30.0


def test_the_arm_and_gripper_spawners_start_after_a_broadcaster_that_exited_0(sim):
    assert sorted(controller_of(a) for a in started_after_exit(sim, 0)) == sorted(ARM_AND_GRIPPER + LIFT)


@pytest.mark.parametrize("returncode", [1, -2])
def test_no_arm_or_gripper_spawner_starts_after_a_broadcaster_that_failed(sim, returncode):
    assert started_after_exit(sim, returncode) == [], "spawners started after the broadcaster failed"


# --- issue #44: the lift guard ------------------------------------------------------------------------------------------------------------
def is_node(executable):
    return lambda entity: type(entity).__name__ == "Node" and entity.node_executable == executable


def test_lift_controller_is_spawned_inactive(sim):
    lift = [a for a in started_after_exit(sim, 0) if controller_of(a) == "lift_controller"]
    assert len(lift) == 1 and "--inactive" in spawner_arguments(lift[0]), "lift_controller must start inactive: the guard activates it"


def test_the_arms_are_moved_to_park_once_with_the_spawners_and_not_after_a_failed_broadcaster(sim):
    assert len(started_after_exit(sim, 0, kind=is_node("park_arms.py"))) == 1
    assert started_after_exit(sim, 1, kind=is_node("park_arms.py")) == []


def test_the_guard_runs_and_its_exit_deactivates_lift_controller(sim):
    from launch import LaunchContext
    description = sim.generate_launch_description()
    guards = [e for e in description.entities if is_node("lift_guard.py")(e)]
    assert len(guards) == 1, f"expected one lift guard, found {len(guards)}"
    on_guard_exit = [h for h in exit_handlers(description) if h.matches(exit_event(guards[0], -9))]
    assert len(on_guard_exit) == 1, "no OnProcessExit on the guard: a SIGKILLed guard would leave lift_controller as it was"
    commands = [["".join(getattr(part, "text", "") for part in argument) for argument in action.cmd]
                for action in on_guard_exit[0].handle(exit_event(guards[0], -9), LaunchContext()) or []]
    deactivations = [c for c in commands if c[-6:] == ["ros2", "control", "switch_controllers", "--deactivate", "lift_controller",
                                                         "--best-effort"]]
    assert deactivations, commands
    assert deactivations[0][0] == "timeout", "the fallback must be bounded: ros2 control waits for a missing manager without end"
    assert not [e for e in description.entities if is_node("lift_guard_odom_stub.py")(e)], "gazebo has the DiffDrive /odom: no stub"
