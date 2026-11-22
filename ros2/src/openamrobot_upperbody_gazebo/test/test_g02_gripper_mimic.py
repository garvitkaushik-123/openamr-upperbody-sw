# SPDX-License-Identifier: MIT
"""G02: in Gazebo both grippers reach a goal off zero (they open part way), and finger_joint2 follows finger_joint1 in the physics (the P06 mimic question).

Why this test passes `physics_engine` explicitly (P10 task 0, 6 Oct): sim.launch.py defaults to DART, the engine platform-sw
tuned the base on, and a gripper-mimic convenience is not grounds to re-baseline someone else's physics. The mimic and the
gripper need the other engine, and that is the limitation this check makes visible instead of hiding behind a default:

  Bullet-Featherstone (passed here)   the GripperActionController goal is reached and the URDF <mimic> is honoured by the
                                      engine: finger_joint2 equals finger_joint1.
  DART (the launch default)           the fingers do not move under position or velocity commands (effort works), and the
                                      engine logs that it has no mimic constraint. Launched on DART this test fails at the
                                      first goal: that is the documented limitation, not a defect of the test.

So this test is the guard on the physics engine as much as on the mimic. It reads
the finger angles from Gazebo's link poses, because /joint_states computes finger_joint2 from finger_joint1 and so could never
show the follower failing to move. The right gripper's joint range is [-0.7854, 0] (mirrored), so its goal is negative.

The gripper controller type and action interface are not touched by any of this (ADR-009 point 4): the goals are
control_msgs/action/GripperCommand, exactly as upstream's MoveIt configuration sends them.
"""
import os
import random
import sys
import time
import unittest

os.environ["ROS_DOMAIN_ID"] = str(random.randint(101, 200))
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
os.environ["GZ_PARTITION"] = f"g02_{os.getpid()}"
PARTITION = os.environ["GZ_PARTITION"]      # every process of the launch inherits it: it is what makes a surviving gz server THIS run's

import launch  # noqa: E402
import launch_testing.actions  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from control_msgs.action import GripperCommand  # noqa: E402
from controller_manager_msgs.srv import ListControllers  # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_test_support import amend_g_sidecar, finger_angles, gz_link_poses, launch_engine_constants, write_g_sidecar  # noqa: E402

from ament_index_python.packages import get_package_prefix as _prefix  # noqa: E402

sys.path.insert(0, os.path.join(_prefix("openamrobot_upperbody_description"), "lib", "openamrobot_upperbody_description"))
from launch_guard import (MOVE_GROUP_ALLOWED, SIM_ALLOWED, assert_launch_healthy, assert_no_process_died,  # noqa: E402
                          gz_server_shutdown_report, gz_survivor_message)

EXPECTED_PROCESSES = ["gazebo", "robot_state_publisher", "parameter_bridge", "create", "joint_state_publisher", "spawner"]


# The engine this check REQUIRES, passed to the launch on purpose; sim.launch.py's own default is DART (see the docstring).
ENGINE_THE_MIMIC_NEEDS = launch_engine_constants()[1]

CONTROLLERS = ("joint_state_broadcaster", "left_arm_controller", "right_arm_controller",
               "left_gripper_controller", "right_gripper_controller")
GOALS = {"left": 0.4, "right": -0.4}        # rad; the right finger's range is mirrored
ACTIVE_WITHIN_S = 75.0
SETTLE_S = 3.0                              # wall seconds after the goal result for the fingers to come to rest
FOLLOW_TOLERANCE = 0.02                     # rad between finger_joint2 and finger_joint1, and between finger_joint1 and its goal


@pytest.mark.launch_test
def generate_test_description():
    sim = os.path.join(get_package_share_directory("openamrobot_upperbody_gazebo"), "launch", "sim.launch.py")
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource(sim),
                                                launch_arguments={"headless": "true",
                                                                                 "physics_engine": ENGINE_THE_MIMIC_NEEDS}.items()),
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestGripperMimic(unittest.TestCase):
    record = {}

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("g02_test", parameter_overrides=[Parameter("use_sim_time", value=True)])
        cls.started = time.monotonic()

    @classmethod
    def tearDownClass(cls):
        write_g_sidecar("G02", "gripper_mimic", "grippers reach their goals in Gazebo and finger_joint2 follows finger_joint1 in the physics",
                        cls.record, expected=("left", "right"), physics_engine=ENGINE_THE_MIMIC_NEEDS)
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return predicate()

    def _wait_for_controllers(self):
        client = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        self.assertTrue(client.wait_for_service(timeout_sec=ACTIVE_WITHIN_S), "no controller_manager")
        states = {}
        while time.monotonic() < self.started + ACTIVE_WITHIN_S:
            future = client.call_async(ListControllers.Request())
            self._spin_until(future.done, 2.0)
            states = {c.name: c.state for c in future.result().controller} if future.done() and future.result() else {}
            if all(states.get(name) == "active" for name in CONTROLLERS):
                return
            time.sleep(0.25)
        self.fail(f"controllers not all active: {states}")

    def _close(self, side):
        client = ActionClient(self.node, GripperCommand, f"/{side}_gripper_controller/gripper_cmd")
        self.assertTrue(client.wait_for_server(timeout_sec=10.0), f"no {side} gripper action server")
        goal = GripperCommand.Goal()
        goal.command.position = GOALS[side]
        goal.command.max_effort = 5.0
        send = client.send_goal_async(goal)
        self.assertTrue(self._spin_until(send.done, 10.0), "goal was not answered")
        self.assertTrue(send.result().accepted, "gripper goal rejected")
        result = send.result().get_result_async()
        self.assertTrue(self._spin_until(result.done, 60.0), f"{side} gripper goal never finished (stalled?)")
        outcome = result.result().result
        self.assertTrue(outcome.reached_goal and not outcome.stalled,
                        f"{side} gripper: reached={outcome.reached_goal} stalled={outcome.stalled} position={outcome.position:+.4f}. "
                        "On DART (the launch default) the fingers do not move; this test passes the engine explicitly")
        self._spin_until(lambda: False, SETTLE_S)
        before_failure = finger_angles(gz_link_poses(), side)
        return outcome, before_failure

    def test_g02_1_both_grippers_close_and_the_mimic_follows(self):
        self._wait_for_controllers()
        for side in ("left", "right"):
            outcome, (j1, j2) = self._close(side)
            goal = GOALS[side]
            self.assertAlmostEqual(j1, goal, delta=FOLLOW_TOLERANCE, msg=f"{side} finger_joint1 physically at {j1:+.4f}, goal {goal}")
            self.assertAlmostEqual(j2, j1, delta=FOLLOW_TOLERANCE,
                                   msg=f"{side} finger_joint2 physically at {j2:+.4f} while finger_joint1 is at {j1:+.4f}: the mimic did not follow")
            self.record[side] = {"goal_rad": goal, "reached_goal": outcome.reached_goal, "stalled": outcome.stalled,
                                 "finger_joint1_physical_rad": round(j1, 4), "finger_joint2_physical_rad": round(j2, 4),
                                 "follow_tolerance_rad": FOLLOW_TOLERANCE,
                                 "measured_from": "gz /world/empty/dynamic_pose/info (link poses), not /joint_states"}


class TestProcessesAlive(unittest.TestCase):
    def test_no_process_of_the_launch_died_while_the_tests_were_running(self, proc_info):
        assert_no_process_died(proc_info)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_g02_processes_started_and_exited_cleanly(self, proc_info, proc_output):
        """The processes of the launch started, and none ended with a code that is neither 0 nor a documented shutdown behaviour (launch_guard.py)."""
        assert_launch_healthy(proc_info, expected=EXPECTED_PROCESSES, allowed=SIM_ALLOWED,
                              record=lambda records: amend_g_sidecar("G02", "gripper_mimic", "process_exits", records), proc_output=proc_output)

    def test_g02_no_gz_server_of_this_run_survives_the_teardown(self):
        """P16: after the launch has been torn down no `gz sim` carrying this run's GZ_PARTITION is still alive. `gz sim` is a ruby wrapper
        and the server it forks; the wrapper is what the launch owns and what the check above reads, and the server was left running after
        about 2 of 8 A7 runs (and the first G02 run of P16) with nothing noticing. FAIL, not WARN: a simulator that outlives its test
        holds three cores and a transport partition, and is how the next run's numbers moved. The verdict is taken before the survivor is
        stopped (launch_guard.gz_server_shutdown_report); the finding is also written into G02's sidecar, because the report lists the
        names of failed tests and not their messages."""
        report = gz_server_shutdown_report(PARTITION)
        amend_g_sidecar("G02", "gripper_mimic", "gz_server_after_shutdown", report, fail=bool(report["survivors"]))
        self.assertFalse(report["survivors"], gz_survivor_message(report))

