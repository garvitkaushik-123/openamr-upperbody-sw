# SPDX-License-Identifier: MIT
"""M01: every named pose of both arms (and both grippers) plans and executes on `mock`, through move_group, with a final joint error
below 0.01 rad. One lift calibration stop per run (the argument `lift_stop:=L1000|L1175|L1350`, default L1000; the stops are CALIBRATION IDs of
option B's lift, joint positions and not configuration IDs); CMake registers it once per stop.

S8 with the lift planned (issue #17): the lift is a planned joint (MoveIt group `lift`, lift_controller). Each run starts at L1000, puts both
arms in `park` (group both_arms), PLANS the lift to its stop (from L1000 to L1000 it goes to L1175 and back), runs the twelve poses there, and
plans the lift back to L1000. The lift moves only between the stops and only with both arms reading `park` (a7_support.move_lift): the register
LIFT rule as scenario design, not an interlock. The twelve pose rows are the 12 of 12; the lift and park moves are a record of their own
(`lift_scenario`), and every one of them must succeed too.

The stack is `moveit.launch.py sim:=mock`: robot_state_publisher, ros2_control_node on mock_components, six controllers, move_group.
Each pose is a MoveGroup action goal made of exactly the SRDF's values for that group state: plan, then execute through MoveIt's
trajectory execution manager and moveit_simple_controller_manager into left/right_arm_controller (FollowJointTrajectory) and
left/right_gripper_controller (GripperCommand, option 1 of ADR-009 point 4). The grippers go to `open` and back to `closed` with each
hand's OWN value (left +0.7854, right -0.7854), so the per-hand sign is exercised end to end and not only checked statically (M05).

`simulation` class: mock hardware reflects its command back as state, so this shows that MoveIt, the controller handles and the controller
configuration agree with each other and with the SRDF at this lift stop. It says nothing about tracking, loads or limits of a real arm, and
the acceleration limits it plans under are engineering defaults (ADR-009): "plans and executes in simulation under declared OpenAMRobot
limits".
"""
import os
import random
import sys
import unittest

os.environ["ROS_DOMAIN_ID"] = str(random.randint(1, 100))
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

def _lift_stop_from_arguments(default="L1000"):
    """`lift_stop:=...` as `launch_test` was given it (CMake's ARGS): the test module sees the same argv the launch service does."""
    for argument in sys.argv[1:]:
        if argument.startswith("lift_stop:="):
            return argument.split(":=", 1)[1]
    return default


import launch  # noqa: E402
import launch_testing  # noqa: E402
import launch_testing.actions  # noqa: E402
import launch_testing.asserts  # noqa: E402
import launch_testing.markers  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: E402

from openamrobot_upperbody_moveit.move_group_client import MoveGroupClient  # noqa: E402
from openamrobot_upperbody_moveit.named_poses import ARM_GROUP, GRIPPER_JOINT, SIDES  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from a7_support import (A7_ENABLED, ACCELERATION_SCALING, CONTROLLERS_ACTIVE_WITHIN_S, LIFT_JOINT, LIFT_TOLERANCE_M, SIM_CONTROLLERS,  # noqa: E402
                        SKIP_REASON, TOLERANCE, VELOCITY_SCALING, clamp_record, named_pose_plan, run_s8, stack_alive, stack_healthy,
                        wait_for_controllers_active, write_run_sidecar)

LIFT_STOP = _lift_stop_from_arguments()
READY_WITHIN_S = 90.0


@pytest.mark.launch_test
def generate_test_description():
    if not A7_ENABLED:
        # A launch with no process at all races its own tests ("Processes under test stopped before tests completed": about one run in
        # three failed, found in P12A, and the A6 report dropped those failures because it did not list M01/M02). One process that outlives
        # the skipped test keeps the launch up until launch_testing stops it.
        return launch.LaunchDescription([launch.actions.ExecuteProcess(cmd=["sleep", "30"]), launch_testing.actions.ReadyToTest()]), {}
    moveit = os.path.join(get_package_share_directory("openamrobot_upperbody_bringup"), "launch", "moveit.launch.py")
    from openamrobot_upperbody_moveit.moveit_config import lift_stops
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit),
            launch_arguments={"sim": "mock", "rviz": "false"}.items()),
        launch_testing.actions.ReadyToTest(),
    ]), {}


@unittest.skipUnless(A7_ENABLED, SKIP_REASON)  # tracking: openAMRobot/openamr-upperbody-sw#16
class TestNamedPosesOnMock(unittest.TestCase):
    record = {}

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("m01_test")
        cls.client = MoveGroupClient(cls.node)

    @classmethod
    def tearDownClass(cls):
        rows = cls.record.get("poses", [])
        lift = cls.record.get("lift_scenario") or {}
        ok = bool(rows) and all(r["ok"] for r in rows) and bool(lift.get("moves")) and lift.get("moves_ok") == lift.get("moves_run")
        from openamrobot_upperbody_moveit.moveit_config import lift_stops
        details = {"lift_stop": LIFT_STOP, "lift_joint_position_m": lift_stops()[LIFT_STOP], "lift_planned": True,
                   "lift_state_source": "lift_joint's mock interface (ros2_control.xacro openamr_lift_mock), moved by lift_controller",
                   "lift_tolerance_m": LIFT_TOLERANCE_M,
                   "hardware": "mock_components/GenericSystem", "tolerance_rad": TOLERANCE,
                   "velocity_scaling": VELOCITY_SCALING, "acceleration_scaling": ACCELERATION_SCALING,
                   "gripper_interface": "GripperCommand via moveit_simple_controller_manager (ADR-009 option 1)",
                   "poses_run": len(rows), "poses_ok": sum(1 for r in rows if r["ok"]),
                   "requests_per_pose": 1,       # no retry: a refused request is a FAILed pose (see MoveGroupClient.move)
                   "worst_final_error_rad": max((r["final_max_joint_error_rad"] for r in rows), default=None),
                   "clamp": clamp_record(cls.client),
                   "poses": rows, **{k: v for k, v in cls.record.items() if k != "poses"}}
        write_run_sidecar("M01", f"mock_{LIFT_STOP}", f"named poses plan and execute on mock at {LIFT_STOP}", "PASS" if ok else "FAIL", details)
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_m01_1_every_named_pose_plans_and_executes_with_a_final_error_under_tolerance(self):
        self.assertTrue(self.client.wait_for_server(READY_WITHIN_S), f"move_group's /move_action did not appear within {READY_WITHIN_S:.0f} s")
        # Before the first pose, as in M02 and M07: every arm and gripper controller must be ACTIVE. On a slow machine the first left_arm goal
        # was sent before the mock controllers were up ("Action client not connected") and read as a robot failure. If they never activate
        # this FAILs here, with their states, and no pose is sent.
        ok, states, waited = wait_for_controllers_active(self.node, CONTROLLERS_ACTIVE_WITHIN_S, names=SIM_CONTROLLERS)
        self.record["controllers"] = {"required": list(SIM_CONTROLLERS), "states": states, "waited_s": waited, "timeout_s": CONTROLLERS_ACTIVE_WITHIN_S}
        self.record["controllers_active"] = ok
        self.assertTrue(ok, f"the controllers did not all reach active within {CONTROLLERS_ACTIVE_WITHIN_S:.0f} s (waited {waited} s): {states}; "
                            "no pose was sent")
        joints = [f"openarm_{side}_joint{i}" for side in SIDES for i in range(1, 8)] + [GRIPPER_JOINT[s] for s in SIDES] + [LIFT_JOINT]
        self.assertTrue(self.client.wait_for_joints(joints, 60.0), "/joint_states never listed all 17 controlled joints")
        plan = named_pose_plan()
        self.record["poses"], self.record["lift_scenario"], failures, self.record["start"] = run_s8(self.client, LIFT_STOP, plan)
        self.assertFalse(failures, "\n".join(failures))
        self.assertEqual(len(self.record["poses"]), 4 * 2 + 2 * 2, "both arms x four moves, both grippers x two")


EXPECTED_PROCESSES = ["robot_state_publisher", "ros2_control_node", "spawner", "base_joint_states_stub", "move_group", "joint_state_clamp"]


@unittest.skipUnless(A7_ENABLED, SKIP_REASON)  # tracking: openAMRobot/openamr-upperbody-sw#16
class TestProcessesAlive(unittest.TestCase):
    def test_no_process_of_the_launch_died_while_the_tests_were_running(self, proc_info):
        stack_alive(proc_info)


@launch_testing.post_shutdown_test()
@unittest.skipUnless(A7_ENABLED, SKIP_REASON)  # tracking: openAMRobot/openamr-upperbody-sw#16
class TestShutdown(unittest.TestCase):
    def test_m01_processes_started_and_exited_cleanly(self, proc_info, proc_output):
        stack_healthy(proc_info, expected=EXPECTED_PROCESSES, sidecar=("M01", f"mock_{LIFT_STOP}"), proc_output=proc_output)
