# SPDX-License-Identifier: MIT
"""M04: a goal that puts the left gripper inside the nearest REACHABLE structure volume is REJECTED by planning, and the test says which of the three
things that could happen did: the pipeline refused it, the planner searched and failed, or the planner ran out of time. They are different
claims: "rejected" is the pipeline looking at the goal and refusing it at once; "failed" is a search that gave up; "timed_out" is a search that
used its whole budget. Only the first is the robot refusing to hurt itself; the other two would be a robot that tried.

WHICH VOLUME (VERIFICATION.md M04, corrected 7 Oct: the row said "base collision volume", which was the wrong phrase): the arms cannot reach the
platform's base body at all, so the volume under test is the nearest one they CAN reach. In 6000 random left-arm states at option B's baseline (lift at
L1000, numpy default_rng(4242)) the left gripper touched `base_link`, the wheels, the casters, the lidar and the camera body in none of them (and M03's
states found no collision of any arm link with `base_link` at any lift stop), while it touched `arm_profile_link`, option B's 340 mm HFS6-60120
profile on the carriage that the arms bolt to, in 249. P15 replaced the mast with the lift and `mast_base_link` is gone, so the nearest base-side
volume a left gripper can be put in is `arm_profile_link`'s, and the record says so. That the base body is out of reach is itself a finding about the arm
geometry (A4's question, not ours). The same search found the left gripper in contact with the RIGHT arm's own links in about 3 per cent of the
random left-arm states (openarm_right_base_link in 73 of 6000): the arms are 62 mm apart at their bases (option B's y = +-0.031), a reach fact for A4.

The fixture is a left-arm joint state found by that search (numpy default_rng(4242), sample index 22 of the left-arm-only search, whose only contact
is a left gripper link with `arm_profile_link`); the test checks, through move_group's /check_state_validity, that it really collides with the profile
before using it, so a change of the model fails here with that message and not by quietly testing a goal that no longer collides.

Two goals are sent, both plan-only (nothing executes) and through move_group's /plan_kinematic_path service: a JOINT goal (the fixture's joint
values) and a POSE goal (the position and orientation of the left gripper at the fixture, from /compute_fk). A third request, a named pose that
is free, must plan: the harness is not vacuous. The service and not the MoveGroup action, because the action collapses every planning failure
into the generic FAILURE (99999) with no planning time, which cannot tell a refusal from a timeout; the service returns the pipeline's own code
and planning time.

What the installed MoveIt does (read from move_group's log, MoveIt 2.12.4): for the JOINT goal OMPL finds the contact at once ("Found a contact
between arm_profile_link and openarm_left_ee_base_link"), logs "Unable to sample any valid states for goal tree" after about 20 ms and fails with
GOAL_STATE_INVALID: a refusal. For the POSE goal it spends the WHOLE planning budget looking for a collision-free goal (5.0 s between the same
two log lines) and then fails with the same code GOAL_STATE_INVALID: a search that timed out, with a refusal's name. Same code, different claims."""
import os
import random
import sys
import unittest

os.environ["ROS_DOMAIN_ID"] = str(random.randint(1, 100))
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

import launch  # noqa: E402
import launch_ros.actions  # noqa: E402
import launch_testing.actions  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import Pose  # noqa: E402
from moveit_msgs.msg import BoundingVolume, Constraints, OrientationConstraint, PositionConstraint, RobotState  # noqa: E402
from moveit_msgs.srv import GetPositionFK, GetStateValidity  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from shape_msgs.msg import SolidPrimitive  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))          # launch_test is not pytest: conftest is not on the path
from conftest import write_sidecar  # noqa: E402
from openamrobot_upperbody_moveit.move_group_client import MoveGroupClient  # noqa: E402
from openamrobot_upperbody_moveit.moveit_config import moveit_config, srdf_path  # noqa: E402
from openamrobot_upperbody_moveit.named_poses import read_group_states  # noqa: E402

from ament_index_python.packages import get_package_prefix as _prefix  # noqa: E402

sys.path.insert(0, os.path.join(_prefix("openamrobot_upperbody_description"), "lib", "openamrobot_upperbody_description"))
from launch_guard import MOVE_GROUP_ALLOWED, SIM_ALLOWED, assert_launch_healthy, assert_no_process_died  # noqa: E402

EXPECTED_PROCESSES = ["joint_state_publisher", "move_group"]


# Found by the sampler (see the docstring). Left arm only; everything else at zero.
FIXTURE = {"openarm_left_joint1": -0.8927768711545525, "openarm_left_joint2": -2.6599343349117697,
           "openarm_left_joint3": -0.995094804778054, "openarm_left_joint4": 2.391156576052684,
           "openarm_left_joint5": 0.6164772003836043, "openarm_left_joint6": -0.4074526110205802,
           "openarm_left_joint7": -1.091442811392671}
GRIPPER_LINKS = {"openarm_left_ee_base_link", "openarm_left_ee_link1", "openarm_left_ee_link2"}
VOLUME = "arm_profile_link"
PLANNING_BUDGET_S = 5.0
REJECTED_WITHIN_S = 0.5          # beyond the baseline: a refusal is immediate (about 20 ms measured); a search that ran out of time is not
EE_LINK = "openarm_left_ee_base_link"


def classify(outcome, baseline_wall, budget=PLANNING_BUDGET_S):
    """'rejected' | 'timed_out' | 'failed' | 'succeeded' | 'not_answered'.

    In MoveIt 2.12.4 neither the action nor the service tells the client WHY planning failed: both answer the generic FAILURE (99999) with a
    planning time of 0 (the planner's own GOAL_STATE_INVALID is only in move_group's log). So the claim is made on time spent beyond the
    baseline, the wall time of the free control request (move_group adds a fixed second to every request), plus the planner's log line,
    which the test asserts separately for the joint goal:

    rejected   answered within REJECTED_WITHIN_S of the baseline: the planner looked at the goal and gave up at once
    timed_out  took (about) the whole planning budget more than the baseline: it searched until it ran out of time
    failed     anything in between: it searched, briefly, and gave up"""
    if not outcome.accepted:
        return "not_answered"
    if outcome.success:
        return "succeeded"
    extra = outcome.wall_seconds - baseline_wall
    if outcome.name == "TIMED_OUT" or extra >= 0.9 * budget:
        return "timed_out"
    if extra < REJECTED_WITHIN_S:
        return "rejected"
    return "failed"


@pytest.mark.launch_test
def generate_test_description():
    config = moveit_config("bp000", "mock")
    parameters = [config.to_dict(), {"publish_robot_description_semantic": True}]
    return launch.LaunchDescription([
        # one publisher of the complete joint state (wheels included), all at zero: the robot at rest
        launch_ros.actions.Node(package="joint_state_publisher", executable="joint_state_publisher",
                                parameters=[config.robot_description, {"rate": 20}], output="log"),
        launch_ros.actions.Node(package="moveit_ros_move_group", executable="move_group", output="screen", parameters=parameters),
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestCollisionGoalRejected(unittest.TestCase):
    record = {}

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("m04_test")
        cls.client = MoveGroupClient(cls.node)
        cls.validity = cls.node.create_client(GetStateValidity, "/check_state_validity")
        cls.fk = cls.node.create_client(GetPositionFK, "/compute_fk")

    @classmethod
    def tearDownClass(cls):
        record = cls.record
        joint = record.get("joint_goal", {})
        status = "PASS" if (joint.get("classification") == "rejected" and joint.get("planner_verdict_seen_in_move_group_log")
                            and record.get("control", {}).get("classification") == "succeeded"
                            and record.get("pose_goal", {}).get("classification") != "succeeded") else "FAIL"
        write_sidecar("M04", "collision_goal", "a goal inside the nearest reachable structure volume (arm_profile_link) is rejected by planning; the base body itself is out of the arms' reach",
                      status, {"volume": VOLUME, "base_plate": "bp000", "lift_joint_position_m": 0.0, "planning_budget_s": PLANNING_BUDGET_S,
                               "rejected_means": f"answered within {REJECTED_WITHIN_S} s of the free control, far under the {PLANNING_BUDGET_S} s the planner was given, "
                                                 "and move_group's log carries the planner's own verdict (Planner 'OMPL' failed with error code GOAL_STATE_INVALID). "
                                                 "The reply itself is the generic FAILURE for every outcome, so 'rejected', 'failed' (a search that gave up "
                                                 "in between) and 'timed_out' (used the budget) are told apart by time and by that log line, not by a code",
                               "base_link_volume_reachable_by_the_arms": False,         # a reach fact for A4, not a defect
                               "volume_tested_is": "the nearest structure volume the arms can reach (VERIFICATION.md M04)",
                               "base_link_evidence": "0 of 6000 left-arm samples at the baseline (lift at L1000) put a left gripper link in collision "
                                                     "with base_link, the wheels, the casters, the lidar or the camera body (find_goal search, "
                                                     "seed 4242); arm_profile_link in 249, the right arm's own links in 73 (openarm_right_base_link) "
                                                     "and fewer; M03's samples at the three lift stops found no arm link in collision with "
                                                     "base_link either",
                               **record})
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, client, request, seconds=30.0):
        self.assertTrue(client.wait_for_service(timeout_sec=60.0), f"{client.srv_name} did not appear")
        future = client.call_async(request)
        self.client.spin_until(future.done, seconds)
        self.assertTrue(future.done(), f"{client.srv_name} did not answer")
        return future.result()

    def _state(self, joints):
        state = RobotState()
        state.joint_state = JointState(name=list(joints), position=[float(v) for v in joints.values()])
        return state

    def test_m04_1_a_goal_in_the_nearest_reachable_volume_is_rejected_not_searched_for(self, proc_output):
        self.assertTrue(self.client.wait_for_server(90.0), "move_group's /move_action did not appear")
        states = read_group_states(open(srdf_path()).read())

        # 1. the fixture really is in collision with the arm profile, and only with the left gripper
        request = GetStateValidity.Request(group_name="left_arm", robot_state=self._state(FIXTURE))
        validity = self._call(self.validity, request)
        contacts = [(c.contact_body_1, c.contact_body_2) for c in validity.contacts]
        gripper_in_volume = [c for c in contacts if VOLUME in c and (GRIPPER_LINKS & set(c))]
        self.record["fixture"] = {"joints": FIXTURE, "valid": validity.valid, "contacts": [list(c) for c in contacts]}
        self.assertFalse(validity.valid, "the fixture is no longer in collision: re-run the search (the model changed)")
        self.assertTrue(gripper_in_volume, f"the fixture collides, but not a left gripper link with {VOLUME}: {contacts}")

        # 2. the control: a free named pose plans
        control = self.client.plan_through_the_service(
            self.client.request("left_arm", states[("left_arm", "hands_up")], 0.5, 0.5, PLANNING_BUDGET_S))
        baseline = control.wall_seconds
        self.record["control"] = {"goal": "named pose hands_up", **control.as_dict(), "classification": classify(control, baseline)}
        self.assertEqual(classify(control, baseline), "succeeded", f"a free goal did not plan: {control.as_dict()}")

        # 3. the JOINT goal in the volume
        joint = self.client.plan_through_the_service(self.client.request("left_arm", FIXTURE, 0.5, 0.5, PLANNING_BUDGET_S))
        self.record["joint_goal"] = {"goal": "joint values of the fixture", **joint.as_dict(), "classification": classify(joint, baseline),
                                     "seconds_beyond_the_free_goal": round(joint.wall_seconds - baseline, 3)}
        # the planner's own verdict, from move_group's log: the only place it is visible (the reply is the generic FAILURE)
        planner_verdict = "Planner 'OMPL' failed with error code GOAL_STATE_INVALID"
        self.record["joint_goal"]["planner_verdict_seen_in_move_group_log"] = bool(proc_output.waitFor(planner_verdict, timeout=5))

        # 4. the POSE goal at the fixture's gripper pose
        fk = self._call(self.fk, GetPositionFK.Request(header=__import__("std_msgs.msg", fromlist=["Header"]).Header(frame_id="base_link"),
                                                       fk_link_names=[EE_LINK], robot_state=self._state(FIXTURE)))
        pose = fk.pose_stamped[0].pose
        self.record["pose_goal_target"] = {"link": EE_LINK, "frame": fk.pose_stamped[0].header.frame_id,
                                           "position": [pose.position.x, pose.position.y, pose.position.z]}
        constraints = Constraints()
        region = BoundingVolume()
        region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005])]
        region.primitive_poses = [Pose(position=pose.position, orientation=pose.orientation)]
        constraints.position_constraints = [PositionConstraint(header=fk.pose_stamped[0].header, link_name=EE_LINK,
                                                               constraint_region=region, weight=1.0)]
        constraints.orientation_constraints = [OrientationConstraint(
            header=fk.pose_stamped[0].header, link_name=EE_LINK, orientation=pose.orientation,
            absolute_x_axis_tolerance=0.05, absolute_y_axis_tolerance=0.05, absolute_z_axis_tolerance=0.05, weight=1.0)]
        goal = self.client.request("left_arm", {}, 0.5, 0.5, PLANNING_BUDGET_S)
        goal.request.goal_constraints = [constraints]
        pose_outcome = self.client.plan_through_the_service(goal)
        self.record["pose_goal"] = {"goal": "pose of the left gripper at the fixture", **pose_outcome.as_dict(),
                                    "classification": classify(pose_outcome, baseline),
                                    "seconds_beyond_the_free_goal": round(pose_outcome.wall_seconds - baseline, 3)}

        # What M04 asserts: the colliding JOINT goal was refused, and neither colliding goal produced a plan.
        self.assertEqual(classify(joint, baseline), "rejected",
                         f"the joint goal in the arm profile's volume was {classify(joint, baseline)} ({joint.as_dict()}), not rejected")
        self.assertTrue(self.record["joint_goal"]["planner_verdict_seen_in_move_group_log"],
                        f"move_group's log has no '{planner_verdict}' for the joint goal")
        self.assertFalse(pose_outcome.success, "a pose goal inside the arm profile's volume produced a plan")
        self.assertIn(classify(pose_outcome, baseline), ("rejected", "timed_out", "failed"))


class TestProcessesAlive(unittest.TestCase):
    def test_no_process_of_the_launch_died_while_the_tests_were_running(self, proc_info):
        assert_no_process_died(proc_info)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_m04_processes_started_and_exited_cleanly(self, proc_info, proc_output):
        """move_group and its joint state publisher started, and nothing but move_group's known SIGSEGV at stop ended badly."""
        assert_launch_healthy(proc_info, expected=EXPECTED_PROCESSES, allowed=MOVE_GROUP_ALLOWED, proc_output=proc_output)
