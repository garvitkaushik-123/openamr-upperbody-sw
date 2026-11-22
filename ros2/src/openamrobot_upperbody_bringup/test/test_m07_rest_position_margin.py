# SPDX-License-Identifier: MIT
"""M07: at rest in Gazebo, no joint sits within a margin of either of its limits (P17 task 2).

ALEX'S RULE, quoted from the comment posted on PR #14 by BotshareAI on 20 Oct 2026 ("Proposed M07 rest-state rule (Alex decision, 16 Oct 2026). Om, please
confirm or comment."; once Om confirms it goes into the decisions register as M07-REST-STATE, and it is a proposal until then):

  1. Scope: derive the expected set from the combined model: all 16 limited arm and gripper joints under ros2_control. lift_joint and the mimic
     finger_joint2 joints are recorded as not independently measured and are excluded from the measured set.
  2. Evidence: use raw /arm/joint_states samples during the six-second simulated-time measurement window, after the six-second settle period. Each
     expected joint must have at least 50 samples. A missing joint, fewer than 50 samples, or any non-finite position value (NaN or infinity) is FAIL;
     invalid values must not count as valid samples.
  3. Margin: each measured joint must remain at least 0.001 rad inside both URDF limits throughout the window. The margin and clamp epsilon are unchanged.
  4. Exception: only openarm_{left,right}_finger_joint1 and openarm_{left,right}_joint4 may receive WARN when their measured positions violate the 0.001
     rad margin at their inherited upstream limit. A reading more than 0.001 rad beyond either physical limit is FAIL, including for these four joints.
     All other joints that violate the margin are FAIL.
  5. WARN is a recorded limitation, not acceptance. It remains until the upstream limits or rest pose are corrected and the real arms' rest state is
     checked on hardware during integration.
  6. Evidence handling: retain invalid and incomplete measurements in the sidecar and report them as FAIL; they must never be converted into PASS or WARN.
  ("Tushar, point 2 covers Mohamed's open finding on non-finite values.")
How this file keeps it: the expected set is expected_rest_joints (16, with lift_joint and the mimic finger_joint2s in the sidecar's `not_measured`); the margin is the
datum `joint_rest_margin_rad` read from datums.yaml, never a literal here; non-finite raw positions are rejected BEFORE the coverage count and before any extremum,
never count toward the 50 samples, are kept in the sidecar (`invalid_readings`), and any one of them is a FAIL. The decision is `a7_support.evaluate_m07`, a pure
function with its own regression (test_m07_non_finite.py).

Why this is a check of its own. In option B's body the right finger rests ON its open limit (-4.7e-6 .. +2.7e-8 rad); the old body rested 6.0e-5
rad inside it. A joint that rests on a limit straddles it: solver noise carries it across, the joint-state clamp fires thousands of times a run
(the count scatters 2-3x on an unchanged tree, which is why that count no longer gates anything), and every downstream number moves with it.
That is a modelling defect whether or not anything notices. A version of this check that passes on the right finger is wrong: the joints in
AT_UPSTREAM_LIMIT, whose rest pose is an upstream limit by construction, are WARN (recorded with the reason), never PASS; any other joint
within the margin, or one of those read further past its limit than the margin, is FAIL.

What it measures. Nothing is commanded: the simulator is started (headless, Bullet-Featherstone like G02 and M02, so the fingers behave), the
five controllers are waited for until active, the model settles, and the RAW joint states the physics produced (`/arm/joint_states`, the
joint_state_broadcaster's own topic: not the clamped copy move_group sees, not joint_state_publisher's merge) are read for a window of
simulated time. For every joint with limits the result keeps the lowest and highest reading and the distance of each from the nearer limit
(negative = past it). A joint FAILs when either extreme is closer than the margin.

The margin is a datum, `joint_rest_margin_rad` in datums.yaml, with its source and status there: not a literal here. The test reads it from the
installed description package and records the value it used.

What it does not cover, said so in the sidecar and never silently skipped: `lift_joint` (prismatic, in m; it rests on its
lower stop L1000 by construction, held there by lift_controller since issue #17, and the rad margin datum is not a margin for it) and the wheels
(continuous, no limits).

`simulation` class: how this model rests in this simulator on this machine. It says nothing about a physical robot or its limits.
"""
import os
import random
import sys
import time
import unittest

os.environ["ROS_DOMAIN_ID"] = str(random.randint(101, 200))
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
os.environ["GZ_PARTITION"] = f"m07_{os.getpid()}"

import launch  # noqa: E402
import launch_testing.actions  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
import yaml  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from a7_support import (A7_ENABLED, AT_UPSTREAM_LIMIT, SKIP_REASON, classify, evaluate_m07, expected_rest_joints, rest_rows,  # noqa: E402,F401
                        stack_alive, stack_healthy, wait_for_controllers_active, write_run_sidecar)

from openamrobot_upperbody_moveit.named_poses import urdf_joint_limits  # noqa: E402

ENGINE = "gz-physics-bullet-featherstone-plugin"       # passed explicitly, as G02 and M02 do: DART does not move the fingers
CONTROLLERS = ("joint_state_broadcaster", "left_arm_controller", "right_arm_controller", "left_gripper_controller", "right_gripper_controller")
ACTIVE_WITHIN_S = 120.0       # wall s for the controllers (startup of a headless server is about 10-25 s; M02 allows 150 for the whole stack)
SETTLE_SIM_S = 6.0            # simulated s between "controllers active" and the first reading: the spawn drop and the controllers' first hold
WINDOW_SIM_S = 6.0            # simulated s of readings the extremes are taken over
MIN_MESSAGES = 50             # fewer readings than this is not a measurement of a rest (the window did not run)
MIN_SAMPLES_PER_JOINT = 50    # the same, per expected joint: a joint read fewer times than this in the window was not measured (PR #14 review)
DATUM = "joint_rest_margin_rad"
EXPECTED_PROCESSES = ["gazebo", "robot_state_publisher", "parameter_bridge", "create", "joint_state_publisher", "spawner"]
NOT_MEASURED = {
    "lift_joint": "a prismatic joint (m) whose rest is its lower stop L1000 by construction: the stroke's end and a calibration stop, where "
                  "lift_controller holds it (under ros2_control in the simulated profiles since issue #17). The margin datum is in rad and is "
                  "for the revolute joints, so there is no rest margin of the lift to compare with it",
}


def margin_datum():
    """(value, the datum's own record) from the installed datums.yaml. A missing or non-positive margin is an error of the check, never a pass."""
    share = get_package_share_directory("openamrobot_upperbody_description")
    with open(os.path.join(share, "config", "datums.yaml")) as f:
        doc = yaml.safe_load(f)
    record = doc["datums"][DATUM]
    value = record["value"]
    assert isinstance(value, (int, float)) and value > 0, f"datum {DATUM} must be a positive number, got {value!r}"
    return float(value), {k: record.get(k) for k in ("value", "source", "status", "ref", "owner")}, doc["revision"]


def model_xml():
    import xacro
    path = os.path.join(get_package_share_directory("openamrobot_upperbody_description"), "urdf", "combined_robot.xacro")
    return xacro.process_file(path, mappings={"hardware_profile": "gazebo"}).toxml()


def joint_limits_with_names(xml=None):
    return urdf_joint_limits(xml if xml is not None else model_xml())


@pytest.mark.launch_test
def generate_test_description():
    if not A7_ENABLED:
        return launch.LaunchDescription([launch.actions.ExecuteProcess(cmd=["sleep", "30"]), launch_testing.actions.ReadyToTest()]), {}
    sim = os.path.join(get_package_share_directory("openamrobot_upperbody_gazebo"), "launch", "sim.launch.py")
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource(sim),
                                                launch_arguments={"headless": "true", "physics_engine": ENGINE}.items()),
        launch_testing.actions.ReadyToTest(),
    ]), {}


@unittest.skipUnless(A7_ENABLED, SKIP_REASON)  # tracking: openAMRobot/openamr-upperbody-sw#16
class TestRestPositionMargin(unittest.TestCase):
    record = {}

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("m07_test", parameter_overrides=[Parameter("use_sim_time", value=True)])
        cls.started = time.monotonic()

    @classmethod
    def tearDownClass(cls):
        # The verdict is evaluate_m07's, recorded by the test; a test that died before it (no controllers, no clock, no window) is a FAIL.
        verdict = cls.record.get("verdict")
        result = "FAIL" if cls.record.get("error") or verdict is None else verdict
        details = {"physics_engine_used": ENGINE, "physics_engine_passed_explicitly": True, "reading_topic": "/arm/joint_states",
                   "window_sim_s": WINDOW_SIM_S, "settle_sim_s": SETTLE_SIM_S, "failing_joints": [], "warned_joints": {},
                   "not_measured": NOT_MEASURED, **cls.record}
        if verdict is None and not cls.record.get("error"):
            details["problems"] = ["the measurement did not complete (FAIL)"]
        write_run_sidecar("M07", "rest_position_margin", "no joint rests within the margin of either of its limits", result, details)
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, wall_seconds):
        deadline = time.monotonic() + wall_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return predicate()

    def _wait_for_controllers(self):
        """FAIL (not hang, not skip) if the five controllers are not all active within ACTIVE_WITHIN_S: the shared wait, one absolute deadline."""
        ok, states, waited = wait_for_controllers_active(self.node, timeout_s=ACTIVE_WITHIN_S, names=CONTROLLERS)
        if not ok:
            self.fail(f"controllers not all active within {ACTIVE_WITHIN_S:.0f} s (waited {waited} s): {states}")
        return states

    def test_m07_1_no_joint_rests_within_the_margin_of_a_limit(self):
        margin, record, revision = margin_datum()
        xml = model_xml()
        limits = joint_limits_with_names(xml)
        expected, also_not_measured = expected_rest_joints(xml, NOT_MEASURED)
        self.record.update({"margin_rad": margin, "margin_datum": record, "datums_revision": revision, "expected_joints": expected,
                            "not_measured_from_the_model": also_not_measured, "min_samples_per_joint": MIN_SAMPLES_PER_JOINT})
        try:
            self.record["controllers"] = self._wait_for_controllers()
        except AssertionError as error:
            self.record["error"] = str(error)
            raise
        readings, count = {}, [0]
        window = {"on": False}

        def on_state(msg):
            if not window["on"]:
                return
            count[0] += 1
            for name, position in zip(msg.name, msg.position):
                readings.setdefault(name, []).append(position)

        self.node.create_subscription(JointState, "/arm/joint_states", on_state, QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        clock = self.node.get_clock()
        # simulated time: the clock only moves once /clock arrives, so wait for it to start
        self.assertTrue(self._spin_until(lambda: clock.now().nanoseconds > 0, 30.0), "no /clock: the simulated time never started")
        t0 = clock.now().nanoseconds
        self.assertTrue(self._spin_until(lambda: (clock.now().nanoseconds - t0) * 1e-9 >= SETTLE_SIM_S, 120.0), "the settle period did not elapse")
        window["on"] = True
        t1 = clock.now().nanoseconds
        self.assertTrue(self._spin_until(lambda: (clock.now().nanoseconds - t1) * 1e-9 >= WINDOW_SIM_S, 120.0), "the reading window did not elapse")
        window["on"] = False
        self.record["messages_in_window"] = count[0]
        evaluation = evaluate_m07(readings, expected, limits, margin, MIN_SAMPLES_PER_JOINT, not_measured=NOT_MEASURED)
        self.record.update({"verdict": evaluation["result"], "joints": evaluation["rows"], "joints_read": sorted(readings),
                            "joints_with_limits_read": len(evaluation["rows"]), "failing_joints": evaluation["failing_joints"],
                            "warned_joints": evaluation["warned_joints"], "coverage": evaluation["coverage"],
                            "samples_per_joint": evaluation["valid_samples_per_joint"],       # FINITE readings: what counts toward the minimum
                            "raw_samples_per_joint": evaluation["raw_samples_per_joint"],
                            "invalid_readings": evaluation["invalid_readings"],               # every rejected reading, by joint, kind and position
                            "not_measured_excluded_from_margin": evaluation["not_measured_excluded_from_margin"],
                            "problems": evaluation["problems"]})
        self.assertGreaterEqual(count[0], MIN_MESSAGES, f"only {count[0]} joint-state messages in a {WINDOW_SIM_S} s window: not a measurement of a rest")
        self.assertFalse(evaluation["invalid_readings"], f"non-finite joint positions on /arm/joint_states: {evaluation['invalid_readings']}")
        self.assertTrue(evaluation["rows"], f"no joint of /arm/joint_states has a finite reading and limits in the URDF (read: {sorted(readings)})")
        self.assertFalse(evaluation["coverage"]["missing"] or evaluation["coverage"]["too_few_samples"],
                         f"incomplete coverage of the expected joints by finite readings: {evaluation['coverage']}")
        failing = [r for r in evaluation["rows"] if r["joint"] in evaluation["failing_joints"]]
        self.assertFalse(failing, "joints resting within %.3g rad of a limit: %s" % (margin, "; ".join(
            f"{r['joint']} min {r['min_rad']:+.3e} max {r['max_rad']:+.3e} vs [{r['lower_limit_rad']:+.4f}, {r['upper_limit_rad']:+.4f}] "
            f"(nearest {r['nearest_limit']}, {r['distance_to_nearest_limit_rad']:+.3e})" for r in failing)))


@unittest.skipUnless(A7_ENABLED, SKIP_REASON)  # tracking: openAMRobot/openamr-upperbody-sw#16
class TestProcessesAlive(unittest.TestCase):
    def test_no_process_of_the_launch_died_while_the_tests_were_running(self, proc_info):
        stack_alive(proc_info)


@launch_testing.post_shutdown_test()
@unittest.skipUnless(A7_ENABLED, SKIP_REASON)  # tracking: openAMRobot/openamr-upperbody-sw#16
class TestShutdown(unittest.TestCase):
    def test_m07_processes_started_and_exited_cleanly(self, proc_info, proc_output):
        stack_healthy(proc_info, expected=EXPECTED_PROCESSES, simulator=True, sidecar=("M07", "rest_position_margin"), proc_output=proc_output)
