# SPDX-License-Identifier: MIT
"""The clamp in the loop, with a real move_group: it reproduces the defect deterministically instead of waiting for Gazebo to produce it.

A joint state publisher holds `openarm_left_joint4` (home's joint, on its lower limit 0) at -2.5e-7 rad, the value Gazebo read back. Without the
clamp MoveIt refuses every request from that state (`clamp:=false`, the baseline; the reply is the generic FAILURE, and move_group's log says
"Joint 'openarm_left_joint4' from the starting state is outside bounds by: [-2.5e-07]"); with it the same request plans, the
counters say what happened, and a reading further out than the epsilon is neither clamped nor hidden: it is still refused, and counted as
over_epsilon. `launch_test` runs this module twice (CMake passes `clamp:=true` and `clamp:=false`)."""
import json
import os
import random
import sys
import unittest

os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
os.environ["ROS_DOMAIN_ID"] = str(random.randint(1, 100))

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from openamrobot_upperbody_moveit import joint_state_clamp as jsc  # noqa: E402


def _clamp_argument(default="true"):
    for argument in sys.argv[1:]:
        if argument.startswith("clamp:="):
            return argument.split(":=", 1)[1]
    return default


CLAMP = _clamp_argument()

import launch  # noqa: E402
import pytest  # noqa: E402
import launch_testing.actions  # noqa: E402
import rclpy  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from openamrobot_upperbody_moveit.move_group_client import MoveGroupClient  # noqa: E402
from openamrobot_upperbody_moveit.named_poses import ARM_GROUP, read_group_states  # noqa: E402
from openamrobot_upperbody_moveit.moveit_config import srdf_path  # noqa: E402

from ament_index_python.packages import get_package_prefix as _prefix  # noqa: E402

sys.path.insert(0, os.path.join(_prefix("openamrobot_upperbody_description"), "lib", "openamrobot_upperbody_description"))
from launch_guard import MOVE_GROUP_ALLOWED, SIM_ALLOWED, assert_launch_healthy, assert_no_process_died  # noqa: E402

EXPECTED_PROCESSES = ["move_group"] + (["joint_state_clamp"] if CLAMP == "true" else [])


JOINT = "openarm_left_joint4"
NOISE = -2.5e-7             # what Gazebo read back for a joint on its lower limit


@pytest.mark.launch_test
def generate_test_description():
    move_group = os.path.join(get_package_share_directory("openamrobot_upperbody_moveit"), "launch", "move_group.launch.py")
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(move_group),
            launch_arguments={"hardware_profile": "mock", "rviz": "false", "clamp_joint_states": CLAMP}.items()),
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestClampInTheLoop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import xacro
        import xml.etree.ElementTree as ET
        from openamrobot_upperbody_moveit.moveit_config import combined_xacro
        rclpy.init()
        cls.node = rclpy.create_node("clamp_test")
        cls.client = MoveGroupClient(cls.node)
        root = ET.fromstring(xacro.process_file(combined_xacro(), mappings={"hardware_profile": "mock"}).toxml())
        cls.joints = [j.get("name") for j in root.findall("joint") if j.get("type") != "fixed"]
        cls.value = NOISE
        cls.publisher = cls.node.create_publisher(JointState, "/joint_states", 10)
        cls.timer = cls.node.create_timer(0.05, cls.publish)
        cls.clamped_seen = []
        cls.node.create_subscription(JointState, "/joint_states_clamped",
                                     lambda m: cls.clamped_seen.append(dict(zip(m.name, m.position)).get(JOINT)), 10)

    @classmethod
    def publish(cls):
        msg = JointState(name=cls.joints, position=[cls.value if j == JOINT else 0.0 for j in cls.joints])
        msg.header.stamp = cls.node.get_clock().now().to_msg()
        cls.publisher.publish(msg)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def hands_up(self):
        return read_group_states(open(srdf_path()).read())[(ARM_GROUP["left"], "hands_up")]

    def plan(self):
        """Plan only, through /plan_kinematic_path: the service answers with the pipeline's own error code, where the MoveGroup action
        collapses a refused start state into the generic FAILURE (99999)."""
        self.assertTrue(self.client.wait_for_server(60.0), "move_group did not come up")
        self.client.spin_until(lambda: False, 2.0)                 # let move_group see a few joint states
        goal = self.client.request(ARM_GROUP["left"], self.hands_up(), 0.5, 0.5, 5.0)
        return self.client.plan_through_the_service(goal)

    def stats(self):
        client = self.node.create_client(Trigger, "/joint_state_clamp/get_stats")
        self.assertTrue(client.wait_for_service(timeout_sec=20.0), "no /joint_state_clamp/get_stats")
        future = client.call_async(Trigger.Request())
        self.client.spin_until(future.done, 20.0)
        return json.loads(future.result().message)

    @unittest.skipUnless(CLAMP == "false", "baseline: without the clamp (tracking: openAMRobot/openamr-upperbody-sw#16)")
    def test_without_the_clamp_a_start_state_2p5e_minus_7_past_the_limit_is_refused(self, proc_output):
        outcome = self.plan()
        self.assertFalse(outcome.success, "the defect this clamp exists for no longer reproduces: the request planned")
        # MoveIt 2.12.4 answers the generic FAILURE (99999) here, to the action and to the service alike; the refusal is named only in move_group's log
        self.assertTrue(proc_output.waitFor("from the starting state is outside bounds by", timeout=5), "move_group's log does not name the refusal")

    @unittest.skipUnless(CLAMP == "true", "the clamp is on (tracking: openAMRobot/openamr-upperbody-sw#16)")
    def test_with_the_clamp_the_same_request_plans_and_the_counters_say_what_happened(self):
        outcome = self.plan()
        self.assertTrue(outcome.success, outcome.name)
        self.client.spin_until(lambda: len(self.clamped_seen) > 5, 10.0)
        self.assertTrue(self.clamped_seen and all(v == 0.0 for v in self.clamped_seen), self.clamped_seen[:5])
        self.assertEqual(self.client.joint_states[JOINT], NOISE, "the raw /joint_states is not touched")
        s = self.stats()
        self.assertGreater(s["activations"], 5)
        self.assertEqual((s["episodes"], s["over_epsilon"]), (1, 0))
        self.assertAlmostEqual(s["max_magnitude_rad"], -NOISE, delta=1e-15)
        self.assertEqual(s["max_magnitude_joint"], JOINT)
        self.assertEqual(s["epsilon_rad"], jsc.EPSILON)

    @unittest.skipUnless(CLAMP == "true", "the clamp is on (tracking: openAMRobot/openamr-upperbody-sw#16)")
    def test_zz_a_reading_beyond_the_epsilon_is_not_hidden(self, proc_output):
        # The counts are compared within snapshots of the clamp's own state, never across the switch. The publisher keeps sending the in-epsilon
        # noise (-2.5e-7) until the new value takes effect, so a reading read BEFORE the switch (the old `before`) is not a baseline: a noise
        # message still in flight is a legitimate clamp that lands after it (P28 run 2: 42 != 41). The clamp sees one publisher's messages in
        # order, so once the first beyond-epsilon reading has been counted every earlier noise message has been counted too, and from that
        # snapshot on every reading is beyond the epsilon: no further activation may happen.
        type(self).value = -1e-3
        first = {}

        def first_over_epsilon_seen():
            first.update(self.stats())
            return first["over_epsilon"] > 0

        self.client.spin_until(first_over_epsilon_seen, 15.0)
        self.assertGreater(first.get("over_epsilon", 0), 0, "no reading beyond the epsilon was ever counted")
        outcome = self.plan()
        self.assertFalse(outcome.success, "a joint 1e-3 rad past its limit must still be refused")
        self.assertTrue(proc_output.waitFor("from the starting state is outside bounds by", timeout=5), "move_group's log does not name the refusal")
        s = self.stats()
        self.assertGreater(s["over_epsilon"], 0)
        self.assertAlmostEqual(s["max_over_epsilon_rad"], 1e-3, delta=1e-12)
        self.assertEqual(s["activations"], first["activations"], "a reading beyond the epsilon was counted as a clamp after the first one was seen")
        # one snapshot, no baseline: a 1e-3 reading counted as a clamp (the first one included) would put 1e-3 here, the clamped ones are 2.5e-7
        self.assertAlmostEqual(s["max_magnitude_rad"], -NOISE, delta=1e-15, msg="nothing beyond the epsilon is counted as a clamp")


class TestProcessesAlive(unittest.TestCase):
    def test_no_process_of_the_launch_died_while_the_tests_were_running(self, proc_info):
        assert_no_process_died(proc_info)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_clamp_processes_started_and_exited_cleanly(self, proc_info, proc_output):
        """move_group (and, with the clamp, joint_state_clamp.py) started, and nothing but move_group's known SIGSEGV at stop ended badly."""
        assert_launch_healthy(proc_info, expected=EXPECTED_PROCESSES, allowed=MOVE_GROUP_ALLOWED, proc_output=proc_output)
