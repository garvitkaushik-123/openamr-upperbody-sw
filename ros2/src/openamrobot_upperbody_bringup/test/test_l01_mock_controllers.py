# SPDX-License-Identifier: MIT
"""L01: mock.launch.py brings all five controllers to `active`, and a trajectory on left_arm_controller moves the left arm.

Runs headless: robot_state_publisher, ros2_control_node (mock_components/GenericSystem) and the spawners. Within 30 s of
the launch every controller must be listed `active` by /controller_manager/list_controllers. Then one short
FollowJointTrajectory goal goes to left_arm_controller and /joint_states must reach the target within TOLERANCE, with
the right arm left where it was.

Nothing physical is involved: mock hardware reflects its command back as state, so this shows the controller wiring
(joint names, interfaces, spawning order), not tracking performance.
"""
import os
import sys
import random
import time
import unittest
import xml.etree.ElementTree as ET

# Own DDS domain, localhost only: a second stack on this machine (or another test) cannot answer for ours.
os.environ["ROS_DOMAIN_ID"] = str(random.randint(1, 100))
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

import launch  # noqa: E402
import launch_testing  # noqa: E402
import launch_testing.actions  # noqa: E402
import launch_testing.asserts  # noqa: E402
import launch_testing.markers  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
import xacro  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from builtin_interfaces.msg import Duration  # noqa: E402
from control_msgs.action import FollowJointTrajectory  # noqa: E402
from controller_manager_msgs.srv import ListControllers  # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from trajectory_msgs.msg import JointTrajectoryPoint  # noqa: E402

CONTROLLERS = ("joint_state_broadcaster", "left_arm_controller", "right_arm_controller",
               "left_gripper_controller", "right_gripper_controller")
LEFT = [f"openarm_left_joint{n}" for n in range(1, 8)]
RIGHT = [f"openarm_right_joint{n}" for n in range(1, 8)]
ACTIVE_WITHIN_S = 30.0
MOVE_SECONDS = 2
TOLERANCE = 0.02          # rad


def _joint_limits():
    """{joint: (lower, upper)} of the arm joints, from the same expansion the launch file uses."""
    path = os.path.join(get_package_share_directory("openamrobot_upperbody_description"), "urdf", "combined_robot.xacro")
    root = ET.fromstring(xacro.process_file(path, mappings={"hardware_profile": "mock"}).toxml())
    return {j.get("name"): (float(j.find("limit").get("lower")), float(j.find("limit").get("upper")))
            for j in root.findall("joint") if j.get("name") in LEFT + RIGHT}


@pytest.mark.launch_test
def generate_test_description():
    mock = os.path.join(get_package_share_directory("openamrobot_upperbody_bringup"), "launch", "mock.launch.py")
    description = launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource(mock),
                                                launch_arguments={"rviz": "false"}.items()),
        launch_testing.actions.ReadyToTest(),
    ])
    return description, {}


class TestMockControllers(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("l01_test")
        cls.started = time.monotonic()
        cls.joint_states = {}
        cls.node.create_subscription(JointState, "/joint_states", cls._on_joint_state, 10)
        cls.list_controllers = cls.node.create_client(ListControllers, "/controller_manager/list_controllers")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_joint_state(cls, msg):
        cls.joint_states.update(dict(zip(msg.name, msg.position)))

    def _spin_until(self, predicate, deadline):
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return predicate()

    def _states(self):
        future = self.list_controllers.call_async(ListControllers.Request())
        self._spin_until(future.done, time.monotonic() + 2.0)
        return {c.name: c.state for c in future.result().controller} if future.done() and future.result() else {}

    def test_l01_1_all_five_controllers_active(self):
        deadline = self.started + ACTIVE_WITHIN_S
        self.assertTrue(self.list_controllers.wait_for_service(timeout_sec=ACTIVE_WITHIN_S),
                        f"/controller_manager/list_controllers did not appear within {ACTIVE_WITHIN_S:.0f} s")
        states = {}
        while time.monotonic() < deadline:
            states = self._states()
            if all(states.get(name) == "active" for name in CONTROLLERS):
                return
            time.sleep(0.25)
        self.fail(f"not all controllers active within {ACTIVE_WITHIN_S:.0f} s of the launch: {states}")

    def test_l01_2_a_trajectory_on_left_arm_controller_moves_the_left_arm(self):
        limits = _joint_limits()
        target = {name: 0.0 for name in LEFT}
        for name in ("openarm_left_joint1", "openarm_left_joint4"):
            lower, upper = limits[name]
            target[name] = min(max(0.2, lower + 0.05), upper - 0.05)

        self.assertTrue(self._spin_until(lambda: all(j in self.joint_states for j in LEFT + RIGHT),
                                         time.monotonic() + 10.0), "/joint_states never listed all 14 arm joints")
        # Issue #44: mock.launch.py moves both arms to `park` once at start-up (park_arms.py). Measure "the right arm did not move" only
        # after that move has ended, or it is that move, not this goal, that the comparison sees.
        park = _park_pose()
        self.assertTrue(self._spin_until(lambda: all(abs(self.joint_states[j] - park[j]) <= PARK_SETTLED for j in LEFT + RIGHT),
                                         time.monotonic() + PARK_WITHIN_S),
                        f"the arms did not reach park within {PARK_WITHIN_S:.0f} s of the start-up move")
        right_before = {j: self.joint_states[j] for j in RIGHT}

        client = ActionClient(self.node, FollowJointTrajectory, "/left_arm_controller/follow_joint_trajectory")
        self.assertTrue(client.wait_for_server(timeout_sec=10.0), "left_arm_controller has no action server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = LEFT
        goal.trajectory.points = [JointTrajectoryPoint(positions=[target[j] for j in LEFT], velocities=[0.0] * 7,
                                                       time_from_start=Duration(sec=MOVE_SECONDS))]
        send = client.send_goal_async(goal)
        self.assertTrue(self._spin_until(send.done, time.monotonic() + 5.0), "goal was not answered")
        handle = send.result()
        self.assertTrue(handle.accepted, "left_arm_controller rejected the goal")
        result = handle.get_result_async()
        self.assertTrue(self._spin_until(result.done, time.monotonic() + MOVE_SECONDS + 10.0), "trajectory did not finish")
        self.assertEqual(result.result().result.error_code, FollowJointTrajectory.Result.SUCCESSFUL,
                         result.result().result.error_string)

        # one more /joint_states after the goal finished, then compare
        self.joint_states.clear()
        self.assertTrue(self._spin_until(lambda: all(j in self.joint_states for j in LEFT + RIGHT),
                                         time.monotonic() + 5.0))
        for name in LEFT:
            self.assertAlmostEqual(self.joint_states[name], target[name], delta=TOLERANCE, msg=name)
        self.assertGreater(abs(self.joint_states["openarm_left_joint1"]), TOLERANCE, "the arm did not move at all")
        for name in RIGHT:
            self.assertAlmostEqual(self.joint_states[name], right_before[name], delta=TOLERANCE, msg=name)


PARK_WITHIN_S = 60.0     # the start-up move (park_arms.py) takes 3 s once the arm controllers are active
PARK_SETTLED = 1e-3      # rad: mock hardware echoes its command, so the arms end exactly on park


def _park_pose():
    """{joint: position} of `park` for both arms, from the SRDF move_group uses (the same source as the start-up move)."""
    from openamrobot_upperbody_control.lift_guard import stowed_poses_from_srdf
    from openamrobot_upperbody_moveit.moveit_config import srdf_path
    with open(srdf_path()) as f:
        poses = stowed_poses_from_srdf(f.read(), ["park"])
    return {joint: value for side in poses for joint, value in poses[side][0].items()}


from ament_index_python.packages import get_package_prefix as _prefix  # noqa: E402

sys.path.insert(0, os.path.join(_prefix("openamrobot_upperbody_description"), "lib", "openamrobot_upperbody_description"))
from launch_guard import assert_launch_healthy, assert_no_process_died  # noqa: E402

EXPECTED_PROCESSES = ["robot_state_publisher", "ros2_control_node", "spawner"]


class TestProcessesAlive(unittest.TestCase):
    def test_no_process_of_the_launch_died_while_the_tests_were_running(self, proc_info):
        assert_no_process_died(proc_info)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_l01_processes_started_and_exited_cleanly(self, proc_info):
        assert_launch_healthy(proc_info, expected=EXPECTED_PROCESSES)
