# SPDX-License-Identifier: MIT
"""The software-in-the-loop launch, end to end and headless: Gazebo, both emulators, the real driver, one trajectory, one E-stop.

    sil.launch.py headless:=true create_vcan:=false example_trajectory:=false

What it asserts, in order, against one running stack

  1  both controller managers are up: /sim/controller_manager (inside the simulator) with joint_state_broadcaster,
     lift_controller and the two emulators active, and /controller_manager (the real driver's) with joint_state_broadcaster,
     both arm controllers and both gripper controllers active. All sixteen emulated motors are enabled.
  2  known issue, recorded: the first MIT frame each arm joint got after enable was a step to zero at full gains, and the
     gripper got 0.044 rad (upstream OpenArmHW return_to_zero).
  3  a FollowJointTrajectory goal on the real left_arm_controller completes. The driver's /joint_states and the physics'
     /sim/joint_states agree, so the command went JointTrajectoryController -> OpenArmHW -> vcan -> emulator -> Gazebo and
     the state came back the same way.
  4  E-stop power cut on the left arm: the left arm falls in the physics while the driver's /joint_states stays frozen, the
     hardware component stays `active` and every controller stays active (known issue, recorded). The right arm is unaffected.
  5  reset: the driver's state is live again and shows the fallen arm; the motors are disabled and stay disabled.

BLOCKED (reported as skipped, never as passed) when vcan_arm_l or vcan_arm_r does not exist or the real driver is not
installed. Simulation evidence only: nothing here says anything about a physical arm.
"""
import os
import random
import time
import unittest

os.environ["ROS_DOMAIN_ID"] = str(random.randint(101, 200))
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
os.environ["GZ_PARTITION"] = f"openarm_sil_{os.getpid()}"

import launch  # noqa: E402
import launch_testing  # noqa: E402
import launch_testing.actions  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory  # noqa: E402
from builtin_interfaces.msg import Duration  # noqa: E402
from control_msgs.action import FollowJointTrajectory  # noqa: E402
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: E402
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue  # noqa: E402
from rcl_interfaces.srv import SetParameters  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from trajectory_msgs.msg import JointTrajectoryPoint  # noqa: E402

INTERFACES = ("vcan_arm_l", "vcan_arm_r")
SYS_NET = os.environ.get("OPENARM_EMULATOR_SYS_NET", "/sys/class/net")
SIM_CONTROLLERS = ("joint_state_broadcaster", "lift_controller", "left_arm_emulator", "right_arm_emulator")
REAL_CONTROLLERS = ("joint_state_broadcaster", "left_arm_controller", "right_arm_controller",
                    "left_gripper_controller", "right_gripper_controller")
LEFT = [f"openarm_left_joint{n}" for n in range(1, 8)]
RIGHT = [f"openarm_right_joint{n}" for n in range(1, 8)]
UP_WITHIN_S = 180.0
TARGET = {"openarm_left_joint1": 0.3, "openarm_left_joint4": 0.6}
TRAJECTORY_SECONDS = 3.0
TARGET_TOLERANCE = 0.15     # rad. The MIT position gain alone holds the arm against gravity, so it sags
SAME_TOLERANCE = 0.03       # rad. Driver state against physics, when the motors answer
SWINGING_TOLERANCE = 0.2    # rad. Driver state against physics while a limp arm swings
FELL_AT_LEAST = 0.10        # rad. How far the physics must leave the frozen driver state during the power cut


def blocked_reason():
    missing = [name for name in INTERFACES if not os.path.exists(os.path.join(SYS_NET, name))]
    if missing:
        return f"no {' or '.join(missing)} interface: scripts/setup_vcan.sh creates them and needs the vcan kernel module"
    try:
        get_package_prefix("openarm_hardware")
    except PackageNotFoundError:
        return "the real driver openarm_hardware is not installed (build openarm_ros2 and openarm_can at the pins)"
    return ""


BLOCKED = blocked_reason()


@pytest.mark.launch_test
def generate_test_description():
    actions = []
    if not BLOCKED:
        sil = os.path.join(get_package_share_directory("openamrobot_openarm_emulator"), "launch", "sil.launch.py")
        actions.append(launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sil),
            launch_arguments={"headless": "true", "create_vcan": "false", "example_trajectory": "false"}.items()))
    else:
        # Nothing to launch. One idle process keeps the launch alive until the tests have reported BLOCKED.
        actions.append(launch.actions.ExecuteProcess(cmd=["sleep", "60"], output="screen"))
    actions.append(launch_testing.actions.ReadyToTest())
    return launch.LaunchDescription(actions), {}


class TestSilLaunch(unittest.TestCase):
    record = {}

    @classmethod
    def setUpClass(cls):
        if BLOCKED:
            return
        rclpy.init()
        cls.node = rclpy.create_node("openarm_sil_test")
        cls.driver, cls.physics, cls.motors, cls.buses = {}, {}, {}, {}
        cls.node.create_subscription(JointState, "/joint_states", lambda m: cls.driver.update(zip(m.name, m.position)), 10)
        cls.node.create_subscription(JointState, "/sim/joint_states", lambda m: cls.physics.update(zip(m.name, m.position)), 10)
        for side in ("left", "right"):
            cls.node.create_subscription(DiagnosticArray, f"/sim/{side}_arm_emulator/status", cls._on_status, 10)

    @classmethod
    def _on_status(cls, message):
        for status in message.status:
            values = {pair.key: pair.value for pair in status.values}
            values["message"] = status.message
            parts = status.name.split("/")
            (cls.motors if len(parts) == 3 else cls.buses)[parts[-1]] = values

    @classmethod
    def tearDownClass(cls):
        if BLOCKED:
            return
        print("openarm_sil record:", cls.record)
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        if BLOCKED:
            self.skipTest(f"BLOCKED: {BLOCKED}")

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def call(self, service_type, name, request, timeout=20.0):
        client = self.node.create_client(service_type, name)
        self.assertTrue(client.wait_for_service(timeout_sec=timeout), f"{name} is not available")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertIsNotNone(future.result(), f"{name} did not answer")
        return future.result()

    def active(self, manager):
        client = self.node.create_client(ListControllers, f"{manager}/list_controllers")
        if not client.wait_for_service(timeout_sec=1.0):
            return set()
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        return {c.name for c in future.result().controller if c.state == "active"} if future.result() else set()

    def set_power_cut(self, side, cut):
        value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=cut)
        result = self.call(SetParameters, f"/sim/{side}_arm_emulator/set_parameters",
                           SetParameters.Request(parameters=[Parameter(name="fault.estop_power_cut", value=value)]))
        self.assertTrue(result.results[0].successful, result.results[0].reason)

    def test_1_both_controller_managers_are_up_and_every_motor_is_enabled(self):
        start = time.monotonic()
        sim, real = set(), set()
        while time.monotonic() - start < UP_WITHIN_S:
            sim, real = self.active("/sim/controller_manager"), self.active("/controller_manager")
            if sim >= set(SIM_CONTROLLERS) and real >= set(REAL_CONTROLLERS):
                break
            self.spin(1.0)
        self.assertGreaterEqual(sim, set(SIM_CONTROLLERS), f"active in the simulator after {UP_WITHIN_S} s: {sorted(sim)}")
        self.assertGreaterEqual(real, set(REAL_CONTROLLERS), f"active on the real driver after {UP_WITHIN_S} s: {sorted(real)}")
        self.record["up_after_s"] = round(time.monotonic() - start, 1)
        self.spin(2.0)
        self.assertEqual(len(self.motors), 16, sorted(self.motors))
        for joint, status in self.motors.items():
            self.assertEqual(status["message"], "enabled", f"{joint}: {status}")
        self.assertEqual(set(self.buses), set(INTERFACES))
        self.record["can_fd"] = {name: status["can_fd"] for name, status in self.buses.items()}
        for name, status in self.buses.items():
            self.assertEqual(status["rx_unknown"], "0", f"{name}: {status}")
            self.assertEqual(status["tx_errors"], "0", f"{name}: {status}")
            # The driver sends CAN FD frames exactly when the interface carries them.
            self.assertEqual(int(status["rx_fd"]) > 0, status["can_fd"] == "true", f"{name}: {status}")
            self.assertEqual(int(status["rx_classic"]) > 0, status["can_fd"] != "true", f"{name}: {status}")
        for joint in LEFT + RIGHT:
            self.assertIn(joint, self.driver)
            self.assertIn(joint, self.physics)

    def test_2_known_issue_activation_commanded_a_step_to_zero(self):
        self.record["first_mit_q"] = {joint: self.motors[joint]["first_mit_q"] for joint in sorted(self.motors)}
        self.record["first_mit_position"] = {joint: self.motors[joint]["first_mit_position"] for joint in sorted(self.motors)}
        for joint in LEFT + RIGHT:
            self.assertAlmostEqual(float(self.motors[joint]["first_mit_q"]), 0.0, delta=1e-3, msg=joint)
        self.assertAlmostEqual(float(self.motors["openarm_left_joint1"]["mit_kp"]), 70.0, delta=0.2)
        self.assertAlmostEqual(float(self.motors["openarm_left_joint4"]["mit_kp"]), 60.0, delta=0.2)
        for side in ("left", "right"):
            self.assertAlmostEqual(float(self.motors[f"openarm_{side}_finger_joint1"]["first_mit_q"]), 0.044, delta=1e-3)

    def test_3_a_trajectory_on_the_real_controller_moves_the_simulated_arm(self):
        client = ActionClient(self.node, FollowJointTrajectory, "/left_arm_controller/follow_joint_trajectory")
        self.assertTrue(client.wait_for_server(timeout_sec=20.0))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = LEFT
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=[TARGET.get(joint, 0.0) for joint in LEFT], velocities=[0.0] * 7,
            time_from_start=Duration(sec=int(TRAJECTORY_SECONDS)))]
        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=10.0)
        self.assertTrue(send.result() is not None and send.result().accepted)
        result = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, result, timeout_sec=TRAJECTORY_SECONDS + 20.0)
        self.assertIsNotNone(result.result())
        self.assertEqual(result.result().result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)
        self.spin(1.5)
        for joint, target in TARGET.items():
            self.assertAlmostEqual(self.physics[joint], target, delta=TARGET_TOLERANCE, msg=f"physics {joint}")
            self.assertAlmostEqual(self.driver[joint], self.physics[joint], delta=SAME_TOLERANCE, msg=f"driver against physics {joint}")
        self.record["after_trajectory"] = {joint: {"target": target, "physics": round(self.physics[joint], 4),
                                                   "driver": round(self.driver[joint], 4)} for joint, target in TARGET.items()}
        self.record["right_arm_physics"] = {joint: round(self.physics[joint], 4) for joint in RIGHT}
        for joint in RIGHT:
            self.assertAlmostEqual(self.physics[joint], 0.0, delta=TARGET_TOLERANCE, msg=joint)

    def test_4_known_issue_power_cut_the_arm_falls_and_the_driver_reports_a_healthy_frozen_arm(self):
        self.set_power_cut("left", True)
        self.spin(0.5)
        frozen = {joint: self.driver[joint] for joint in LEFT}
        self.spin(2.5)
        for joint in LEFT:
            self.assertEqual(self.driver[joint], frozen[joint], f"{joint} changed in the driver's state during the power cut")
        fell = {joint: abs(self.physics[joint] - self.driver[joint]) for joint in LEFT}
        self.record["power_cut_physics_minus_driver"] = {joint: round(value, 4) for joint, value in fell.items()}
        self.assertGreater(max(fell.values()), FELL_AT_LEAST, f"no left joint fell: {fell}")
        # ros2_control is told nothing: the hardware components are active and every controller is active.
        components = self.call(ListHardwareComponents, "/controller_manager/list_hardware_components", ListHardwareComponents.Request())
        states = {component.name: component.state.label for component in components.component}
        self.assertEqual(states, {"openarm_left_hardware_interface": "active", "openarm_right_hardware_interface": "active"})
        self.assertGreaterEqual(self.active("/controller_manager"), set(REAL_CONTROLLERS))
        for joint in LEFT:
            self.assertEqual(self.motors[joint]["message"], "no power", joint)
        # The other arm is on its own bus and runs on.
        for joint in RIGHT:
            self.assertEqual(self.motors[joint]["message"], "enabled", joint)
            self.assertAlmostEqual(self.driver[joint], self.physics[joint], delta=SAME_TOLERANCE, msg=joint)

    def test_5_known_issue_after_reset_the_motors_stay_disabled(self):
        enables = {joint: self.motors[joint]["enable_frames"] for joint in LEFT}
        self.set_power_cut("left", False)
        self.spin(2.0)
        self.record["after_reset_driver_minus_physics"] = {joint: round(self.driver[joint] - self.physics[joint], 4) for joint in LEFT}
        for joint in LEFT:
            # Live again. The limp arm may still swing, and the two topics are not sampled together, so the band is wide.
            self.assertAlmostEqual(self.driver[joint], self.physics[joint], delta=SWINGING_TOLERANCE, msg=f"live again: {joint}")
            self.assertEqual(self.motors[joint]["message"], "disabled", joint)
            self.assertEqual(self.motors[joint]["enable_frames"], enables[joint], f"{joint}: the driver sent enable by itself")
        # left_arm_controller still commands the trajectory's end point. Nothing follows it.
        self.assertGreater(max(abs(self.physics[joint] - target) for joint, target in TARGET.items()), FELL_AT_LEAST)
        self.assertGreaterEqual(self.active("/controller_manager"), set(REAL_CONTROLLERS))
