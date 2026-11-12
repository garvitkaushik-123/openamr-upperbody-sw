# SPDX-License-Identifier: MIT
"""G01: the combined robot in Gazebo, headless. Five arm controllers active, an arm goal completes, and nothing of Nav2's moved.

    sim.launch.py headless:=true    ->    gz sim -s, robot_state_publisher, ros_gz bridges, spawners, joint_state_publisher

What it asserts, in order, against one running simulation (budget 90 s for everything, startup included)

  1  all five controllers (joint_state_broadcaster, left/right arm, left/right gripper) are `active`;
  2  a FollowJointTrajectory goal on left_arm_controller completes SUCCESSFUL, the arm tracks it in /joint_states, and
     neither the right arm nor the base moves while it does;
  3  the Nav2-facing topics are exactly platform-sw's: every entry of openamrobot_gazebo/config/gz_bridge.yaml exists under
     its name with its type. This is the ADR-003 / ADR-005 regression guard (our job was to add arms and move nothing of
     platform-sw's), and it reads platform-sw's own file, so it follows that file instead of a copy of it;
  4  /cmd_vel still moves the base and /odom still reports it;
  5  exactly one publisher of odom -> <the DiffDrive plugin's child frame>: base_footprint since platform-sw 935efc7 (REP 105: the
     odom child is the URDF root so that base_link keeps one parent; the test reads the frame from the plugin in the description,
     as it reads the bridge topics from platform-sw's file, so it follows platform-sw's decision and not a copy of it). Its
     stamps are strictly increasing at the DiffDrive plugin's 50 Hz (a second
     publisher would double the rate or repeat stamps, which looks like jitter and not like an error), `odom` is not a link
     of the description, and robot_state_publisher publishes nothing with `odom` in it (check W02 made visible);
  6  /joint_states has one publisher, and carries the two wheel joints, the sixteen arm joints and lift_joint, each once: a joint is
     reported by one source only. lift_joint (issue #17, owner decision 1 Nov 2026) has a gz_ros2_control interface in this
     profile, so joint_state_broadcaster reports it on /arm/joint_states, exactly once, and the base source never does. On the real
     profile it has no interface and only the merger's default reports it (sim_test_support.lift_source_problems).

Simulation evidence only. Nothing here says anything about a physical robot, and the numbers (rates, distances) are
properties of this simulator on this machine. The base's physical numbers (wheel radius, separation, accelerations) are
platform-sw's, not measured here.
"""
import math
import os
import random
import sys
import time
import unittest

# Own DDS domain and own Gazebo transport partition: nothing else on this machine can answer for this simulation, and it
# cannot be seen by another one. Both must be set before the launch starts the simulator.
os.environ["ROS_DOMAIN_ID"] = str(random.randint(101, 200))
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
os.environ["GZ_PARTITION"] = f"g01_{os.getpid()}"
PARTITION = os.environ["GZ_PARTITION"]      # what makes a surviving gz server THIS run's (launch_guard.gz_server_shutdown_report)

import launch  # noqa: E402
import launch_testing  # noqa: E402
import launch_testing.actions  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
import yaml  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from builtin_interfaces.msg import Duration  # noqa: E402
from control_msgs.action import FollowJointTrajectory  # noqa: E402
from controller_manager_msgs.srv import ListControllers  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402
from trajectory_msgs.msg import JointTrajectoryPoint  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_test_support import ros_type_name, write_g_sidecar  # noqa: E402

from ament_index_python.packages import get_package_prefix as _prefix  # noqa: E402

sys.path.insert(0, os.path.join(_prefix("openamrobot_upperbody_description"), "lib", "openamrobot_upperbody_description"))
from launch_guard import (MOVE_GROUP_ALLOWED, SIM_ALLOWED, assert_launch_healthy, assert_no_process_died,  # noqa: E402
                          gz_server_shutdown_report, gz_survivor_message)

EXPECTED_PROCESSES = ["gazebo", "robot_state_publisher", "parameter_bridge", "create", "joint_state_publisher", "spawner"]


CONTROLLERS = ("joint_state_broadcaster", "left_arm_controller", "right_arm_controller",
               "left_gripper_controller", "right_gripper_controller")
PARK_WITHIN_S = 60.0     # the start-up move (park_arms.py, issue #44) takes 3 s of sim time once the arm controllers are active
PARK_SETTLED = 0.005     # rad: the position controllers track park closely in gz; well inside TRACK_TOLERANCE
PARK_STILL = 1e-4        # rad between two /joint_states samples


def _park_pose():
    """{joint: position} of `park` for both arms, from the SRDF the start-up move reads."""
    from openamrobot_upperbody_control.lift_guard import stowed_poses_from_srdf
    from openamrobot_upperbody_moveit.moveit_config import srdf_path
    with open(srdf_path()) as f:
        poses = stowed_poses_from_srdf(f.read(), ["park"])
    return {joint: value for side in poses for joint, value in poses[side][0].items()}


LEFT = [f"openarm_left_joint{n}" for n in range(1, 8)]
RIGHT = [f"openarm_right_joint{n}" for n in range(1, 8)]
FINGERS = ["openarm_left_finger_joint1", "openarm_right_finger_joint1"]
WHEELS = ["left_wheel_joint", "right_wheel_joint"]
LIFT = "lift_joint"
ACTIVE_WITHIN_S = 75.0
ARM_TARGET = {"openarm_left_joint1": 0.2, "openarm_left_joint2": -0.3, "openarm_left_joint4": 0.8}   # inside every limit
ARM_SECONDS = 3                 # simulated seconds
TRACK_TOLERANCE = 0.02          # rad
BASE_STILL_WITHIN = 0.01        # m, while an arm moves
DRIVE_SPEED = 0.2               # m/s, platform-sw's limit is 1.0
DRIVE_SECONDS = 3.0             # simulated seconds
DIFFDRIVE_ODOM_HZ = 50.0        # <odom_publish_frequency> in platform-sw's gazebo_control.xacro
SINGLE_PUBLISHER_BAND = (35.0, 65.0)        # sim-time Hz: a second publisher would be near 100


@pytest.mark.launch_test
def generate_test_description():
    sim = os.path.join(get_package_share_directory("openamrobot_upperbody_gazebo"), "launch", "sim.launch.py")
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource(sim),
                                                launch_arguments={"headless": "true"}.items()),
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestGazeboSmoke(unittest.TestCase):
    record = {}

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("g01_test", parameter_overrides=[Parameter("use_sim_time", value=True)])
        cls.started = time.monotonic()
        cls.joint_states, cls.odom = {}, []
        cls.node.create_subscription(JointState, "/joint_states", lambda m: cls.joint_states.update(zip(m.name, m.position)), 10)
        cls.node.create_subscription(Odometry, "/odom", lambda m: cls.odom.append(m), 100)
        cls.cmd_vel = cls.node.create_publisher(Twist, "/cmd_vel", 10)
        cls.list_controllers = cls.node.create_client(ListControllers, "/controller_manager/list_controllers")

    @classmethod
    def tearDownClass(cls):
        write_g_sidecar("G01", "gazebo_smoke", "combined robot in Gazebo: arm controllers, arm goal, Nav2-facing topics unchanged",
                        cls.record)
        cls.node.destroy_node()
        rclpy.shutdown()

    # --- helpers ---------------------------------------------------------------------------------------------------------
    def _spin_until(self, predicate, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return predicate()

    def _sim_now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _spin_sim_seconds(self, seconds, on_tick=None):
        start = self._sim_now()
        while self._sim_now() - start < seconds:
            if on_tick:
                on_tick()
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _settled_at(self, pose):
        """Every arm joint within PARK_SETTLED of `pose`, and the last two /joint_states samples PARK_STILL apart."""
        if not all(j in self.joint_states for j in pose) or any(abs(self.joint_states[j] - pose[j]) > PARK_SETTLED for j in pose):
            self._park_last = None
            return False
        now = {j: self.joint_states[j] for j in pose}
        last, self._park_last = getattr(self, "_park_last", None), now
        return last is not None and all(abs(now[j] - last[j]) <= PARK_STILL for j in pose)

    def _controller_states(self):
        future = self.list_controllers.call_async(ListControllers.Request())
        self._spin_until(future.done, 2.0)
        return {c.name: c.state for c in future.result().controller} if future.done() and future.result() else {}

    def _base_xy(self):
        self._spin_until(lambda: bool(self.odom), 10.0)
        p = self.odom[-1].pose.pose.position
        return p.x, p.y

    # --- 1 ---------------------------------------------------------------------------------------------------------------
    def test_g01_1_all_five_controllers_active(self):
        self.assertTrue(self.list_controllers.wait_for_service(timeout_sec=ACTIVE_WITHIN_S),
                        f"/controller_manager/list_controllers did not appear within {ACTIVE_WITHIN_S:.0f} s")
        states = {}
        while time.monotonic() < self.started + ACTIVE_WITHIN_S:
            states = self._controller_states()
            if all(states.get(name) == "active" for name in CONTROLLERS):
                self.record["controllers"] = states
                self.record["controllers_active_after_s"] = round(time.monotonic() - self.started, 1)
                return
            time.sleep(0.25)
        self.fail(f"not all controllers active within {ACTIVE_WITHIN_S:.0f} s of the launch: {states}")

    # --- 2 ---------------------------------------------------------------------------------------------------------------
    def test_g01_2_an_arm_goal_completes_and_only_that_arm_moves(self):
        self.assertTrue(self._spin_until(lambda: all(j in self.joint_states for j in LEFT + RIGHT + WHEELS), 20.0),
                        "/joint_states never listed all arm and wheel joints")
        # issue #44: park_arms.py moves both arms from home to park at start-up (joint4 0.1 rad). Measure only once both are there and
        # still, or the right arm's start-up move reads as "moved by the left arm's goal".
        park = _park_pose()
        self.assertTrue(self._spin_until(lambda: self._settled_at(park), PARK_WITHIN_S),
                        f"the arms did not settle at park within {PARK_WITHIN_S:.0f} s of the start-up move")
        right_before = {j: self.joint_states[j] for j in RIGHT}
        base_before = self._base_xy()
        target = {j: ARM_TARGET.get(j, 0.0) for j in LEFT}

        client = ActionClient(self.node, FollowJointTrajectory, "/left_arm_controller/follow_joint_trajectory")
        self.assertTrue(client.wait_for_server(timeout_sec=10.0), "left_arm_controller has no action server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = LEFT
        goal.trajectory.points = [JointTrajectoryPoint(positions=[target[j] for j in LEFT], velocities=[0.0] * 7,
                                                       time_from_start=Duration(sec=ARM_SECONDS))]
        send = client.send_goal_async(goal)
        self.assertTrue(self._spin_until(send.done, 10.0), "goal was not answered")
        handle = send.result()
        self.assertTrue(handle.accepted, "left_arm_controller rejected the goal")
        result = handle.get_result_async()
        self.assertTrue(self._spin_until(result.done, 60.0), "trajectory did not finish")
        self.assertEqual(result.result().result.error_code, FollowJointTrajectory.Result.SUCCESSFUL,
                         result.result().result.error_string)
        self._spin_sim_seconds(0.5)
        final = {j: self.joint_states[j] for j in LEFT}
        for name in LEFT:
            self.assertAlmostEqual(final[name], target[name], delta=TRACK_TOLERANCE, msg=f"{name} did not reach its target")
        for name in RIGHT:
            self.assertAlmostEqual(self.joint_states[name], right_before[name], delta=TRACK_TOLERANCE, msg=f"{name} moved")
        base_after = self._base_xy()
        moved = math.hypot(base_after[0] - base_before[0], base_after[1] - base_before[1])
        self.assertLess(moved, BASE_STILL_WITHIN, f"the base moved {moved:.4f} m while an arm moved")
        self.record["arm_goal"] = {"controller": "left_arm_controller", "target": target, "final": {k: round(v, 4) for k, v in final.items()},
                                   "tolerance_rad": TRACK_TOLERANCE, "base_moved_m": round(moved, 5),
                                   "right_arm_moved_rad": round(max(abs(self.joint_states[j] - right_before[j]) for j in RIGHT), 5)}

    # --- 3 ---------------------------------------------------------------------------------------------------------------
    def test_g01_3_the_nav2_facing_topics_are_platform_sws_by_name_and_type(self):
        bridge = os.path.join(get_package_share_directory("openamrobot_gazebo"), "config", "gz_bridge.yaml")
        entries = yaml.safe_load(open(bridge))
        self.assertTrue(entries, f"{bridge} lists nothing")
        graph = {}
        self._spin_until(lambda: False, 1.0)
        for name, types in self.node.get_topic_names_and_types():
            graph[name] = types
        checked, problems = {}, []
        for entry in entries:
            topic = "/" + entry["ros_topic_name"].lstrip("/")
            want = ros_type_name(entry["ros_type_name"])
            have = graph.get(topic)
            checked[topic] = {"type": want, "direction": entry["direction"], "found": have}
            if have is None:
                problems.append(f"{topic}: not in the graph")
            elif want not in have:
                problems.append(f"{topic}: type {have}, platform-sw's bridge file says {want}")
        self.assertEqual(problems, [], "platform-sw's topics changed under us")
        self.record["platform_sw_topics"] = checked
        self.record["platform_sw_bridge_file"] = bridge

    # --- 4 ---------------------------------------------------------------------------------------------------------------
    def test_g01_4_cmd_vel_moves_the_base_and_odom_reports_it(self):
        before = self._base_xy()
        count = len(self.odom)
        twist = Twist()
        twist.linear.x = DRIVE_SPEED
        self._spin_sim_seconds(DRIVE_SECONDS, on_tick=lambda: self.cmd_vel.publish(twist))
        self.cmd_vel.publish(Twist())
        self._spin_sim_seconds(0.5)
        after = self._base_xy()
        distance = math.hypot(after[0] - before[0], after[1] - before[1])
        # speed x time, less the DiffDrive plugin's own acceleration ramp (0.5 m/s^2): about 0.04 m at this speed
        self.assertGreater(distance, 0.8 * DRIVE_SPEED * DRIVE_SECONDS, f"/cmd_vel {DRIVE_SPEED} m/s for {DRIVE_SECONDS} s moved the base {distance:.3f} m")
        self.assertGreater(len(self.odom) - count, 50, "/odom stopped publishing")
        self.record["drive"] = {"cmd_vel_m_s": DRIVE_SPEED, "sim_seconds": DRIVE_SECONDS, "odom_moved_m": round(distance, 3),
                                "odom_messages": len(self.odom) - count}

    # --- 5 ---------------------------------------------------------------------------------------------------------------
    def test_g01_5_exactly_one_publisher_of_odom_to_the_diffdrive_child_frame(self):
        stamps, others = [], {}
        description = self._robot_description()
        import re
        children = re.findall(r"<child_frame_id>\s*([A-Za-z0-9_]+)\s*</child_frame_id>", description)
        self.assertEqual(len(children), 1, f"expected one DiffDrive <child_frame_id> in the description, found {children}")
        child = children[0]
        self.assertIn(child, ("base_footprint", "base_link"), f"the DiffDrive plugin's odom child {child!r} is not a base frame")
        self.assertIn(f'<link name="{child}"', description.replace("'", '"'), f"the odom child {child} is not a link of the description")

        def on_tf(msg):
            for t in msg.transforms:
                key = (t.header.frame_id, t.child_frame_id)
                stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
                if key == ("odom", child):
                    stamps.append(stamp)
                elif "odom" in key:
                    others[key] = others.get(key, 0) + 1

        sub = self.node.create_subscription(TFMessage, "/tf", on_tf, 200)
        self._spin_sim_seconds(5.0)
        self.node.destroy_subscription(sub)
        self.assertGreater(len(stamps), 100, f"odom -> {child} was not published")
        increasing = all(b > a for a, b in zip(stamps, stamps[1:]))
        rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        self.assertTrue(increasing, f"odom -> {child} has repeated or out-of-order stamps: two publishers")
        self.assertTrue(SINGLE_PUBLISHER_BAND[0] <= rate <= SINGLE_PUBLISHER_BAND[1],
                        f"odom -> {child} arrives at {rate:.1f} Hz; one DiffDrive publisher gives {DIFFDRIVE_ODOM_HZ:.0f}")
        self.assertEqual(others, {}, f"a transform other than odom -> {child} involves the odom frame")

        publishers = sorted({i.node_name for i in self.node.get_publishers_info_by_topic("/tf")})
        self.assertEqual(publishers, ["platform_bridge", "robot_state_publisher"], publishers)

        self.assertNotIn('<link name="odom"', description.replace("'", '"'), "`odom` is a link of the description")
        self.assertNotIn('name="base_planar_joint"', description.replace("'", '"'),
                         "the planar joint is in the description: KDL would make it a fixed joint (W02)")
        self.record["odom_to_base_frame"] = {"child_frame": child, "samples": len(stamps), "sim_rate_hz": round(rate, 1), "stamps_strictly_increasing": increasing,
                                            "expected_rate_hz": DIFFDRIVE_ODOM_HZ, "other_transforms_with_odom": others,
                                            "tf_publishers": publishers, "odom_is_a_link_of_the_description": False,
                                            "planar_joint_in_description": False}

    def _robot_description(self):
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile
        from std_msgs.msg import String
        got = []
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        sub = self.node.create_subscription(String, "/robot_description", lambda m: got.append(m.data), qos)
        self.assertTrue(self._spin_until(lambda: bool(got), 10.0), "no /robot_description")
        self.node.destroy_subscription(sub)
        return got[0]

    # --- 6 ---------------------------------------------------------------------------------------------------------------
    def test_g01_6_joint_states_carry_wheels_and_arms_each_once(self):
        sources = {"/joint_states": [], "/base/joint_states": [], "/arm/joint_states": []}
        subs = [self.node.create_subscription(JointState, topic, lambda m, t=topic: sources[t].append(list(m.name)), 50)
                for topic in sources]
        self._spin_sim_seconds(2.0)
        for sub in subs:
            self.node.destroy_subscription(sub)
        for topic, messages in sources.items():
            self.assertTrue(messages, f"{topic} published nothing")
            for names in messages:
                self.assertEqual(len(names), len(set(names)), f"{topic}: a joint appears twice in one message: {names}")

        merged = set(sources["/joint_states"][-1])
        arm_joints = set(LEFT + RIGHT + FINGERS)
        self.assertTrue(set(WHEELS) <= merged and arm_joints <= merged, f"/joint_states is missing joints: {sorted((set(WHEELS) | arm_joints) - merged)}")
        base_names, arm_names = set(sources["/base/joint_states"][-1]), set(sources["/arm/joint_states"][-1])
        # issue #17 (owner decision 1 Nov 2026): on gazebo lift_joint is reported exactly once, by joint_state_broadcaster; on real only the
        # merger's default reports it. sim_test_support.lift_source_problems holds the rule; test_g01_lift_source.py tests it both ways.
        from sim_test_support import lift_source_problems
        lift_problems = lift_source_problems("gazebo", sources["/base/joint_states"], sources["/arm/joint_states"], merged)
        self.assertFalse(lift_problems, "; ".join(lift_problems))
        self.assertEqual(base_names, set(WHEELS), "the base's joint states are not exactly the two wheels")
        self.assertTrue(arm_joints <= arm_names, "joint_state_broadcaster is missing arm joints")
        self.assertEqual(base_names & arm_names, set(), f"joints reported by both sources: {sorted(base_names & arm_names)}")

        publishers = {topic: sorted({i.node_name for i in self.node.get_publishers_info_by_topic(topic)}) for topic in sources}
        self.assertEqual(publishers["/joint_states"], ["joint_state_publisher"], "/joint_states must have exactly one publisher")
        self.assertEqual(publishers["/base/joint_states"], ["base_joint_states_bridge"])
        self.assertEqual(publishers["/arm/joint_states"], ["joint_state_broadcaster"])
        self.record["joint_states"] = {"publishers": publishers, "merged_joint_count": len(merged),
                                       "merged_joints": sorted(merged), "base_source_joints": sorted(base_names),
                                       "arm_source_joint_count": len(arm_names),
                                       "joints_reported_by_both_sources": [], "duplicates_within_a_message": []}


class TestProcessesAlive(unittest.TestCase):
    def test_no_process_of_the_launch_died_while_the_tests_were_running(self, proc_info):
        assert_no_process_died(proc_info)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_g01_processes_started_and_exited_cleanly(self, proc_info):
        """The processes of the launch started, and none ended with a code that is neither 0 nor a documented shutdown behaviour (launch_guard.py)."""
        from sim_test_support import amend_g_sidecar as _amend
        assert_launch_healthy(proc_info, expected=EXPECTED_PROCESSES, allowed=SIM_ALLOWED,
                              record=lambda records: _amend("G01", "gazebo_smoke", "process_exits", records))

    def test_g01_no_gz_server_of_this_run_survives_the_teardown(self):
        """P16: as in G02. A natural orphan was seen after G01's launch in the second evidence pass of P16 (partition g01_*): it ran for the
        rest of that A7 run at about three cores and M02, two tests later, failed a pose. The verdict is taken before the survivor is stopped."""
        report = gz_server_shutdown_report(PARTITION)
        from sim_test_support import amend_g_sidecar
        amend_g_sidecar("G01", "gazebo_smoke", "gz_server_after_shutdown", report, fail=bool(report["survivors"]))
        self.assertFalse(report["survivors"], gz_survivor_message(report))

