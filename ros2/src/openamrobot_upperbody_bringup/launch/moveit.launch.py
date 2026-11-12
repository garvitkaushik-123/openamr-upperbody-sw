# SPDX-License-Identifier: MIT
"""MoveIt on the combined OpenAMRobot model: the stack (mock or Gazebo), then move_group, optionally RViz's MotionPlanning display.

    ros2 launch openamrobot_upperbody_bringup moveit.launch.py [base_plate:=bp000] [sim:=mock|gazebo] [rviz:=false]
                                                              [lift_position:=0.0] [headless:=false] [physics_engine:=<gz-physics plugin>]

  sim=mock     mock.launch.py (arms and lift on mock_components/GenericSystem, six controllers, the lift guard), a stub that reports the wheel
               joints at rest (the base is not under ros2_control), then move_group. Nothing else of the base runs. With a non-zero
               `lift_position`, one trajectory is sent to lift_controller to take the lift there from 0, once the lift guard allows it
               (issue #44: mock.launch.py runs the guard, the arms' start-up move to park and the odometry stub).
  sim=gazebo   openamrobot_upperbody_gazebo's sim.launch.py (Gazebo, the base on its DiffDrive plugin, the arms and the lift on
               gz_ros2_control, the six controllers), then move_group on the simulator's clock.
  In both, lift_joint is a planned joint (issue #17): MoveIt group `lift`, lift_controller. The real profile has no command interface for it.

Args
  column       dold_v4_350 | tl3_400   (default dold_v4_350; tl3_400 has no supplier data and does not expand)
  base_plate   bp000 | bp050             (default bp000: option B's baseline; column axis x +0.060 / +0.110 from base_link)
  head_pitch   hp15 | hp20 | hp25 | hp30 | hp35   (default hp30)
  base_tilt    bt05 | bt10 | bt15        (default bt10)
  lift_position   sim:=mock only. m, where lift_controller takes lift_joint after start-up (one FollowJointTrajectory goal at the
                  DOLD no-load speed, sent once the lift guard allows the lift, #44): 0.0 / 0.175 / 0.350 are the calibration stops
                  L1000 / L1175 / L1350 (default 0.0: nothing is sent, the lift stays at its mock start 0). With sim:=gazebo the lift
                  starts at the same 0.0 and nothing is sent. The A7 scenario does not use this: it PLANS the lift through MoveIt.
  sim             mock | gazebo
  rviz            true | false  RViz2 with the MotionPlanning display (config/moveit.rviz of the moveit package). With sim:=gazebo the
                  simulator's own RViz is not started: this one replaces it.
  headless        sim:=gazebo only. true runs `gz sim -s` (no Gazebo window), as the checks do
  physics_engine  sim:=gazebo only. The gz-physics plugin, passed to sim.launch.py. EMPTY by default, which means sim.launch.py's own
                  default: DART, the engine platform-sw tuned the base on. DART does not close the grippers (its fingers do not follow
                  position commands at the default gain, and it has no mimic constraint), so the arm named poses work there and the
                  gripper ones may not. Pass gz-physics-bullet-featherstone-plugin for gripper poses to close: M02 does, for that reason.

Everything here is simulation: it permits no statement about a physical robot or a limit of one (ADR-009).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def include(package, launch_file, **launch_arguments):
    path = get_package_share_directory(package) + "/launch/" + launch_file
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=launch_arguments.items())


LIFT_SPEED_FRACTION = 0.5       # of the DOLD no-load speed (datum lift_max_velocity_*): the move to lift_position takes twice the fastest time


def lift_to(position, column):
    """sim:=mock: [] for 0.0, else one FollowJointTrajectory goal to lift_controller taking lift_joint from its mock start 0 to `position` (m),
    refused outside 0 .. the column's stroke. Not a planned motion and not the A7 scenario: a viewing aid.
    Issue #44: lift_controller is spawned inactive and rejects goals until the lift guard activates it, so the goal is sent by
    wait_for_lift_guard.py once it has seen /lift_guard/status OK, and it exits 0 at a teardown (`ros2 action send_goal` exited 1 when a
    teardown interrupted the goal, which every launch test's exit-code check fails on)."""
    if position == 0.0:
        return []
    import math
    import yaml
    share = get_package_share_directory("openamrobot_upperbody_description")
    with open(share + "/config/upperbody_configuration.yaml") as f:
        stroke = yaml.safe_load(f)["column"][column]["stroke"]
    with open(share + "/config/datums.yaml") as f:
        speed = yaml.safe_load(f)["datums"][f"lift_max_velocity_{column}"]["value"]
    if not 0.0 <= position <= stroke:
        raise RuntimeError(f"lift_position:={position}: lift_joint ranges 0 to {stroke} m")
    seconds = math.ceil(position / (speed * LIFT_SPEED_FRACTION))
    return [Node(package="openamrobot_upperbody_control", executable="wait_for_lift_guard.py", name="wait_for_lift_guard", output="log",
                 parameters=[{"lift_position": float(position), "duration_s": float(seconds)}])]


def launch_setup(context, *args, **kwargs):
    config = {key: LaunchConfiguration(key).perform(context) for key in ("column", "base_plate", "head_pitch", "base_tilt")}
    sim = LaunchConfiguration("sim").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context)
    if sim not in ("mock", "gazebo"):
        raise RuntimeError(f"sim:={sim}: accepted values are mock, gazebo")
    if sim == "mock":
        # The base is not under ros2_control, so on mock nothing reports its wheel joints and MoveIt warns every second that the
        # robot's complete state is unknown. The stub reports them at rest (the base does not move during manipulation, AD-30).
        stack = [include("openamrobot_upperbody_bringup", "mock.launch.py", rviz="false", **config),
                 Node(package="openamrobot_upperbody_bringup", executable="base_joint_states_stub.py", output="log"),
                 *lift_to(float(LaunchConfiguration("lift_position").perform(context)), config["column"])]
    else:
        sim_args = {**config, "headless": LaunchConfiguration("headless").perform(context), "rviz": "false"}
        engine = LaunchConfiguration("physics_engine").perform(context)
        if engine:
            sim_args["physics_engine"] = engine
        stack = [include("openamrobot_upperbody_gazebo", "sim.launch.py", **sim_args)]
    move_group = include("openamrobot_upperbody_moveit", "move_group.launch.py", hardware_profile=sim, rviz=rviz, **config)
    return [*stack, move_group]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("column", default_value="dold_v4_350", description="dold_v4_350 | tl3_400 (no supplier data: refused)"),
        DeclareLaunchArgument("base_plate", default_value="bp000", description="bp000 | bp050"),
        DeclareLaunchArgument("head_pitch", default_value="hp30", description="hp15 | hp20 | hp25 | hp30 | hp35"),
        DeclareLaunchArgument("base_tilt", default_value="bt10", description="bt05 | bt10 | bt15"),
        DeclareLaunchArgument("lift_position", default_value="0.0",
                              description="sim:=mock only: lift_joint's starting position (m); 0.0 / 0.175 / 0.350 = L1000 / L1175 / L1350"),
        DeclareLaunchArgument("sim", default_value="mock", description="mock | gazebo"),
        DeclareLaunchArgument("rviz", default_value="false", description="true | false. RViz2 with the MotionPlanning display"),
        DeclareLaunchArgument("headless", default_value="false", description="sim:=gazebo only: true runs gz sim -s (no window)"),
        DeclareLaunchArgument("physics_engine", default_value="",
                              description="sim:=gazebo only: gz-physics plugin; empty = sim.launch.py's default (DART)"),
        OpaqueFunction(function=launch_setup),
    ])
