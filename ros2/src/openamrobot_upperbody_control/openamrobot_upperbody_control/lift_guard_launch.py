# SPDX-License-Identifier: MIT
"""The lift guard's launch actions (issue #44), shared by mock.launch.py (bringup) and sim.launch.py (gazebo). Simulated profiles only.

    lift_guard_actions(profile, srdf)   the guard node, its exit handler, and in mock the odometry stub
    park_arms_action(profile, srdf)     the start-up move of both arms to park (it can start with the arm controllers' spawners)
    LIFT_SPAWNER_ARGUMENTS        lift_controller is spawned inactive; the guard activates it

One place for the per-profile settings, as accepted on #44:
  arm_state_topic     mock: /joint_states (straight from the broadcaster); gazebo: /arm/joint_states (the raw broadcaster output: the merged
                      /joint_states keeps publishing frozen arm positions with an advancing stamp, sim.launch.py's merger)
  stowed_tolerance    per profile. Mock 0.01 rad. Gazebo 0.01 rad too: the settled park error measured there (#44, G03-06) is at most
                      3e-5 rad at rest and 2.9e-4 rad while the base drives, about 1/35 of 0.01 rad
  odometry            mock: lift_guard_odom_stub.py on /odom (there is no base); gazebo: the DiffDrive plugin's /odom
  use_sim_time        gazebo only
The other thresholds keep lift_guard.DEFAULTS (provisional until the firmware design sets them).

`srdf` is the text of the SRDF move_group uses (openamrobot_upperbody_moveit's config/openamrobot.srdf); the stowed poses come from it and
nowhere else. The caller reads it: this package cannot depend on the moveit package, which already depends on this one.
"""
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

PACKAGE = "openamrobot_upperbody_control"
LIFT_CONTROLLER = "lift_controller"
LIFT_SPAWNER_ARGUMENTS = ("--inactive",)
# The fallback after a guard dies without handling a signal (SIGKILL, a crash). Bounded: `ros2 control` waits for the controller manager's
# service without end, and a fallback that never exits would keep the launch alive if the manager were gone too (seen in the mock stack).
FALLBACK_TIMEOUT_S = "15"
FALLBACK_COMMAND = ("timeout", "--kill-after=2", FALLBACK_TIMEOUT_S,
                    "ros2", "control", "switch_controllers", "--deactivate", LIFT_CONTROLLER, "--best-effort")
PROFILES = {
    "mock": {"arm_state_topic": "/joint_states", "stowed_tolerance": 0.01, "use_sim_time": False, "odom_stub": True},
    "gazebo": {"arm_state_topic": "/arm/joint_states", "stowed_tolerance": 0.01, "use_sim_time": True, "odom_stub": False},
}


def _profile(profile):
    if profile not in PROFILES:
        raise ValueError(f"lift guard profile {profile!r}: accepted values are {', '.join(PROFILES)} (the real profile has no guard)")
    return PROFILES[profile]


def lift_guard_actions(profile, srdf):
    """[guard node, its OnProcessExit handler, (mock) odometry stub]. On SIGINT/SIGTERM the guard deactivates lift_controller itself; on any
    other exit (SIGKILL, a crash) the handler does, through the controller manager (best effort: an inactive controller is not an error),
    bounded by FALLBACK_TIMEOUT_S. During the launch's own shutdown the handler starts nothing (launch does not execute processes then)."""
    settings = _profile(profile)
    guard = Node(package=PACKAGE, executable="lift_guard.py", name="lift_guard", output="screen",
                 parameters=[{"robot_description_semantic": srdf, "arm_state_topic": settings["arm_state_topic"],
                              "stowed_tolerance": settings["stowed_tolerance"], "use_sim_time": settings["use_sim_time"]}])
    deactivate = ExecuteProcess(cmd=list(FALLBACK_COMMAND), output="screen")
    actions = [guard, RegisterEventHandler(OnProcessExit(target_action=guard, on_exit=[deactivate]))]
    if settings["odom_stub"]:
        actions.append(Node(package=PACKAGE, executable="lift_guard_odom_stub.py", name="lift_guard_odom_stub", output="screen"))
    return actions


def park_arms_action(profile, srdf):
    """The start-up move to park. It resends a goal the arm controller rejects while not yet active, so it can start with their spawners."""
    settings = _profile(profile)
    return Node(package=PACKAGE, executable="park_arms.py", name="park_arms", output="screen",
                parameters=[{"robot_description_semantic": srdf, "state": "park", "use_sim_time": settings["use_sim_time"]}])
