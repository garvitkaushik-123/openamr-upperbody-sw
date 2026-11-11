# SPDX-License-Identifier: MIT
"""The combined OpenAMRobot 2.0 robot in Gazebo Harmonic: the base on its DiffDrive plugin, both arms on gz_ros2_control.

    ros2 launch openamrobot_upperbody_gazebo sim.launch.py [base_plate:=bp000] [headless:=true] [world:=/path/x.sdf]
                                                           [rviz:=true]

What it starts, in this order of dependence
  gz sim                 -r (run), plus -s (server only, no window) when headless:=true
  robot_state_publisher  the combined description at hardware_profile:=gazebo, use_sim_time
  ros_gz_sim create      spawns the model from the robot_description topic; the model carries the base's DiffDrive and
                         JointStatePublisher plugins (platform-sw's, unchanged) and our gz_ros2_control plugin, which
                         starts the controller manager inside the simulator
  ros_gz_bridge          platform-sw's own gz_bridge.yaml, BY REFERENCE (clock, /cmd_vel, /odom, /tf, sensors; every topic
                         Nav2 and docking use, under the names they use), plus one bridge of ours for the base's joint
                         states onto a separate topic (see below)
  spawners               joint_state_broadcaster (switch timeout JSB_SWITCH_TIMEOUT_S, service-call timeout longer than that), then,
                         only if that spawner exits 0,
                         left_arm, right_arm, left_gripper and right_gripper controllers, and lift_controller (issue #17),
                         INACTIVE (issue #44), with a one-off move of both arms to park (park_arms.py)
  lift_guard             activates lift_controller only while both arms are stowed and the base is stopped (issue #44); it reads the
                         raw /arm/joint_states and the DiffDrive /odom, and deactivates lift_controller on exit
  joint_state_publisher  the ONLY publisher of /joint_states: it merges the base's wheel joints and the arms' and lift's joints
                         (lift_joint comes from the physics through joint_state_broadcaster since issue #17)

/joint_states: platform-sw's JointStatePublisher (scoped by its own <joint_name> list to the two wheels) and our
joint_state_broadcaster (the sixteen arm joints) must not both write to /joint_states. The wheels come in on
/base/joint_states, the arms on /arm/joint_states, and joint_state_publisher merges them. See the package README for what
was observed before this was decided.

odom -> base_footprint: from the DiffDrive plugin through the /tf bridge, and from nowhere else (platform-sw 935efc7: the odom
child is base_footprint so that base_link keeps one parent, REP 105). robot_state_publisher cannot publish it, because `odom`
is not a link of the description (and with the planar joint enabled KDL turns it into a fixed joint, check W02). The default
here is enable_planar_base:=false.

Args
  column       dold_v4_350 | tl3_400   (default dold_v4_350; tl3_400 has no supplier data and does not expand)
  base_plate   bp000 | bp050             (default bp000: option B's baseline; column axis x +0.060 / +0.110 from base_link)
  head_pitch   hp15 | hp20 | hp25 | hp30 | hp35   (default hp30)
  base_tilt    bt05 | bt10 | bt15        (default bt10)
  world     path to a Gazebo world .sdf                       (default: worlds/empty.sdf in this package; platform-sw's is
            $(ros2 pkg prefix --share openamrobot_gazebo)/worlds/walled_world.sdf)
  headless  true | false     (default false) true runs `gz sim -s`: no Gazebo window, as the smoke test does
  physics_engine  the gz-physics plugin for whichever world is loaded (default gz-physics-dartsim-plugin: DART, the engine
            platform-sw tuned the base's DiffDrive on, so this launch does not re-baseline someone else's physics).
            DART cannot run this model's grippers: its fingers do not follow position commands and it has no mimic
            constraint, so finger_joint2 never follows finger_joint1. Pass gz-physics-bullet-featherstone-plugin to
            close the grippers (G02 does). See the package README.
  rviz      true | false     (default false) RViz2 with the fixed frame `odom`, beside the simulator, for a person
"""
import json
import os
import signal
import subprocess
import sys
import time

# --- stopping the simulator (P17 task 1) --------------------------------------------------------------------------------------------------
# What was measured, because the first version of this handler rested on a guess:
#   * launch starts `gz sim` as `/bin/sh -c "ruby .../gz sim ..."` (ros_gz_sim, shell=True) and signals the SHELL only, so under launch_testing the
#     server receives no signal of the launch at all: whatever stops it has to be done to it directly. (From a terminal, Ctrl-C reaches the whole
#     foreground group, the server included, which is why this never showed by hand.)
#   * the first version asked the server to stop through its /server_control service (the request is answered `data: true`). Over 18 launches
#     under launch_testing it went wrong 4 times: twice the server segfaulted inside libgz-sim8 during its teardown and lingered until the shell was
#     SIGTERMed at 5 s, and twice it accepted the request and never began its teardown (no controller_manager destructor output), ignoring SIGTERM:
#     only SIGKILL ended it, and launch's SIGTERM of the shell at 5 s had left it running as an orphan. From the log history (154 launches) the
#     same failures are 14 segfaults and about 10 silent hangs.
#   * SIGINT sent to the server process itself, which is what Ctrl-C does and which the server's own handler turns into a stop, went wrong 0 times
#     in 24 launches (1.32 to 1.54 s each, against 1.64 to 1.79 s for the service). 24 clean runs do not prove it cannot fail: with a failure
#     rate of 20% the chance of 24 clean ones is under 1%, and with 5% it is 29%.
#   * launch's own OnShutdown fires twice (the shell's death, `on_exit_shutdown`, is a second Shutdown), and a stop run as an ExecuteProcess is not
#     started at all when `context.is_shutdown` is already set (`ros2 launch` and SIGINT: the first firing is a no-op, the second raises
#     "executed more than once" and abandons the rest of the shutdown).
# So, once per launch and from a detached helper (this file run as a script: it imports nothing of ROS, starts in milliseconds and does not depend on
# the launch's state), in this order: SIGINT to the `gz sim` processes; after GZ_STOP_RETRY_AFTER_S, if one is still there, the /server_control
# request as well; after GZ_STOP_GRACE_S, SIGTERM and then SIGKILL. The servers are followed by pid, because launch's SIGTERM of the shell at 5 s
# reparents them out of its tree. The helper writes its report to gz_stop_report_path() BEFORE it signals anything harder than SIGINT, and
# launch_guard.gz_server_shutdown_report reads it (from the evidence directory): a server that had to be forced is a WARN (A7 check W04), and one that is
# still running after that is the survivor check's FAIL.
GZ_STOP_RETRY_AFTER_S = 3.0     # s after the SIGINT before the service request is made as well (a stop takes 1.3-1.8 s when it works)
GZ_STOP_GRACE_S = 8.0           # s after the first request that a server may take to go on its own; then SIGTERM, then SIGKILL
GZ_STOP_TERM_WAIT_S = 1.0       # s between the SIGTERM and the SIGKILL
GZ_STOP_REQUEST_TIMEOUT_S = 3.0


def gz_stop_report_path(partition):
    """Where the helper leaves its report for the partition `partition` (the test's GZ_PARTITION, which every process of the launch inherits):
    `gz_stop_<partition>.json` in the EVIDENCE directory, because the report is part of a run's evidence (the test copies it into its sidecar) and
    evidence is never kept in a temp directory (test_evidence_dir.py). Resolved with the description package's own evidence_common, the same
    function launch_guard.gz_stop_report_path uses, so the two cannot name different files; None when that package cannot be imported (the helper
    then stops the server and writes no report). Not run by the helper itself: the launch resolves it and passes it on the command line."""
    try:
        from ament_index_python.packages import get_package_prefix
        lib = os.path.join(get_package_prefix("openamrobot_upperbody_description"), "lib", "openamrobot_upperbody_description")
        if lib not in sys.path:
            sys.path.insert(0, lib)
        import evidence_common
        return os.path.join(evidence_common.evidence_dir(evidence_common.share_dir()), f"gz_stop_{partition}.json")
    except Exception:       # noqa: BLE001 - a report that cannot be placed is not a reason to leave the server running
        return None


def _read(path):
    try:
        with open(path, "rb") as f:
            return f.read()
    except OSError:
        return b""


def _stat_fields(pid, proc_root="/proc"):
    raw = _read(os.path.join(proc_root, str(pid), "stat")).decode("utf-8", "replace")
    return raw.rpartition(")")[2].split()


def _live(pid, proc_root="/proc"):
    after = _stat_fields(pid, proc_root)
    return bool(after) and after[0] not in ("Z", "X")


def _start_time(pid, proc_root="/proc"):
    """Field 22 of /proc/<pid>/stat: with the pid it names one process for the life of the boot, so a reused pid is not mistaken for the server."""
    after = _stat_fields(pid, proc_root)
    return after[19] if len(after) > 19 else None


def _descendants(root, proc_root="/proc"):
    """Pids below `root` (not `root` itself), read from /proc."""
    parent_of = {}
    for name in os.listdir(proc_root):
        if name.isdigit():
            after = _read(os.path.join(proc_root, name, "stat")).decode("utf-8", "replace").rpartition(")")[2].split()
            if len(after) >= 2:
                parent_of[int(name)] = int(after[1])
    found, frontier = set(), {root}
    while frontier:
        frontier = {pid for pid, ppid in parent_of.items() if ppid in frontier and pid not in found}
        found |= frontier
    return found


def _is_gz_sim(pid, proc_root="/proc"):
    """`gz sim ...` (the server, which rewrites its argv into one string) or `ruby .../gz sim ...`: `sim` among the first arguments and a `gz` in
    the first two. Not `/bin/sh -c ruby ...` (the shell is launch's own to end) and not a `gz service` request."""
    tokens = _read(os.path.join(proc_root, str(pid), "cmdline")).decode("utf-8", "replace").replace("\0", " ").split()
    return "sim" in tokens[1:4] and any(os.path.basename(t) == "gz" for t in tokens[:2])


def request_server_stop(timeout_s=GZ_STOP_REQUEST_TIMEOUT_S):
    """Ask the server to stop through its own service. -> (accepted, text). `data: true` is the server's answer; it does NOT mean the server then
    went (see above), which is why the caller watches."""
    cmd = ["gz", "service", "-s", "/server_control", "--reqtype", "gz.msgs.ServerControl", "--reptype", "gz.msgs.Boolean",
           "--timeout", str(int(timeout_s * 1000)), "--req", "stop: true"]
    try:
        done = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s + 2.0)
    except (OSError, subprocess.SubprocessError) as error:
        return False, f"{type(error).__name__}: {error}"
    text = (done.stdout + done.stderr).strip()
    return "data: true" in text, text[:200]


def request_server_stop_by_signal(shell_pid, proc_root="/proc"):
    """The other way to ask: SIGINT to the `gz sim` processes below `shell_pid`, which is what Ctrl-C in a terminal does (the whole foreground
    group gets it) and which the server's own handler turns into a stop. -> (sent, text)."""
    sent = []
    for pid in sorted(_descendants(shell_pid, proc_root)) if shell_pid else ():
        if _live(pid, proc_root) and _is_gz_sim(pid, proc_root):
            try:
                os.kill(pid, signal.SIGINT)
                sent.append(pid)
            except (ProcessLookupError, PermissionError):
                pass
    return bool(sent), f"SIGINT to {sent}"


def stop_gz_server(shell_pid, interrupt=None, request=request_server_stop, retry_after_s=GZ_STOP_RETRY_AFTER_S, grace_s=GZ_STOP_GRACE_S,
                   term_wait_s=GZ_STOP_TERM_WAIT_S, report_path=None, proc_root="/proc", poll_s=0.1):
    """Stop the `gz sim` server below `shell_pid`, once, and see that it goes. Returns (and writes to `report_path`) {state, ...}:
         "stopped"        every server was gone within `grace_s` of the first request, on its own
         "force_stopped"  one was not; it was sent SIGTERM and, `term_wait_s` later, SIGKILL (`signals` says which were needed)
         "no_server"      there was no `gz sim` below `shell_pid` (never started, or already gone)
       Steps: `interrupt` (default: SIGINT to every `gz sim` of the tree) at once; `request` (the /server_control service) as well if a server is
       still there after `retry_after_s`; signals after `grace_s`. `retried_with_service` says whether the second step was needed.
       The servers are followed by pid and start time, not by their place in the tree (see `tracked`).
       The file holds {"state": "started"} while the helper is still watching and is rewritten with the state BEFORE any SIGTERM or SIGKILL."""
    started = time.monotonic()
    interrupt = interrupt or (lambda: request_server_stop_by_signal(shell_pid, proc_root))
    report = {"state": "started", "shell_pid": shell_pid, "retry_after_s": retry_after_s, "grace_s": grace_s, "retried_with_service": False}

    def write():
        if report_path:
            tmp = f"{report_path}.{os.getpid()}"
            with open(tmp, "w") as f:
                json.dump(report, f)
            os.replace(tmp, report_path)

    tracked = {}        # pid -> start time of every `gz sim` seen below the shell. Kept by pid: when launch SIGTERMs the shell at 5 s the
                        # server is reparented to init and is no longer below it, and a helper that only walked the tree concluded it was gone

    def servers():
        for pid in (_descendants(shell_pid, proc_root) if shell_pid else ()):
            if pid not in tracked and _live(pid, proc_root) and _is_gz_sim(pid, proc_root):
                tracked[pid] = _start_time(pid, proc_root)
        return sorted(pid for pid, started_at in tracked.items()
                      if _live(pid, proc_root) and _start_time(pid, proc_root) == started_at and _is_gz_sim(pid, proc_root))

    write()
    report["servers_at_request"] = servers()
    report["interrupt"] = interrupt()[1]
    remaining = servers()
    while remaining and time.monotonic() - started < grace_s:
        if not report["retried_with_service"] and time.monotonic() - started >= retry_after_s:
            report["retried_with_service"] = True
            report["service_request"] = request()[1]
        time.sleep(poll_s)
        remaining = servers()
    report["elapsed_s"] = round(time.monotonic() - started, 2)
    if not remaining:
        report["state"] = "stopped" if report["servers_at_request"] else "no_server"
        write()
        return report
    report.update(state="force_stopped", survivors=remaining, signals=[])
    write()       # before the signal: the detector must be able to see what is about to be done
    for sig in (signal.SIGTERM, signal.SIGKILL):
        for pid in remaining:
            try:
                os.kill(pid, sig)
            except (ProcessLookupError, PermissionError):
                pass
        report["signals"].append(sig.name)
        end = time.monotonic() + (term_wait_s if sig == signal.SIGTERM else 2.0)
        while time.monotonic() < end and any(_live(pid, proc_root) for pid in remaining):
            time.sleep(poll_s)
        if not any(_live(pid, proc_root) for pid in remaining):
            break
    report["elapsed_s"] = round(time.monotonic() - started, 2)
    report["gone"] = not any(_live(pid, proc_root) for pid in remaining)
    write()
    return report


if __name__ == "__main__" and len(sys.argv) in (3, 4) and sys.argv[1] == "--stop-gz-server":
    # the detached helper (see above); nothing below this point is imported. argv[3], when given, is where it writes its report.
    stop_gz_server(int(sys.argv[2]) or None, report_path=(sys.argv[3] if len(sys.argv) == 4 else None))
    raise SystemExit(0)

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from openamrobot_upperbody_control.lift_guard_launch import LIFT_SPAWNER_ARGUMENTS, lift_guard_actions, park_arms_action
from openamrobot_upperbody_moveit.moveit_config import srdf_path

DESCRIPTION = "openamrobot_upperbody_description"
CONTROL = "openamrobot_upperbody_control"
GAZEBO = "openamrobot_upperbody_gazebo"
PLATFORM_GAZEBO = "openamrobot_gazebo"
ARM_AND_GRIPPER_CONTROLLERS = ("left_arm_controller", "right_arm_controller",
                               "left_gripper_controller", "right_gripper_controller")
# The lift (issue #17): lift_joint has a gz_ros2_control interface in this profile (ros2_control.xacro openamr_lift_ros2_control), and
# lift_controller is spawned with the arm and gripper controllers, after joint_state_broadcaster.
LIFT_CONTROLLER = "lift_controller"
# Packages whose meshes the model references as package://<name>/...; Gazebo resolves that against the parent of the
# package's share directory (the same trick platform-sw's gz_simulator.launch.py uses for its own description).
MESH_PACKAGES = ("openamrobot_description", "openarm_description", DESCRIPTION)
# DART is the engine platform-sw's DiffDrive was tuned on (their launch runs it), so it is the default here (decided 6 Oct, P10
# task 0). Gripper motion needs BULLET_FEATHERSTONE_ENGINE, and the launch does not hide that behind the default: pass it.
DEFAULT_PHYSICS_ENGINE = "gz-physics-dartsim-plugin"          # the tests record it
BULLET_FEATHERSTONE_ENGINE = "gz-physics-bullet-featherstone-plugin"   # honours the URDF <mimic>; what G02 and M02 pass
SPAWN_HEIGHT = "0.15"          # m above the ground plane; the model settles onto its wheels and casters in the first second
CONTROLLER_MANAGER_TIMEOUT = "120"      # s the spawners wait for the controller manager that the simulator hosts
# s the joint_state_broadcaster's switch may take. The spawner's default is 5 s; on a slow machine (CPU-only rendering, the simulator at
# about 0.01x real time during startup) the first switch timed out, the spawner exited 1, and the arm and gripper spawners still started,
# so the stack ran with four of five controllers active (16 Oct verification). The Jazzy spawner takes --switch-timeout.
JSB_SWITCH_TIMEOUT_S = "30"
# s the spawner waits for the switch_controller service's answer. Its default (10 s) is shorter than the switch timeout above, so a slow
# switch was abandoned at 10 s and retried, and the spawner exited 1 although the switch then completed: measured, 16 Oct. Longer than the
# switch timeout, so the controller manager's own timeout is what decides.
JSB_SERVICE_CALL_TIMEOUT_S = "45"


def spawner(controller, *extra):
    return Node(package="controller_manager", executable="spawner", output="screen",
                arguments=[controller, "--controller-manager", "/controller_manager",
                           "--controller-manager-timeout", CONTROLLER_MANAGER_TIMEOUT, *extra],
                parameters=[{"use_sim_time": True}])


def read_srdf():
    """The SRDF move_group uses: the lift guard and the start-up park move read the stowed poses from it (issue #44)."""
    with open(srdf_path()) as f:
        return f.read()


def after_joint_state_broadcaster(event, context):
    """OnProcessExit of the joint_state_broadcaster spawner: the arm and gripper spawners, only if it exited 0. A broadcaster that did not
    activate leaves the stack without /arm/joint_states; starting the other controllers anyway hid that (four of five active), so a failed
    broadcaster now stops the chain and says why. The tests that wait for five active controllers then fail, naming the broadcaster.
    Issue #44: lift_controller is spawned inactive (the lift guard activates it), and the arms are moved to park once, with the spawners."""
    if event.returncode == 0:
        return [*(spawner(name) for name in ARM_AND_GRIPPER_CONTROLLERS), spawner(LIFT_CONTROLLER, *LIFT_SPAWNER_ARGUMENTS),
                park_arms_action("gazebo", read_srdf())]
    return [LogInfo(msg=f"joint_state_broadcaster spawner exited {event.returncode}: the arm, gripper and lift spawners are not started")]


def generate_launch_description():
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    physics_engine = LaunchConfiguration("physics_engine")
    rviz = LaunchConfiguration("rviz")
    gazebo_share = get_package_share_directory(GAZEBO)
    platform_gazebo_share = get_package_share_directory(PLATFORM_GAZEBO)

    resource_paths = [AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(gazebo_share, "worlds"))]
    for package in MESH_PACKAGES:
        resource_paths.append(AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", os.path.dirname(get_package_share_directory(package))))

    # platform-sw's walled_world.sdf <include>s model://apriltag_dock, which openamrobot_docking ships; platform-sw's own launch
    # adds that directory too. Without the package the empty world and any world that does not need it are unaffected.
    try:
        resource_paths.append(AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", os.path.join(get_package_share_directory("openamrobot_docking"), "models")))
    except PackageNotFoundError:
        pass

    # The description knows nothing about where a controller config lives (ADR-003); this launch depends on both packages, so it
    # is what hands the controller manager inside the simulator its parameter file, as an absolute path.
    controllers_file = os.path.join(get_package_share_directory(CONTROL), "config", "arm_controllers.yaml")

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([FindPackageShare(DESCRIPTION), "urdf", "combined_robot.xacro"]),
            " column:=", LaunchConfiguration("column"),
            " base_plate:=", LaunchConfiguration("base_plate"),
            " head_pitch:=", LaunchConfiguration("head_pitch"),
            " base_tilt:=", LaunchConfiguration("base_tilt"),
            " hardware_profile:=gazebo",
            " controllers_file:=", controllers_file,
        ]),
        value_type=str,
    )

    gz_sim = os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    gz_sim_window = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim),
        launch_arguments={"gz_args": ["-r -v 2 --physics-engine ", physics_engine, " ", world],
                          "on_exit_shutdown": "true"}.items(),
        condition=UnlessCondition(headless))
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim),
        launch_arguments={"gz_args": ["-r -s -v 2 --physics-engine ", physics_engine, " ", world],
                          "on_exit_shutdown": "true"}.items(),
        condition=IfCondition(headless))

    robot_state_publisher = Node(package="robot_state_publisher", executable="robot_state_publisher",
                                 name="robot_state_publisher", output="screen",
                                 parameters=[{"use_sim_time": True, "robot_description": robot_description}])

    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-name", "openamrobot", "-topic", "robot_description",
                            "-x", "0", "-y", "0", "-z", SPAWN_HEIGHT])

    # Platform-sw's bridge, by reference: it bridges exactly the topics Nav2 and docking consume, under their names.
    platform_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="platform_bridge", output="screen",
        parameters=[{"config_file": os.path.join(platform_gazebo_share, "config", "gz_bridge.yaml"),
                     "qos_overrides./tf_static.publisher.durability": "transient_local"}])
    # Ours: only what platform-sw's file leaves out, the base's joint states, onto a topic of their own.
    base_joint_states_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="base_joint_states_bridge", output="screen",
        parameters=[{"config_file": os.path.join(gazebo_share, "config", "gz_bridge_base_joint_states.yaml")}])

    joint_state_broadcaster = spawner(
        "joint_state_broadcaster", "--switch-timeout", JSB_SWITCH_TIMEOUT_S, "--service-call-timeout", JSB_SERVICE_CALL_TIMEOUT_S,
        "--controller-ros-args", "-r joint_states:=/arm/joint_states")
    then_the_rest = RegisterEventHandler(OnProcessExit(
        target_action=joint_state_broadcaster, on_exit=after_joint_state_broadcaster))

    joint_state_merger = Node(
        package="joint_state_publisher", executable="joint_state_publisher", name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True, "source_list": ["/base/joint_states", "/arm/joint_states"], "rate": 50}])

    # Stopping the simulator: ONCE per launch, from whichever Shutdown comes first, through a detached helper that asks the server to stop
    # and then sees that it does (the module's header says what was measured and why). The shell that ros_gz_sim wraps `gz sim` in is the
    # root of the process tree the helper watches; its pid is taken from the ProcessStarted event.
    gazebo_shell = {"pid": None, "stop_requested": False}

    def remember_the_shell(event, context):
        if event.process_name.startswith("gazebo") and gazebo_shell["pid"] is None:
            gazebo_shell["pid"] = event.pid

    def stop_the_server_once(event, context):
        if gazebo_shell["stop_requested"]:      # launch's second Shutdown (the shell's death, on_exit_shutdown): nothing more to do
            return None
        gazebo_shell["stop_requested"] = True
        helper = os.path.join(gazebo_share, "launch", "sim.launch.py")
        partition = os.environ.get("GZ_PARTITION", "")
        report = gz_stop_report_path(partition) if partition else None
        subprocess.Popen([sys.executable, helper, "--stop-gz-server", str(gazebo_shell["pid"] or 0), *([report] if report else [])],
                         stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        return None

    track_gazebo_shell = RegisterEventHandler(OnProcessStart(on_start=remember_the_shell))
    stop_server = RegisterEventHandler(OnShutdown(on_shutdown=stop_the_server_once))

    rviz_node = Node(package="rviz2", executable="rviz2", output="screen", condition=IfCondition(rviz),
                     parameters=[{"use_sim_time": True}],
                     arguments=["-d", os.path.join(gazebo_share, "config", "sim.rviz")])

    return LaunchDescription([
        DeclareLaunchArgument("column", default_value="dold_v4_350", description="dold_v4_350 | tl3_400 (no supplier data: refused)"),
        DeclareLaunchArgument("base_plate", default_value="bp000", description="bp000 | bp050"),
        DeclareLaunchArgument("head_pitch", default_value="hp30", description="hp15 | hp20 | hp25 | hp30 | hp35"),
        DeclareLaunchArgument("base_tilt", default_value="bt10", description="bt05 | bt10 | bt15"),
        DeclareLaunchArgument("world", default_value=os.path.join(gazebo_share, "worlds", "empty.sdf"),
                              description="Path to the Gazebo world .sdf"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="true | false. gz sim -s: server only, no Gazebo window"),
        DeclareLaunchArgument("physics_engine", default_value=DEFAULT_PHYSICS_ENGINE,
                              description="gz-physics plugin; default DART (platform-sw's), which cannot close the grippers: "
                                          "pass gz-physics-bullet-featherstone-plugin for that (README)"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="true | false. RViz2 beside the simulator, fixed frame odom"),
        *resource_paths,
        gz_sim_window,
        gz_sim_server,
        robot_state_publisher,
        platform_bridge,
        base_joint_states_bridge,
        spawn,
        joint_state_broadcaster,
        then_the_rest,
        joint_state_merger,
        *lift_guard_actions("gazebo", read_srdf()),
        rviz_node,
        track_gazebo_shell,
        stop_server,
    ])
