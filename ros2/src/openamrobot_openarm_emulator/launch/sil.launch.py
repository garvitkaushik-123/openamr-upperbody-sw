# SPDX-License-Identifier: MIT
"""Software in the loop for the OpenArm 2.0 arms: the real driver on virtual CAN, emulated motors, Gazebo physics.

    ros2 launch openamrobot_openarm_emulator sil.launch.py [headless:=true] [example_trajectory:=false]

What it starts, in this order of dependence
  setup_vcan.sh          creates vcan_arm_l and vcan_arm_r, with CAN FD (MTU 72) where the kernel allows, and reports it
  gz sim                 this package's world (1 ms physics, real time, no sensor rendering)
  /sim side: the plant
    robot_state_publisher   the combined model at hardware_profile:=gazebo with sim_dynamics:=off (no joint damping or friction)
    ros_gz_sim create       spawns it; its gz_ros2_control plugin starts a controller manager at /sim/controller_manager
    spawners                joint_state_broadcaster (/sim/joint_states: what the physics does), lift_controller, and the two
                            emulators left_arm_emulator and right_arm_emulator, each on its own interface
  root namespace: what would run on the robot
    robot_state_publisher   the combined model at hardware_profile:=real, unchanged, with the two vcan interface names
    ros2_control_node       /controller_manager with openarm_hardware/OpenArmHW, the unmodified upstream driver
                            (enactic/openarm_ros2 at the commit of config/sil_pins.yaml), and the controller file every
                            profile uses (openamrobot_upperbody_control/config/arm_controllers.yaml)
    spawners                joint_state_broadcaster (/joint_states: what the driver believes), both arm controllers, both
                            gripper controllers
  example_arm_trajectory.py   one trajectory per arm, out and back (example_trajectory:=false to leave it out)

The model is not edited. Two things are set from outside it. The gz_ros2_control plugin element gets a <ros><namespace>
child in the expanded URDF, so the simulator's controller manager is /sim/controller_manager and the root namespace is
free for the real driver's. The CAN interface names reach upstream's macro through urdf/openarm_sil_real.urdf.xacro.

The real driver starts only after both emulators are active. When ros2_control_node activates the hardware, the driver
enables the motors and drives every arm joint to zero (upstream behaviour, see the README). The arms spawn at zero.

Args
  headless            true | false (default false). true runs gz sim -s: no window
  create_vcan         true | false (default true). false: the interfaces must exist already
  left_can_interface  default vcan_arm_l
  right_can_interface default vcan_arm_r
  can_fd              auto | true | false (default auto: true when both interfaces have MTU 72)
  supply_voltage      V, default 25.6. The robot's range is 25.6 to 29.2
  timing_model        true | false (default true). false: the emulators answer at once, with no frame time
  physics_engine      gz-physics plugin (default gz-physics-bullet-featherstone-plugin: it honours the finger mimic)
  world               world .sdf (default worlds/sil_empty.sdf of this package)
  example_trajectory  true | false (default true)
  spawn_z             m (default 0.002, as openamrobot_upperbody_gazebo)

Simulation only. No part of this launch is a safety function, and a result from it is not hardware evidence.
"""
import os
import subprocess
import sys
import tempfile

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = "openamrobot_openarm_emulator"
DESCRIPTION = "openamrobot_upperbody_description"
CONTROL = "openamrobot_upperbody_control"
GAZEBO = "openamrobot_upperbody_gazebo"
MESH_PACKAGES = ("openamrobot_description", "openarm_description", DESCRIPTION)
SIM_NAMESPACE = "sim"
SIM_CONTROLLER_MANAGER = f"/{SIM_NAMESPACE}/controller_manager"
REAL_CONTROLLER_MANAGER = "/controller_manager"
EMULATORS = ("left_arm_emulator", "right_arm_emulator")
REAL_CONTROLLERS = ("left_arm_controller", "right_arm_controller", "left_gripper_controller", "right_gripper_controller")
GZ_PLUGIN_OPEN = '<plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">'
BULLET_FEATHERSTONE_ENGINE = "gz-physics-bullet-featherstone-plugin"
CONTROLLER_MANAGER_TIMEOUT = "120"
SWITCH_TIMEOUT_S = "30"
SERVICE_CALL_TIMEOUT_S = "45"


def run_setup_vcan(interfaces):
    """-> (exit status, output) of scripts/setup_vcan.sh for `interfaces`."""
    script = os.path.join(get_package_share_directory(PACKAGE), "..", "..", "lib", PACKAGE, "setup_vcan.sh")
    done = subprocess.run([os.path.normpath(script), *interfaces], capture_output=True, text=True)
    return done.returncode, (done.stdout + done.stderr).strip()


def interface_mtu(interface, sys_root=None):
    """MTU of a network interface, or None when it does not exist. 72 is CAN FD, 16 is classic CAN.
    OPENARM_EMULATOR_SYS_NET names another directory to read instead of /sys/class/net (tests of this launch)."""
    sys_root = sys_root or os.environ.get("OPENARM_EMULATOR_SYS_NET", "/sys/class/net")
    try:
        with open(os.path.join(sys_root, interface, "mtu")) as f:
            return int(f.read().strip())
    except (OSError, ValueError):
        return None


def decide_can_fd(requested, mtus):
    """-> (can_fd, text). `requested` is auto, true or false; `mtus` the MTU of each interface."""
    capable = all(mtu == 72 for mtu in mtus)
    if requested == "auto":
        return capable, ("CAN FD on: every interface has MTU 72" if capable else
                         f"CAN FD OFF: interface MTUs are {mtus}, so the kernel's vcan carries classic CAN frames only here. "
                         "The driver runs with can_fd false")
    if requested == "true":
        return True, ("CAN FD on (requested)" if capable else
                      f"CAN FD requested but interface MTUs are {mtus}: the driver's FD writes will fail")
    return False, "CAN FD off (requested): classic CAN frames"


def sim_description(urdf):
    """The expanded gazebo-profile URDF with the simulator's controller manager moved into the /sim namespace.
    The plugin element is the combined model's own; it gets one child and nothing else changes."""
    if urdf.count(GZ_PLUGIN_OPEN) != 1:
        raise RuntimeError("the combined model's gz_ros2_control plugin element was not found exactly once: "
                           "ros2_control.xacro changed, and this launch has to follow it")
    return urdf.replace(GZ_PLUGIN_OPEN, f"{GZ_PLUGIN_OPEN}<ros><namespace>/{SIM_NAMESPACE}</namespace></ros>")


def sim_controllers(source, left_can, right_can, supply_voltage, timing_model):
    """The simulator-side controller file with this launch's arguments written in. -> dict."""
    with open(source) as f:
        config = yaml.safe_load(f)
    for name, interface in (("left_arm_emulator", left_can), ("right_arm_emulator", right_can)):
        parameters = config[f"/**/{name}"]["ros__parameters"]
        parameters["can_interface"] = interface
        parameters["supply_voltage"] = float(supply_voltage)
        parameters["timing"]["model"] = bool(timing_model)
    return config


def xacro(path, **mappings):
    command = ["xacro", path, *[f"{key}:={value}" for key, value in mappings.items()]]
    return subprocess.run(command, check=True, capture_output=True, text=True).stdout


def spawner(controllers, controller_manager, sim_time):
    return Node(package="controller_manager", executable="spawner", output="screen",
                arguments=[*controllers, "--controller-manager", controller_manager,
                           "--controller-manager-timeout", CONTROLLER_MANAGER_TIMEOUT,
                           "--switch-timeout", SWITCH_TIMEOUT_S, "--service-call-timeout", SERVICE_CALL_TIMEOUT_S],
                parameters=[{"use_sim_time": sim_time}])


def then(target, next_actions, what):
    """After `target` exits 0, start `next_actions`. Any other exit code stops the launch and says why."""
    def on_exit(event, context):
        if event.returncode == 0:
            return next_actions
        return [LogInfo(msg=f"{what} exited {event.returncode}: the software-in-the-loop stack cannot start"),
                Shutdown(reason=f"{what} failed")]
    return RegisterEventHandler(OnProcessExit(target_action=target, on_exit=on_exit))


def launch_setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    left_can, right_can = value("left_can_interface"), value("right_can_interface")
    headless = value("headless") == "true"
    actions = []

    # 1. The interfaces.
    if value("create_vcan") == "true":
        status, text = run_setup_vcan([left_can, right_can])
        actions.append(LogInfo(msg=f"setup_vcan.sh (exit {status}):\n{text}"))
    mtus = [interface_mtu(left_can), interface_mtu(right_can)]
    if None in mtus:
        return [*actions, LogInfo(msg=f"BLOCKED: {left_can} and {right_can} are needed, MTUs found: {mtus}. "
                                      "Create them with: ros2 run openamrobot_openarm_emulator setup_vcan.sh"),
                Shutdown(reason="no virtual CAN interface")]
    can_fd, text = decide_can_fd(value("can_fd"), mtus)
    actions.append(LogInfo(msg=text))

    # 2. The two descriptions and the simulator-side controller file.
    share = get_package_share_directory(PACKAGE)
    description_share = get_package_share_directory(DESCRIPTION)
    controllers = sim_controllers(os.path.join(share, "config", "sim_controllers.yaml"), left_can, right_can,
                                  value("supply_voltage"), value("timing_model") == "true")
    handle, controllers_file = tempfile.mkstemp(prefix="openarm_sil_controllers_", suffix=".yaml")
    with os.fdopen(handle, "w") as f:
        yaml.safe_dump(controllers, f)
    # sim_dynamics off: the gazebo profile's joint damping (config/sim_dynamics/openarm_v2.yaml) is not used here. This launch drives
    # the joints by effort on 1 ms steps, and the physics steps joint damping explicitly: damping times the step over the link inertia
    # about the joint is 0.35 to 6.6 for these joints (above 2 on four of the eight), and an explicit step is only stable below 2 (README, "Damping and the physics
    # step"). The upstream values assume the reflected rotor inertia, which URDF cannot carry. With them on, the CI run of this
    # launch had an arm that should hold still swinging by more than 1 rad. The position-servo simulation is not affected.
    sim_urdf = sim_description(xacro(os.path.join(description_share, "urdf", "combined_robot.xacro"),
                                     hardware_profile="gazebo", sim_dynamics="off", controllers_file=controllers_file))
    real_urdf = xacro(os.path.join(share, "urdf", "openarm_sil_real.urdf.xacro"), hardware_profile="real",
                      left_can_interface=left_can, right_can_interface=right_can, can_fd="true" if can_fd else "false")

    # 3. The simulator.
    for package in MESH_PACKAGES:
        actions.append(AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(get_package_share_directory(package))))
    gz_args = f"-r {'-s ' if headless else ''}-v 2 --physics-engine {value('physics_engine')} {value('world')}"
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items()))
    actions.append(Node(package="ros_gz_bridge", executable="parameter_bridge", name="clock_bridge", output="screen",
                        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]))

    # 4. The plant side, in /sim.
    actions.append(Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher",
                        namespace=SIM_NAMESPACE, output="screen",
                        parameters=[{"use_sim_time": True, "robot_description": ParameterValue(sim_urdf, value_type=str)}],
                        remappings=[("/tf", f"/{SIM_NAMESPACE}/tf"), ("/tf_static", f"/{SIM_NAMESPACE}/tf_static")]))
    actions.append(Node(package="ros_gz_sim", executable="create", output="screen",
                        arguments=["-name", "openamrobot", "-topic", f"/{SIM_NAMESPACE}/robot_description",
                                   "-x", "0", "-y", "0", "-z", value("spawn_z")]))
    sim_broadcaster = spawner(["joint_state_broadcaster"], SIM_CONTROLLER_MANAGER, True)
    sim_rest = spawner(["lift_controller", *EMULATORS], SIM_CONTROLLER_MANAGER, True)
    actions.append(sim_broadcaster)

    # 5. The real side, in the root namespace. Wall time: the driver sleeps and times out on wall time.
    real_state_publisher = Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher",
                                output="screen", parameters=[{"robot_description": ParameterValue(real_urdf, value_type=str)}])
    real_control = Node(package="controller_manager", executable="ros2_control_node", output="screen",
                        parameters=[os.path.join(get_package_share_directory(CONTROL), "config", "arm_controllers.yaml")])
    real_broadcaster = spawner(["joint_state_broadcaster"], REAL_CONTROLLER_MANAGER, False)
    real_rest = spawner(list(REAL_CONTROLLERS), REAL_CONTROLLER_MANAGER, False)
    example = Node(package=PACKAGE, executable="example_arm_trajectory.py", output="screen")

    actions.append(then(sim_broadcaster, [sim_rest], "the simulator's joint_state_broadcaster spawner"))
    actions.append(then(sim_rest, [real_state_publisher, real_control, real_broadcaster], "the emulator spawner"))
    actions.append(then(real_broadcaster, [real_rest], "the real driver's joint_state_broadcaster spawner"))
    if value("example_trajectory") == "true":
        actions.append(then(real_rest, [example], "the real driver's arm controller spawner"))

    # 6. Stopping the simulator: openamrobot_upperbody_gazebo's helper, once per launch (its launch file says what was measured).
    gazebo_shell = {"pid": None, "stop_requested": False}

    def remember_the_shell(event, context):
        if event.process_name.startswith("gazebo") and gazebo_shell["pid"] is None:
            gazebo_shell["pid"] = event.pid

    def stop_the_server_once(event, context):
        if gazebo_shell["stop_requested"]:
            return None
        gazebo_shell["stop_requested"] = True
        try:
            os.unlink(controllers_file)
        except OSError:
            pass
        try:
            helper = os.path.join(get_package_share_directory(GAZEBO), "launch", "sim.launch.py")
        except PackageNotFoundError:
            return None
        subprocess.Popen([sys.executable, helper, "--stop-gz-server", str(gazebo_shell["pid"] or 0)],
                         stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        return None

    actions.append(RegisterEventHandler(OnProcessStart(on_start=remember_the_shell)))
    actions.append(RegisterEventHandler(OnShutdown(on_shutdown=stop_the_server_once)))
    return actions


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false", description="true | false. gz sim -s: server only, no window"),
        DeclareLaunchArgument("create_vcan", default_value="true",
                              description="true | false. Run setup_vcan.sh first (needs root or passwordless sudo)"),
        DeclareLaunchArgument("left_can_interface", default_value="vcan_arm_l", description="SocketCAN interface of the left arm"),
        DeclareLaunchArgument("right_can_interface", default_value="vcan_arm_r", description="SocketCAN interface of the right arm"),
        DeclareLaunchArgument("can_fd", default_value="auto", choices=["auto", "true", "false"],
                              description="auto: CAN FD when both interfaces have MTU 72"),
        DeclareLaunchArgument("supply_voltage", default_value="25.6", description="V. Arm supply the emulated motors see"),
        DeclareLaunchArgument("timing_model", default_value="true", description="true | false. Model frame time on the bus"),
        DeclareLaunchArgument("physics_engine", default_value=BULLET_FEATHERSTONE_ENGINE, description="gz-physics plugin"),
        DeclareLaunchArgument("world", default_value=os.path.join(share, "worlds", "sil_empty.sdf"), description="World .sdf"),
        DeclareLaunchArgument("example_trajectory", default_value="true", description="true | false. Run the example trajectory"),
        DeclareLaunchArgument("spawn_z", default_value="0.002", description="m above the ground plane the model is spawned at"),
        OpaqueFunction(function=launch_setup),
    ])
