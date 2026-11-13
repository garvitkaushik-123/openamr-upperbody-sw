# openamrobot_upperbody_control

`ros2_control` configuration for the two OpenArm arms. One file, `config/arm_controllers.yaml`, serves every hardware
profile (`mock`, `gazebo`, `real`; ADR-003): the plugin differs, the controllers do not.

| Controller | Type | Joints |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | all |
| `left_arm_controller`, `right_arm_controller` | `joint_trajectory_controller/JointTrajectoryController` | `openarm_{left,right}_joint1..7`, position command, position and velocity state |
| `left_gripper_controller`, `right_gripper_controller` | `position_controllers/GripperActionController` | `openarm_{left,right}_finger_joint1` |

The controller manager runs at 100 Hz. The file is derived from `enactic/openarm_ros2` (`ros2_controllers.yaml` at
`4e837e1`, Apache-2.0); the header lists the three changes. The one that matters: the Jazzy `GripperActionController`
reads `joint:` (a string), and upstream's `joints:` list makes it fail to configure with "Joint name cannot be empty".

- **The mobile base is not under `ros2_control`.** The base keeps its existing drive plugin and its `/cmd_vel` and `/odom`
  interface; this package must not touch them.
- **`lift_controller` is for the simulated profiles only** (issue #17). In `mock` and `gazebo` `lift_joint` has a position command
  interface and `lift_controller` (a JointTrajectoryController) follows MoveIt's lift plans. On `real` the lift is option B's prismatic
  joint with `effort="0" velocity="0"` and no `ros2_control` interface, so this controller cannot load there and nothing spawns it: the lift's
  driver and interlocks belong to the lift node firmware (CTL-017) and the work packages that own it.

## The simulated lift guard (issue #44)

`scripts/lift_guard.py` keeps `lift_controller` inactive unless both arms are stowed and the base is stopped, mirroring the robot's
lift interlock (CTL-017) in `mock` and `gazebo`. It does not replace that interlock and is not safety evidence. The decision is
`openamrobot_upperbody_control/lift_guard.py`, unit-tested without ROS in `test/test_lift_guard_logic.py`; its docstring lists the
blocking codes. Status: `/lift_guard/status` (`diagnostic_msgs/DiagnosticStatus`): OK (the lift may move), WARN with `blocking`
empty and `lift_controller: activating` (switching on, at most `switch_timeout_s`), ERROR (blocked, always with a code). On SIGINT or
SIGTERM the node deactivates `lift_controller` before it exits; the launch's exit handler covers SIGKILL. The thresholds are provisional
parameters until the firmware design sets them.

Wiring (`openamrobot_upperbody_control/lift_guard_launch.py`, used by `mock.launch.py` and `sim.launch.py`): `lift_controller` is
spawned `--inactive`; the guard runs with an `OnProcessExit` handler that deactivates `lift_controller` (SIGKILL, crash);
`park_arms.py` moves both arms to `park` once at start-up; in mock `lift_guard_odom_stub.py` publishes `/odom`.
`wait_for_lift_guard.py` sends the start-up goal of `moveit.launch.py lift_position:=X` once the guard reports OK. Every helper exits 0
on SIGINT/SIGTERM (`ros_exit.py`), as every process of a launch must at teardown, and ends with `os._exit` without destroying its node:
destroying the DDS participant can deadlock in rmw_fastrtps and the launch then SIGKILLs the process (`ros_exit.py` has the evidence).
`park_arms.py` counts a park goal that a newer goal replaced as not failed. The real profile has no guard.

### Known limits and open decisions (review on #44)

What the simulated guard implements is not the agreed robot behaviour, and its numbers are not robot limits:

- **Base-stopped thresholds are provisional.** The defaults are 0.01 m/s, 0.01 rad/s, held 0.5 s. The firmware proposal (D14) is
  0.01 m/s and 0.02 rad/s for 0.5 s. Neither set is authoritative until `openamr-platform-fw#20` decides; then the defaults follow it.
- **Only `park` counts as stowed**, as a conservative interim choice. The robot rule is stowed or carry, but there is no `carry`
  group state in the SRDF yet. When it is added, it goes into `stowed_states`. Tests should not assume that only `park` is stowed.
- **A hung guard is not covered.** If the process is alive but stops evaluating, `lift_controller` stays as it was. The robot stops on
  a missing heartbeat (D12); the simulation has no heartbeat. The launch's exit handler covers a guard that dies, not one that hangs.
- **The guard stops the lift only.** It does not stop the base or the arms; on the robot, other arbitration rules cover those.
- **Motion does not resume by itself.** Deactivating `lift_controller` cancels its active goal (JointTrajectoryController: "Current
  goal cancelled during deactivate transition"). When the guard activates it again, the controller holds its position, and the lift
  moves only on a new command.
- **Stopping times and extra travel measured in simulation are measured values**, not accepted limits (D16) and not evidence of the
  physical robot's stopping performance.

## The gripper controller's deprecation warning: do not "fix" it

Every configure of the gripper controllers logs a deprecation warning. It is left in place on purpose.

1. **What is in use.** `position_controllers/GripperActionController`, serving the `control_msgs/action/GripperCommand`
   action, which is the one upstream's MoveIt configuration expects.
2. **The deprecation is a warning, not a removal.** Jazzy 4.42 still ships and loads it.
3. **The replacement is not a rename.** The non-deprecated `parallel_gripper_action_controller/GripperActionController`
   serves `control_msgs/action/ParallelGripperCommand`, a different action type with a different goal. Moving to it
   changes the gripper interface; it does not tidy a package name.
4. **The warning misspells its own replacement** as `parallel_gripper_controllers/GripperActionController`. That name is
   not registered and will not load.
5. **The choice is not ours to make in this package.** It belongs to `openamrobot-manipulation#11` and ADR-009 point 4
   (open with Om and Adnan), and it will be decided in P10, with MoveIt planning to a gripper, which is the only way to
   learn whether `moveit_simple_controller_manager` can drive `ParallelGripperCommand` at all.

Established in the Jazzy 4.42 binaries (`libgripper_action_controller.so` and `libparallel_gripper_action_controller.so`
strings, `ros2 control list_controller_types`) and live (`ros2 action list -t` shows `ParallelGripperCommand` on the
parallel controller), not from documentation. A third candidate, a `JointTrajectoryController` on the finger joints, is
what `openamrobot-manipulation` bring-up provides today; ADR-009 point 4 lists all three.

## Tests

`test/test_arm_controllers_yaml.py` checks, for every profile, that each joint the YAML names is declared under
`<ros2_control>` with the interfaces its controller needs, that every declared joint has exactly one controller, and that
the `finger_joint2` mimic has none. The runtime counterpart is L01 in `openamrobot_upperbody_bringup`.

See [`openamrobot_upperbody_description`](../openamrobot_upperbody_description/README.md) for the `ros2_control` blocks.
