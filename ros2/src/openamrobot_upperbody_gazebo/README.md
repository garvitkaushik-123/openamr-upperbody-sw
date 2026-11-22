# openamrobot_upperbody_gazebo

The combined OpenAMRobot 2.0 robot in Gazebo Harmonic: the base on platform-sw's DiffDrive plugin, both arms on
`gz_ros2_control` (ADR-003), and nothing of platform-sw's moved (ADR-005). Evidence from this package is class
`simulation`. It says how the model behaves in this simulator and nothing about a physical robot.

```bash
source /opt/ros/jazzy/setup.bash && source ~/openamr_ws/install/setup.bash
ros2 launch openamrobot_upperbody_gazebo sim.launch.py                    # Gazebo window
ros2 launch openamrobot_upperbody_gazebo sim.launch.py headless:=true     # gz sim -s, no window
ros2 launch openamrobot_upperbody_gazebo sim.launch.py rviz:=true         # RViz beside it (fixed frame odom)
```

| Argument | Default | |
|---|---|---|
| `column`, `base_plate`, `head_pitch`, `base_tilt` | `dold_v4_350`, `bp000`, `hp30`, `bt10` | option B's configuration IDs (its baseline) |
| `world` | `worlds/empty.sdf` (steps of 1/1500 s, see [the controller rate](#joint-dynamics-and-the-controller-rate)) | path to any world; platform-sw's is `$(ros2 pkg prefix --share openamrobot_gazebo)/worlds/walled_world.sdf` |
| `headless` | `false` | `true` runs `gz sim -s` |
| `physics_engine` | `gz-physics-dartsim-plugin` (DART) | applied to whatever world is loaded. DART is the engine platform-sw tuned the base on, and it **cannot move the grippers**: pass `gz-physics-bullet-featherstone-plugin` for that (G02 does). See [the engine](#the-physics-engine-and-the-gripper-mimic) |
| `rviz` | `false` | RViz2 with `config/sim.rviz` |

Start-up takes about a minute on this machine (software rendering, real-time factor about 0.65). The five arm controllers go
`active` in the second half of it.

## What it starts

| Process | Does |
|---|---|
| `gz sim -r [-s]` | the world; `-r` runs it. launch starts it as `/bin/sh -c "ruby .../gz sim ..."` and signals the shell only, so on shutdown the launch itself stops the **server**: once, from a detached helper (SIGINT to the `gz sim` processes, the `/server_control` request as well after 3 s if one is still there, SIGTERM then SIGKILL after 8 s; see below). Left to launch alone the server is orphaned |
| `robot_state_publisher` | the combined description at `hardware_profile:=gazebo`, `use_sim_time` |
| `ros_gz_sim create` | spawns the model from the `robot_description` topic. The model carries platform-sw's DiffDrive and JointStatePublisher plugins, unchanged, and our `gz_ros2_control` plugin, which starts a controller manager inside the simulator |
| `platform_bridge` | platform-sw's `openamrobot_gazebo/config/gz_bridge.yaml`, **by reference** |
| `base_joint_states_bridge` | ours: `config/gz_bridge_base_joint_states.yaml`, the one thing platform-sw's file leaves out |
| spawners | `joint_state_broadcaster`, then the two arm and two gripper controllers |
| `joint_state_publisher` | merges the base's and the arms' joint states into `/joint_states` |

### The bridge

Platform-sw's bridge file fits, so it is used as it is: it bridges the clock, `/cmd_vel`, `/odom`, `/tf`, `/imu`, `/scan`,
`/rgb_image` and `camera/camera_info` under exactly the names and types Nav2 and docking use. Nothing is copied and nothing is
renamed. G01 reads that file at run time and asserts every entry exists in the ROS graph with its type, so it follows
platform-sw's file instead of a copy of it.

### Why the model plugin is in the description and not in the world

`gz_ros2_control`'s `GazeboSimROS2ControlPlugin` can only be attached to a **model**; anywhere else the library refuses to start
(`... is not a model. Please make sure that Gazebo ROS 2 Control is attached to a valid model`, read from the installed
`libgz_ros2_control-system.so`). So it is emitted with the model by the description's `gazebo` profile
(`filename="gz_ros2_control-system"`, `name="gz_ros2_control::GazeboSimROS2ControlPlugin"`, with `arm_controllers.yaml` by
installed path), and this package's world holds only the world systems (physics, user commands, scene broadcaster, sensors).
This corrects the idea that the plugin loads "into the world"; ADR-003's boundary holds: arms on `gz_ros2_control`, base on its
own plugin, in one model, with no conflict.

## Joint dynamics and the controller rate

Since 14 Nov 2026 the simulated OpenArm joints are no longer frictionless.

- **Joint damping and friction.** The `gazebo` profile of the description carries `<dynamics damping friction>` on the eighteen movable
  OpenArm joints, from the simulator dynamics profile (`openamrobot_upperbody_description/config/sim_dynamics/openarm_v2.yaml`, values
  from upstream's MuJoCo model, each with its source). The description's README has the table. Rotor inertia (armature) is **not**
  applied: URDF cannot carry it. `sim_dynamics:=off` on the xacro gives the old frictionless model for comparison; this launch has no
  argument for it.
- **Controller rate.** The launch hands `gz_ros2_control` the control package's `sim_overlay.yaml` after `arm_controllers.yaml`: the
  controller manager runs at upstream's 750 Hz in the simulation (100 Hz before, and still 100 Hz on mock and real).
- **World step.** `gz_ros2_control` updates the controllers on the first physics step at which one controller period has passed, so
  the step has to divide the period. `worlds/empty.sdf` now steps at 1/1500 s (written 0.00066667 s). Measured here by counting
  `joint_state_broadcaster` messages per second of simulated time: 499.5 Hz with 1 ms steps, 743.6 Hz with this step (the most common interval between updates is 1.333 ms; a short count window and lost messages put the count a little under 750). A world with
  1 ms steps, platform-sw's `walled_world.sdf` included, still works and runs the arm controllers at 500 Hz. The finer step costs
  simulation speed: on the machine this was written on the real-time factor went from about 0.5 to about 0.3. The base now steps finer
  than the 1 ms platform-sw tuned its drive on; G01's drive and odometry checks pass on it.
- **What this is not.** The position command interface of `gz_ros2_control` is still a velocity servo (gain 0.1), not the MIT
  impedance law the real arm runs. Damping, friction and the rate bring the plant and the timing closer to upstream's; they do not
  make tracking or contact forces transfer to the robot.

`test/test_sim_control_rate.py` holds the step arithmetic and checks that the launch passes the overlay.

## `/joint_states`: what was observed, and what was chosen

Two things want to publish joint states in this model, and the layout was decided after looking at what each really publishes.

**Observed (Gazebo Harmonic 8.15, `ros_gz_bridge` 1.0.24, `gz_ros2_control` 1.2.20):**

- Platform-sw's `JointStatePublisher` plugin writes the **gz** topic `/joint_states`, scoped by its own `<joint_name>` list to
  `left_wheel_joint` and `right_wheel_joint` and nothing else. It writes on every physics step: about 650 messages a second of
  wall time at this real-time factor. Platform-sw's `gz_bridge.yaml` has that entry commented out, so with platform-sw's bridge
  alone the wheels' joint states never reach ROS; its own launch file publishes zeros for them with `joint_state_publisher`.
- `joint_state_broadcaster` publishes the sixteen arm joints (`openarm_{left,right}_joint1..7` and `finger_joint1`; with the
  finger mimic joints it reports `finger_joint2` as well when declared, which this tree does not) at the controller manager's rate (100 Hz of simulated time when this was observed; 750 Hz since the simulation overlay of 14 Nov 2026).
- **If both are put on `/joint_states`** (tried: the wheels bridged straight onto it, the broadcaster left alone): two publishers;
  in 5 s, 2486 messages carried the two wheels only and 249 carried the sixteen arm joints only, and not one message carried
  both. No joint is written twice, but a consumer sees a stream where ten messages in eleven describe two joints, and
  `robot_state_publisher` is fed partial states at wheel rate.

**Chosen:** no one writes `/joint_states` except a merger.

```
gz JointStatePublisher (wheels) --ros_gz_bridge--> /base/joint_states --+
                                                                        +--> joint_state_publisher --> /joint_states --> robot_state_publisher
gz_ros2_control joint_state_broadcaster -----------> /arm/joint_states -+
```

`joint_state_publisher` (`source_list: [/base/joint_states, /arm/joint_states]`, 50 Hz) is the only publisher of `/joint_states`.
The broadcaster is remapped onto `/arm/joint_states` by its spawner (`--controller-ros-args "-r joint_states:=/arm/joint_states"`),
which leaves `arm_controllers.yaml` untouched. Result, measured by G01: one publisher; 20 names in every message (2 wheels, 16
arm joints, and the 2 `finger_joint2` joints, which `joint_state_publisher` computes from the URDF `<mimic>`), none twice,
none reported by both sources. Cost: one more node, which uses about half a core here because of the wheel stream's rate, and
a 50 Hz sampling of the arms' stream. Nothing of platform-sw's is edited.

## `odom -> base_footprint`: one publisher

`robot_state_publisher` converts `base_planar_joint` to a fixed joint because KDL has no planar joint (check W02), so in the
simulator the base's motion must come from the DiffDrive plugin and never from `robot_state_publisher`. This is what happens,
and G01 asserts it: `odom -> base_footprint` (the DiffDrive plugin's `<child_frame_id>` since platform-sw 935efc7, REP 105: the
odom child is the URDF root so that `base_link` keeps one parent; G01 reads the frame from the description, so it follows platform-sw's
decision) arrives at **50.0 Hz of simulated time** (the plugin's `odom_publish_frequency`), the
stamps are strictly increasing with no repeats (two publishers would give about 100 Hz, or repeated stamps, and would look like
jitter rather than an error), `/tf` has exactly two publishers (`platform_bridge`, `robot_state_publisher`), no other
transform involves the `odom` frame, and `odom` is not a link of the description. The launch uses the default
`enable_planar_base:=false`; with `true` the planar joint would sit above `base_footprint` and that guarantee would need rechecking.

## The physics engine and the gripper mimic

A note on words (14 Nov 2026): the hand starts at 0, which is the **closed** hand, and a goal off zero **opens** it. Earlier text
called such a goal "closing"; it now says "moving" or "reaching the goal". The measurements are unchanged.

Measured with the simulator running (the P06 question: does `finger_joint2` follow `finger_joint1`?). **The launch defaults to DART**
(decided 6 Oct, P10 task 0): it is the engine platform-sw tuned the base's DiffDrive on, a gripper-mimic convenience is not grounds
to re-baseline someone else's physics, and the drive check compared kinematics, not contact. So the limitation is visible, not hidden
behind a default: with the default engine the arms and the base work and the grippers do not move. G02 passes
`physics_engine:=gz-physics-bullet-featherstone-plugin` explicitly and says why; anything that needs a gripper to move does the same.

| Engine | Gripper goal (`GripperCommand`) | `finger_joint2` | Gazebo says |
|---|---|---|---|
| DART (Gazebo Harmonic's default) | **not reached**: `stalled`, both fingers stay at 0 | does not follow | `Attempting to create a mimic constraint for joint [...] but the chosen physics engine does not support mimic constraints` |
| Bullet-Featherstone | reached; both fingers at the goal (0.4 and -0.4 rad) | equals `finger_joint1`, read from the link poses | nothing |

Under DART the finger joints do not respond to the position command at the default gain (an effort command moves them freely,
and arm joints respond to position commands normally). **Tuning finding (P10, a ten-minute check, 6 Oct):** this is a gain, not
a missing capability. `gz_ros2_control`'s position interface is a velocity servo whose gain is the plugin's
`position_proportional_gain` (logged by the `gz_ros_control` node: "The position_proportional_gain has been set to: 0.1";
default 0.1, read from the installed library's strings). Measured on DART with the same gripper goal (0.4 rad, left hand,
finger angle read from Gazebo's link poses):

| `position_proportional_gain` | Left `GripperCommand` goal 0.4 | `finger_joint1` (link poses) |
|---|---|---|
| 0.1 (default) | `stalled`, not reached | 0, exactly |
| 1.0 | `stalled`, not reached | 0, exactly |
| 5.0 | `reached_goal`, controller position 0.392 | tracks the goal but oscillates around it (readings 0.37, 0.40, 0.37 rad) |

The gain was set for the experiment by a `position_proportional_gain` parameter under `/**:` in a scratch copy of the controllers
file handed to the plugin; nothing in the repository sets it. What this says: the position command does move a finger on DART
once the velocity command is large enough, which is the hypothesis P09 left open. What it does not say: **why** the threshold
sits between 1 and 5 (a velocity command too small for DART's solver, or a finger resting on its lower joint limit at 0 that
small velocities cannot leave, were not told apart), whether 5.0 is a good value (the gain is plugin-wide, so it applies to the
arm joints too, which were not tried at it, and the finger oscillated), or anything about `finger_joint2`: with no mimic
constraint on DART it is not driven and drifted freely (0.38 to 0.13 rad over about nine seconds), so the two-finger gripper still
does not move symmetrically on DART and **G02 stays on Bullet-Featherstone**. When this was measured the fingers' joints carried no
`<dynamics>` element in the URDF (no damping or friction was declared; what the SDF conversion or DART adds was not looked at), so
"high damping" was not a property of the description. Since 14 Nov 2026 the `gazebo` profile declares finger damping 0.6 N*m*s/rad and
zero friction; the DART gain table above was not re-measured with it. This is a tuning finding for Kunal, who tuned platform-sw's DART setup; nothing is filed upstream, and
sdformat's missing DART mimic constraint is documented behaviour. What was ruled out earlier (P09): whether `finger_joint2` is
declared in `ros2_control`, whether the URDF `<mimic>` is present, the gripper controller type (a `JointTrajectoryController` on the
finger behaves the same), `hold_joints`, and the left/right side.
Under Bullet-Featherstone the URDF `<mimic>` (turned into an SDF mimic constraint by `sdformat`) is honoured on its own, so:

- **nothing is declared for `finger_joint2` in `ros2_control`.** Upstream's commented-out block cannot be enabled: Jazzy's
  `ros2_control` refuses it ("Activated mimic joints cannot have command interfaces"), and with `<param name="mimic">` and no
  command interface it adds nothing the physics does not already do. This is also why `gz_ros2_control#343`, which upstream cites,
  does not matter here;
- the launch's `physics_engine` argument sets `--physics-engine`, a flag that applies to **any world**, including
  platform-sw's `walled_world.sdf`, which was tried with Bullet-Featherstone (controllers active, base drives, gripper and mimic
  track). Platform-sw's own launch runs DART, as this one now does by default; nothing of theirs changed;
- the gripper controller type (`position_controllers/GripperActionController`) and its `GripperCommand` action are untouched:
  this is a physics-engine setting, not an interface change (ADR-009 point 4 stays with Om and Adnan, P10);
- the right gripper's joint range is `[-0.7854, 0]`, the mirror of the left's, so its goals are negative.

G02 guards this: it passes Bullet-Featherstone explicitly, and launched on DART it fails at the first gripper goal. That failure is
the documented limitation, not a defect. The base was driven on Bullet-Featherstone far enough to
compare distance and yaw with the command (0.2 m/s for 4 s gave 0.759 m, 0.5 rad/s for 3 s gave 1.376 rad, which match the
commanded speed less the plugin's acceleration ramp), and G01 repeats the straight drive. How the base behaves in contact
against DART, which platform-sw tuned for, was not compared. That is for the base's owner, if it matters.

## Tests

`colcon test --packages-select openamrobot_upperbody_gazebo`, or `ros2 run openamrobot_upperbody_description verify_gazebo.sh`,
which also writes the `gazebo` evidence report. Each test starts a headless server in its own ROS domain and Gazebo partition.

| | Budget | Asserts |
|---|---|---|
| G01 | 90 s (about 35 s), default engine (DART) | five controllers `active`; an arm goal completes and tracks while the other arm and the base stay still; platform-sw's bridge topics exist by name and type; `/cmd_vel` moves the base and `/odom` reports it; one publisher of `odom -> base_footprint`; `/joint_states` has one publisher carrying wheels, arms and `lift_joint` once each |
| G02 | 90 s (about 20 s), Bullet-Featherstone passed explicitly | both grippers reach a goal off zero (they open part way), and `finger_joint2` follows `finger_joint1` in the physics (link poses, not `/joint_states`) |

G02 and G01 also assert, after their launch has been torn down, that **no `gz sim` of their own `GZ_PARTITION` is still alive** (P16; M02 and
M07, in the bringup package, ask the same through `stack_healthy(simulator=True)`): the ruby wrapper is what the launch owns and what the process
guard reads, and the server it forks was left running after about 2 of 8 A7 runs with nothing noticing (after G02 in P15; after G01 in the
second evidence pass of P16, where it ran at about three cores until the end of the run and M02 failed a pose). It is a FAIL, never a WARN, and it
keys on the partition the test put in its environment before the launch (every process of the launch inherits it), so a simulator of another run,
another test or the person at the keyboard is never matched. A server that was asked to stop is given `GZ_SERVER_GRACE_S` (10 s) to go; the
verdict is fixed before anything is signalled, and only then is a survivor stopped (SIGTERM, then SIGKILL) so a failed run does not leave three
cores busy under the next one. The finding is in the test's sidecar (`details.gz_server_after_shutdown`). `test/canary_gz_orphan.sh` runs G02
against a copy of this package whose `sim.launch.py` lacks the stop handler and exits 0 only if G02 failed naming the survivor;
`test_launch_guard_gz_orphan.py` (description package) pins the detector on decoy processes. Read the record before relying on it: see
`evidence/reports/README.md` (P16, P17), which says how often the orphan was seen and what was and was not demonstrated.

**How the launch stops the server (P17), and why it is a helper and not a launch action.** What was measured is in the header of
`launch/sim.launch.py`; in short: (1) launch signals the `/bin/sh` that wraps `gz sim`, never the server, so under `launch_testing` nothing of the
launch's reaches the server; (2) the first version asked the server to stop through `/server_control` and, over 18 launches, 2 servers segfaulted
inside libgz-sim8 during that teardown and 2 accepted the request, never began it and ignored SIGTERM (only SIGKILL ends such a server; launch's own
SIGTERM of the shell at 5 s then leaves it orphaned); (3) `OnShutdown` fired twice per launch (the shell's death is a second `Shutdown`) and re-executed
the same `ExecuteProcess`, which raises "executed more than once" and abandons the rest of the shutdown, and on the `ros2 launch` + SIGINT path the
request was never started at all because launch refuses to start an action once `context.is_shutdown` is set. So the handler now runs once per
launch and starts a detached helper (`sim.launch.py` run as a script, which imports nothing of ROS): SIGINT to the `gz sim` processes (what Ctrl-C
does; 24 of 24 launches stopped in 1.3 to 1.5 s), the `/server_control` request as well if one is still there after 3 s, SIGTERM then SIGKILL after
8 s. The servers are followed by pid, not by their place in the process tree, because launch's SIGTERM of the shell reparents them out of it.
**A server that had to be forced is a WARN, check ID W04, in the A7 report** (P18; it was a FAIL in P17): the helper writes its report
(`gz_stop_<partition>.json` in the evidence directory) before it sends SIGTERM, `launch_guard.gz_server_shutdown_report` records it under `forced`,
and `build_a7_evidence.py` turns the sidecars' reports into W04. **The survivor check is unchanged and runs after the force-stop**: it waits up to
10 s for the run's `gz sim` processes to go, longer than the helper's 9 s, and any that is still running then is a survivor and a FAIL of the
launch test that owns it, a server that survived its own force-stop included. Nothing is allow-listed. What was **not** found:
why a server hangs in its teardown after being asked to stop; that is inside gz-sim and gz_ros2_control, not in this package. `test_sim_launch_shutdown.py`
pins the helper on decoy processes (including a server that outlives its shell, and a hung server that ignores SIGTERM).

Mutations checked: a second `odom -> base_footprint` publisher and a second writer on `/joint_states` fail G01; the DART engine
fails G02; a launch without the stop handler fails G02 (the canary above).

## Notes for whoever runs this

- A stray `robot_state_publisher` from another session on the same ROS domain shares a node name with this launch's and
  answers `ros2 node list` and `/tf`. Use a separate `ROS_DOMAIN_ID` (and `GZ_PARTITION` for Gazebo) when something else is
  running; the tests choose their own.
- Stopping the launch with a signal to the `ros2 launch` process only (not Ctrl-C in its terminal) leaves the simulator
  running unless the shutdown handler could reach it; `pgrep -af 'gz sim'` shows what is left.
- The model is spawned at its resting height, 0.002 m (the drive wheels' lowest point below `base_footprint`), so it is placed, not dropped.
  `spawn_z:=<m>` overrides it for diagnosis only. At the old 0.15 m the landing pushed `lift_joint` past its lower stop before
  `lift_controller` was active and failed A7 M02 (upperbody-sw #48); `a7_lift_settle_probe.py` in the bringup package records that.
- `ros2 action send_goal` is slow while a simulator is running; read results against `/clock`, as the tests do.

## Not here

The MoveIt configuration, its limits and the gripper evidence are in [`openamrobot_upperbody_moveit`](../openamrobot_upperbody_moveit/README.md);
`ros2 launch openamrobot_upperbody_bringup moveit.launch.py sim:=gazebo` starts this simulator and `move_group` together, and M02 (named poses
in Gazebo at the baseline, lift at `L1000`) runs on the **Bullet-Featherstone** engine for the reason given above. Platform-sw's launch, bridge, world and plugins
are read and used, never edited.
