# openamrobot_upperbody_moveit

MoveIt 2 configuration for the combined OpenAMRobot 2.0 model (A7): the two OpenArm v2.0 arms on option B's lift carriage. The lift
(`lift_joint`, prismatic, 0 to 0.350 m) is a planned joint in the simulated profiles (issue #17): group `lift` with the calibration stops
`L1000`, `L1175`, `L1350` (0, 0.175 and 0.350 m) as named states, 800 N and 0.018 m/s (DOLD V4, register LIFT) in the URDF of `mock` and
`gazebo`, and `lift_controller`. Group `both_arms` (the two arm groups) plans both arms in one request. The real profile has no command
interface for the lift (the lift node firmware owns it, CTL-017). Everything this package shows is class **`simulation`**: how the model plans and executes under
MoveIt 2.12.4 and **declared OpenAMRobot limits**, on `mock_components` and in Gazebo. It permits no statement about a physical
robot, a payload, a torque, or a validated limit (ADR-009).

```bash
source /opt/ros/jazzy/setup.bash && source ~/openamr_ws/install/setup.bash
ros2 launch openamrobot_upperbody_bringup moveit.launch.py sim:=mock rviz:=true                     # arms on mock_components
ros2 launch openamrobot_upperbody_bringup moveit.launch.py sim:=gazebo rviz:=true \
    physics_engine:=gz-physics-bullet-featherstone-plugin                                           # Gazebo, see "Which engine"
ros2 launch openamrobot_upperbody_moveit move_group.launch.py hardware_profile:=mock                # move_group alone (a stack is running)
```

In RViz's MotionPlanning panel pick a planning group (`left_arm`, `right_arm`, `left_gripper`, `right_gripper`), a named goal state
(`home`, `hands_up`, `park`; `open`, `closed`), Plan, then Execute. The fixed frame is `base_link`; the model's root, and the frame the virtual joint
attaches to `world`, is `base_footprint`. The configuration is built with `moveit_configs_utils`' `MoveItConfigsBuilder` over the combined xacro and
its args (option B's `column`, `base_plate`, `head_pitch`, `base_tilt` and `hardware_profile`), in `openamrobot_upperbody_moveit/moveit_config.py`; nothing is read from upstream's MoveIt
package (ADR-007).

## The SRDF

`config/openamrobot.srdf` has three sections, labelled in the file.

1. **Imported from upstream**, with attribution: the groups `left_arm`, `right_arm`, `left_gripper`, `right_gripper`, the group
   states `home` and `hands_up`, the end effectors, the passive `*_finger_joint2` joints and the arm-internal `disable_collisions`,
   copied from `enactic/openarm_ros2` `openarm_bimanual_moveit_config/config/openarm_v2.0/openarm_bimanual.srdf` at **`4e837e1`**
   (Apache-2.0). One pair is dropped on purpose: `openarm_body_link0` / `openarm_right_link1`, because this model has no OpenArm body
   (ADR-008). `test/test_srdf_import.py` diffs these elements against the checkout at the pinned commit.
2. **Ours:** a `virtual_joint` that is **fixed**, `world -> base_link`; the group state `park`; the gripper states `open` and
   `closed`. The virtual joint is fixed because the base does not move during manipulation (AD-30), and because of W02: KDL has no
   planar joint, so planning against a planar root would be planning against a joint `robot_state_publisher` has already turned
   into a fixed one. Base motion in simulation comes from Gazebo's `odom -> base_link`.
3. **Derived:** the rest of the self-collision matrix, between the `BEGIN`/`END GENERATED ACM` markers (the markers keep their old name): structural
   neighbours (`Adjacent`) and pairs with no movable joint between them that are clear in one evaluation (`ConstantTransform`). Together with the pairs
   upstream declares (section 1) it is **the collision-matrix contract**: this checked-in file is what M03 validates, and its fingerprint is recorded in
   `config/acm/acm_generation.yaml` (a test fails if the matrix changes without the record changing). **Every other pair is enabled.**

### The collision matrix is derived, and a checked-in contract

A pair is disabled for exactly three reasons, each computed from the model or read from upstream, none from a sampler:

| Reason | Meaning | Pairs |
|---|---|---|
| **declared by upstream** | listed in upstream's own SRDF (`enactic/openarm_ros2` @ `4e837e1`); section 1, copied verbatim, diffed against the pinned checkout by `test_srdf_import.py`. All 20 are `Adjacent` and none is a `Never` | 20 |
| **`Adjacent`** | a structural neighbour: both links carry collision geometry, no other geometry-bearing link lies between them on the tree path (frames and geometry-less links do not count), and at most one movable joint does. Contact is governed by the joint's limits and the assembly, not by the matrix | 36 (18 of them upstream's, 16 derived, 2 upstream's only: a gripper's two fingers are siblings through two movable joints) |
| **`ConstantTransform`** | certified by construction: no movable joint between the links, so their relative transform is a constant and **one evaluation settles every configuration**; disabled only if it found them clear | 77 |

That is **113 disabled pairs**; of the 561 pairs of the 34 geometry-bearing links, **448 are enabled**. A pair whose only credential would be "the Setup Assistant
sampled it and saw no collision" is enabled, because sampling falsifies and never certifies, and the asymmetry decides it: a false `Never` hides a real
self-collision from the planner, a missing one costs CPU (measured below). Every constant pair that collides is either `Adjacent` (it overlaps by design) or
reported by the derivation as a permanent collision and not disabled; there are none.

`scripts/derive_acm.py` (`ros2 run openamrobot_upperbody_moveit regenerate_acm.sh`) derives the matrix; `scripts/validate_acm.py` (M03) recomputes the
reasons independently and checks the checked-in file against them. **Regenerating is now stable: it gives the contract back every time**, because nothing
in the derivation is random (`test_acm_generation.py` derives it twice and requires the contract each time). Before P12A the matrix came from the Setup
Assistant's `collisions_updater`, which has no `--seed` option (checked on the installed tool: `--help`, the headers) and gave a different set of `Never`
pairs on every run (473 and 468 at 3000 trials, 455, 456 and 458 at 10000, and a fresh run that differed from the contract in 11 pairs); that is why the
matrix was a contract that could not be reproduced, and why it is no longer sampled. The tool is no longer used.

```bash
ros2 run openamrobot_upperbody_moveit regenerate_acm.sh [--base-plate bp000]          # a PROPOSAL: prints the diff against the contract, changes nothing
ros2 run openamrobot_upperbody_moveit regenerate_acm.sh --write                        # replaces the contract (and the fingerprint): a decision, read the diff
```

The contract (`contract:` in `acm_generation.yaml`): **180 pairs (48 `Adjacent`, 132 `ConstantTransform`), 20 of them declared by upstream**, fingerprint
`sha256:2263a2f658a2d2d3d4f1e93b5e6aed7e4f691b11a276767153eaf10701a3a39d`, re-derived on 10 Nov when the platform pin moved to `713574f` (the merge of
platform-sw #126): camera_link has no collision geometry since platform-sw #106, so its 14 pairs are gone (1 `Adjacent` with base_link, 13 `ConstantTransform`
with the base side's links); none added or relabelled; 180 of 1035 pairs of 46 geometry links disabled, 855 enabled. It was 194 pairs (49 + 145), fingerprint
`sha256:49250d8c4f6313d21adf668c597bd2db6af7aad0b422f9eb92b48bfe50095fcb`, re-derived on 8 Nov (P-03 rev20.4 item 28) when the second E-stop moved onto
the adapter plate (15 static `ConstantTransform` pairs gone, 11 moving ones new; 194 of 1081 pairs disabled, 887 enabled). It was 198 pairs (49 + 149), fingerprint
`sha256:4c464d7634e6b9625e85b66cd256093c3bd1d4dcfe4ffb08f4cb61327c0dfb2b`, re-derived on 1 Nov when the ZED Mini body (head_camera_link) and the
OpenArm mount placeholder (head_camera_bracket_link) became geometry links (19 new pairs: 3 `Adjacent`, 16 `ConstantTransform`; none removed or
relabelled; 198 of 1081 pairs of 47 geometry links disabled, 883 enabled). It was 179 pairs (46 + 133), fingerprint
`sha256:75c1ab9a0d97ec9bd144af9003077d282bf53b8edfec262ef705a9d9a3926331`, derived earlier on 1 Nov when the P-03 item 24 equipment added eight fixed
geometry links (80 new pairs: 8 `Adjacent`, 72 `ConstantTransform`; none removed or relabelled; 179 of 990 pairs of 45 geometry links disabled,
811 enabled). It was 99 pairs (38 + 61), fingerprint `sha256:42b5d25db43a7c285da91c59b4351afea52ac0165062ecd90c3f8920cd0bb18a`, re-derived on 28 Oct (P28) when the adapter plate became a geometry link under the carriage and the arm profile its child
(four new pairs, one relabelled `Adjacent` to `ConstantTransform`); it was 95 pairs (37 + 58) from 16 Oct, when the lift column's two stages got placeholder
boxes (follow-up (a)); it was 83 pairs (38 + 45) in P15 and 113 pairs (36 + 77) with the fixed mast; P15 re-derived it (the model <!-- decision-allow: LIFT historical: the matrix count with the superseded fixed mast -->
changed and the contract is the file, so this is a contract change, in the diff). **The by-construction set shrank from 77 to 45 on purpose:** a
prismatic joint between two links removes them from what a fixed transform certifies, and `lift_joint` now lies between the base and the arms
(`test_acm_derive.py` asserts that the shrink is exactly that: no constant pair spans the lift). 567 of 666 pairs are enabled (535 of 630 until P28, 478 in P15, 448 with the mast).

## Named poses

| Group | State | Values | Status |
|---|---|---|---|
| `left_arm`, `right_arm` | `home` | all seven joints 0 | upstream's, imported |
| `left_arm`, `right_arm` | `hands_up` | joint 4 = 1.5 rad, the rest 0 | upstream's, imported |
| `left_arm`, `right_arm`, `both_arms` | `park` | **`home` with joint 4 = 0.1 rad** | **placeholder**: `home` moved off joint 4's lower limit (issue #17), until Adnan's safe-pose definition lands under the manipulation action contract. It was deliberately given no safer-looking shape: a placeholder that looks deliberate gets trusted |
| `lift` | `L1000` / `L1175` / `L1350` | 0 / 0.175 / 0.350 m | ours (issue #17); the calibration stops (register ARM-MOUNT, CONFIG-IDS); `L1000` and `L1350` are the ends of the stroke |
| `left_gripper` | `open` / `closed` | **+0.7854** / 0 | ours; `open` is the far end of the left finger's own URDF range, `closed` is 0 |
| `right_gripper` | `open` / `closed` | **-0.7854** / 0 | ours; `open` is the far end of the right finger's own URDF range, `closed` is 0 |

`home` has `openarm_*_joint4 = 0`, which is exactly that joint's lower limit. See "Known limitations".

**Margins (issue #17).** `test/test_named_pose_margins.py` lists every named pose and joint with its margin from its URDF limits and fails
below 0.05 rad (revolute joints; the prismatic lift is listed in m). `park`, `hands_up` and `both_arms/park` meet it (smallest 0.1 rad).
Six joint values are still ON a limit, each pinned there by another recorded decision and its check, and listed by name in the test:
upstream's `home` (joint 4 = 0; ADR-007, `test_srdf_import.py`) and the gripper `open` / `closed` states (M05, ADR-009 point 5). Moving
them changes those checks, which is not this package's to decide.

### The two grippers take opposite-signed commands (ADR-009 point 5)

`GripperCommand` positions are `[0, 0.7854]` on the left hand and `[-0.7854, 0]` on the right. An "open the gripper" written as one
number is valid for one hand and out of range for the other, and nothing fails where it is written. So the values above are written
**per hand**, `named_poses.open_value(side, limits)` derives each hand's value from its own URDF limit, and
`test/test_gripper_sign.py` (check **M05**, adopted 7 Oct) fails the single-number version in both directions, by hand, plus a state that
is in range but not the far limit, a missing state and an extra joint. The sign is also exercised live: M01 and M02 open and close
each hand with its own value through the real controllers.

**Which end is which (corrected 14 Nov 2026).** Zero is the closed hand; the far end of each range is the open hand. Until that date the
SRDF and M05 had the two names the other way round (`open` 0, `closed` at the far end), so a `closed` command opened the hand, and M05
passed because it asserted the same convention. The evidence: upstream's docs ("the closed position is used as zero by default"), its
driver (`openarm_can`: "0=closed, 1=open"), its MuJoCo model (ratio 0 closed, 1 fully open at the range end away from 0), and this
model's own geometry: the fingertip collision meshes are 0.1 mm apart at 0 and 139 mm apart at the far end. M05 now measures that gap on
the expanded URDF and fails the swapped names. The motions the tests command are the same as before (to the far end and back to 0);
only the names changed, so tables below that were measured earlier name the far end `closed`. The motor sign per hand on the real
gripper is still not documented upstream and needs a bench check.

## The collision matrix disables only what can be justified (M03)

M03 is `scripts/validate_acm.py`. It recomputes, from the URDF's kinematic tree and the SRDF, the reason every disabled pair is disabled for, and **FAILs**:

* a disabled pair that none of the three reasons covers (a pair disabled because it "never collides");
* a `reason` label that is not true: `Adjacent` on a pair that is neither a structural neighbour nor upstream's, `ConstantTransform` on a pair with a
  movable joint between its links, any other label (a sampled `Never` included);
* a `ConstantTransform` pair that collides in the one evaluation (the claim is that it is clear), and, as a check on the reasoning and not as a
  certification, any disagreement between that evaluation and the first 200 sampled states;
* a constant pair that is **not** disabled and collides: a permanent collision, every state of the model invalid.

**There is no WARN class.** (Until P12A there was one, for "inherited" pairs. The premise was false: upstream declares 20 pairs, all `Adjacent`, and every
one of the 114 "inherited" `Never` pairs was written by our own sampler on our own model, 51 within one arm and 63 left against right with their relative
pose set by our plates. `openarm_left_ee_link1` <-> `openarm_left_link2` was our wrong claim about upstream's geometry.) `Adjacent` and upstream-declared
pairs touch by design and are not claims about collision.

**What sampling does.** The sampled part is **information about the enabled pairs**, not a result and not a reason: N random joint states inside the
limits (numpy `default_rng(seed)`, `lift_joint` held at each calibration stop in turn and the same states at every stop), and the enabled pairs found colliding with how often. Sampling falsifies and never
certifies; a pair absent from the table may still collide. A pair that collides often although the matrix is right tells A4 something about reach and
clearance, which is why it is reported. Collision is checked through `moveit_py` with an allowed-collision matrix of its own that allows everything except the
pairs under test, so the SRDF's list is never consulted; the contact map is not readable from Python (`CollisionResult.contacts` raises "Unregistered type"),
so the colliding pairs are found by bisection.

**Result at the P15 commit** (`dada509`, `evidence/reports/20261013T102944Z_a7.json`): **M03 PASS** at all three lift stops. 83 pairs are disabled (38 `Adjacent`, 45 `ConstantTransform`, 20 declared by upstream) and 478 are enabled; every disabled pair has a reason that can be shown; the 45 constant pairs are clear in the one evaluation that decides them and agree with the first 200 sampled states. Information, not a result: 58 of the 478 enabled pairs collided in some of 2000 random states at seed 20261006 (420 in none), the same at every lift stop, counts included (none of the 58 involves a base-side link, so the lift position cannot change which pairs collide in these states); the commonest are arm links against `arm_profile_link`, option B's 340 mm box (`openarm_right_ee_base_link` 49, `openarm_left_ee_base_link` 37, `openarm_right_ee_link1` 31 of 2000 states). Before P15 it was 37 of 448, mostly against `mast_base_link`.

**The mutation tests** (`test_m03_acm_sampling.py`) plant each defect in a copy of the SRDF and require M03 to name the pair: a disabled pair with no reason
(`openarm_left_link1` <-> `openarm_left_link3` as `Never`), a false `ConstantTransform` label on a pair with arm joints between its links
(`arm_profile_link` <-> `openarm_left_ee_base_link`, which collides in about 2 of 100 random states), a `ConstantTransform` pair that collides
(`base_link` <-> `bl_caster_link`, no joint between them, overlapping by design), and a permanent collision left enabled (the same pair removed from the
matrix). Each pair was measured to behave so with nothing ignored; the P10 mutation never worked because it assumed instead (README, P11).

## M04: a goal inside the nearest reachable volume is refused

`test/test_m04_collision_goal_rejected.py` (VERIFICATION.md's M04 row used to say "base collision volume"; it is corrected to *reachable*,
because the arms cannot reach the base body) plans (plan only) to a left-arm joint state in which a left gripper link collides with
`arm_profile_link`, the nearest structure volume they can reach (the lift column has placeholder collision boxes since `b45d8f6`, so the arms can no longer plan through it, but the boxes are not the supplier's shape and their lateral side is not sourced; this test's goal sits in `arm_profile_link`, not in the column), through `move_group`'s `/plan_kinematic_path`:

| Request | What happened |
|---|---|
| a free named pose (`hands_up`), the control | planned |
| the **joint goal** inside the profile's volume | **rejected**: answered 0.071 s after the control; `move_group`'s log says OMPL found the contact, "Unable to sample any valid states for goal tree", "Invalid goal state", `Planner 'OMPL' failed with error code GOAL_STATE_INVALID` |
| the **pose goal** at the same place | **timed out**: it searched 4.98 s more than the control (the whole budget) looking for a valid goal sample, then failed with the same log verdict |

"Rejected" and "timed out" are different claims and MoveIt 2.12.4 reports both as `FAILURE (99999)` with a planning time of 0 at the
action and at the service, so the check classifies on time spent beyond the control and asserts the planner's own log line for the joint
goal. Only the first is the robot refusing to hurt itself; the second would be a robot that tried until it gave up. Neither produced a plan.

**The arms cannot reach the platform's base body.** In 6000 random left-arm states at the baseline (`lift_joint` at `L1000`, seed 4242) the left
gripper touched `base_link`, the wheels, the casters, the lidar and the camera body in none of them, and M03's all-joint states at the three lift stops
found no collision of any arm link with `base_link`; the left gripper touched `arm_profile_link` in 249 of the 6000. So the refused goal is inside
`arm_profile_link`'s volume, the nearest structure volume a gripper can be put in. That the base body is out of reach is a reach fact for A4, not a
defect here. **The same search found the left gripper in contact with the right arm's own links in about 3 per cent of the random left-arm
states** (`openarm_right_base_link` 73, `openarm_right_link1` 33, `openarm_right_link3` 19 of 6000): the arms are 62 mm apart at their bases (option
B's y = +-0.031), and at the home pose, at all three lift stops and both base plates, no two arm links touch.

## The gripper fork (ADR-009 point 4, manipulation#11): what the installed plugins support

`moveit_controllers.yaml` is **option 1**, `position_controllers/GripperActionController` serving `GripperCommand`, and nothing else changed
(`arm_controllers.yaml` is untouched). The question the ADR left open was whether `moveit_simple_controller_manager`, as installed, can drive
a `ParallelGripperCommand` action at all. It was answered by reading the installed artefacts and then by running it, not from documentation:

* `/opt/ros/jazzy/include/moveit_simple_controller_manager/moveit_simple_controller_manager/` holds `gripper_command_controller_handle.hpp`
  **and `parallel_gripper_command_controller_handle.hpp`**. The second sends a `control_msgs/action/ParallelGripperCommand` goal for one
  `command_joint` (position from the last trajectory point; velocity and effort only when `max_velocity` / `max_effort` are set).
* `strings libmoveit_simple_controller_manager.so` (2.12.4) contains the type strings `ParallelGripperCommand`, `FollowJointTrajectory`
  and `GripperCommand`, the keys `command_joint`, `max_effort`, `max_velocity`, `joints`, `action_ns`, and the messages "Unknown controller
  type" and "ParallelGripperCommand controller only supports commanding a single joint": the manager dispatches the type.
* `ros2 action list -t` (live): `parallel_gripper_action_controller/GripperActionController` serves `control_msgs/action/ParallelGripperCommand`
  at `/<hand>_gripper_controller/gripper_cmd`, the same action name as option 1's `GripperCommand`.
* **Run on mock, in a scratch configuration outside the repository:** the two gripper controllers of `arm_controllers.yaml` switched to
  `parallel_gripper_action_controller/GripperActionController`, and MoveIt's entries to `type: ParallelGripperCommand` with
  `command_joint` set. Planning and executing `closed` and `open` through MoveIt drove both hands, each with its own sign, to within 0.001 rad.
  No handle plugin was needed.

| Option | Facts | Not established |
|---|---|---|
| 1. `position_controllers/GripperActionController` + `GripperCommand` | what this package uses; M01 and M02 drive both hands through it, on mock and in Gazebo (Bullet-Featherstone). Deprecated in Jazzy: a warning, not a removal | |
| 2. `parallel_gripper_action_controller/GripperActionController` + `ParallelGripperCommand` | **the installed MoveIt can drive it** through its own `ParallelGripperCommand` handle, no new plugin; works end to end on mock | Gazebo and hardware. The controller also has `max_effort_interface` / `max_velocity_interface` parameters (they name interfaces it would claim); whether `gz_ros2_control` and `OpenArmHW` provide them was not tested |
| 3. `JointTrajectoryController` on the finger joints + `FollowJointTrajectory` | the `FollowJointTrajectory` handle is what drives our arms, so it exists and works | not run for the fingers in this step |

So ADR-009 point 4's premise, "unknown whether `moveit_simple_controller_manager` can drive it; if not, it costs a controller-handle plugin",
resolves to: **it can, and it costs nothing**, at least on mock. The choice between the three stays with Om and Adnan.

## Limits are ours (ADR-009)

Upstream OpenArm v2.0 declares **no acceleration limits** (`openarm_bimanual_moveit_config/config/openarm_v2.0/joint_limits.yaml` @ `4e837e1`
sets `has_acceleration_limits: false` on every joint), and MoveIt 2.12's time parameterisation rejects a plan without them
(openamrobot-manipulation#10). `config/joint_limits.yaml` is this package's own file: every acceleration entry is tagged
`source: engineering_default`, `status: placeholder`, with the rule `max_acceleration = max_velocity / 2 s` (the URDF's own velocity
limit). It is **not vendor data**, and no payload, torque, inertia or controller analysis stands behind it. Velocity and position limits come from the
URDF (upstream's) and are not repeated.

`scripts/limits_sensitivity.py` (check **M06**, adopted 7 Oct) plans all twelve named poses under the configured limits scaled by 1, 0.1, 0.01
and 0.001 and under no acceleration limits at all. Result: **12 of 12 succeed under every scaling** (planned duration 16 s in total at x1,
517 s at x0.001: the effective limits go down to about 0.003 to 0.01 rad/s²) **and 0 of 12 succeed with none** (which reproduces #10). The plans
need *some* acceleration limit to exist and do not depend on how generous ours are, so the A7 stop condition ("planning succeeds only because
the limits we invented are generous") did not occur. **Both halves of the conclusion, or it is an overclaim:** planning requires that some
acceleration limits exist and is insensitive to our values across three orders of magnitude, which retires that stop condition; **and that does
not make the limits any less of a placeholder**: they stay engineering defaults (`source: engineering_default`, `status: placeholder`), with no
payload, torque or controller analysis behind them. The test fails if either half is missing from the recorded conclusion.

## Which physics engine M02 runs under, and why

`sim:=gazebo` includes the gazebo package's `sim.launch.py`, which **defaults to DART**: the engine platform-sw tuned the base on (decided
6 Oct). On DART the fingers do not follow the position command at the default gain and `finger_joint2` has no mimic constraint, so a
`closed` gripper pose may not close. **M02 passes `physics_engine:=gz-physics-bullet-featherstone-plugin` explicitly**, as G02 does, because
its poses include the grippers and the per-hand sign is part of what it checks; the arm poses do not depend on the engine. The base's contact
behaviour was compared on neither engine. `moveit.launch.py`'s `physics_engine` is empty by default, which means the simulator's default.

## Planning time before and after the matrix change (P12A)

Enabling every pair that has no reason to be disabled costs the planner CPU, and a false `Never` costs nothing and hides a collision, so the cost was measured
(`scripts/planning_time.py`: the twelve named-pose plans, OMPL RRTConnect, 20 repeats after a warm-up round, twice, old SRDF against new, median wall time per plan):

| Named-pose plan (offline, `moveit_py`) | before: 458 sampled `Never` + 36 `Adjacent` | after: 113 justified pairs | after, worst of 40 |
|---|---|---|---|
| left_arm/hands_up (from home) | 0.0210 s | 0.0227 s | 0.0340 s |
| left_arm/home (from hands_up) | 0.0220 s | 0.0230 s | 0.0380 s |
| left_arm/hands_up (from home) | 0.0212 s | 0.0250 s | 0.0330 s |
| left_arm/park (from hands_up) | 0.0225 s | 0.0255 s | 0.0460 s |
| right_arm/hands_up (from home) | 0.0210 s | 0.0245 s | 0.0380 s |
| right_arm/home (from hands_up) | 0.0222 s | 0.0262 s | 0.0380 s |
| right_arm/hands_up (from home) | 0.0215 s | 0.0243 s | 0.0390 s |
| right_arm/park (from hands_up) | 0.0230 s | 0.0253 s | 0.0400 s |
| left_gripper/closed (from open) | 0.0612 s | 0.0650 s | 0.0760 s |
| left_gripper/open (from closed) | 0.0620 s | 0.0622 s | 0.0750 s |
| right_gripper/closed (from open) | 0.0628 s | 0.0622 s | 0.0800 s |
| right_gripper/open (from closed) | 0.0630 s | 0.0625 s | 0.0790 s |

The whole sequence took a median of 0.418 s and 0.433 s (two rounds) before and 0.455 s and 0.455 s after: about **5 to 9 per cent longer, roughly 3 ms per plan**, and every
one of the 40 plans per pose succeeded both before and after. No named pose that planned before fails now. The real stack agrees: M01 and M02 plan and execute all twelve poses
(20 poses over five runs, none failed; M01's wall time per pose is unchanged at about 2.2 s; M02's, which includes execution in a simulator running below real time, was 2.7 to 2.9 s before and 3.3 to 3.4 s
in a re-measurement, which the planning figures above do not explain and which was not separated from the simulator's load). MoveIt's action reports a planning time of 0, and `moveit_py`'s plan result carries no
collision-check count, so none is reported.

## The joint-state clamp: simulator float noise, and nothing else

MoveIt 2.12.4 refuses a request whose start state is any amount outside a joint's limits. The `CheckStartStateBounds` adapter is strict
("Joint 'openarm_left_joint4' from the starting state is outside bounds by: [-2.5e-07] should be in the range [0], [2.4435]"), there is no
`FixStartStateBounds` in the installed adapter set, and removing the adapter does not help because OMPL's planning context refuses the same state.
`home` puts joint 4 exactly on its lower limit, and Gazebo reads a joint resting on a limit back a hair past it. **MoveIt answers the generic
`FAILURE` (99999) to this, to the action and to the service alike** (an earlier version of this README, and the retry loop of M01/M02, named
a specific reason code; this path never returns one, and the reason is only in `move_group`'s log).

Of the three ways out the one taken is to clamp the joint state, and only that: widening a limit would invent vendor data (what ADR-009 exists to
forbid), and moving `home` would invent a pose (what `park`'s placeholder comment refuses to do). `scripts/joint_state_clamp.py` subscribes to
`/joint_states` and republishes it on `/joint_states_clamped`, which **only** `move_group` is remapped to (`move_group.launch.py`, argument
`clamp_joint_states`, default `auto`, issue #18: on for the simulated profiles `mock` and `gazebo`, off for any other profile, where a
reading past a limit is real and the clamp could hide it; an explicit `true` / `false` wins; the raw topic everything else reads is
untouched, and M07 reads the raw `/arm/joint_states`). Its logic, and the reasons, are in
`openamrobot_upperbody_moveit/joint_state_clamp.py`:

| | |
|---|---|
| **Epsilon** | `EPSILON = 1e-5` rad, a named constant. Measured in Gazebo here: `openarm_left_joint4` read back up to **9.9e-7** rad past its limit from a cold start (2585 of 3356 messages), 8.0e-7 at idle, `openarm_right_joint4` 2.6e-7 after a pose sequence; earlier sessions saw 2.5e-7 and 9e-8. A 1e-6 epsilon would clear the worst of those by one per cent, so it is ten times the largest value seen |
| **Clamps TO the limit** | a reading past a limit by at most epsilon is published *at* the limit, never beyond it |
| **Beyond epsilon is not noise** | a reading further out is published unchanged, logged as an error, counted as `over_epsilon`, and the request is still refused (`test_joint_state_clamp_loop.py` shows it); the clamp does not hide a real drift |
| **Logs** | every *episode* of clamping (a joint resting on its limit is one episode of many activations) is logged at WARN with the joint and the magnitude; each activation is logged at DEBUG and counted |
| **Counts and what gates** | `~/get_stats` returns `activations`, `episodes`, `max_magnitude_rad`, `over_epsilon`, per joint. **M02 gates on exactly two of these numbers** (P17): `over_epsilon` must be 0 and the largest overshoot magnitude (`max_magnitude_rad`, or `max_over_epsilon_rad` if larger) must be below the epsilon. `activations` and `episodes` are **diagnostic only**: they scatter 2-3x on an unchanged tree (646-1054 right-finger activations in one configuration, P16), so they are reported under `diagnostic_only`, labelled, and gate nothing. M01 and M02 put them in their sidecars and the A7 report sums them in `clamp` |

A deterministic reproduction needs no Gazebo: `test_joint_state_clamp_loop.py` holds `openarm_left_joint4` at -2.5e-7 and shows the request
refused without the clamp (`clamp:=false`) and planning with it. The start gate of M01/M02 (wait until the joints are inside their limits for a
second) stays, widened by the epsilon: it waits out spawn transients, which the clamp does not touch.

**The retry loop is gone.** M01 and M02 used to retry a refused start state up to three times and count the retries. It never fired in the P10
record (0 retries at every lift stop and in Gazebo), it was keyed on a code MoveIt does not return here, and with the clamp a retry that never fires
is dead code and one that fires says something else is wrong. A refused request is now a failed pose.

## Known limitations

* **W01:** `base_link` carries a 4.18 kg inertial from platform-sw and KDL drops the inertia of a root link, so **MoveIt never sees it**
  (the planning model logs "The root link base_link has an inertia specified in the URDF, but KDL does not support a root link with an
  inertia"). It matters for nothing planned here (the base does not move), and it is not fixed: the workaround changes the contract's root
  and the base is platform-sw's (ADR-005).
* **`park` is a placeholder** (`home` with joint 4 off its limit), until the safe-pose definition lands.
* **`home` and the gripper states are still on a joint limit** (above, "Margins"): issue #17's 0.05 rad is not met for them.
* **The acceleration limits are our engineering defaults**, not vendor data (above).
* **The gripper fork is open** on manipulation#11; this package is option 1 because it is the combination known to work, and the evidence above
  is for Om and Adnan.
* **The mimic needs `bullet-featherstone`**: `finger_joint2` follows `finger_joint1` only on that engine.
* **The collision matrix is derived and enables most pairs** (above): planning pays for every pair that has no reason to be disabled. The cost is measured in
  "Planning time before and after" below. The sampled information about enabled pairs says nothing about whether a pair that did not collide can.
* **MoveIt's start-state check has no tolerance, so the simulated joint state is clamped** (next section). `home` sets `openarm_*_joint4 = 0`,
  exactly that joint's lower limit, which is upstream's pose and is itself reportable upstream: it breaks planning from `home` for any
  float-valued state source.
* **`move_group` exits with SIGSEGV when it is stopped** (MoveIt 2.12.4, observed on every stop, with and without a test around it). The
  launch tests allow exit code -11 for that one process and no other.
* **The wheel joints are not under `ros2_control`**, so on `mock` nothing reports them and MoveIt would warn every second that the robot's
  state is incomplete; `moveit.launch.py` starts a stub that reports them at rest. In Gazebo they come from the simulator.
* **`move_group` does not publish `robot_description`** (`robot_state_publisher` does). Two publishers raced, and `ros_gz_sim` spawned the
  model from `move_group`'s copy, which has no `controllers_file`, so the simulator's controller manager never started ("found an empty
  parameters file").
* `moveit_py` segfaults when its objects are torn down; M03 and M06 run it in subprocesses that leave with `os._exit` after writing their result.

## Tests

`colcon test --packages-select openamrobot_upperbody_moveit`, or `ros2 run openamrobot_upperbody_bringup run_a7_regression.sh` for the
whole A7 report.

| Test | Check | Proves |
|---|---|---|
| `test_srdf.py` | AUX | names exist at every base plate; groups, virtual joint (fixed, not planar, `world` to `base_footprint`), poses inside the limits; `park` is `home` with joint 4 at 0.1 rad and says it is a placeholder; the `lift` and `both_arms` groups and states; each pair disabled once |
| `test_srdf_import.py` | AUX | the imported elements equal upstream's at the pinned commit (SKIP, with the reason, only if there is no checkout) |
| `test_acm_generation.py` | AUX | the matrix's fingerprint is the one on record; the record says derived, no sampling, three reasons; **regenerating twice gives the contract back**; a proposal leaves the contract alone and only `--write` replaces it (on copies) |
| `test_acm_derive.py` | AUX | every disabled pair has a reason that can be shown at every base plate and a true label; only `Adjacent`/`ConstantTransform`; the counts the report states; the by-construction set is 45, not P11's 77, and no constant pair spans the lift joint; the constant pairs have a constant relative pose in an independent forward kinematics |
| `test_config_files.py` | AUX | limit tags and rule, controller names against `arm_controllers.yaml`, option 1 unchanged, OMPL defaults are MoveIt's |
| `test_gripper_sign.py` | M05 | per-hand gripper values; the single-number version fails, in both directions |
| `test_m03_acm_sampling.py` | M03 | every disabled pair has a reason that can be shown at each lift stop; four mutation tests (no reason, false label, constant pair that collides, permanent collision enabled); the sampled enabled pairs are information that never changes the result; the bisection |
| `test_m04_collision_goal_rejected.py` | M04 | a goal in the nearest reachable volume (`arm_profile_link`) is rejected; the pose goal times out |
| `test_m06_limits_sensitivity.py` | M06 | the plans need some acceleration limit and do not depend on its value |

M01 (mock, every lift stop) and M02 (Gazebo, baseline, lift at `L1000`) are tests of `openamrobot_upperbody_bringup`: they start the stacks, and that package
depends on this one (a test dependency back would be a colcon cycle).

M05 and M06 were proposed by P10 and are adopted in `design/VERIFICATION.md` (7 Oct); the tests follow those rows.

See [`openamrobot_upperbody_description`](../openamrobot_upperbody_description/README.md) and
[`openamrobot_upperbody_bringup`](../openamrobot_upperbody_bringup/README.md).
