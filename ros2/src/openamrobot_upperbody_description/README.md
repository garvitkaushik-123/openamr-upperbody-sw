# openamrobot_upperbody_description

The OpenAMRobot 2.0 robot description, in two layers that do not edit each other:

- **option B**, Alex's upper-body model (P-03 rev18.9 item 18, rev19.0 item 19), merged on `main`: the lift column on a two-position
  base plate, a prismatic `lift_joint`, the lift carriage, the arm profile, the OpenArm mount points and the head camera.
  `urdf/openamrobot_upperbody.xacro` (the macro), `urdf/openamrobot_upperbody.urdf.xacro` (its standalone wrapper),
  `config/upperbody_configuration.yaml` (the one source of the configuration IDs, the baseline and the geometry) and
  `launch/upperbody_description.launch.py` are his, and `test/test_upperbody_*.py` are his tests of them;
- **the combined robot**, ours: `urdf/combined_robot.xacro` composes option B with platform-sw's base and the two upstream OpenArm
  v2.0 arms, and adds the datums, the frame contract, the `ros2_control` profiles, the configuration id and the verification.

> **Evidence class: static + mock.** Everything here is verified by xacro expansion, `check_urdf`, forward kinematics and unit
> tests, plus one headless launch of the mock bringup (L01). Nothing has been launched on hardware; the `gazebo` and `real`
> `ros2_control` blocks are checked by expansion here (the Gazebo ones run in `openamrobot_upperbody_gazebo`). Read
> [Known limitations](#known-limitations) before relying on any number in the model.

## What it is

`urdf/combined_robot.xacro` composes, without editing upstream's files:

```
base_footprint                  platform-sw (openamrobot_description), included unchanged; the root, on the floor (REP 105)
  -> base_link                  0.098 m up, the drive-axle midpoint; the wheels, casters, lidar, base camera, IMU and the six
                                near-field sensor frames (us_*, tof_*) are platform-sw's and hang from it
  -> lift_column_base_link      option B: on the base plate, 0.304 m up, x +0.060 (bp000) or +0.110 (bp050) from base_link
       -> lift_joint            option B: prismatic, 0 .. 0.350 m; real: effort 0 and velocity 0, no interface; mock/gazebo: 800 N, 0.018 m/s, planned (#17)
            -> lift_carriage_link
                 -> arm_profile_link                       option B: HFS6-60120, 340 mm, the one collision body of the structure
                 -> head_camera_mount_link -> head_camera_link -> head_camera_optical_frame     option B (ZED Mini)
                 -> openarm_{left,right}_base_link         x +0.180, y +-0.031, z +0.166 from the carriage (option B's pose)
                      -> openarm_{left,right}_*            two arm calls of the upstream OpenArm macros, one per arm
                           -> {left,right}_wrist_camera_link -> *_optical_frame     upper_sensors.xacro
                 -> so101_mount_link                       only with arm_package:=so101 (stub)
world -> base_planar_joint -> base_footprint               only with enable_planar_base:=true
```

The default expansion has 57 links and one root, `base_footprint`. There is no OpenArm body (ADR-008) and there is no mast: the
shoulder height is the **position of `lift_joint`**, `1.000 m + lift` from the floor (0.304 base plate + 0.530 retracted column +
0.166 mount), so 1.000 m at lift 0 and 1.350 m at the 0.350 m stroke. Upstream `openarm_description` is a pinned dependency and
is never modified.

**The lift is state, not configuration.** Its three calibration stops `L1000`, `L1175`, `L1350` (shoulder 1.000, 1.175, 1.350 m;
`lift_joint` 0, 0.175, 0.350 m) are **calibration IDs**: a joint position with a calibration, never an argument of the model and
never an input of the configuration id. ADR-010 had them as configuration IDs and was wrong; option B's own config says so.

### How the arms attach, and the one edit this makes to option B's file

Option B declares `openarm_{left,right}_base_link` and a mount joint of the same name as upstream's arm macro emits
(`openarm_left_base_link_mount_joint`): the arm's base link IS its mount frame. Declaring both is a duplicate link, so the
combined robot has the upstream arm macro emit them, at option B's pose read from `upperbody_configuration.yaml`
(`geometry.arm_mount`), and tells option B's macro not to (`emit_arm_mounts:=false`, an opt-in parameter that defaults to his
behaviour). It adds one more opt-in parameter, `column_inertial`, which carries the lift's catalogue mass on
`lift_column_base_link` (he models no masses). With both at their defaults the macro expands exactly as on `main`, which
`test_t04_optb_composition.py` asserts, and that test also asserts that **every frame of his standalone model is in the combined
robot at the same pose**, at every lift position, base plate and head pitch.

The two upstream calls are the macro that `openarm_v20_robot` itself calls (`openarm_v20_emit_attached_components`), because
`openarm_v20_robot` declares its `root_frame` as a link and option B already declares `lift_carriage_link`. Pinned to the upstream
commit of `config/upstream_pins.yaml` (C02); the preset files keep upstream's zero `connection.origin` and the pose is set in
`combined_robot.xacro`, so there is one source for it.

## Arguments

| Arg | Values | Default | Effect |
|---|---|---|---|
| `column` | `dold_v4_350`, `tl3_400` | `dold_v4_350` | Option B's lift column. `tl3_400` has `null` supplier data and **does not expand** (a configuration that cannot expand, not a placeholder that does). |
| `base_plate` | `bp000`, `bp050` | `bp000` | Option B's two base-plate positions: the column axis at x +0.060 m or +0.110 m from `base_link` (330 mm behind the chassis front edge; not on the drive axle). |
| `head_pitch` | `hp15`, `hp20`, `hp25`, `hp30`, `hp35` | `hp30` | Head-camera pitch down, degrees. |
| `base_tilt` | `bt05`, `bt10`, `bt15` | `bt10` | Base-camera up-tilt, degrees: the same arg platform-sw's base file reads for `base_camera_link`. |
| `arm_package` | `openarm`, `so101`, `none` | `openarm` | `openarm`: two upstream OpenArm v2.0 arms. `so101`: a single `so101_mount_link` stub on `lift_carriage_link`, nothing below it. `none`: base and upper structure only. Option B's mount frames are there with every package. |
| `hardware_profile` | `mock`, `gazebo`, `real` | `mock` | Selects the arm `ros2_control` plugin (ADR-003): `mock` = `mock_components/GenericSystem` and `real` = `openarm_hardware/OpenArmHW`, both through the upstream bimanual macro; `gazebo` = our own block with `gz_ros2_control/GazeboSimSystem`. Part of the configuration id. See [ros2_control](#ros2_control). |
| `enable_planar_base` | `true`, `false` | `false` | `true` adds a `world` link and `base_planar_joint` (type `planar`, axis `0 0 1`) above `base_footprint`. Off by default: base and arm never move together in OpenAMRobot 2.0. |
| `datums_file` | path | `config/datums.yaml` | Alternative datum file, used by the tests. |
| `controllers_file` | absolute path | empty | The controller parameter file the `gazebo` profile hands to `gz_ros2_control`. Empty: no `<parameters>` element. Supplied by the simulation launch; ignored by `mock` and `real`. See [ros2_control](#ros2_control). |
| `controllers_overlay_file` | absolute path | empty | A second controller parameter file for the `gazebo` profile, emitted as a second `<parameters>` after `controllers_file` (a later file wins). The simulation launch passes the control package's `sim_overlay.yaml` (upstream update rate, gripper `max_effort`). Ignored by `mock` and `real`. |
| `sim_dynamics` | `auto`, `on`, `off` | `auto` | Simulator joint dynamics on the OpenArm joints, from `config/sim_dynamics/openarm_v2.yaml`. `auto`: on for `gazebo`, off for `mock` and `real`. `on`: for a MuJoCo-facing URDF (with `hardware_profile:=mock`); **refused with `real`**. `off`: a simulation without them, for comparison. See [Simulator dynamics](#simulator-dynamics). |

The defaults of the first four are option B's `baseline:`; T01 asserts they match. An unknown value for any arg stops expansion
with an error that lists the accepted values. There is no `mast_id`. <!-- decision-allow: CONFIG-IDS historical/negative: mast_id is gone, or the inputs before P15 -->

```bash
xacro $(ros2 pkg prefix openamrobot_upperbody_description)/share/openamrobot_upperbody_description/urdf/combined_robot.xacro \
      base_plate:=bp050 arm_package:=openarm > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

## Build

From a workspace that contains this repo, `openamr-platform-sw` and the pinned upstream `openarm_description`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-skip openarm_hardware openarm
source install/setup.bash
```

`openarm_hardware` and the `openarm` metapackage need the OpenArm CAN library and are not part of the description.
The tests and the evidence report describe the **installed** package, so rebuild before running them.

## View it

```bash
ros2 launch openamrobot_upperbody_bringup view.launch.py base_plate:=bp000
```

The launch file lives in `openamrobot_upperbody_bringup`; the model and the RViz config stay here. It starts `robot_state_publisher`, `joint_state_publisher_gui` and RViz2 (fixed frame `base_link`; the GUI's sliders move `lift_joint` too). Source the workspace
overlay first: the OpenArm and base meshes are `package://` URIs that only resolve through it, and without it RViz loads
the model but draws the arms and base as missing geometry. The launch accepts the four configuration IDs, `arm_package` and
`enable_planar_base`. With `enable_planar_base:=true` the robot cannot be driven from here: `robot_state_publisher` treats
the planar joint as fixed (check W02).

A person looked at the mast version of this model once (P14, V01 item 1); the lift model has not been looked at, and check V01 is a
WARN until someone answers `docs/v01_visual_check.md`.

## ros2_control

`urdf/ros2_control.xacro` defines `openamr_arm_ros2_control(hardware_profile, controllers_file)`, called from `combined_robot.xacro` for
`arm_package:=openarm` only. The mobile base never gets a `<ros2_control>` block (ADR-003). The lift gets one in the simulated profiles only
(issue #17): `openamr_lift_ros2_control(hardware_profile)` gives `lift_joint` its own system (`openamr_lift_mock` on
`mock_components/GenericSystem`, `openamr_lift_gazebo` on `gz_ros2_control/GazeboSimSystem`) with a position command interface, and
`combined_robot.xacro` gives the joint the DOLD V4 limits from the datums (`lift_max_effort_dold_v4_350` 800 N,
`lift_max_velocity_dold_v4_350` 0.018 m/s, register LIFT). With `real` the lift keeps effort 0 velocity 0 and no interface: the lift node
firmware owns it (CTL-017).

| `hardware_profile` | Arm plugin | How |
|---|---|---|
| `mock` | `mock_components/GenericSystem` | upstream `openarm_bimanual_ros2_control`, `use_fake_hardware` true |
| `gazebo` | `gz_ros2_control/GazeboSimSystem` | our block `openamr_arms_gazebo`, the same joints and interfaces |
| `real` | `openarm_hardware/OpenArmHW` | upstream `openarm_bimanual_ros2_control`, `use_fake_hardware` false (CAN-FD) |

`mock` and `real` call the upstream macro as upstream's own top-level v2.0 file does (same gains file, same finger-mimic
file, same prefixes); the CAN interfaces are not passed and take upstream's defaults (`can1` left, `can0` right). T14 and
T16 assert that the two blocks equal that file's expansion. The controllers are in `openamrobot_upperbody_control`
(`config/arm_controllers.yaml`, one file for all three profiles) and are started by
`ros2 launch openamrobot_upperbody_bringup mock.launch.py`.

The `gazebo` profile also carries the Gazebo-side plugin, a **model** plugin (`gz_ros2_control-system`,
`gz_ros2_control::GazeboSimROS2ControlPlugin`). The library refuses to attach anywhere but a model, so it is emitted here with
the model; the world, the launch and the physics engine are in `openamrobot_upperbody_gazebo`. Its `<parameters>` (the
controller parameter file) is the **`controllers_file` xacro arg**, empty by default; with it empty the plugin carries no
`<parameters>` element at all. This description declares the hardware contract and knows nothing about where a controller
config lives (ADR-003), so it neither resolves `openamrobot_upperbody_control` nor needs a dependency on it: the launch that
starts the simulation, which legitimately depends on both packages, supplies the absolute path
(`openamrobot_upperbody_gazebo/launch/sim.launch.py` passes `arm_controllers.yaml`'s installed path). T15 checks the element
appears when the arg is given, that the default expansion is still valid URDF, and the plugin's names against the installed
`gz_ros2_control`.

`openarm_*_finger_joint2` is not declared under `ros2_control`, as upstream leaves it out; the URDF `<mimic>` follows
`finger_joint1`. Decided in P09 by running it: Gazebo's Bullet-Featherstone engine honours that mimic on its own and DART (the launch's default, platform-sw's engine)
does not, and upstream's commented-out declaration cannot load under Jazzy's `ros2_control` ("Activated mimic joints
cannot have command interfaces"). The gazebo package's README has the measurements. The multiplier it follows is read from
upstream's `joint_mimics.yaml`, never written in `ros2_control.xacro` (`test_t15_finger_mimic_multiplier.py` fails on a
literal, including one equal to today's value).

## Simulator dynamics

The canonical model has no joint damping, friction or rotor inertia on the OpenArm joints: upstream's description gives none (its
`friction.yaml` is empty at the pin and at main). A simulator needs them, so they live in one file that is kept apart from the
kinematics, `config/sim_dynamics/openarm_v2.yaml`, and reach only the simulated outputs.

| Joints | Motor | Damping N*m*s/rad | Friction N*m | Armature kg*m^2 |
|---|---|---|---|---|
| `openarm_{left,right}_joint1`, `joint2` | DM-J8009P-2EC | 1.0 | 0.2 | 0.016 |
| `openarm_{left,right}_joint3`, `joint4` | DM4340 series | 0.9 | 0.1 | 0.030 |
| `openarm_{left,right}_joint5` to `joint7` | DM-J4310-2EC | 0.9 | 0.04 | 0.002 |
| `openarm_{left,right}_finger_joint1`, `finger_joint2` | DM-J4310-2EC (one per hand) | 0.6 | 0 | 0.002 |

- **Where the values come from.** Upstream's MuJoCo model of OpenArm 2.0 (`enactic/openarm_mujoco` at `1daab06`,
  `v2/openarm_bimanual.xml`, Apache-2.0), read on 14 Nov 2026. The file records the commit, the place in the source and the status
  of every value, and a schema and a test refuse a value without them. Upstream describes its armatures as motor rotor readouts
  times gear ratio squared, with the gearbox contribution unconfirmed. **None is a measurement of a delivered arm.** They replace
  "no dynamics at all", and system identification on the purchased arms replaces them.
- **What is applied.** `urdf/sim_dynamics.xacro` adds `<dynamics damping friction>` to the eighteen movable OpenArm joints (the
  sixteen controlled ones and the two mimic followers). It does this by redefining the one upstream macro that emits every
  joint's `<limit>`, with the same `<limit>` and the `<dynamics>` after it; the file's header says why and what a pin move must
  re-check. Nothing else in the expansion changes: with those eighteen lines removed the text equals the expansion without them.
- **What is not applied.** Armature: URDF has no element for it, and nothing is added to a link's inertia. Gazebo Harmonic
  therefore runs without reflected rotor inertia. A MuJoCo model built from the MuJoCo-facing URDF must set joint `armature` from the
  profile; no exporter is in this repository yet. The effort and velocity limits in the profile are copies of the URDF's, for
  reference; the URDF keeps its own and the test compares them.
- **A limit found in CI (15 Nov 2026): position-servo joints only.** In Gazebo the damping is stepped explicitly. Without the rotor
  inertia, damping times a 1 ms step over the link inertia about the joint is up to 6.6, and an explicit step is only stable below 2.
  The position interface of `gz_ros2_control` drives the joint by a velocity constraint and is not affected (A7 passes). A joint driven
  by effort is: the software-in-the-loop launch (`openamrobot_openarm_emulator`) had an arm swinging with the damping on, so it
  expands the model with `sim_dynamics:=off`. With the armature the same ratio would be at most 0.42.
- **Where it applies.** `hardware_profile:=gazebo` (by default), and `sim_dynamics:=on` for a MuJoCo-facing URDF. **Never the real
  profile**: `hardware_profile:=real` has no `<dynamics>` on any OpenArm joint and its expansion is the same text as before, and
  `sim_dynamics:=on` with `real` stops the expansion. `mock` has none by default (it has no physics).
- **The gripper.** The profile also records which end is closed (zero) with the evidence, and a grip limit from the motor data:
  3.0 N*m, the rated torque of the gripper motor, with its 2.5 A rated current. See the control package's README for how far that
  limit is applied.

```bash
xacro .../urdf/combined_robot.xacro hardware_profile:=gazebo > /tmp/gazebo.urdf                  # <dynamics> on the OpenArm joints
xacro .../urdf/combined_robot.xacro hardware_profile:=mock sim_dynamics:=on > /tmp/mujoco.urdf   # the same joints, for MuJoCo
xacro .../urdf/combined_robot.xacro hardware_profile:=real > /tmp/real.urdf                      # none
```

`test/test_sim_dynamics_profile.py` checks the file (schema, a listed source and a locator for every value, a method for every
estimate, full commit hashes, the pins equal `upstream_pins.yaml`, the joint list equals the model's, the limit copies equal the
URDF's, upstream's `friction.yaml` is still empty). `test/test_sim_dynamics_urdf.py` checks the expansion (all 16 controlled joints
and the two followers carry the profile's values in `gazebo`; none in `real`; nothing else moved).

## Add a configuration ID

The IDs are option B's: add a value under `configuration_axes` in `config/upperbody_configuration.yaml` (and its data), and
everything here reads it: the accepted-value check in `combined_robot.xacro`, the expansion matrix, the tests and the
configuration ids. A value whose supplier data is `null` is accepted as an ID and refused at expansion, as `tl3_400` is. Change
that file only with its owner (Alex): it is the single source of the IDs for his tests and for ours, and it is the hash input
set of his declaration.

## Add an arm package

`arm_package` selects a model subtree and nothing more. It does **not** define what an arm can do: capability semantics,
the Device Package schema and its interfaces are the Manipulation team's I2 work (Om Mangesh Jagtap, Adnan Khalid). This
package only decides which links and joints are attached to the lift carriage.

To add a package:

1. Add the value to the accepted list and a new `xacro:if` block in `urdf/combined_robot.xacro`, attached to
   `lift_carriage_link` (or to `openarm_{left,right}_base_link` if it uses the OpenArm mount points).
2. Add the frames to `config/frame_contract.yaml` with `req: C` and `condition: "arm_package == <value>"`. T04 fails on
   any link in our namespaces that the contract does not list.
3. Add the value to `ARM_PACKAGES` in `scripts/evidence_common.py` (`test/conftest.py` reads it from there), so T01, T02 and the
   configuration ids cover it.
4. Every number the new subtree needs goes in `config/datums.yaml` (below), not in the xacro.

The `so101` stub is the model: a link, a fixed joint, and a note in the frame contract that A9 owns everything below it.

## Datums

Every number in the model that is not owned by upstream OpenArm lives in `config/datums.yaml`, with its provenance.
Computed values (for example the lift's inertia tensor) are not stored; upstream values, option B's geometry included, are read
from their YAML, not copied.

```yaml
datums:
  <name>:
    value: <number | [x, y] | [x, y, z] | {xyz: [..], rpy: [..]} | null>
    source: <measured | contractor_m01 | f2s | supplier | plan | upstream | derived | placeholder>
    status: <placeholder | provisional | verified>
    ref: "<document, drawing number, file:line, or person + date>"
    owner: "<who can change it>"
    note: "<optional>"
    axis_convention: {x: fore_aft, y: lateral}       # required for a list of two or three dimensions
    density_band: {min, max, reason, bounding_box}   # a mass of ours: the density it is expected to imply, declared by its author
```

Rules, enforced by tests T12 and T13 (`config/datums.schema.json` holds the schema):

1. `source` and `status` are mandatory and from the lists above.
2. `status: verified` needs a measured or documented source (`measured`, `contractor_m01`, `f2s`, `supplier`, `plan`)
   and a non-empty `ref`.
3. `source: placeholder` means `status: placeholder`. **Never write a supplier or plan figure as verified if it is not.**
4. Every datum is either referenced by the xacro, listed under `report_only` (reported, never feeds the model), or
   listed under `pending_reference` with the step that will consume it. A datum in none of the three is a failure.
5. Change a value, bump `revision`. The revision goes into every configuration id, and `test_datums_revision.py` fails
   if the content changed against `HEAD~` and the revision did not.
6. A number that only affects how the model is *drawn* (the camera marker cube edge, `openamr_sensor_marker_edge`) is a
   named visual-only constant in the xacro, not a datum. It may appear only inside a `<visual>`.
7. **A `null` value is a hard-fail.** Supplier data that does not exist yet is `value: null` with `source` and `status`
   `placeholder`, and the xacro that reads it stops with `datum <name> is null` instead of expanding with a guess (option B's
   `xacro.fatal` on `tl3_400`, adopted). `lift_mass_tl3_400` is the one today.
8. **A list of dimensions declares its axis convention** (`x: fore_aft`, `y: lateral`, `z: vertical`), and T12 asserts the
   assignment against an expansion: a pair whose axes are only a note in prose is how the mast profile was probably rotated
   90 degrees for six steps (V01-F1, confirmed by Alex 12 Oct).
9. **A mass of ours declares a `density_band`**, and T10 reports its implied density (mass over its bounding box) and WARNs
   outside the band, placeholders included. The band is per datum and the check is scoped to our own masses: upstream's
   inertials are not ours to judge (arm `link2` implies 297 kg/m3 over its bounding box, which is mostly air).

Current state (revision `2026-10-28.a`): **16 datums: 4 placeholder, 11 provisional, 1 verified** (the arithmetic is in the P28 CHANGELOG entry). Four of them are
placeholders: `lift_mass_tl3_400` (null, hard-fail), the two wrist-camera poses, and `joint_rest_margin_rad` (M07's margin, report-only; it feeds no
model). `lift_column_section` was a fifth until P28: the DOLD datasheet gives it, 240 x 60 mm, and the lift's lateral side is no longer invented.
`ros2 run openamrobot_upperbody_description datums_report.py` lists them with their owners.

## Verify

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run openamrobot_upperbody_description verify_a6.sh
```

`verify_a6.sh` runs this package's `colcon test` (and that of `openamrobot_upperbody_control` and `openamrobot_upperbody_bringup`, whose launch test is L01), the `check_urdf` matrix, the datum report and the configuration ids, and
writes `<workspace>/evidence/<UTC timestamp>_dev.json` (`openamr.evidence.v1`, evidence class `static+mock`). Set
`OPENAMR_EVIDENCE_DIR` to put it elsewhere. **It writes nothing from a dirty tree**: it prints the results, says
`Evidence: NOT WRITTEN` and names the changed files (untracked ones included), because a report records a commit and a tree
with uncommitted changes is not one; every file in `evidence/` is a clean-commit run. The exit status is 1 if any check is
`FAIL`, 3 if all passed but the tree was dirty, 2 if the evidence cannot be assembled. `verify_gazebo.sh` is the Gazebo
counterpart: it runs the launch tests of `openamrobot_upperbody_gazebo` (G01, G02) and writes `<timestamp>_gazebo.json`
(evidence class `simulation`). It needs
`colcon`, `xacro`, `check_urdf` (`liburdfdom-tools`) and `numpy` on the machine; they are declared as test dependencies, so
`rosdep install --from-paths src` installs them.

Result vocabulary: `PASS`, `FAIL`, `WARN`, `SKIP`. **`SKIP` and `WARN` are not `PASS`.**

| Check | What it proves |
|---|---|
| T01 | 14 expansions, one factor at a time around option B's baseline (every `base_plate` x `arm_package` on `mock`, every other `head_pitch` and `base_tilt`, `openarm` on `gazebo` and `real`) expand; there is no mast anywhere; the arg defaults are the baseline; a `null`-data column does not expand |
| T02 | `check_urdf` accepts every T01 expansion, the planar-flag expansions and `openarm` on `gazebo` and `real` at `bp050` (the last two groups are labelled extra) |
| T03 | exactly one root and no duplicate link names |
| T04 | the frame contract: required frames and parents, conditional frames; the near-field frames are **consumed from platform-sw, not declared** (`consumed_from`); and `test_t04_optb_composition.py`: every frame of option B's standalone model is in the combined robot at the same pose, the rev19.0 numbers, `base_link` 0.098 m up |
| T05 | shoulder height per lift stop: `openarm_{left,right}_base_link` is 1.000 / 1.175 / 1.350 m above the floor at `L1000` / `L1175` / `L1350`, within 1 mm, by forward kinematics through the lift joint |
| T06 | an unknown arg value fails with the accepted values named |
| T07 / T08 | collision geometry exists or is declared absent; a collision mesh that is the visual mesh is reported |
| T09 / T10 | every massed link has positive mass and a physically valid inertia; the modelled mass is reported by owner against 120 kg, next to the F2S vehicle estimate (93 kg, `f2s_vehicle_mass_estimate`), with "per-link masses still placeholders" printed while any mass group is `placeholder` in `config/mass_provenance.yaml` (three states, `placeholder` / `external_nominal` / `verified`, over base, lift, openarm and sensors), plus the density plausibility of our masses, and a separate line naming the `external_nominal` groups |
| T11 | no planar joint by default; with `enable_planar_base:=true` a `planar` joint, parent `world`, child `base_footprint` |
| T12 / T13 | datum schema, provenance rules (null is a hard-fail; a dimension list declares and asserts its axes) and references |
| C01 | every configuration id is `sha256:` plus 64 hex characters and matches the C9 dataset schema pattern; every pin equals the `openamrobot-manipulation` baseline pin |
| C02 | every pin in `config/upstream_pins.yaml` is a quoted full 40-hex hash, and equals `HEAD` of its clean checkout under `src/upstream/` when there is one (a pin with no checkout is listed as not compared; SKIP only when no pin has one) |
| I01 | the installed package matches the source tree (catches evidence generated against a stale install) |
| REV | a datum change carries a revision change |
| T14 / T15 / T16 | `ros2_control` per profile, by expansion only: `mock` has `GenericSystem` and no `OpenArmHW`, `gazebo` has `GazeboSimSystem` for the same joints and interfaces plus the model plugin with the controller yaml, `real` has `OpenArmHW` with its CAN parameters; the base has none in any profile; `mock` and `real` equal upstream's own expansion; the finger-mimic multiplier is read from upstream's yaml (T15) |
| L01 | launch test in `openamrobot_upperbody_bringup`: `mock.launch.py` brings all five controllers to `active` within 30 s and a trajectory on `left_arm_controller` moves `/joint_states` to the target |
| E01 | **WARN, never PASS**: height, width and depth as lower bounds, per lift stop |
| W01 | **PASS** since P15: the root is `base_footprint` and carries no inertial, so KDL consumers lose nothing (was a WARN) |
| W02, V01 | **WARN**: see Known limitations |

### Configuration id

`ros2 run openamrobot_upperbody_description print_config_id.py column:=dold_v4_350 base_plate:=bp000 head_pitch:=hp30 base_tilt:=bt10 arm_package:=openarm hardware_profile:=mock`
prints the id of one expansion: `sha256:` followed by the SHA-256 (64 hex characters) of the canonicalised expanded URDF, the
six args, the datum revision and **every** upstream pin (`config/upstream_pins.yaml`). A 12-character prefix is for log lines
only and is labelled as a prefix wherever it is shown. `hardware_profile` is part of the id because the id names the
configuration, not just the geometry.

**What a consumer must keep (C9/C10).** The digest is of a payload, not of a file: the compact, sorted-key JSON object that `config_id()` in
`print_config_id.py` serialises (`json.dumps(members, sort_keys=True, separators=(",", ":"), ensure_ascii=True)`, UTF-8) and hashes, with the members listed in
that script's docstring. **To reproduce or verify a digest with C10, retain that exact payload, not merely the raw URDF/Xacro file**: the raw file is only one
member of it (as C14N 2.0 canonical XML, comments and whitespace-only text dropped), and the same file hashed alone, or inside a differently serialised object,
gives a different digest that still looks like an id. (Satya's review of #14.) The lift stops (L1000 / L1175 / L1350) are calibration IDs, not configuration
axes, and are not members of the payload; this matches ADR-010 as corrected by rev19.0 (Satya's review of #14 confirms it).

| | Inputs |
|---|---|
| before P15 | canonical URDF, `mast_id`, `arm_package`, `hardware_profile`, datums revision, `openarm_description` commit <!-- decision-allow: CONFIG-IDS historical/negative: mast_id is gone, or the inputs before P15 --> |
| now | canonical URDF, `column`, `base_plate`, `head_pitch`, `base_tilt`, `arm_package`, `hardware_profile`, datums revision, **all** upstream pins |

The format is unchanged and still validates against C9, **so nothing fails to say that the meaning changed: an id recorded before
P15 is not comparable with one computed now.** The lift stop is not an input (a calibration ID), and `enable_planar_base` is not a
separate member (it is already in the URDF text). The pins are read from the git checkout of each upstream repository in the
workspace; with no checkout (CI, an installed package without sources) they fall back to `config/upstream_pins.yaml` and the
report says so; a checkout with local changes is an error, never replaced by the pin. A test fails if a pin disagrees with its
checkout (C02) or with the `openamrobot-manipulation` baseline pin (C01), so it moves in the same commit as the checkout does.
Every pin is a full 40-hex hash in quotes: unquoted, YAML reads an all-digit hash as a number, and a prefix cannot be compared
exactly.

## CI

`.github/workflows/model-tests.yml` (workflow **Model tests**) builds the combined model on `ubuntu-24.04` with ROS 2 Jazzy in a workspace of this
repository, `openamr-platform-sw` at the platform pin (`openamr_platform_sw` in `config/upstream_pins.yaml` when present, else the default in the
workflow) and the `openarm_description` and `openarm_ros2` pins of `config/upstream_pins.yaml` under `src/upstream/` (so C02 compares them).
`rosdep` installs the dependencies, test dependencies included, of the packages `colcon build --packages-up-to` needs; the job log and summary
list the ROS, MoveIt and Gazebo package versions. The dependencies' build is cached on the three pins; this repository's packages are always built
fresh.

| Job | When | What it runs | Uploads |
|---|---|---|---|
| `a6` | every pull request, every push to `main` | `verify_a6.sh` headless (60 min limit) | `a6-evidence`: `evidence/<ts>_dev.json`, the test XML |
| `a7` | on demand | Gazebo Harmonic from `rosdep`, headless server with software rendering; `run_a7_regression.sh` once (120 min limit) | `a7-evidence`: `evidence/<ts>_a7.json`, the test XML, the colcon logs |

A job fails if its script exits non-zero (a `FAIL`, evidence that cannot be assembled, a dirty tree) or if no test case executed: nothing
found, or every case skipped (`tools/count_ci_tests.py`). The job summary lists test cases per package as discovered / skipped / executed / failed;
`colcon test-result` prints a larger total because it also counts each registered CTest test once, and the summary prints that sum too. To start A7:
**Actions** tab, **Model tests**, **Run workflow** (pick the branch), or add the label `run-a7` to a pull request (it then runs again on every
push while the label stays). The jobs use no secrets and reach no hardware endpoint.

**CI results are A6 (`static+mock`) or A7 (`simulation`) evidence only**: never hardware, physical acceptance, release readiness or safety
evidence. A7 on a hosted runner has no GPU and runs slower than real time.

## Known limitations

Nothing in this package is a measured value or a hardware result. In particular:

- **Base inertials are platform-sw CAD placeholders.** The base comes from `openamr-platform-sw` unchanged; its masses and
  inertias are CAD-export values. The arm masses are upstream nominal values, and the lift's 11.7 kg is a supplier catalogue
  figure (provisional), not a weighed one.
- **The lift column is a mass on two boxes, the supplier's sections on both stages (P28; lengths from the DOLD STEP, P-03 rev20.4 item 28).** The outer
  (fixed) stage is `lift_column_section`, 240 x 60 mm (x fore-aft, option B's own statement of the 240 mm side, the DOLD Hexalift datasheet page 4,
  "Outer profile 240x60", confirmed by the DOLD supplier CAD `Hexalift_EB530_Hub350_mit_Montagelaschen.STEP` of 8 Nov 2026); the inner (moving) stage is
  `lift_inner_section`, 209.5 x 53.3 x 470 mm (the STEP; the datasheet drawing said 209.5 x 51 and gave no length), strictly smaller in section and
  centred on the column axis. Both are visual and collision boxes: the outer, 0.505 m (to the static top, `lift_static_profile_top_below_column_top`
  0.025 below the retracted top), on `lift_column_base_link`; the inner, 0.470 m, on `lift_carriage_link` with its top at the carriage frame, so its bottom
  is 60 mm above the column base at L1000 (opt-in `column_geometry`, which the combined robot passes). The STEP's foot with mounting tabs
  (280 x 60.4 x 10 mm) is not modelled. The lift is fixed through its four foot tab holes on the 260 x 30 mm pattern into the lift base plate
  MEC-019 (P-03 item 30 A3, superseding item 28); the 224 x 40 mm pattern is internal to the DOLD assembly and not a fixing interface. The outer section also sizes the inertia tensor. **They are boxes, not the
  supplier's shapes**, and the outer collision is solid, so the two stages still overlap in volume; the pair is a structural neighbour, so the collision
  matrix does not evaluate it. The adapter plate (one 6082-T6 part, 425 x 100 x 10 mm, `adapter_plate_section`, P-03 rev20.4 item 28; it was 360 mm,
  P-03 rev19.6 item 18) is its own link on the carriage, its front edge on the arm profile's front face and its rear edge 65 mm behind the column's rear
  face (`adapter_plate_rear_extension`, for the flag rod), and the arm profile stands on it (`arm_profile_link` is its child; its pose from the carriage
  did not change). The whole catalogue mass sits on the fixed side at mid-height of the retracted column: the catalogue does not split it between the
  fixed part and the moving stage.
- **The base's pose above the floor is a decision record, not a measurement.** `base_link_height_above_floor` = 0.098 and the
  column axis at x +0.060 are P-03 rev19.0 values (provisional); T04 asserts the datum equals platform-sw's joint in the expansion.
- **The arms are 62 mm apart at their bases** (option B's y = +-0.031). They do not touch at the home pose at any lift stop or
  base plate, but the left gripper reaches the right arm's own links in some random left-arm states (M04 / M03 record how often): a
  reach and clearance question for A4 and Alex, and this repository does not move his mounts.
- **The wrist-camera poses are placeholders** (`left/right_wrist_camera_pose`, kit datum unknown). They are marker frames with no
  collision or inertial, which is why they are not hard-fail: that would stop every `openarm` expansion.
- **The ZED wrapper is deferred.** The wrist cameras are plain frames with a marker cube, not sensor models; the head camera is
  option B's frames. The upstream ZED wrapper belongs with camera calibration.
- **E01 is WARN.** Meshes are not loaded, so height, width and depth come from link origins and box, cylinder and sphere
  extents only, at each lift stop. They are lower bounds: a figure under the limit proves nothing, and a figure over it is real.
  The envelope is 1700 x 600 x 800 mm (provisional).
- **W02: `robot_state_publisher` converts the planar joint to fixed**, because KDL has no planar joint. The joint is
  present and `check_urdf` accepts it, which is what A6 asks for, but planar base motion can never come from
  `robot_state_publisher`; in simulation it comes from the Gazebo `odom -> base_footprint` transform.
- **V01: nobody has answered the visual checklist of the lift model.** Numeric forward kinematics is not a visual check. The
  checklist is in `docs/v01_visual_check.md` (repository root), and every item states what to measure; "looks fine" does not
  clear it. P14's look at the mast found two defects, both closed by this model (see the file).
- **`ros2_control` runs on `mock` (L01) and on `gazebo` (G01, G02, in `openamrobot_upperbody_gazebo`); `real` is an
  expansion that nothing has loaded:** `openarm_hardware` cannot be built here (it needs OpenArmCAN). Mock hardware reflects
  its command back as state, so L01 shows the controller wiring, not tracking performance. In Gazebo the arms' position
  interface is a velocity servo, so G01's tracking says nothing about the real arm.
- **The simulated lift is a catalogue column.** In mock and Gazebo it follows `lift_controller` under the DOLD V4 no-load speed and push
  rating (issue #17); nothing says how the real column moves under load, and the real profile has no interface for it.
- **No physical evidence.** Static expansion and forward kinematics in this package's report; a simulator in the Gazebo report.
