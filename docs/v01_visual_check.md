# V01: look at the robot

V01 has been `WARN` since A6, and it stays `WARN` until a person has looked at the robot on a screen and said what they saw.
Numeric checks are not visual ones: the 80 mm square mast section passed every test we had and was caught by someone looking.
**"Looks fine" does not clear it.** Answer each of the eight items below with what you **measured or counted**, not with how it looked:
every item names the number to read and where to read it from, and the number it should be.

## Status: WARN by design. The owner accepted the visual check on 1 Nov 2026 (last section); the report builders keep V01 WARN while `lift_mass_tl3_400` is null

*1 Nov 2026: Alex Reznichenko, the owner, accepted the visual check on the six configurations of the final model on 1 Nov 2026 (P-03 rev20.1 item 25), at main `58a5cf8`; the six-configuration renders were made on 30 Oct 2026. The record and its limits are the last section, "Recorded: owner acceptance". V01 still reads `WARN` in A6 and A7, by design: the reason is in that section. The notes below are the state before the acceptance.*

*28 Oct 2026 (P28): Alex answered the column geometry on 22 Oct and the geometry items are answered in the last section: three close (the lateral side, the arm profile on its adapter plate, the carriage no longer a copy of the outer box), one does not (the two stages still overlap in volume), and V01 stays `WARN` because `lift_mass_tl3_400` is still null and because nobody has looked at a screen and answered the checklist. Read the P25 text below as the state before that answer.*

*21 Oct 2026 (P25): looking at the renders of the lift model found five more things in the placeholder geometry; one is fixed and four are measured and wait for Alex. They are the last section of this page. V01 stays WARN. The statements below that the column has no collision geometry are the 14 Oct state (see `CHANGELOG.md`, b45d8f6).*

**P14 looked at the mast (then item 1) and found two defects no automated check was examining.** The mast is gone, so that answer
describes an object that no longer exists and is not counted as an answer for the lift model. **The eight items below were measured in P16
(table at the end); V01 is still `WARN`, because `lift_mass_tl3_400` is null.** A closed finding is not an answered checklist, and a check whose value was finding defects is
not recorded as having found nothing.

- **V01-F1, the profile was probably rotated 90 degrees. Confirmed correct by the design owner.** The mast box was 60 mm fore-aft and
  120 mm across the robot; Alex's option B has the same MISUMI HFS6-60120 cross-section 120 mm fore-aft and 60 mm across, the stiff
  direction against the arms' forward cantilever. Alex, 12 Oct: *"thanks for catching the profile orientation, that's exactly the kind
  of check we need"*. The 90 degree rotation was real. The value had been right and its axis assignment was only a prose note in the
  datum that no test asserted; the mast and `mast_section` are deleted, and the rule it pointed to is implemented (a dimension list
  declares `axis_convention` and test T12 asserts the assignment against an expansion).
- **V01-F2, the mast mass was implausible. Closed by the catalogue figure.** `mast_mass` was 3.0 kg (placeholder) for 1.2365 m of that
  profile, about 337 kg/m3 over its bounding box. The lift is a DOLD Hexalift V4 350 and its **catalogue mass is 11.7 kg** (rev19.0,
  12 Oct 2026), which lands at the top of the 7.5 to 12 kg bracket P14's estimate gave (the estimate was honest and it held). The
  rule it pointed to is implemented: a mass of ours declares a `density_band` and test T10 checks the implied density. 11.7 kg is a
  catalogue figure, not a weighing, so the datum is `provisional`.

**Item wording.** P14 found that the old item 1 listed "a thin spike" among the wrong things to see, so its own correct answer was
recorded as wrong. **Every item below states what to measure, not what it should look like**: an aesthetic judgement cannot be
falsified, a reading of 0.834 m can.

## Start the simulator and RViz

    export ROS_DOMAIN_ID=42 GZ_PARTITION=v01          # in every terminal you use below (see the note)
    source /opt/ros/jazzy/setup.bash && source ~/openamr_ws/install/setup.bash
    ros2 launch openamrobot_upperbody_gazebo sim.launch.py rviz:=true

*Note:* if `ros2 node list` already shows a `robot_state_publisher` from an earlier session (the `oa_*` helpers leave one
running), RViz would draw two models and `/tf` would carry two sources. A separate `ROS_DOMAIN_ID` and `GZ_PARTITION` keep
this run to itself.

That opens the Gazebo window and RViz beside it (fixed frame `odom`, RobotModel and TF displays). The first start takes about a
minute; wait until `ros2 control list_controllers` in another terminal shows six controllers `active` (the five arm and gripper ones and
`lift_controller`, issue #17). Since issue #44 `lift_controller` starts inactive and the lift guard activates it once the start-up move
has put both arms in `park`; `/lift_guard/status` says why while it is not.

For items 1 to 4 the lift has to be moved. In the simulator `lift_controller` moves it (plan group `lift` to `L1175` or `L1350` in RViz's
MotionPlanning panel with both arms in `park`, item 8's setup), or use the model viewer, whose `joint_state_publisher_gui` has a slider for
`lift_joint`:

    ros2 launch openamrobot_upperbody_bringup view.launch.py base_plate:=bp000

and read frames with `ros2 run tf2_ros tf2_echo base_footprint <frame>` (translation x, y, z in metres, from the floor).

For item 8 (MoveIt) start MoveIt with RViz instead, which opens the MotionPlanning display and replaces the simulator's own RViz:

    ros2 launch openamrobot_upperbody_bringup moveit.launch.py sim:=gazebo rviz:=true physics_engine:=gz-physics-bullet-featherstone-plugin

(same `ROS_DOMAIN_ID` and `GZ_PARTITION` as above; `physics_engine` is passed so a gripper pose can close: the launch's own default is DART,
where it may not. `sim:=mock rviz:=true lift_position:=0.175` shows the same preview with no Gazebo window, `lift_controller` taking the lift to its middle
stop after start-up.)

## Items

Look at both windows for each item. Gazebo shows what the physics built; RViz shows what the description says. Write down the numbers.

1. **The lift column and the carriage: three readings.** `tf2_echo base_footprint lift_column_base_link` reads **x 0.060, z 0.304**
   at `bp000` (x 0.110 at `bp050`). With `lift_joint` at 0, 0.175 and 0.350 m (the slider; calibration stops `L1000`, `L1175`,
   `L1350`), `tf2_echo base_footprint openarm_left_base_link` reads **z 1.000, 1.175 and 1.350**, and its x and y do not change
   between the three (the carriage goes straight up). Record the nine numbers you read. The column is drawn as two boxes since P28: the outer stage
   **0.240 x 0.060 x 0.530 m** on `lift_column_base_link` and the inner stage **0.2095 x 0.051 x 0.350 m** on the carriage (the DOLD datasheet's sections; the
   lengths are ours). Say whether what you see is a narrow column with a thinner stage rising out of it, and read the plate's box (`adapter_plate_link`,
   **0.360 x 0.100 x 0.010 m**, rear edge flush with the column's rear face) under the arm profile.
2. **The arm profile, the plate it stands on, and the two mounts: orientation and spacing.** From above in RViz (orbit to look straight down, TF display on),
   measure `arm_profile_link`'s box: **0.120 m fore-aft (x) by 0.060 m across (y)**, 0.340 m tall, its centre **0.180 m ahead** of the
   column axis. `tf2_echo lift_carriage_link openarm_left_base_link` reads **x 0.180, y +0.031, z 0.166** and the right one **y -0.031**:
   the two arm bases are **62 mm** apart. Record which side of the box is the long one (V01-F1: it must be the fore-aft side). Since P28 the profile
   stands on `adapter_plate_link` (its parent): its bottom face is the plate's top face (gap **0.000**) and its front face is the plate's front edge (**0.240 m** from the column axis).
3. **The arms at the home pose: contact count.** At every joint zero and each of the three stops, count the pairs of arm links whose
   drawn volumes overlap (left against right, either against `arm_profile_link`, either against the base): the model's own
   collision check says **0** at all three stops and both base plates. Record your count per stop, and if it is not 0, the two link
   names and how deep. This is a reach question for A4 and Alex, not a reason to move the mounts here.
4. **The sensors: heights and directions.** `tf2_echo base_footprint head_camera_mount_link` reads **z 1.223 + lift** (1.223 at
   `L1000`, 1.573 at `L1350`) and **x 0.262** at `bp000` (0.060 column axis + 0.180 + 0.022); `head_camera_optical_frame`'s z axis
   points along the lens and is pitched **25 degrees down** at `hp25`. The two wrist cameras sit at their placeholder poses: record
   `tf2_echo openarm_left_ee_base_link left_wrist_camera_link` (y **+0.05**, z **-0.03**) and say whether that is near the hand.
5. **Driving: distance and offset.** `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"` for 5 s of
   simulated time, then `{angular: {z: 0.5}}` for 5 s. `ros2 topic echo --once /odom` before and after: record the distance
   (about 1.0 m) and the yaw (about 2.5 rad). Then `tf2_echo base_footprint lift_carriage_link` again: it must read the same as before
   driving (the upper body goes with the base: **offset change 0.000**). Say if RViz's model and Gazebo's model are further apart
   than the width of a wheel.
   (Stop with Ctrl-C and `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"`.)
6. **An arm trajectory: what moves.** Run the script below, or any `FollowJointTrajectory` goal on `/left_arm_controller`. Record the
   `openarm_left_joint1/2/4` values at the end (**0.3, -0.6, 1.2**, within 0.02), that `openarm_right_*` and the wheels changed by
   **less than 0.001**, and the largest single-step jump you saw in `ros2 topic echo /joint_states` at the start (it should be a
   fraction of the final value, not the final value).
7. **The gripper: both fingers.** `ros2 action send_goal /left_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.4, max_effort: 5.0}}"`
   (the right gripper's range is mirrored: use `-0.4` on `/right_gripper_controller`). Record `openarm_left_finger_joint1` and
   `openarm_left_finger_joint2` in `/joint_states` afterwards: they should be **equal within 0.01** (finger_joint2 mimics
   finger_joint1), and for the right hand opposite in sign to the left's.
8. **MoveIt: a named pose, preview and execution (A7).** In RViz's MotionPlanning panel choose Planning Group `left_arm`, open the
   Planning tab, set the Goal State to `hands_up`, press **Plan** and then **Execute**. Record the final `openarm_left_joint4`
   (**1.5**, within 0.01) of the real robot (the Gazebo window, and the Scene Robot in RViz), and that the Scene Robot's
   `openarm_left_ee_base_link` and the real one's differ by **less than 0.01 m** (`tf2_echo`). Repeat with `right_arm`, then `home` and
   `park` (`park` is a placeholder: `home` with joint 4 at **0.1** instead of 0, off its lower limit, issue #17), then the grippers
   (`left_gripper` / `right_gripper`, `closed` at **+0.7854 / -0.7854** and `open` at 0: the two hands take opposite-signed values).
   *If RViz says "Planning failed" right after the arm has been at `home`:* that should not happen any more. In Gazebo `home` sets elbow
   joint 4 exactly on its lower limit (0) and the simulator reads it a hair past it, which MoveIt refuses; `move_group` plans from a
   joint state with that noise clamped (see the MoveIt package README, "The joint-state clamp"). If you see it, say so: it is a finding.

A trajectory to run for item 6 (moves the left arm's joints 1, 2 and 4 over 4 seconds):

    ros2 action send_goal /left_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
      "{trajectory: {joint_names: [openarm_left_joint1, openarm_left_joint2, openarm_left_joint3, openarm_left_joint4, openarm_left_joint5, openarm_left_joint6, openarm_left_joint7], points: [{positions: [0.3, -0.6, 0.2, 1.2, 0.0, 0.2, 0.0], time_from_start: {sec: 4}}]}}"

## Record the answers

One line per item: the item number and the numbers you read. They go into the step's report and into the evidence for V01; the check
turns `PASS` only when all eight have an answer with the numbers, and `FAIL` if any reading is outside what the item says.

## Recorded: the eight items of the lift model, measured (P16, 14 Oct 2026)

Every item was measured by script, on the lift model (`dold_v4_350`, baseline configuration), and the reading is written next to the number
the item expects. **A person did not look at a screen**: where an item asks for a judgement that needs eyes, that part is marked
CANNOT-MEASURE and nothing stands in for it. Raw readings: `evidence/v01/`. Renders (one per lift stop, one isometric, drawn from the URDF's
own visual geometry, no simulator): `evidence/renders/`.

| # | Expected | Measured | Verdict |
|---|---|---|---|
| 1 | column axis x 0.060, z 0.304 (bp000), x 0.110 (bp050); arm base z 1.000 / 1.175 / 1.350, x and y unchanged | x 0.060, z 0.304 / x 0.110; arm base z 1.000 / 1.175 / 1.350 at x 0.240, y +0.031 at all three | **PASS**. The column has no `<visual>` and no `<collision>`: nothing is drawn for it |
| 2 | profile 0.120 fore-aft x 0.060 x 0.340, centre 0.180 ahead; arm bases (0.180, +-0.031, 0.166), 62 mm apart | box 0.12 x 0.06 x 0.34 at x 0.180; bases (0.180, +0.031, 0.166) and (0.180, -0.031, 0.166); long side fore-aft | **PASS** (read from the URDF, not from RViz) |
| 3 | 0 colliding enabled pairs at each stop and both plates | 0 of 478 enabled pairs at L1000, L1175, L1350 on bp000 and bp050; the 18 touching pairs are the disabled Adjacent ones | **PASS** on the collision geometry (the quantity the item names); the arms' visual meshes are different files and were not intersected separately |
| 4 | head mount z 1.223 + lift, x 0.262; optical axis 25 deg down; wrist cameras (0, +-0.05, -0.03) | z 1.223 / 1.398 / 1.573, x 0.262 (0.312 at bp050); elevation -25.0 deg; (0, +-0.05, -0.03), 0.058 m from the hand base | **PASS**. "Near the hand" is a judgement about the real hand and was not made |
| 5 | about 1.0 m, about 2.5 rad; offset change 0.000 | 0.9995 m, 2.465 rad; offset change 0.0 | **PASS** for the numbers. CANNOT-MEASURE: whether RViz's and Gazebo's models are further apart than a wheel width (two windows by eye; the script's lookup of Gazebo's base pose returned nothing) |
| 6 | joints 1/2/4 0.3 / -0.6 / 1.2 +-0.02; right arm, wheels < 0.001; first-second step a fraction of the final value | 0.30001 / -0.60001 / 1.19995; right arm 1.2e-5, wheels 0.0; largest step 0.0015 / 0.0030 / 0.0060 rad | **PASS** |
| 7 | finger_joint2 = finger_joint1 within 0.01; right opposite in sign | left 0.4008 / 0.4008, right -0.4000 / -0.4000 in `/joint_states` (a kinematic mimic: cannot disagree); physical link-pose angles left 0.4008 / 0.4002, right -0.4000 / -0.4000 | **PASS** |
| 8 | joint4 1.5 +-0.01; scene vs real end effector < 0.01 m; park == home; grippers +0.7854 / -0.7854, open 0 | joint4 1.50037 (left), 1.49916 (right); goal-state vs real end effector at most 0.0005 m (six arm poses); park == home (both arms); closed 0.78457 / -0.78456; 10 of 10 requests SUCCESS | **PASS** for the numbers. CANNOT-MEASURE: "the Scene Robot in RViz" is an RViz object; the goal state's forward kinematics against the real tf stands in for it |

**What this does not change.** V01 stays `WARN`, and would whatever these eight said: `lift_mass_tl3_400` is null (no supplier mass for the
fallback TL3 400 column), so one of the two columns the model offers cannot be built, and nothing here measured it. These eight are
measured on `dold_v4_350` alone. The column has no collision geometry and one invented lateral dimension; the renders draw its envelope as a
magenta wire box that is an annotation, not model geometry.

Findings of this pass: none of the eight reads outside its expected value. By eye, in the renders: at `L1175` and `L1350` the carriage and
`arm_profile_link` float above the base with nothing drawn between them, which is what "no column geometry" looks like and is the reason
the renders carry the annotation.

## Recorded: the placeholder geometry, looked at (P25, 21 Oct 2026)

V01 found five things in the renders of the lift model by looking, which is what V01 is for. **One was fixed; four are measured and wait for Alex.**
V01 stays `WARN` (`lift_mass_tl3_400` is still null). Raw numbers: `evidence/p25/p25_measurements.json` (made by `evidence/p25/measure_p25.py`
from the expanded URDF, both base plates, the three stops); captures: `evidence/p25/rviz/` and `evidence/p25/gazebo/`.
**Nothing below changes a mount pose, a stop height, a joint or a frame.**

| # | Finding | What was done | Status |
|---|---|---|---|
| 1 | The three placeholder boxes had no `<material>`: RViz drew them red, Gazebo black | one named material and a `<gazebo>` block per link | **FIXED** |
| 2 | The column's two stages interpenetrate (identical 240 x 240 section), and M03 cannot see it | overlap volume and depth measured at six configurations; a way out proposed, not applied | **MEASURED**, waits on Alex |
| 2b | The column section is square because its lateral side was set equal to the sourced fore-aft side | compared with intent; no number derived | **MEASURED**, waits on the supplier drawing |
| 3 | The arm profile does not overlap the column (ADR-010 section 2) and does not stand on it either | gap and cantilever measured; the register entry quoted | **MEASURED**, a question for Alex |
| 4 | `arm_profile_link`'s visual and collision have no `<origin>` | read against the mount poses | **CONFIRMED** not an omission |
| 5 | `lift_column_base_joint`'s parent is `base_footprint` | dependents listed; a re-parent tried on a copy | **CONFIRMED**, not changed |

### 1. Materials: fixed

`lift_column_base_link`, `lift_carriage_link` and `arm_profile_link` use the material `placeholder_geometry`, rgba 0.78 0.64 0.45 1.0 (flat cardboard tan;
nothing else in the combined robot uses it: the base is white and grey, the arms and sensors grey and black). Each link also has a `<gazebo reference>`
block with the same ambient and diffuse, because a URDF material does not carry into the SDF the way it looks: with the URDF material alone `gz sdf -p`
gives the column stages 0.975 0.8 0.5625 (the URDF colour times 1.25), and with neither the SDF has no material at all.

**One consequence for Alex's file.** sdformat applies a link's `<gazebo><visual><material>` block only to a visual that has no name of its own. The two
column visuals were named `placeholder_lift_column_*_stage`, and with that name the block was ignored (the arm profile's visual is unnamed and took it).
The two visuals are now unnamed; their collisions keep the `placeholder_*` names. Nothing referenced the visual names (searched). `config_id` changes,
because the URDF text does. `test_p25_placeholder_material.py` pins the colour in the URDF, in the `<gazebo>` block and in the SDF `gz sdf -p` makes; it
fails on the old xacro and on a re-named column visual (both tried).

**Same colour in both windows? Same colour definition and same hue; not the same pixels.** At `L1000`, `bp050` (lift joint read back as 0.000 m in both):

| | URDF / SDF | RViz capture | Gazebo capture |
|---|---|---|---|
| median rgb of the box pixels | 0.78 0.64 0.45 | 0.733 0.600 0.424 (17 334 px) | 0.420 0.380 0.322 (8 501 px) |
| hue | 34.5 deg | 34.2 deg | 36.0 deg |
| saturation | 0.42 | 0.42 | 0.23 |
| channel ratios (max = 1) | 1, 0.82, 0.58 | 1, 0.82, 0.58 | 1, 0.91, 0.77 |

Before, the P19 RViz capture of the same configuration shows the boxes red (median 0.69 0 0, 15 514 px); a Gazebo capture before was not taken (the SDF carried no
material). Both windows now draw a tan box, and RViz reproduces the URDF colour almost exactly. Gazebo's is the same hue and noticeably greyer and darker. The
ratios Gazebo gives (1, 0.91, 0.77) are close to the URDF's raised to 1/2.2 (1, 0.91, 0.78), which is what a renderer that takes the rgba as linear light and
sRGB-encodes the result would produce; that is a reading of the numbers, not something I checked in gz-rendering. I did not compensate the SDF numbers to hide
it: the URDF and the `<gazebo>` block carry the same four numbers, and the test says so. If Alex wants the windows to match on screen, the way is a
linear-light value in the `<gazebo>` block only, and that is a presentation choice for him.

### 2. The column and carriage interpenetrate: measured

`lift_column_base_link`'s box is 0.240 x 0.240 x 0.530 m, z 0.304 to 0.834. `lift_carriage_link`'s is 0.240 x 0.240 x 0.350 m centred 0.175 m below the
carriage frame, so at `lift_joint` 0 it spans z 0.484 to 0.834: the same cross-section, fully inside the column. Axis-aligned, so the intersection is exact
(cross-checked by 2 million seeded random points: 0.020160 against 0.020160 and 0.010080 against 0.010085 m3).

| stop | `lift_joint` | z overlap (stroke still engaged) | overlap volume | penetration depth (shortest axis-aligned separation) |
|---|---|---|---|---|
| L1000 | 0.000 m | 0.350 m (z 0.484 to 0.834) | 0.020160 m3 (20.2 litres) | 0.240 m (sideways; 0.350 m vertically) |
| L1175 | 0.175 m | 0.175 m (z 0.484 + 0.175 = 0.659 to 0.834) | 0.010080 m3 | 0.175 m (vertical) |
| L1350 | 0.350 m | 0 (the carriage box's bottom face touches the column's top face at z 0.834) | 0 | 0 (contact, no overlap) |

The numbers are the same on `bp000` and `bp050`: the plate moves the column axis (x 0.060 or 0.110) and both boxes with it. All six rows are in the JSON.

**M03 cannot see this, and the reason is structural.** `lift_column_base_link` and `lift_carriage_link` are a pair across `lift_joint`, and the derivation labels
every such neighbour `Adjacent` (SRDF line 243, `reason="Adjacent"`). M03's claim is that a disabled pair is "declared by upstream, a structural neighbour, or
has no movable joint between its links and is clear"; a structural neighbour is disabled without being evaluated, so a collision between the two stages is not
in anything M03 reads. **This is the same defect class as platform-sw #70's chassis/caster overlap**: collision shapes that overlap permanently where the real
parts do not. #70's was caught, because those pairs are `ConstantTransform` and so are evaluated once; this one is two orders of magnitude deeper (0.175 to 0.350 m
of vertical overlap against 2.4 mm) and cannot be caught the same way. It does no harm today: the pair is disabled in MoveIt and the SDF sets no `self_collide`
(its default is off; I did not test contacts in the simulator). It is a model error that a different checker, or a change to the matrix, would report.

**Not applied, proposed.** The real Hexalift is telescopic: the inner stage runs inside the outer one, so the *volumes* overlap in the real column too, and what is
wrong is the model, which has no way to say "inside". A smaller inner section does not clear it while the outer stage's collision is a solid box: the inner box is
inside the outer box's volume for z 0.484 + lift to 0.834 whatever its section (at 234.8 mm square the overlap at `L1000` is still 0.019296 m3 and the depth
0.2348 m). Clearance of the kind #71 gave (2.6 mm) needs the outer stage's collision to be a hollow shell (four wall slabs of thickness t, open at the top) and the
inner section to be at most

    s_inner = 0.240 - 2 t - 2 x 0.0026 = 0.2348 - 2 t   (m, square)

t, the housing wall, is in no document we hold, so there is no sourced number: the one value that invents nothing is the bound, **s_inner < 234.8 mm (t = 0)**,
and anything smaller has to come from DOLD's drawing. Both sections are also relative to the invented lateral side (item 2b). Because M03 will not look at the
pair, the regression has to be a test of the kind `test_m03_caster_clearance.py` is (old geometry fails, new geometry clears, analytic, no ROS); I did not write it.

### 2b. The column section against design intent: measured and compared

Our section is **0.240 x 0.240 m, aspect ratio 1.00 : 1** (fore-aft : lateral), drawn square. **Only the fore-aft side is sourced**: the register's LIFT entry
(`decisions.yaml` line 1081) says "column 530 mm retracted, 880 mm extended, mounted with its 240 mm side fore-aft", and `datums.yaml` records the lateral side as
"in no document we hold: it is set equal to x only so the placeholder inertia and the density bound can be evaluated" (`lift_column_section`, "the one invented
number of the lift"; since `b45d8f6` it also sizes both collision boxes). The P25 prompt reports that Alex's published dark-store render (docs.openamrobot.ai,
"Shoulder height 1000 mm, DOLD Hexalift V4") shows a column visibly narrower laterally than it is deep; **I did not look at that render**, and I derived no number
from it. A render is evidence of intent, that the lateral side is probably less than 240 mm; it is not a dimension, and the datum stays `placeholder` until the
supplier drawing gives one. **This is the same failure mode as the earlier, superseded HFS6-60120 mast section** (DESIGN_LOG, DATUMS.md: `mast_section` was `[0.08, 0.08]`, a square
invented for a profile whose part number says 60 x 120): a section drawn square because one side was known or assumed and the other was invented as equal. The
difference is that for the mast neither side was sourced; here one is, which makes the invented side harder to notice. V01 found the mast by looking; the
renders are what made this one visible.

### 3. The arm profile against ADR-010 section 2: measured, a question for Alex

`arm_profile_joint` is at (0.18, 0, 0.18) m from the carriage; the box is 0.12 x 0.06 x 0.34 m with no visual origin. On either base plate it spans, from the
column axis, x 0.12 to 0.24 m, y -0.03 to +0.03 m, and 0.010 m above the carriage frame to 0.350 m above it.

| stop | profile z range | gap to the column's top (fixed stage) | gap to the carriage box (moving stage) top | overlap volume | forward gap (rear face to column front face) |
|---|---|---|---|---|---|
| L1000 | 0.844 to 1.184 m | 0.010 m | 0.010 m | 0 | 0.000 m (flush) |
| L1175 | 1.019 to 1.359 m | 0.185 m | 0.010 m | 0 | 0.000 m |
| L1350 | 1.194 to 1.534 m | 0.360 m | 0.010 m | 0 | 0.000 m |

So the profile does not overlap the column at any stop; it sits 10 mm above the carriage box's top face (the 10 mm adapter plate, not modelled), and its
**entire footprint (0.12 x 0.06 m) is in front of the column's footprint**: the rear face of the profile is the plane of the column's front face, so none of it stands over
the column's top face (footprint overlap 0 m2). Its centre is 0.180 m ahead of the column axis, 0.060 m ahead of the column's front face, and its far face is 0.120 m
ahead of it. **Cantilever, at all three stops:** the horizontal offset is the same (the carriage goes straight up): centre 0.180 m, far face 0.240 m from the
axis. What changes with the stop is how much inner stage is left in the housing (0.350, 0.175, 0 m of the 0.350 m stage) and how high the load stands above the
housing's top: the profile's centre 0.180, 0.355 and 0.530 m, the arm mount points 0.166, 0.341 and 0.516 m (z 1.000, 1.175, 1.350 m from the floor).

**The register entry.** `decisions.yaml`, ARM-MOUNT (P-03 rev19.0, 12 Oct 2026, item 19), lines 1127 and 1128:
"arm mount point 0.180 m ahead of the lift column axis and 0.166 m above the column top plate" and
"arm profile MISUMI HFS6-60120 (60 mm lateral, 120 mm fore-aft), 340 mm, end-tapped, standing on a 10 mm adapter plate on the column top".
**The sentence the prompt and ADR-010 section 2 attribute to the register, "the arm profile overlaps the lift column (it does not sit on top), so the shoulder
reaches 1000 mm without cutting into the chassis", is not in the register.** It is Alex's Slack post of 7 Oct (`findings/HW_DECISIONS_2026-10-07.md`, item 3, rev18.7), which
ADR-010 quotes as "Register and Slack both"; `decisions.yaml` has no "overlap", "sit on top" or "not sit" wording at any of its 8 commits (`git show <commit>:decisions.yaml | grep`, `openAMRobot/.github` at `9d422aa`, the file last changed in `9109d3e`), and the
rev19.0 ARM-MOUNT entry that replaced it says the opposite, "standing on ... the column top". The model matches neither statement: "standing on" is true in z (10 mm above)
and not in x (the footprint is all in front), and "overlaps" is false (volume 0). LIFT's own arithmetic, "shoulder ... 304 + 530 or 880 + 166 mm", is the standing-on
geometry. **The question for Alex:** which is the design, and what is the 10 mm adapter plate, since it has to reach at least 0.12 m past the column's front face to carry
the profile. Nothing was changed.

### 4. `arm_profile_link` has no `<origin>` on its visual or collision: confirmed, not an omission

With no `<origin>` the box is centred on the link frame (URDF default: identity); the SDF shows the same, the visual at pose (0.18, 0, 0.18) in the carriage. That is consistent
with the joint that places it: `arm_profile_joint`'s z is `adapter_plate_thickness + length / 2` = 0.010 + 0.170 = 0.180 and its x is `center_x_from_column_axis`, so the
joint *is* the box centre by construction, and the missing origin is the other half of that choice. It is also consistent with the mount poses: the arm mount points (x
0.180, y +-0.031, z 0.166 above the column top) are at the profile's centre in x, 156 mm above its bottom face (the box is 340 mm tall), and **1.0 mm outside its lateral faces** (the box is
+-0.030, the mounts +-0.031; the register's ARM-MOUNT mentions "one 1 mm spacer under each J1_A plate", which is probably this millimetre, though I did not check that), the same at all six configurations. The two column links do the opposite (frame at the base of the box, visual
and collision offset by half the length), so the three boxes do not share a convention; that is a difference to know about, not an error.

### 5. `lift_column_base_joint`'s parent is `base_footprint`, not `base_link`: confirmed, not changed

The joint is (0.060, 0, 0.304) m from `base_footprint` at `bp000` (0.110 at `bp050`); `base_footprint_joint` puts `base_link` 0.098 m above it. **What depends on the
choice:** the 0.304 (`base_plate_top_z`, "294 mm deck + 10 mm plate") and every height of the lift (register LIFT 304 + 530 or 880 + 166) are from the floor, which is
`base_footprint`, so the shoulder height, E01's envelope, T05, and the calibration stops L1000 / L1175 / L1350 read straight off the tree; the tests
and contract that assert the parent (`test_t03_tree.py`, `test_s4_sensors.py`, `frame_contract.yaml`'s two column entries and T04's reading of them);
`openamrobot_upperbody.urdf.xacro` (the standalone wrapper), which declares `base_footprint` only and has no `base_link` to parent to; and Gazebo, where every
fixed joint lumps into `base_footprint` either way. **What would change if it were re-parented to `base_link`:** tried on a copy of the expanded URDF, with the z origin lowered by
0.098 to 0.206 m, **0 of 57 link poses in `base_footprint` change** (largest difference 0.0); without lowering it, 32 of 57 links (everything from the column up) move up by 0.098 m.
So the change is a one-line joint edit plus the tests and contract above, a new coupling (the upper-body xacro would have to know platform-sw's 0.098 m, a datum
the base owns), and a changed `config_id`. The one reason to want it is physical: the column is bolted to the chassis, which platform-sw places relative to `base_link`, so if
the axle height datum ever moved, a column parented to the floor would stay put while the chassis it stands on moved. I did not derive the collision matrix for the
re-parented tree (the `Adjacent` pair `base_link` / `lift_column_base_link` stays a direct neighbour, and the `ConstantTransform` pairs are unaffected by where a fixed
joint hangs; expected unchanged, not run). Whether to do it is Alex's: it is his file and his heights-from-the-floor convention.

### What now waits on Alex

1. **The arm profile**: overlapping the column (the 7 Oct Slack post) or standing on its top on an adapter plate (the rev19.0 register): the model is neither, and the 10 mm adapter plate has to reach 0.12 m past the column's front face.
2. **The column's nesting and wall**: how the two stages are to be represented (hollow outer shell, wall thickness t) so that the stages clear each other; the proposal needs DOLD's drawing.
3. **The column's lateral side**: the supplier drawing; until then 240 mm is an invented number that is collision geometry.
4. **The column's parent frame**: `base_footprint` (floor-referenced heights) or `base_link` (follows the chassis); and whether the standalone wrapper then declares `base_link`.
5. **Acceptance of the two edits to his xacro** (the material and the unnamed column visuals), as for P15's opt-in parameters, and whether the Gazebo colour should be compensated to match RViz on screen.

## Recorded: the geometry items, answered (P28, 28 Oct 2026)

Alex answered on 22 Oct (P-03 rev19.6 item 18): the column section is the DOLD Hexalift's, and there is a 360 x 100 x 10 mm adapter plate under the arm profile. The
inner stage's section is in the same datasheet (page 4, the two dimensioned drawings: "Outer profile 240x60", "Inner profile 209.5x51"; they are embedded images, the
PDF's text layer does not carry them). Measured by script on the expanded URDF and with MoveIt's collision check at the six configurations (`evidence/p28/measure_p28.py`,
`v01_geometry.json`); six real RViz2 captures in `evidence/p28/rviz/` (the lift joint read back from `/joint_states` equals the stop in all six). **I looked at two of the six
captures (L1000/bp000 and L1350/bp050); nobody has gone through the checklist on a screen, so V01 stays `WARN` for that too.**

| # | P25 finding | P28 answer | Status |
|---|---|---|---|
| 2b | the column's lateral side was invented (square 240 x 240) | `lift_column_section` is **0.240 x 0.060** (supplier, provisional): sourced, no longer invented | **CLOSED** |
| 2 | the two stages interpenetrate (identical 240 x 240 section) | the carriage is the inner stage, **0.2095 x 0.051**, strictly smaller on both axes and nested in the outer section (margins 15.25 mm fore-aft, 4.5 mm each side laterally). Overlap volume **0.003740 m3 at L1000** (was 0.020160), **0.001870 at L1175** (was 0.010080), 0 at L1350; penetration depth **0.051 m** (was 0.240). Cross-checked by 2 million random points (0.003740) | **NOT CLOSED**: smaller, not gone. The outer stage's collision is a solid box, so the inner box is inside its volume whatever its section (P25's argument stands). A hollow outer needs the housing wall, which the datasheet does not give; what it does bound is that wall plus sliding clearance is at most **4.5 mm laterally** and **15.25 mm fore-aft** per side. The pair is a structural neighbour, so M03 still does not evaluate it |
| 3 | the arm profile neither overlapped the column nor stood on it (ADR-010 §2 against the register) | the register says it stands on the plate, and the plate exists: `adapter_plate_link`, 0.360 x 0.100 x 0.010, **rear edge flush with the column's rear face (-0.120 from the axis), front edge +0.240**, bottom face on the carriage frame's plane. The profile (x +0.120 to +0.240) stands on it: **gap 0.000 m**, footprint inside the plate's, **front faces coincide**. The plate overhangs the column's front face by 0.120 m, which is all of the profile's footprint (the profile's footprint over the column's top face is 0 m2, as in P25: the plate carries it) | **CLOSED** (ADR-010 §2 corrected; the quotation was Slack's, not the register's) |
| 4 | `arm_profile_link` has no `<origin>` | unchanged, and its joint is now relative to the plate (x 0.060, z 0.175 from the plate frame): its pose from the carriage is **(0.180, 0, 0.180), as before** | **CONFIRMED** |
| 1 | the three boxes had no named colour | unchanged; the plate uses the same material | FIXED (P25) |
| 5 | the column's parent frame (`base_footprint` or `base_link`) | not a geometry item, not changed | **WAITS on Alex** |
| (P25 5) | acceptance of the edits to his xacro, and the Gazebo colour | P28 adds a third edit to his macro: opt-in `adapter_plate` and the profile re-parented under it, and a fourth parameter value for `column_geometry` (the inner section) | **WAITS on Alex** |

The P16 items read again on the new geometry (frames from the expansion; the plate and the sections did not move a mount, a stop or a frame):

| item | expected | measured at L1000 / L1175 / L1350, bp000 | verdict |
|---|---|---|---|
| 1 | column axis x 0.060, z 0.304; arm base z 1.000 / 1.175 / 1.350, x and y unchanged | arm bases (0.240, +-0.031, 1.000 / 1.175 / 1.350) from `base_footprint`, x and y the same at all three; the same at bp050 with x +0.050 | PASS |
| 2 | profile 0.120 x 0.060 x 0.340, centre 0.180 ahead; bases (0.180, +-0.031, 0.166) | box 0.12 x 0.06 x 0.34, centre (0.180, 0, 0.180) from the carriage; bases unchanged | PASS |
| 3 | 0 colliding enabled pairs at each stop and both plates | **0 of 567** enabled pairs colliding at all six configurations (MoveIt, lift held at the stop, every other joint at its default). 18 disabled pairs touch at L1000 and L1175, 17 at L1350: the `Adjacent` ones, among them the plate and the profile (face to face by design) and the two stages | PASS (on the collision geometry, as before) |
| 4 | head camera mount z 1.223 + lift, x 0.262 | (0.262, 0, 1.223) at L1000 | PASS |
| 5 to 8 | driving, a trajectory, the gripper, MoveIt | not repeated by hand. A7 on the new geometry: G01, M02 (named poses in Gazebo), M01 and M04 PASS in all three runs, G02 PASS in two of three (the third: the bridge crash at shutdown, `parameter_bridge-3` -11) | not re-measured |

**What closes and what does not.** Closed: the lateral side (sourced), the arm profile standing on its plate (the model matches the register), the carriage no longer a second copy
of the outer box. Not closed: the stages' overlap in volume (needs the housing wall), the inner stage's length (the datasheet gives none, it is the 0.350 m stroke), whether the
inner section sits centred and the same way round (the drawing does not say; taken so), whether the plate's rear edge is flush with the column's rear face (derived from "240 of
column plus 120 ahead", not stated), the column's parent frame and the acceptance of the xacro edits (Alex's), and `lift_mass_tl3_400` (null: V01 `WARN`). The density band of the lift
mass had to be re-fitted (100 to 1600 kg/m3, was 1200) because the sourced section puts 11.7 kg at 1533 kg/m3: that is a judgement call recorded in the datum, not a measurement.

## Sheet for a person: the six P28 captures (to be filled in by a human)

**Nothing on this sheet has been filled in or inferred, and nothing here is closed.** The two right-hand columns are empty on purpose: V01 only turns from `WARN` when a person has looked and written down what they read. A reading is a number, not "looks fine".

The six captures are in `evidence/p28/rviz/` (`rviz_<stop>_<plate>.png`, 1500 x 945, real RViz2 with the MotionPlanning display on the mock stack, fixed frame `base_link`, one fixed camera; each `.json` beside it records the lift joint read back from `/joint_states`: 0.000, 0.175 and 0.350 m for L1000, L1175 and L1350). They are stills: they can show what is drawn, not a TF reading, not a motion, not a controller. The arms are in the mock stack's initial joint state; I did not check that it is the `home` pose. Where an item needs a number from `tf2_echo`, run `ros2 launch openamrobot_upperbody_bringup view.launch.py base_plate:=bp000` (the checklist above says how) and read it there.

**Part A, about ten minutes (items 1 to 4: what the stills can show).**

| # | What to read | Expected | Where | Captures that show it | Your reading | PASS / FAIL (human) |
|---|---|---|---|---|---|---|
| 1 | the column and the carriage, three stops | `tf2_echo base_footprint lift_column_base_link`: x 0.060, z 0.304 at `bp000`, x 0.110 at `bp050`. `tf2_echo base_footprint openarm_left_base_link`: z **1.000 / 1.175 / 1.350** at L1000 / L1175 / L1350, x and y the same at all three (x 0.240 at `bp000`, 0.290 at `bp050`, y +0.031). Drawn: outer stage 0.240 x 0.060 x 0.530 m, inner stage 0.2095 x 0.051 x 0.350 m rising with the carriage | live (`tf2_echo`); the picture for the drawn part | `rviz_L1000_bp000.png`, `rviz_L1175_bp000.png`, `rviz_L1350_bp000.png` (the carriage height at the three stops); `rviz_L1000_bp050.png` against `rviz_L1000_bp000.png` (the 0.050 m shift) | | |
| 2 | the arm profile, the adapter plate and the two mounts | profile box 0.120 fore-aft x 0.060 across x 0.340 m, centre 0.180 m ahead of the column axis, long side fore-aft. Plate 0.360 x 0.100 x 0.010 m under it: rear edge flush with the column's rear face, front edge level with the profile's front face (0.240 m from the axis), **gap between the plate's top and the profile's bottom 0.000**. `tf2_echo lift_carriage_link openarm_left_base_link`: x 0.180, y +0.031, z 0.166; the right one y -0.031 (62 mm apart) | live (`tf2_echo`); the picture for the plate and the profile | `rviz_L1350_bp000.png` and `rviz_L1350_bp050.png` (the profile is highest there); any other for the orientation | | |
| 3 | the arms against each other, the profile and the base: contact count | **0** pairs of arm links whose drawn volumes overlap, at every stop and both plates (MoveIt's check on the collision geometry says 0 of 567 enabled pairs; `evidence/p28/v01_geometry.json`). Count what you see, and if it is not 0 write the two link names and how deep | the picture | all six | | |
| 4 | the sensors: heights and directions | `tf2_echo base_footprint head_camera_mount_link`: z 1.223 + lift (1.223 at L1000, 1.573 at L1350), x 0.262 at `bp000` (0.312 at `bp050`); the optical frame 25 degrees down at `hp25`. Wrist cameras at (0, +-0.05, -0.03) from the hand base: say whether that is near the hand | live (`tf2_echo`); the stills only if the sensor frames are drawn in them (not checked) | `rviz_L1000_bp000.png` and `rviz_L1350_bp000.png` (the head camera at the two ends of the range) | | |

**Part B, not in the ten minutes (items 5 to 8: no capture shows them).** They need a running simulator, a trajectory or a goal; listing them here so that nothing is dropped. Procedure and numbers are in "Items" above.

| # | What to read | Expected | Where | Captures that show it | Your reading | PASS / FAIL (human) |
|---|---|---|---|---|---|---|
| 5 | driving: distance and offset | about 1.0 m and about 2.5 rad; `base_footprint` -> `lift_carriage_link` unchanged by driving (offset change 0.000); say whether RViz's and Gazebo's models are further apart than a wheel's width | live (simulator) | none | | |
| 6 | an arm trajectory: what moves | `openarm_left_joint1/2/4` at the end 0.3 / -0.6 / 1.2 (within 0.02); right arm and wheels moved by less than 0.001 | live (simulator) | none | | |
| 7 | the gripper: both fingers | `finger_joint1` and `finger_joint2` equal within 0.01; the right hand opposite in sign | live (simulator) | none | | |
| 8 | MoveIt: a named pose, preview and execution | `hands_up`: final `openarm_left_joint4` 1.5 (within 0.01), real and Scene Robot end-effector within 0.01 m; `park` equal to `home`; grippers `closed` +0.7854 / -0.7854, `open` 0 | live (simulator, MoveIt) | none | | |

Also for a person, and not answerable from a still: whether the stages, the plate and the profile look like the supplier's parts (they are boxes, and the two stages still overlap in volume), and whether the colour you see in RViz is the colour you see in Gazebo (P25 finding; Gazebo's is greyer and darker).

## Recorded: owner acceptance (1 Nov 2026)

| Field | Value |
|---|---|
| Check | V01 |
| Decision | accepted |
| Accepted by (owner) | Alex Reznichenko |
| Reviewer | Alex Reznichenko |
| Date | 1 Nov 2026 (the owner decision recorded on #26) |
| Renders looked at | the six-configuration renders, made on 30 Oct 2026 |
| Source | P-03 rev20.1 item 25 |
| Model | the final model, `main` at `58a5cf8` (the squash merge of #26; its PR head was `7ebeb3d`) |
| Configurations | the six: L1000, L1175 and L1350, each on `bp000` and `bp050` |
| Result in the A6 and A7 reports | `WARN`, by design (see below) |

The acceptance is the owner's decision on the model as drawn. It does not fill in the per-item readings of the sheet above, which stay empty because nobody wrote readings down. It is a visual check of a model, not hardware, mechanical, electrical or physical acceptance.

Open items recorded with the acceptance:

- The item 24 equipment positions (the upper-body equipment of #26) are placeholders until M-09.

- The overlap of the portrait tablet and the head light, 12.1 mm, is an input to M-09, not a defect closed here.

**Why V01 stays `WARN`.** Both report builders set V01 to `WARN` unconditionally (`ros2/src/openamrobot_upperbody_description/scripts/build_evidence.py` for A6, `ros2/src/openamrobot_upperbody_bringup/scripts/build_a7_evidence.py` for A7) and neither reads this page. They do so while `lift_mass_tl3_400`, the mass of the TiMOTION TL3 400 fallback column, is null: one of the two columns the model offers cannot be built with a mass. TiMOTION does not publish that mass. The selected column, the DOLD Hexalift V4 350, already carries its catalogue mass, 11.7 kg (`lift_mass_dold_v4_350`). DOLD's written holding confirmation was received on 14 Oct 2026 (P-03 rev20.2 item 26), so the fallback is not being pursued and its mass is not being sourced. No check code, threshold or allowlist was changed to record this acceptance.

Retiring the `tl3_400` column from the configuration axes would be a separate change under the CONFIG-IDS decision, with its own contract change request; this record does not make it.
