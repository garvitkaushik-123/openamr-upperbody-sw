# Changelog

All notable changes to this repository. Evidence for the description is static and mock only, and the Gazebo evidence is class
`simulation`; nothing here is a hardware result.

## [Unreleased]

### Added (15 Nov 2026; OpenArm software in the loop on virtual CAN)

- **New package `openamrobot_openarm_emulator`.** The unmodified upstream driver (`openarm_hardware/OpenArmHW`, enactic/openarm_ros2 `4e837e1`,
  with enactic/openarm_can `bfd4e52`, tag 1.4.1) talks over Linux virtual CAN to emulated Damiao motors. One emulator per arm, each on its own
  SocketCAN interface (`vcan_arm_l`, `vcan_arm_r`). Design input: the digital-twin audit of 28 Sep 2026 (openAMRobot/audits,
  `digital-twin/openarm/openarm_interface.md`, section 5). Simulation only. No safety logic. The combined model is not changed.
- **Frames.** The emulated motors answer enable, disable, set zero, clear error, MIT, position-velocity, velocity and position-force commands,
  refresh, and parameter read, write and save, with the real state layout (position, velocity, torque, two temperatures, status or error code).
  The codec is written independently and tested bit for bit against the upstream encoder and decoder.
- **Motion from Gazebo through ros2_control.** Each emulator is a controller (`openamrobot_openarm_emulator/OpenArmCanEmulator`) in the
  controller manager that `gz_ros2_control` hosts: it reads joint position and velocity, runs the MIT law and commands joint effort every physics
  step. The Gazebo position servo is not used for the arms on this path.
- **Timing.** A model of one shared bus per arm gives each frame its length in bits at 1 and 5 Mbit/s, one frame at a time, lowest id first.
  Motor reply latency is a parameter and is 0 until it is measured.
- **Fault injection:** CAN loss, single-motor timeout, over-voltage, over-temperature, E-stop power cut (bus up, motors silent, then reset), any
  motor fault code, and a configurable supply voltage (25.6 to 29.2 V on the robot). Parameters of the emulator controllers; `inject_fault.py` sets them.
- **Launch `sil.launch.py`:** creates the vcan interfaces (CAN FD with MTU 72 where the kernel allows, and it reports when not), starts Gazebo
  with the combined model, both emulators, the real driver pinned by commit, and an example arm trajectory. The simulator's controller manager is
  `/sim/controller_manager`; the real driver's is `/controller_manager`.
- **Tests and CI.** `test_codec`, `test_motor_model`, `test_real_driver`, `test_package_static` and `test_sil_launch`, registered in the package's
  CMakeLists.txt. New workflow `.github/workflows/openarm-emulator-tests.yml` builds the driver at the pins, creates vcan on the hosted runner and
  runs them. A test that needs vcan or the driver and cannot have it is BLOCKED, counted apart, and the job does not pass.
- **Driver behaviour recorded, not fixed** (the tests assert what upstream does today): every arm joint is driven to zero on activation, with a
  step command first; the gripper is sent 0.044 rad on activation; `read()` and `write()` return OK on CAN loss, with the interface down and
  during a power cut, with the state frozen; motor fault codes and temperatures are decoded and not acted on; after a power cut the motors are
  only re-enabled by deactivating and activating the hardware component.
- **Numerical damping treatment.** At a 1 ms physics step the MIT damping gain makes the light links chatter at torque saturation. The emulator
  attenuates the part of its torque above 200 Hz as an implicit step of the damping term would, using each link's inertia from the URDF. Static
  torque is unchanged. This is numerical, not a motor model; the package README says why.
- **Assumptions.** Motor behaviour that no source in scope fixes (state after power-up, reply while disabled, fault thresholds, CAN timeout,
  reply latency, speed at 25.6 to 29.2 V) is a parameter with a default, listed in the package README for bench measurement.
### Added (14 Nov 2026; simulator dynamics for the OpenArm 2.0 joints, simulation only)

Base: main `895ce34`. Inputs: audits `audit/2026-09-28` (`digital-twin/openarm/`, gaps 3, 6, 9 and 10 of `findings.md`). Simulation only:
no safety logic, no firmware, no change to a joint limit, origin, axis, mass or inertia. The real-hardware profile expands to the same text as before.

- **Simulator dynamics profile, kept apart from the kinematics** (`config/sim_dynamics/openarm_v2.yaml`, schema `sim_dynamics.schema.json`).
  Per joint and gripper: damping, friction, armature, and a copy of the URDF effort and velocity limits, each with its source, the place in
  that source and a status. The values are upstream's own simulator values, from its MuJoCo model of OpenArm 2.0 (`openarm_mujoco` `1daab06`,
  `v2/openarm_bimanual.xml`): joints 1 and 2 damping 1.0 N*m*s/rad, friction 0.2 N*m, armature 0.016 kg*m^2; joints 3 and 4 0.9, 0.1, 0.030;
  joints 5 to 7 0.9, 0.04, 0.002; fingers 0.6, 0, 0.002. `openarm_description` gives none: its `friction.yaml` is empty at the pin `14ff67b`
  and at main `38c64af`. The Isaac Lab asset (`cb7ff75`) is the 1.0 hand and has no joint friction or armature. Nothing is estimated except
  two cross-checks that are not applied (the rotor inertia each armature implies, and a fingertip force), each with its method.
  None of it is a measurement of a delivered arm.
- **Applied through xacro to the simulated outputs only.** New `urdf/sim_dynamics.xacro` and xacro arg `sim_dynamics` (`auto`, `on`, `off`).
  `hardware_profile:=gazebo` now has `<dynamics damping friction>` on the sixteen controlled arm and gripper joints and on the two finger mimic
  followers. `sim_dynamics:=on` gives the same for a MuJoCo-facing URDF. `hardware_profile:=real` has none, and `sim_dynamics:=on` with `real`
  stops the expansion. `mock` has none by default. The upstream macros take no dynamics argument, so the file redefines the one upstream macro
  that emits every joint's `<limit>`, with the same `<limit>` and the `<dynamics>` after it; with the eighteen lines removed the expansion
  equals the one without them. Armature is not applied: URDF has no element for it and no link inertia is changed.
- **Controller rate in the simulation: 750 Hz, upstream's rate for the real driver** (`openarm_ros2` `4e837e1`,
  `openarm_bringup/config/controllers/openarm_bimanual_controllers.yaml`). New `openamrobot_upperbody_control/config/sim_overlay.yaml`, handed to
  `gz_ros2_control` after `arm_controllers.yaml` through the new xacro arg `controllers_overlay_file` (a second `<parameters>`).
  `arm_controllers.yaml` is unchanged: mock and real stay at 100 Hz. `gz_ros2_control` updates on whole physics steps, so
  `openamrobot_upperbody_gazebo/worlds/empty.sdf` now steps at 1/1500 s (was 1 ms). Measured: 499.5 Hz with 1 ms steps, 743.6 Hz with the new
  step (most common interval between updates 1.333 ms). The simulation runs slower for it. A world with 1 ms steps runs the arm controllers at 500 Hz.
- **Gripper grip limit from the motor data**: 3.0 N*m, the rated torque of the gripper motor (rated current 2.5 A), in the profile and as
  `max_effort` of both gripper controllers in the simulation overlay. It is carried on the command path and not enforced: the position
  command interface ignores `max_effort` and the URDF effort limit (7 N*m, the motor's peak) is unchanged.
- **Tests, registered in CMake**: `test_sim_dynamics_profile.py` (schema, a source and locator for every value, a method for every estimate,
  full commit hashes, the pins equal `upstream_pins.yaml`, the joint list equals the model's, the limit copies equal the URDF's),
  `test_sim_dynamics_urdf.py` (`<dynamics>` on all 16 arm and gripper joints in `gazebo`, none in `real`, nothing else moved),
  `test_sim_overlay_yaml.py` (the overlay equals the profile and adds no controller), `test_sim_control_rate.py` (the world step gives 750 Hz).

- **Not used by the software-in-the-loop launch (15 Nov 2026).** `openamrobot_openarm_emulator/launch/sil.launch.py` expands the model with
  `sim_dynamics:=off`. It drives the joints by effort on 1 ms steps, where the profile's damping is stepped explicitly and is unstable
  without the rotor inertia (damping times the step over the link inertia is up to 6.6; the limit is 2). Found by that package's CI on the
  first merge of the two changes: an arm that should hold at zero read -1.33 rad. The position-servo simulation is not affected. A static
  test recomputes the ratio and checks the launch.

### Fixed (14 Nov 2026; gripper `open` and `closed` were swapped in the SRDF)

- **`open` is now the far end of each hand's range (+0.7854 left, -0.7854 right) and `closed` is 0.** They were the other way round, so a
  `closed` command opened the hand. Audit finding 6 holds. Evidence: upstream docs ("the closed position is used as zero by default"), the
  upstream driver (`openarm_can`: "0=closed, 1=open"), the upstream MuJoCo model (ratio 0 closed, 1 fully open at the range end away from 0),
  and this model's geometry: the fingertip collision meshes are 0.1 mm apart at 0 and 139 mm apart at the far end.
- **M05 now ties the names to the geometry.** It asserted the swapped convention and so passed with the defect. It now measures the fingertip
  gap on the expanded URDF (under 1 mm at `closed`, over 100 mm at `open`), fails the swapped names, and still fails the single-number
  version in both directions. `named_poses.open_value` is the per-hand far limit; `closed_value` is 0.
- **Test sequences keep the same motion under the right names**: the gripper sequence of M01, M02 and M06 is `open` then `closed` (was `closed`
  then `open`): to the far end and back to 0, as before. Text that said "close" for moving the fingers now says "move" or "reach the goal".
  Not changed: the joint limits, the sign per hand, the controllers. The motor sign per hand on the real gripper is still undocumented upstream.
### Added (14 Nov 2026; night-shift actuator-ledger, proposal)

- **Actuator and electrical ledger** `config/actuator_ledger.yaml` with `config/actuator_ledger.schema.json`: one entry per actuator or bus
  node (2 x ZLLG80ASM250-L-B2 V1.01 hub motor, ZLAC8015D V4.2, DOLD Hexalift V4 350, CTL-017 lift node, Daly BMS, and the 8 OpenArm v2.0 motors
  per arm including the gripper). Every value carries unit (SI), source, status and owner; unknown values are null with status `unknown` and
  a note naming what closes them. Data only: no URDF, xacro, controller, firmware or launch file reads it, and `config_id` does not change.
- **`test/test_actuator_ledger.py`**: schema validation, provenance of every value, and equality of the ledger's arm and lift limits with the
  expanded combined URDF. Registered in `CMakeLists.txt` as `actuator_ledger`, so `colcon test` runs it.
- **Hub motor values from the selected B2 V1.01 drawing** (platform-hw `8ed2b06`, `datasheets/ZLTech/wheel_ZLLG80ASM250`): part and revision, direct
  drive, 5.5 / 16 N*m, 200 / 260 rpm, 6.5 / 19 A, 24 V. Rotor inertia, damping and friction stay unknown.

### Changed (14 Nov 2026; platform pin to platform-sw main 8e7a454)

- **Platform pin `713574f` -> `8e7a454`** (`config/upstream_pins.yaml`, README checkout line): the merge of platform-sw #93, decision NEAR-FIELD-FRAMES
  (P-03 rev19.7 item 19 D6, register 20 Oct 2026). The only model change is the TF-Luna cliff sensor pitch, `tof_front` and `tof_rear` 20 -> 10 deg down
  (`sensor_frames.yaml`, `sensors.xacro`). Neither link has collision geometry or an inertial, so the collision matrix (180 pairs), the masses and the
  `roadmap_mass_export.yaml` copy are unchanged. The other platform-sw commit in the range (#121) changes tests only. Datums revision unchanged.
  A6 and A7 rerun on this pin; the evidence report records platform-sw `8e7a454`. Register PLATFORM-PIN follows this pin once it is merged.

### Changed (10 Nov 2026; P-03 rev20.6 item 30, upperbody-sw #48 follow-up)

- **Lift fixing text: 260 x 30 mm foot tab holes, not 224 x 40 mm** (P-03 item 30 A3, decisions D-20 and D-21, superseding item 28). The lift is fixed to the
  lift base plate MEC-019 through the four foot tab holes on the 260 x 30 mm pattern with 4 x DIN 7984 M8 x 14 class 8.8 into tapped M8 threads; the 224 x 40 mm
  pattern is internal to the DOLD assembly. Text only (the `lift_column_section` datum note and the description README); neither pattern is modelled, so no
  geometry changes. Datums revision `2026-11-08.b` -> `2026-11-10.a`, so every `config_id` changes.
- **Mass inputs from the reviewed roadmap export on openamrobot-docs main** (`ff8ca57`, docs #84; was the branch export `eaa26aa`, 8 Nov 2026):
  `config/roadmap_mass_export.yaml` is the verbatim copy, `roadmap_mass_export.source.yaml` carries the new commit and SHA-256. For the upper body only
  `lift_column_base_link` changes: 9.100 -> 9.132 kg (the four DIN 7984 M8 x 14 foot screws, LFT-008, BOM Issue 7.15, D-20), com z 0.1239 -> 0.1234 m.
  `lift_carriage_link` stays 6.721 kg. The base links come from platform-sw at the merge commit of platform-sw #126 (`713574f`), which carries the
  export's base figure (58.16 kg).

- **Collision matrix re-derived for the platform pin `713574f`** (`scripts/derive_acm.py --write`, ROS 2 Jazzy with moveit_py 2.12.4): 180 disabled pairs
  (20 upstream + 28 `Adjacent` + 132 `ConstantTransform`), was 194; fingerprint `sha256:2263a2f6...a39d`. camera_link has no collision geometry since platform-sw #106
  (a frame only; its placeholder box was removed), so its 14 pairs are gone: `Adjacent` with base_link and `ConstantTransform` with the four wheels, the four
  castors, lidar_link, lift_column_base_link and the three stop sensors. None added or relabelled. `test_acm_derive.py` and `test_m03_acm_sampling.py` expect
  180 / 132 (structural neighbours 46, was 47); the moveit README states the new contract.

### Changed (8 Nov 2026; owner decisions, P-03 rev20.4 item 28)

The combined model now carries the values the roadmap model (openamrobot-docs branch `roadmap/2026-11-08-owner-decisions`, not yet merged) draws.
Register change openAMRobot/.github#90, change request openAMRobot/.github#89. Datums revision `2026-11-08.b`, so every `config_id` changes.

- **Head camera lens 0.060 m ahead of the arm mount** (`upperbody_configuration.yaml` `geometry.head_camera_lens.x_ahead_of_arm_mount`, was 0.042, P-03 rev20.0 item 18;
  the docs model had 0.057 on 2 Nov 2026).
- **Getac ZX80 portrait only, on a fixed mount.** `getac_tablet_size` is the portrait box `[0.0176, 0.1498, 0.234]` (was landscape), its centre 0.191 above the column top
  (was 0.246: 55 mm lower than the P-03 rev20.2 item 26 placeholder). The rotatable mount is withdrawn: `getac_tablet_portrait_size` and `getac_tablet_portrait_pivot`
  are deleted, and `tools/report_equipment_collisions.py` drops its portrait variant and assumed pivot. Head light top edge 10 mm below the Getac (`head_light_gap_below_tablet`
  0.030 -> 0.010; head light centre 0.0555 above the column top).
- **DOLD supplier CAD** (`Hexalift_EB530_Hub350_mit_Montagelaschen.STEP`, owner, 8 Nov 2026): `lift_static_profile_top_below_column_top` 0.018 (placeholder) -> 0.025
  (supplier, provisional), and it now sets the fixed stage's box (0 to 0.505 m, was 0 to 0.530); `lift_inner_section` `[0.2095, 0.051]` -> `[0.2095, 0.0533, 0.470]`
  (a triple: the stage length is no longer the stroke), its top at the carriage frame, bottom 60 mm above the column base at L1000; `lift_column_section` `[0.240, 0.060]`
  confirmed. The STEP's foot (280 x 60.4 x 10 mm) is **not modelled** (noted in the datum). The fixing pattern this entry first gave is superseded: see 10 Nov 2026 above.
- **Adapter plate one 6082-T6 part, 425 x 100 x 10 mm, with a 65 mm rear extension** behind the column rear face for the flag rod (`adapter_plate_section` `[0.360, 0.100]`
  -> `[0.425, 0.100]`, new datum `adapter_plate_rear_extension` 0.065; rear edge x -0.185 from the column axis, was flush at -0.120). The arm profile's pose is unchanged.
- **Second E-stop on the moving adapter plate** behind the arm post, above the column axis, centred in y, button up, in a 64 x 50 x 50 mm housing envelope:
  `second_estop_link` is now a child of `adapter_plate_link` (was `lift_column_base_link`, on the static profile rear face facing rearward). New datums
  `second_estop_panel_above_plate` 0.048 and `second_estop_housing_size`; `second_estop_panel_behind_profile` and `second_estop_below_profile_top` are deleted. The three
  E-stop part sizes are unchanged; their axis convention now says x is up. Geometry only, no safety logic.
- **Lift stop sensors on the rear face of the static column** (owner correction A6): sensor axes along y, 28 mm behind the rear face (x -0.148), centred in y, sensing faces
  toward +y (y 0.024); the bracket's face leg on the rear face. The flag rod sits behind them (x -0.183 to -0.163, y 0.028 to 0.048, 9 mm clear of the face discs) under the
  plate's extension, the flag (now 45 fore-aft x 2 across x 20) on the rod's side face, 2 mm off the faces. New datums `lift_stop_sensor_axis_behind_rear_face` 0.028 and
  `lift_flag_rod_clearance` 0.009; deleted `lift_stop_sensor_face_x`, `lift_stop_sensor_axis_from_face`, `lift_sensor_bracket_sensor_leg_from_back`,
  `lift_flag_offset_from_face`, `lift_flag_rod_offset_from_face`. The corner bracket the docs model draws is not modelled (LIFT-JSON `corner_x`, not exported).
- **Docs-model fixture refreshed** (`test/fixtures/docs_model_item24.json`, `merged: false`, base `8d743e2`); `tools/export_docs_model_values.py` exports the new
  CHEST-JSON and LIFT-JSON keys and equals the local model.js (`--model-js`: equal). Frame contract, READMEs and the tests that pinned the old values updated.
- **Collision matrix re-derived** (`scripts/derive_acm.py --write`, ROS 2 Jazzy with moveit_py, after the review on #48): 194 disabled pairs (20 upstream +
  29 `Adjacent` + 145 `ConstantTransform`), was 198. The second E-stop's pairs with the base, wheels, castors, LiDAR, camera, column base and stop sensors are gone (it
  rides the lift now, so they are checked); its pairs with the adapter plate (`Adjacent`), arm profile, carriage, tablet, head light, head camera and bracket, flag rod,
  flag and both arm base links are new (`ConstantTransform`). `test_acm_derive.py` and `test_m03_acm_sampling.py` expect 194 / 145.
- **Upper-body masses from the roadmap model, every part once on the link where it sits** (owner decisions 8 Nov 2026, P-03 rev20.5 item 29 and rev20.6 item 30 A6):
  `config/roadmap_mass_export.yaml` is a verbatim copy of openamrobot-docs `mass_export.yaml`, pinned by commit and SHA-256 in `config/roadmap_mass_export.source.yaml`.
  New `urdf/roadmap_inertial.xacro`: every upper-body link with mass reads its inertial from the export: `lift_column_base_link` 9.10 kg (static DOLD share 8.89 kg of the
  catalogue 11.7 kg, stop sensors, brackets, lift node), `lift_carriage_link` 6.72 kg (moving DOLD share 2.81 kg, adapter plate set, arm profile, Getac mount, second E-stop,
  flag rod and flag), `getac_tablet_link`, `head_light_link`, `head_camera_link` (ZED Mini with its mount) and the two wrist camera links. The arm links keep the upstream
  masses (the ARM-001 line). New xacro arg `roadmap_masses` (default `true`; `false` gives the datum inertials, kept for the datum tests and the TL3 fallback).
  `test/test_roadmap_masses.py` checks the expanded URDF link by link against the export, the pin, and that nothing is counted twice; the T10, S4, item 24 and T12 tests
  follow. `mass_provenance.yaml` says where each mass is. Design values, not weighed.

### Fixed (P29, 28 Oct 2026; test-only, one race and two false lines)

- **The AUX race in `test_zz_a_reading_beyond_the_epsilon_is_not_hidden` (P28 A7 run 2: 42 != 41).** Root cause: the test compared `activations` read **before** switching the injected value to -1e-3 with
  `activations` read **after**; the publisher keeps sending the in-epsilon noise (-2.5e-7) until the switch takes effect and each such message is a legitimate clamp, so one still in flight when the first number was
  read made the two differ by one. Nothing beyond the epsilon was ever counted as a clamp; the baseline was wrong, not the clamp. Fix (test only): the baseline is the snapshot in which the first beyond-epsilon
  reading has been counted (the clamp sees one publisher's messages in order, so every earlier noise message is counted by then), and a single-snapshot check that `max_magnitude_rad` is still the noise
  (2.5e-7) is added: a 1e-3 reading counted as a clamp would put 1e-3 there. **As strict as before, and stricter for a reading hidden once**: against two broken clamps (every beyond-epsilon reading also counted
  as a clamp; only the first) the new test fails on both, with the real clamp it passes, run alone 30 of 30 (`evidence/p29/`). No clamp, epsilon or margin was touched.
- **Two false lines in the design folder** (`DATUMS.md` line 64, `VERIFICATION.md` lines 75 to 77) said the column's lateral side is invented; both now say it is sourced (240 x 60 outer, 209.5 x 51 inner, DOLD Hexalift
  datasheet p.4). `VERIFICATION.md` line 78 still says V01's "0 of 478" has not been re-taken; it was (0 of 567, P28). It was outside the lines P29 may edit.
- **The ros_gz draft** (`docs/issues/ros_gz-bridge_teardown_segfault.md`) has the P25, P27 and P28 rows from the committed reports. Recorded since P24: 48 launches with the bridge, -11 in 1 (P28 run 2, G02, `parameter_bridge-3`,
  which is `platform_bridge`, not our bridge), -6 in 1 (P25 run 3, G01). Observed up to P23: 3 of 72. Reported separately, not added. Still **not filed** (no credential).
- **A blank V01 sheet for a person** (`docs/v01_visual_check.md`, last section): items 1 to 8 with the expected value and the capture that shows it; the reading and PASS/FAIL columns are empty. V01 stays WARN.
- **A7 x3 on `c84cfbd`: 14 / 2 / 0 each** (`evidence/p29/reports/`, not in `evidence/reports/`: see `evidence/p29/README.md`); no check changed result against the clean P28 run, no `parameter_bridge` -11 or -6. A6 was not re-run (test-only change).

### Changed (P28, 28 Oct 2026; Alex's column answer of 22 Oct, P-03 rev19.6 item 18)

- **The lift column's lateral side is no longer invented.** `lift_column_section` was `[0.240, 0.240]`, placeholder: x was option B's own statement and y was set equal to it. It is now
  **`[0.240, 0.060]`, source `supplier`, status `provisional`**, from the DOLD Hexalift datasheet (version 2023_08_16, page 4, the dimensioned drawings: "Outer profile 240x60";
  https://www.dold-mechatronik.de/documents/Datasheets/AluminumProfiles/TelescopicLifters/telescopic_lifters_en.pdf), and it matches option B drawing sheet 2 rev A. Provisional because
  it is read from a raster drawing in a supplier datasheet, not from CAD. The note "the one invented number of the lift" is gone with it; the `placeholder` count of the datums goes from 5 to 4
  (16 datums: 4 placeholder, 11 provisional, 1 verified). Datums revision `2026-10-28.a`, so every `config_id` changes.
- **The carriage is the Hexalift's inner stage, and the datasheet gives its section.** New datum `lift_inner_section` `[0.2095, 0.051]` (supplier, provisional; the same drawing, "Inner profile
  209.5x51"), replacing a second copy of the outer section on `lift_carriage_link`. **The placeholder case did not apply**: the section is in the datasheet, on page 4, as an embedded image the
  PDF's text layer does not contain. Taken the same way round as the outer stage and centred (the drawing does not say); its length is still the 0.350 m stroke (the datasheet gives no stage length).
  It nests in the outer section with 15.25 mm fore-aft and 4.5 mm lateral clearance each side. **It does not remove the overlap in volume**: the outer stage's collision is a solid box
  (stage overlap 0.003740 m3 at L1000, was 0.020160; `evidence/p28/v01_geometry.json`).
- **The adapter plate is its own link.** `adapter_plate_link`, **360 x 100 x 10 mm** (new datum `adapter_plate_section` `[0.360, 0.100]`, plan, provisional; Alex, 22 Oct, P-03 rev19.6 item 18; the 360 is 240 of
  column plus 120 ahead of its front face, so its rear edge is flush with the column's rear face and its front edge is the arm profile's front face), visual and collision, on `lift_carriage_link`,
  closing the 10 mm gap under the arm profile. **`arm_profile_link` is now the plate's child**, not the carriage's: the two faces touch by design, siblings that touch are a permanent collision in
  the matrix derivation (it reported `adapter_plate_link <-> arm_profile_link` and wrote nothing), a parent and its child are a structural neighbour. The profile's pose from the carriage is unchanged
  (`test_p28_lift_geometry.py`). Opt-in `adapter_plate` on option B's macro (default none, as `column_geometry`); `column_geometry` gains `inner_x` and `inner_y`.
- **Collision matrix re-derived: 99 disabled pairs** (20 upstream + 18 `Adjacent` + 61 `ConstantTransform`), was 95; **567 of 666 pairs enabled** (37 geometry-bearing links), was 535 of 630. The four new
  pairs are the plate's (with the carriage and the profile, `Adjacent`; with the two arm base links, `ConstantTransform`); `arm_profile_link` <-> `lift_carriage_link` moved from `Adjacent` to
  `ConstantTransform` because the plate now lies between them. **Nesting the inner stage moved no pair**: the two stages are a pair across `lift_joint`, a structural neighbour, disabled without being evaluated.
  M03 PASS in all three runs. The sampled information moves with the narrower column (for example `lift_column_base_link` <-> `openarm_right_ee_base_link` 64 to 35 of 2000 samples at L1000); it is information, not a result.
- **The lift mass's density band was re-fitted, and that is a judgement call.** 11.7 kg over the sourced 0.240 x 0.060 x 0.530 m is 1533 kg/m3, outside the band 100 to 1200 whose upper bound was the datum author's 40 per cent
  fill estimate over an invented square section. The band is now 100 to 1600 (reason in the datum, which says the check is weaker on the upper side); T10 pins 1533. The DOLD datasheet (page 5) lists 11.7 kg for the V4,
  the catalogue figure the datum already had.
- **Runs on `a14a3ff`, platform-sw `dfd44a5`: A6 x3 25 / 3 / 0; A7 x3 14 / 2 / 0, 12 / 2 / 2 (G02: the bridge -11 at shutdown, not allow-listed; AUX: a race in `test_zz_a_reading_beyond_the_epsilon_is_not_hidden`), 14 / 2 / 0.**
  A first attempt on `25843dc` failed H01 (the new test file was in no CMake line) and was repeated from the start on the amended commit. V01 record: `docs/v01_visual_check.md`, last section; six RViz captures in `evidence/p28/rviz/`.

### Fixed (P27, 21 Oct 2026; the three reviewer findings on PR #14 at `ba5b4ea`)

- **M07 rejects non-finite joint positions and FAILs on them (Mohammad, twice).** Before: a NaN compares false with everything, so 50 NaNs per expected joint
  counted as 50 samples with no extremum and passed, and 49 finite readings plus one NaN counted 50 and passed (`evidence/p27/m07_nan_before.txt`). Now the NaNs and
  infinities are split off BEFORE the coverage count and before any extremum (`a7_support.split_finite`), a non-finite reading never counts toward the 50-sample minimum,
  the rejected readings are kept in the sidecar (`invalid_readings`: per joint, count, kind, position), and any non-finite reading is a FAIL, never a PASS or a WARN. The
  decision is now one pure function, `a7_support.evaluate_m07`, that the launch test records; `test_m07_non_finite.py` covers both reproductions, mixed readings, a missing
  joint, +/-inf, and Alex's rule (WARN only for the four upstream-limit joints, FAIL more than 0.001 rad beyond a limit, those four included). Two boundaries of the WARN
  exception now follow Alex's point 4: a listed joint read exactly 0.001 rad past its limit is WARN ("more than 0.001 rad" is the FAIL; it was FAIL), and WARN applies only at the
  INHERITED limit (the zero end); a `joint4` within the margin of 2.4435 or a finger within it of +-0.7854 is FAIL (it was WARN: either end). No margin, model, clamp epsilon or named pose changed.
- **`test_evidence_dir` no longer requires a root-level `src/`** (Om): this repository's sources are under `ros2/src`. The workspace is found from the share path alone
  (`install` ancestor), with cases for a checkout with no `src/`, a build from `ros2/`, and a conventional workspace. **`test_build_evidence` asserts the simulation-only
  disclaimer is PRESENT, not last** (any later limitation, such as an upstream pin read from metadata, used to break it); cases: a limitation after the disclaimer passes, a
  report without it fails, a physical claim fails.
- **The red `repository-quality` check (run 39575711096).** The merge-marker scan is the organisation's (`openAMRobot/.github`, `repository-quality-reusable.yml@main`): any tracked
  line starting with seven `=` matches, and pytest's summary separator in `evidence/p24/pytest_process_exits.txt:14` did. The file is now `pytest_process_exits.md` (git mv, content
  unchanged; the scan skips `*.md`), and `test_merge_markers.py` runs the organisation's exact command over the tracked files, with canaries.

### Changed (P25, 21 Oct 2026)

- **The three placeholder boxes have one named, explicit colour.** `lift_column_base_link`, `lift_carriage_link` and `arm_profile_link` had no
  `<material>`, so RViz drew them red and Gazebo black. They now use the material `placeholder_geometry` (rgba 0.78 0.64 0.45 1.0, a flat cardboard
  tan that nothing else in the combined robot uses) in the URDF, and each link has a `<gazebo reference>` block with the same ambient and diffuse,
  because sdformat takes a link's colour from that block. **The two column visuals lost their `name` attribute** (`placeholder_lift_column_*_stage`):
  sdformat applies the `<gazebo>` material block only to a visual with no name of its own, so a named column visual kept whatever the URDF material
  converts to (1.25 times brighter in the SDF). The collisions keep their `placeholder_*` names. No pose, mount, stop height, frame or joint changed;
  `config_id` changes because the URDF text does. `test_p25_placeholder_material.py` pins the colour in the URDF, the `<gazebo>` block and the SDF
  that `gz sdf -p` produces.
- **P25 runs on `1165166`, platform-sw `dfd44a5`: A6 x3 25 / 3 / 0; A7 x3 14 / 2 / 0, nothing moved against P24** (`evidence/p25/after_material_vs_p24.txt`; the 14
  `config_id`s differ because the URDF text does). Run level: 3 of the 18 A7 runs on `dfd44a5` failed on the bridge crash, 17 percent. The V01 record of the placeholder geometry
  (one fix, four measurements waiting on Alex) is the last section of `docs/v01_visual_check.md`; the numbers are in `evidence/p25/`.

### Changed (follow-ups to 49d2368, accepted by Alex on 16 Oct)

- **The lift column has placeholder geometry.** Both stages carry a 240 x 240 mm box (datum `lift_column_section`, placeholder), visual and
  collision, named `placeholder_lift_column_*`: the fixed stage on `lift_column_base_link` (retracted length), the moving stage on
  `lift_carriage_link` (stroke, below the carriage). Opt-in parameter `column_geometry` on option B's macro (default: none, as on `main`); the
  combined robot passes the section. `frame_contract.yaml` no longer calls `lift_column_base_link` collision-free. **The collision matrix is
  re-derived: 95 pairs (20 upstream + 17 `Adjacent` + 58 `ConstantTransform`), was 83; 535 of 630 pairs are enabled, was 478** (the two stages
  are geometry links now). The arms can no longer plan through the column body; the box is not the supplier's shape.
- **M07 reports WARN, not FAIL, for the four joints whose rest pose is an upstream limit** (`openarm_{left,right}_finger_joint1`,
  `openarm_{left,right}_joint4`, reason "rest pose at an upstream limit"), within the margin on either side of that limit. Any other joint within the
  margin, or one of those read further past its limit than the margin, is still a FAIL; WARN is not PASS.
- **Controller start-up in `sim.launch.py`: the joint_state_broadcaster spawner has a 30 s switch timeout (`--switch-timeout`, with
  `--service-call-timeout 45` so the spawner's 10 s default call timeout does not end the wait first), and the arm and gripper spawners start only
  when it exits 0.** On a 4-core machine with CPU-only rendering the broadcaster's first switch timed out after the
  spawner's default 5 s, the spawner exited 1, and the other four spawners still started (G01, G02, M02 and M07 then saw four of five controllers
  active). `test_sim_launch_spawners.py` pins both.
- **The A7 controller wait honours one absolute deadline** (PR #14 review): `a7_support.wait_for_controllers_active` caps the service wait,
  each request, each spin and each sleep by the time that is left, and accepts an all-active answer only if it arrived before the deadline (before,
  an answer at 1.5 s of a 1.0 s budget returned ok). M07 uses the shared wait instead of its own copy. `test_a7_controller_deadline.py` (fake clock
  and client) pins it.
- **M07 fails on incomplete joint-state coverage** (PR #14 review): the expected joints come from the model (the 16 limited joints under
  `ros2_control`; `lift_joint` and the mimic `finger_joint2`s are recorded as not measured, with the reason), and every expected joint needs 50
  samples in the window; a missing or under-sampled joint is a FAIL recorded as incomplete coverage. `test_m07_coverage.py` pins it.

### Added (P24): the exit code of every launch process is recorded

- **`launch_guard.assert_launch_healthy(record=...)`** writes `{process, exit_code}` for every process of the launch into the sidecar of the check whose launch it was (G01, G02, M01, M02, M07: key `process_exits`),
  before it asserts, so a launch that fails the check still leaves its exit codes. `build_a7_evidence.py` lists them per launch as `process_exits` in the A7 report. Nothing is allow-listed or widened: `parameter_bridge` -11 is still a
  FAIL (a test pins it). The processes are read by action, not by name (`proc_info["spawner-1"]` is a substring match).
- **The bridge exit rate can now be BACKED, for new runs only.** `tools/check_evidence_backing.py` has the ledger kind `launch_exits`: "N of M launches with a `parameter_bridge` ended in -11" derives from `process_exits` (reports before P24 contribute no launch).
  Canary: a synthetic -11 in a sidecar comes out of the report as a rate the check backs, and a different number is a MISMATCH (`test_a7_process_exits.py`, `check_evidence_backing.py --canary`). **The three past occurrences stay OBSERVED-NOT-RECORDED**: they cannot be recovered.
- **P24 runs on `15109f9`, platform-sw `dfd44a5`: A6 x3 25 / 3 / 0; A7 x3 14 / 2 / 0.** Recorded: 12 launches with a `parameter_bridge` (two per launch), 0 ended in -11. Run-level: 3 of the 12 A7 runs of P19 to P23 on `dfd44a5` failed on this crash, 25 percent, across two different
  launches (G02 twice, M07 once); with P24's three clean runs, 3 of 15, 20 percent.
- **PR body**: Mohammad's words on the column finding are quoted (his review of `9406af6`), the run-level rate is stated, and the P23 sentence that the statement "did not describe the model" is corrected: for the P18 reports he reviewed it was true.

### Changed (P23): the false column statement, the datums citation, the bridge rate

- **The report generators no longer say the lift column has no collision geometry.** `build_a7_evidence.py` (the limitation) and `build_evidence.py` (the limitation, and the V01 item 1 note
  "nothing is drawn for the column") now say what `b45d8f6` made true: a 240 x 240 mm placeholder box, visual and collision, on each stage, whose lateral side is invented. The old sentence was a
  generator string and not a finding. It is in 56 committed reports: true in the 32 of P15 to P18, **false in the 24 of P19 to P21** (`20261018T191032Z_dev.json` to `20261019T010818Z_a7.json`). None is edited;
  `evidence/reports/README.md`, section P23, lists them. Mohammad's review of `9406af6` on #14 (on GitHub, quoted in the PR body) cited it; for the P18 reports he reviewed it was true, and P24 corrected the P23 wording that said it "did not describe the model".
- **`datums.yaml` re-cited; datums revision `2026-10-20.a`; every `config_id` changed.** `base_link_height_above_floor` cited FRAMES-REP105 at P-03 rev19.0; 0.098 m is in the register's CAMERAS at
  rev19.4 (`decisions.yaml:600`). The value is unchanged; the `ref` and the `lift_column_section` note (which still said "no collision, no visual") changed, so the revision moved and with it all 14 ids of the A6
  reports and all 9 of the A7 reports. An id recorded under another datums revision is not comparable, which the existing non-comparability note (Satya, C9; the PR body, "`config_id` inputs changed") already says; no input was added.
  `tools/check_decision_currency.py`: **0 STALE** (31 CURRENT, 3 HISTORICAL), was 1.
- **The bridge exit rate is OBSERVED-NOT-RECORDED.** The A7 JSON does not carry process exit codes, so "2 of 60" was as unbacked as the retracted "1 of 7". Chosen: print it as OBSERVED-NOT-RECORDED everywhere
  it appears (PR body, this file, the reports index, the issue draft, the design folder) and have `check_evidence_backing.py` accept such a rate only while the text on its line says so. Not chosen: recording
  the exit code in the stop-helper sidecar, which would make the next occurrence data but recover none of the past ones. **A third occurrence came with the P23 evidence run**: run 1 of 3 on `77374a0`, in M07's launch
  (the first two were G02's); M07 and AUX FAILed, the console tail is kept in `evidence/p23/a7_run1_console_tail.txt`. Now 3 of 72 launches since P18, 3 of 48 on `dfd44a5`, 3 of 12 A7 runs on that pin. Nothing was allow-listed.
- **A6 x3: 25 PASS / 3 WARN / 0 FAIL. A7 x3: 14 / 2 / 0 twice and 13 / 1 / 2 once**, platform-sw `dfd44a5`, tested code `77374a0` (`evidence/reports/20261020T*`).
- **`openamrobot-manipulation` for the P19 to P23 runs is `795143e`**, not `095e6c7` (Alex's container): the local checkout has one reflog entry, the clone of 5 Oct (`evidence/p23/workspace_checkouts.txt`), and `095e6c7`
  is two docs-only commits ahead (GitHub compare). The PR body is corrected.
- `tools/check_evidence_backing.py`: bare `a / b / c` triples followed by a comma were not scanned (a hole in the P22 pattern); they are now, and a triple that equals a committed report's PASS / WARN / FAIL is backed.
  `check_decision_currency.py`: the `scan` globs are anchored (`README.md` meant every README).

### Added / Changed (P22): audit of the sources of truth; two prototype checks

- **Documents brought into line with the register (`openAMRobot/.github` `decisions.yaml`, P-03 rev19.6) and with what the repository decided in P17 to P21.** Design folder
  (outside this repository): `DESIGN_LOG.md` (a 19 Oct entry for M07 as Alex's WARN rule, W03 demoted to evidence, W04 and its NOT_RUN fix, the `dfd44a5` pin, the T08
  attribution, the `parameter_bridge` SIGSEGV, the retracted "1 of 7"), `VERIFICATION.md`, `DATUMS.md`, `FRAMES.md`, `decisions/ADR-010`, `00_INDEX.md`, `ARCHITECTURE.md` section 7.
  In this repository: the description and MoveIt READMEs (the column has placeholder geometry since `b45d8f6`; 14 datums, 5 placeholder), `docs/verification.md` (M07 row, column
  row, enabled pairs, the WARN table), `docs/pr/PR14_BODY.md`.
- **T08 is why A6 went from 24 PASS / 4 WARN to 25 / 3, and the cause is not in this repository.** It had listed six platform-sw links whose collision mesh is their visual STL;
  platform-sw #70 (`d619012`) replaced them with boxes and cylinders. No commit here changed T08 or its inputs (`evidence/reports/README.md`, P20). Recorded here because the changelog did not say it.
- **Two prototype checks, run by hand and wired into no CI** (`tools/README.md`): `tools/check_decision_currency.py` fails when one of our documents cites a register decision at a
  revision other than the one the register records (`docs/decision_citations.yaml`), and `tools/check_evidence_backing.py` requires every count and rate in `docs/pr/PR14_BODY.md`
  and in the newest generated reports to resolve to a file under `evidence/` or be printed as UNBACKED (`docs/pr/claim_backing.yaml` says which are acknowledged and why). Each has a
  `--canary` that plants faults in temporary copies and shows they are caught. They are the reaction to the "about 1 of 7" rate (below, P21), which would have been printed as UNBACKED.
- **The backing check found a wrong count on its first run.** `evidence/reports/README.md` (P17 table) and Tushar's text in `docs/pr/PR14_BODY.md` said 13 PASS for A7 runs 2 and 3 on
  `013b524`; the two reports (`20261015T121048Z_a7.json`, `20261015T121946Z_a7.json`) have 15 checks, 12 of them PASS. Both places now say so (the second as a bracketed note, because
  that text is kept as written).
- **Left as they are on purpose** (each would change a generated report or the datums revision, so every `config_id`): the note of `lift_column_section` in `datums.yaml` ("no collision, no
  visual, no joint") and the `ref` of `base_link_height_above_floor` (cites FRAMES-REP105 at rev19.0; the register states 0.098 m in CAMERAS at rev19.4); the limitation texts
  "the lift column has no collision geometry" in `build_evidence.py` and `build_a7_evidence.py`, which every committed A6 and A7 report from P19 on repeats; `docs/v01_visual_check.md`, a record of
  the P16 measurement on the earlier geometry. The decision-currency check lists the citations it finds stale.

### Changed (P21): the review items on #14

- **W04 now returns NOT_RUN, never PASS, on evidence that is not there** (Mohammad's inline finding on `9406af6`). The stop helper's report could still be `{"state": "started"}`
  when `read_gz_stop_report` gave up waiting, and `gz_server_shutdown_report` kept that object in the sidecar; W04 saw a truthy report that was not `force_stopped` and returned PASS,
  and returned PASS for a report with no state or an unrecognised one as well. It now accepts `stopped` (PASS) and `force_stopped` (WARN) and nothing else: `started`, no state, an
  unknown or non-string state, a malformed or absent report, and `no_server` (the helper found no server, which shows nothing about one stopping on request) are NOT_RUN. **This is the
  H01/H02 class of hole, a check passing when its evidence is missing, reintroduced in a check we wrote ourselves**; the absent-helper case was covered and the timed-out one was not.
  `test_a7_w04.py` drives the timed-out path end to end (a report left in `started` through `read_gz_stop_report` and `gz_server_shutdown_report` into W04) and the malformed forms;
  six of the new tests fail on the old code. The survivor check is untouched.
- **M03 regression for platform-sw #71** (`test_m03_caster_clearance.py`, no ROS graph): the old chassis box (bottom 0.050 m) overlaps the caster-wheel spheres (top 0.052421 m) by
  2.421 mm and `acm_derive.derive` reports four permanent collisions; the pinned box (bottom 0.055 m) clears them by 2.579 mm. Fails on platform-sw `d619012`, passes on `dfd44a5`.
- **`config_id`: keep the payload.** The description README now says that the digest is of the compact sorted-key JSON payload `print_config_id.py` hashes; to use it with C10, retain that
  exact payload, not merely the raw URDF/Xacro file (Satya's review). The lift stops are calibration IDs, not configuration axes (ADR-010 as corrected by rev19.0).
- **The 1-in-7 figure was an unbacked claim.** A report to the architecture chat in P16 said `parameter_bridge` exits -11 at shutdown in about 1 of 7 G02 runs. The repository has no
  such record: no -11 bridge exit in any P16 or P17 report or log (the bridge exits recorded before P19 are SIGABRT; one G02 FAIL in P17's 36 runs was a JSON-parsing bug in the test).
  **The only observed rate is 2 of 60 launches since P18, and it is OBSERVED-NOT-RECORDED** (see P23 below). Recorded here so the claim is not quietly dropped.
- `PR14_BODY.md`: states why W03 does not replace M07 (W03 is static, M07 dynamic; "a static named-pose report does not establish dynamic sample coverage"), that M07 is Alex's as
  written, and that **one agreed rest-state rule, including what happens to missing joint-state evidence, is still pending with Alex and Om**.

### Changed (P18, P19): W04 added; W03 is evidence only; M07 stays

- **M07 stays, as Alex wrote it** (WARN on the four upstream-limit joints, `joint_rest_margin_rad` and its datums revision bump). P18 had withdrawn it;
  Alex's decision on it stands (P19 merge), so P19 restored it exactly. If Alex accepts the withdrawal, removing M07 again is one commit. The reasons
  P18 gave stay on the record: P17's own measurement refuted the premise that the new body puts the right finger on a limit where the old body does
  not (same effort sign in both bodies; the old body also rests within 2.5e-5 rad of the same four limits).
- **W03 is evidence files, not a check.** The named poses of the SRDF that coincide with a joint limit, with the distance, for the old mast body
  (`7daa2ab`) and the lift body: the same four joints (both `joint4`, both `finger_joint1`; eight poses) in `evidence/w03/`, produced by
  `python -m openamrobot_upperbody_moveit.named_poses <srdf> <urdf>`. It is not an entry of the A7 report and writes no sidecar while M07 exists.
- **W04** (A7 report, WARN): a `gz sim` server that the launch's stop helper had to force. It was a FAIL in P17. It reads the sidecars of G01, G02, M02
  and M07. **The survivor check is unchanged and runs after the force-stop**: any `gz sim` of the run still alive when it looks (it waits up to 10 s,
  the helper needs at most about 9 s) is a survivor and a FAIL, including one that survived its own force-stop; a test pins both.
- **platform-sw is pinned at `dfd44a5`** (#70 + #71; P20). A second upstream issue is drafted, `docs/issues/ros_gz-bridge_teardown_segfault.md` (`parameter_bridge` SIGSEGV at
  shutdown, 2 of 36 launches on that pin, OBSERVED-NOT-RECORDED); **not filed**. The process guard still does not allow SIGSEGV for `parameter_bridge`.
- **RViz screenshots** at the three lift stops x two base plates, from real RViz2 (`evidence/renders/rviz_*.png`, `evidence/renders/rviz/` for the launch, config and window grab that made them).
- `docs/pr/PR1_BODY.md` is deleted; `docs/pr/PR14_BODY.md` is the body of #14 and carries the open findings.
- The gz-sim issue (segfault in a libgz-sim8 thread during teardown; the hang that ignores SIGTERM) is drafted in `docs/issues/` with the counts and a
  reproduction. **It is not filed: the workspace has no GitHub credential.** A bare server (empty world, no ROS) did not reproduce it: 30 of 30 clean.

### Added / Changed / Fixed (P17)

- **The simulator is stopped once per launch, by a detached helper, and a server that has to be forced is a FAIL.** `sim.launch.py`'s `OnShutdown`
  fired twice on every shutdown (the shell's death is a second `Shutdown`) and re-executed the same `ExecuteProcess`: "executed more than once",
  which abandoned the rest of the shutdown; on the `ros2 launch` + SIGINT path the stop request was not started at all (launch refuses to start an
  action once `context.is_shutdown` is set). Measured before changing anything (`launch/sim.launch.py` header has the numbers): launch signals the
  `/bin/sh` around `gz sim`, never the server; the `/server_control` request went wrong 4 times in 18 launches (two server segfaults inside
  libgz-sim8 during teardown, two servers that accepted the request, never began teardown and ignored SIGTERM, one of which was orphaned by
  launch's SIGTERM of the shell at 5 s). SIGINT sent to the server itself, as Ctrl-C does, went wrong 0 times in 24 launches. The helper now does:
  SIGINT to the `gz sim` processes; the service request as well after 3 s if one is still there; SIGTERM then SIGKILL after 8 s; servers followed by
  pid (a first version walked the process tree and called a reparented server "gone"; found by the driver, not by a test). Its report is written
  before it sends anything harder than SIGINT and `launch_guard.gz_server_shutdown_report` reads it: a forced server is a survivor even though it
  is gone. **The ORPH detector is unchanged for a server that is still running and no allowance was added.** Not found: why a server hangs in its
  teardown after being asked to stop (inside gz-sim / gz_ros2_control). The force-stop path is proven on decoy processes only; it was never
  exercised live (36 of 36 runs stopped on the first step).
- **M07, the rest-position check** (`test_m07_rest_position_margin.py`, A7): no joint's rest position may be within `joint_rest_margin_rad` of
  either limit; FAIL, not WARN. *(Superseded: P18 withdrew it, and P19 restored it as Alex's rule, WARN for the four joints whose rest pose is an upstream limit and FAIL otherwise; see "Changed (P18, P19)" above. The "FAILs today" below is the P17 state.)* The margin is a datum (`datums.yaml`, `placeholder`, 1e-3 rad, report-only; revision bumped, so every
  `config_id` changes), taken from the between-run scatter of a rest position (up to 2.8e-4 rad) and not tuned to a verdict. **It FAILs today** on
  `openarm_right_finger_joint1` (rests at +2.6e-8 rad, past its open limit), and on `openarm_left_finger_joint1` and both `joint4`; `lift_joint`
  is listed as not measured, with the reason.
- **M02 gates on exactly two numbers**: readings beyond epsilon (0) and the largest overshoot magnitude (below 1e-5). Activation and episode
  counts are reported under `diagnostic_only`, labelled, and gate nothing. The epsilon is not widened. **M02 waits for the five controllers to be
  `active`** (120 s, FAIL naming their states) before it sends the first pose.
- **G02's own helper no longer fails on two JSON messages** (`gz topic -n 1` printed two; `json.loads` raised "Extra data": 1 of 12 live G02 runs).
- **Finger effort at rest was read in both trees** (`evidence/m02_bisect/RESULTS.md`, last section): the sign did not change between the bodies,
  which refutes P16's hypothesis; the old body also rests within 2.5e-5 rad of the same four limits; why the new right finger sits at the limit
  and the left one does not is not answered.
- **Evidence pass on `013b524`** (`evidence/reports/README.md`, P17): A6 24 PASS / 4 WARN / 0 FAIL, three runs (the first pass, on `a713533`, failed
  3 of 3: the helper's report was in the system temp directory, which `test_evidence_dir.py` forbids; fixed). A7: M07 FAILs in all three runs by
  design (and AUX with it); run 1 also failed G01 because its server ignored SIGINT, the service request and SIGTERM and was SIGKILLed by the
  helper at 9.2 s: the force-stop path, seen live once in 49 launches, leaving nothing running. **0 orphans in 12 runs each of G01, G02 and M02
  after the fix (natural rate before: about 1 in 13)** is a count, not a claim that the hang is gone. `test_a7_controller_wait.py` pins M02's
  controller wait and its FAIL, against a fake controller manager.
- `tools/compare_option_b_wrapper.sh`: the byte-level comparison of option B's wrapper against `upstream/main` that the PR body cites.

### Added / Changed (P16)

- **G02, G01 and M02 fail when a `gz sim` of their own partition outlives their launch.** (G02 first; G01 and M02 after a natural orphan of G01's
  launch degraded the rest of an A7 run in the second evidence pass.) The ruby wrapper is what the launch owns and what the process guard reads;
  the server it forks was left running after about 2 of 8 A7 runs and nothing noticed. G02's post-shutdown phase now looks (within a 10 s grace
  period) for any `gz sim` whose environment carries the test's own `GZ_PARTITION`; a survivor is a FAIL (not a WARN), judged before it is
  signalled and only then stopped, and written into G02's sidecar (`details.gz_server_after_shutdown`). Chosen over a new G03 because the orphan
  is a property of G02's own teardown and a G03 would need a second simulator launch of its own to observe anything. Proven by a decoy-process
  unit test, a canary (`test/canary_gz_orphan.sh`: the real G02 against a launch without the `/server_control` stop handler, which must fail naming
  the survivor) and one natural orphan found in the first run of the session, before the check existed. **Not proven on a natural orphan with the
  check in place: 0 of 12 live runs produced one** (`evidence/reports/README.md`, P16). M02's launch leaves a simulator behind too after a failed
  run (the check now covers it).
- **The eight V01 items of the lift model are measured** (`docs/v01_visual_check.md`, `evidence/v01/`), none reads outside its expected value, two
  sub-questions that need eyes are CANNOT-MEASURE, and V01 stays WARN because `lift_mass_tl3_400` is null. Renders per lift stop and an isometric
  are in `evidence/renders/`.
- **M02's clamp rise was bisected and the cause was not found** (`evidence/m02_bisect/RESULTS.md`): the ACM, the mass, the arm separation and the
  open command are each ruled out, and so is platform-sw's new base (old body on the new base reads 188 and 216); the rise is
  `right_finger_joint1` resting on its open limit in the new body. The epsilon is untouched.

### Fixed (P15, task 0)

- **`joint_state_clamp.py` exits 0 on SIGINT and SIGTERM.** P14 ran A7 twice on an unchanged commit and got 2 FAIL, then 0 FAIL; the cause was
  this node, ours, exiting 1 at shutdown. Reproduced standalone against a flooded `/joint_states` before fixing: every SIGTERM exited 1 (30 of
  30, an uncaught `ExternalShutdownException`, which rclpy raises out of `spin` for every shutdown) and 1 of 30 SIGINTs exited 1 with the
  `take_message` `RuntimeError` the signal provokes when it lands between the wait set and the take. `run()` now treats `KeyboardInterrupt`,
  `ExternalShutdownException` and that `RuntimeError` **with a closed context** as a clean stop; the same error with a live context, and
  anything else, still raises. The node is destroyed before `rclpy.try_shutdown()`. Not allow-listed: `launch_guard.py` is unchanged.
  `test_joint_state_clamp_shutdown.py` sends real signals to a real node under a message flood (against the old script it fails for
  both signals: SIGTERM on the first attempt, SIGINT on the third) and unit-tests the decision with an injected `spin`.

### Changed (P15, rebase onto main and adopt option B)

- **Both branches are rebased onto `upstream/main`**, where Alex's option B (two commits, authorship intact) and the rev19.0 column position are
  merged. Nobody's commits were re-authored. Main won for everything describing the upper body or the baseline (README wording, package
  descriptions, his config, xacro, launch file and tests, all byte-identical to main afterwards); ours won for the arm composition, control
  profiles, Gazebo, MoveIt and the harness; manifests and CMake took both.
- **The mast is deleted and the arms are rooted on option B's mount frames.** `urdf/mast.xacro`, `config/mast_ids.yaml` and the datums
  `mast_base_pose`, `left_arm_mount_pose`, `right_arm_mount_pose`, `mast_section`, `mast_mass` and `head_camera_pose` are gone; `mast_id` is not
  an argument. `combined_robot.xacro` composes option B's macro on platform-sw's `base_footprint` and calls the upstream arm macros at his
  `openarm_{left,right}_base_link` pose (`geometry.arm_mount`: x +0.180, y +-0.031, z +0.166 from the carriage). ADR-008 is otherwise unchanged:
  two calls, derived single-arm presets, no OpenArm body. Two things had to give, both recorded in the package README: option B's macro
  gains two opt-in parameters (`emit_arm_mounts`, `column_inertial`, defaults = its own behaviour) because the upstream arm macro emits a link
  and a joint of the same names as his mount frames; and the arms are placed by the macro `openarm_v20_robot` itself calls, because that macro
  declares its `root_frame` as a link and `lift_carriage_link` is already his.
- **rev19.0 numbers**: two base-plate positions (`bp000`, `bp050`; ADR-010's five were rev18.7); the column axis at x +0.060 / +0.110 from
  `base_link`, not on the drive axle (`column_axis_x_bp000`); `base_link_height_above_floor` 0.098 (a placeholder since S3, now
  provisional with a source), asserted against platform-sw's `base_footprint_joint`; the lift is a DOLD Hexalift V4 350 of **11.7 kg**
  catalogue (`lift_mass_dold_v4_350`, provisional) on `lift_column_base_link`, replacing `mast_mass` (3.0 kg, placeholder).
- **Three things option B does better, adopted**: `effort="0" velocity="0"` on a joint we have no authority to command (kept, and
  asserted); a null datum is a hard-fail that names itself (`lift_mass_tl3_400`, as its `tl3_400` lengths; the schema allows `null`
  only with `source` and `status` `placeholder`); and **calibration IDs are not configuration IDs** (the lift stops `L1000`, `L1175`,
  `L1350` are joint positions; T05 and E01 are per stop; they are reported beside the config ids, not inside them).
- **`config_id` changed meaning and kept its format.** Before: canonical URDF, `mast_id`, `arm_package`, `hardware_profile`, datums
  revision, `openarm_description` commit. Now: canonical URDF, `column`, `base_plate`, `head_pitch`, `base_tilt`, `arm_package`,
  `hardware_profile`, datums revision, every upstream pin. It still validates against C9, so nothing fails to say that an id recorded
  before this is not comparable with one computed now. The frozen A6 set (`evidence/frozen_a6_993b000/`) keeps the old inputs.
- **The near-field frames leave our contract.** The six declared-but-absent frames (`tof_front_left_link` ... `ultrasonic_rear_link`, `req: W`,
  T04's WARN since S4) are removed, not renamed. platform-sw declares `us_front_left`, `us_front_right`, `us_rear_left`, `us_rear_right`
  (4 x MB7060) and `tof_front`, `tof_rear` (2 x TF-Luna) in 935efc7, merged on its main; the contract says they are `consumed_from` there and
  T04 reports which of them each expansion contains, without a WARN or a FAIL. T04 is PASS by deletion.
- **platform-sw 935efc7 (REP 105) is followed**: the root is `base_footprint`, `base_link` is 0.098 m above it; the planar joint's child, the
  SRDF virtual joint and G01's odom frame (`odom -> base_footprint`, read from the DiffDrive plugin) follow. W01 is PASS: the root carries no
  inertial, so KDL sees `base_link`'s 4.18 kg (was a WARN since S3).
- **The two rules V01 found are implemented**: a dimension list declares `axis_convention` and T12 asserts the assignment against an
  expansion (`lift_column_section`: x fore-aft, y lateral); a mass of ours declares a `density_band` and T10 reports its implied density,
  scoped to our own masses (upstream arm `link2` implies 297 kg/m3 over its bounding box, below the old mast's 337, so a global band cannot
  work). The one placeholder of the lift, the column's lateral side, is the one invented number, set equal to the sourced 240 mm side.
- **The collision matrix is re-derived: 83 pairs (20 upstream + 18 `Adjacent` + 45 `ConstantTransform`), was 113 (20 + 16 + 77).** The
  by-construction set shrinks because a prismatic joint now lies between the base and the arms; 478 pairs are enabled (448). M03 holds
  `lift_joint` at each stop and samples the rest: PASS at all three. M04's fixture is found again against `arm_profile_link`; the base body is
  still out of the arms' reach (0 of 6000). M01 runs at three lift stops (was four mast IDs), M02 at the baseline with the lift at `L1000`.
- **Moved numbers, each with its reason**: A6 22 PASS / 6 WARN becomes 24 / 4 (T04 by deletion, W01 by the root); modelled mass 21.96 kg
  becomes 30.66 kg (the lift's 11.7 kg replaces the mast's 3.0 kg); E01 heights 1.5 m becomes 1.223 / 1.398 / 1.573 m per stop, depth
  0.686 m becomes 0.780 m (platform-sw's `us_*`/`tof_*` frames sit on the chassis edge) and width 0.505 m becomes 0.482 m (the wrist cameras
  are 62 mm apart at their bases now, so the wheels set the width); datums 15 (8 placeholder) become 13 (4 placeholder, 8 provisional).
  `docs/verification.md` and the package READMEs carry the rest.
- **V01 stays WARN**, with every checklist item stating what to measure. P14's look at the mast is kept as history: V01-F1 was confirmed
  correct by the design owner and V01-F2 is closed by the 11.7 kg catalogue figure; none of the eight items of the lift model has an answer.
- `base_joint_states_stub.py` reports `lift_joint` at `lift_position` on mock (robot_state_publisher publishes nothing above a joint nobody
  reports); on Gazebo `joint_state_publisher` reports it at zero. H01 learns `ament_add_ros_isolated_pytest_test`, which option B uses.

### Changed (P14, publish and record)

- **V01 records what the first manual look found, and stays WARN.** A person looked at the mast (item 1) and found two defects no automated
  check was examining: the mast profile is probably rotated 90 degrees (`mast_section` `[0.060, 0.120]` is 60 mm fore-aft; option B orients the
  same HFS6-60120 cross-section 120 mm fore-aft; the value was right and its axis assignment was only a prose note no test asserted), and
  `mast_mass = 3.0` kg is implausible for 1.2365 m of that profile (about 337 kg/m3 over the bounding box, low by roughly 2.5 to 4 times; an
  estimate, the datasheet is the source). The dimensions themselves are a real HFS6-60120 profile 14 mm from Alex's 1250 mm. The reports of
  all three profiles carry the findings (`details.findings`, `answered_items`, `unanswered_items`) and a limitation line; V01 is not PASS (items
  2 to 8 have no recorded answer) and is not recorded as having found nothing. Neither defect is fixed: both go with `mast.xacro` when option B
  is adopted. Two rules are proposed to the design owners and implemented nowhere: a dimension-pair or -triple datum declares its axis
  convention with a test asserting it, and a mass is checked for plausible implied density, placeholders included. `docs/v01_visual_check.md`
  has the status, and item 1 no longer lists "a thin spike" as a wrong thing to see (a 60 x 120 x 1236.5 mm section reads as one from the side).
- **The A6 expansion set is published** under `evidence/frozen_a6_993b000/`: the 14 expansions of the contract matrix at `993b000`, each with
  its configuration id (plain text, 487 KB), an index with the commit, datums revision and pins, and `verify_config_ids.py`, which recomputes
  every id from the files with the standard library only. It is the **fixed-mast** configuration (P-03 rev18.2) that option B supersedes, and
  says so. `mast_1350/openarm/gazebo` is the only id that differs between the A6 and A7 heads (A7 adds the `gz_ros2_control` model plugin); its
  A7 expansion is kept apart in `a7_variant/`. No configuration id was changed by this step.

### Changed (P12A, the harness and the matrix)

- **The evidence harness can no longer lie by omission.** Failures planted in each of the five packages (`scripts/harness_canary.py`) showed that the control package's tests were run and never read
  (exit 3, 0 FAIL, ten failing canaries), that a result under a check ID no report lists was dropped, that `colcon test` exits 0 whatever its tests did, that a sidecar with no test behind it was a PASS, and that a process
  of a launch test that dies of SIGSEGV or cannot start leaves the test passing. `NOT_RUN` is a new result value (never PASS, fails the exit status); every registered test must leave a result file (H02); every launch
  test checks that its processes started, that none died while the tests ran, and how they exited (H01, `launch_guard.py`). The A6 and A7 numbers survived (22/6/0 and 13/1/0 with the two new checks); the guard found
  a `parameter_bridge` abort, a `joint_state_publisher` exit 1 and a left-over gz server at shutdown, and the A7-gated M01/M02 launch tests failed two of three runs in a sample of three (they raced their own empty launch) while the A6 report dropped those failures.
- **The collision matrix disables only what can be justified, and is derived, not sampled.** 113 pairs: 20 declared by upstream, 36 `Adjacent`, 77 `ConstantTransform`; 448 enabled. The 458 sampled `Never` pairs are
  gone (upstream declares none; every one was written by our own Setup Assistant run). M03 recomputes the reasons and has no WARN class; the sampled part is information about the enabled pairs. Regenerating gives the
  contract back every time. The named-pose plans take about 5 to 9 per cent longer offline and all still plan and execute.
- The top-level `warnings` list of the A7 report is gone (every WARN has a check ID); a reduced M01 is a WARN. No reason code the library never returns is quoted anywhere.

### Changed (P11, A7 close-out)

- `openamrobot_upperbody_moveit`: M03 splits the collision matrix by who owns the claim. A pair whose links are both upstream's is
  *inherited* and a collision there is a WARN (pair, seed, sample index); every other pair is *ours* and a collision is a FAIL. The split is
  derived from the frame contract's owner column and the `openarm_` prefix, and the kinematic tree: our pairs with no joint between their
  links (77) are certified by construction with one evaluation, the rest (267) are sampled, and the sampled result is worded "no counterexample
  in N samples at seed S", never "validated". The P10 mutation test never worked (it relabelled a pair that does not collide); the new
  mutations use pairs measured to collide.
- The collision matrix is a checked-in contract with a recorded fingerprint; `regenerate_acm.sh` proposes and prints the diff, `--write`
  replaces it. `collisions_updater` has no seed, so regenerating is not a verification step.
- `joint_state_clamp.py`: `move_group` plans from `/joint_states_clamped`, the simulated joint state with float noise past a limit (at most
  1e-5 rad; measured up to 9.9e-7) clamped to the limit, logged and counted. Nothing else reads it, no limit was widened and no pose moved. The
  start-state retry loop of M01/M02 is removed (it never fired, and it was keyed on a code MoveIt does not return here); M01/M02 and the report
  carry the clamp's counters.
- M04's wording follows VERIFICATION.md (nearest reachable volume), M05 fails the single-number version in both directions, and M06's
  conclusion states both halves (the stop condition is retired; the limits remain placeholders).

### Added

- `openamrobot_upperbody_description`: the canonical OpenAMRobot 2.0 robot description (A6). One xacro entry point,
  `combined_robot.xacro`, composes the platform base (unchanged), a fixed mast selected by `mast_id` (`mast_1300`,
  `mast_1350` release, `mast_1400`, `mast_1450`) and two upstream OpenArm v2.0 arms rooted on mast plate links, with
  `arm_package` (`openarm`, `so101` stub, `none`), `hardware_profile` (`mock`, `gazebo`, `real`: selects the arm `ros2_control`
  plugin, see below) and `enable_planar_base`
  (a `planar` joint `world -> base_link`, present and disabled by default, axis `0 0 1`).
- Head camera and wrist camera frames as plain frames with a visual marker (the ZED wrapper is deferred).
- `config/datums.yaml` (15 datums with provenance: 8 placeholder, 6 provisional, 1 verified), its JSON schema, and
  `config/frame_contract.yaml`, the machine-readable frame and joint contract that the tests enforce.
- Verification suite and `verify_a6.sh`, which writes an `openamr.evidence.v1` report: expansion matrix, `check_urdf`,
  single root, frame contract, shoulder height per mast ID, argument validation, collision and inertia checks, mass report,
  planar joint, datum rules, configuration ids (`sha256:` plus 64 hex characters, matching the C9 dataset schema),
  installed-versus-source check, and warnings for the envelope (E01), KDL root inertia (W01), the planar joint under KDL
  (W02) and the missing visual inspection (V01). T14-T16 and L01 cover `ros2_control` (next entries).
- `view.launch.py` (in `openamrobot_upperbody_bringup`): robot_state_publisher, joint_state_publisher_gui and RViz2 for any
  mast ID. It started in the description package and moved to bringup with the mock bringup, with no forwarder.
- Arm `ros2_control` profiles (P06): `urdf/ros2_control.xacro`, `openamr_arm_ros2_control(hardware_profile)`, for
  `arm_package:=openarm` only. `mock` (`mock_components/GenericSystem`) and `real` (`openarm_hardware/OpenArmHW`, CAN-FD) call
  the upstream bimanual macro as upstream's own top-level v2.0 file does; `gazebo` is our own block with
  `gz_ros2_control/GazeboSimSystem` for the same joints and interfaces. The mobile base has no `ros2_control` block in any
  profile. Checks T14, T15, T16 (expansion only; nothing loads the Gazebo or real plugin).
- `openamrobot_upperbody_control/config/arm_controllers.yaml`: controller manager at 100 Hz, `joint_state_broadcaster`, a
  `JointTrajectoryController` per arm and a gripper controller per arm, one file for all three profiles. Derived from
  `enactic/openarm_ros2` at `4e837e1` (Apache-2.0); the header lists the changes.
- `openamrobot_upperbody_bringup/launch/mock.launch.py`: robot_state_publisher, `ros2_control_node` and the five spawners on
  mock hardware (args `mast_id`, `rviz`). Check L01 (`launch_testing`): all five controllers `active` within 30 s and one
  trajectory on `left_arm_controller` followed within 0.02 rad. Mock hardware reflects commands as state, so this shows the
  wiring, not tracking performance.
- `verify_a6.sh` also runs the control and bringup tests, and the evidence report reads the bringup results for L01.
- A6 evidence has no SKIP: T14-T16 and L01 are results, under `mock_components/GenericSystem` (19 PASS, 6 WARN, 0 SKIP,
  0 FAIL at `2083c76`). That is weaker than hardware, and `evidence_class` stays `static+mock`: no controller has driven a
  motor.
- `config/upstream_pins.yaml`: the config id falls back to this pinned `openarm_description` commit when there is no git
  checkout (a checkout with local changes is still an error); a test keeps it equal to the checkout and the baseline pin.
- T02 also runs `check_urdf` on `gazebo` and `real` at `mast_1300` (labelled extras).
- `.gitattributes`: `*.sh text eol=lf`, so `verify_a6.sh` cannot pick up CRLF through the `/mnt/c` checkout.
- T10 reports, after the modelled total, the `gross_mass_limit` it compared against and the F2S vehicle estimate (93 kg), each
  with its source, and prints "per-link masses still placeholders" if and only if a reported group draws a mass from a datum
  whose status is `placeholder`. The clause is computed from `datums.yaml` on every run, so it disappears when real masses
  land. T10 is still report-only: it fails only when the modelled total exceeds `gross_mass_limit`.
- `f2s_vehicle_mass_estimate` datum (`report_only`, source `f2s`, status `provisional`): Alex's F2S design proposal of 28 Sep,
  about 93 kg gross against the 120 kg cap. Second-hand: the F2S document itself is not in this repository.
- Check C02: `config/upstream_pins.yaml` must equal the HEAD of a clean `src/upstream/openarm_description`. A difference is a
  FAIL naming both hashes. With no checkout it is a SKIP that says so, the only one this tree is allowed to have.
- `docs/verification.md`: how to run `verify_a6.sh`, the evidence report field by field, what `static+mock` permits, and the
  six WARNs with what would clear each.
- READMEs for the description package and, stating their scaffold status, for the control, bringup, Gazebo and MoveIt
  packages.
- Test dependencies `python3-numpy` and `liburdfdom-tools` declared in the description package, so
  `rosdep install --from-paths src` installs what T02 and T05 use.

- `openamrobot_upperbody_gazebo` (A7, part 1): `sim.launch.py` (args `mast_id`, `world`, `headless`, plus `physics_engine` and
  `rviz`) spawns the combined robot in Gazebo Harmonic with the base on platform-sw's DiffDrive plugin and both arms on
  `gz_ros2_control`; an empty world; platform-sw's `gz_bridge.yaml` used by reference, plus one bridge for the wheels' joint
  states; `joint_state_publisher` merging `/base/joint_states` and `/arm/joint_states` so `/joint_states` has one publisher;
  `odom -> base_link` from the DiffDrive plugin only. The `gazebo` profile loads `gz_ros2_control` into the model (model plugin,
  `gz_ros2_control-system`, with `arm_controllers.yaml` by installed path), names read from the installed library.
- Checks G01 (headless smoke test: five controllers `active`, an arm goal completes, platform-sw's topics unchanged by name and
  type, `/cmd_vel` and `/odom` work, one `odom -> base_link` publisher, `/joint_states` once each) and G02 (grippers close and
  `finger_joint2` follows in the physics). `verify_gazebo.sh` writes a `gazebo` evidence report, `evidence_class: "simulation"`.
- `docs/v01_visual_check.md`: the checklist for V01, to be answered by a person with the simulator and RViz open.
- Check T15 gains the finger-mimic multiplier test (a hard-coded multiplier, including one equal to the yaml's, fails), and the
  plugin tests. T10 gained a tripwire (a verified mass datum with no base/arm provenance entry fails), replaced in P10 by the provenance map below.

- `openamrobot_upperbody_moveit` (A7, part 2): `config/openamrobot.srdf` (the groups, `home` / `hands_up`, end effectors, passive joints and arm-internal
  pairs imported from `enactic/openarm_ros2` @ `4e837e1` with attribution; a fixed `world -> base_link` virtual joint; `park`, a placeholder
  copy of `home`; per-hand gripper `open` / `closed`; the self-collision matrix generated with the Setup Assistant's `collisions_updater` at
  `mast_1350`, 10000 samples, never/adjacent/default on, always off, reproducible with `scripts/regenerate_acm.sh`); `kinematics.yaml` (KDL),
  `pilz_cartesian_limits.yaml` (upstream's, inert), `ompl_planning.yaml` (MoveIt's defaults), `joint_limits.yaml` (our acceleration entries,
  tagged `engineering_default` / `placeholder`, naming manipulation#10 and the upstream file, ADR-009), `moveit_controllers.yaml` for our
  controller names (option 1 of the gripper fork, `GripperCommand`; the controller type is unchanged); `launch/move_group.launch.py`, built with
  `MoveItConfigsBuilder` over the combined xacro and its args.
- Checks M03 (the collision matrix validated by sampling, `scripts/validate_acm.py`), M04 (a goal inside a base-side volume is rejected by
  planning, not merely unsuccessful), and the proposed M05 (per-hand gripper values, the single-number version fails) and M06 (the plans need
  some acceleration limit and do not depend on its value).
- `openamrobot_upperbody_bringup`: `moveit.launch.py` (`mast_id`, `sim` mock|gazebo, `rviz`; `headless`, `physics_engine` for Gazebo), a stub for the
  wheel joints on mock, checks M01 (every named pose on mock at all four mast IDs) and M02 (the same in Gazebo at `mast_1350`, Bullet-Featherstone
  passed explicitly), `run_a7_regression.sh` and `build_a7_evidence.py` (`evidence/<ts>_a7.json`, `evidence_class: "simulation"`, a dirty tree
  writes nothing). M01 and M02 skip unless `OPENAMR_A7=1`, so `verify_a6.sh` does not start them.
- `docs/v01_visual_check.md` has an eighth item (MoveIt: a named pose, preview against execution); V01 stays `WARN`.

### Changed

- The evidence harness writes no report from a dirty tree: it prints the results, says `Evidence: NOT WRITTEN`, and exits 3 if
  nothing failed. Every file in `evidence/` is a clean-commit run.
- C02 checks every pin in `config/upstream_pins.yaml`, not only `openarm_description`, and the file is normalised to quoted
  40-hex hashes (`openarm_ros2` was a 7-character hash nothing compared). A pin with no checkout is listed as not compared; C01
  compares every pin with the manipulation baseline.
- In `ros2_control.xacro` the finger-mimic multiplier property moved from inside the mock/real branch to file scope so it can be
  read and tested; the expansion of every profile (mock, real; gazebo before the plugin was added) is byte-identical.
- Repository README: status updated, and a pointer to the description package. The mast configuration text is unchanged.
- `mast_section` datum: the invented 80 mm square placeholder is now the MISUMI HFS6-60120 profile, 60 x 120 mm (`source: plan`,
  `status: provisional`; which axis carries the 120 mm is unconfirmed). Datums revision `2026-10-08.d`, so every
  configuration id changed.
- `package.xml` descriptions of the bringup, control and MoveIt packages describe the 2.0 fixed-mast scope instead of a lift.
- The control README's note on the gripper controller's deprecation warning now states, in order: what is in use, that it is
  a warning and not a removal, that the replacement serves a different action type, that the warning misspells the
  replacement's name, and that the choice is manipulation#11, ADR-009 point 4 and P10's. The controller type is unchanged.
- Datums revision `2026-10-08.e` (the new F2S datum), so every configuration id changed. C01 no longer compares the pin file
  with a checkout (that is C02); it keeps the format and the baseline-pin comparison.

- `sim.launch.py`'s `physics_engine` defaults to DART again (`gz-physics-dartsim-plugin`), not Bullet-Featherstone: a
  gripper-mimic convenience is not grounds to re-baseline platform-sw's physics. G02 passes `gz-physics-bullet-featherstone-plugin`
  explicitly and its sidecar records which engine each Gazebo check ran on.

- The description no longer resolves `openamrobot_upperbody_control`'s path. The `gazebo` profile's `gz_ros2_control` `<parameters>`
  is the new `controllers_file` xacro arg (default empty, no `<parameters>` element then); `sim.launch.py`, which depends on both
  packages, passes `arm_controllers.yaml`'s installed path. This removes the `$(find)` that the description could neither declare
  (a colcon cycle) nor leave undeclared. The `gazebo` configuration id changes; the other 13 expansions of the matrix are
  byte-identical to before. T15 asserts the element appears with the arg and that the default expansion is valid URDF.

- `config/mass_provenance.yaml`, the mass-provenance map: `placeholder`, `external_nominal` or `verified` for each of the four mass
  groups (base and openarm are `external_nominal`, mast and sensors are `placeholder`, nothing is `verified`). T10 reads the
  caveat off it: `per-link masses still placeholders` when any group is `placeholder`, and a separate line naming the
  `external_nominal` groups. A group that draws its mass from a datum (the mast, `mast_mass`) must agree with the datum's status;
  `verified` needs a `document`. This replaces P09's tripwire, which only guarded the absence of the map. Because the sensors
  carry no inertial (zero mass is a figure chosen so the model expands), that group is a placeholder, so the caveat now needs the
  cameras weighed as well as the mast.

- Gazebo README: the DART finger finding (P10 task 0, a ten-minute check). The position command moves a finger on DART once
  `position_proportional_gain` is raised (0.1 and 1.0: no motion; 5.0: `finger_joint1` reaches the goal and oscillates), so the
  missing finger motion is a gain and not an engine fault; why the threshold is where it is was not isolated, the gain is
  plugin-wide and was not tried on the arms, and `finger_joint2` still does not follow on DART, so G02 stays on
  Bullet-Featherstone. A tuning note for platform-sw's owner; nothing is filed upstream.

### Removed

- `config/lift/`: an empty scaffold directory. An actuated lift is OpenAMRobot 3.0 scope.

### Known limitations

- **M03 FAILS** (A7): the collision matrix disables `openarm_left_ee_link1` / `openarm_left_link2` as `Never` and they collide at sample 1112 of 2000 at every mast ID.
  The pair is not re-enabled. A 20000-sample diagnostic found 14 of the 458 `Never` pairs colliding, all arm-against-arm. The matrix is not certified.
- MoveIt never sees `base_link`'s inertial (W01); `park` is a placeholder copy of `home`; the acceleration limits are engineering defaults, not vendor
  data; the gripper fork is open on manipulation#11 (option 1 is used; option 2 was driven on mock, not in Gazebo); MoveIt 2.12.4 refuses a start state
  1e-7 rad past a joint limit, so planning from `home` in Gazebo is intermittently refused (M01/M02 retry and count); `move_group` exits with SIGSEGV when
  stopped. Everything A7 shows is `simulation` class.

- Base inertials are platform-sw CAD placeholders; the mast mass, mast and plate transforms and all camera poses are
  placeholder datums. No physical evidence.
- `ros2_control` runs on `mock` (L01) and in Gazebo (G01, G02); the `real` block is checked by expansion only,
  `openarm_hardware` cannot be built in the reference workspace (it needs OpenArmCAN). In Gazebo the arms' position interface is
  a velocity servo, so tracking there says nothing about the real arm. `finger_joint2` is not declared under `ros2_control`:
  Gazebo's Bullet-Featherstone engine follows the URDF `<mimic>` on its own, its DART default does not move the fingers at all.
- The Gazebo launch's `physics_engine` defaults to DART, the engine platform-sw tuned the base on, and the grippers do not close
  there. G02 passes Bullet-Featherstone explicitly. The base's contact behaviour was compared on neither engine (G01 measures
  kinematics).
- The gripper controllers stay `position_controllers/GripperActionController` (deprecated in Jazzy) on purpose: the replacement,
  `parallel_gripper_action_controller`, serves `ParallelGripperCommand` instead of `GripperCommand`, so switching decides the
  gripper interface (manipulation#11, ADR-009 point 4), which is open.
- Issue #7: the description strings it names are fixed here; whether that closes it is for the code owners to judge.
