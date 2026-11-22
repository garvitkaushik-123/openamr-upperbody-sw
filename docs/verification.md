# Verifying the robot description (A6) and its simulation (A7)

`verify_a6.sh` runs the description, control and bringup tests, then writes one JSON evidence report. This page says how
to run it, what the report contains, what it lets you claim, and why four checks are `WARN` (T08, E01, W02, V01). The Gazebo report and the A7 report
(MoveIt, `run_a7_regression.sh`) follow the A6 one.

## Run it

```bash
source /opt/ros/jazzy/setup.bash
cd ~/openamr_ws
colcon build --symlink-install --packages-skip openarm_hardware openarm   # the tests describe the INSTALLED package (I01)
source install/setup.bash
ros2 run openamrobot_upperbody_description verify_a6.sh                   # about 2 minutes
```

It prints a summary and writes `<workspace>/evidence/<UTC timestamp>_dev.json`, **from a clean tree only**: with uncommitted
changes (untracked files included) it prints the results, says `Evidence: NOT WRITTEN` and names the files, because a report
records a commit and a dirty tree is none. The exit status is 1 if a check is `FAIL` or `NOT_RUN`, 3 if everything passed but the tree was
dirty, 2 if the evidence cannot be assembled. `OPENAMR_EVIDENCE_DIR` moves the output; `OPENAMR_WS` names the workspace.

## The evidence report (`openamr.evidence.v1`)

| Field | Meaning |
|---|---|
| `schema`, `repo`, `profile` | `openamr.evidence.v1`, `openamr-upperbody-sw`, `dev` |
| `git_hash`, `dirty` | The commit the report describes. `dirty` is always `false` in a written file: the harness refuses to write evidence from a dirty tree (it prints the results, says `Evidence: NOT WRITTEN` and names the files, and exits 3 if nothing failed), because a report from uncommitted changes describes no commit and a file that exists gets read |
| `timestamp` | UTC, when the report was built |
| `evidence_class` | Always `static+mock`; see below |
| `upstream` | Commits of `openarm_description`, `openarm_ros2`, `openamrobot-manipulation` and `openamr-platform-sw` (the base is included unchanged, so its commit is an input of every expansion). Without a git checkout of an upstream pin its commit is the pin in `config/upstream_pins.yaml`, and `limitations` says so |
| `datums_revision` | `revision` of `datums.yaml`; one input of every config id |
| `checks[]` | `id`, `name`, `result`, and `details` (pytest counts plus whatever the check recorded) |
| `config_ids` | `sha256:` plus 64 hex characters, one per expansion (`column/base_plate/head_pitch/base_tilt/arm_package/hardware_profile`). Two reports with the same id describe the same URDF, datums revision and upstream pins |
| `config_id_inputs`, `calibration_ids` | What the id is made of now and before P15 (the format is unchanged and its meaning is not: an old id is not comparable), and the lift's calibration stops as joint positions, beside the ids and never inside them |
| `provenance` | Datum counts by status: `placeholder`, `provisional`, `verified` |
| `limitations` | What the report does not establish, in words. Read it before quoting the report |

## What `static+mock` permits

It permits: the xacro expands for every configuration of the matrix (option B's IDs, arm package, hardware profile); `check_urdf` accepts the result; frames,
inertials and the planar joint have the properties the checks name; and the five controllers reach `active` on
`mock_components/GenericSystem` and follow one trajectory (L01).

It does not permit any claim that a controller has driven a motor, anything about Gazebo, MoveIt or hardware, tracking
performance (mock hardware reflects commands back as state), or the real mass, envelope or lift column geometry (placeholder
datums and a catalogue mass). Words such as "validated" or "works on hardware" do not belong in anything quoting it. The shared rules say it
directly: *"SKIP/BLOCKED is not PASS; fixtures/fake hardware are not integrated simulation, physical acceptance or
release readiness."*

## Simulator dynamics (14 Nov 2026)

Four supporting tests, counted under `AUX` in the A6 and A7 reports (they have no check ID of their own):

| Test | Package | Asserts |
|---|---|---|
| `test_sim_dynamics_profile.py` | description (A6) | `config/sim_dynamics/openarm_v2.yaml` matches its schema; every value names a listed source and a place in it; every estimate has a method; every source is a full commit hash; the pins equal `upstream_pins.yaml`; the joints are exactly the model's movable OpenArm joints; the limit copies equal the URDF limits in every profile; upstream's `friction.yaml` is still empty at the pin |
| `test_sim_dynamics_urdf.py` | description (A6) | `hardware_profile:=gazebo` has `<dynamics>` with the profile's values on all 16 controlled arm and gripper joints (and the two mimic followers); `real` has none and its text does not depend on `sim_dynamics`; `sim_dynamics:=on` with `real` is refused; with the 18 lines removed an expansion equals the one without them |
| `test_sim_overlay_yaml.py` | control (A6) | the simulation overlay equals the profile (750 Hz, gripper `max_effort`), adds no controller, and `arm_controllers.yaml` is unchanged |
| `test_sim_control_rate.py` | gazebo (A7) | the launch passes the overlay after the main file, and the world step makes the controllers update at 750 Hz (a 1 ms step gives 500 Hz) |

What they permit: the statement that the simulated model carries these values and the real profile does not, and that each value has a
recorded source. What they do not permit: any statement that the values match a physical arm. They are upstream's simulator values, not
identified ones, and the armature is not applied in Gazebo.

## The Gazebo report (`verify_gazebo.sh`)

```bash
ros2 run openamrobot_upperbody_description verify_gazebo.sh            # about 1.5 minutes: two headless Gazebo servers
```

It runs the launch tests of `openamrobot_upperbody_gazebo` and writes `<workspace>/evidence/<UTC timestamp>_gazebo.json` with
`profile: "gazebo"` and `evidence_class: "simulation"`, plus a `simulator` block (Gazebo, `gz_ros2_control` and `ros_gz_sim`
versions). It refuses to write from a dirty tree, like the other report. Its checks:

| ID | Proves |
|---|---|
| G01 | in a headless Gazebo on the launch's default engine (DART): the five arm controllers are `active`; a `left_arm_controller` goal completes and tracks while the other arm and the base stay still; every topic of platform-sw's bridge file exists by name and type; `/cmd_vel` moves the base and `/odom` reports it; exactly one publisher of `odom -> base_footprint` (platform-sw's DiffDrive child frame since 935efc7, read from the description; strictly increasing stamps at the plugin's 50 Hz); `/joint_states` has one publisher carrying the wheels, the arms and `lift_joint` once each |
| G02 | both grippers reach a goal off zero (they open part way) and `finger_joint2` follows `finger_joint1` in the physics (link poses, not `/joint_states`); it passes `physics_engine:=gz-physics-bullet-featherstone-plugin` explicitly, because the launch defaults to DART (platform-sw's engine) and the grippers do not move there: on DART it fails, which is the documented limitation, not a defect |
| I01 | the installed gazebo and description packages match the source |
| V01 | `WARN`. The eight items of the lift model were measured by script in P16 (none reads outside its expected value; two sub-questions that need eyes are CANNOT-MEASURE); it stays `WARN` until `lift_mass_tl3_400` is sourced, whatever the eight say. "Looks fine" does not clear it |

**What `simulation` permits:** the model behaves as G01 and G02 say in that simulator on that machine. **What it does not:** any
statement about a physical robot, about the real arm's controller, gains or latency (the arms' position interface is a velocity
servo in `gz_ros2_control`), about payload, torque or mass (the masses are placeholders), about the base's real wheel or
motor parameters (platform-sw's plugin values, not measured), or about how the base behaves on the physics engine platform-sw
tuned for (DART). The report's last limitation is the A6 report's own sentence: *no physical evidence*.

## The A7 report (`run_a7_regression.sh`)

```bash
ros2 run openamrobot_upperbody_bringup run_a7_regression.sh            # about 15 minutes; headless
```

It runs the gazebo, moveit and bringup tests (`OPENAMR_A7=1`, which un-gates M01, M02 and M07: they skip otherwise so that `verify_a6.sh` never
starts them), then `build_a7_evidence.py` writes `<workspace>/evidence/<UTC timestamp>_a7.json` with `profile: "a7"`, `evidence_class:
"simulation"`, a `simulator` block (Gazebo, `gz_ros2_control`, `ros_gz_sim` and MoveIt versions) and a `moveit_config` block (hashes of the
SRDF, which carries the collision matrix, and of the other configuration files). It inherits P08's rule: **a dirty tree writes no file**. The
exit status is 1 if a check is `FAIL` or `NOT_RUN`, 3 if everything passed but the tree is dirty, 2 if the evidence cannot be assembled.

| ID | Proves |
|---|---|
| G01, G02 | as in the Gazebo report above (G01 on the default DART engine, G02 on Bullet-Featherstone passed explicitly) |
| M01 | on `mock`, at **each of the three lift calibration stops** (`L1000`, `L1175`, `L1350`: `lift_joint` 0, 0.175, 0.350 m, joint positions and not configuration IDs), every SRDF named pose plans and executes through `move_group` into the real controllers and ends under 0.01 rad of its goal: both arms (`hands_up`, `home`, `hands_up`, `park`) and both grippers (`open` then `closed`, each hand with its own value) **S8 with the lift planned (issue #17):** each run first puts both arms in `park` (group `both_arms`), PLANS the lift from `L1000` to its stop (to `L1175` and back for `L1000`) through group `lift` and `lift_controller`, and plans it back to `L1000` after the poses; a lift goal is sent only between the stops and only with both arms reading `park` (the register LIFT rule as scenario design, not an interlock), and every lift and park move must succeed (lift final error under 2 mm) |
| M02 | the same in Gazebo at the baseline configuration with the lift at `L1000`, planned there from `L1175` first (issue #17: the lift has a `gz_ros2_control` interface and `lift_controller`, its state is the physics'), on Bullet-Featherstone passed explicitly because the gripper poses are not reached on DART. **It first waits for the six controllers to be `active`** (120 s, a `FAIL` naming their states if they never are, and no pose is sent), because a pose sent early is a test of the harness. **The clamp part of its verdict is exactly two numbers**: readings beyond epsilon (must be 0) and the largest overshoot magnitude (must be below epsilon, 1e-5 rad; the epsilon is not widened by any outcome). Activation and episode counts are in the report under `diagnostic_only`, labelled as such, and gate nothing: they scatter 2-3x on an unchanged tree (646-1054 right-finger activations in one configuration, P16) |

| M03 | the collision matrix disables only what can be justified. A pair is disabled for exactly three reasons: declared by upstream (the 20 pairs of section 1, all `Adjacent`), `Adjacent` (a structural neighbour) or `ConstantTransform` (no movable joint between the links, so one evaluation settles every configuration, and it found them clear: 45 pairs since P15, 77 before the lift put a movable joint between the base and the arms). Everything else, 478 pairs, is enabled. A disabled pair with no reason, a false label, a `ConstantTransform` pair that collides, or a constant pair left enabled that collides is a `FAIL`; **there is no `WARN` class**. The sampled part is information about the enabled pairs (below) |
| M04 | a goal inside the nearest **reachable** structure volume (`arm_profile_link`, option B's 340 mm box; the arms cannot reach the base body at all) is **rejected** by planning (the joint goal: answered at once; the reply is the generic `FAILURE`, and the planner's verdict `GOAL_STATE_INVALID` exists only in `move_group`'s log, which the check reads), not merely unsuccessful; the pose goal at the same place **times out**, and the record says which is which |
| M05 | the per-hand gripper values: `open` is each hand's own far limit, the signs are opposite, `closed` is 0, and the single-number version fails in both directions (ADR-009 point 5). Since 14 Nov 2026 it also measures the fingertip gap on the expanded URDF (under 1 mm at `closed`, over 100 mm at `open`) and fails the swapped names, which the SRDF had until then |
| M06 | the named-pose plans need some acceleration limit and are insensitive to our values across three orders of magnitude, which retires the stop condition "planning succeeds only because the limits we invented are generous"; **it does not make the limits any less of a placeholder** (ADR-009) |
| M07 | **at rest in Gazebo, no joint sits within `joint_rest_margin_rad` of either of its limits** (datum, `placeholder`, 1e-3 rad: 3.6 times the largest between-run scatter of a rest position measured, 2.8e-4 rad). Nothing is commanded; the raw `/arm/joint_states` are read for 6 s of simulated time after the model has settled. **Rule as Alex wrote it (restored in P19; P17's first version was a FAIL for every joint, P18 withdrew it, 19 Oct):** `WARN`, with the reason "rest pose at an upstream limit" and never `PASS`, for the four joints whose rest pose is an upstream limit by construction (`openarm_{left,right}_finger_joint1`, `openarm_{left,right}_joint4`); `FAIL` for any other joint within the margin, for one of those four read further past its limit than the margin, and for incomplete joint-state coverage (all 16 expected joints, 50 samples each). The sidecar lists every joint with its distance, so nothing is hidden. `lift_joint` and the mimic `finger_joint2`s are listed under `not_measured` with the reason. **Non-finite raw positions (NaN, +inf, -inf) are rejected before the coverage count and before any extremum, never count toward the 50 samples, are kept in the sidecar (`invalid_readings`), and make the check `FAIL`, never `PASS` or `WARN` (P27).** `simulation` class: how this model rests in this simulator. One agreed rest-state rule, including what happens to missing joint-state evidence, is still pending with Alex and Om |
| W04 | **WARN**: a `gz sim` server had to be force-stopped by the launch's stop helper (it accepted the stop and did not go; SIGINT, the service request and SIGTERM did not end it). Read from the stop-helper report in the sidecars of G01, G02, M02 and M07; `NOT_RUN` when one is missing. A server still running after the force-stop is not W04's: it is a survivor, and the launch test that owns it FAILs |
| H01 | every `test_*.py` is registered in its package's CMakeLists, and every launch test checks that its processes started and exited cleanly (`launch_guard.py`); static, over all five packages |
| H02 | every test CMake registered left a fresh, complete result file; a timeout, a signal, a collection error or an empty package is listed by name and makes the checks it backs `NOT_RUN` |
| L01, I01, V01, AUX | as in the A6 report, for the packages this one touches; V01 has **eight** checklist items, each stating what to measure (item 8: an arm executes a named pose in RViz and the preview matches what the robot then does) and stays `WARN` until a person has answered them |

**Recorded run, P15 (the lift, option B adopted)**: commit `dada509`, the code of the step (later commits are documentation and evidence copies only), from a clean rebuild of every package on platform-sw `174731c`: **13 `PASS`, 1 `WARN` (V01), 0 `FAIL`, 0 `NOT_RUN`**, as in the P14 run before it, with these numbers moved and each for a reason: M01 12 of 12 poses at each of three lift stops (was four mast IDs), worst final error 0.001 rad; M02 12 of 12; M03 83 pairs disabled (38 `Adjacent`, 45 `ConstantTransform`, 20 declared by upstream), **478 enabled (was 113 / 77 / 448)**, because a prismatic joint now lies between the base and the arms; M04 rejected the joint goal 0.071 s after the control and timed out on the pose goal, now against `arm_profile_link` (the mast is gone). The clamp (epsilon 1e-5): 0 activations in M01; in M02 more than before (see `evidence/reports/README.md`), all under the epsilon, 0 readings beyond it. The A6 report: `verify_a6.sh` gives **24 `PASS`, 4 `WARN` (T08, E01, W02, V01), 0 `FAIL`, 0 `NOT_RUN`** (was 22 / 6 / 0 / 0: T04 resolved by deleting the six near-field frames from the contract, W01 because the root is `base_footprint` and carries no inertial). The reproducibility of that run and of the P14 flake is below. P12A's record follows as history.

Recorded run (P12A, the fixed mast): commit `bcd8059` (code; later commits are documentation only), from a clean build of the five upper-body packages, evidence `20261007T164239Z_a7.json`: **13 `PASS`, 1 `WARN` (V01), 0 `FAIL`, 0 `NOT_RUN`**, with no `warnings` list: M03 has no WARN class. M01 passed 12 of 12 poses at each of the four mast IDs (worst final error 0.001 rad), M02 12 of 12 at `mast_1350` (0.00111 rad), M04 rejected the joint goal and timed out on the pose goal. M03: 113 pairs disabled (36 `Adjacent`, 77 `ConstantTransform`, 20 of them declared by upstream), 448 enabled; every disabled pair has a reason that can be shown, the 77 constant pairs are clear in their one evaluation at every mast ID. The clamp (epsilon 1e-5 rad): 0 activations in M01; in M02 242 activations in 22 episodes, the largest 8.3e-07 rad, 0 readings beyond the epsilon. The A6 report: `verify_a6.sh` at the same commit gives 22 `PASS`, 6 `WARN`, 0 `FAIL`, 0 `NOT_RUN` and the same 14 config ids (`20261007T164436Z_dev.json`). <!-- decision-allow: LIFT historical: the P12A recorded run on the superseded fixed mast -->

**The harness fix did not change a number that mattered.** The suites re-run on the tree before the matrix changed, with the fixed harness: A6 `20261007T155047Z_dev.json` 22 `PASS` / 6 `WARN` / 0 `FAIL` (the 20 / 6 / 0 of P11 plus the new H01 and H02, every old check with the same result and the same 14 config ids), and A7 `20261007T160453Z_a7.json` 13 `PASS` / 1 `WARN` / 0 `FAIL` (the 11 / 1 / 0 of P11 plus H01 and H02). The runs between those and the recorded one are in the evidence directory and are failures worth reading: `e682825` failed G02 through the new process guard (`joint_state_publisher` exited 1 at shutdown, and the `gz` server of that run was left running after the launch ended), and `832b990` failed AUX on a bug in the guard itself, which the report named.

**What the sampled part of M03 is now.** Information, not a result and not a reason: 2000 random joint states per lift stop at seed 20261006, and the ENABLED pairs found colliding with how often (P15: 58 of 478 enabled pairs collided in some of them, the same at every lift stop, the commonest an arm link against `arm_profile_link`: `openarm_right_ee_base_link` 49 of 2000; P12A: 37 of 448, the same at every mast ID, the commonest against `mast_base_link`: 48 of 2000). Sampling falsifies and never certifies, so nothing in the matrix rests on it; a pair that collides often although the matrix is right tells A4 something about reach and clearance. The by-construction part does not depend on N or on the seed: no movable joint between two links means a constant transform, one evaluation settles every configuration, tested against an independent forward kinematics. The matrix is a checked-in file (`config/openamrobot.srdf`, fingerprint in `config/acm/acm_generation.yaml`) and it is **derived, not sampled**: regenerating it gives the same matrix every time (`test_acm_generation.py` does it twice), where the Setup Assistant's sampler, which has no seed, gave a different one on every run.

**Planning time before and after** the matrix went from 458 sampled `Never` pairs (plus 36 `Adjacent`) to 113 justified pairs, offline, 20 repeats x 2 rounds of the twelve named-pose plans (OMPL RRTConnect): the whole sequence took a median of 0.42 to 0.43 s before and 0.455 s after, every plan succeeded both times; per pose in the MoveIt package README. In the real stack (M01 and M02, through `move_group` and the controllers) every named pose still plans and executes. MoveIt does not expose a collision-check count to `moveit_py`.

**What `simulation` permits here:** every claim reads *"plans and executes in simulation under declared OpenAMRobot limits"*. The limits
are engineering defaults (`source: engineering_default`, `status: placeholder`) because upstream declares no acceleration limits
(manipulation#10, ADR-009); they are not vendor data. **What it does not permit:** any statement about a physical robot, a payload, a
torque, a validated limit, the real arm's controller or latency (the Gazebo position interface is a velocity servo), or the base's
behaviour on the engine platform-sw tuned (DART, on which the grippers do not move).

The A7 WARNs and findings, beyond the A6 ones (W01, W02, E01, T04, T08, V01 above):

| Item | What | What would clear it |
|---|---|---|
| W01 in A7 | resolved by P15: the root is `base_footprint` (platform-sw 935efc7) and carries no inertial, so MoveIt sees `base_link`'s 4.18 kg | nothing: it stays a guard (a root with an inertial fails W01 again) |
| the lift column | placeholder geometry since `b45d8f6` (19 Oct): a 240 x 240 mm box, visual and collision, on each stage, and a placeholder box inertia; the arms can no longer plan through the column body, but the box is not the supplier's shape and its lateral side is not sourced | the supplier drawing |
| `park` | a placeholder: `home` with joint 4 at 0.1 rad, off its lower limit (issue #17) | Adnan's safe-pose definition under the manipulation action contract |
| named poses on a joint limit | upstream's `home` (joint 4 = 0) and the gripper `open` / `closed` states are still ON a limit; issue #17 asks 0.05 rad from every limit, and M05 and the ADR-007 import comparison pin these values (`test_named_pose_margins.py` lists them) | a decision on M05 / ADR-009 point 5 and on ADR-007's imported `home` |
| joint-state clamp default | `clamp_joint_states:=auto` (issue #18): on for `mock` and `gazebo`, off for any other profile; no encoder-resolution bound is modelled for hardware | a hardware bring-up that needs one |
| acceleration limits | engineering defaults, tagged placeholder | vendor data (manipulation#10) |
| gripper interface | option 1 is in use; the choice is open on manipulation#11 | Om and Adnan; the evidence is in the MoveIt package README |
| start-state tolerance | MoveIt 2.12.4 refuses a start state any amount past a joint limit, and in Gazebo `home` (joint 4 on its lower limit 0) reads back up to 9.9e-7 rad past it. `move_group` plans from a joint state clamped to the limit by at most 1e-5 rad (`joint_state_clamp.py`); the clamp's count and largest magnitude are in the report (`clamp`); a reading beyond the epsilon is never clamped. Upstream's `home` on its own limit is itself reportable upstream | nothing here: the clamp is a simulation workaround, and the choice of where a tolerance may live on a real robot is not made |
| `move_group` shutdown | exits with SIGSEGV when stopped; allowed for that one process | upstream MoveIt |
| enabled pairs | **28 Oct (P28): 99 of 666 disabled (20 upstream, 18 `Adjacent`, 61 `ConstantTransform`), 567 enabled**, since the adapter plate is a geometry link; M03 PASS in three runs, and in them 92, 90 and 82 enabled pairs collide in some of 2000 random states at `L1000`, `L1175` and `L1350` (86, 82, 76 in P27: the column is 60 mm across now). 19 Oct: the matrix disables 95 of 630 pairs of geometry-bearing links (20 upstream, 17 `Adjacent`, 58 `ConstantTransform`) and enables the rest (535), since the column boxes of `b45d8f6` made the two column stages geometry links. The P15 figures this row used to carry (83 of 561 disabled, 478 enabled; 58 enabled pairs colliding in some random states, all an arm link against `arm_profile_link` or the other arm) were measured on the earlier geometry. The sampled part is re-taken in every A7 report: in `20261019T010818Z_a7.json` (M03), 86, 82 and 76 enabled pairs collide in some of 2000 random states at `L1000`, `L1175` and `L1350`, and the commonest are now a column stage against an arm link | nothing to clear: information for A4 (reach and clearance) |

## Reproducibility is part of a result (P15)

P14 ran A7 twice on an unchanged commit and got 2 `FAIL`, then 0. The cause was our own `joint_state_clamp.py` exiting 1 at shutdown, in a
launch of M01; it was neither allow-listed nor ignored, and P15 fixed it. Reproduced standalone against a flooded `/joint_states` before the fix,
every SIGTERM exited 1 (30 of 30, an uncaught `ExternalShutdownException`) and 1 of 30 SIGINTs exited 1 with rclpy's `take_message` `RuntimeError`;
after it, 60 of 60 and 60 of 60 exit 0, and `test_joint_state_clamp_shutdown.py` sends real signals to a real node. `launch_guard.py` is unchanged:
its two allowances (`parameter_bridge` -6, `joint_state_publisher` 1) stand because those are stock processes we cannot change.

**Five consecutive A7 runs on the unchanged commit `7daa2ab`** (the fix, before the rebase), every launch of every run exiting as the guard
expects:

| Check | run 1 | run 2 | run 3 | run 4 | run 5 | tests per run |
|---|---|---|---|---|---|---|
| G01, G02 | PASS | PASS | PASS | PASS | PASS | 7, 2 |
| M01 (four launches) | PASS | PASS | PASS | PASS | PASS | 8 (12 of 12 poses in each of the 20 launches, worst error 0.00100 rad) |
| M02 | PASS | PASS | PASS | PASS | PASS | 2 (12 of 12) |
| M03, M04, M05, M06 | PASS | PASS | PASS | PASS | PASS | 6, 2, 6, 1 |
| L01, H01, H02, I01, AUX | PASS | PASS | PASS | PASS | PASS | 3, 13, -, -, 89 |
| V01 | WARN | WARN | WARN | WARN | WARN | - |
| **Totals** | 13 / 1 / 0 / 0 | 13 / 1 / 0 / 0 | 13 / 1 / 0 / 0 | 13 / 1 / 0 / 0 | 13 / 1 / 0 / 0 | PASS / WARN / FAIL / NOT_RUN |

The same A7 regression on the final commit `dada509` (option B adopted, platform-sw `174731c`) was run three times: see
`evidence/reports/README.md` for the files. Three runs say it is not a coin flip on this commit; they do not say it cannot flake, and the rule
stands: **where a check has ever been observed flaky, the evidence records how many runs were made and how many passed**.

## The harness cannot lie (P12A)

A report is only worth the failures it can show. P12A planted failures on purpose (`scripts/harness_canary.py`: an assertion, a collection error, a timeout, a
self-inflicted SIGSEGV, a test named for an unlisted check ID, and for launch tests a process that dies of SIGSEGV or cannot start, with and without a process
check) in each of the five packages that contribute checks, ran the real entry points, and read the reports.

**Before the fix**, the holes (each demonstrated by running something, none argued from the code alone):

| Hole | What happened |
|---|---|
| a package that is run but not read | `verify_a6.sh` ran the control package's tests and the report never opened its results: ten canaries failing, **exit 3 and 0 FAIL** (3 is "everything passed, tree dirty") |
| a result under a check ID no report lists | `test_q99_*` fails in the xunit file and the report drops it: nothing says so. In A6 the same happened for real: the flaky failures of the A7-gated M01/M02 (two of three runs failed, "Processes under test stopped before tests completed") were dropped because A6 does not list M01/M02 |
| the exit status | `colcon test` exits **0 whatever its tests did**; only `colcon test-result` exits 1. The scripts passed the former to the report, so a step "that exited 0" meant nothing |
| a check with a sidecar and no test behind it | `combine("T04", None, <a PASS sidecar>)` returned `PASS`, so a test that wrote its sidecar and then died would leave its check green (shown on the function; no real suite produced it) |
| a launch whose processes die | a process of a launch test that dies of SIGSEGV while the test passes, or that cannot be started, is a passing test: `launch_testing` never looks. The log of a real G01 run showed `parameter_bridge` aborting at shutdown (`corrupted double-linked list`), reported by nobody |

What was **not** swallowed: assertion failures, collection errors, timeouts and pytest processes that die of a signal were all caught (ament writes a
`missing_result` stand-in), as AUX failures, in the four packages that were read. The P10 M03 mutation test failed in P10's own xunit and the report did say
`M03: FAIL` with that test named in `failed_tests`; what was missing was a reader. The scope is therefore specific, not general, and the numbers survive (below).

**The mechanism now:**

* `NOT_RUN` is a result of its own (above). A `PASS` needs a test that ran behind it; a sidecar alone is `NOT_RUN`.
* **H02**: every test CMake registered (`ctest -N`) must have left a fresh, complete result file. A timeout, a signal, a collection error, a package with
  nothing registered and a stale file are listed by name, and the checks they back are `NOT_RUN`.
* results under an ID the report does not list are folded into AUX and fail it; every package a script runs is read, and cleared before the run; the scripts
  take `colcon test-result`'s exit status.
* **H01**: every `test_*.py` is registered in its package's CMakeLists, and every launch test module ends with a post-shutdown test that names the processes it
  cannot do without and calls `launch_guard.assert_launch_healthy`, and its last active test calls `assert_no_process_died`: in the active phase `proc_info` holds the
  processes that have exited so far, so a non-zero exit there is a crash in the middle of a test, caught before the shutdown allowances below can hide it. A process that
  never started is not in `proc_info` at all (an ExecuteProcess with a missing executable: the launch carries on and the test passes), so only the list of expected processes notices it.
  The non-zero exits it allows are listed with their reasons in `launch_guard.py`: `move_group` -11 (known, at every stop), the `gz sim` ruby wrapper -2 or -15
  (the launch's own shutdown signals), and `parameter_bridge` -6, a heap-corruption abort at shutdown that is **a finding to take to platform-sw**, allowed only
  so that it does not make G01/G02/M02 flaky.

| Package | assert | collect | timeout | signal (pytest) | launch: process dies | launch: process never starts | unlisted ID |
|---|---|---|---|---|---|---|---|
| description | AUX → AUX | AUX → AUX | AUX → AUX, H02 | AUX → AUX, H02 | — → H01 / AUX | — → H01 / AUX | — → AUX |
| control | — → AUX | — → AUX | — → AUX, H02 | — → AUX, H02 | — → H01 / AUX | — → H01 / AUX | — → AUX |
| bringup | AUX → AUX | AUX → AUX | AUX → AUX, H02 | AUX → AUX, H02 | — → H01 / AUX | — → H01 / AUX | — → AUX |
| gazebo † | AUX → AUX | AUX → AUX | AUX → AUX, H02 | AUX → AUX, H02 | — → H01 / AUX | — → H01 / AUX | — → AUX |
| moveit † | AUX → AUX | AUX → AUX | AUX → AUX, H02 | AUX → AUX, H02 | — → H01 / AUX | — → H01 / AUX | — → AUX |

Each cell is *the check that showed the failure before → after*; **—** means the report showed nothing and the entry point exited 0 (3 on the dirty tree, which is how the ten failing canaries of the control package ended). `H01 / AUX`: H01 fails an unguarded launch test statically, and a guarded one fails dynamically as an AUX failure. † gazebo and moveit were exercised in the combined A7 run (their tests only run there); description, control and bringup in `verify_a6.sh`, one package at a time, and bringup again in the A7 run.

Re-run the canaries: `python3 scripts/harness_canary.py inject <package dirs>`, `colcon build`, run `verify_a6.sh` or `run_a7_regression.sh`, `... remove <package dirs>`.
They are never left in a tree: a tree with canaries is dirty and a report is not written from it.

## Result values

| Value | Meaning |
|---|---|
| `PASS` | The check ran and its condition held |
| `WARN` | It ran, and found something a reader must know; not a defect in this package. It is not `PASS` |
| `SKIP` | It did not run, with the reason. Not `PASS`. The only legitimate `SKIP` is C02 with no `src/upstream/` checkout |
| `FAIL` | A condition was violated. Always fails the run |
| `NOT_RUN` | The check's backing test did not run to a result: no test for the check, a test that timed out or died of a signal, a sidecar with no test result behind it, a registered test that left no result file. **Never `PASS`, never `SKIP`**, and it fails the exit status like `FAIL`. Added in P12A; the shared vocabulary of `VERIFICATION.md` (H-CI-2) has four values and has to adopt it |

Current result (A6, `evidence/20261007T164436Z_dev.json`): 22 `PASS`, 6 `WARN`, 0 `SKIP`, 0 `FAIL`, 0 `NOT_RUN` with the upstream checkout present.

## The six WARNs

*19 Oct (P22): this table is the 7 Oct state. On the current pin (platform-sw `dfd44a5`) A6 has three WARNs, E01, W02 and V01 (`evidence/reports/README.md`, P20 and P21): W01 was resolved by the root change (P15); T04 passes **by deletion**, because the near-field frames are consumed from platform-sw and no longer declared here (VERIFICATION.md, T04); T08 cleared because platform-sw #70 replaced the six STL collision meshes with boxes and cylinders, not because of any change here.*

| ID | Why a WARN and not a FAIL | What would clear it |
|---|---|---|
| T04 | Six base sensor frames (`tof_*`, `ultrasonic_*`) expected from platform-sw are missing. The base is not ours (ADR-005) | platform-sw adds them (WP-F5, static TFs into URDF sensor macros) |
| T08 | Six platform-sw base links (`base_link`, four casters, `lidar_link`) use the full-detail visual STL as collision mesh: slower planning, not a wrong model | platform-sw supplies simplified collision meshes |
| E01 | Height, width and depth are lower bounds from link origins and primitives; meshes are not loaded. A figure under the limit proves nothing, so it is never `PASS` by design | A4 measures the envelope with meshes. Until then it stays `WARN` |
| W01 | `base_link` has a 4.18 kg inertial and KDL drops the inertia of a root link, so `robot_state_publisher` and MoveIt never see it. The usual workaround changes the contract's root, and the base is platform-sw's | A base-macro refactor by the WP-F5 owner. It matters in A7 |
| W02 | `robot_state_publisher` converts the planar joint to fixed, because KDL has no planar joint. A consumer limitation; A6's acceptance (joint present, `check_urdf` accepts it) is unaffected | Not clearable here. In A7 planar motion comes from Gazebo's `odom -> base_link` |
| V01 | The eight items are measured (P16, `evidence/v01/`, renders in `evidence/renders/`), but no person has looked at the running simulator and RViz, and `lift_mass_tl3_400` is null so one of the two columns was not measured. A numeric check is not a visual one | A person runs the simulator with RViz beside it, looks, and answers the two sub-questions marked CANNOT-MEASURE in [`v01_visual_check.md`](v01_visual_check.md); the datum comes from the supplier |

## Reading a report

Check `git_hash` first (every file in `evidence/` is from a clean commit), then `limitations`, then every non-`PASS` row. T10 prints the modelled mass beside the
120 kg limit and the F2S vehicle estimate (93 kg); the modelled total is what the description contains, not what the robot
weighs. Each mass row carries its provenance state from `config/mass_provenance.yaml`: `placeholder` (no source), `external_nominal`
(another owner's figure, not verified by us: the base CAD export and the upstream arm nominals) or `verified` (measured, with a
document); none is `verified` today.
