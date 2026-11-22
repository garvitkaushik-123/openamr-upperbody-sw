# openamrobot_openarm_emulator

Software in the loop for the OpenArm 2.0 arms. The real upstream driver (`openarm_hardware/OpenArmHW`, not modified) talks
over Linux virtual CAN to emulated Damiao motors. The motors' motion comes from Gazebo.

Simulation only. Nothing in this package is a safety function. A result from it is development evidence. It is never
hardware evidence, physical acceptance or safety evidence. The combined model is not changed.

Design input: the digital-twin audit of 28 Sep 2026 (`openAMRobot/audits`, branch `audit/2026-09-28`,
`digital-twin/openarm/openarm_interface.md` and `findings.md`).

## What runs

```
 root namespace: what would run on the robot          /sim namespace: the plant
 ----------------------------------------------       ---------------------------------------------
 arm and gripper controllers                          gz sim, combined model, hardware_profile gazebo
   | position commands                                  ^ joint effort        | joint position, velocity
 /controller_manager (ros2_control_node)              /sim/controller_manager (gz_ros2_control)
   OpenArmHW left      OpenArmHW right                  left_arm_emulator     right_arm_emulator
        |                   |                                |                      |
        +---- vcan_arm_l ---|--------------------------------+                      |
                            +---- vcan_arm_r ---------------------------------------+
```

- One emulator per arm, each on its own SocketCAN interface: `vcan_arm_l` and `vcan_arm_r`. Each plays eight motors:
  J1 to J7 and the gripper, ids 0x01 to 0x08, replies on 0x11 to 0x18.
- Joint motion and torque come from Gazebo **through ros2_control**. Each emulator is a ros2_control controller
  (`openamrobot_openarm_emulator/OpenArmCanEmulator`) in the controller manager that `gz_ros2_control` hosts inside the
  simulator. Every physics step it reads the eight joints' position and velocity state interfaces, runs the motors'
  control laws, and writes the eight effort command interfaces. There is no joint-state topic bridge in the loop.
- The emulator computes the MIT law, not Gazebo. The impedance stays on the motor side, as on the real arm. The Gazebo
  position servo of the normal `gazebo` profile is not used for the arms here.
- `/joint_states` is what the driver believes. `/sim/joint_states` is what the physics does. Most fault cases are about
  the difference between the two.

## Build

The driver and its CAN library are not released for Jazzy. Build them from source in the same workspace, at the commits
of `config/sil_pins.yaml`:

| Repository | Commit | Note |
|---|---|---|
| enactic/openarm_ros2 | `4e837e1d0dae692ff67b560b69d8d281d7a8d4ed` | the driver. Same pin as the model's `upstream_pins.yaml` |
| enactic/openarm_can | `bfd4e52194aefe38389fa6e670be3738136facb9` | tag 1.4.1. Upstream openarm_ros2 does not pin it |
| enactic/openarm_description | `14ff67b638ff1c738a1b9a6be8aaa5ce5ed2c831` | same pin as the model |
| enactic/openarm | `e15e09e056098348d6f437389bbf48179caa82b6` | CAN setup document, read only |

```bash
cd ws/src
git clone https://github.com/enactic/openarm_ros2 upstream/openarm_ros2 && git -C upstream/openarm_ros2 checkout 4e837e1d0dae692ff67b560b69d8d281d7a8d4ed
git clone https://github.com/enactic/openarm_can upstream/openarm_can && git -C upstream/openarm_can checkout bfd4e52194aefe38389fa6e670be3738136facb9
cd .. && rosdep install --from-paths $(colcon list --packages-up-to openamrobot_openarm_emulator --paths-only) --ignore-src -y
colcon build --packages-up-to openamrobot_openarm_emulator
```

## Run

```bash
ros2 launch openamrobot_openarm_emulator sil.launch.py                 # with the Gazebo window
ros2 launch openamrobot_openarm_emulator sil.launch.py headless:=true  # server only
```

The launch creates the interfaces, starts Gazebo with the combined model, both emulators, the real driver and an example
trajectory (`scripts/example_arm_trajectory.py`: both arms out to a bent pose and back). Arguments: see the launch file's
header. `example_trajectory:=false` leaves the trajectory out.

When `ros2_control_node` activates the hardware, the driver enables the motors and drives every arm joint to zero. That
is upstream behaviour (see "What the driver does today"). In the simulation the arms hang limp until then.

### Virtual CAN and CAN FD

`scripts/setup_vcan.sh` creates the interfaces (the launch runs it; `create_vcan:=false` skips it). It needs root or
passwordless sudo, and the `vcan` kernel module (Ubuntu: `linux-modules-extra-$(uname -r)`).

```bash
ros2 run openamrobot_openarm_emulator setup_vcan.sh             # vcan_arm_l and vcan_arm_r
ros2 run openamrobot_openarm_emulator setup_vcan.sh --delete
```

It sets MTU 72 so the interface carries CAN FD frames, and prints `CAN FD yes (mtu 72)` or `CAN FD NO (mtu 16)` per
interface. With `can_fd:=auto` (default) the launch gives the driver `can_fd true` only when both interfaces have MTU 72.
Otherwise it says so and runs the driver with classic CAN frames. The emulator answers in the frame type it was asked in.

vcan has no bit rate, no sample point, no acknowledgement and no bus-off. The real bus is 1 Mbit/s arbitration and
5 Mbit/s data. The emulator models frame time in software (below). It does not model error frames or bus-off.

## Frames

Layout source: enactic/openarm_can 1.4.1, `dm_motor_control.cpp`. The codec here is written independently and
`test_codec` compares it bit for bit with the upstream encoder and decoder.

| Host frame | CAN id | Emulated motor |
|---|---|---|
| Enable `FF FF FF FF FF FF FF FC` | send id | enabled, setpoints cleared. State reply |
| Disable `.. FD` | send id | disabled, no torque. State reply |
| Set zero `.. FE` | send id | position offset. State reply |
| Clear error `.. FB` | send id | clears a latched fault. State reply |
| MIT control | send id | stored when enabled and in MIT mode. State reply |
| Position-velocity | send id + 0x100 | stored when enabled and in that mode. State reply |
| Velocity, position-force | send id + 0x200, + 0x300 | as above |
| Refresh `idLo idHi CC` | 0x7FF | State reply |
| Read parameter `idLo idHi 33 RID` | 0x7FF | Parameter reply with the value |
| Write parameter `idLo idHi 55 RID value` | 0x7FF | stored in RAM, echoed. RID 10 changes the control mode |
| Save `idLo idHi AA` | 0x7FF | RAM to flash when disabled, reply `AA 01` |

State reply, on the reply id, 8 bytes: D0 low nibble motor id and high nibble status or error code; D1 D2 position
(16 bits, -12.5 to 12.5 rad); D3 and D4 high nibble velocity (12 bits); D4 low nibble and D5 torque (12 bits); D6 MOSFET
temperature; D7 rotor temperature. Codes: 0 disabled, 1 enabled, 8 over-voltage, 9 under-voltage, A over-current,
B MOS over-temperature, C coil over-temperature, D communication lost, E overload.

Position and velocity are the joint's, from Gazebo. Torque is what the emulated motor applied. No gear ratio and no sign,
as in the driver.

Parameters with a value: UV_Value (0), OT_Value (2), MST_ID (7), ESC_ID (8), TIMEOUT (9), CTRL_MODE (10), PMAX (21),
VMAX (22), TMAX (23), OV_Value (29). Any other register reads 0. Writing TIMEOUT stores the value and has no effect.

## Timing

vcan delivers a frame at once. A wire does not. With `timing.model: true` (default) the emulator keeps a model of one
shared bus per arm: each frame, the host's and the motors', takes its length in bits, one frame at a time, lowest id
first. An 8-byte CAN FD frame with BRS at 1 and 5 Mbit/s is about 52 us in this model (10 % stuff bits assumed). The
audit estimates about 50 us. A reply is put on vcan when its modelled transmission ends.

Effect on the unmodified driver: `read()` sends eight refresh frames and waits up to 500 us for the first reply. The
first reply ends after two frame times, the last after sixteen (about 0.83 ms). So `read()` usually takes one fresh
state and the rest arrive in `write()` or in the next cycle. This is the model's prediction. It is not measured on
hardware. `timing.turnaround_us` (motor reply latency) is 0 until it is measured. `timing.model: false` answers at once.

The driver runs on wall time. The world (`worlds/sil_empty.sdf`) runs 1 ms physics at real time and has no sensor
rendering, so a machine without a GPU can keep real time. If the simulation runs slower than real time, the emulated arm
is slower than the driver expects.

## Damping and the physics step

A numerical treatment, on by default (`motor.implicit_damping`). It is not a model of the motor.

The MIT law's damping gain is stiff for the light links: kd times the 1 ms step, divided by the link inertia about its
joint, is up to 6. An explicit step is only stable below 2. Without a treatment the wrist and upper-arm joints chatter
at torque saturation in Gazebo (seen while this package was written: plus and minus 25 rad/s every step). The real motor does not have
this problem. It closes its loop faster, on a rotor with reflected inertia. Neither is modelled, and no source in scope
gives the rotor inertia (audit gap 3).

The emulator therefore low-passes its applied torque at `motor.damping_split_hz` (200 Hz). What the law asks beyond that
slow part is divided by `1 + kd * dt / (0.5 * damping_inertia)`, which is what an implicit step of the damping term
gives. `damping_inertia` is each joint's child link about the joint axis, from the URDF (`test_package_static`
recomputes it). Torque at rest and motion well below 200 Hz are the law's own. `test_motor_model` shows the chatter
without it, the stable response with it, and unchanged static torque.

**The simulator dynamics profile is switched off in this launch.** Since 15 Nov 2026 the `gazebo` profile of the combined model carries
joint damping and friction from upstream's MuJoCo model (0.6 to 1.0 N*m*s/rad). The physics steps that damping explicitly. With the
link inertias about the joints (1.1e-4 to 2.5e-3 kg*m^2) and the 1 ms step, damping times the step over the inertia is 0.35 to 6.6, above
the limit of 2 on four of the eight joints of an arm, the same problem as the MIT gain above. Upstream's values go with a reflected rotor inertia (0.002 to
0.030 kg*m^2) that would bring the ratio down to 0.03 to 0.42, and URDF cannot carry it. Seen in CI on the first merge of the two
changes: with the damping on, the right arm, which should hold at zero, read -1.33 rad. So `sil.launch.py` passes `sim_dynamics:=off`
and this package behaves as it did before the profile existed. The position-servo simulation (`openamrobot_upperbody_gazebo`) is not
affected: there the joint is driven by a velocity constraint, and its tests pass with the damping on. Using the profile here needs
the rotor inertia in the physics first.

## Supply voltage

`supply_voltage` (default 25.6 V) is the arm supply the motors see. The robot's range is 25.6 to 29.2 V, and the
emulator accepts any value. Two effects, both assumptions:

- No-load speed scales with supply over 24 V. The torque left for driving falls linearly to zero at that speed. Braking
  is not limited. `motor.voltage_speed_model: false` turns this off.
- Above `motor.over_voltage_v` (32 V) every motor trips 0x8. Below `motor.under_voltage_v` (15 V) it trips 0x9.

## Fault injection

Each fault is a parameter of `/sim/left_arm_emulator` or `/sim/right_arm_emulator`. `inject_fault.py` sets them.

```bash
ros2 run openamrobot_openarm_emulator inject_fault.py left estop on        # E-stop power cut
ros2 run openamrobot_openarm_emulator inject_fault.py left estop off       # reset
ros2 run openamrobot_openarm_emulator inject_fault.py left can_loss on
ros2 run openamrobot_openarm_emulator inject_fault.py left silent 4 on     # J4 stops answering
ros2 run openamrobot_openarm_emulator inject_fault.py both over_voltage on
ros2 run openamrobot_openarm_emulator inject_fault.py right over_temperature 2 on
ros2 run openamrobot_openarm_emulator inject_fault.py both supply 29.2
ros2 run openamrobot_openarm_emulator inject_fault.py both clear
sudo ip link set vcan_arm_l down                                           # the interface itself
```

| Fault | Parameter | Emulated motors | What the unmodified driver shows today |
|---|---|---|---|
| CAN loss | `fault.can_loss` | nothing reaches them, nothing comes back. They hold their last setpoint | `read()` and `write()` return OK. State frozen at the last value. Stream resumes by itself |
| Interface down | `ip link set down` | as CAN loss | every write fails with ENETDOWN inside openarm_can. The driver still returns OK |
| Single-motor timeout | `fault.silent_motors` | one motor runs and does not answer | that joint's state is frozen while the joint moves. OK returned |
| Over-voltage | `supply_voltage` above 32 V, or `fault.error_codes` 8 | code 0x8, no torque, still answering | the joints go limp. The state shows it. The code is decoded and never checked. OK returned |
| Over-temperature | `fault.temperature_mos_c`, `fault.temperature_rotor_c`, `fault.temperature_rate_c_per_s` | temperature ramps in D6 or D7, then code 0xB or 0xC, no torque | as over-voltage. Temperature is exported nowhere |
| E-stop power cut | `fault.estop_power_cut: true` | bus up, every motor silent, no torque | the arm falls in the physics. The driver's state is frozen. Hardware component and controllers stay active |
| Reset | `fault.estop_power_cut: false` | motors answer again, disabled, positions kept | state is live again and shows the fallen arm. The driver never sends enable by itself. Only deactivate and activate re-enable, and that drives the arm to zero |
| Any fault code | `fault.error_codes` (8 to 14 per motor) | that code, no torque | as over-voltage |

A fault stays until a clear-error frame or a power cycle. The driver sends neither.

`/sim/<side>_arm_emulator/status` (diagnostic_msgs/DiagnosticArray, 10 Hz) reports the bus and each motor: powered,
enabled, code, position, torque, temperatures, frame counts, and the first MIT frame after the last enable.

## What the driver does today

Recorded by the tests against upstream `OpenArmHW` at the pin. These are records, not approvals. This package does not
fix the driver.

1. **Move to zero on activate.** After enable, the first MIT frame to each arm joint is position 0 at full gains, sent
   wherever the joint is. Then a 2 s ramp to zero. Nothing plans the motion or checks it for collision. Activation
   blocks for it (about 2.2 s per arm).
2. **Gripper command on activate.** The gripper is sent 0.044 rad: the 1.0 gripper's open width in metres, unconverted.
3. **Healthy report on CAN loss.** `read()` and `write()` always return OK. A silent bus, a downed interface and a
   power cut all leave the last state in place with no sign that it is stale.
4. **Motor fault codes are not acted on.** The code is decoded and never read. Temperature is not exported.
5. **No re-enable after a power cut.** Enable is sent only in `on_activate`.
6. **Gripper velocity and effort states are 0**, hard-coded.

## Assumptions

No source in scope fixes these. Each is a parameter with a default. Measure on the bench, then set the value.

| Parameter | Default | Open question |
|---|---|---|
| `motor.answers_when_disabled` | true | does a disabled motor answer refresh? |
| `motor.power_up_enabled` | false | motor state after supply returns |
| `motor.enable_clears_fault` | false | does enable clear a fault, or only clear-error? |
| `motor.can_timeout_s` | 0 (off) | the TIMEOUT register as shipped, and its unit |
| `motor.over_voltage_v`, `motor.under_voltage_v` | 32, 15 | thresholds (registers 29 and 0) |
| `motor.over_temperature_c` | 120 | threshold (register 2) |
| `motor.voltage_speed_model` | true | speed and torque at 25.6 to 29.2 V |
| `timing.turnaround_us` | 0 | motor reply latency and jitter |
| `reply_brs` | true | BRS flag on motor replies |
| position-velocity, velocity and position-force laws | simple cascade | the firmware's own loops and gains |
| joint friction, damping, rotor inertia | none: the launch expands the model with `sim_dynamics:=off` | audit gap 3. The simulator dynamics profile (`openamrobot_upperbody_description/config/sim_dynamics/openarm_v2.yaml`) has sourced damping and friction, but they cannot be used here yet, see below |

Also not emulated: error frames, bus-off, dropped or late frames, wrong ids, over-current and overload detection,
a thermal model, the gripper's motor sign per hand.

## Tests

```bash
sudo modprobe vcan && ros2 run openamrobot_openarm_emulator setup_vcan.sh
colcon test --packages-select openamrobot_openarm_emulator && colcon test-result --verbose
```

| Test | Needs | What it checks |
|---|---|---|
| `test_codec` | openarm_can | frame bytes, then encode and decode against the upstream library |
| `test_motor_model` | nothing | motor state machine, parameters, faults, supply voltage, damping, bus timing |
| `test_real_driver` | vcan, driver | the real driver enables, reads and commands a joint through the emulator. Each fault. Both arms |
| `test_package_static` | nothing | pins, configuration, launch logic, `setup_vcan.sh` |
| `test_sil_launch` | vcan, driver, Gazebo | the whole launch headless: trajectory, E-stop power cut, reset |

A test that needs vcan or the driver and cannot have it reports **BLOCKED** (a skip with that word). It never passes.
`scripts/count_sil_tests.py` counts passed, failed, BLOCKED and skipped apart. The CI job
(`.github/workflows/openarm-emulator-tests.yml`) fails when a test is BLOCKED.
