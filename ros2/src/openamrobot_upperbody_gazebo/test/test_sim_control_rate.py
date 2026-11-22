# SPDX-License-Identifier: MIT
"""The simulated arm controllers run at the upstream rate: the launch hands gz_ros2_control the simulation overlay, and this
package's world steps so that the overlay's rate is the rate the controllers really update at. Static: no Gazebo is started.

gz_ros2_control (1.2.20, read from its log and measured) updates the controller manager on the first physics step at which the
simulated time since the last update is at least one controller period. Both are whole nanoseconds. So the real update period is
the step times ceil(period / step), and it equals the asked period only if the step divides it.
"""
import math
import os
import re
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_LAUNCH = os.path.join(HERE, os.pardir, "launch", "sim.launch.py")
WORLD = os.path.join(HERE, os.pardir, "worlds", "empty.sdf")
CONTROL = get_package_share_directory("openamrobot_upperbody_control")
UPSTREAM_RATE_HZ = 750


def effective_rate_hz(step_s, rate_hz):
    """The rate the controllers update at for a world step and an asked rate, in gz_ros2_control's nanosecond arithmetic. The step is
    truncated to whole nanoseconds, the worst case for "at least one period"."""
    step_ns, period_ns = math.floor(step_s * 1e9 + 1e-6), int(1e9 / rate_hz)
    return 1e9 / (step_ns * math.ceil(period_ns / step_ns))


def _load(*parts):
    with open(os.path.join(*parts)) as f:
        return yaml.safe_load(f)


def world_step_s():
    physics = ET.parse(WORLD).getroot().find("world/physics")
    return float(physics.findtext("max_step_size")), float(physics.findtext("real_time_factor"))


def test_the_overlay_asks_for_the_upstream_rate_and_the_main_file_does_not():
    assert _load(CONTROL, "config", "sim_overlay.yaml")["controller_manager"]["ros__parameters"]["update_rate"] == UPSTREAM_RATE_HZ
    assert _load(CONTROL, "config", "arm_controllers.yaml")["controller_manager"]["ros__parameters"]["update_rate"] == 100


def test_this_packages_world_steps_so_that_the_controllers_update_at_the_upstream_rate():
    step, real_time_factor = world_step_s()
    assert real_time_factor == 1.0
    rate = effective_rate_hz(step, UPSTREAM_RATE_HZ)
    assert abs(rate - UPSTREAM_RATE_HZ) < 0.01, f"a {step} s step gives {rate:.2f} Hz, not {UPSTREAM_RATE_HZ}"
    assert step <= 0.001, "coarser than platform-sw's 1 ms step, which the base's drive was tuned on"


def test_the_arithmetic_can_fail_a_1_ms_step_gives_500_hz():
    """What was measured before the step changed (499.5 Hz over 5.96 s of simulated time), and why the step changed."""
    assert effective_rate_hz(0.001, UPSTREAM_RATE_HZ) == 500.0
    assert effective_rate_hz(0.001, 100) == 100.0, "the 100 Hz of arm_controllers.yaml was exact on 1 ms steps"
    assert effective_rate_hz(0.000666666, UPSTREAM_RATE_HZ) < 501, "a step a hair under 1/1500 s falls to every third step"


def test_sim_launch_hands_the_overlay_to_the_description_after_the_main_file():
    with open(SIM_LAUNCH) as f:
        source = f.read()
    assert re.search(r'controllers_overlay_file = os\.path\.join\(get_package_share_directory\(CONTROL\), "config", "sim_overlay\.yaml"\)', source)
    main, overlay = source.index('" controllers_file:=", controllers_file,'), source.index('" controllers_overlay_file:=", controllers_overlay_file,')
    assert main < overlay
    assert '" hardware_profile:=gazebo"' in source

