# SPDX-License-Identifier: MIT
"""sim_overlay.yaml: the simulation-only controller parameters, read after arm_controllers.yaml by the Gazebo profile.

Each value is a copy of the simulator dynamics profile (openamrobot_upperbody_description config/sim_dynamics/openarm_v2.yaml),
which carries its source. The overlay may change values of controllers that exist; it may not add a controller, a joint or an
interface, and arm_controllers.yaml (every profile, the real one included) stays at 100 Hz with no max_effort.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory

CONTROL = get_package_share_directory("openamrobot_upperbody_control")
DESCRIPTION = get_package_share_directory("openamrobot_upperbody_description")
GRIPPER_CONTROLLERS = ("left_gripper_controller", "right_gripper_controller")


def _load(*parts):
    with open(os.path.join(*parts)) as f:
        return yaml.safe_load(f)


def _overlay():
    return _load(CONTROL, "config", "sim_overlay.yaml")


def _profile():
    return _load(DESCRIPTION, "config", "sim_dynamics", "openarm_v2.yaml")


def test_sim_overlay_sets_the_upstream_update_rate_of_the_profile():
    rate = _profile()["controller"]["update_rate"]
    assert _overlay()["controller_manager"]["ros__parameters"] == {"update_rate": rate["value"]}
    assert rate["value"] == 750 and rate["unit"] == "Hz"


def test_sim_overlay_sets_the_gripper_max_effort_of_the_profile_on_both_hands():
    torque = _profile()["gripper"]["grip_limit"]["torque"]
    assert torque["unit"] == "N*m"
    for name in GRIPPER_CONTROLLERS:
        assert _overlay()[name]["ros__parameters"] == {"max_effort": torque["value"]}, name


def test_sim_overlay_only_touches_controllers_that_exist_and_adds_none():
    overlay, base = _overlay(), _load(CONTROL, "config", "arm_controllers.yaml")
    assert sorted(overlay) == sorted(("controller_manager",) + GRIPPER_CONTROLLERS)
    assert set(overlay) <= set(base)
    for name, block in overlay.items():
        assert list(block) == ["ros__parameters"], name
    assert not any(isinstance(value, dict) for value in overlay["controller_manager"]["ros__parameters"].values()), \
        "the overlay must not declare a controller"


def test_the_file_every_profile_reads_is_unchanged_by_the_overlay():
    """The real profile reads arm_controllers.yaml alone: 100 Hz, and no max_effort on the gripper controllers."""
    base = _load(CONTROL, "config", "arm_controllers.yaml")
    assert base["controller_manager"]["ros__parameters"]["update_rate"] == 100
    for name in GRIPPER_CONTROLLERS:
        assert "max_effort" not in base[name]["ros__parameters"], name
