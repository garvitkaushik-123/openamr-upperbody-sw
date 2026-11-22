# SPDX-License-Identifier: MIT
"""Simulator joint dynamics in the expanded URDF: present in the simulated model, absent from the real profile. Expansion only.

  gazebo           <dynamics damping friction> on all 16 controlled arm and gripper joints (and on the two mimic followers),
                   with the values of config/sim_dynamics/openarm_v2.yaml
  real             none on any OpenArm joint, and the expansion is the same text whatever sim_dynamics is asked for
  mock             none by default; sim_dynamics:=on gives the MuJoCo-facing URDF with the same values
  nothing else     with the <dynamics> lines removed, an expansion with them equals the expansion without them, character for
                   character: no limit, origin, axis, mass or interface moves
"""
import os
import re
import subprocess
import tempfile

import pytest
import xacro
import yaml

from conftest import SHARE, expand, expand_xml

PROFILE_FILE = os.path.join(SHARE, "config", "sim_dynamics", "openarm_v2.yaml")
CONTROLLED = [f"openarm_{side}_joint{i}" for side in ("left", "right") for i in range(1, 8)] \
    + [f"openarm_{side}_finger_joint1" for side in ("left", "right")]
FOLLOWERS = [f"openarm_{side}_finger_joint2" for side in ("left", "right")]
OPENARM_DYNAMICS_LINE = re.compile(r'<dynamics damping="[^"]*" friction="[^"]*"/>')
OVERLAY = "/opt/example/share/some_controllers_package/config/sim_overlay.yaml"          # paths the description never reads
CONTROLLERS = "/opt/example/share/some_controllers_package/config/arm_controllers.yaml"


@pytest.fixture(scope="module")
def profile():
    with open(PROFILE_FILE) as f:
        return yaml.safe_load(f)


def openarm_dynamics(root):
    """{joint: [<dynamics> elements]} for every OpenArm joint of an expansion."""
    return {j.get("name"): j.findall("dynamics") for j in root.findall("joint") if j.get("name").startswith("openarm_")}


def other_dynamics(root):
    """(joint, damping, friction) of every <dynamics> that is not on an OpenArm joint (platform-sw's wheels and castors)."""
    return sorted((j.get("name"), d.get("damping"), d.get("friction"))
                  for j in root.findall("joint") if not j.get("name").startswith("openarm_") for d in j.findall("dynamics"))


def without_openarm_dynamics(root_text, joints):
    """The expansion text with the <dynamics> element of the named joints removed. The profile's lines are told from platform-sw's
    own by the joint they sit in, not by their look."""
    out, removed = root_text, 0
    for name in joints:
        start = out.index(f'<joint name="{name}"')
        end = out.index("</joint>", start)
        body, count = OPENARM_DYNAMICS_LINE.subn("", out[start:end])
        removed += count
        out = out[:start] + body + out[end:]
    return out, removed


def test_sim_dynamics_gazebo_has_dynamics_on_all_16_arm_and_gripper_joints(profile):
    found = openarm_dynamics(expand(hardware_profile="gazebo"))
    assert len(CONTROLLED) == 16
    for name in CONTROLLED + FOLLOWERS:
        assert len(found[name]) == 1, f"{name}: {len(found[name])} <dynamics> elements"
        element, entry = found[name][0], profile["joints"][name]
        assert set(element.attrib) == {"damping", "friction"}, name
        assert float(element.get("damping")) == entry["damping"]["value"], name
        assert float(element.get("friction")) == entry["friction"]["value"], name
    assert sorted(n for n, elements in found.items() if elements) == sorted(CONTROLLED + FOLLOWERS), "a fixed joint got <dynamics>"


def test_sim_dynamics_the_arm_joints_are_damped_and_the_values_are_not_all_one_number(profile):
    """Guards a profile or macro that degenerates to zeros or to one value for every joint."""
    found = openarm_dynamics(expand(hardware_profile="gazebo"))
    damping = {name: float(found[name][0].get("damping")) for name in CONTROLLED}
    friction = {name: float(found[name][0].get("friction")) for name in CONTROLLED}
    assert all(value > 0 for value in damping.values())
    assert all(friction[f"openarm_{side}_joint{i}"] > 0 for side in ("left", "right") for i in range(1, 8))
    assert len(set(damping.values())) >= 3 and len(set(friction.values())) >= 4


@pytest.mark.parametrize("sim_dynamics", [None, "auto", "off"])
def test_sim_dynamics_the_real_profile_has_none(sim_dynamics):
    args = {"hardware_profile": "real", **({"sim_dynamics": sim_dynamics} if sim_dynamics else {})}
    found = openarm_dynamics(expand(**args))
    assert len(found) >= 18 and not any(found.values()), sorted(n for n, e in found.items() if e)
    assert expand_xml(**args) == expand_xml(hardware_profile="real"), "sim_dynamics changed the real expansion"


def test_sim_dynamics_on_is_refused_for_the_real_profile():
    with pytest.raises(xacro.XacroException) as error:
        expand_xml(hardware_profile="real", sim_dynamics="on")
    assert "never applied to the real-hardware profile" in str(error.value)


def test_sim_dynamics_the_mock_profile_has_none_by_default():
    assert not any(openarm_dynamics(expand(hardware_profile="mock")).values())
    assert expand_xml(hardware_profile="mock", sim_dynamics="off") == expand_xml(hardware_profile="mock")


def test_sim_dynamics_on_gives_the_mujoco_facing_urdf_with_the_same_values():
    mujoco_facing, gazebo = (openarm_dynamics(expand(**args)) for args in (dict(hardware_profile="mock", sim_dynamics="on"),
                                                                           dict(hardware_profile="gazebo")))
    assert {n: [e.attrib for e in elements] for n, elements in mujoco_facing.items()} == \
           {n: [e.attrib for e in elements] for n, elements in gazebo.items()}
    assert all(len(mujoco_facing[name]) == 1 for name in CONTROLLED + FOLLOWERS)


@pytest.mark.parametrize("with_dynamics, without", [
    (dict(hardware_profile="gazebo"), dict(hardware_profile="gazebo", sim_dynamics="off")),
    (dict(hardware_profile="mock", sim_dynamics="on"), dict(hardware_profile="mock")),
])
def test_sim_dynamics_is_the_only_difference(with_dynamics, without):
    """18 elements are added and nothing else moves: the limits, origins, axes, inertials and interfaces are the same characters."""
    stripped, removed = without_openarm_dynamics(expand_xml(**with_dynamics), CONTROLLED + FOLLOWERS)
    assert removed == 18
    assert stripped == expand_xml(**without)


@pytest.mark.parametrize("args", [dict(hardware_profile="gazebo"), dict(hardware_profile="gazebo", sim_dynamics="off"),
                                  dict(hardware_profile="mock"), dict(hardware_profile="real")])
def test_sim_dynamics_leaves_platform_sws_own_dynamics_alone(args):
    assert other_dynamics(expand(**args)) == other_dynamics(expand(hardware_profile="mock"))
    assert other_dynamics(expand(**args)), "platform-sw's wheel and castor <dynamics> are gone"


def test_sim_dynamics_joint_limits_are_the_same_in_every_profile():
    def limits(root):
        return {j.get("name"): dict(j.find("limit").attrib) for j in root.findall("joint")
                if j.get("name").startswith("openarm_") and j.find("limit") is not None}
    real = limits(expand(hardware_profile="real"))
    assert len(real) == 18
    assert limits(expand(hardware_profile="gazebo")) == real == limits(expand(hardware_profile="mock", sim_dynamics="on"))


def test_sim_dynamics_an_unknown_value_is_refused_with_the_accepted_list():
    with pytest.raises(xacro.XacroException) as error:
        expand_xml(sim_dynamics="yes")
    assert "sim_dynamics" in str(error.value) and "auto, on, off" in str(error.value)


@pytest.mark.parametrize("arm_package", ["none", "so101"])
def test_sim_dynamics_without_openarm_there_is_nothing_to_apply(arm_package):
    assert expand_xml(arm_package=arm_package, sim_dynamics="on") == expand_xml(arm_package=arm_package)


def test_sim_dynamics_the_gazebo_expansion_is_valid_urdf():
    with tempfile.NamedTemporaryFile("w", suffix=".urdf") as f:
        f.write(expand_xml(hardware_profile="gazebo"))
        f.flush()
        result = subprocess.run(["check_urdf", f.name], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


# --- the simulation overlay of the controller parameters (controllers_overlay_file) --------------------------------------------------

def _parameters(**args):
    root = expand(**args)
    return [p.text for g in root.findall("gazebo") for plugin in g.findall("plugin") for p in plugin.findall("parameters")]


def test_sim_overlay_is_a_second_parameters_element_after_the_first():
    assert _parameters(hardware_profile="gazebo", controllers_file=CONTROLLERS, controllers_overlay_file=OVERLAY) == [CONTROLLERS, OVERLAY]
    assert _parameters(hardware_profile="gazebo", controllers_file=CONTROLLERS) == [CONTROLLERS]
    assert _parameters(hardware_profile="gazebo") == []


def test_sim_overlay_adds_one_element_and_nothing_else():
    default = expand_xml(hardware_profile="gazebo", controllers_file=CONTROLLERS)
    given = expand_xml(hardware_profile="gazebo", controllers_file=CONTROLLERS, controllers_overlay_file=OVERLAY)
    assert given != default and given.replace(f"<parameters>{OVERLAY}</parameters>", "") == default


@pytest.mark.parametrize("profile_name", ["mock", "real"])
def test_sim_overlay_is_ignored_by_the_other_profiles(profile_name):
    assert expand_xml(hardware_profile=profile_name, controllers_overlay_file=OVERLAY) == expand_xml(hardware_profile=profile_name)
