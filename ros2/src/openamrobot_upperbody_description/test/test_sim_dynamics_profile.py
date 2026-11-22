# SPDX-License-Identifier: MIT
"""The simulator dynamics profile (config/sim_dynamics/openarm_v2.yaml): schema and provenance. No ROS graph.

The profile is the one place where joint damping, friction and armature of the OpenArm joints are written, each with where it
comes from. These tests fail a value without a source, an estimate without a method, a source that is not a full commit, a pin
that differs from config/upstream_pins.yaml, a joint list that differs from the model, and a limit copy that differs from the URDF.
"""
import json
import os
import re

import jsonschema
import pytest
import yaml

from conftest import SHARE, expand

PROFILE_FILE = os.path.join(SHARE, "config", "sim_dynamics", "openarm_v2.yaml")
SCHEMA_FILE = os.path.join(SHARE, "config", "sim_dynamics", "sim_dynamics.schema.json")
PINS_FILE = os.path.join(SHARE, "config", "upstream_pins.yaml")
LEDGER_FILE = os.path.join(SHARE, "config", "actuator_ledger.yaml")      # upperbody-sw pull request 65; not on main when this was written
QUANTITY_KEYS = {"value", "unit", "status", "source", "locator"}
JOINT_QUANTITIES = ("damping", "friction", "armature", "effort_limit", "velocity_limit")
UNIT_OF = {"damping": "N*m*s/rad", "friction": "N*m", "armature": "kg*m^2", "effort_limit": "N*m", "velocity_limit": "rad/s"}
GEAR_RATIO_FIELD = "gear_ratio"


def load(path):
    with open(path) as f:
        return yaml.safe_load(f) if path.endswith(".yaml") else json.load(f)


@pytest.fixture(scope="module")
def profile():
    return load(PROFILE_FILE)


def quantities(node, path=""):
    """Every {value, unit, status, source, locator} mapping in the profile, with its path."""
    if isinstance(node, dict):
        if QUANTITY_KEYS <= set(node):
            yield path, node
            return
        for key, value in node.items():
            yield from quantities(value, f"{path}/{key}")
    elif isinstance(node, list):
        for index, value in enumerate(node):
            yield from quantities(value, f"{path}/{index}")


def movable_openarm_joints(root):
    return {j.get("name"): j for j in root.findall("joint") if j.get("name").startswith("openarm_") and j.get("type") != "fixed"}


def test_sim_dynamics_profile_matches_its_schema(profile):
    schema = load(SCHEMA_FILE)
    jsonschema.Draft7Validator.check_schema(schema)
    errors = sorted(jsonschema.Draft7Validator(schema).iter_errors(profile), key=lambda e: list(e.path))
    assert not errors, [f"{'/'.join(str(p) for p in e.path)}: {e.message}" for e in errors[:5]]


def test_sim_dynamics_schema_rejects_a_value_without_a_source_and_an_estimate_without_a_method(profile):
    """The schema can fail: the two defects it exists for, each made on a copy."""
    validator = jsonschema.Draft7Validator(load(SCHEMA_FILE))
    no_source = json.loads(json.dumps(profile))
    del no_source["motor_classes"]["DM8009"]["damping"]["source"]
    assert any("source" in e.message for e in validator.iter_errors(no_source))
    no_method = json.loads(json.dumps(profile))
    del no_method["motor_classes"]["DM8009"]["rotor_inertia_implied"]["method"]
    assert any("method" in e.message for e in validator.iter_errors(no_method))
    invented = json.loads(json.dumps(profile))
    invented["motor_classes"]["DM8009"]["damping"]["status"] = "guessed"
    assert list(validator.iter_errors(invented))


def test_sim_dynamics_every_quantity_names_a_listed_source_and_a_locator(profile):
    found = list(quantities(profile))
    assert len(found) >= 18 * len(JOINT_QUANTITIES), "the per-joint quantities are missing"
    for path, quantity in found:
        assert quantity["source"] in profile["sources"], f"{path}: source {quantity['source']!r} is not a key of sources"
        assert len(quantity["locator"].strip()) >= 10, f"{path}: the locator does not say where in the source the value is"
        assert isinstance(quantity["value"], (int, float)) and not isinstance(quantity["value"], bool), path
        assert quantity["value"] >= 0, f"{path}: a negative value"
        if quantity["status"] == "estimated":
            assert len(quantity.get("method", "").strip()) >= 20, f"{path}: an estimate needs its method"


def test_sim_dynamics_every_source_is_a_full_commit_and_the_pins_are_the_models_pins(profile):
    pins = load(PINS_FILE)["pins"]
    for name, source in profile["sources"].items():
        assert re.fullmatch(r"[0-9a-f]{40}", source["commit"]), f"{name}: {source['commit']!r} is not a full commit hash"
    assert profile["sources"]["openarm_description_pin"]["commit"] == pins["openarm_description"]
    assert profile["sources"]["openarm_ros2_pin"]["commit"] == pins["openarm_ros2"]
    used = {quantity["source"] for _, quantity in quantities(profile)}
    assert "openarm_mujoco" in used and "openarm_description_pin" in used


def test_sim_dynamics_upstream_friction_yaml_is_still_empty_at_the_pin():
    """The profile takes joint friction from the upstream MuJoCo model because the pinned description's own friction.yaml holds
    nothing. If a pin move fills that file, this fails: read it then, and prefer it."""
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory("openarm_description"), "assets", "robot", "openarm_v2.0", "config", "arm",
                        "control", "friction.yaml")
    assert os.path.isfile(path), f"{path} is gone: the upstream layout changed, re-check the profile's sources"
    assert load(path) is None, "upstream friction.yaml now has content: take the joint friction from it and update the profile"


def test_sim_dynamics_joints_are_exactly_the_movable_openarm_joints_of_the_model(profile):
    root = expand(hardware_profile="gazebo")
    in_model = movable_openarm_joints(root)
    assert sorted(profile["joints"]) == sorted(in_model)
    declared = {j.get("name") for block in root.findall("ros2_control") for j in block.findall("joint")} - {"lift_joint"}
    controlled = {name for name, joint in profile["joints"].items() if joint["controlled"]}
    assert controlled == declared and len(controlled) == 16
    followers = set(profile["joints"]) - controlled
    assert followers == {name for name, joint in in_model.items() if joint.find("mimic") is not None}
    assert sorted(profile["gripper"]["joints"]) == sorted(n for n in controlled if "finger" in n)


def test_sim_dynamics_units_and_statuses_of_the_joint_quantities(profile):
    for name, joint in profile["joints"].items():
        for key in JOINT_QUANTITIES:
            assert joint[key]["unit"] == UNIT_OF[key], f"{name}/{key}"
        for key in ("damping", "friction", "armature"):
            assert joint[key]["status"] in ("upstream_sim_asset", "datasheet", "estimated"), f"{name}/{key}"
            assert joint[key] == profile["motor_classes"][joint["motor_class"]][key], f"{name}/{key} differs from its motor class"
        for key in ("effort_limit", "velocity_limit"):
            assert joint[key]["status"] == "upstream_urdf", f"{name}/{key}"


def test_sim_dynamics_limit_copies_equal_the_urdf_limits(profile):
    """The profile does not set limits; its copies must equal what the pinned upstream description gives every profile."""
    for hardware_profile in ("mock", "gazebo", "real"):
        in_model = movable_openarm_joints(expand(hardware_profile=hardware_profile))
        for name, joint in profile["joints"].items():
            limit = in_model[name].find("limit")
            assert float(limit.get("effort")) == pytest.approx(joint["effort_limit"]["value"], abs=1e-9), (hardware_profile, name)
            assert float(limit.get("velocity")) == pytest.approx(joint["velocity_limit"]["value"], abs=1e-9), (hardware_profile, name)


def test_sim_dynamics_both_fingers_of_a_hand_have_the_same_values(profile):
    """urdf/sim_dynamics.xacro gives finger_joint2 the finger_joint1 entry (it cannot tell the two calls apart). That is only right
    while the profile says the same for both."""
    for side in ("left", "right"):
        one, two = (profile["joints"][f"openarm_{side}_finger_joint{i}"] for i in (1, 2))
        for key in JOINT_QUANTITIES:
            assert one[key] == two[key], f"{side} finger {key}"
        assert one["controlled"] and not two["controlled"]


def test_sim_dynamics_left_and_right_are_the_same(profile):
    for name, joint in profile["joints"].items():
        if "_left_" in name:
            assert joint == profile["joints"][name.replace("_left_", "_right_")], name


def test_sim_dynamics_the_grip_limit_is_within_the_urdf_effort_and_the_rate_is_upstreams(profile):
    grip = profile["gripper"]["grip_limit"]
    finger = profile["joints"][profile["gripper"]["joints"][0]]
    assert 0 < grip["torque"]["value"] <= finger["effort_limit"]["value"]
    assert grip["torque"]["unit"] == "N*m" and grip["current"]["unit"] == "A" and grip["torque"]["status"] == "datasheet"
    assert profile["controller"]["update_rate"]["value"] == 750 and profile["controller"]["update_rate"]["unit"] == "Hz"
    assert profile["controller"]["update_rate"]["source"] == "openarm_ros2_pin"


def test_sim_dynamics_the_profile_never_applies_to_the_real_profile(profile):
    assert profile["applies_to"]["never"] == ["real"] and "real" not in profile["applies_to"]["hardware_profiles"]


def test_sim_dynamics_agrees_with_the_actuator_ledger_once_it_is_on_main(profile):
    """The motor data lives in the actuator ledger (pull request 65). When that file is present: every controlled joint has a
    ledger entry, the grip limit is the ledger's rated torque and current, and each implied rotor inertia is armature / ratio^2."""
    if not os.path.isfile(LEDGER_FILE):
        pytest.skip("config/actuator_ledger.yaml is not in this tree (upperbody-sw pull request 65 is not merged): nothing to compare")
    ledger = {entry["id"]: entry for entry in load(LEDGER_FILE)["actuators"]}
    controlled = [name for name, joint in profile["joints"].items() if joint["controlled"]]
    assert set(controlled) <= set(ledger), sorted(set(controlled) - set(ledger))
    finger = ledger[profile["gripper"]["joints"][0]]
    assert profile["gripper"]["grip_limit"]["torque"]["value"] == pytest.approx(finger["rated_torque"]["value"])
    assert profile["gripper"]["grip_limit"]["current"]["value"] == pytest.approx(finger["rated_current"]["value"])
    for name, motor_class in profile["motor_classes"].items():
        entry = ledger[motor_class["ledger_example_entry"]]
        if "rotor_inertia_implied" in motor_class:
            expected = motor_class["armature"]["value"] / entry[GEAR_RATIO_FIELD]["value"] ** 2
            assert motor_class["rotor_inertia_implied"]["value"] == pytest.approx(expected, rel=1e-3), name
    for name in controlled:
        assert profile["joints"][name]["effort_limit"]["value"] == pytest.approx(ledger[name]["peak_torque"]["value"]), name
