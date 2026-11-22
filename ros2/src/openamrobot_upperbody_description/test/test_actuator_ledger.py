# SPDX-License-Identifier: MIT
"""Actuator ledger: schema, provenance, and agreement with the combined URDF joint limits.

config/actuator_ledger.yaml is data only (no generator reads it yet). These checks keep it honest:
  - it validates against config/actuator_ledger.schema.json;
  - every non-null value has a source, a known status and an owner; every null value says why;
  - where the ledger and the expanded combined URDF both carry an arm or lift limit, they are equal.
"""
import math
import os

import pytest
from jsonschema import Draft202012Validator

from conftest import ARG_DEFAULTS, SHARE, expand, load_yaml

LEDGER_FILE = os.path.join(SHARE, "config", "actuator_ledger.yaml")
LEDGER_SCHEMA = os.path.join(SHARE, "config", "actuator_ledger.schema.json")
KNOWN = {
    "datasheet",
    "supplier_confirmation",
    "upstream_config",
    "upstream_urdf",
    "cad",
    "measured",
    "estimated",
    "proposed",
    "decision",
}
EMPTY = {"unknown", "not_applicable"}

# ledger quantity -> URDF <limit> attribute
ARM_LIMITS = {
    "joint_lower_limit": "lower",
    "joint_upper_limit": "upper",
    "urdf_effort_limit": "effort",
    "urdf_velocity_limit": "velocity",
}
LIFT_LIMITS = {"peak_force": "effort", "max_speed": "velocity"}


def _ledger():
    return load_yaml(LEDGER_FILE)


def _schema():
    import json

    with open(LEDGER_SCHEMA, encoding="utf-8") as f:
        return json.load(f)


def _quantities(entry):
    for key, val in entry.items():
        if isinstance(val, dict) and "status" in val:
            yield key, val
    for key, val in (entry.get("extra") or {}).items():
        yield f"extra.{key}", val


def _urdf_limits():
    root = expand(**ARG_DEFAULTS)
    out = {}
    for j in root.findall("joint"):
        lim = j.find("limit")
        if lim is not None:
            out[j.get("name")] = {k: float(v) for k, v in lim.attrib.items()}
    return out


def test_ledger_schema_is_valid_2020_12():
    schema = _schema()
    assert schema["$schema"] == "https://json-schema.org/draft/2020-12/schema"
    Draft202012Validator.check_schema(schema)


def test_ledger_validates_against_schema():
    errors = list(Draft202012Validator(_schema()).iter_errors(_ledger()))
    assert not errors, "\n".join(f"{'/'.join(map(str, e.absolute_path))}: {e.message}" for e in errors[:20])


def test_ledger_ids_unique_and_urdf_joints_unique():
    acts = _ledger()["actuators"]
    ids = [a["id"] for a in acts]
    assert len(ids) == len(set(ids)), "duplicate actuator id"
    joints = [a["urdf_joint"] for a in acts if a["urdf_joint"]]
    assert len(joints) == len(set(joints)), "two entries claim the same URDF joint"


def test_every_value_has_provenance():
    bad = []
    for a in _ledger()["actuators"]:
        for key, q in _quantities(a):
            where = f"{a['id']}.{key}"
            if not q.get("owner"):
                bad.append(f"{where}: no owner")
            if q["value"] is None:
                if q["status"] not in EMPTY:
                    bad.append(f"{where}: null value with status {q['status']}")
                if not q.get("note"):
                    bad.append(f"{where}: null value without a note saying why")
            else:
                if q["status"] not in KNOWN:
                    bad.append(f"{where}: value with status {q['status']}")
                if not (isinstance(q.get("source"), str) and q["source"].strip()):
                    bad.append(f"{where}: value without a source")
                if isinstance(q["value"], (int, float)) and not isinstance(q["value"], bool):
                    if not math.isfinite(q["value"]):
                        bad.append(f"{where}: non-finite number")
                    if q.get("unit") is None:
                        bad.append(f"{where}: number without a unit")
    assert not bad, "\n".join(bad)


def test_schema_rejects_value_without_source():
    doc = _ledger()
    doc["actuators"][0]["rated_torque"] = {"value": 1.0, "unit": "N*m", "source": None,
                                           "status": "estimated", "owner": "x"}
    assert list(Draft202012Validator(_schema()).iter_errors(doc))


def test_schema_rejects_null_with_datasheet_status():
    doc = _ledger()
    doc["actuators"][0]["rated_torque"] = {"value": None, "unit": "N*m", "source": "x",
                                           "status": "datasheet", "owner": "x"}
    assert list(Draft202012Validator(_schema()).iter_errors(doc))


@pytest.fixture(scope="module")
def urdf_limits():
    return _urdf_limits()


def test_every_combined_model_arm_joint_has_a_ledger_entry(urdf_limits):
    covered = {a["urdf_joint"] for a in _ledger()["actuators"]}
    # finger_joint2 mimics finger_joint1 and has no motor of its own
    actuated = {n for n in urdf_limits if n.startswith("openarm_") and not n.endswith("finger_joint2")}
    assert actuated, "combined URDF has no OpenArm joints with limits"
    assert actuated <= covered, f"missing ledger entries: {sorted(actuated - covered)}"


def test_ledger_limits_equal_combined_urdf(urdf_limits):
    checked, bad = 0, []
    for a in _ledger()["actuators"]:
        joint = a["urdf_joint"]
        if joint not in urdf_limits:
            continue
        if a["kind"] in ("joint_motor", "gripper_motor"):
            pairs = [((a.get("extra") or {}).get(k), attr) for k, attr in ARM_LIMITS.items()]
        elif a["kind"] == "linear_actuator":
            pairs = [(a.get(k), attr) for k, attr in LIFT_LIMITS.items()]
        else:
            continue
        for q, attr in pairs:
            if q is None or q["value"] is None or attr not in urdf_limits[joint]:
                continue
            checked += 1
            if not math.isclose(float(q["value"]), urdf_limits[joint][attr], rel_tol=0, abs_tol=1e-9):
                bad.append(f"{joint}.{attr}: ledger {q['value']} != URDF {urdf_limits[joint][attr]}")
    assert checked > 0, "no limits compared"
    assert not bad, "\n".join(bad)


def test_left_and_right_arm_motor_data_agree():
    """Motor-class data must be identical on both arms; only side-specific fields may differ."""
    data = load_yaml(LEDGER_FILE)
    by_id = {a["id"]: a for a in data["actuators"]}
    compared = 0
    for left_id, left in by_id.items():
        if not left_id.startswith("openarm_left_"):
            continue
        right = by_id[left_id.replace("openarm_left_", "openarm_right_")]
        for key in ("part", "gear_ratio", "rated_torque", "peak_torque", "rated_speed", "max_speed",
                    "rated_current", "peak_current", "rated_voltage"):
            assert left[key]["value"] == right[key]["value"], f"{left_id} vs right: {key}"
            assert left[key]["status"] == right[key]["status"], f"{left_id} vs right: {key} status"
            compared += 1
    assert compared > 0
