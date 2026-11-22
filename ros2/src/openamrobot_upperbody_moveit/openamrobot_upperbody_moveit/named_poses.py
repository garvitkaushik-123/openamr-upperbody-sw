# SPDX-License-Identifier: MIT
"""Named poses read from the SRDF, and the per-hand gripper sign rule (ADR-009 point 5).

The two grippers take opposite-signed GripperCommand positions: left finger_joint1 range [0, 0.7854], right [-0.7854, 0] (upstream's
convention, not ours). Zero is the CLOSED hand and the far end of each hand's own range is the OPEN hand: upstream's docs, its driver
(openarm_can: "0=closed, 1=open") and its MuJoCo model say so, and in the expanded URDF the fingertips are 0.1 mm apart at 0 and 139 mm
apart at the far end. Until 14 Nov 2026 this module and the SRDF had the two names the other way round.

A named pose or helper that opens both hands from one number is wrong on one hand WITHOUT failing at the point it is written, which
is the class of defect that reaches a demo. So the open value of a hand is never a constant here: it is that hand's own far limit,
read from the URDF, and `gripper_sign_problems` is the check that fails the single-number version.
"""
import xml.etree.ElementTree as ET

SIDES = ("left", "right")
GRIPPER_JOINT = {side: f"openarm_{side}_finger_joint1" for side in SIDES}
GRIPPER_GROUP = {side: f"{side}_gripper" for side in SIDES}
ARM_GROUP = {side: f"{side}_arm" for side in SIDES}
ARM_POSES = ("home", "hands_up", "park")       # per arm; `park` is a placeholder copy of `home`
GRIPPER_POSES = ("open", "closed")             # per hand, explicit values
CLOSED_VALUE = 0.0                             # rad: the end both hands' ranges share; the gripper motor's zero is the closed hand


def read_group_states(srdf_text):
    """{(group, state name): {joint: value}} from an SRDF."""
    root = ET.fromstring(srdf_text)
    return {(gs.get("group"), gs.get("name")): {j.get("name"): float(j.get("value")) for j in gs.findall("joint")}
            for gs in root.findall("group_state")}


def urdf_joint_limits(urdf_text):
    """{joint: (lower, upper)} for the revolute joints of a URDF."""
    root = ET.fromstring(urdf_text)
    out = {}
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if limit is not None and limit.get("lower") is not None:
            out[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))
    return out


def far_limit(side, limits):
    """The end of a hand's own finger range away from zero: where `open` is. Left +0.7854, right -0.7854, from the URDF."""
    lower, upper = limits[GRIPPER_JOINT[side]]
    return upper if abs(upper) >= abs(lower) else lower


def open_value(side, limits):
    """The `open` position for one hand. Per hand and explicit: there is no function of a single number for both."""
    return far_limit(side, limits)


def closed_value(side, limits):
    """The `closed` position for one hand: 0, the end of its range that is not the far limit. Read from the URDF, so a range that
    stops containing 0 fails here and not on the robot."""
    lower, upper = limits[GRIPPER_JOINT[side]]
    near = lower if abs(lower) < abs(upper) else upper
    if near != CLOSED_VALUE:
        raise ValueError(f"{GRIPPER_JOINT[side]}: the near end of [{lower}, {upper}] is {near}, not {CLOSED_VALUE}: the closed hand is not at zero")
    return near


def gripper_sign_problems(states, limits, tolerance=1e-4):
    """Messages for every way the gripper group states break ADR-009 point 5; empty when they are right. States are
    {(group, name): {joint: value}} (read_group_states), limits are {joint: (lower, upper)} (urdf_joint_limits).

    Checks, per hand: `open` and `closed` exist; each value is inside that hand's own URDF range (a left-hand number on the right
    hand is outside it: that is the single-number defect); `open` is that hand's far limit, not merely something in range; `closed`
    is 0, the end both ranges share (the motor's zero is the closed hand); and across hands `open` has opposite signs, because the
    ranges do. States with the two names swapped (the defect fixed on 14 Nov 2026) fail on both counts."""
    problems = []
    opened = {}
    for side in SIDES:
        joint, group = GRIPPER_JOINT[side], GRIPPER_GROUP[side]
        lower, upper = limits[joint]
        for name in GRIPPER_POSES:
            values = states.get((group, name))
            if values is None:
                problems.append(f"{group}: no group_state `{name}`")
                continue
            if set(values) != {joint}:
                problems.append(f"{group}/{name}: sets {sorted(values)}, expected only {joint}")
                continue
            value = values[joint]
            if not (lower - tolerance <= value <= upper + tolerance):
                problems.append(f"{group}/{name}: {value:+.4f} is outside {side}'s own range [{lower:+.4f}, {upper:+.4f}] of {joint}")
            if name == "open":
                opened[side] = value
                if abs(value - far_limit(side, limits)) > tolerance:
                    problems.append(f"{group}/open: {value:+.4f} is not {side}'s far limit {far_limit(side, limits):+.4f}")
            if name == "closed" and abs(value - CLOSED_VALUE) > tolerance:
                problems.append(f"{group}/closed: {value:+.4f} is not {CLOSED_VALUE:+.4f}, where the fingertips meet")
    if len(opened) == 2 and opened["left"] * opened["right"] >= 0:
        problems.append(f"open values {opened['left']:+.4f} (left) and {opened['right']:+.4f} (right) do not have opposite signs: "
                        "one hand's `open` is outside the other's range")
    return problems


# --- named poses that coincide with a joint limit (P18's W03; evidence/w03/, not a registered check since P19) -------------------------
# Information only, gating nothing. A joint commanded to a value that IS one of its limits is a fact about the pose definitions (upstream's `home` is
# joint 4 at its lower limit 0, a gripper's `closed` and `open` are the two ends of its range), not a defect of any body: it is true of the old mast body and of
# the lift body alike, and the report is for whichever SRDF and URDF it is given. It is not M07, which measures a simulated rest position
# against a margin datum; it registers nothing while M07 exists.
ON_LIMIT_TOLERANCE = 1e-6      # rad. NOT a margin: the numeric slack within which two numbers written in files are the same number


def poses_on_limits(states, limits, tolerance=ON_LIMIT_TOLERANCE):
    """Every (group, named pose, joint) whose value equals one of the joint's URDF limits to within `tolerance`, with the distance.
    `states` is read_group_states(srdf), `limits` is urdf_joint_limits(urdf). A value past a limit has a negative distance and is listed
    too. Joints with no URDF limit are not in `limits` and are listed under the second return value, never silently skipped."""
    rows, unlimited = [], []
    for (group, pose), joints in sorted(states.items()):
        for joint, value in sorted(joints.items()):
            if joint not in limits:
                unlimited.append({"group": group, "pose": pose, "joint": joint})
                continue
            lower, upper = limits[joint]
            to_lower, to_upper = value - lower, upper - value
            distance, which = (to_lower, "lower") if to_lower <= to_upper else (to_upper, "upper")
            if abs(distance) <= tolerance:
                rows.append({"group": group, "pose": pose, "joint": joint, "value_rad": value, "limit": which,
                             "limit_rad": lower if which == "lower" else upper, "distance_rad": distance})
    return rows, unlimited


def on_limit_report(srdf_text, urdf_text, tolerance=ON_LIMIT_TOLERANCE):
    rows, unlimited = poses_on_limits(read_group_states(srdf_text), urdf_joint_limits(urdf_text), tolerance)
    return {"tolerance_rad": tolerance, "poses_on_a_limit": rows, "joints_on_a_limit": sorted({r["joint"] for r in rows}),
            "named_pose_joints_without_a_urdf_limit": unlimited}


# --- the margin of every named pose from its joints' limits (issue #17) ------------------------------------------------------------------
# Every joint of a named pose is to sit at least MIN_POSE_MARGIN_RAD inside both of its URDF limits. The rule is in rad and is for the revolute
# joints; a prismatic joint (lift_joint, whose named states are the calibration stops and include both ends of the stroke) is listed with its
# margin in m and is not held to it. test_named_pose_margins.py lists every pose and joint with its margin and fails below the rule.
MIN_POSE_MARGIN_RAD = 0.05


def urdf_joint_types(urdf_text):
    """{joint: URDF type} for every joint of a URDF."""
    return {j.get("name"): j.get("type") for j in ET.fromstring(urdf_text).findall("joint")}


def pose_margins(states, limits, types):
    """[{group, pose, joint, type, unit, value, lower, upper, to_lower, to_upper, margin}] for every joint of every named pose, sorted by margin
    (smallest first). `margin` is the smaller of the two distances to a limit; negative means past it. A joint with no URDF limit gets margin
    None and is listed, never skipped. `states` is read_group_states, `limits` urdf_joint_limits, `types` urdf_joint_types."""
    rows = []
    for (group, pose), joints in sorted(states.items()):
        for joint, value in sorted(joints.items()):
            kind = types.get(joint)
            row = {"group": group, "pose": pose, "joint": joint, "type": kind, "unit": "m" if kind == "prismatic" else "rad", "value": value}
            if joint in limits:
                lower, upper = limits[joint]
                row.update(lower=lower, upper=upper, to_lower=value - lower, to_upper=upper - value, margin=min(value - lower, upper - value))
            else:
                row.update(lower=None, upper=None, to_lower=None, to_upper=None, margin=None)
            rows.append(row)
    return sorted(rows, key=lambda r: (r["margin"] is None, r["margin"] if r["margin"] is not None else 0.0, r["group"], r["pose"], r["joint"]))


def below_margin(rows, minimum=MIN_POSE_MARGIN_RAD):
    """The rows of revolute (rad) joints closer than `minimum` to a limit, or past it, or with no limit at all."""
    return [r for r in rows if r["unit"] == "rad" and (r["margin"] is None or r["margin"] < minimum)]


if __name__ == "__main__":
    import argparse
    import json
    parser = argparse.ArgumentParser(description="named poses of an SRDF that coincide with a joint limit of a URDF")
    parser.add_argument("srdf")
    parser.add_argument("urdf")
    args = parser.parse_args()
    with open(args.srdf) as f_srdf, open(args.urdf) as f_urdf:
        print(json.dumps(on_limit_report(f_srdf.read(), f_urdf.read()), indent=1))
