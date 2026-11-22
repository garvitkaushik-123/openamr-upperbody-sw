# SPDX-License-Identifier: MIT
"""M05 (adopted 7 Oct, VERIFICATION.md; names corrected 14 Nov 2026): the two grippers take opposite-signed GripperCommand positions, no
named pose may open both hands from one number (ADR-009 point 5), and `closed` is where the fingertips meet.

Two silent defects are guarded.

1. One number for both hands. Left finger_joint1 ranges [0, 0.7854] and right [-0.7854, 0], so "open" written as +0.7854 for both hands is
   a valid number for one hand and out of range for the other, and nothing fails where it is written. The check reads the SRDF's gripper
   states and the URDF's own limits, per hand, and the two `single_number` tests prove it can fail.
2. The names the wrong way round. Until 14 Nov 2026 the SRDF had `open` = 0 and `closed` = the far limit, and this check asserted exactly
   that, so it passed while a `closed` command opened the hand. Zero is the closed hand: upstream's docs ("the closed position is used as
   zero by default"), its driver (openarm_can: "0=closed, 1=open") and its MuJoCo model say so. `test_m05_the_geometry_*` now measures
   it on the expanded URDF, so the names are tied to where the fingers are and not to a convention someone wrote down."""
import copy
import math
import os
import re
import struct
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from conftest import srdf_text, urdf_text, write_sidecar
from openamrobot_upperbody_moveit.named_poses import (CLOSED_VALUE, GRIPPER_GROUP, GRIPPER_JOINT, SIDES, closed_value, gripper_sign_problems,
                                                      open_value, read_group_states, urdf_joint_limits)


def real():
    return read_group_states(srdf_text()), urdf_joint_limits(urdf_text())


# --- where the fingers are, from the URDF: no MoveIt, no simulator ---------------------------------------------------------------------

def _rotation(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr), (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def _about(axis, angle):
    n = math.sqrt(sum(a * a for a in axis))
    x, y, z = (a / n for a in axis)
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return ((t * x * x + c, t * x * y - s * z, t * x * z + s * y), (t * x * y + s * z, t * y * y + c, t * y * z - s * x),
            (t * x * z - s * y, t * y * z + s * x, t * z * z + c))


def _apply(rotation, translation, point):
    return tuple(sum(rotation[i][k] * point[k] for k in range(3)) + translation[i] for i in range(3))


def _floats(text, default):
    return tuple(float(v) for v in text.split()) if text else default


def _stl_vertices(path):
    with open(path, "rb") as f:
        data = f.read()
    count = struct.unpack("<I", data[80:84])[0]
    assert len(data) == 84 + 50 * count, f"{path} is not a binary STL"
    return [struct.unpack_from("<3f", data, 84 + 50 * i + 12 + 12 * k) for i in range(count) for k in range(3)]


def _mesh_path(uri):
    package, relative = re.fullmatch(r"package://([^/]+)/(.*)", uri).groups()
    return os.path.join(get_package_share_directory(package), relative)


def fingertip_gap(urdf, side, angle):
    """Smallest distance (m) between the distal thirds of the two finger collision meshes of one hand, in the hand's base frame, with
    finger_joint1 at `angle` and finger_joint2 following its <mimic>."""
    root = ET.fromstring(urdf)
    tips = []
    for index in (1, 2):
        joint = root.find(f"joint[@name='openarm_{side}_finger_joint{index}']")
        mimic = joint.find("mimic")
        q = angle if mimic is None else angle * float(mimic.get("multiplier", "1")) + float(mimic.get("offset", "0"))
        origin = joint.find("origin")
        joint_rotation, joint_xyz = _rotation(*_floats(origin.get("rpy"), (0, 0, 0))), _floats(origin.get("xyz"), (0, 0, 0))
        turn = _about(_floats(joint.find("axis").get("xyz"), (1, 0, 0)), q)
        points = []
        for collision in root.find(f"link[@name='{joint.find('child').get('link')}']").findall("collision"):
            mesh = collision.find("geometry/mesh")
            c_origin = collision.find("origin")
            c_rotation = _rotation(*_floats(c_origin.get("rpy") if c_origin is not None else None, (0, 0, 0)))
            c_xyz = _floats(c_origin.get("xyz") if c_origin is not None else None, (0, 0, 0))
            scale = _floats(mesh.get("scale"), (1, 1, 1))
            for vertex in _stl_vertices(_mesh_path(mesh.get("filename")))[::5]:
                in_link = _apply(c_rotation, c_xyz, tuple(v * s for v, s in zip(vertex, scale)))
                points.append(_apply(joint_rotation, joint_xyz, _apply(turn, (0, 0, 0), in_link)))
        assert points, f"openarm_{side}_finger_joint{index}: its link has no collision mesh"
        cut = sorted(p[2] for p in points)[len(points) // 3]        # the fingers point along -z of the hand base: the lowest third is the tip
        tips.append([p for p in points if p[2] <= cut])
    return min(math.dist(a, b) for a in tips[0] for b in tips[1][::3])


def named_state_gaps(states, urdf):
    """{side: {state name: fingertip gap in m}} for the SRDF's gripper states."""
    return {side: {name: round(fingertip_gap(urdf, side, states[(GRIPPER_GROUP[side], name)][GRIPPER_JOINT[side]]), 5)
                   for name in ("open", "closed")} for side in SIDES}


def test_m05_the_geometry_says_zero_is_closed_and_the_far_limit_is_open():
    """The evidence for the names: at `closed` the fingertips are within a millimetre, at `open` they are more than 100 mm apart."""
    states, _ = real()
    gaps = named_state_gaps(states, urdf_text())
    for side in SIDES:
        assert gaps[side]["closed"] < 0.001, f"{side}: `closed` leaves the fingertips {gaps[side]['closed'] * 1000:.1f} mm apart"
        assert gaps[side]["open"] > 0.100, f"{side}: `open` leaves the fingertips only {gaps[side]['open'] * 1000:.1f} mm apart"


def test_m05_the_srdf_gripper_states_are_per_hand_and_opposite_signed():
    states, limits = real()
    problems = gripper_sign_problems(states, limits)
    closed = {side: states[(GRIPPER_GROUP[side], "closed")][GRIPPER_JOINT[side]] for side in SIDES}
    opened = {side: states[(GRIPPER_GROUP[side], "open")][GRIPPER_JOINT[side]] for side in SIDES}
    write_sidecar("M05", "gripper_sign", "per-hand gripper values: open is each hand's own far limit, opposite signs; closed is 0 (ADR-009 point 5)",
                  "FAIL" if problems else "PASS",
                  {"problems": problems, "closed": closed, "open": opened,
                   "urdf_ranges": {side: list(limits[GRIPPER_JOINT[side]]) for side in SIDES},
                   "single_number_version_fails": True, "swapped_names_fail": True,
                   "fingertip_gap_m": named_state_gaps(states, urdf_text()),
                   "fingertip_gap_method": "smallest distance between the distal thirds of the two finger collision meshes, expanded URDF"})
    assert not problems, problems
    assert opened["left"] > 0 > opened["right"] and closed == {"left": 0.0, "right": 0.0}


def test_m05_the_helpers_derive_open_from_each_hands_own_limit_and_closed_is_zero():
    _, limits = real()
    assert open_value("left", limits) == 0.7854 and open_value("right", limits) == -0.7854
    assert closed_value("left", limits) == CLOSED_VALUE == closed_value("right", limits) == 0.0


def test_m05_the_swapped_names_fail():
    """The SRDF as it was until 14 Nov 2026: `open` 0 and `closed` at the far limit. The old check passed it; this one must not."""
    states, limits = real()
    swapped = copy.deepcopy(states)
    for side in SIDES:
        group = GRIPPER_GROUP[side]
        swapped[(group, "open")], swapped[(group, "closed")] = states[(group, "closed")], states[(group, "open")]
    problems = gripper_sign_problems(swapped, limits)
    for side in SIDES:
        assert any(p.startswith(f"{side}_gripper/open") and "far limit" in p for p in problems), problems
        assert any(p.startswith(f"{side}_gripper/closed") and "where the fingertips meet" in p for p in problems), problems
    assert any("opposite signs" in p for p in problems), problems


def test_m05_the_single_number_version_fails():
    """Both hands opened with the left hand's number: valid for left, outside right's range. The check must say so, by hand."""
    states, limits = real()
    broken = copy.deepcopy(states)
    broken[("right_gripper", "open")] = {GRIPPER_JOINT["right"]: states[("left_gripper", "open")][GRIPPER_JOINT["left"]]}
    problems = gripper_sign_problems(broken, limits)
    assert any("right_gripper/open" in p and "outside right's own range" in p for p in problems), problems
    assert any("opposite signs" in p for p in problems), problems
    assert not any(p.startswith("left_gripper") for p in problems), "left is still right"


def test_m05_the_other_single_number_version_fails_too():
    """The mirror image: both hands opened with the right hand's (negative) number."""
    states, limits = real()
    broken = copy.deepcopy(states)
    broken[("left_gripper", "open")] = {GRIPPER_JOINT["left"]: states[("right_gripper", "open")][GRIPPER_JOINT["right"]]}
    problems = gripper_sign_problems(broken, limits)
    assert any("left_gripper/open" in p and "outside left's own range" in p for p in problems), problems


def test_m05_a_value_in_range_but_not_the_far_limit_is_not_an_open_pose():
    states, limits = real()
    short = copy.deepcopy(states)
    short[("left_gripper", "open")] = {GRIPPER_JOINT["left"]: 0.4}
    assert any("not left's far limit" in p for p in gripper_sign_problems(short, limits))


def test_m05_a_closed_pose_that_is_not_zero_is_reported():
    states, limits = real()
    ajar = copy.deepcopy(states)
    ajar[("right_gripper", "closed")] = {GRIPPER_JOINT["right"]: -0.1}
    assert any(p.startswith("right_gripper/closed") and "where the fingertips meet" in p for p in gripper_sign_problems(ajar, limits))


def test_m05_a_missing_or_extra_state_is_reported():
    states, limits = real()
    missing = {k: v for k, v in states.items() if k != ("right_gripper", "open")}
    assert any("right_gripper: no group_state `open`" in p for p in gripper_sign_problems(missing, limits))
    extra = copy.deepcopy(states)
    extra[("left_gripper", "open")] = {GRIPPER_JOINT["left"]: 0.7854, "openarm_left_finger_joint2": 0.7854}
    assert any("expected only" in p for p in gripper_sign_problems(extra, limits))
