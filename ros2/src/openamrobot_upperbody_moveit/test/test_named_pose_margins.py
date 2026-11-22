# SPDX-License-Identifier: MIT
"""Issue #17: every named pose keeps its joints at least MIN_POSE_MARGIN_RAD (0.05 rad) inside both URDF limits.

The test lists every named pose and joint with its margin (printed, smallest first, and in the assertion message) and fails for any revolute
joint below 0.05 rad, at both base plates. The rule is in rad: lift_joint's states are the calibration stops L1000 / L1175 / L1350 and two of
them ARE the ends of the stroke (register ARM-MOUNT, CONFIG-IDS), so the lift is listed with its margin in m and is not held to it.

EXEMPT names the named-pose joints that stay ON a limit by owner decision (Alex Reznichenko, 1 Nov 2026), each with its reason and ADR:
upstream `home` (joint 4 = 0, both arms; ADR-007, the imported pose equals upstream's) and the gripper `open` / `closed` states (both hands;
ADR-009 point 5, M05). Any other revolute pose and joint below 0.05 rad fails, and so does a stale exemption
(`test_the_exemptions_are_exactly_the_joints_below_the_margin`)."""
import pytest

from conftest import BASE_PLATES, srdf_text, urdf_text
from openamrobot_upperbody_moveit.named_poses import MIN_POSE_MARGIN_RAD, below_margin, pose_margins, read_group_states, urdf_joint_limits, urdf_joint_types

OWNER = "owner decision, Alex Reznichenko, 1 Nov 2026"
UPSTREAM_HOME = f"ADR-007: upstream's `home`, imported verbatim (test_srdf_import.py compares it with openarm_ros2 @ the pin): joint4 at its lower limit 0 ({OWNER})"
M05_OPEN = f"ADR-009 point 5 / M05 (test_gripper_sign.py): `open` is each hand's own far limit ({OWNER}; names corrected 14 Nov 2026)"
M05_CLOSED = f"ADR-009 point 5 / M05 (test_gripper_sign.py): `closed` is the end both hands' finger ranges share, 0 ({OWNER}; names corrected 14 Nov 2026)"
EXEMPT = {
    ("left_arm", "home", "openarm_left_joint4"): UPSTREAM_HOME,
    ("right_arm", "home", "openarm_right_joint4"): UPSTREAM_HOME,
    ("left_gripper", "open", "openarm_left_finger_joint1"): M05_OPEN,
    ("right_gripper", "open", "openarm_right_finger_joint1"): M05_OPEN,
    ("left_gripper", "closed", "openarm_left_finger_joint1"): M05_CLOSED,
    ("right_gripper", "closed", "openarm_right_finger_joint1"): M05_CLOSED,
}


def margins(base_plate="bp000"):
    text = urdf_text(base_plate)
    return pose_margins(read_group_states(srdf_text()), urdf_joint_limits(text), urdf_joint_types(text))


def table(rows):
    return "\n".join(f"  {r['group']:<13} {r['pose']:<9} {r['joint']:<28} {r['value']:+.4f} {r['unit']:<3} margin "
                     + (f"{r['margin']:+.4f} {r['unit']}" if r["margin"] is not None else "NO LIMIT")
                     + (f"   exempt: {EXEMPT[(r['group'], r['pose'], r['joint'])]}"
                        if (r["group"], r["pose"], r["joint"]) in EXEMPT else "") for r in rows)


@pytest.mark.parametrize("base_plate", BASE_PLATES)
def test_every_named_pose_joint_is_at_least_0_05_rad_inside_both_limits(base_plate):
    rows = margins(base_plate)
    print(f"\nnamed-pose margins at {base_plate}, smallest first (rule: >= {MIN_POSE_MARGIN_RAD} rad for revolute joints):\n{table(rows)}")
    failing = [r for r in below_margin(rows) if (r["group"], r["pose"], r["joint"]) not in EXEMPT]
    assert not failing, f"named-pose joints closer than {MIN_POSE_MARGIN_RAD} rad to a limit:\n{table(failing)}\n\nall:\n{table(rows)}"
    gated = [r for r in rows if r["unit"] == "rad" and (r["group"], r["pose"], r["joint"]) not in EXEMPT]
    assert gated and min(r["margin"] for r in gated) >= MIN_POSE_MARGIN_RAD


def test_the_exemptions_are_exactly_the_joints_below_the_margin():
    """No stale exemption (an exempt joint moved off its limit must leave the list) and no silent new one."""
    below = {(r["group"], r["pose"], r["joint"]) for r in below_margin(margins())}
    assert below == set(EXEMPT), {"below but not listed": sorted(below - set(EXEMPT)),
                                           "listed but not below": sorted(set(EXEMPT) - below)}


def test_the_smallest_margin_over_the_non_exempt_poses():
    rows = [r for r in margins() if r["unit"] == "rad" and (r["group"], r["pose"], r["joint"]) not in EXEMPT]
    smallest = min(rows, key=lambda r: r["margin"])
    print(f"\nsmallest non-exempt margin: {smallest['margin']:.4f} rad ({smallest['group']}/{smallest['pose']} {smallest['joint']})")
    assert smallest["margin"] >= MIN_POSE_MARGIN_RAD


def test_park_both_arms_park_and_hands_up_meet_the_margin():
    """The poses issue #17 names: `park` sat exactly on joint4's limit; it and the other arm poses this repository owns now meet the rule."""
    rows = [r for r in margins() if r["group"] in ("left_arm", "right_arm", "both_arms") and r["pose"] in ("park", "hands_up")]
    assert rows and all(r["margin"] >= MIN_POSE_MARGIN_RAD for r in rows), table(rows)
    assert {r["pose"] for r in rows if r["group"] == "both_arms"} == {"park"}


def test_the_lift_stops_are_listed_in_m_and_inside_the_stroke():
    rows = [r for r in margins() if r["joint"] == "lift_joint"]
    assert {r["pose"] for r in rows} == {"L1000", "L1175", "L1350"}
    assert all(r["unit"] == "m" and r["type"] == "prismatic" and r["margin"] >= 0.0 for r in rows), table(rows)
    assert not [r for r in below_margin(rows)], "the rad rule is not applied to the prismatic lift"


SRDF = """<robot name="x">
  <group_state name="near" group="a"><joint name="elbow" value="0.049"/></group_state>
  <group_state name="ok" group="a"><joint name="elbow" value="0.05"/></group_state>
  <group_state name="past" group="a"><joint name="elbow" value="-0.01"/></group_state>
  <group_state name="odd" group="a"><joint name="mystery" value="0.3"/></group_state>
  <group_state name="top" group="s"><joint name="slide" value="0.0"/></group_state>
</robot>"""
URDF = """<robot name="x">
  <joint name="elbow" type="revolute"><limit lower="0" upper="2.4"/></joint>
  <joint name="mystery" type="revolute"/>
  <joint name="slide" type="prismatic"><limit lower="0" upper="0.35"/></joint>
</robot>"""


def test_the_rule_fails_below_0_05_rad_past_a_limit_and_without_a_limit_and_passes_at_it():
    rows = pose_margins(read_group_states(SRDF), urdf_joint_limits(URDF), urdf_joint_types(URDF))
    assert sorted(r["pose"] for r in below_margin(rows)) == ["near", "odd", "past"]
    assert [r["margin"] for r in rows if r["pose"] == "past"] == [pytest.approx(-0.01)]
    assert [r["unit"] for r in rows if r["pose"] == "top"] == ["m"]
