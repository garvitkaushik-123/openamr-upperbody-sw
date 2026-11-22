# SPDX-License-Identifier: MIT
"""Named poses of the SRDF that put a joint exactly on one of its URDF limits, and how far (P18's W03 helper; P19 made it plain evidence, not a check).

`python -m openamrobot_upperbody_moveit.named_poses <srdf> <urdf>` prints the report; evidence/w03/ holds it for the old mast body (7daa2ab) and
the lift body. It is NOT an entry of the A7 report and writes no sidecar: while M07 exists W03 is not registered as a check. The helper is a
function of the SRDF and URDF it is given, with no knowledge of either body, which `*_any_body` pins on a synthetic second body."""
from conftest import srdf_text, urdf_text
from openamrobot_upperbody_moveit.named_poses import ON_LIMIT_TOLERANCE, on_limit_report, poses_on_limits, read_group_states, urdf_joint_limits

# Upstream's `home` (joint4 = 0) and the gripper states are on a limit by construction; `park` no longer is (issue #17). lift_joint is listed
# because its states L1000 and L1350 are the two ends of the stroke by definition (calibration stops, in m).
ON_A_LIMIT = ["lift_joint", "openarm_left_finger_joint1", "openarm_left_joint4", "openarm_right_finger_joint1", "openarm_right_joint4"]


def test_named_poses_on_a_joint_limit_are_listed_with_their_distance():
    report = on_limit_report(srdf_text(), urdf_text())
    rows = report["poses_on_a_limit"]
    assert report["joints_on_a_limit"] == ON_A_LIMIT, report["joints_on_a_limit"]
    assert all(abs(r["distance_rad"]) <= ON_LIMIT_TOLERANCE for r in rows)
    by = {(r["group"], r["pose"], r["joint"]): r for r in rows}
    assert by[("left_arm", "home", "openarm_left_joint4")]["limit"] == "lower"
    assert ("left_arm", "park", "openarm_left_joint4") not in by and ("right_arm", "park", "openarm_right_joint4") not in by
    assert by[("lift", "L1000", "lift_joint")]["limit"] == "lower" and by[("lift", "L1350", "lift_joint")]["limit"] == "upper"
    assert by[("right_gripper", "closed", "openarm_right_finger_joint1")]["limit"] == "upper"
    assert by[("left_gripper", "closed", "openarm_left_finger_joint1")]["limit"] == "lower"
    assert by[("left_gripper", "open", "openarm_left_finger_joint1")]["limit"] == "upper"
    assert by[("right_gripper", "open", "openarm_right_finger_joint1")]["limit"] == "lower"
    assert report["named_pose_joints_without_a_urdf_limit"] == []


SRDF_OTHER_BODY = """<robot name="x">
  <group_state name="home" group="a"><joint name="elbow" value="0"/><joint name="shoulder" value="0.5"/></group_state>
  <group_state name="open" group="g"><joint name="finger" value="0.04"/></group_state>
  <group_state name="odd" group="a"><joint name="mystery" value="1"/></group_state>
</robot>"""
URDF_OTHER_BODY = """<robot name="x">
  <joint name="elbow" type="revolute"><limit lower="0" upper="2.4"/></joint>
  <joint name="shoulder" type="revolute"><limit lower="-1" upper="1"/></joint>
  <joint name="finger" type="prismatic"><limit lower="0" upper="0.04"/></joint>
</robot>"""


def test_lists_the_same_kind_of_thing_for_any_body_and_does_not_invent_or_skip():
    """A second, different body: a pose on a lower limit, one on an upper limit, one comfortably inside, and one named joint with no URDF limit,
    which is reported as such and not dropped."""
    report = on_limit_report(SRDF_OTHER_BODY, URDF_OTHER_BODY)
    assert report["joints_on_a_limit"] == ["elbow", "finger"]
    assert {r["joint"]: r["limit"] for r in report["poses_on_a_limit"]} == {"elbow": "lower", "finger": "upper"}
    assert [u["joint"] for u in report["named_pose_joints_without_a_urdf_limit"]] == ["mystery"]


def test_a_value_past_a_limit_is_listed_with_a_negative_distance_and_one_just_inside_is_not():
    states = {("g", "past"): {"j": -0.0000005}, ("g", "near"): {"j": 0.0001}}
    rows, _ = poses_on_limits(states, {"j": (0.0, 1.0)})
    assert [(r["pose"], r["distance_rad"] < 0) for r in rows] == [("past", True)]
    # 1e-4 rad inside is not "on" the limit: the tolerance is numeric equality of two written numbers, not a margin
    assert ON_LIMIT_TOLERANCE <= 1e-6


def test_it_is_not_a_property_of_the_body_the_real_srdf_and_a_second_urdf_agree():
    """The same SRDF against the real URDF and against a URDF whose joint4 limits differ: the report follows the limits, not the body."""
    states, limits = read_group_states(srdf_text()), dict(urdf_joint_limits(urdf_text()))
    moved = dict(limits, openarm_left_joint4=(-0.5, 2.4435))
    rows, _ = poses_on_limits(states, moved)
    assert "openarm_left_joint4" not in {r["joint"] for r in rows}
    assert "openarm_right_joint4" in {r["joint"] for r in rows}
