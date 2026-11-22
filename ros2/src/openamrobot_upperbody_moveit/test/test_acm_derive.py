# SPDX-License-Identifier: MIT
"""The collision matrix, derived: a pair is disabled only for a reason that can be shown (acm_derive.py), and every reason is recomputed here from the
model. No ROS runtime and no simulator: the xacro Python API, the SRDF as a file, and a small forward-kinematics of the URDF's own joints.

P12A replaced the inherited/ours split (a premise that was false: upstream declares 20 pairs, all `Adjacent`, and every `Never` pair was written by our own
sampler) with three reasons: declared by upstream, `Adjacent`, `ConstantTransform`. Everything else is enabled."""
import itertools
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from conftest import BASE_PLATES, SRDF_FILE, read_text, srdf_root, urdf_root
from openamrobot_upperbody_moveit import acm_derive as derive_mod
from openamrobot_upperbody_moveit import acm_split as split

BEGIN = "<!-- BEGIN GENERATED ACM -->"
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))


def srdf_pairs():
    return split.acm_pairs(srdf_root())


def upstream_pairs():
    """Section 1 of the SRDF: the pairs upstream declares (diffed against the pinned checkout by test_srdf_import.py)."""
    head = read_text(SRDF_FILE).split(BEGIN)[0] + "</robot>"
    return split.acm_pairs(ET.fromstring(head))


def tree(xml):
    return split.urdf_tree(ET.fromstring(xml))


def test_a_path_of_fixed_joints_has_no_movable_joint_and_one_movable_joint_anywhere_on_the_path_is_enough():
    t = tree("""<robot>
      <joint name="j_a" type="fixed"><parent link="root"/><child link="a"/></joint>
      <joint name="j_b" type="fixed"><parent link="a"/><child link="b"/></joint>
      <joint name="j_c" type="revolute"><parent link="a"/><child link="c"/></joint>
      <joint name="j_d" type="fixed"><parent link="c"/><child link="d"/></joint>
      <joint name="j_m" type="fixed"><parent link="root"/><child link="m"/><mimic joint="j_c"/></joint></robot>""")
    assert split.movable_joints_between(t, "a", "b") == []
    assert split.movable_joints_between(t, "root", "b") == []
    assert split.movable_joints_between(t, "b", "d") == ["j_c"]
    assert split.movable_joints_between(t, "d", "root") == ["j_c"]
    assert split.movable_joints_between(t, "m", "a") == ["j_m"], "a mimic joint moves its link whatever type it is declared"
    with pytest.raises(ValueError):
        split.movable_joints_between(t, "a", "elsewhere")


# --- an independent forward kinematics, to test the reasoning rather than repeat it -----------------------------------------------------
def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, angle):
    axis = np.array(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def forward_kinematics(root, values):
    """{link: 4x4 pose in the root link's frame} for joint values {joint: q} (a missing joint is 0; a mimic follows its leader)."""
    joints = {j.get("name"): j for j in root.findall("joint")}

    def q(name):
        j = joints[name]
        m = j.find("mimic")
        return values.get(name, 0.0) if m is None else values.get(m.get("joint"), 0.0) * float(m.get("multiplier", 1)) + float(m.get("offset", 0))

    poses = {}
    children = {j.find("child").get("link"): j for j in joints.values()}
    parents = {j.find("parent").get("link") for j in joints.values()}
    roots = [p for p in parents if p not in children]
    assert len(roots) == 1
    poses[roots[0]] = np.eye(4)

    def pose(link):
        if link not in poses:
            j = children[link]
            origin = j.find("origin")
            xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
            rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
            t = np.eye(4)
            t[:3, :3], t[:3, 3] = rpy_matrix(*rpy), xyz
            kind = j.get("type")
            if kind in ("revolute", "continuous"):
                a = j.find("axis")
                r = np.eye(4)
                r[:3, :3] = axis_angle([float(v) for v in (a.get("xyz") if a is not None else "1 0 0").split()], q(j.get("name")))
                t = t @ r
            elif kind == "prismatic":
                a = j.find("axis")
                d = np.array([float(v) for v in (a.get("xyz") if a is not None else "1 0 0").split()]) * q(j.get("name"))
                s = np.eye(4)
                s[:3, 3] = d
                t = t @ s
            else:
                assert kind == "fixed", f"{j.get('name')}: {kind}"
            poses[link] = pose(j.find("parent").get("link")) @ t
        return poses[link]

    for link in [l.get("name") for l in root.findall("link")]:
        pose(link)
    return poses


def random_configuration(root, rng):
    values = {}
    for j in root.findall("joint"):
        if j.get("type") in ("revolute", "continuous", "prismatic") and j.find("mimic") is None:
            lim = j.find("limit")
            lo, hi = (float(lim.get("lower")), float(lim.get("upper"))) if j.get("type") != "continuous" else (-math.pi, math.pi)
            values[j.get("name")] = rng.uniform(lo, hi)
    return values


@pytest.mark.parametrize("base_plate", BASE_PLATES)
def test_the_constant_pairs_have_the_same_relative_transform_in_every_configuration(base_plate):
    """The reasoning, tested against a forward kinematics that shares no code with it: for every pair the tree calls constant, the relative pose
    is identical at several random configurations; and the pairs it calls non-constant do move (the rule is not vacuous)."""
    root = urdf_root(base_plate)
    certified = sorted(derive_mod.constant_pairs(root))
    sampled = sorted(p for p in itertools.combinations(sorted(derive_mod.geometry_links(root)), 2) if p not in set(certified))
    rng = np.random.default_rng(7)
    configs = [forward_kinematics(root, random_configuration(root, rng)) for _ in range(4)]
    for a, b in certified:
        relative = [np.linalg.inv(c[a]) @ c[b] for c in configs]
        for other in relative[1:]:
            assert np.allclose(relative[0], other, atol=1e-9), f"{a} <-> {b} is called constant but moves"
    moves = [any(not np.allclose(np.linalg.inv(configs[0][a]) @ configs[0][b], np.linalg.inv(c[a]) @ c[b], atol=1e-9) for c in configs[1:])
             for a, b in sampled]
    assert all(moves), [p for p, m in zip(sampled, moves) if not m][:5]



@pytest.mark.parametrize("base_plate", BASE_PLATES)
def test_every_disabled_pair_has_a_reason_that_can_be_shown_and_a_true_label(base_plate):
    """The static half of M03 at every base plate, with no collision checker: no pair is disabled on a statistic, and no label lies."""
    import validate_acm
    result = validate_acm.check_matrix(urdf_root(base_plate), srdf_pairs(), upstream_pairs())
    assert result["unjustified"] == [] and result["bad_labels"] == []


def test_the_matrix_has_only_the_three_reasons_and_no_pair_twice():
    pairs = srdf_pairs()                                  # raises on a pair disabled twice
    assert set(pairs.values()) == {"Adjacent", "ConstantTransform"}, "no `Never`, no `Default`: sampling is not a reason"
    up = upstream_pairs()
    assert len(up) == 20 and set(up.values()) == {"Adjacent"}, "upstream's v2.0 SRDF declares 20 pairs, all Adjacent"
    assert set(up) <= set(pairs) and all(pairs[p] == r for p, r in up.items())


def test_the_counts_the_report_states():
    pairs = srdf_pairs()
    root = urdf_root()
    adjacent, constant = derive_mod.structural_adjacent(root), derive_mod.constant_pairs(root)
    up = upstream_pairs()
    labelled = {reason: sum(1 for r in pairs.values() if r == reason) for reason in set(pairs.values())}
    # P12A: 113 = 20 + 16 + 77. P15: the arms are above a prismatic joint now, so the pairs between the base and the arms are no longer
    # constant, and the mast's go with the mast: 83 = 20 + 18 + 45. Expected, and explained pair by pair in the next test.
    # Follow-up (a), 16 Oct: the lift column's two stages carry placeholder boxes, so lift_column_base_link and lift_carriage_link are
    # geometry links: 95 = 20 + 17 + 58 (the column base joins the base side's constant set; the carriage becomes the arms' neighbour).
    # P28, 28 Oct: the adapter plate is a geometry link under the carriage, and the arm profile stands on it: 99 = 20 + 18 + 61 (the plate is the
    # carriage's and the profile's neighbour; the profile and the carriage are no longer neighbours but a constant pair; the plate joins the
    # two arm base links in the constant set; the stages' pair across lift_joint stays Adjacent, so nesting the inner stage moved no pair).
    # Item 24, 1 Nov: the P-03 item 24 equipment adds eight geometry links on fixed joints: 179 = 20 + 26 + 133. Each new link is its parent's
    # neighbour (+8 Adjacent: tablet and head light on the profile, the rod on the plate, the flag on the rod, the second E-stop and the three
    # stop sensors on the column base) and joins its side's constant set (+72 ConstantTransform: the static ones with the base's links, the
    # moving ones with the plate, the profile, the carriage and the arm base links). Nothing spans the lift and no existing pair changed.
    # Head camera, 1 Nov: the ZED Mini body makes head_camera_link a geometry link and the OpenArm mount placeholder adds
    # head_camera_bracket_link: 198 = 20 + 29 + 149 (+3 Adjacent: the body and the mount to the carriage across the geometry-free lens pivot,
    # and to each other; +16 ConstantTransform with the moving side's links). Nothing spans the lift and no existing pair changed.
    # P-03 rev20.4 item 28, 8 Nov: the second E-stop moves from the column base to the adapter plate: its Adjacent pair is now with the
    # plate (49 stays), and it leaves the static constant set (15 pairs) for the moving one (11 pairs): 194 = 20 + 29 + 145.
    # Platform pin 713574f (platform-sw #126), 10 Nov: camera_link has no collision geometry since platform-sw #106, so its 14 pairs are
    # gone (-1 Adjacent with base_link, -13 ConstantTransform with the base side's links): 180 = 20 + 28 + 132. No pair added or relabelled.
    assert len(pairs) == 180 and labelled == {"Adjacent": 48, "ConstantTransform": 132}
    assert len(adjacent) == 46 and len(up) == 20
    assert set(up) - adjacent == {("openarm_left_ee_link1", "openarm_left_ee_link2"), ("openarm_right_ee_link1", "openarm_right_ee_link2")}, \
        "the two finger pairs are siblings through two movable joints: upstream's declaration is their only reason"
    assert {p for p, r in pairs.items() if r == "ConstantTransform"} == constant - adjacent - set(up) and len(constant - adjacent - set(up)) == 132


@pytest.mark.parametrize("base_plate", BASE_PLATES)
def test_the_by_construction_set_is_61_not_77_because_a_prismatic_joint_now_lies_between_the_base_and_the_arms(base_plate):
    """P12A certified 77 pairs by construction (P11 got 77 by a different route). P15 EXPECTS fewer: a prismatic joint between two links
    removes them from what a fixed transform certifies, and the lift is such a joint. This asserts the shrink is exactly that and
    nothing else: a constant pair has both its links on one side of lift_joint, and every pair that was constant only because the
    mast was fixed has left the set. 61 is what remains (58 until P28): the base's own links among themselves (the lift column's fixed stage,
    lift_column_base_link, among them since follow-up (a)) and, above the lift, the adapter plate, the arm profile and the arm base links among
    themselves and the profile with the carriage (P28 added the plate's two and the profile-carriage one). 133 since the P-03 item 24
    equipment (1 Nov): its eight fixed geometry links join their side's constant set, and none spans the lift (asserted below). 149 with
    the head camera body and its mount (1 Nov). 145 since the second E-stop rides the adapter plate (8 Nov: -15 static, +11 moving). 132 since
    the platform pin 713574f (10 Nov): camera_link has no collision geometry since platform-sw #106 (-13 static)."""
    root = urdf_root(base_plate)
    constant = derive_mod.constant_pairs(root) - derive_mod.structural_adjacent(root) - set(upstream_pairs())
    assert len(constant) == 132
    t = split.urdf_tree(root)
    def above_the_lift(link):
        return "lift_joint" in [j for j in split.movable_joints_between(t, "base_footprint", link)]
    spanning = [p for p in derive_mod.constant_pairs(root) if above_the_lift(p[0]) != above_the_lift(p[1])]
    assert spanning == [], f"a constant pair spans the lift: {spanning[:3]}"
    # the 32 that left: every pair of a base-side link with something above the lift, and everything the mast had
    old_constant_shapes = {(a, b) for a in derive_mod.geometry_links(root) for b in derive_mod.geometry_links(root)
                           if a < b and above_the_lift(a) != above_the_lift(b)}
    assert old_constant_shapes and not (old_constant_shapes & constant)


URDF = """<robot name="r">
  <link name="a"><collision/></link><link name="frame1"/><link name="b"><collision/></link><link name="c"><collision/></link>
  <link name="d"><collision/></link><link name="e"><collision/></link><link name="f"><collision/></link><link name="nogeo"/>
  <joint name="j_a_f" type="fixed"><parent link="a"/><child link="frame1"/></joint>
  <joint name="j_f_b" type="fixed"><parent link="frame1"/><child link="b"/></joint>
  <joint name="j_b_c" type="revolute"><parent link="b"/><child link="c"/></joint>
  <joint name="j_c_d" type="revolute"><parent link="c"/><child link="d"/></joint>
  <joint name="j_c_e" type="revolute"><parent link="c"/><child link="e"/></joint>
  <joint name="j_b_f" type="fixed"><parent link="b"/><child link="f"/></joint>
</robot>"""


def test_structural_neighbours_are_decided_by_geometry_bearing_links_and_movable_joints_on_the_path():
    root = ET.fromstring(URDF)
    adjacent = derive_mod.structural_adjacent(root)
    assert ("a", "b") in adjacent, "a geometry-less frame between two links does not separate them"
    assert ("b", "c") in adjacent and ("c", "d") in adjacent, "parent and child across one movable joint"
    assert ("a", "c") not in adjacent, "b, a geometry-bearing link, lies between them"
    assert ("b", "d") not in adjacent
    assert ("d", "e") not in adjacent, "siblings through two movable joints are not neighbours (upstream declares the gripper's own pair)"
    assert ("b", "f") in adjacent and ("a", "f") not in adjacent
    assert not any("nogeo" in p or "frame1" in p for p in adjacent), "links without collision geometry are never in the matrix"


def test_constant_pairs_have_no_movable_joint_between_their_links():
    root = ET.fromstring(URDF)
    assert derive_mod.constant_pairs(root) == {("a", "b"), ("a", "f"), ("b", "f")}


def test_derive_disables_a_clear_constant_pair_by_construction_and_reports_one_that_collides_instead_of_hiding_it():
    root = ET.fromstring(URDF)
    upstream = {("c", "e"): "Adjacent"}                                   # kept exactly as upstream declares it
    matrix, report = derive_mod.derive(root, upstream, lambda pairs: {("a", "f")} & set(pairs))
    assert matrix[("c", "e")] == "Adjacent" and report["upstream"] == [("c", "e")]
    assert matrix[("a", "b")] == matrix[("b", "c")] == matrix[("b", "f")] == "Adjacent"
    assert report["constant_and_colliding"] == [("a", "f")] and ("a", "f") not in matrix, "a permanent collision is reported, not disabled"
    assert ("d", "e") not in matrix and ("a", "d") not in matrix, "nothing else is disabled: everything not justified stays enabled"
    clear, _ = derive_mod.derive(root, {}, lambda pairs: set())
    assert clear[("a", "f")] == "ConstantTransform"


def test_derive_does_not_depend_on_anything_random():
    root = urdf_root()
    first = derive_mod.derive(root, upstream_pairs(), lambda pairs: set())
    second = derive_mod.derive(root, upstream_pairs(), lambda pairs: set())
    assert first == second and split.acm_fingerprint(first[0]) == split.acm_fingerprint(second[0])


def test_justifications_say_which_reasons_apply():
    root = ET.fromstring(URDF)
    assert derive_mod.justifications(("b", "c"), root, {}) == {derive_mod.ADJACENT}
    assert derive_mod.justifications(("a", "f"), root, {}) == {derive_mod.CONSTANT}
    assert derive_mod.justifications(("d", "e"), root, {}) == set()
    assert derive_mod.justifications(("d", "e"), root, {("e", "d"): "Adjacent"}) == {derive_mod.UPSTREAM}


def test_a_pair_disabled_on_a_statistic_is_not_justified_and_a_label_that_lies_is_caught():
    import validate_acm
    root = urdf_root()
    pairs = dict(srdf_pairs())
    pairs[("openarm_left_link1", "openarm_left_link3")] = "Never"                       # the old sampled claim
    pairs[("base_link", "openarm_left_ee_base_link")] = "ConstantTransform"             # the lift and arm joints lie between them
    result = validate_acm.check_matrix(root, pairs, upstream_pairs())
    assert [tuple(u["pair"]) for u in result["unjustified"]] == [("base_link", "openarm_left_ee_base_link"), ("openarm_left_link1", "openarm_left_link3")]
    problems = {tuple(b["pair"]): b["problem"] for b in result["bad_labels"]}
    assert "not one of" in problems[("openarm_left_link1", "openarm_left_link3")]
    assert "movable joint" in problems[("base_link", "openarm_left_ee_base_link")]
    wrong_adjacent = dict(srdf_pairs())
    wrong_adjacent[("openarm_left_link1", "openarm_left_link4")] = "Adjacent"
    assert validate_acm.check_matrix(root, wrong_adjacent, upstream_pairs())["bad_labels"][0]["problem"].startswith("labelled Adjacent")


