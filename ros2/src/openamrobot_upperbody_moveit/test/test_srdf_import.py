# SPDX-License-Identifier: MIT
"""ADR-007: what was imported from upstream's SRDF is still what upstream has at the pinned commit.

The groups, the group states `home` and `hands_up`, the end effectors, the passive joints and the arm-internal disable_collisions are
compared, as sets of attributes, with enactic/openarm_ros2's openarm_bimanual.srdf in the CHECKOUT under src/upstream (which C02 holds
at the pinned commit). One element is allowed to differ, on purpose: the pair with openarm_body_link0, dropped because this model has no
OpenArm body (ADR-008). The groups this SRDF adds of its own (`lift` and `both_arms`, issue #17) are not imported and are not compared; every
group upstream declares is still here, exactly as upstream declares it, and no other group is added. With no checkout there is nothing to compare, and that is a SKIP that says so, the same legitimate case as C02's:
the SRDF is committed, so nothing is lost but this comparison."""
import os

import pytest

from conftest import description_scripts, srdf_root

UPSTREAM_SRDF = os.path.join("openarm_bimanual_moveit_config", "config", "openarm_v2.0", "openarm_bimanual.srdf")
DROPPED_LINK = "openarm_body_link0"


def upstream_srdf():
    description_scripts()
    import evidence_common as ev
    from xml.etree import ElementTree as ET
    workspace = ev.workspace_root()
    checkout = ev.upstream_checkout(workspace, "openarm_ros2")
    path = os.path.join(checkout, UPSTREAM_SRDF)
    if not os.path.isfile(path):
        pytest.skip(f"no checkout of openarm_ros2 at {checkout}: the imported SRDF elements cannot be compared with upstream's "
                    f"(config/upstream_pins.yaml pins the commit; C02 compares it with the checkout when there is one)")  # tracking: openAMRobot/openamr-upperbody-sw#16
    pinned = ev.all_pins().get("openarm_ros2")
    head = ev.git_head(checkout)
    assert head == pinned, f"the checkout is at {head}, the pin is {pinned}: this comparison would not be against the pinned commit"
    return ET.parse(path).getroot()


def normal(root, tag, attribute_names, children=None):
    """Set of (attributes, children) for every `tag` element, children being the (name, value) of the <joint> children."""
    out = set()
    for el in root.findall(tag):
        attrs = tuple((n, el.get(n)) for n in attribute_names)
        kids = tuple(sorted((j.get("name"), j.get("value")) for j in el.findall("joint"))) if children else ()
        out.add((attrs, kids))
    return out


def pairs(root):
    return {frozenset((d.get("link1"), d.get("link2"))): d.get("reason") for d in root.findall("disable_collisions")}


OURS_GROUPS = {"lift", "both_arms"}       # section 2 of the SRDF (issue #17): not imported


def imported_groups(root):
    return {(a, k) for (a, k) in normal(root, "group", ["name"], True) if dict(a)["name"] not in OURS_GROUPS}


def test_the_imported_groups_end_effectors_and_passive_joints_are_upstreams():
    up, ours = upstream_srdf(), srdf_root()
    assert not OURS_GROUPS & {g.get("name") for g in up.findall("group")}, "upstream now declares a group of the same name: re-import (ADR-007)"
    assert imported_groups(ours) == normal(up, "group", ["name"], True)
    assert {g.get("name") for g in ours.findall("group")} - {dict(a)["name"] for a, _ in imported_groups(ours)} == OURS_GROUPS
    assert normal(ours, "end_effector", ["name", "parent_link", "group"]) == normal(up, "end_effector", ["name", "parent_link", "group"])
    assert normal(ours, "passive_joint", ["name"]) == normal(up, "passive_joint", ["name"])


def test_the_imported_group_states_home_and_hands_up_are_upstreams():
    up, ours = upstream_srdf(), srdf_root()
    wanted = lambda root: {(a, k) for (a, k) in normal(root, "group_state", ["name", "group"], True) if dict(a)["name"] in ("home", "hands_up")}
    assert wanted(ours) == wanted(up) and len(wanted(up)) == 4


def test_every_upstream_arm_internal_pair_is_disabled_here_with_the_same_reason_except_the_dropped_body_pair():
    up, ours = upstream_srdf(), srdf_root()
    ours_pairs, up_pairs = pairs(ours), pairs(up)
    dropped = {p for p in up_pairs if DROPPED_LINK in p}
    assert len(dropped) == 1, "upstream's SRDF changed: re-import deliberately (ADR-007)"
    for pair, reason in up_pairs.items():
        if pair in dropped:
            assert pair not in ours_pairs
        else:
            assert ours_pairs.get(pair) == reason, f"{sorted(pair)}: upstream says {reason!r}, this SRDF says {ours_pairs.get(pair)!r}"
    assert len(up_pairs) - len(dropped) == 20
