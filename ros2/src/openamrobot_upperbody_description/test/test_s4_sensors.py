# SPDX-License-Identifier: MIT
"""Sensor frames and the SO-101 stub: where they attach, what they carry, and when they exist.

The head camera is option B's (head_camera_mount_link -> head_camera_link -> head_camera_optical_frame on the lift carriage, with
sourced numbers); the two wrist cameras are ours, at the pose of their datums. P15 deleted A6's head_camera_pose placeholder."""
import os
import numpy as np
import pytest

from conftest import ARM_PACKAGES, BASE_PLATES, SHARE, expand, load_datums, load_yaml, tree_facts
from urdf_utils import fixed_fk, pose_to_matrix, position

WRIST_CAMERAS = {
    "left_wrist_camera": ("openarm_left_ee_base_link", "left_wrist_camera_pose"),
    "right_wrist_camera": ("openarm_right_ee_base_link", "right_wrist_camera_pose"),
}
CAMERAS = {"head_camera": None, **WRIST_CAMERAS}


@pytest.mark.parametrize("base_plate", BASE_PLATES)
@pytest.mark.parametrize("name", WRIST_CAMERAS)
def test_wrist_camera_frames_attach_where_the_contract_says_at_the_datum_pose(name, base_plate):
    parent, datum = WRIST_CAMERAS[name]
    root = expand(base_plate=base_plate, arm_package="openarm")
    _, _, child_of = tree_facts(root)
    assert child_of[f"{name}_link"] == parent
    assert child_of[f"{name}_optical_frame"] == f"{name}_link"
    pose = load_datums()["datums"][datum]["value"]
    assert np.allclose(fixed_fk(root, parent, f"{name}_link"), pose_to_matrix(pose["xyz"], pose["rpy"]))


@pytest.mark.parametrize("name", CAMERAS)
def test_optical_frame_is_rep103(name):
    """z forward, x right, y down, with the link frame x forward, y left, z up."""
    root = expand(arm_package="openarm")
    rot = fixed_fk(root, f"{name}_link", f"{name}_optical_frame")
    assert np.allclose(position(rot), 0)
    assert np.allclose(rot[:3, 2], [1, 0, 0])      # optical z = link x (forward)
    assert np.allclose(rot[:3, 0], [0, -1, 0])     # optical x = -link y (right)
    assert np.allclose(rot[:3, 1], [0, 0, -1])     # optical y = -link z (down)


@pytest.mark.parametrize("name", WRIST_CAMERAS)
def test_wrist_camera_links_are_visual_only(name):
    link = expand(arm_package="openarm").find(f"link[@name='{name}_link']")
    assert link.find("visual") is not None
    assert link.find("collision") is None
    # the wrist camera mass (BOM ARM-004) from the roadmap export, counted once (8 Nov 2026); no inertial on the datum path
    export = load_yaml(os.path.join(SHARE, "config", "roadmap_mass_export.yaml"))
    want = export["groups"]["sensors"]["links"][f"{name}_link"]["mass_kg"]
    assert float(link.find("inertial/mass").get("value")) == pytest.approx(want)
    assert expand(arm_package="openarm", roadmap_masses="false").find(f"link[@name='{name}_link']").find("inertial") is None


def test_the_head_camera_is_option_bs_not_ours_and_has_no_datum():
    assert "head_camera_pose" not in load_datums()["datums"], "A6's placeholder pose is deleted: option B's frames are sourced"


@pytest.mark.parametrize("arm_package", ARM_PACKAGES)
def test_head_camera_is_on_the_lift_carriage_not_on_an_arm(arm_package):
    root = expand(arm_package=arm_package)
    chain, link = [], "head_camera_link"
    _, _, child_of = tree_facts(root)
    while link in child_of:
        link = child_of[link]
        chain.append(link)
    assert chain == ["head_camera_mount_link", "lift_carriage_link", "lift_column_base_link", "base_footprint"]
    assert not [n for n in chain if n.startswith("openarm_")]


@pytest.mark.parametrize("arm_package", [a for a in ARM_PACKAGES if a != "openarm"])
def test_no_camera_frames_without_openarm(arm_package):
    links, joints, _ = tree_facts(expand(arm_package=arm_package))
    # option B's head camera is there for every arm package; the wrist cameras are the arms' and need the arms
    assert not [n for n in links + joints if "camera" in n and not n.startswith("camera_") and not n.startswith("base_camera")
                and not n.startswith("head_camera")]


def test_so101_stub_is_one_link_at_identity_on_the_lift_carriage():
    root = expand(arm_package="so101")
    links, joints, child_of = tree_facts(root)
    assert child_of["so101_mount_link"] == "lift_carriage_link"
    assert np.allclose(fixed_fk(root, "lift_carriage_link", "so101_mount_link"), np.eye(4))
    assert [n for n in links if n.startswith("so101_")] == ["so101_mount_link"]
    assert [n for n in joints if n.startswith("so101_")] == ["so101_mount_joint"]
    assert root.find("joint[@name='so101_mount_joint']").get("type") == "fixed"
    # no arm links; option B's two mount FRAMES are there with every arm package
    assert sorted(n for n in links if n.startswith("openarm_")) == ["openarm_left_base_link", "openarm_right_base_link"]


@pytest.mark.parametrize("arm_package", ["openarm", "none"])
def test_no_so101_frames_unless_so101(arm_package):
    links, joints, _ = tree_facts(expand(arm_package=arm_package))
    assert not [n for n in links + joints if n.startswith("so101_")]
