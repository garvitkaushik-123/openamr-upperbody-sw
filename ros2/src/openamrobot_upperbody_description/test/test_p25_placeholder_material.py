# SPDX-License-Identifier: MIT
"""P25 (P28: four boxes since the adapter plate became its own link): the placeholder boxes (the two lift-column stages, the adapter plate and arm_profile_link) have one named material with an explicit colour,
in the URDF for RViz and in a <gazebo reference> block for Gazebo, and the two carry the same numbers.

Why a test: a visual with no <material> is drawn red by RViz and black by Gazebo, so the two windows disagreed about the same box; and
sdformat applies a link's <gazebo><visual><material> block only to a visual that has no name of its own, so renaming a column visual
back to placeholder_lift_column_*_stage silently puts the colour back to whatever the URDF material converts to (sdformat scales a URDF
colour by 1.25 on the way into the SDF, so the two windows would differ in brightness, not in hue). The SDF part runs the same
conversion `ros_gz_sim create` does (`gz sdf -p`) and is skipped when `gz` is not on the PATH.
"""
import os
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import pytest

from conftest import MATRIX, expand, expand_xml

LINKS = ["lift_column_base_link", "lift_carriage_link", "adapter_plate_link", "arm_profile_link",
         # P-03 item 24 equipment (upper_body_equipment.xacro): placeholder geometry in the column's placeholder material
         "getac_tablet_link", "head_light_link", "second_estop_link", "lift_stop_sensor_l1000_link", "lift_stop_sensor_l1175_link",
         "lift_stop_sensor_l1350_link", "lift_flag_rod_link", "lift_flag_link",
         # the ZED Mini body on option B's head_camera_link and the OpenArm mount placeholder (1 Nov 2026)
         "head_camera_link", "head_camera_bracket_link"]
MATERIAL = "placeholder_geometry"


def _floats(text):
    return [float(v) for v in text.split()]


def _rgba(root):
    materials = [m for m in root.findall("material") if m.get("name") == MATERIAL]
    assert len(materials) == 1, f"expected one robot-level <material name={MATERIAL!r}>, found {len(materials)}"
    color = materials[0].find("color")
    assert color is not None and color.get("rgba"), f"material {MATERIAL!r} has no explicit <color rgba>"
    rgba = _floats(color.get("rgba"))
    assert len(rgba) == 4 and all(0.0 <= v <= 1.0 for v in rgba), rgba
    return rgba


@pytest.mark.parametrize("base_plate,arm_package", MATRIX, ids=[f"{p}-{a}" for p, a in MATRIX])
def test_urdf_material_and_gazebo_block(base_plate, arm_package):
    root = expand(base_plate=base_plate, arm_package=arm_package)
    rgba = _rgba(root)
    links = {l.get("name"): l for l in root.findall("link")}
    gazebo = {g.get("reference"): g for g in root.findall("gazebo") if g.get("reference")}
    for name in LINKS:
        visuals = links[name].findall("visual")
        assert visuals, f"{name} has no <visual>"
        for visual in visuals:
            material = visual.find("material")
            assert material is not None and material.get("name") == MATERIAL, f"{name}: a visual without material {MATERIAL!r}"
            assert visual.get("name") is None, (f"{name}: a named visual ({visual.get('name')!r}) does not take the <gazebo> material "
                                                f"block (sdformat matches only the visual it names <link>_visual); leave it unnamed")
        assert name in gazebo, f"{name}: no <gazebo reference> block"
        material = gazebo[name].find("visual/material")
        assert material is not None, f"{name}: the <gazebo> block has no <visual><material>"
        for tag in ("ambient", "diffuse"):
            element = material.find(tag)
            assert element is not None, f"{name}: the <gazebo> material has no <{tag}>"
            assert _floats(element.text) == pytest.approx(rgba), f"{name}: Gazebo {tag} {element.text!r} is not the URDF colour {rgba}"


def test_colour_is_not_used_by_any_other_visual():
    """The colour says 'placeholder': no other link of the combined robot may use the material."""
    root = expand(base_plate="bp000", arm_package="openarm")
    users = sorted(l.get("name") for l in root.findall("link")
                   if any(v.find("material") is not None and v.find("material").get("name") == MATERIAL for v in l.findall("visual")))
    assert users == sorted(LINKS), users


@pytest.mark.skipif(shutil.which("gz") is None, reason="gz is not on the PATH")  # tracking: openAMRobot/openamr-upperbody-sw#16
@pytest.mark.parametrize("base_plate", ["bp000", "bp050"])
def test_sdf_carries_the_same_colour(base_plate):
    text = expand_xml(base_plate=base_plate)
    rgba = _rgba(ET.fromstring(text))
    with tempfile.TemporaryDirectory() as tmp:
        urdf = os.path.join(tmp, "robot.urdf")
        with open(urdf, "w") as f:
            f.write(text)
        sdf = subprocess.run(["gz", "sdf", "-p", urdf], capture_output=True, text=True, timeout=120, check=True).stdout
    seen = {}
    for m in re.finditer(r"<visual name='([^']*)'>(.*?)</visual>", sdf, re.S):
        name, body = m.group(1), m.group(2)
        for link in LINKS:
            if re.search(rf"(^|__){re.escape(link)}_visual(_\d+)?$", name):
                seen[link] = {tag: _floats(re.search(rf"<{tag}>([^<]*)</{tag}>", body).group(1)) for tag in ("ambient", "diffuse")}
    assert sorted(seen) == sorted(LINKS), f"visuals found in the SDF for {sorted(seen)}"
    for link, colours in seen.items():
        for tag, value in colours.items():
            assert value == pytest.approx(rgba, abs=1e-6), f"{link}: SDF {tag} {value} is not the URDF colour {rgba}"
