# SPDX-License-Identifier: MIT
"""sim.launch.py spawns the model at its resting height (upperbody-sw #48), derived here from the expanded URDF, not copied.

At 0.15 m the drop landed about 0.17 s of simulated time later and pushed lift_joint past its lower stop before lift_controller was active
(A7 M02). The resting height is the distance from base_footprint down to the lowest collision point (the drive wheels, axles horizontal)."""
import importlib.util
import os
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_LAUNCH = os.path.join(HERE, os.pardir, "launch", "sim.launch.py")
TOLERANCE_M = 0.0005        # half a millimetre: a placed model, not a drop; the old value was 148 mm off


def sim():
    spec = importlib.util.spec_from_file_location("sim_launch_spawn_under_test", SIM_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def lowest_collision_z(urdf_xml):
    """Lowest collision point below the root, through fixed and moving joints at q = 0. Spheres, boxes, and cylinders whose axis is horizontal
    (roll of pi/2 about x, as the wheels are); anything else is refused so a new shape cannot slip through unmeasured."""
    root = ET.fromstring(urdf_xml)
    by_child = {j.find("child").get("link"): j for j in root.findall("joint")}

    def z_of(link):
        z = 0.0
        while link in by_child:
            origin = by_child[link].find("origin")
            if origin is not None and origin.get("xyz"):
                z += float(origin.get("xyz").split()[2])
            link = by_child[link].find("parent").get("link")
        return z

    lowest = None
    for link in root.findall("link"):
        for col in link.findall("collision"):
            origin = col.find("origin")
            oz = float(origin.get("xyz").split()[2]) if origin is not None and origin.get("xyz") else 0.0
            rpy = [float(v) for v in origin.get("rpy").split()] if origin is not None and origin.get("rpy") else [0.0, 0.0, 0.0]
            shape = col.find("geometry")[0]
            if shape.tag == "sphere":
                half = float(shape.get("radius"))
            elif shape.tag == "box" and all(abs(a) < 1e-6 for a in rpy[:2]):
                half = float(shape.get("size").split()[2]) / 2
            elif shape.tag == "cylinder" and abs(abs(rpy[0]) - 1.5707963) < 1e-4 and abs(rpy[1]) < 1e-6:
                half = float(shape.get("radius"))
            elif shape.tag == "cylinder" and all(abs(a) < 1e-6 for a in rpy[:2]):
                half = float(shape.get("length")) / 2
            elif shape.tag == "mesh":
                continue        # meshes are visual-grade here; the base's ground contacts are primitives (asserted below)
            else:
                raise AssertionError(f"{link.get('name')}: collision {shape.tag} with rpy {rpy} is not handled")
            bottom = z_of(link.get("name")) + oz - half
            if lowest is None or bottom < lowest[0]:
                lowest = (bottom, link.get("name"))
    return lowest


@pytest.fixture(scope="module")
def urdf():
    xacro = pytest.importorskip("xacro")
    share = pytest.importorskip("ament_index_python.packages").get_package_share_directory("openamrobot_upperbody_description")
    return xacro.process_file(os.path.join(share, "urdf", "combined_robot.xacro"), mappings={"hardware_profile": "gazebo"}).toxml()


def test_the_model_is_spawned_at_its_resting_height(urdf):
    bottom, link = lowest_collision_z(urdf)
    assert link in ("left_wheel_link", "right_wheel_link"), f"the lowest collision is on {link}, not a drive wheel"
    assert float(sim().SPAWN_HEIGHT) == pytest.approx(-bottom, abs=TOLERANCE_M)


def test_the_spawn_height_is_a_launch_argument_defaulting_to_it():
    module = sim()
    names = {a.name: a for a in module.generate_launch_description().entities if type(a).__name__ == "DeclareLaunchArgument"}
    assert "spawn_z" in names and names["spawn_z"].default_value[0].text == module.SPAWN_HEIGHT
