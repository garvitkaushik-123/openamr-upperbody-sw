# SPDX-License-Identifier: MIT
"""Static checks of the software-in-the-loop package: pins, configuration, the launch file's own logic.
No CAN interface, no Gazebo, no ROS graph."""
import importlib.util
import os
import re
import subprocess

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(HERE)
HEX40 = re.compile(r"[0-9a-f]{40}")


def load(path):
    with open(path) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope="module")
def sil_launch():
    spec = importlib.util.spec_from_file_location("sil_launch", os.path.join(PACKAGE, "launch", "sil.launch.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_every_pin_is_a_quoted_full_hash():
    path = os.path.join(PACKAGE, "config", "sil_pins.yaml")
    pins = load(path)["pins"]
    assert set(pins) == {"openarm_ros2", "openarm_can", "openarm_description", "openarm_docs"}
    for name, commit in pins.items():
        assert isinstance(commit, str) and HEX40.fullmatch(commit), f"{name}: {commit!r}"
    lines = [line for line in open(path).read().splitlines() if re.match(r"^  \w+: ", line)]
    assert len(lines) == len(pins)
    for line in lines:
        assert re.fullmatch(r'  \w+: "[0-9a-f]{40}"', line), line


def test_the_driver_pin_is_the_models_pin():
    model = os.path.join(PACKAGE, "..", "openamrobot_upperbody_description", "config", "upstream_pins.yaml")
    if not os.path.exists(model):
        pytest.skip("openamrobot_upperbody_description is not beside this package")
    ours, theirs = load(os.path.join(PACKAGE, "config", "sil_pins.yaml"))["pins"], load(model)["pins"]
    assert ours["openarm_ros2"] == theirs["openarm_ros2"]
    assert ours["openarm_description"] == theirs["openarm_description"]


def test_the_ci_workflow_checks_out_the_pins_file_commits():
    workflow = os.path.join(PACKAGE, "..", "..", "..", ".github", "workflows", "openarm-emulator-tests.yml")
    if not os.path.exists(workflow):
        pytest.skip("the repository's workflow directory is not beside this package")
    text = open(workflow).read()
    assert "config/sil_pins.yaml" in text                    # the commits are read from the file, not copied
    assert not HEX40.search(re.sub(r"uses: \S+@[0-9a-f]{40}", "", text)), "a commit hash is written into the workflow"


def test_sim_controllers_declare_both_emulators_on_their_own_interfaces():
    config = load(os.path.join(PACKAGE, "config", "sim_controllers.yaml"))
    manager = config["/**/controller_manager"]["ros__parameters"]
    assert manager["update_rate"] == 1000
    for name, interface, prefix in (("left_arm_emulator", "vcan_arm_l", "left_"), ("right_arm_emulator", "vcan_arm_r", "right_")):
        assert manager[name]["type"] == "openamrobot_openarm_emulator/OpenArmCanEmulator"
        parameters = config[f"/**/{name}"]["ros__parameters"]
        assert parameters["can_interface"] == interface
        assert parameters["arm_prefix"] == prefix
        assert parameters["motor_types"] == ["DM8009", "DM8009", "DM4340", "DM4340", "DM4310", "DM4310", "DM4310", "DM4310"]
        assert parameters["peak_torque"] == [40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 7.0, 7.0]
        assert parameters["supply_voltage"] == 25.6
        assert parameters["timing"] == {"model": True, "nominal_bitrate": 1000000.0, "data_bitrate": 5000000.0, "turnaround_us": 0.0}
        assert not parameters["fault"]["can_loss"] and not parameters["fault"]["estop_power_cut"]
        for key in ("silent_motors", "error_codes", "temperature_mos_c", "temperature_rotor_c"):
            assert len(parameters["fault"][key]) == 8
    left, right = (dict(config[f"/**/{n}"]["ros__parameters"]) for n in ("left_arm_emulator", "right_arm_emulator"))
    for parameters in (left, right):
        del parameters["can_interface"], parameters["arm_prefix"]
    assert left == right


def test_can_fd_decision(sil_launch):
    assert sil_launch.decide_can_fd("auto", [72, 72])[0] is True
    fd, text = sil_launch.decide_can_fd("auto", [16, 16])
    assert fd is False and "CAN FD OFF" in text and "classic" in text       # reported, not silent
    assert sil_launch.decide_can_fd("auto", [72, 16])[0] is False
    assert sil_launch.decide_can_fd("false", [72, 72])[0] is False
    fd, text = sil_launch.decide_can_fd("true", [16, 16])
    assert fd is True and "will fail" in text


def test_interface_mtu_reads_sysfs(sil_launch, tmp_path):
    (tmp_path / "vcan_arm_l").mkdir()
    (tmp_path / "vcan_arm_l" / "mtu").write_text("72\n")
    assert sil_launch.interface_mtu("vcan_arm_l", str(tmp_path)) == 72
    assert sil_launch.interface_mtu("vcan_arm_r", str(tmp_path)) is None


def test_sim_namespace_is_added_to_the_models_plugin_element_only(sil_launch):
    urdf = f'<robot><gazebo>{sil_launch.GZ_PLUGIN_OPEN}<parameters>/x.yaml</parameters></plugin></gazebo><link name="a"/></robot>'
    out = sil_launch.sim_description(urdf)
    assert out == urdf.replace(sil_launch.GZ_PLUGIN_OPEN, sil_launch.GZ_PLUGIN_OPEN + "<ros><namespace>/sim</namespace></ros>")
    with pytest.raises(RuntimeError):
        sil_launch.sim_description("<robot/>")          # the model changed: fail loudly, do not start un-namespaced
    with pytest.raises(RuntimeError):
        sil_launch.sim_description(urdf + urdf)


def test_launch_arguments_reach_the_simulator_side_controller_file(sil_launch):
    config = sil_launch.sim_controllers(os.path.join(PACKAGE, "config", "sim_controllers.yaml"), "vcan_a", "vcan_b", "29.2", False)
    left, right = (config[f"/**/{n}"]["ros__parameters"] for n in ("left_arm_emulator", "right_arm_emulator"))
    assert (left["can_interface"], right["can_interface"]) == ("vcan_a", "vcan_b")
    assert left["supply_voltage"] == right["supply_voltage"] == 29.2
    assert left["timing"]["model"] is False and right["timing"]["model"] is False


def test_the_launch_expands_the_simulated_model_without_the_simulator_dynamics_profile():
    """Effort-driven joints on 1 ms steps: the profile's joint damping is stepped explicitly and is unstable without the rotor inertia
    (README, "Damping and the physics step"). The launch must ask for sim_dynamics off, and with it the model has no <dynamics> on an
    OpenArm joint. The numbers behind it are recomputed from the profile and the model."""
    with open(os.path.join(PACKAGE, "launch", "sil.launch.py")) as f:
        source = f.read()
    assert re.search(r'hardware_profile="gazebo",\s*sim_dynamics="off"', source), "sil.launch.py no longer switches the profile off"
    try:
        import xacro
        from ament_index_python.packages import get_package_share_directory
        description = get_package_share_directory("openamrobot_upperbody_description")
        get_package_share_directory("openarm_description")
        get_package_share_directory("openamrobot_description")
    except Exception as error:      # noqa: BLE001
        pytest.skip(f"description packages are not installed: {error}")
    combined = os.path.join(description, "urdf", "combined_robot.xacro")
    off = xacro.process_file(combined, mappings={"hardware_profile": "gazebo", "sim_dynamics": "off"}).documentElement
    on = xacro.process_file(combined, mappings={"hardware_profile": "gazebo"}).documentElement

    def openarm_dynamics(root):
        return [j.getAttribute("name") for j in root.getElementsByTagName("joint")
                if j.getAttribute("name").startswith("openarm_") and j.getElementsByTagName("dynamics")]
    assert openarm_dynamics(off) == [] and len(openarm_dynamics(on)) == 18
    with open(os.path.join(description, "config", "sim_dynamics", "openarm_v2.yaml")) as f:
        profile = yaml.safe_load(f)
    with open(os.path.join(PACKAGE, "config", "sim_controllers.yaml")) as f:
        inertias = yaml.safe_load(f)["/**/left_arm_emulator"]["ros__parameters"]["damping_inertia"]
    joints = [f"openarm_left_joint{i}" for i in range(1, 8)] + ["openarm_left_finger_joint1"]
    step = 0.001
    explicit = [profile["joints"][j]["damping"]["value"] * step / inertia for j, inertia in zip(joints, inertias)]
    with_rotor = [profile["joints"][j]["damping"]["value"] * step / (inertia + profile["joints"][j]["armature"]["value"])
                  for j, inertia in zip(joints, inertias)]
    assert max(explicit) > 2.0, "the profile's damping would now be stable on 1 ms steps: the launch could use it"
    assert max(with_rotor) < 2.0, "with the reflected rotor inertia the same damping is stable"


def test_setup_vcan_script_reports_blocked_or_creates(tmp_path):
    """The script either brings an interface up and says whether it carries CAN FD, or says BLOCKED and exits 3. Whichever
    this machine allows is accepted here; the hosted CI job requires the first (its workflow fails on BLOCKED)."""
    script = os.path.join(PACKAGE, "scripts", "setup_vcan.sh")
    done = subprocess.run(["sh", script, "vcan_emu_t0"], capture_output=True, text=True)
    text = done.stdout + done.stderr
    try:
        if done.returncode == 0:
            assert re.search(r"vcan_emu_t0: (created|already there), up, CAN FD (yes \(mtu 72\)|NO \(mtu \d+\))", text), text
            assert re.search(r"^CAN_FD=(yes|no)$", text, re.M), text
        else:
            assert done.returncode == 3 and "BLOCKED" in text and "CAN_FD=no" in text or "ip (iproute2)" in text, text
    finally:
        subprocess.run(["sh", script, "--delete", "vcan_emu_t0"], capture_output=True)


def test_the_real_profile_wrapper_changes_only_the_can_parameters():
    """urdf/openarm_sil_real.urdf.xacro against combined_robot.xacro at hardware_profile:=real: the same document apart
    from the two interface names and can_fd. Needs the description packages (skipped without them)."""
    xacro = pytest.importorskip("xacro")
    try:
        from ament_index_python.packages import get_package_share_directory
        description = get_package_share_directory("openamrobot_upperbody_description")
        get_package_share_directory("openarm_description")
        get_package_share_directory("openamrobot_description")
    except Exception as error:      # noqa: BLE001
        pytest.skip(f"description packages are not installed: {error}")

    def body(path, **mappings):
        document = xacro.process_file(path, mappings={"hardware_profile": "real", **mappings})
        text = "".join(node.toxml() for node in document.documentElement.childNodes)
        text = re.sub(r"<!--.*?-->", "", text, flags=re.S)      # each file has its own header comment
        return re.sub(r">\s+<", "><", text).strip()

    plain = body(os.path.join(description, "urdf", "combined_robot.xacro"))
    wrapped = body(os.path.join(PACKAGE, "urdf", "openarm_sil_real.urdf.xacro"), can_fd="false")
    assert plain.count("openarm_hardware/OpenArmHW") == 2
    assert '<param name="can_interface">vcan_arm_l</param>' in wrapped
    assert '<param name="can_interface">vcan_arm_r</param>' in wrapped
    assert wrapped.count('<param name="can_fd">False</param>') == 2
    restored = (wrapped.replace('<param name="can_interface">vcan_arm_l</param>', '<param name="can_interface">can1</param>')
                .replace('<param name="can_interface">vcan_arm_r</param>', '<param name="can_interface">can0</param>')
                .replace('<param name="can_fd">False</param>', '<param name="can_fd">True</param>'))
    assert restored == plain


def test_damping_inertia_defaults_are_the_models_link_inertias():
    """The emulator's damping_inertia (the numerical damping treatment, README) is each joint's child link, with what is
    fixed to it, about the joint axis. Recomputed here from the expanded combined model, for both arms."""
    np = pytest.importorskip("numpy")
    xacro = pytest.importorskip("xacro")
    try:
        from ament_index_python.packages import get_package_share_directory
        description = get_package_share_directory("openamrobot_upperbody_description")
        get_package_share_directory("openarm_description")
        get_package_share_directory("openamrobot_description")
    except Exception as error:      # noqa: BLE001
        pytest.skip(f"description packages are not installed: {error}")
    root = xacro.process_file(os.path.join(description, "urdf", "combined_robot.xacro"),
                              mappings={"hardware_profile": "gazebo"}).documentElement

    def numbers(element, key, default):
        text = element.getAttribute(key) if element is not None and element.hasAttribute(key) else default
        return np.array([float(x) for x in text.split()])

    def rotation(rpy):
        r, p, y = rpy
        cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
        return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])

    def child(element, tag):
        found = [n for n in element.childNodes if n.nodeType == n.ELEMENT_NODE and n.tagName == tag]
        return found[0] if found else None

    links, joints, below = {}, {}, {}
    for node in root.childNodes:
        if node.nodeType != node.ELEMENT_NODE:
            continue
        if node.tagName == "link":
            inertial = child(node, "inertial")
            if inertial is None:
                links[node.getAttribute("name")] = None
                continue
            origin, tensor = child(inertial, "origin"), child(inertial, "inertia")
            i = {k: float(tensor.getAttribute(k)) for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}
            links[node.getAttribute("name")] = (
                float(child(inertial, "mass").getAttribute("value")), numbers(origin, "xyz", "0 0 0"),
                rotation(numbers(origin, "rpy", "0 0 0")),
                np.array([[i["ixx"], i["ixy"], i["ixz"]], [i["ixy"], i["iyy"], i["iyz"]], [i["ixz"], i["iyz"], i["izz"]]]))
        elif node.tagName == "joint":
            origin, axis = child(node, "origin"), child(node, "axis")
            joints[node.getAttribute("name")] = {
                "type": node.getAttribute("type"), "child": child(node, "child").getAttribute("link"),
                "xyz": numbers(origin, "xyz", "0 0 0"), "R": rotation(numbers(origin, "rpy", "0 0 0")),
                "axis": numbers(axis, "xyz", "1 0 0")}
            below.setdefault(child(node, "parent").getAttribute("link"), []).append(node.getAttribute("name"))

    def rigid_group(link, R, p):
        parts = []
        if links[link] is not None:
            mass, com, Rl, tensor = links[link]
            parts.append((mass, p + R @ com, R @ Rl @ tensor @ Rl.T @ R.T))
        for name in below.get(link, []):
            if joints[name]["type"] == "fixed":
                parts += rigid_group(joints[name]["child"], R @ joints[name]["R"], p + R @ joints[name]["xyz"])
        return parts

    def about_axis(joint):
        axis = joints[joint]["axis"] / np.linalg.norm(joints[joint]["axis"])
        total = 0.0
        for mass, com, tensor in rigid_group(joints[joint]["child"], np.eye(3), np.zeros(3)):
            arm = com - axis * (com @ axis)
            total += axis @ tensor @ axis + mass * (arm @ arm)
        return total

    expected = load(os.path.join(PACKAGE, "config", "sim_controllers.yaml"))["/**/left_arm_emulator"]["ros__parameters"]["damping_inertia"]
    source = open(os.path.join(PACKAGE, "src", "arm_emulator.cpp")).read()
    in_code = re.search(r"damping_inertia\[8\] = \{([^}]*)\}", source).group(1)
    assert [float(x) for x in in_code.split(",")] == expected      # the yaml states the code's defaults
    for side in ("left", "right"):
        names = [f"openarm_{side}_joint{n}" for n in range(1, 8)] + [f"openarm_{side}_finger_joint1"]
        for name, value in zip(names, expected):
            assert about_axis(name) == pytest.approx(value, rel=0.02), name
