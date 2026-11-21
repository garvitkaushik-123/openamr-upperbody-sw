# SPDX-License-Identifier: MIT
"""W02 (WARN): with enable_planar_base:=true, KDL has no planar joint, so robot_state_publisher converts base_planar_joint
into a fixed joint and KDL consumers see world -> base_link as static.

Why this is a WARN and not a FAIL, and why it does not touch A6's acceptance:
  - the acceptance is "planar base joint present and disabled". The joint IS present, with type="planar" in the
    description (T11), and check_urdf accepts it (T02, planar expansions);
  - the conversion is a limitation of a downstream consumer (kdl_parser), not a defect of the model;
  - A7 consequence: planar base motion can never come from robot_state_publisher. It comes from Gazebo's odom -> base_footprint,
    which ADR-003 and ADR-005 already assume.

The test observes the message from a real robot_state_publisher (a few seconds, localhost only) rather than asserting it
from memory, and checks the default expansion does not produce it. It fails only if the condition disappears (then W02 can
be retired), like W01. If robot_state_publisher cannot be run, the sidecar says observed: false and why, and the test SKIPs.
"""
import os
import signal
import subprocess
import textwrap
import warnings

import pytest

from conftest import expand_xml, write_sidecar

MESSAGE = "Converting unknown joint type of joint 'base_planar_joint' into a fixed joint"
SECONDS = 4


def _executable():
    from ament_index_python.packages import PackageNotFoundError, get_package_prefix
    try:
        path = os.path.join(get_package_prefix("robot_state_publisher"), "lib", "robot_state_publisher", "robot_state_publisher")
    except PackageNotFoundError:
        return None
    return path if os.path.isfile(path) else None


def run_robot_state_publisher(urdf, tmp_path):
    """Output of robot_state_publisher started on this URDF and stopped after SECONDS. A multi-line XML parameter cannot be
    given with -p (rcl rejects it), so the URDF goes in a params file as a YAML block scalar."""
    params = tmp_path / "rsp.yaml"
    params.write_text("robot_state_publisher:\n  ros__parameters:\n    robot_description: |\n" + textwrap.indent(urdf, "      "))
    env = {**os.environ, "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST"}
    proc = subprocess.Popen([_executable(), "--ros-args", "--params-file", str(params)],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    try:
        out, _ = proc.communicate(timeout=SECONDS)
    except subprocess.TimeoutExpired:
        proc.send_signal(signal.SIGINT)
        out, _ = proc.communicate(timeout=15)
    return out


def test_w02_kdl_converts_the_planar_joint_to_fixed(tmp_path):
    details = {
        "acceptance": "unaffected: A6's acceptance is 'planar base joint present and disabled'. The joint is present with "
                      "type=\"planar\" in the description (T11) and check_urdf accepts it (T02, planar expansions)",
        "classification": "downstream consumer limitation (kdl_parser has no planar joint), not a model defect",
        "consequence": "KDL consumers see world -> base_link as static when enable_planar_base:=true",
        "a7_consequence": "planar base motion can never come from robot_state_publisher; it comes from Gazebo's "
                          "odom -> base_footprint, which ADR-003 and ADR-005 already assume",
        "action": "none in A6: the flag is an experiment switch, disabled by default (AD-30)",
        "expected_message": MESSAGE,
    }
    planar = expand_xml(enable_planar_base="true")
    assert 'type="planar"' in planar and "base_planar_joint" in planar         # the premise: the joint is in the model

    if _executable() is None:
        write_sidecar("W02", "planar_kdl", "KDL converts base_planar_joint to a fixed joint", "WARN",
                      {**details, "observed": False, "not_observed_because": "robot_state_publisher is not installed"})
        pytest.skip("robot_state_publisher is not installed: W02 recorded without an observation")  # tracking: openAMRobot/openamr-upperbody-sw#16

    on = run_robot_state_publisher(planar, tmp_path)
    off = run_robot_state_publisher(expand_xml(), tmp_path)
    observed = MESSAGE in on
    write_sidecar("W02", "planar_kdl", "KDL converts base_planar_joint to a fixed joint", "WARN",
                  {**details, "observed": observed, "default_expansion_emits_it": MESSAGE in off,
                   "observed_in": "robot_state_publisher (Jazzy), enable_planar_base:=true, " + f"{SECONDS} s run",
                   "log_line": next((line.strip() for line in on.splitlines() if MESSAGE in line), None)})
    assert "Robot initialized" in on and "Robot initialized" in off, f"robot_state_publisher did not start:\n{on}\n{off}"
    assert MESSAGE not in off, "the default expansion must not trigger the planar conversion"
    assert observed, "the KDL conversion message is gone: the condition disappeared, retire W02"
    warnings.warn("W02 WARN: KDL converts base_planar_joint (enable_planar_base:=true) into a fixed joint; not an A6 "
                  "acceptance issue (joint present, type planar, check_urdf accepts it); planar motion for A7 comes "
                  "from Gazebo's odom -> base_footprint", UserWarning, stacklevel=1)
