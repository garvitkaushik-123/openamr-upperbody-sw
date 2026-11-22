# SPDX-License-Identifier: MIT
"""The pure parts of scripts/a7_lift_settle_probe.py (upperbody-sw #48): the lift joint origin from the URDF, the ground truth from Gazebo poses and the per-phase summary."""
import importlib.util
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("a7_lift_settle_probe", os.path.join(HERE, os.pardir, "scripts", "a7_lift_settle_probe.py"))
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)

URDF = """<robot name="r">
  <link name="base_footprint"/><link name="base_link"/><link name="lift_column_base_link"/><link name="lift_carriage_link"/>
  <joint name="base_joint" type="fixed"><parent link="base_footprint"/><child link="base_link"/><origin xyz="0 0 0.098"/></joint>
  <joint name="col" type="fixed"><parent link="base_link"/><child link="lift_column_base_link"/><origin xyz="0.06 0 0.2" rpy="0 0 0"/></joint>
  <joint name="lift_joint" type="prismatic"><parent link="lift_column_base_link"/><child link="lift_carriage_link"/><origin xyz="0 0 0.53"/></joint>
</robot>"""


def test_the_joint_origin_is_the_sum_of_the_fixed_offsets():
    z, x = probe.lift_joint_origin_z(URDF)
    assert z == pytest.approx(0.828) and x == pytest.approx(0.06)


def test_a_rotated_joint_is_refused():
    with pytest.raises(ValueError):
        probe.lift_joint_origin_z(URDF.replace('rpy="0 0 0"', 'rpy="0 0.1 0"'))


def sample(source, value, phase="settle", t=0.0, stamp=0.1):
    return {"t": t, "stamp": stamp, "source": source, "value": value, "phase": phase}


def test_the_summary_finds_the_ci_excursion_and_uses_the_clamp_epsilon_unchanged():
    samples = [sample("raw_arm", -3.0e-4, stamp=0.18), sample("clamped", -3.244758e-05, stamp=0.2), sample("clamped", -9.9e-6),
               sample("controller_feedback", -1.0, phase="poses"), sample("raw_arm", 0.0, phase="poses")]
    s = probe.summarise(samples)
    assert s["beyond_epsilon_count"] == {"raw_arm": 1, "clamped": 1}           # -9.9e-6 is inside 1e-5; controller feedback is not judged
    assert s["first_beyond_epsilon"]["clamped"]["stamp"] == 0.2 and s["worst"]["clamped"] == pytest.approx(-3.244758e-05)
    assert s["per_phase"]["poses"]["raw_arm"] == {"n": 1, "min": 0.0, "max": 0.0}
    assert probe.EPSILON_M == 1e-5


def test_a_quiet_run_has_nothing_beyond_epsilon():
    assert probe.summarise([sample("ground_truth", -3.6e-7), sample("clamped", 0.0)])["beyond_epsilon_count"] == {}


def test_ground_truth_subtracts_the_spawn_height():
    # spawned at 0.002 m, lift at zero: the joint position is 0, not +0.002 m (review 5840679401)
    z0 = 0.828
    model_z, gt, x = probe.ground_truth_from_poses([("openamrobot", 0.0, 0.002), ("lift_carriage_link", 0.06, 0.002 + z0)], z0)
    assert model_z == pytest.approx(0.002) and gt == pytest.approx(0.0, abs=1e-12) and x == pytest.approx(0.06)


def test_ground_truth_shows_a_lower_limit_excursion_at_nonzero_spawn():
    z0 = 0.828
    _, gt, _ = probe.ground_truth_from_poses([("lift_carriage_link", 0.06, 0.002 + z0 - 3.0e-4), ("openamrobot", 0.0, 0.002)], z0)
    assert gt == pytest.approx(-3.0e-4)
    s = probe.summarise([sample("ground_truth", gt)])
    assert s["beyond_epsilon_count"] == {"ground_truth": 1}


def test_ground_truth_follows_a_settling_base():
    z0 = 0.828
    _, gt, _ = probe.ground_truth_from_poses([("openamrobot", 0.0, -0.004), ("lift_carriage_link", 0.06, -0.004 + z0 + 0.175)], z0)
    assert gt == pytest.approx(0.175)


def test_no_model_pose_means_no_ground_truth():
    model_z, gt, _ = probe.ground_truth_from_poses([("lift_carriage_link", 0.06, 0.83)], 0.828)
    assert model_z is None and gt is None
