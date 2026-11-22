# SPDX-License-Identifier: MIT
"""The two numbers M02 gates the clamp on, and nothing else (P17 task 3). No ROS graph, no simulator.

DESIGN_LOG 2026-10-14: activation counts scatter 2-3x on an unchanged tree (646-1054 in one configuration), so a count is not a measurement and
gates nothing. What gates is (1) readings beyond epsilon, which must be 0, and (2) the largest overshoot magnitude, which must be below the
epsilon of 1e-5 rad. The epsilon is not widened by any outcome; these tests pin that a widened running clamp fails."""
import importlib.util
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import a7_support  # noqa: E402

EPS = a7_support.EPSILON


def stats(**over):
    base = {"epsilon_rad": EPS, "messages": 5000, "activations": 0, "episodes": 0, "max_magnitude_rad": 0.0, "max_magnitude_joint": None,
            "over_epsilon": 0, "max_over_epsilon_rad": 0.0, "max_over_epsilon_joint": None, "joints": {}}
    base.update(over)
    return base


def test_the_epsilon_is_the_one_that_was_set():
    assert EPS == 1e-5, "the clamp's epsilon is 1e-5 rad and is not widened under any outcome (DESIGN_LOG 2026-10-14)"


def test_a_quiet_run_passes():
    assert a7_support.clamp_gate(stats())["passes"]


def test_p16s_worst_case_passes_with_a_margin_of_about_two_and_a_half():
    gate = a7_support.clamp_gate(stats(max_magnitude_rad=4.2e-6, max_magnitude_joint="openarm_left_joint4", activations=1832, episodes=40))
    assert gate["passes"] and gate["largest_overshoot_rad"] == pytest.approx(4.2e-6) and gate["largest_overshoot_joint"] == "openarm_left_joint4"


@pytest.mark.parametrize("count", [0, 873, 1832, 10 ** 6])
def test_the_number_of_activations_gates_nothing(count):
    assert a7_support.clamp_gate(stats(activations=count, episodes=count // 10))["passes"]


def test_one_excursion_beyond_epsilon_fails():
    gate = a7_support.clamp_gate(stats(over_epsilon=1, max_over_epsilon_rad=3e-5, max_over_epsilon_joint="openarm_right_finger_joint1"))
    assert not gate["passes"] and gate["excursions_beyond_epsilon"] == 1
    assert gate["largest_overshoot_rad"] == pytest.approx(3e-5) and gate["largest_overshoot_joint"] == "openarm_right_finger_joint1"


def test_a_largest_overshoot_at_epsilon_is_not_below_it():
    gate = a7_support.clamp_gate(stats(max_magnitude_rad=EPS))
    assert not gate["passes"] and any("not below epsilon" in r for r in gate["reasons"])


def test_no_clamp_is_not_a_pass():
    assert not a7_support.clamp_gate(None)["passes"]


def test_a_running_clamp_with_a_wider_epsilon_fails_even_with_quiet_numbers():
    gate = a7_support.clamp_gate(stats(epsilon_rad=2e-5))
    assert not gate["passes"] and any("wider than" in r for r in gate["reasons"])


def test_the_sidecar_labels_the_counts_diagnostic_only_and_keeps_them_out_of_the_gate():
    class Client:
        def clamp_stats(self):
            return stats(activations=1795, episodes=63, messages=9000, joints={"j": {"activations": 1795}})

    record = a7_support.clamp_record(Client())
    assert record["gate"]["passes"]
    assert "DIAGNOSTIC ONLY" in record["diagnostic_only"]["label"]
    assert record["diagnostic_only"]["activations"] == 1795 and record["diagnostic_only"]["episodes"] == 63
    assert "activations" not in record and "episodes" not in record, "counts live only under diagnostic_only"
    assert "activations" not in record["gate"] and "episodes" not in record["gate"]


def load_builder():
    spec = importlib.util.spec_from_file_location("build_a7_evidence_under_test", os.path.join(HERE, os.pardir, "scripts", "build_a7_evidence.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run_of(clamp_record):
    return {"clamp": {"present": True, "epsilon_rad": EPS, "gate": clamp_record["gate"], "diagnostic_only": clamp_record["diagnostic_only"]}}


def test_the_report_gates_on_the_two_numbers_and_labels_the_counts():
    builder = load_builder()

    class Client:
        def __init__(self, **kw):
            self.kw = kw

        def clamp_stats(self):
            return stats(**self.kw)

    clean = a7_support.clamp_record(Client(activations=5000, episodes=200, max_magnitude_rad=3e-6))
    totals = builder.clamp_totals({"L1000": run_of(clean)})
    assert totals["gate"]["passes"] and totals["gate"]["excursions_beyond_epsilon"] == 0
    assert totals["diagnostic_only"]["activations"] == 5000 and "DIAGNOSTIC ONLY" in totals["diagnostic_only"]["label"]
    bad = a7_support.clamp_record(Client(activations=3, over_epsilon=2, max_over_epsilon_rad=4e-5))
    totals = builder.clamp_totals({"L1000": run_of(bad)})
    assert not totals["gate"]["passes"] and totals["gate"]["excursions_beyond_epsilon"] == 2


def test_the_lift_excursion_of_run_42527494905_fails_and_is_reported_in_metres():
    """upperbody-sw #48: the A7 failure was lift_joint (prismatic) 3.24e-5 past its lower limit; the message said rad. Same gate, same epsilon."""
    gate = a7_support.clamp_gate(stats(over_epsilon=1, max_over_epsilon_rad=3.244758e-05, max_over_epsilon_joint="lift_joint",
                                       max_magnitude_rad=5.4e-6, max_magnitude_joint="lift_joint"))
    assert not gate["passes"] and gate["largest_overshoot_unit"] == "m"
    assert any("lift_joint: 3.24e-05 m" in r for r in gate["reasons"]) and any("3.24e-05 m on lift_joint is not below epsilon 1e-05 m" in r for r in gate["reasons"])
    assert a7_support.EPSILON == 1e-5              # not widened


def test_a_revolute_overshoot_is_still_reported_in_radians():
    gate = a7_support.clamp_gate(stats(over_epsilon=1, max_over_epsilon_rad=3e-5, max_over_epsilon_joint="openarm_right_finger_joint1"))
    assert gate["largest_overshoot_unit"] == "rad" and any("3e-05 rad" in r for r in gate["reasons"])
