# SPDX-License-Identifier: MIT
"""M03: the SRDF's collision matrix disables only what can be justified (scripts/validate_acm.py, acm_derive.py, moveit_py).

A pair may be disabled for exactly three reasons: declared by upstream (section 1 of the SRDF, the 20 pairs upstream states), `Adjacent` (a structural
neighbour) or `ConstantTransform` (no movable joint between the links: one evaluation settles every configuration, and it found them clear). Everything
else is ENABLED, including every pair whose only credential would be that a sampler saw no collision. M03 recomputes the reasons from the model and
FAILs a disabled pair with no reason, a label that is not true, a ConstantTransform pair that collides, and an enabled constant pair that collides (a
permanent collision). There is no WARN class. The sampled part is information about the ENABLED pairs, for A4.

THIS TEST IS NOT SOFTENED: the mutation tests below show that each way the matrix can be wrong fails the check, by planting that defect in a copy of the
SRDF, and that the defects are real: every pair they use is one that really collides or really has a movable joint between its links."""
import json
import os
import re
import subprocess
import sys
import tempfile

from ament_index_python.packages import get_package_prefix

from conftest import SRDF_FILE, load_yaml, read_text, write_sidecar
from openamrobot_upperbody_moveit.acm_scene import colliding_pairs

SCRIPT = os.path.join(get_package_prefix("openamrobot_upperbody_moveit"), "lib", "openamrobot_upperbody_moveit", "validate_acm.py")
SAMPLES = 2000
STOPS = ("L1000", "L1175", "L1350")           # option B's lift calibration stops: lift_joint held there, everything else sampled


def run_validate(*args, timeout):
    env = {**os.environ, "ROS_DOMAIN_ID": str(__import__("random").randint(201, 232)), "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
           "PYTHONDONTWRITEBYTECODE": "1"}
    return subprocess.run([sys.executable, SCRIPT, *args], capture_output=True, text=True, timeout=timeout, env=env)


def mutate(text, *, remove=None, relabel=None, add=None):
    """The installed SRDF's text with one defect: `remove` a pair (enable it), `relabel` (pair, from, to), or `add` (pair, label)."""
    if remove:
        text, count = re.subn(rf'    <disable_collisions link1="{remove[0]}" link2="{remove[1]}" reason="[A-Za-z]+"/>\n', "", text)
        assert count == 1, f"{remove} is not disabled in the SRDF"
    if relabel:
        pair, was, now = relabel
        text, count = re.subn(rf'(link1="{pair[0]}" link2="{pair[1]}" reason=")({was})(")', rf"\g<1>{now}\3", text)
        assert count == 1, f"{pair} is not disabled as {was}"
    if add:
        pair, label = add
        assert f'link1="{pair[0]}" link2="{pair[1]}"' not in text, f"{pair} is disabled already"
        text = text.replace("    <!-- END GENERATED ACM -->", f'    <disable_collisions link1="{pair[0]}" link2="{pair[1]}" reason="{label}"/>\n    <!-- END GENERATED ACM -->')
    return text


def validate_mutated(tmp, lift_stop="L1000", samples=20, **defect):
    srdf, out = os.path.join(tmp, "mutated.srdf"), os.path.join(tmp, "m03.json")
    with open(srdf, "w") as f:
        f.write(mutate(read_text(SRDF_FILE), **defect))
    done = run_validate("--lift-stop", lift_stop, "--samples", str(samples), "--srdf", srdf, "--json", out, timeout=300)
    with open(out) as f:
        return done, json.load(f)["results"][0]


# Pairs measured to behave as the mutation says, with nothing ignored (the P10 mutation never worked because it assumed instead of measuring):
CONSTANT_AND_COLLIDING = ("base_link", "lidar_link")                         # fixed joint, the lidar cylinder sits inside the chassis box, so it collides in every state (base_link/bl_caster_link stopped colliding with platform-sw #71)
COLLIDES_SOMETIMES = ("arm_profile_link", "openarm_left_ee_base_link")       # arm joints between them; collides in about 2 of 100 random states (9 of 400)
UNJUSTIFIED = ("openarm_left_link1", "openarm_left_link3")                   # the old sampled `Never` pair: arm joints between, no neighbour, not upstream's


def test_m03_1_every_disabled_pair_has_a_reason_that_can_be_shown_at_every_lift_stop():
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, "m03.json")
        done = run_validate("--samples", str(SAMPLES), "--json", out, timeout=2400)
        assert os.path.isfile(out), f"validate_acm.py produced no result (exit {done.returncode}):\n{done.stdout[-1500:]}\n{done.stderr[-1500:]}"
        with open(out) as f:
            summary = json.load(f)
    meta = load_yaml("acm/acm_generation.yaml")
    results = summary["results"]
    first = results[0] if results else {}
    status = "PASS" if summary["result"] == "PASS" else "FAIL"
    failing = {r["lift_stop"]: {"unjustified": r["unjustified"], "bad_labels": r["bad_labels"], "constant_transform_colliding": r["constant_transform"]["colliding"],
                              "cross_check_disagreements": r["constant_transform"]["cross_check_with_sampled_states"]["disagreements"],
                              "enabled_constant_pairs_colliding": r["enabled_constant_pairs_colliding"]}
               for r in results if r["result"] == "FAIL"}
    n, seed = summary["samples_per_lift_stop"], summary["seed"]
    write_sidecar("M03", "acm_justification", "ACM: every disabled pair has a reason that can be shown (upstream-declared, Adjacent, ConstantTransform); everything "
                  "else is enabled", status, {
        "claim": "every disabled pair is declared by upstream, a structural neighbour, or has no movable joint between its links and is clear in the one "
                 "evaluation that settles every configuration; every other pair is enabled. Nothing in the matrix rests on sampling",
        "disabled_pairs": first.get("disabled_pairs"), "enabled_pairs": first.get("enabled_pairs"),
        "by_construction": {"constant_transform_pairs": len(first.get("constant_transform", {}).get("pairs", [])),
                            "evaluated": first.get("constant_transform", {}).get("evaluated"),
                            "colliding": {r["lift_stop"]: r["constant_transform"]["colliding"] for r in results},
                            "cross_check_with_sampled_states": {r["lift_stop"]: r["constant_transform"]["cross_check_with_sampled_states"] for r in results},
                            "pairs_at_the_first_lift_stop": first.get("constant_transform", {}).get("pairs"),
                            "was_in_p12a": 77, "why_fewer": "a prismatic joint (lift_joint) between the base and the arms removes the pairs it separates from what a fixed transform certifies"},
        "unjustified": {r["lift_stop"]: r["unjustified"] for r in results}, "bad_labels": {r["lift_stop"]: r["bad_labels"] for r in results},
        "enabled_constant_pairs_colliding": {r["lift_stop"]: r["enabled_constant_pairs_colliding"] for r in results},
        "information_enabled_pairs_sampled": {
            "note": "INFORMATION, not a result: the enabled pairs found colliding in random joint states inside the limits, and how often. Sampling "
                    "falsifies and never certifies: a pair absent here may still collide. For A4 (reach and clearance)",
            "samples_per_lift_stop": n, "seed": seed,
            "per_lift_stop": {r["lift_stop"]: {"colliding_pairs": r["enabled_pairs_sampled"]["colliding_pairs"],
                                              "never_collided": r["enabled_pairs_sampled"]["never_collided"]} for r in results},
            "most_often_at_the_first_lift_stop": first.get("enabled_pairs_sampled", {}).get("most_often")},
        "method": "the reasons are recomputed from the URDF's kinematic tree and the SRDF; collision is checked through moveit_py with an allowed-collision "
                  "matrix of its own that allows everything except the pairs under test, so the SRDF's disabled list is never consulted",
        "acm_record": {k: meta[k] for k in ("method", "date", "base_plate", "lift_joint_position_m", "counts", "deterministic")},
        "failures": failing,
        "seconds_per_lift_stop": {r["lift_stop"]: r["seconds"] for r in results},
        "stop_condition": "A disabled pair that cannot be justified, or a constant-transform pair that collides, means the matrix is wrong: report the pair, "
                          "do not re-disable it on statistics" if failing else None,
    })
    assert summary["result"] == "PASS", "\n" + done.stdout[-3500:]
    assert [r["lift_stop"] for r in results] == list(STOPS)
    # 77 in P12A, 45 in P15; 58 with the column's placeholder boxes; 61 with the adapter plate (P28); 133 with the P-03 item 24 equipment (1 Nov);
    # 149 with the head camera body and its mount (1 Nov); 145 with the second E-stop on the adapter plate (8 Nov); 132 since the platform
    # pin 713574f (10 Nov): camera_link has no collision geometry since platform-sw #106
    assert all(len(r["constant_transform"]["pairs"]) == 132 for r in results)
    assert all(not r["constant_transform"]["cross_check_with_sampled_states"]["disagreements"] for r in results)


def test_m03_2_a_pair_disabled_without_a_reason_is_a_fail_there_is_no_warn_class():
    """The old claim, `Never`, on a pair that has arm joints between its links, is neither upstream's nor a neighbour: unjustified, whatever a sampler saw."""
    with tempfile.TemporaryDirectory() as tmp:
        done, result = validate_mutated(tmp, add=(UNJUSTIFIED, "Never"))
    assert done.returncode == 1, f"expected exit 1, got {done.returncode}:\n{done.stdout[-1200:]}\n{done.stderr[-800:]}"
    assert result["result"] == "FAIL"
    assert [tuple(u["pair"]) for u in result["unjustified"]] == [UNJUSTIFIED]
    assert f"FAIL: {UNJUSTIFIED[0]} <-> {UNJUSTIFIED[1]} is disabled with no reason" in done.stdout
    assert "WARN" not in done.stdout


def test_m03_3_a_label_that_is_not_true_and_a_constant_pair_that_collides_are_fails():
    with tempfile.TemporaryDirectory() as tmp:
        # a pair that collides SOMETIMES, disabled as if it had a constant transform: the label is false (arm joints lie between its links)
        done, sometimes = validate_mutated(tmp, add=(COLLIDES_SOMETIMES, "ConstantTransform"))
        assert done.returncode == 1 and sometimes["result"] == "FAIL", done.stdout[-1200:]
        assert [tuple(b["pair"]) for b in sometimes["bad_labels"]] == [COLLIDES_SOMETIMES] and "movable joint" in sometimes["bad_labels"][0]["problem"]
        # a constant pair that really collides (it is Adjacent by design), relabelled as clear-by-construction: the evaluation finds the collision
        done, constant = validate_mutated(tmp, relabel=(CONSTANT_AND_COLLIDING, "Adjacent", "ConstantTransform"))
        assert done.returncode == 1 and constant["result"] == "FAIL", done.stdout[-1200:]
        assert [tuple(p) for p in constant["constant_transform"]["colliding"]] == [CONSTANT_AND_COLLIDING]
        assert f"FAIL: {CONSTANT_AND_COLLIDING[0]} <-> {CONSTANT_AND_COLLIDING[1]} is disabled as ConstantTransform" in done.stdout


def test_m03_4_a_permanent_collision_that_is_not_disabled_is_a_fail():
    """Enable the pair that overlaps by design (constant transform, Adjacent): every state of the model is then invalid, and M03 must say so."""
    with tempfile.TemporaryDirectory() as tmp:
        done, result = validate_mutated(tmp, remove=CONSTANT_AND_COLLIDING)
    assert done.returncode == 1 and result["result"] == "FAIL", done.stdout[-1200:]
    assert [tuple(p) for p in result["enabled_constant_pairs_colliding"]] == [CONSTANT_AND_COLLIDING]
    assert "a permanent collision" in done.stdout


def test_m03_5_the_enabled_pairs_that_collide_are_information_and_never_change_the_result():
    """The sampled part reports the enabled pairs that collide in random states, and a clean matrix stays PASS whatever it finds."""
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, "m03.json")
        done = run_validate("--lift-stop", "L1000", "--samples", "150", "--json", out, timeout=300)
        with open(out) as f:
            result = json.load(f)["results"][0]
    assert done.returncode == 0 and result["result"] == "PASS"
    info = result["enabled_pairs_sampled"]
    assert info["colliding_pairs"] > 0 and info["colliding_pairs"] + info["never_collided"] == result["enabled_pairs"]["total"]
    assert all(row["collided_in_samples"] >= 1 and row["of"] == 150 for row in info["most_often"])
    assert "information, not a result" in done.stdout


def test_m03_6_bisection_finds_every_colliding_pair_and_calls_the_check_once_when_nothing_collides():
    pairs = [(f"a{i}", f"b{i}") for i in range(37)]
    for hits in (set(), {pairs[0]}, {pairs[36]}, {pairs[3], pairs[4], pairs[30]}, set(pairs)):
        calls = []

        def check(subset):
            calls.append(len(subset))
            return any(p in hits for p in subset)

        assert sorted(colliding_pairs(check, pairs)) == sorted(hits)
        if not hits:
            assert calls == [37]
