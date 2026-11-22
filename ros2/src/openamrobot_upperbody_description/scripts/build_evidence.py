#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Assemble the A6 evidence report. Called by verify_a6.sh after `colcon test` and check_urdf_matrix.py have run.

    build_evidence.py --since EPOCH [--profile dev|gazebo] [--step name=returncode ...] [--repo PATH]
                      [--results-dir PATH] [--bringup-results-dir PATH] [--gazebo-results-dir PATH]

Writes `<evidence dir>/<UTC timestamp>_<profile>.json` in the openamr.evidence.v1 shape of VERIFICATION.md and prints a
summary. Profile `dev` (default, scripts/verify_a6.sh) is the A6 report, evidence_class `static+mock`. Profile `gazebo`
(scripts/verify_gazebo.sh) is the Gazebo report, evidence_class `simulation`: G01 and G02 from the launch tests of
openamrobot_upperbody_gazebo, I01 for that package, and V01, which stays WARN until a person has answered the checklist. Exit code is 1 if any check is FAIL, 2 if the evidence cannot be assembled, and 3 if every check passed but the
working tree is dirty.

A dirty tree writes NO file (decided P09). A report records `git_hash`, and a tree with uncommitted changes is not that
commit, so a file written from one is an untrustworthy run lying next to the trustworthy ones, and "which one is the
record" would be something a reviewer has to know rather than something the file says. The summary is still printed so a
developer iterating before a commit sees every result; it is labelled as not recorded. The evidence directory therefore
only ever holds runs of a clean commit, with `dirty: false`.

Where each result comes from
  - pytest results (colcon's xunit files, from this package and from openamrobot_upperbody_bringup, whose launch test
    is L01) are mapped to a check ID by the `test_<id>_` prefix of the test function, else of
    its module. A failing or erroring test fails its ID.
  - a check's sidecar (`<ID>_<slug>.json` in the evidence directory, written by the test or by check_urdf_matrix.py) adds
    its details and can raise a WARN. Sidecars and xunit files older than --since are ignored, so a stale result from an
    earlier run is never mixed into this one.
  - a check with no test and no sidecar is NOT_RUN ("no result"), except those listed in NOT_IMPLEMENTED, which are SKIP
    with the reason: SKIP is a statement, a missing result is not.
  - NOT_RUN is a result value of its own (P12A): the check's backing test did not run to a result. It is not FAIL (nothing was observed to be
    wrong) and not SKIP (nobody said it would not run), it is never PASS, and it fails the exit status like FAIL does.
  - every test CMake registered for the packages a report covers (`ctest -N`) must have left a fresh result file, whatever its name says it
    is: a registered test with no result, a package with no tests registered, and a result file produced by ament's own `missing_result`
    stand-in (a test that timed out or died by a signal) are all listed by check H02 and make the checks they back NOT_RUN. Results whose test
    names carry a check ID this report does not list are not dropped: they land in AUX, and fail it.
Nothing here decides a PASS on its own: the report only combines what the tests recorded, and a PASS needs a test that ran behind it.
"""
import argparse
import datetime
import filecmp
import json
import os
import re
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict

import yaml

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import evidence_common as ev  # noqa: E402
import print_config_id as pci  # noqa: E402

REPO = "openamr-upperbody-sw"

# Every A6 check ID in VERIFICATION.md (T01-T16, L01, E01), in order, then the extra IDs this report also carries.
A6_CHECKS = [
    ("T01", "expansion matrix"),
    ("T02", "check_urdf accepts every T01 expansion"),
    ("T03", "single root, unique link names"),
    ("T04", "frame contract"),
    ("T05", "shoulder height per lift stop"),
    ("T06", "arg validation"),
    ("T07", "every link with an inertial or visual has a collision or is collision_free"),
    ("T08", "collision mesh is the same file as the visual mesh"),
    ("T09", "massed links: mass > 0, principal moments > 0, triangle inequality, tensor PD"),
    ("T10", "modelled mass by owner vs gross_mass_limit"),
    ("T11", "planar base joint absent by default, planar world -> base_footprint when enabled"),
    ("T12", "datum schema and provenance rules"),
    ("T13", "datum references; visualization constants"),
    ("T14", "mock profile has no OpenArmHW plugin"),
    ("T15", "gazebo profile uses GazeboSimSystem for arm joints"),
    ("T16", "real profile expands with OpenArmHW and its CAN parameters"),
    ("L01", "launch test: five controllers active on mock, one trajectory followed"),
    ("E01", "assembled envelope height, width, depth (lower bound)"),
]
EXTRA_CHECKS = [
    ("C01", "config id is sha256:<64 hex> and accepted by the C9 pattern"),
    ("C02", "upstream pin file agrees with the upstream checkout"),
    ("W01", "the root link has no inertial, so KDL consumers lose nothing"),
    ("W02", "KDL converts base_planar_joint to a fixed joint"),
    ("REV", "datums.yaml content change carries a revision change"),
    ("H01", "every test file is registered and every launch test checks that its processes started and exited cleanly"),
    ("H02", "every test CMake registered left a result file (nothing registered but not run)"),
    ("V01", "a person has looked at the model and answered the checklist"),
    ("I01", "installed package matches the source the tests and this report describe"),
    ("AUX", "supporting tests and steps (urdf_utils, datums_report, s4_sensors, evidence_dir, build_evidence, scripts)"),
]
# The `gazebo` profile's checks (scripts/verify_gazebo.sh). Everything static is already covered by the `dev` report.
GAZEBO_CHECKS = [
    ("G01", "combined robot in Gazebo: arm controllers, arm goal, Nav2-facing topics unchanged"),
    ("G02", "grippers reach their goals in Gazebo and finger_joint2 follows finger_joint1 in the physics"),
    ("H02", "every test CMake registered left a result file (nothing registered but not run)"),
    ("I01", "installed packages match the source the tests and this report describe"),
    ("V01", "a person has looked at the simulator and RViz and answered the checklist"),
    ("AUX", "supporting steps (colcon test of the gazebo package)"),
]
GAZEBO_PKG = "openamrobot_upperbody_gazebo"
PROFILES = {"dev": "static+mock", "gazebo": "simulation"}
# A check that has no test yet is listed here with the reason, and reports SKIP. Empty since P06 landed T14-T16 and L01.
# (C02 SKIPs by its own sidecar, only when there is no upstream checkout; that is a result, not a missing test.)
NOT_IMPLEMENTED = {}
BRINGUP_PKG = "openamrobot_upperbody_bringup"

# V01 after a person looked (P14; the architecture chat's V01_MAST_VISUAL finding, recorded in docs/v01_visual_check.md), updated by P15. It
# stays WARN, and the report says in words what the looking found and what has become of it. Two things about it are deliberate. It is not
# PASS: PASS needs every checklist item answered, and only item 1 (the mast, which no longer exists) has an answer on record. And it is not
# recorded as a check that found nothing: its value was that it found two defects no automated check was looking for.
# build_a7_evidence.py reads these constants, so the three reports cannot tell different stories.
V01_FINDINGS = [
    {"id": "V01-F1", "item": 1,
     "found": "the mast profile was probably rotated 90 degrees: mast_section [0.060, 0.120] put 60 mm fore-aft and 120 mm lateral; option B "
              "(P-03 rev18.9 item 18) orients the same HFS6-60120 cross-section (its 340 mm arm profile, not a full-height mast) 120 mm "
              "fore-aft and 60 mm lateral, the stiff direction against the arms' forward cantilever. The value was right. Its axis "
              "assignment was only a prose note in the datum ('orientation unconfirmed') and no test asserted it",
     "outcome": "confirmed correct by the design owner (Alex, 12 Oct 2026: 'thanks for catching the profile orientation, that's exactly "
                "the kind of check we need'); the 90 degree rotation was real",
     "resolved_by": "the mast and mast_section are deleted (P15); the rule it proposed is implemented: a dimension list declares "
                    "axis_convention and T12 asserts the assignment against the expansion"},
    {"id": "V01-F2", "item": 1,
     "found": "mast_mass was 3.0 kg (placeholder) for 1.2365 m of HFS6-60120: about 337 kg/m3 over the bounding box, low by roughly 2.5 to 4 "
              "times for an aluminium extrusion (an estimate, not a datasheet figure). Labelled placeholder, so nothing lied, but "
              "the placeholder was implausible and nothing checked plausibility",
     "outcome": "closed by the supplier catalogue figure: the lift is a DOLD Hexalift V4 350 of 11.7 kg (rev19.0, 12 Oct 2026). The "
                "estimate was 7.5 to 12 kg; the catalogue value is at the top of that bracket",
     "resolved_by": "mast_mass is deleted with the mast; lift_mass_dold_v4_350 carries 11.7 kg (provisional: a catalogue figure, not "
                    "weighed); the rule it proposed is implemented: a mass datum of ours declares a density_band and T10 checks it"},
]
V01_ANSWERED = {"1": "(item 1 as it stood in P14, about the mast) the mast read as a thin spike. Its dimensions were those of an HFS6-60120 "
                     "profile 14 mm shorter than the 1250 mm Alex specified on 28 Sep, so the appearance was an accurate thin extrusion "
                     "and not a sizing defect; its orientation and its mass were defects (V01-F1, V01-F2), both now closed. The mast is "
                     "gone: item 1 of the checklist now asks about the lift column and has no recorded answer"}
# The eight items of the lift model, measured in P16 (raw readings: evidence/v01/*.json; renders: evidence/renders/). Items 1 to 4 are read
# from the expanded URDF by forward kinematics (what tf2_echo prints), item 3 from MoveIt's collision check, items 5 to 8 from a headless
# Gazebo + MoveIt run. "cannot_measure" names what a person with RViz would have judged by eye and a script cannot.
V01_LIFT_MEASURED = {
    "1": {"verdict": "PASS", "expected": "base_footprint->lift_column_base_link x 0.060 z 0.304 (bp000), x 0.110 (bp050); openarm_left_base_link z 1.000 / 1.175 / 1.350, x and y unchanged",
          "measured": "x 0.060 z 0.304 (bp000), x 0.110 (bp050); left base z 1.000 / 1.175 / 1.350 at x 0.240 y +0.031 at all three stops",
          "note": "positions only: the frames did not move in P28 (the plate and the sections change geometry, not a mount, a stop or a frame; evidence/p28/v01_geometry.json). When this was measured (P16) neither column stage had a <visual> or a <collision>; the boxes are the DOLD datasheet's sections since P28"},
    "2": {"verdict": "PASS", "expected": "arm_profile_link 0.120 fore-aft x 0.060 across x 0.340, centre 0.180 ahead of the column axis; carriage->arm base x 0.180 y +-0.031 z 0.166 (62 mm apart)",
          "measured": "box 0.12 x 0.06 x 0.34, centre x 0.180 from the carriage origin; left base (0.180, +0.031, 0.166), right (0.180, -0.031, 0.166); long side is fore-aft",
          "note": "read from the URDF, not from RViz seen from above"},
    "3": {"verdict": "PASS", "expected": "0 enabled pairs colliding at each of three stops and both base plates",
          "measured": "P16 (no column boxes): 0 of 478 enabled pairs colliding at L1000, L1175, L1350 on bp000 and bp050. P28 (the boxes and the adapter plate): see evidence/p28/v01_geometry.json, enabled pairs colliding at the six configurations",
          "note": "counted on the collision geometry (MoveIt's check), which is the quantity the item names; the visual meshes of the arms are different files and were not intersected separately"},
    "4": {"verdict": "PASS", "expected": "head_camera_mount_link z 1.223 + lift, x 0.262 (bp000); optical axis 25 degrees down; wrist camera y +-0.05 z -0.03 from ee_base",
          "measured": "z 1.223 / 1.398 / 1.573, x 0.262 (0.312 at bp050); optical z axis elevation -25.0 degrees; wrist cameras (0, +-0.05, -0.03), 0.058 m from the hand base origin",
          "note": "whether 0.058 m is 'near the hand' is a judgement about the real hand and was not made; head_camera_mount_link carries no geometry"},
    "5": {"verdict": "PASS", "expected": "about 1.0 m and about 2.5 rad after 5 s each; carriage offset change 0.000",
          "measured": "distance 0.9995 m (odom), yaw 2.465 rad, base_footprint->lift_carriage_link unchanged (offset change 0.0 m)",
          "cannot_measure": "'RViz's model and Gazebo's model further apart than a wheel width' compares two windows by eye; the script's lookup of Gazebo's own base pose returned nothing, so no number stands in for it"},
    "6": {"verdict": "PASS", "expected": "joint1/2/4 at 0.3 / -0.6 / 1.2 within 0.02; right arm and wheels change < 0.001; largest single-step jump a fraction of the final value",
          "measured": "0.30001 / -0.60001 / 1.19995; right arm max change 1.2e-5, wheels 0.0; largest step in the first second 0.0015 / 0.0030 / 0.0060 rad",
          "note": "result error_code 0"},
    "7": {"verdict": "PASS", "expected": "finger_joint1 and finger_joint2 equal within 0.01; the right hand opposite in sign",
          "measured": "left 0.4008 / 0.4008, right -0.4000 / -0.4000 (/joint_states); physical link-pose angles left 0.4008 / 0.4002, right -0.4000 / -0.4000",
          "note": "/joint_states carries finger_joint2 as the URDF mimic of finger_joint1 and so cannot disagree with it: the link-pose angles are the physical reading (G02's method)"},
    "8": {"verdict": "PASS", "expected": "left/right hands_up joint4 1.5 within 0.01; scene robot and real end effector differ < 0.01 m; park equals home; grippers at the far end of each hand's range +0.7854 / -0.7854 and back at 0 (the far end was named `closed` when this was measured; it is `open` since 14 Nov 2026, the motion is the same)",
          "measured": "joint4 1.50037 (left), 1.49916 (right); goal-state vs real end-effector difference at most 0.0005 m over six arm poses; park == home in the SRDF for both arms; "
                      "closed 0.78457 / -0.78456, open within 0.001 of 0; 10 of 10 requests SUCCESS, no 'Planning failed' after home",
          "cannot_measure": "'the Scene Robot in RViz' is an RViz object; the goal state's end-effector (forward kinematics of the goal joint values) against the real one's tf stands in for it"},
}
V01_STAYS_WARN_BECAUSE = ("lift_mass_tl3_400 is null (no supplier mass for the fallback TL3 400 column), so one of the two columns the model offers cannot be "
                          "built; V01 stays WARN until that datum is sourced, however the eight items land. The eight below are measured on dold_v4_350 only")
V01_READING = ("The manual check found what the automated ones could not: no test of ours examined an axis assignment or the plausibility "
               "of a placeholder, and the two rules it proposed are now checks (T12, T10). The eight items of the lift model were measured in P16 "
               "(see answered_items: none FAIL, two carry a part that only eyes could judge). V01 stays WARN: " + V01_STAYS_WARN_BECAUSE)
V01_LIMITATION = ("V01 is WARN. P14's look at the mast found two defects (V01-F1 confirmed correct by the design owner, V01-F2 closed by the 11.7 kg catalogue figure), both resolved by the lift. P16 measured the eight items of the lift model "
                  "(evidence/v01/): none reads outside its expected value, and two sub-questions that need eyes are marked cannot_measure. It stays "
                  "WARN because " + V01_STAYS_WARN_BECAUSE + ". This report launches nothing for V01 and records no inspection of its own")


def v01_details(items, **extra):
    """The details of V01 in every report: what was measured, what was found, what is still open. `items` is the checklist's length."""
    answered = {k: dict(v) for k, v in V01_LIFT_MEASURED.items()}
    return {"performed": "P14: a person looked at the mast (item 1 of the old checklist, since deleted). P16: the eight items of the lift model "
                         "were measured by script (evidence/v01/), not looked at by a person",
            "answered_items": answered, "answered_before_the_lift": dict(V01_ANSWERED),
            "unanswered_items": [i for i in range(1, items + 1) if str(i) not in answered],
            "stays_warn_because": V01_STAYS_WARN_BECAUSE,
            "findings": [dict(f) for f in V01_FINDINGS], "reading": V01_READING, "checklist": "docs/v01_visual_check.md", **extra}
# Checks whose sidecar is where their details (and any WARN) live: no sidecar means the test did not complete.
SIDECAR_REQUIRED = {"T02", "T04", "T07", "T08", "T09", "T10", "T13", "E01", "C01", "C02", "W01", "W02", "G01", "G02"}
# Test modules with no check ID in their name.
MODULE_IDS = {"test_datums_revision": "REV"}
ID_IN_NAME = re.compile(r"^test_([a-z]\d{2})_")
EXPECTED_STEPS = {"dev": ("colcon_test", "check_urdf_matrix", "datums_report"), "gazebo": ("colcon_test",)}
REFUSED_DIRTY = 3          # exit status: every check passed, but the tree is dirty so no evidence file was written
NOT_RUN = "NOT_RUN"        # the check's backing test did not run to a result (P12A): never PASS, fails the exit status
FAILING = ("FAIL", NOT_RUN)
RESULT_ORDER = ("PASS", "WARN", "SKIP", "FAIL", NOT_RUN)
# The packages whose tests each profile runs (verify_a6.sh runs the first three, verify_gazebo.sh the gazebo package). The A7 builder
# (build_a7_evidence.py) passes its own list. Every one of them is read: a package that is run but not read is a place for failures to hide.
PROFILE_PACKAGES = {"dev": (ev.PKG, "openamrobot_upperbody_control", "openamrobot_upperbody_bringup"), "gazebo": ("openamrobot_upperbody_gazebo",)}


# --- mapping and combining (pure, tested in test_build_evidence.py) --------------------------------------------------

def check_id_of(classname, name):
    """Check ID for a pytest testcase: the `test_<id>_` prefix of the function, else of its module, else AUX."""
    for text in (name.split("[")[0], classname.rsplit(".", 1)[-1]):
        match = ID_IN_NAME.match(text)
        if match:
            return match.group(1).upper()
    return MODULE_IDS.get(classname.rsplit(".", 1)[-1], "AUX")


def read_xunit(results_dir, since):
    """({id: {tests, failures, skipped}}, [stale file names]) from colcon's xunit files."""
    counts = defaultdict(lambda: {"tests": 0, "failures": 0, "skipped": 0, "failed_tests": []})
    stale = []
    for filename in sorted(os.listdir(results_dir)) if os.path.isdir(results_dir) else []:
        if not filename.endswith(".xunit.xml"):
            continue
        path = os.path.join(results_dir, filename)
        if os.path.getmtime(path) < since:
            stale.append(filename)
            continue
        for case in ET.parse(path).getroot().iter("testcase"):
            cid = check_id_of(case.get("classname", ""), case.get("name", ""))
            row = counts[cid]
            row["tests"] += 1
            if case.find("failure") is not None or case.find("error") is not None:
                row["failures"] += 1
                row["failed_tests"].append(case.get("name"))
            elif case.find("skipped") is not None:
                row["skipped"] += 1
    return dict(counts), stale


def read_xunit_dirs(results_dirs, since):
    """read_xunit over several result directories; counts for the same ID add up."""
    merged, stale = {}, []
    for results_dir in results_dirs:
        counts, old = read_xunit(results_dir, since)
        stale += old
        for cid, row in counts.items():
            if cid in merged:
                for key in ("tests", "failures", "skipped"):
                    merged[cid][key] += row[key]
                merged[cid]["failed_tests"] += row["failed_tests"]
            else:
                merged[cid] = row
    return merged, stale


def combine(check_id, counts, sidecar, missing=None):
    """-> (result, details) for one check from its pytest counts (or None), its sidecar dict (or None) and the registered tests attributed to
    it that left no result (`missing`, names). A PASS needs a test that ran behind it; FAIL beats NOT_RUN beats everything else."""
    details = dict(sidecar["details"]) if sidecar else {}
    if counts:
        details = {"tests": counts["tests"], "failed": counts["failures"], "skipped": counts["skipped"], **details}
        if counts["failed_tests"]:
            details["failed_tests"] = counts["failed_tests"][:40]
    if missing:
        details["tests_without_a_result"] = list(missing)[:10]
    if counts and counts["failures"]:
        return "FAIL", details
    if sidecar and sidecar["result"] == "FAIL":
        return "FAIL", details
    if missing:
        return NOT_RUN, {**details, "reason": f"registered test(s) left no result: {', '.join(list(missing)[:3])}"}
    if not counts:
        if check_id in NOT_IMPLEMENTED:
            return "SKIP", {"reason": NOT_IMPLEMENTED[check_id]}
        why = "a sidecar exists but no test result backs it: the test that wrote it did not complete" if sidecar \
            else "no result: no test ran for this check ID"
        return NOT_RUN, {**details, "reason": why}
    if check_id in SIDECAR_REQUIRED and not sidecar:
        return NOT_RUN, {**details, "reason": "no sidecar: the test that records this check did not complete"}
    if counts["tests"] == counts["skipped"]:
        if sidecar and sidecar["result"] in ("WARN", "SKIP"):
            return sidecar["result"], details
        return "SKIP", details
    if sidecar and sidecar["result"] in ("WARN", "SKIP"):
        return sidecar["result"], details
    return "PASS", details


# --- accounting: what CMake registered against what left a result ------------------------------------------------------

def test_name_id(test_name):
    """Check ID of a ctest test by its name: `t04_frame_contract` and `test_test_l01_mock_controllers.py` -> T04, L01; else AUX."""
    name = test_name
    while name.startswith("test_"):
        name = name[5:]
    match = re.match(r"^([a-z]\d{2})_", name)
    if match:
        return match.group(1).upper()
    return MODULE_IDS.get("test_" + name) or MODULE_IDS.get(test_name) or "AUX"


def registered_tests(workspace, package):
    """Names of the tests CMake registered for the package (`ctest -N` in its build directory), or None if that cannot be read."""
    import subprocess
    build = os.path.join(workspace, "build", package)
    try:
        done = subprocess.run(["ctest", "--test-dir", build, "-N"], capture_output=True, text=True, timeout=120)
    except (OSError, subprocess.SubprocessError):
        return None
    if done.returncode != 0:
        return None
    return re.findall(r"^\s*Test\s+#\d+:\s+(\S+)\s*$", done.stdout, flags=re.M)


def result_status(results_dir, test, since):
    """`ok`, or why the test left no usable result: no_result_file, stale, empty, or did_not_complete (ament's own `missing_result`
    stand-in, written when a test times out or dies of a signal before it can write its file)."""
    path = os.path.join(results_dir, test + ".xunit.xml")
    if not os.path.isfile(path):
        return "no_result_file"
    if os.path.getmtime(path) < since:
        return "stale"
    try:
        cases = list(ET.parse(path).getroot().iter("testcase"))
    except ET.ParseError:
        return "unreadable"
    if not cases:
        return "empty"
    if any(c.get("name", "").endswith(".xunit.missing_result") for c in cases):
        return "did_not_complete"
    return "ok"


def accounting(workspace, packages, since):
    """{registered, with_a_result, problems[{package, test, status, id}]} over the packages' registered tests. A package with no registered
    test is itself a problem: a suite that vanished must not look like a suite that passed."""
    registered, problems = 0, []
    for package in packages:
        names = registered_tests(workspace, package)
        if not names:
            problems.append({"package": package, "test": None, "id": "AUX",
                             "status": "no registered tests (no ctest project in the build directory, or none registered)"})
            continue
        results_dir = os.path.join(workspace, "build", package, "test_results", package)
        for test in names:
            registered += 1
            status = result_status(results_dir, test, since)
            if status != "ok":
                problems.append({"package": package, "test": test, "status": status, "id": test_name_id(test)})
    return {"registered": registered, "with_a_result": registered - len([p for p in problems if p["test"]]), "problems": problems}


def fold_unclaimed(counts, claimed_ids):
    """Results whose ID is not one of the report's checks are not dropped: they are added to AUX (and returned by ID, for the report to say
    so). `counts` is changed in place."""
    unclaimed = {cid: counts.pop(cid) for cid in [c for c in counts if c not in claimed_ids]}
    if unclaimed:
        aux = counts.setdefault("AUX", {"tests": 0, "failures": 0, "skipped": 0, "failed_tests": []})
        for row in unclaimed.values():
            for key in ("tests", "failures", "skipped"):
                aux[key] += row[key]
            aux["failed_tests"] += row["failed_tests"]
    return unclaimed


def missing_by_id(problems, claimed_ids):
    """{check ID: [test names]} of the registered tests with no result; an ID the report does not list is attributed to AUX."""
    out = {}
    for p in problems:
        cid = p["id"] if p["id"] in claimed_ids else "AUX"
        out.setdefault(cid, []).append(p["test"] or p["package"])
    return out


# Names that carry no content: a scaffold placeholder and compiled caches. Anything else only in the install (a stale
# .xacro or .yaml that `$(find ...)` could still pick up) is a difference.
CONTENT_FREE = [".gitkeep", "__pycache__"]


def trees_differ(source, installed, names=("urdf", "config", "launch", "meshes")):
    """Relative paths that differ between the package source and its installed copy (empty = same)."""
    problems = []

    def walk(cmp, prefix):
        problems.extend(f"{prefix}{n}" for n in cmp.left_only + cmp.right_only + cmp.diff_files + cmp.funny_files)
        for name, sub in cmp.subdirs.items():
            walk(sub, f"{prefix}{name}/")

    for name in names:
        a, b = os.path.join(source, name), os.path.join(installed, name)
        if not os.path.isdir(a):
            continue
        if not os.path.isdir(b):
            problems.append(f"{name}/ (not installed)")
            continue
        walk(filecmp.dircmp(a, b, ignore=filecmp.DEFAULT_IGNORES + CONTENT_FREE), f"{name}/")
    return sorted(problems)


# --- reading the sidecars --------------------------------------------------------------------------------------------

def fresh_sidecars(directory, since):
    out = {}
    for doc in ev.read_sidecars(directory).values():
        if os.path.getmtime(os.path.join(directory, doc["file"])) >= since:
            out[doc["id"]] = doc
    return out


# --- the report ------------------------------------------------------------------------------------------------------

# The inputs of a config id, as the report states them (print_config_id.py is the definition). P15 changed the set while keeping the
# format; the old set is recorded so that a reader of an old id can tell it is not comparable.
CONFIG_ID_INPUTS = {
    "members": ["canonical_urdf", "column", "base_plate", "head_pitch", "base_tilt", "arm_package", "hardware_profile",
                "datums_revision", "upstream_pins"],
    "not_inputs": ["lift stop (L1000/L1175/L1350): a calibration ID", "enable_planar_base (already in canonical_urdf)"],
    "before_p15": ["canonical_urdf", "mast_id", "arm_package", "hardware_profile", "datums_revision", "openarm_description_commit"],  # decision-allow: CONFIG-IDS historical: config_id inputs before P15
    "note": "the format is unchanged (sha256:<64 hex>) and so is validity against the C9 schema; the meaning is not, so an id "
            "recorded before P15 is not comparable with one in this report",
}


def calibration_ids(upperbody):
    """The lift's calibration stops, each as the joint position it is: reported beside the config ids, never inside them."""
    column = upperbody["baseline"]["column"]
    return {stop: {"shoulder_height_m": height, "lift_joint_position_m": round(ev.lift_joint_position(upperbody, column, stop), 6),
                   "column": column}
            for stop, height in upperbody["lift_calibration_stops"].items()}


def limitations(datums, sidecars):
    placeholders = sorted(n for n, d in datums["datums"].items() if d["status"] == "placeholder")
    out = [
        f"placeholder datums: {len(placeholders)} of {len(datums['datums'])} ({', '.join(placeholders)})",
        "base inertials are platform-sw CAD-export placeholders, the lift mass is a catalogue figure (11.7 kg, provisional) on "
        "a placeholder box, and the arm masses are upstream nominal values; none of them is a measured mass",
        "T05 asserts the shoulder height at every lift stop (height(openarm_{left,right}_base_link) above the floor = "
        "lift_calibration_stops[stop]) and the three stops are calibration IDs, not inputs of the configuration id; it does not "
        "validate the datums themselves",
        "E01 is WARN, never PASS: height, width and depth are lower bounds from zero pose, link frame origins and "
        "box/cylinder/sphere extents; mesh extents are not included, so a figure under the limit proves nothing about the "
        "real envelope. A4 decides",
        "T10 reports what the description models. The battery, hub motors, electronics and the lift column's structure beyond "
        "its catalogue mass are not modelled, so the total is not the gross mass of the robot",
        "the lift column is two boxes, visual and collision (since b45d8f6; P28 changed their sizes): the outer (fixed) stage 240 x 60 mm and the "
        "inner (moving) stage 209.5 x 51 mm, both from the DOLD Hexalift datasheet page 4 (datums lift_column_section and lift_inner_section, "
        "provisional: read from a raster drawing, not from CAD). The boxes are not the supplier's shapes: the outer stage's collision is solid, so the "
        "two stages still overlap in volume (a hollow outer needs the housing wall, which the datasheet does not give), and the inner stage's length "
        "is the stroke, not a datasheet figure. The adapter plate (360 x 100 x 10 mm, Alex 22 Oct, P-03 rev19.6 item 18) and the arm profile "
        "(option B's 340 mm box) are the other collision bodies of the upper structure",
        "T14, T15 and T16 are expansion checks: nothing here loads the gazebo or the real plugin (the gazebo profile report, "
        "G01 and G02, is where the gazebo one runs), and the real profile cannot even be built here (openarm_hardware needs "
        "OpenArmCAN)",
        "the gripper controllers are the deprecated position_controllers/GripperActionController on purpose: the "
        "replacement serves ParallelGripperCommand instead of GripperCommand, which would decide manipulation#11 / "
        "ADR-009 point 4 (open)",
        "L01 launches mock.launch.py only: controller_manager on mock_components/GenericSystem, which reflects its command "
        "back as state, so it shows the controller wiring and not tracking performance",
        "T02 check_urdf parses the XML and builds the link tree; it does not resolve package:// mesh URIs",
        "W01: KDL consumers (robot_state_publisher, MoveIt in A7) drop the inertia of the root link; the root is base_footprint "
        "since platform-sw 935efc7 and carries none, so base_link's 4.18 kg is no longer the one dropped",
        "W02: with enable_planar_base:=true KDL consumers see world -> base_footprint as static; planar base motion can "
        "never come from robot_state_publisher (A7: it comes from Gazebo's odom -> base_footprint, ADR-003 / ADR-005 / REP 105)",
        V01_LIMITATION,
        "config ids are hashes of xacro's output text: a different xacro version that formats a number differently "
        "gives a different id for the same sources",
        "`static+mock` names the evidence class: expansions of every hardware_profile, plus a headless launch of the mock "
        "bringup (L01) and the robot_state_publisher observation for W02. Nothing in this report ran on Gazebo or real hardware",
        "no physical evidence; static expansion and zero-pose FK only",
    ]
    return out


def gazebo_limitations(datums, sidecars, simulator):
    """What the `gazebo` report does not say. The last line is the A6 report's own sentence about physical evidence, kept in
    its words on purpose: a simulation result permits no physical claim of any kind."""
    def engine_of(check_id):
        return (sidecars.get(check_id) or {}).get("details", {}).get("physics_engine_used", "unknown")
    return [
        f"`simulation` names the evidence class: a headless Gazebo Harmonic server ({simulator['gz_sim']}) with "
        f"gz_ros2_control {simulator['gz_ros2_control']}, run by launch tests: G01 on the physics engine {engine_of('G01')} "
        f"(the launch default), G02 on {engine_of('G02')} (passed explicitly). It is a statement about how the model behaves "
        "in that simulator and about nothing else",
        "the arms' <position> interface in gz_ros2_control is a velocity servo (position_proportional_gain 0.1 per step), so "
        "tracking to 0.02 rad says nothing about the real arm's controller, gains, compliance or latency",
        "arm, lift and base masses are placeholders, catalogue figures, upstream nominals or platform-sw's CAD export (T10); the base reaction "
        "to an arm motion, tracking and any torque or payload figure in a simulation are not claims about the robot",
        "the launch defaults to DART, the engine platform-sw tuned the base on; under DART neither gripper moves and "
        "finger_joint2 never follows finger_joint1, so G02 passes Bullet-Featherstone explicitly (it fails on DART, which is "
        "the documented limitation and not a defect). The base's contact behaviour was compared on neither engine; G01 "
        "measures kinematics (distance and yaw against the command)",
        "the base's wheel radius, separation and acceleration limits are the DiffDrive plugin's parameters, platform-sw's, "
        "and were not measured here; /odom here is the simulator's own odometry, not an estimate from sensing",
        "G01's rates and distances are properties of this simulator on this machine (headless, software rendering, real-time "
        "factor below 1); they are not requirements on the robot",
        "the gripper controller is still the deprecated position_controllers/GripperActionController on purpose "
        "(manipulation#11 / ADR-009 point 4, open): nothing here chooses between the gripper interfaces",
        V01_LIMITATION,
        "no physical evidence; simulation only. This report is not a statement about hardware",
    ]


def simulator_versions():
    """Versions of what the simulation ran on, read from the installed packages and the simulator itself."""
    import subprocess
    from ament_index_python.packages import get_package_share_directory
    import xml.etree.ElementTree as ET
    try:
        gz = subprocess.run(["gz", "sim", "--versions"], capture_output=True, text=True, timeout=30).stdout.strip() or "unknown"
    except (OSError, subprocess.SubprocessError):
        gz = "unknown"
    versions = {"gz_sim": gz}
    for package in ("gz_ros2_control", "ros_gz_sim"):
        try:
            root = ET.parse(os.path.join(get_package_share_directory(package), "package.xml")).getroot()
            versions[package] = root.findtext("version")
        except (OSError, LookupError, ET.ParseError):
            versions[package] = "unknown"
    return versions


def build(args):
    share = ev.share_dir()
    workspace = ev.workspace_root(share)
    edir = ev.evidence_dir(share)
    repo = args.repo or os.path.join(workspace, "src", REPO)
    pkg_src = os.path.join(repo, "ros2", "src", ev.PKG)
    results_dir = args.results_dir or os.path.join(workspace, "build", ev.PKG, "test_results", ev.PKG)
    steps = dict(s.split("=", 1) for s in args.step)

    profile = args.profile
    bringup_results_dir = args.bringup_results_dir or os.path.join(workspace, "build", BRINGUP_PKG, "test_results", BRINGUP_PKG)
    gazebo_results_dir = args.gazebo_results_dir or os.path.join(workspace, "build", GAZEBO_PKG, "test_results", GAZEBO_PKG)
    packages = PROFILE_PACKAGES[profile]
    result_dirs = [os.path.join(workspace, "build", p, "test_results", p) for p in packages]
    fixture = bool(args.results_dir or (profile == "gazebo" and args.gazebo_results_dir))        # the tests of this script point at fixtures
    if args.results_dir:
        result_dirs = [args.results_dir] + ([bringup_results_dir] if profile == "dev" else [])
    elif fixture:
        result_dirs = [args.gazebo_results_dir]
    counts, stale_xunit = read_xunit_dirs(result_dirs, args.since)
    accounted = accounting(workspace, packages, args.since) if not fixture else {"registered": 0, "with_a_result": 0, "problems": []}
    sidecars = fresh_sidecars(edir, args.since)
    check_list = (A6_CHECKS + EXTRA_CHECKS) if profile == "dev" else GAZEBO_CHECKS
    claimed = {i for i, _ in check_list}
    unclaimed = fold_unclaimed(counts, claimed)
    missing = missing_by_id(accounted["problems"], claimed)

    gazebo_src = os.path.join(repo, "ros2", "src", GAZEBO_PKG)
    gazebo_share = ev.package_share(GAZEBO_PKG) if profile == "gazebo" else None
    differ = trees_differ(pkg_src, share)
    gazebo_differ = trees_differ(gazebo_src, gazebo_share, names=("launch", "config", "worlds")) if gazebo_share else []
    checks = []
    for check_id, name in check_list:
        if check_id == "V01" and profile == "gazebo":
            result, details = "WARN", v01_details(8)
        elif check_id == "V01":
            result, details = "WARN", v01_details(8, last_launch="S4: robot_state_publisher + joint_state_publisher_gui + rviz2 started "
                                                                 "with no xacro or mesh errors (evidence/S4.json)")
        elif check_id == "I01" and profile == "gazebo":
            problems = sorted(gazebo_differ + [f"{ev.PKG}/{d}" for d in differ])
            result, details = ("FAIL", {"differing": problems, "hint": "run colcon build, then verify_gazebo.sh again"}) if problems \
                else ("PASS", {"compared": [f"{GAZEBO_PKG}: launch, config, worlds", f"{ev.PKG}: urdf, config, launch, meshes"]})
        elif check_id == "I01":
            result, details = ("FAIL", {"differing": differ, "hint": "run colcon build, then verify_a6.sh again"}) if differ \
                else ("PASS", {"compared": ["urdf", "config", "launch", "meshes"]})
        elif check_id == "H02":
            result, details = ("NOT_RUN", {"reason": f"{len(accounted['problems'])} registered test(s) left no usable result"
                                                     if accounted["problems"] else "no registered tests were found to account for",
                                           "problems": accounted["problems"][:20], "registered": accounted["registered"],
                                           "with_a_result": accounted["with_a_result"]}) if (accounted["problems"] or not accounted["registered"]) \
                else ("PASS", {"registered": accounted["registered"], "with_a_result": accounted["with_a_result"], "packages": list(packages)})
        elif check_id == "AUX" and profile == "gazebo":
            result, details = ("PASS", {}) if not counts.get("AUX") else combine("AUX", counts["AUX"], None, missing.get("AUX"))
        else:
            result, details = combine(check_id, counts.get(check_id), sidecars.get(check_id), missing.get(check_id))
        if check_id == "T02" and steps.get("check_urdf_matrix") not in (None, "0"):
            result, details["step"] = "FAIL", f"check_urdf_matrix.py exited {steps['check_urdf_matrix']}"
        if check_id == "AUX" and unclaimed:
            details["unclaimed_check_ids"] = {cid: {"tests": r["tests"], "failures": r["failures"]} for cid, r in unclaimed.items()}
        if check_id == "AUX":
            problems = {}
            for step in EXPECTED_STEPS[profile]:
                if steps.get(step) is None:
                    problems[step] = "step was not run"
                elif steps[step] != "0":
                    problems[step] = f"exited {steps[step]}"
            if stale_xunit:
                problems["stale_xunit"] = stale_xunit
            details["steps"] = steps
            if problems:
                result, details["step_problems"] = "FAIL", problems
        checks.append({"id": check_id, "name": (sidecars.get(check_id) or {}).get("name", name), "result": result,
                       "details": details})

    with open(os.path.join(share, "config", "datums.yaml")) as f:
        datums = yaml.safe_load(f)
    upperbody = ev.load_upperbody(share)
    try:
        pins, pin_sources = pci.pins_of(workspace)
        commit = pins.get("openarm_description")
        matrix = ev.expansion_matrix(upperbody) if profile == "dev" else \
            [r for r in ev.expansion_matrix(upperbody) + ev.profile_extras(upperbody) if r["hardware_profile"] == "gazebo"]
        config_ids = {ev.row_label(r): pci.config_id_of(share, pins, r) for r in matrix}
    except ev.EvidenceError as err:        # no pinned commit means no config id: say so in the report, do not guess
        commit, pins, pin_sources, config_ids = None, {}, {}, {}
        for c in checks:
            if c["id"] in ("C01", "AUX"):
                c["result"], c["details"]["config_ids"] = "FAIL", f"not produced: {err}"

    provenance = defaultdict(int)
    for d in datums["datums"].values():
        provenance[d["status"]] += 1
    dirty, dirty_files = ev.git_dirty(repo)
    upstream = {"openarm_description": commit or "unknown"}
    ros2_head = ev.git_head(ev.upstream_checkout(workspace, "openarm_ros2"))
    if ros2_head:
        upstream["openarm_ros2"] = ros2_head
    manipulation = ev.git_head(os.path.join(workspace, "src", "openamrobot-manipulation"))
    if manipulation:
        upstream["openamrobot-manipulation"] = manipulation[:7]
    platform = ev.git_head(os.path.join(workspace, "src", "openamr-platform-sw"))
    if platform:        # the base is included unchanged, so its commit is an input of every expansion (it is inside canonical_urdf)
        upstream["openamr-platform-sw"] = platform[:7]

    simulator = simulator_versions() if profile == "gazebo" else None
    now = datetime.datetime.now(datetime.timezone.utc)
    doc = {
        "schema": "openamr.evidence.v1",
        "repo": REPO,
        "git_hash": ev.git_head(repo),
        "dirty": dirty,
        "timestamp": now.strftime("%Y-%m-%dT%H:%M:%SZ"),
        "profile": profile,
        "evidence_class": PROFILES[profile],
        **({"simulator": simulator} if profile == "gazebo" else {}),
        "upstream": upstream,
        "datums_revision": datums["revision"],
        "checks": checks,
        "config_ids": config_ids,
        "config_id_inputs": CONFIG_ID_INPUTS,
        "calibration_ids": calibration_ids(upperbody),
        "provenance": {k: provenance[k] for k in ("placeholder", "provisional", "verified")},
        "limitations": (limitations(datums, sidecars) if profile == "dev" else gazebo_limitations(datums, sidecars, simulator)) + (
            [f"upstream pin {name} ({value[:7]}) was read from config/upstream_pins.yaml, not from a git checkout"
             for name, value in sorted(pins.items()) if pin_sources.get(name) == ev.METADATA_SOURCE]),
    }
    if dirty:
        doc["dirty_files"] = dirty_files        # for the console summary only: a dirty report is never written
        return doc, None, sidecars
    path = os.path.join(edir, now.strftime("%Y%m%dT%H%M%SZ") + f"_{profile}.json")
    with open(path, "w") as f:
        json.dump(doc, f, indent=2)
        f.write("\n")
    return doc, path, sidecars


def summary(doc, path, sidecars):
    lines = ["", f"{'Gazebo' if doc['profile'] == 'gazebo' else 'A6'} evidence  {doc['repo']} @ {doc['git_hash'][:7]}{' (dirty)' if doc['dirty'] else ''}  "
                 f"datums {doc['datums_revision']}  class {doc['evidence_class']}"]
    for c in doc["checks"]:
        extra = c["details"].get("reason") or ""
        if not extra and c["details"].get("failed_tests"):
            extra = "failing: " + ", ".join(c["details"]["failed_tests"][:3])
        tests = f"  [{c['details']['tests']} tests]" if "tests" in c["details"] else ""
        lines.append(f"  {c['result']:<4}  {c['id']:<4} {c['name'][:78]}{tests}{('  -- ' + extra[:90]) if extra else ''}")
    report = (sidecars.get("T10") or {}).get("details", {}).get("report")
    if report:
        lines += [""] + [f"  {line}" for line in report]
    constants = (sidecars.get("T13") or {}).get("details", {}).get("constants")
    if constants:
        lines += ["", "Visualization constants (DATUMS.md; visual-only, not datums):"]
        lines += [f"  {c['name']} = {c['value']:g}   in {c['file']}, {c['uses']} use(s)" for c in constants]
    lines += ["", "Config ids (full value is the id; a 12-hex prefix is shown for humans and is only a prefix)"]
    for label, cid in doc["config_ids"].items():
        lines.append(f"  {label:<48} {cid}   prefix {cid.split(':')[1][:12]}")
    if doc.get("calibration_ids"):
        lines += ["", "Calibration ids (joint positions with a calibration; NOT inputs of any config id)"]
        for stop, row in doc["calibration_ids"].items():
            lines.append(f"  {stop:<8} shoulder {row['shoulder_height_m']:.3f} m = lift_joint {row['lift_joint_position_m']:.3f} m")
    counts = defaultdict(int)
    for c in doc["checks"]:
        counts[c["result"]] += 1
    if path is None:
        files = doc["dirty_files"]
        lines += ["", f"Evidence: NOT WRITTEN. {doc['repo']} has uncommitted changes ({len(files)}: "
                      f"{', '.join(files[:5])}{', ...' if len(files) > 5 else ''}), so a report would describe no commit.",
                  "          Commit, then run this again. Nothing above is a record; the evidence directory only holds clean-tree runs."]
    else:
        lines += ["", f"Evidence: {path}"]
    bad = counts["FAIL"] + counts[NOT_RUN]
    lines += ["Result: " + ("FAIL" if counts["FAIL"] else ("NOT_RUN" if counts[NOT_RUN] else "no FAIL")) +
             "  (" + ", ".join(f"{counts[r]} {r}" for r in RESULT_ORDER) + ")"]
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--since", type=float, help="epoch seconds: results older than this are stale (required to build)")
    parser.add_argument("--clear", action="store_true",
                        help="delete the previous run's sidecars of the chosen profile (G* for gazebo, the rest for dev) from the evidence "
                             "directory, print it, and stop")
    parser.add_argument("--step", action="append", default=[], metavar="NAME=RC",
                        help="return code of a step verify_a6.sh / verify_gazebo.sh ran: colcon_test, check_urdf_matrix, datums_report")
    parser.add_argument("--profile", choices=sorted(PROFILES), default="dev",
                        help="dev: the A6 report (static+mock). gazebo: the Gazebo report (simulation)")
    parser.add_argument("--gazebo-results-dir", default=None,
                        help=f"colcon's xunit directory for {GAZEBO_PKG} (G01, G02); default <workspace>/build/...")
    parser.add_argument("--repo", default=None, help=f"the {REPO} checkout (default <workspace>/src/{REPO})")
    parser.add_argument("--results-dir", default=None, help="colcon's xunit directory for the package")
    parser.add_argument("--bringup-results-dir", default=None,
                        help=f"colcon's xunit directory for {BRINGUP_PKG} (L01); default <workspace>/build/...")
    args = parser.parse_args(argv)
    if args.clear:
        directory = ev.evidence_dir()
        ev.clear_sidecars(directory, only=(lambda f: f.startswith("G")) if args.profile == "gazebo"
                          else (lambda f: not f.startswith("G")))
        print(directory)
        return 0
    if args.since is None:
        parser.error("--since is required")
    try:
        doc, path, sidecars = build(args)
    except ev.EvidenceError as err:
        print(f"build_evidence: {err}", file=sys.stderr)
        return 2
    print(summary(doc, path, sidecars))
    if any(c["result"] in FAILING for c in doc["checks"]):
        return 1
    return REFUSED_DIRTY if path is None else 0


if __name__ == "__main__":
    sys.exit(main())
