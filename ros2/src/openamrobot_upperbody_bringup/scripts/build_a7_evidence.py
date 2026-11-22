#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Assemble the A7 evidence report. Called by run_a7_regression.sh after the tests ran.

    build_a7_evidence.py --since EPOCH [--step name=returncode ...] [--repo PATH]
    build_a7_evidence.py --clear

Writes `<evidence dir>/<UTC timestamp>_a7.json` in the openamr.evidence.v1 shape (VERIFICATION.md), `profile: "a7"`,
`evidence_class: "simulation"`: G01 and G02 (the Gazebo package's launch tests), M01 (once per lift stop), M02, M03, M04, the M05 and M06 checks
this step proposes, I01, V01 (still WARN: a person has to answer docs/v01_visual_check.md) and AUX. It reuses the A6 report's machinery
(build_evidence.py and evidence_common.py of the description package, installed next to it) and its rules:

  * a check with no result is NOT_RUN ("no result"), never a pass and never silently a skip; a skipped M01/M02 is no result. NOT_RUN fails the
    exit status like FAIL (build_evidence.py explains the value). Every test CMake registered must have left a result file (check H02), and
    results filed under an ID this report does not list land in AUX instead of being dropped
  * a dirty tree writes NO file (P08): a report records a commit, and a tree with uncommitted changes is no commit. The summary still prints,
    labelled as not recorded, and the exit status is 3 when nothing failed
  * sidecars and xunit files older than --since are ignored

Exit status: 1 if any check is FAIL, 3 if nothing failed but the tree is dirty (nothing recorded), 2 if the report cannot be assembled, else 0.
Everything in this report is `simulation`: it permits no statement about a physical robot, a payload, a torque, or a validated limit.
"""
import argparse
import datetime
import hashlib
import json
import os
import sys
from collections import defaultdict

from ament_index_python.packages import get_package_prefix, get_package_share_directory


def _description_scripts():
    path = os.path.join(get_package_prefix("openamrobot_upperbody_description"), "lib", "openamrobot_upperbody_description")
    if path not in sys.path:
        sys.path.insert(0, path)
    return path


_description_scripts()
import build_evidence as be  # noqa: E402
import evidence_common as ev  # noqa: E402

REPO = be.REPO
PROFILE = "a7"
EVIDENCE_CLASS = "simulation"
# Option B's lift calibration stops: joint positions with a calibration, NOT configuration IDs (they are not in the config id). M01 runs the
# mock stack once at each; M02 runs Gazebo once at the one the simulator's default state is (L1000: lift_joint at its zero).
LIFT_STOPS = ("L1000", "L1175", "L1350")
SIM_LIFT_STOP = "L1000"
A7_CHECKS = [
    ("G01", "combined robot in Gazebo: arm controllers, arm goal, Nav2-facing topics unchanged"),
    ("G02", "grippers reach their goals in Gazebo and finger_joint2 follows finger_joint1 in the physics (Bullet-Featherstone, passed explicitly)"),
    ("M01", "named poses plan and execute on mock at every lift calibration stop (L1000, L1175, L1350), the lift planned to each stop with both "
            "arms in park (S8, issue #17)"),
    ("M02", "named poses plan and execute in Gazebo at the baseline configuration (lift_joint at L1000), the lift planned to L1175 and back "
            "with both arms in park (S8, issue #17)"),
    ("M03", "ACM: every disabled pair has a reason that can be shown (upstream-declared, Adjacent, ConstantTransform); every other pair is enabled"),
    ("M04", "a goal inside the nearest reachable structure volume (arm_profile_link) is rejected by planning (the base body is out of the arms' reach)"),
    ("M05", "per-hand gripper values: open is each hand's own far limit, opposite signs, closed is 0 where the fingertips meet (ADR-009 point 5); the single-number and swapped-name versions fail"),
    ("M06", "named-pose plans need some acceleration limit and are insensitive to its value across three orders of magnitude (the limits stay placeholders)"),
    ("M07", "at rest in Gazebo no joint sits within the joint_rest_margin_rad datum of either of its limits (FAIL; WARN for the joints whose rest pose is an "
            "upstream limit; a non-finite reading is a FAIL; lift_joint and the mimic finger_joint2s are not measured, and say why)"),
    ("W04", "a gz server had to be force-stopped by the launch's stop helper (WARN; a server still running after that is the survivor check's FAIL, in the test that owns it)"),
    ("L01", "launch test: five controllers active on mock, one trajectory followed"),
    ("H01", "every test file is registered and every launch test checks that its processes started and exited cleanly"),
    ("H02", "every test CMake registered left a result file (nothing registered but not run)"),
    ("I01", "installed packages match the source the tests and this report describe"),
    ("V01", "a person has looked at the simulator and RViz and answered the checklist (eight items)"),
    ("AUX", "supporting tests and steps (SRDF, imported elements, ACM record, config files, harness, colcon test of the packages)"),
]
PACKAGES = {"gazebo": "openamrobot_upperbody_gazebo", "moveit": "openamrobot_upperbody_moveit", "bringup": "openamrobot_upperbody_bringup"}
EXPECTED_STEPS = ("colcon_test_gazebo", "colcon_test_moveit", "colcon_test_bringup")
# A check recorded by one sidecar per run, and the runs it must have. M01 is once per lift stop: the cut order allows fewer (with the reason
# recorded) through A7_M01_STOPS, and the report then says so.
SIDECAR_REQUIRED = {"G01", "G02", "M01", "M02", "M03", "M04", "M05", "M06", "M07"}
REFUSED_DIRTY = be.REFUSED_DIRTY


def expected_m01_stops():
    stops = tuple(m for m in os.environ.get("A7_M01_STOPS", ",".join(LIFT_STOPS)).split(",") if m)
    unknown = [m for m in stops if m not in LIFT_STOPS]
    if unknown:
        raise ev.EvidenceError(f"A7_M01_STOPS names {unknown}; accepted: {', '.join(LIFT_STOPS)}")
    return stops


def all_fresh_sidecars(directory, since):
    """{check id: [sidecar dicts]}: unlike evidence_common.read_sidecars, EVERY file of an id (M01 has one per lift stop)."""
    out = defaultdict(list)
    for filename in sorted(os.listdir(directory)):
        if ev.SIDECAR_RE.match(filename) and os.path.getmtime(os.path.join(directory, filename)) >= since:
            with open(os.path.join(directory, filename)) as f:
                doc = json.load(f)
            out[doc["id"]].append({**doc, "file": filename})
    return out


CLAMP_GATE_KEYS = ("excursions_beyond_epsilon", "largest_overshoot_rad", "largest_overshoot_joint", "epsilon_rad", "passes", "reasons")
CLAMP_DIAGNOSTIC_LABEL = ("DIAGNOSTIC ONLY: gates nothing. Activation and episode counts scatter 2-3x on an unchanged tree (P16), so they are "
                          "reported and are not a criterion")


def run_summary(details):
    clamp = details.get("clamp") or {}
    diagnostic = clamp.get("diagnostic_only") or {}
    lift = details.get("lift_scenario") or {}
    return {"poses_ok": details.get("poses_ok"), "poses_run": details.get("poses_run"), "worst_final_error_rad": details.get("worst_final_error_rad"),
            "lift_planned": details.get("lift_planned"),
            "lift_moves": {k: lift.get(k) for k in ("lift_moves_ok", "lift_moves_run", "moves_ok", "moves_run", "worst_lift_final_error_m")},
            "clamp": {"present": clamp.get("present"), "epsilon_rad": clamp.get("epsilon_rad"),
                      "gate": {k: (clamp.get("gate") or {}).get(k) for k in CLAMP_GATE_KEYS},
                      "diagnostic_only": {"label": CLAMP_DIAGNOSTIC_LABEL, **{k: diagnostic.get(k) for k in ("messages", "activations", "episodes")}}}}


def clamp_totals(runs):
    """The clamp across the runs of M01/M02. Two numbers are the verdict (`gate`): readings beyond epsilon and the largest overshoot magnitude.
    Activations and episodes are summed under `diagnostic_only`, labelled as such, and are never read by the verdict."""
    rows = [(name, r["clamp"]) for name, r in runs.items() if r.get("clamp")]
    present = [(n, c) for n, c in rows if c.get("present")]
    gates = [c["gate"] for _, c in present if c.get("gate")]
    excursions = sum(g["excursions_beyond_epsilon"] or 0 for g in gates)
    largest = max((g["largest_overshoot_rad"] or 0.0 for g in gates), default=None)
    epsilon = next((c["epsilon_rad"] for _, c in present if c.get("epsilon_rad") is not None), None)
    reasons = [r for g in gates for r in (g.get("reasons") or [])]
    return {"runs_with_a_clamp": len(present), "runs": len(rows),
            "gate": {"epsilon_rad": epsilon, "excursions_beyond_epsilon": excursions, "excursions_must_be": 0,
                     "largest_overshoot_rad": largest, "largest_overshoot_must_be_below_rad": epsilon,
                     "passes": bool(gates) and len(gates) == len(rows) and all(g.get("passes") for g in gates), "reasons": reasons},
            "diagnostic_only": {"label": CLAMP_DIAGNOSTIC_LABEL,
                                "activations": sum((c.get("diagnostic_only") or {}).get("activations") or 0 for _, c in present),
                                "episodes": sum((c.get("diagnostic_only") or {}).get("episodes") or 0 for _, c in present)}}


SIM_LAUNCH_CHECKS = ("G01", "G02", "M02", "M07")      # the launch tests that start a gz server and record the stop helper's report in their sidecars


# The states the stop helper's report can end in (sim.launch.py, stop_gz_server). W04 can only say something about the two that describe a server that was
# asked to stop: "stopped" (it went on request) and "force_stopped" (it did not). "no_server" is declared too, but it says the helper found no server to stop, so
# it shows nothing about a server stopping on request. Everything else, and above all "started" (the helper's first write: launch_guard.read_gz_stop_report
# returns it when its wait expires and the helper never recorded a result), is evidence that is not there.
GZ_STOP_STATE_PASS = "stopped"
GZ_STOP_STATE_WARN = "force_stopped"


def gz_force_stop_check(sidecars):
    """W04 from the sidecars of the launch tests: WARN if the stop helper had to signal a server in any of them (the server accepted the stop
    and did not go: a finding, not a failure of the run), PASS if every launch's helper recorded `stopped`, and NOT_RUN for anything less than a
    completed result: no sidecar, no helper report, a report that is not an object, one still in state `started` (the helper never finished: the
    wait for it expired), one with no state or an unrecognised one, or `no_server`. Incomplete, malformed or unrecognised evidence is never a PASS
    (the H01/H02 rule). A server that survived even its force-stop is not W04's business: the launch test's own survivor check FAILs it."""
    launches, forced, unread = {}, [], []
    for check_id in SIM_LAUNCH_CHECKS:
        docs = sidecars.get(check_id) or []
        if not docs:
            unread.append(f"{check_id}: no sidecar")
            continue
        helper = ((docs[0].get("details") or {}).get("gz_server_after_shutdown") or {}).get("stop_helper")
        if not helper:
            unread.append(f"{check_id}: no stop-helper report in the sidecar")
            continue
        if not isinstance(helper, dict):
            unread.append(f"{check_id}: the stop-helper report is not an object ({type(helper).__name__})")
            continue
        state = helper.get("state")
        if state == "started":
            unread.append(f"{check_id}: the stop helper never recorded a result (its report was still in state 'started' when the wait for it expired)")
            continue
        if state not in (GZ_STOP_STATE_PASS, GZ_STOP_STATE_WARN):
            unread.append(f"{check_id}: the stop-helper report has no completed state ({state!r}; W04 reads {GZ_STOP_STATE_PASS!r} and {GZ_STOP_STATE_WARN!r})")
            continue
        launches[check_id] = {k: helper.get(k) for k in ("state", "elapsed_s", "retried_with_service", "interrupt", "signals")}
        if state == GZ_STOP_STATE_WARN:
            forced.append(check_id)
    details = {"launches": launches, "forced": forced}
    if unread:
        return be.NOT_RUN, {**details, "reason": "; ".join(unread)}
    if forced:
        return "WARN", {**details, "problems": [f"the stop helper had to signal the gz server of {', '.join(forced)} (it accepted the stop request and "
                                                "did not go; SIGINT, the service request and SIGTERM were ignored or it was SIGKILLed); nothing was left running"]}
    return "PASS", details


def process_exits_of(sidecars):
    """[{"check", "sidecar", "processes": [{"process", "exit_code"}]}] for every launch whose sidecar carries `process_exits` (P24): the exit code of each
    process of each launch, as the launch test saw it after the launch stopped (launch_guard.process_exit_records). One entry per launch, in check-id and
    file order. A launch whose test did not record them (a sidecar without the key) is simply not here, so a rate computed from this list counts only
    launches that were recorded, and the report says how many that was by listing them; it is not a result of any check and cannot turn one into a PASS."""
    out = []
    for check_id in sorted(sidecars):
        for doc in sorted(sidecars[check_id] or [], key=lambda d: d.get("file") or ""):
            records = (doc.get("details") or {}).get("process_exits")
            if not isinstance(records, list):
                continue
            processes = [{"process": r.get("process"), "exit_code": r.get("exit_code")} for r in records if isinstance(r, dict)]
            out.append({"check": check_id, "sidecar": doc.get("file"), "processes": processes})
    return out


def combine_multi(check_id, counts, docs, expected_stops, missing=None):
    """-> (result, details) for a check recorded once per lift stop (M01) or once (M02, with its stop in the details)."""
    details = {}
    if counts:
        details = {"tests": counts["tests"], "failed": counts["failures"], "skipped": counts["skipped"]}
        if counts["failed_tests"]:
            details["failed_tests"] = counts["failed_tests"][:40]
    by_stop = {d["details"].get("lift_stop"): d for d in docs}
    runs, problems, no_result = {}, [], []
    if missing:
        details["tests_without_a_result"] = list(missing)[:40]
    for stop in expected_stops:
        doc = by_stop.get(stop)
        if doc is None:
            problems.append(f"no result for {stop}")
            no_result.append(stop)
            continue
        runs[stop] = {"result": doc["result"], "lift_joint_position_m": doc["details"].get("lift_joint_position_m"),
                      **run_summary(doc["details"])}
        if doc["result"] != "PASS":
            problems.append(f"{stop}: {doc['result']}")
    details["expected_lift_stops"] = list(expected_stops)
    details["runs"] = runs
    if check_id == "M01" and tuple(expected_stops) != LIFT_STOPS:
        details["reduced_set"] = f"the cut order reduced M01 to {', '.join(expected_stops)} (A7_M01_STOPS); the other lift stops were not run"
    # The clamp's counters are part of the result: no counters is no evidence that the joint state was only ever float noise away from a limit,
    # and a reading beyond the epsilon is something other than float noise moving a joint, which a PASS must not hide.
    totals = clamp_totals(runs)
    details["clamp_totals"] = totals
    for name, run in runs.items():
        if not (run.get("clamp") or {}).get("present"):
            problems.append(f"{name}: no joint-state clamp counters in the run")
    # M02 is gated on the two numbers and nothing else: a run with any excursion beyond epsilon, or a largest overshoot not below it, FAILS.
    # M01 (mock: no physics) keeps its older, weaker reading of the same number: WARN. The activation and episode counts gate neither.
    if check_id == "M02" and runs and not totals["gate"]["passes"]:
        problems.append("clamp gate: " + ("; ".join(totals["gate"]["reasons"]) or "no clamp numbers"))
    if counts and counts["failures"]:
        return "FAIL", {**details, "problems": problems or ["a test of this check failed"]}
    if any(doc["result"] == "FAIL" for doc in docs):
        return "FAIL", {**details, "problems": problems}
    if missing:
        return be.NOT_RUN, {**details, "problems": problems + [f"registered test(s) left no result: {', '.join(list(missing)[:3])}"]}
    if no_result:
        return be.NOT_RUN, {**details, "problems": problems}
    if problems:
        return "FAIL", {**details, "problems": problems}
    if details.get("reduced_set"):
        return "WARN", {**details, "problems": [details["reduced_set"]]}
    if totals["gate"]["excursions_beyond_epsilon"]:
        return "WARN", {**details, "problems": [f"the clamp met {totals['gate']['excursions_beyond_epsilon']} reading(s) beyond its epsilon (largest "
                                                f"{totals['gate']['largest_overshoot_rad']:.3g} rad): not float noise, passed through unclamped; the poses "
                                                f"still reached their goals, but look at why"]}
    return "PASS", details


def moveit_identity():
    """What the MoveIt configuration IS, as hashes: the SRDF (which carries the generated ACM) and the other config files, and the ACM's own
    fingerprint (the contract: the matrix as a set of pairs, whatever the file's order)."""
    share = get_package_share_directory("openamrobot_upperbody_moveit")
    import xml.etree.ElementTree as ET
    from openamrobot_upperbody_moveit.acm_split import acm_fingerprint, acm_pairs
    out = {"acm_fingerprint": acm_fingerprint(acm_pairs(ET.parse(os.path.join(share, "config", "openamrobot.srdf")).getroot()))}
    for name in ("openamrobot.srdf", "joint_limits.yaml", "moveit_controllers.yaml", "kinematics.yaml", "ompl_planning.yaml",
                 "pilz_cartesian_limits.yaml", "acm/acm_generation.yaml"):
        with open(os.path.join(share, "config", name), "rb") as f:
            out[name] = "sha256:" + hashlib.sha256(f.read()).hexdigest()
    return out


def limitations(sidecars):
    m03 = (sidecars.get("M03") or [{}])[0]
    m03_details = m03.get("details", {})
    out = [
        "`simulation` names the evidence class: every result here is how the model behaves in Gazebo Harmonic or on mock_components, planned by "
        "MoveIt 2.12.4 under OpenAMRobot's own declared limits. Every claim reads 'plans and executes in simulation under declared OpenAMRobot "
        "limits'; nothing here is a validated limit, a payload, a torque, or a statement about a physical robot",
        "the acceleration limits are engineering defaults (source: engineering_default, status: placeholder; rule: velocity / 2 s) written because "
        "upstream declares none (openamrobot-manipulation#10, ADR-009); M06 shows the plans need SOME limit and do not depend on its value",
        "the lift is a planned joint in the simulated profiles only (issue #17): mock and gazebo give lift_joint a position command interface and "
        "the DOLD V4 limits (800 N, 0.018 m/s, register LIFT), MoveIt plans it (group lift) and lift_controller follows it. M01 and M02 move it "
        "only between the calibration stops and only with both arms in park, as scenario design: it is not an interlock, and the real profile "
        "has no command interface for the lift (the lift node firmware owns it, CTL-017). The lift's no-load speed is a catalogue figure, its "
        "acceleration an engineering default; nothing here says how the real column moves under load. The three lift stops are calibration "
        "IDs, not configuration IDs",
        "the lift column is two boxes, visual and collision (since b45d8f6; P28 changed their sizes): the outer (fixed) stage 240 x 60 mm and the "
        "inner (moving) stage 209.5 x 51 mm, from the DOLD Hexalift datasheet page 4 (datums lift_column_section, lift_inner_section; provisional, "
        "read from a raster drawing). The boxes are not the supplier's shapes: the outer collision is solid, so the stages still overlap in volume, "
        "and the inner stage's length is the stroke, not a datasheet figure. The adapter plate (360 x 100 x 10 mm, Alex, P-03 rev19.6 item 18) and "
        "arm_profile_link, option B's 340 mm box, are the other collision bodies of the upper structure",
        "`park` is a placeholder: upstream's `home` with joint4 at 0.1 rad instead of on its lower limit 0 (issue #17), until the safe-pose "
        "definition lands under the manipulation action contract. Upstream's `home` (joint4 = 0) and the gripper `open` / `closed` states are "
        "still ON a joint limit: M05 and the import comparison (ADR-007) pin them there, and test_named_pose_margins.py lists them",
        "the gripper controller is still the deprecated position_controllers/GripperActionController serving GripperCommand (option 1 of "
        "ADR-009 point 4): this report does not choose between the three gripper interfaces (manipulation#11)",
        "G02 and M02 pass the Bullet-Featherstone engine explicitly; sim.launch.py defaults to DART, the engine platform-sw tuned the base on, on "
        "which an open gripper pose is not reached. The base's contact behaviour was compared on neither engine",
        "the arms' position interface in gz_ros2_control is a velocity servo, so a final error under 0.01 rad in M02 says nothing about the real "
        "arm's controller, gains, compliance or latency",
        "MoveIt 2.12.4 refuses a start state any amount outside a joint limit ('Joint ... from the starting state is outside bounds by'), and a "
        "simulated joint resting on its limit (openarm_*_joint4 = 0, the home pose) reads back a hair past it. The joint state move_group plans "
        "from is therefore clamped to the limit by joint_state_clamp.py, by at most its epsilon, and only that topic: no limit was widened and no "
        "pose moved. The clamp's counters are in M01/M02 (clamp_totals) and a reading beyond the epsilon is passed through, never clamped",
        "move_group exits with SIGSEGV when it is stopped (MoveIt 2.12.4); the checks allow that code for that one process and no other",
        "the arms cannot reach the platform's base body: M04's refused goal is inside arm_profile_link's volume, the nearest structure volume a "
        "left gripper can be put in; the arms are 62 mm apart at their bases (option B's y = +-0.031) and the left gripper reaches the right "
        "arm's own links in some random states, which is a reach fact for A4",
        "the collision matrix is DERIVED, not sampled (scripts/derive_acm.py): a pair is disabled only if upstream declares it, it is a structural "
        "neighbour (Adjacent), or no movable joint lies between its links and it is clear in the one evaluation that settles every configuration "
        "(ConstantTransform); every other pair is enabled. The Setup Assistant's sampler, which has no seed and gave a different matrix on every "
        "run, is no longer used. The checked-in SRDF is the contract and M03 validates that file",
        "M03's sampled part is information about the ENABLED pairs, never a reason: sampling falsifies and does not certify. It lists the enabled pairs "
        "that collide in random states and how often (for A4); nothing in the matrix rests on it",
        be.V01_LIMITATION,
        "no physical evidence; simulation only. This report is not a statement about hardware",
    ]
    if m03.get("result") == "FAIL":
        failing = m03_details.get("failures", {})
        pairs = sorted({f"{p['pair'][0]} <-> {p['pair'][1]}" for item in failing.values() for key in ("unjustified", "bad_labels") for p in item.get(key, [])}
                       | {f"{p[0]} <-> {p[1]}" for item in failing.values() for key in ("constant_transform_colliding", "enabled_constant_pairs_colliding")
                          for p in item.get(key, [])})
        out.insert(0, "M03 FAILS: the collision matrix is wrong for " + ", ".join(pairs) + ". A disabled pair with no reason that can be shown, a false label, "
                      "a constant pair that collides, or a permanent collision that is enabled: the pair is the finding, and it is not re-disabled on statistics")
    return out


def build(args):
    share = ev.share_dir()
    workspace = ev.workspace_root(share)
    edir = ev.evidence_dir(share)
    repo = args.repo or os.path.join(workspace, "src", REPO)
    steps = dict(s.split("=", 1) for s in args.step)
    result_dirs = [os.path.join(workspace, "build", p, "test_results", p) for p in PACKAGES.values()]
    counts, stale_xunit = be.read_xunit_dirs(result_dirs, args.since)
    sidecars = all_fresh_sidecars(edir, args.since)
    accounted = be.accounting(workspace, list(PACKAGES.values()), args.since)
    claimed = {i for i, _ in A7_CHECKS}
    unclaimed = be.fold_unclaimed(counts, claimed)
    missing = be.missing_by_id(accounted["problems"], claimed)

    source = lambda pkg: os.path.join(repo, "ros2", "src", pkg)
    differing = []
    for pkg, names in (("openamrobot_upperbody_description", ("urdf", "config", "launch", "meshes")),
                       ("openamrobot_upperbody_control", ("config",)),
                       ("openamrobot_upperbody_gazebo", ("launch", "config", "worlds")),
                       ("openamrobot_upperbody_moveit", ("launch", "config")),
                       ("openamrobot_upperbody_bringup", ("launch",))):
        differing += [f"{pkg}/{d}" for d in be.trees_differ(source(pkg), ev.package_share(pkg), names=names)]

    stops = expected_m01_stops()
    checks = []
    for check_id, name in A7_CHECKS:
        docs = sidecars.get(check_id, [])
        if check_id == "M01":
            result, details = combine_multi("M01", counts.get("M01"), docs, stops, missing.get("M01"))
        elif check_id == "M02":
            result, details = combine_multi("M02", counts.get("M02"), docs, (SIM_LIFT_STOP,), missing.get("M02"))
        elif check_id == "W04":
            result, details = gz_force_stop_check(sidecars)
        elif check_id == "V01":
            result, details = "WARN", be.v01_details(8)
        elif check_id == "I01":
            result, details = ("FAIL", {"differing": differing, "hint": "run colcon build, then run_a7_regression.sh again"}) if differing \
                else ("PASS", {"compared": ["description: urdf, config, launch, meshes", "control: config", "gazebo: launch, config, worlds",
                                            "moveit: launch, config", "bringup: launch"]})
        elif check_id == "H02":
            result, details = (be.NOT_RUN, {"reason": f"{len(accounted['problems'])} registered test(s) left no usable result"
                                                      if accounted["problems"] else "no registered tests were found to account for",
                                            "problems": accounted["problems"][:20], "registered": accounted["registered"],
                                            "with_a_result": accounted["with_a_result"]}) if (accounted["problems"] or not accounted["registered"]) \
                else ("PASS", {"registered": accounted["registered"], "with_a_result": accounted["with_a_result"],
                               "packages": list(PACKAGES.values())})
        elif check_id == "AUX":
            result, details = ("PASS", {}) if not counts.get("AUX") else be.combine("AUX", counts["AUX"], None, missing.get("AUX"))
        else:
            result, details = be.combine(check_id, counts.get(check_id), docs[0] if docs else None, missing.get(check_id))
            if check_id in SIDECAR_REQUIRED and not docs and result in ("PASS", "WARN", "SKIP"):
                result, details["reason"] = be.NOT_RUN, "no sidecar: the test that records this check did not complete"
        if check_id == "AUX":
            if unclaimed:
                details["unclaimed_check_ids"] = {cid: {"tests": r["tests"], "failures": r["failures"]} for cid, r in unclaimed.items()}
            problems = {}
            for step in EXPECTED_STEPS:
                if steps.get(step) is None:
                    problems[step] = "step was not run"
                elif steps[step] != "0":
                    problems[step] = f"exited {steps[step]} (the failing checks above say which test)"
            if stale_xunit:
                problems["stale_xunit"] = stale_xunit
            aux_counts = counts.get("AUX")
            if aux_counts:
                details.update({"tests": aux_counts["tests"], "failed": aux_counts["failures"], "skipped": aux_counts["skipped"]})
                if aux_counts["failures"]:
                    problems["failed_tests"] = aux_counts["failed_tests"][:40]
            details["steps"] = steps
            if problems:
                result, details["step_problems"] = "FAIL", problems
        checks.append({"id": check_id, "name": name, "result": result, "details": details})

    with open(os.path.join(share, "config", "datums.yaml")) as f:
        datums = yaml_load(f)
    upperbody = ev.load_upperbody(share)
    import print_config_id as pci
    pins, pin_sources = pci.pins_of(workspace)
    commit = pins["openarm_description"]
    matrix = [r for r in ev.expansion_matrix(upperbody) if r["arm_package"] == "openarm" and r["hardware_profile"] in ("mock", "gazebo")]
    config_ids = {ev.row_label(r): pci.config_id_of(share, pins, r) for r in matrix}
    provenance = defaultdict(int)
    for d in datums["datums"].values():
        provenance[d["status"]] += 1

    dirty, dirty_files = ev.git_dirty(repo)
    upstream = {"openarm_description": commit}
    for name in ("openarm_ros2",):
        head = ev.git_head(ev.upstream_checkout(workspace, name))
        if head:
            upstream[name] = head
    platform = ev.git_head(os.path.join(workspace, "src", "openamr-platform-sw"))
    if platform:        # the base is included unchanged: its commit is an input of every expansion (inside canonical_urdf)
        upstream["openamr-platform-sw"] = platform[:7]
    simulator = be.simulator_versions()
    try:
        import subprocess
        moveit_version = subprocess.run(["dpkg-query", "-W", "-f=${Version}", "ros-jazzy-moveit-core"], capture_output=True, text=True).stdout or "unknown"
    except OSError:
        moveit_version = "unknown"
    now = datetime.datetime.now(datetime.timezone.utc)
    doc = {
        "schema": "openamr.evidence.v1", "repo": REPO, "git_hash": ev.git_head(repo), "dirty": dirty,
        "timestamp": now.strftime("%Y-%m-%dT%H:%M:%SZ"), "profile": PROFILE, "evidence_class": EVIDENCE_CLASS,
        "simulator": {**simulator, "moveit": moveit_version}, "upstream": upstream, "datums_revision": datums["revision"],
        "checks": checks, "config_ids": config_ids, "config_id_inputs": be.CONFIG_ID_INPUTS,
        "calibration_ids": be.calibration_ids(upperbody), "moveit_config": moveit_identity(),
        "provenance": {k: provenance[k] for k in ("placeholder", "provisional", "verified")},
        "limitations": limitations(sidecars),
        # What the joint-state clamp did, summed over M01/M02. Not a result of its own: a reading beyond the epsilon makes M01/M02 WARN.
        "clamp": {c["id"]: c["details"].get("clamp_totals") for c in checks if c["id"] in ("M01", "M02")},
        # P24: the exit code of every process of every recorded launch. A crash like parameter_bridge -11 at shutdown is data here, and a rate of it
        # derives from this list (tools/check_evidence_backing.py, kind launch_exits). Reports before P24 have no such key.
        "process_exits": process_exits_of(sidecars),
    }
    if dirty:
        doc["dirty_files"] = dirty_files
        return doc, None
    path = os.path.join(edir, now.strftime("%Y%m%dT%H%M%SZ") + f"_{PROFILE}.json")
    with open(path, "w") as f:
        json.dump(doc, f, indent=2)
        f.write("\n")
    return doc, path


def yaml_load(f):
    import yaml
    return yaml.safe_load(f)


def summary(doc, path):
    lines = ["", f"A7 evidence  {doc['repo']} @ {(doc['git_hash'] or '?')[:7]}{' (dirty)' if doc['dirty'] else ''}  class {doc['evidence_class']}  "
                 f"MoveIt {doc['simulator'].get('moveit')}"]
    for c in doc["checks"]:
        extra = (c["details"].get("problems") or [c["details"].get("reason") or ""])[0] if c["result"] in ("FAIL", "WARN", be.NOT_RUN) else ""
        if not extra and c["details"].get("failed_tests"):
            extra = "failing: " + ", ".join(c["details"]["failed_tests"][:3])
        tests = f"  [{c['details']['tests']} tests]" if "tests" in c["details"] else ""
        lines.append(f"  {c['result']:<4}  {c['id']:<4} {c['name'][:84]}{tests}{('  -- ' + str(extra)[:90]) if extra else ''}")
    counts = defaultdict(int)
    for c in doc["checks"]:
        counts[c["result"]] += 1
    lines += ["", "Limitations"] + [f"  - {text}" for text in doc["limitations"][:4]] + ["  - ... (all in the evidence file)"]
    if path is None:
        files = doc["dirty_files"]
        lines += ["", f"Evidence: NOT WRITTEN. {doc['repo']} has uncommitted changes ({len(files)}: {', '.join(files[:5])}{', ...' if len(files) > 5 else ''}), "
                      "so a report would describe no commit.",
                  "          Commit, then run this again. Nothing above is a record; the evidence directory only holds clean-tree runs."]
    else:
        lines += ["", f"Evidence: {path}"]
    lines += ["Result: " + ("FAIL" if counts["FAIL"] else ("NOT_RUN" if counts[be.NOT_RUN] else "no FAIL")) +
              "  (" + ", ".join(f"{counts[r]} {r}" for r in be.RESULT_ORDER) + ")"]
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--since", type=float, help="epoch seconds: results older than this are stale (required to build)")
    parser.add_argument("--clear", action="store_true", help="delete the previous run's G* and M* sidecars from the evidence directory, print it, and stop")
    parser.add_argument("--step", action="append", default=[], metavar="NAME=RC", help="return code of a step run_a7_regression.sh ran")
    parser.add_argument("--repo", default=None, help=f"the {REPO} checkout (default <workspace>/src/{REPO})")
    args = parser.parse_args(argv)
    if args.clear:
        directory = ev.evidence_dir()
        ev.clear_sidecars(directory, only=lambda filename: filename[0] in "GM")
        print(directory)
        return 0
    if args.since is None:
        parser.error("--since is required")
    try:
        doc, path = build(args)
    except ev.EvidenceError as err:
        print(f"build_a7_evidence: {err}", file=sys.stderr)
        return 2
    print(summary(doc, path))
    if any(c["result"] in be.FAILING for c in doc["checks"]):
        return 1
    return REFUSED_DIRTY if path is None else 0


if __name__ == "__main__":
    sys.exit(main())
