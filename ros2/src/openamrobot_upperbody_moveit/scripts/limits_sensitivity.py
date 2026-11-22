#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Do the named-pose plans depend on the VALUES of the acceleration limits we invented, or only need SOME values to exist? (ADR-009)

    ros2 run openamrobot_upperbody_moveit limits_sensitivity.py [--base-plate bp000] [--json OUT]

Plans every named pose of both arms and both grippers, one after the other, offline (moveit_py, no controllers, no simulator), under:

  * the limits as they are in config/joint_limits.yaml, with the acceleration scaling factor of the request set to 1.0, 0.1, 0.01 and
    0.001: the effective acceleration limits span a factor of 1000, down to a few thousandths of a rad/s^2;
  * the limits with `has_acceleration_limits: false` on every joint, which is what upstream's own file says (manipulation#10): MoveIt's
    time parameterisation has nothing to work with.

What it can show. If every plan succeeds under the four scalings, the plans NEED some acceleration limit and do not depend on how generous
ours are: only the time each plan takes changes. If they succeed with no limits at all, the invented values are not even needed. If they
fail without limits and succeed with them, the placeholder is load-bearing in the narrow sense that a value must exist (which ADR-009 says),
and the result says whether it is load-bearing in the stronger sense that the particular numbers matter. The A7 stop condition is the
stronger one: "planning succeeds only because the acceleration limits we invented are generous".

Exit status 1 when a plan fails under a tight scaling (the plans then DO depend on the values), else 0. The process leaves with os._exit:
moveit_py segfaults at teardown.
"""
import argparse
import json
import os
import sys
import time

_KEEP_ALIVE = []
SCALINGS = (1.0, 0.1, 0.01, 0.001)
VELOCITY_SCALING = 0.5
SIDES = ("left", "right")
ARM_SEQUENCE = ("hands_up", "home", "hands_up", "park")
GRIPPER_SEQUENCE = ("open", "closed")      # from 0 (`closed`) to the far end and back: the motion measured before the names were fixed


def make_moveit_py(base_plate, strip_acceleration_limits=False, srdf_file=None):
    from moveit.planning import MoveItPy
    from openamrobot_upperbody_moveit.moveit_config import moveit_config
    cfg = moveit_config(base_plate, "mock", srdf_file=srdf_file).to_dict()
    if strip_acceleration_limits:
        for entry in cfg["robot_description_planning"]["joint_limits"].values():
            entry["has_acceleration_limits"] = False
            entry["max_acceleration"] = 0.0
    cfg["planning_scene_monitor_options"] = {
        "name": "planning_scene_monitor", "robot_description": "robot_description", "joint_state_topic": "/joint_states",
        "attached_collision_object_topic": "/moveit_cpp/planning_scene_monitor",
        "publish_planning_scene_topic": "/moveit_cpp/publish_planning_scene",
        "monitored_planning_scene_topic": "/moveit_cpp/monitored_planning_scene", "wait_for_initial_state_timeout": 0.0}
    cfg["planning_pipelines"] = {"pipeline_names": ["ompl"]}
    cfg["plan_request_params"] = {"planning_attempts": 1, "planning_pipeline": "ompl", "max_velocity_scaling_factor": VELOCITY_SCALING,
                                  "max_acceleration_scaling_factor": 1.0, "planning_time": 5.0}
    moveit_py = MoveItPy(node_name="limits_sensitivity", config_dict=cfg)
    _KEEP_ALIVE.append(moveit_py)
    return moveit_py


def sequence():
    """[(group, [(pose, previous pose)])]: every pose planned from a different one, the first from `home` (arms) or `closed` (grippers)."""
    out = []
    for side in SIDES:
        out.append((f"{side}_arm", list(zip(ARM_SEQUENCE, ("home",) + ARM_SEQUENCE[:-1]))))
    for side in SIDES:
        out.append((f"{side}_gripper", list(zip(GRIPPER_SEQUENCE, ("closed",) + GRIPPER_SEQUENCE[:-1]))))
    return out


def plan_all(moveit_py, scaling):
    from moveit.planning import PlanRequestParameters
    # PlanRequestParameters does not pick up the node's plan_request_params here (it comes back with an empty pipeline and defaults), so
    # every field is set explicitly. planner_id RRTConnect is what the OMPL default for these groups resolves to (move_group's log).
    params = PlanRequestParameters(moveit_py, "plan_request_params")
    params.planning_pipeline = "ompl"
    params.planner_id = "RRTConnect"
    params.planning_time = 5.0
    params.planning_attempts = 1
    params.max_velocity_scaling_factor = VELOCITY_SCALING
    params.max_acceleration_scaling_factor = scaling
    rows = []
    for group, steps in sequence():
        component = moveit_py.get_planning_component(group)
        for pose, previous in steps:
            component.set_start_state(configuration_name=previous)
            component.set_goal_state(configuration_name=pose)
            started = time.monotonic()
            result = component.plan(single_plan_parameters=params)
            ok = result.error_code.val == 1
            rows.append({"group": group, "pose": pose, "from": previous, "ok": ok, "error_code": result.error_code.val,
                         "duration_s": round(result.trajectory.duration, 3) if ok else None,
                         "wall_s": round(time.monotonic() - started, 3)})
    return rows


def run(args):
    out = {"base_plate": args.base_plate, "velocity_scaling": VELOCITY_SCALING, "variants": []}
    moveit_py = make_moveit_py(args.base_plate)
    for scaling in SCALINGS:
        rows = plan_all(moveit_py, scaling)
        out["variants"].append({"variant": f"configured limits x {scaling:g}", "acceleration_scaling": scaling,
                                "plans": len(rows), "plans_ok": sum(r["ok"] for r in rows), "rows": rows,
                                "total_duration_s": round(sum(r["duration_s"] or 0 for r in rows), 2)})
    return out, moveit_py


def run_without_limits(args):
    moveit_py = make_moveit_py(args.base_plate, strip_acceleration_limits=True)
    rows = plan_all(moveit_py, 1.0)
    return {"variant": "no acceleration limits (what upstream's joint_limits.yaml says)", "acceleration_scaling": 1.0, "plans": len(rows),
            "plans_ok": sum(r["ok"] for r in rows), "rows": rows}


def print_summary(out):
    print(f"\nAcceleration-limit sensitivity of the named-pose plans at {out['base_plate']} (lift at 0) (velocity scaling {out['velocity_scaling']})")
    for v in out["variants"]:
        print(f"  {v['variant']:<58} {v['plans_ok']:>2}/{v['plans']} plans   total planned duration "
              f"{v.get('total_duration_s', '-')} s")
    nl = out.get("without_limits")
    if nl:
        print(f"  {nl['variant']:<58} {nl['plans_ok']:>2}/{nl['plans']} plans")
    print("  " + out["verdict"])
    if out.get("conclusion"):
        print("  " + out["conclusion"])


PLACEHOLDER_CLAUSE = ("It does NOT make the limits any less of a placeholder: they stay engineering defaults (velocity / 2 s; source "
                      "engineering_default, status placeholder; ADR-009) with no payload, torque or controller analysis behind them.")


def conclusion(out):
    """The conclusion in words, both halves, or it is an overclaim (VERIFICATION.md M06): what the sensitivity retires, and what it leaves."""
    if out["result"] == "FAIL":
        return ("The plans depend on the values we invented, so the A7 stop condition ('planning succeeds only because the acceleration limits "
                "we invented are generous') HAS occurred. " + PLACEHOLDER_CLAUSE)
    needs_some = out["without_limits"]["plans_ok"] < out["without_limits"]["plans"]
    first = ("Planning requires that SOME acceleration limits exist and is insensitive to our values across three orders of magnitude "
             "(x1 to x0.001), which retires the A7 stop condition 'planning succeeds only because the acceleration limits we invented are "
             "generous'." if needs_some else
             "Planning succeeds under every scaling and with no acceleration limits at all, so it does not even need our values, which retires "
             "the A7 stop condition 'planning succeeds only because the acceleration limits we invented are generous'.")
    return first + " " + PLACEHOLDER_CLAUSE


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--base-plate", default="bp000", choices=("bp000", "bp050"))
    parser.add_argument("--json", default=None)
    parser.add_argument("--worker", choices=("scaled", "none"), help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.worker:
        result = run(args)[0] if args.worker == "scaled" else run_without_limits(args)
        with open(args.json, "w") as f:
            json.dump(result, f)
        sys.stdout.flush()
        os._exit(0)
    import subprocess
    import tempfile
    results = {}
    with tempfile.TemporaryDirectory() as tmp:
        for worker in ("scaled", "none"):
            path = os.path.join(tmp, f"{worker}.json")
            done = subprocess.run([sys.executable, os.path.abspath(__file__), "--worker", worker, "--base-plate", args.base_plate, "--json", path],
                                  capture_output=True, text=True, env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})
            if done.returncode != 0 or not os.path.isfile(path):
                sys.stderr.write(done.stdout[-1500:] + done.stderr[-1500:])
                return 2
            with open(path) as f:
                results[worker] = json.load(f)
    out = results["scaled"]
    out["without_limits"] = results["none"]
    tight_fail = [v for v in out["variants"] if v["plans_ok"] != v["plans"]]
    if tight_fail:
        out["verdict"] = ("The plans DEPEND on the values: they fail when the configured limits are scaled to "
                          + ", ".join(f"x{v['acceleration_scaling']:g}" for v in tight_fail) + ". The placeholder is load-bearing.")
    elif out["without_limits"]["plans_ok"] == out["without_limits"]["plans"]:
        out["verdict"] = "The plans succeed under every scaling AND with no acceleration limits at all: the invented values are not even needed."
    else:
        out["verdict"] = ("The plans succeed under every scaling of the configured limits (x1000 range) and fail with none: they need SOME "
                          "acceleration limit to exist and do not depend on its value; only the planned durations change.")
    out["result"] = "FAIL" if tight_fail else "PASS"
    out["conclusion"] = conclusion(out)
    print_summary(out)
    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=1)
    return 1 if tight_fail else 0


if __name__ == "__main__":
    sys.exit(main())
