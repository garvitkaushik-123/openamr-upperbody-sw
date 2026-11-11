# SPDX-License-Identifier: MIT
"""Shared by the M01 and M02 launch tests: the named-pose sequence, the settle-and-measure step, and the per-run sidecar."""
import math
import os
import sys
import time

from openamrobot_upperbody_moveit.joint_state_clamp import EPSILON
from openamrobot_upperbody_moveit.moveit_config import srdf_path
from openamrobot_upperbody_moveit.named_poses import ARM_GROUP, GRIPPER_GROUP, GRIPPER_JOINT, SIDES, read_group_states

# M01 and M02 start whole stacks (M02 a Gazebo server), and `verify_a6.sh` runs this package's whole `colcon test`: so they run only when
# OPENAMR_A7=1, which run_a7_regression.sh sets. Otherwise they SKIP with this reason, and the A6 verification neither starts Gazebo nor
# depends on an A7 check. A skipped M01/M02 is never a result: the A7 report treats a missing sidecar as a FAIL.
A7_ENABLED = os.environ.get("OPENAMR_A7") == "1"
SKIP_REASON = ("A7 simulation check: set OPENAMR_A7=1 (scripts/run_a7_regression.sh does) so verify_a6.sh does not start it; "
               "tracking: openAMRobot/openamr-upperbody-sw#16")  # skip tracked in openAMRobot/openamr-upperbody-sw#16
TOLERANCE = 0.01            # rad: the final joint error every named pose must stay under (A7)
VELOCITY_SCALING = 0.5      # explicit in every request: the SRDF-side default (0.1) would only make the tests slower
ACCELERATION_SCALING = 0.5
SETTLE_SECONDS = 1.0        # of the node's clock after a pose is reported done, before the final joint state is read
# Arms: each pose is reached from a DIFFERENT pose (home -> hands_up -> home -> hands_up -> park), so no plan is a trivial
# start-equals-goal one. `park` is a placeholder copy of `home`; it is still a named pose that must plan and execute.
ARM_SEQUENCE = ("hands_up", "home", "hands_up", "park")
# Grippers: closed then open, per hand, each with that hand's own value from the SRDF (ADR-009 point 5).
GRIPPER_SEQUENCE = ("closed", "open")


# The lift (issue #17, S8 with the lift planned). lift_joint is a planned joint in the simulated profiles: MoveIt group `lift`, lift_controller.
# Every run moves it ONLY between the calibration stops (the SRDF's L1000 / L1175 / L1350) and ONLY with both arms in `park`: the register LIFT
# rule (lift motion only in the stowed or carry pose with the base stopped) as scenario design. It is not an interlock: on the robot the rule is
# the lift node firmware's (CTL-017), and nothing here is a safety function. A lift goal is not sent unless the arms READ `park` (within
# TOLERANCE) just before it; a run that cannot show that FAILs instead.
LIFT_GROUP = "lift"
BOTH_ARMS_GROUP = "both_arms"
PARK = "park"
LIFT_JOINT = "lift_joint"
LIFT_CONTROLLER = "lift_controller"
LIFT_STOPS = ("L1000", "L1175", "L1350")
LIFT_TOLERANCE_M = 0.002        # m: the final lift_joint error a lift move must stay under (A7 scenario, simulation only)
LIFT_RESULT_TIMEOUT_S = 300.0   # s for one lift move: 0.175 m at half the DOLD no-load speed is about 20 s of the node's clock


def lift_route(stop, start="L1000"):
    """The calibration stops the lift visits to reach `stop` from `start`, each one a planned move: never a stop it is already at, and from
    L1000 to L1000 the lift goes to L1175 and back, so every run plans and executes the lift at least twice (once up, once down)."""
    if stop not in LIFT_STOPS or start not in LIFT_STOPS:
        raise ValueError(f"lift stop {stop!r} / {start!r}: accepted {', '.join(LIFT_STOPS)}")
    return ["L1175", "L1000"] if stop == start == "L1000" else ([stop] if stop != start else [])


def description_scripts():
    from ament_index_python.packages import get_package_prefix
    path = os.path.join(get_package_prefix("openamrobot_upperbody_description"), "lib", "openamrobot_upperbody_description")
    if path not in sys.path:
        sys.path.insert(0, path)
    return path


def write_run_sidecar(check_id, slug, name, result, details):
    description_scripts()
    import evidence_common as ev
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory("openamrobot_upperbody_description")
    return ev.write_sidecar(check_id, slug, name, result, details, directory=ev.evidence_dir(share))


def named_pose_plan():
    """[(group, pose name, {joint: value})] in the order they are run: both arms, then both grippers."""
    states = read_group_states(open(srdf_path()).read())
    plan = []
    for side in SIDES:
        plan += [(ARM_GROUP[side], pose, states[(ARM_GROUP[side], pose)]) for pose in ARM_SEQUENCE]
    for side in SIDES:
        plan += [(GRIPPER_GROUP[side], pose, states[(GRIPPER_GROUP[side], pose)]) for pose in GRIPPER_SEQUENCE]
    return plan


def controlled_limits():
    """{joint: (lower, upper)} of the sixteen controlled joints, from the same expansion MoveIt's robot_description comes from."""
    import xacro
    from ament_index_python.packages import get_package_share_directory
    from openamrobot_upperbody_moveit.named_poses import GRIPPER_JOINT, urdf_joint_limits
    path = os.path.join(get_package_share_directory("openamrobot_upperbody_description"), "urdf", "combined_robot.xacro")
    limits = urdf_joint_limits(xacro.process_file(path, mappings={"hardware_profile": "mock"}).toxml())
    wanted = [f"openarm_{side}_joint{i}" for side in SIDES for i in range(1, 8)] + [GRIPPER_JOINT[s] for s in SIDES]
    return {j: limits[j] for j in wanted}


def run_sequence(client, plan, result_timeout=120.0, start_wait=60.0):
    """Plan and execute every pose through move_group; return (rows, failures, start). A row is one pose: the outcome, the largest joint
    error after settling, and whether it is under TOLERANCE. `start` records the wait for a valid start state (see
    MoveGroupClient.wait_inside_limits) and what it saw, so the start-up transient is on the record."""
    inside, start = start_gate(client, start_wait)
    if not inside:
        return [], [f"the joint states never stayed inside the URDF limits for 1 s within {start_wait:.0f} s: {start}"], start
    rows, failures = run_poses(client, plan, result_timeout)
    return rows, failures, start


def start_gate(client, start_wait=60.0):
    """(inside, record): wait for a valid start state (MoveGroupClient.wait_inside_limits) and say what was seen on the way."""
    inside, transient = client.wait_inside_limits(controlled_limits(), start_wait, tolerance=EPSILON)
    return inside, {"start_state_inside_limits": inside, "out_of_limit_readings_seen_while_waiting":
                    [{"joint": j, "value": v, "lower": lo, "upper": hi} for j, v, lo, hi in transient]}


def run_s8(client, stop, plan, result_timeout=120.0, start_wait=60.0):
    """S8 with the lift planned (issue #17), one run at calibration stop `stop`: the start gate, both arms to `park`, the lift planned from L1000
    to `stop` (lift_route), the named-pose sequence `plan` at that stop (unchanged: twelve poses, the arms end in `park`), then the lift planned
    back to L1000 when it is not there. -> (pose rows, lift record, failures, start). Lift goals are sent only with the arms reading `park`."""
    inside, start = start_gate(client, start_wait)
    if not inside:
        return [], lift_scenario_record([]), [f"the joint states never stayed inside the URDF limits for 1 s within {start_wait:.0f} s: {start}"], start
    moves, failures = [], []
    row, failure = park_both_arms(client)
    moves.append(row)
    if failure:
        return [], lift_scenario_record(moves), [failure], start
    lift_rows, failures = move_lift(client, lift_route(stop))
    moves += lift_rows
    if failures:
        return [], lift_scenario_record(moves), failures, start
    rows, failures = run_poses(client, plan, result_timeout)
    if stop != "L1000":
        back, back_failures = move_lift(client, ["L1000"])
        moves += back
        failures += back_failures
    return rows, lift_scenario_record(moves), failures, start


def run_poses(client, plan, result_timeout=120.0):
    """Plan and execute every pose of `plan` through move_group. -> (rows, failures)."""
    rows, failures = [], []
    for group, pose, goal in plan:
        outcome = client.move(group, goal, execute=True, velocity_scaling=VELOCITY_SCALING,
                              acceleration_scaling=ACCELERATION_SCALING, result_timeout=result_timeout)
        client.settle(SETTLE_SECONDS)
        error = client.max_error(goal) if all(j in client.joint_states for j in goal) else float("nan")
        ok = outcome.success and error < TOLERANCE
        rows.append({"group": group, "pose": pose, "goal": {j: round(v, 4) for j, v in goal.items()}, **outcome.as_dict(),
                     "final_max_joint_error_rad": round(error, 5), "under_tolerance": bool(error < TOLERANCE), "ok": ok})
        if not ok:
            failures.append(f"{group}/{pose}: {outcome.name}, final error {error:.4f} rad (limit {TOLERANCE})")
    return rows, failures


def park_goal():
    states = read_group_states(open(srdf_path()).read())
    return states[(BOTH_ARMS_GROUP, PARK)]


def lift_goal(stop):
    return read_group_states(open(srdf_path()).read())[(LIFT_GROUP, stop)]


def _lift_row(client, group, pose, goal, outcome, tolerance):
    client.settle(SETTLE_SECONDS)
    error = client.max_error(goal) if all(j in client.joint_states for j in goal) else float("nan")
    ok = outcome is not None and outcome.success and error < tolerance
    return {"group": group, "pose": pose, "goal": {j: round(v, 4) for j, v in goal.items()},
            **(outcome.as_dict() if outcome is not None else {"accepted": False, "error_name": "NOT_SENT"}),
            "final_max_joint_error": round(error, 5), "tolerance": tolerance, "under_tolerance": bool(error < tolerance), "ok": ok}


def park_both_arms(client):
    """Both arms to `park` in one plan (group both_arms). -> (row, failure or None)."""
    goal = park_goal()
    outcome = client.move(BOTH_ARMS_GROUP, goal, execute=True, velocity_scaling=VELOCITY_SCALING, acceleration_scaling=ACCELERATION_SCALING,
                          result_timeout=120.0)
    row = _lift_row(client, BOTH_ARMS_GROUP, PARK, goal, outcome, TOLERANCE)
    return row, None if row["ok"] else f"{BOTH_ARMS_GROUP}/{PARK}: {row['error_name']}, final error {row['final_max_joint_error']} (limit {TOLERANCE})"


LIFT_GUARD_STATUS_TOPIC = "/lift_guard/status"
LIFT_GUARD_WAIT_S = 30.0        # the guard activates lift_controller about 0.5 s after the arms settle in park (base hold, freshness)


def wait_for_lift_guard(client, seconds=LIFT_GUARD_WAIT_S):
    """(ok, last status message or None): spin `client`'s node until the lift guard (issue #44) reports OK on /lift_guard/status, i.e.
    lift_controller is active. Since #44 lift_controller is spawned inactive and rejects goals until the guard activates it, so a lift goal
    sent before that would be rejected. Subscribes once per client, with the guard's own QoS (transient local)."""
    from diagnostic_msgs.msg import DiagnosticStatus
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    if not hasattr(client, "_lift_guard_status"):
        client._lift_guard_status = [None]
        latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        client.node.create_subscription(DiagnosticStatus, LIFT_GUARD_STATUS_TOPIC,
                                        lambda msg: client._lift_guard_status.__setitem__(0, msg), latched)
    ok = client.spin_until(lambda: client._lift_guard_status[0] is not None
                           and client._lift_guard_status[0].level == DiagnosticStatus.OK, seconds)
    return ok, client._lift_guard_status[0]


def move_lift(client, stops):
    """Plan and execute the lift through `stops` (calibration stops), each only after the arms read `park` and the lift guard (issue #44)
    reports OK. -> (rows, failures). A lift goal whose precondition does not hold is not sent: its row says NOT_SENT and the run fails
    (scenario design; the guard mirrors the interlock in simulation, CTL-017 is the interlock)."""
    rows, failures, park = [], [], park_goal()
    for stop in stops:
        arms_error = client.max_error(park) if all(j in client.joint_states for j in park) else float("nan")
        goal = lift_goal(stop)
        if not arms_error < TOLERANCE:
            row = _lift_row(client, LIFT_GROUP, stop, goal, None, LIFT_TOLERANCE_M)
            row["arms_in_park_error_rad"] = round(arms_error, 5)
            rows.append(row)
            failures.append(f"{LIFT_GROUP}/{stop}: not sent, the arms are not in `park` (largest error {arms_error:.4f} rad)")
            break
        guard_ok, guard_status = wait_for_lift_guard(client)
        if not guard_ok:
            row = _lift_row(client, LIFT_GROUP, stop, goal, None, LIFT_TOLERANCE_M)
            row["arms_in_park_error_rad"] = round(arms_error, 5)
            row["lift_guard_status"] = None if guard_status is None else guard_status.message
            rows.append(row)
            failures.append(f"{LIFT_GROUP}/{stop}: not sent, the lift guard did not report OK within {LIFT_GUARD_WAIT_S:g} s "
                            f"(last status: {row['lift_guard_status']!r})")
            break
        outcome = client.move(LIFT_GROUP, goal, execute=True, velocity_scaling=VELOCITY_SCALING, acceleration_scaling=ACCELERATION_SCALING,
                              result_timeout=LIFT_RESULT_TIMEOUT_S)
        row = _lift_row(client, LIFT_GROUP, stop, goal, outcome, LIFT_TOLERANCE_M)
        row["arms_in_park_error_rad"] = round(arms_error, 5)
        rows.append(row)
        if not row["ok"]:
            failures.append(f"{LIFT_GROUP}/{stop}: {row['error_name']}, final error {row['final_max_joint_error']} m (limit {LIFT_TOLERANCE_M})")
            break
    return rows, failures


def lift_scenario_record(rows):
    """The lift part of a run, for the sidecar: every planned lift and both-arms move, and the counts."""
    lift = [r for r in rows if r["group"] == LIFT_GROUP]
    return {"rule": "lift moves only between the calibration stops, with both arms in `park` (register LIFT, as scenario design; on the robot "
                    "the lift node firmware enforces it, CTL-017: this is not an interlock)",
            "moves": rows, "lift_moves_run": len(lift), "lift_moves_ok": sum(1 for r in lift if r["ok"]),
            "moves_run": len(rows), "moves_ok": sum(1 for r in rows if r["ok"]),
            "worst_lift_final_error_m": max((r["final_max_joint_error"] for r in lift), default=None)}


CLAMP_DIAGNOSTIC_LABEL = ("DIAGNOSTIC ONLY: gates nothing. P16 measured 646-1054 activations inside one unchanged configuration, so a count "
                          "that scatters 2-3x on an unchanged tree is not a measurement and is not a criterion (DESIGN_LOG 2026-10-14)")


def clamp_gate(stats):
    """The TWO numbers M02 gates on, from the clamp's `get_stats` dict (or None when there was no clamp):
       1. excursions beyond epsilon: readings further past a limit than epsilon, passed through unclamped. Must be 0.
       2. the largest overshoot magnitude of the run, clamped or not. Must be below epsilon (1e-5 rad).
    Epsilon is the module's own constant: a clamp RUNNING with a wider one fails here, and no outcome widens it. Activation and episode counts are
    not in this function and gate nothing."""
    if stats is None:
        return {"passes": False, "reasons": ["no joint-state clamp counters in the run: no evidence about the overshoot at all"],
                "epsilon_rad": EPSILON, "excursions_beyond_epsilon": None, "largest_overshoot_rad": None}
    excursions = int(stats.get("over_epsilon", 0))
    largest = max(float(stats.get("max_magnitude_rad") or 0.0), float(stats.get("max_over_epsilon_rad") or 0.0))
    running = stats.get("epsilon_rad")
    reasons = []
    if excursions != 0:
        reasons.append(f"{excursions} reading(s) beyond epsilon {EPSILON:g} rad (must be 0)")
    if not largest < EPSILON:
        reasons.append(f"largest overshoot {largest:.3g} rad is not below epsilon {EPSILON:g} rad")
    if running is None or float(running) > EPSILON:
        reasons.append(f"the running clamp's epsilon is {running!r}, wider than the {EPSILON:g} rad it was set to")
    return {"passes": not reasons, "reasons": reasons, "epsilon_rad": EPSILON, "excursions_beyond_epsilon": excursions,
            "excursions_must_be": 0, "largest_overshoot_rad": largest, "largest_overshoot_must_be_below_rad": EPSILON,
            "largest_overshoot_joint": stats.get("max_over_epsilon_joint") if float(stats.get("max_over_epsilon_rad") or 0.0) > float(
                stats.get("max_magnitude_rad") or 0.0) else stats.get("max_magnitude_joint")}


def clamp_record(client):
    """What the joint-state clamp did during this run, for the sidecar. `gate` holds the two numbers a run is judged on (clamp_gate);
    `diagnostic_only` holds the counts that are reported and gate nothing, labelled as such. Or the statement that no clamp ran."""
    stats = client.clamp_stats()
    if stats is None:
        return {"present": False, "gate": clamp_gate(None),
                "note": "no /joint_state_clamp/get_stats: move_group planned from the raw /joint_states"}
    diagnostic = {k: stats[k] for k in ("messages", "activations", "episodes", "joints") if k in stats}
    return {"present": True, "epsilon_rad": stats.get("epsilon_rad"), "gate": clamp_gate(stats),
            "over_epsilon": stats.get("over_epsilon"), "max_magnitude_rad": stats.get("max_magnitude_rad"),
            "max_magnitude_joint": stats.get("max_magnitude_joint"), "max_over_epsilon_rad": stats.get("max_over_epsilon_rad"),
            "max_over_epsilon_joint": stats.get("max_over_epsilon_joint"),
            "diagnostic_only": {"label": CLAMP_DIAGNOSTIC_LABEL, **diagnostic},
            "note": "joint_state_clamp.py: a reading past a joint limit by at most epsilon_rad is published at the limit, to move_group only; "
                    "a reading further out is passed through and counted as over_epsilon. No limit was widened and no pose moved"}


ARM_CONTROLLERS = ("joint_state_broadcaster", "left_arm_controller", "right_arm_controller", "left_gripper_controller", "right_gripper_controller")
SIM_CONTROLLERS = ARM_CONTROLLERS + (LIFT_CONTROLLER,)      # what M01 and M02 wait for since issue #17: the five above and lift_controller
# (since issue #44 lift_controller is spawned inactive and turns active when the lift guard allows it, once the start-up move to park is done)
CONTROLLERS_ACTIVE_WITHIN_S = 120.0     # wall s from the call; the controller manager of a headless server is up in 10-25 s on this machine


def wait_for_controllers_active(node, timeout_s=CONTROLLERS_ACTIVE_WITHIN_S, names=ARM_CONTROLLERS, manager="/controller_manager"):
    """Block until every controller in `names` reports `active`, or `timeout_s` of wall time passes. -> (ok, {name: state}, waited_s).
    A pose sent before this is true is a test of the harness, not of the robot (a controller that is not active refuses or drops the goal),
    so a caller treats `ok` False as a FAIL of its own with the states in the message, and never sends the pose.

    One absolute deadline (PR #14 review): the service wait, each request, each spin and each sleep are capped by the time that is left, and
    an all-active answer counts only if it arrived before the deadline. Before, each request had a fresh 2 s and success was not checked
    against the deadline, so an answer at 1.5 s of a 1.0 s budget returned ok. test_a7_controller_deadline.py pins it."""
    import rclpy
    from controller_manager_msgs.srv import ListControllers
    started = time.monotonic()
    deadline = started + timeout_s

    def left():
        return deadline - time.monotonic()

    def waited():
        return round(time.monotonic() - started, 1)

    client = node.create_client(ListControllers, f"{manager}/list_controllers")
    states = {}
    try:
        if left() <= 0 or not client.wait_for_service(timeout_sec=max(0.0, left())):
            return False, states, waited()
        while left() > 0:
            future = client.call_async(ListControllers.Request())
            end = min(time.monotonic() + 2.0, deadline)
            while not future.done() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=max(0.0, min(0.05, end - time.monotonic())))
            if future.done() and future.result() is not None:
                answered_in_time = time.monotonic() <= deadline
                states = {c.name: c.state for c in future.result().controller}
                if answered_in_time and all(states.get(name) == "active" for name in names):
                    return True, states, waited()
            if left() > 0:
                time.sleep(min(0.25, left()))
        return False, states, waited()
    finally:
        node.destroy_client(client)


def expected_rest_joints(urdf_xml, not_measured):
    """(expected, also_not_measured) for M07, from the MODEL, not from what happened to be published (PR #14 review): every joint with limits
    that the model puts under ros2_control (what joint_state_broadcaster publishes on /arm/joint_states), minus the explicit exclusions in
    `not_measured`. A limited joint outside ros2_control (the mimic finger_joint2s) is returned in `also_not_measured` with its reason, so the
    report says so instead of the joint silently not appearing."""
    import xml.etree.ElementTree as ET
    root = ET.fromstring(urdf_xml)
    limited = {j.get("name") for j in root.iter("joint")
               if j.get("type") in ("revolute", "prismatic") and j.find("limit") is not None}
    controlled = {j.get("name") for system in root.iter("ros2_control") for j in system.findall("joint")}
    expected = sorted((limited & controlled) - set(not_measured))
    also = {name: "not under ros2_control (a mimic or undriven joint): joint_state_broadcaster does not publish it"
            for name in sorted(limited - controlled - set(not_measured))}
    return expected, also


# Joints whose rest pose is one of their upstream URDF limits by construction (`home` and `open` are 0, the lower or upper limit; evidence/m02_bisect/RESULTS.md).
# M07 reports them WARN with this reason when they rest within the margin of that limit, on either side, and FAIL when one is read further past it than the margin.
AT_UPSTREAM_LIMIT = {
    "openarm_left_finger_joint1": "rest pose at an upstream limit",
    "openarm_right_finger_joint1": "rest pose at an upstream limit",
    "openarm_left_joint4": "rest pose at an upstream limit",
    "openarm_right_joint4": "rest pose at an upstream limit",
}


INHERITED_LIMIT_RAD = 0.0       # the upstream limit all four joints rest on: finger_joint1 left [0, 0.7854] lower, right [-0.7854, 0] upper, joint4 [0, 2.4435] lower


def non_finite_kind(value):
    """'nan', '+inf' or '-inf' for a reading that is not a finite number, else None. Anything that is not a real number (None, a string) is 'nan'."""
    try:
        value = float(value)
    except (TypeError, ValueError):
        return "nan"
    if math.isfinite(value):
        return None
    return "nan" if math.isnan(value) else ("+inf" if value > 0 else "-inf")


def split_finite(readings):
    """(valid, invalid) for M07: `readings` is {joint: [raw positions in arrival order]}. `valid` keeps only the finite ones, in order; `invalid` is
    {joint: {"count", "by_kind": {"nan": n, ...}, "indices": [position in that joint's raw list ...]}} for every joint that had a non-finite
    reading, so the sidecar keeps what was rejected. Done BEFORE anything counts or takes an extremum: a NaN compares false with everything, so it
    can sit in a list of 50 readings and leave `min`, `max` and a distance-to-limit that look fine (P27, Mohammad's two reproductions)."""
    valid, invalid = {}, {}
    for joint, values in readings.items():
        keep = []
        for index, value in enumerate(values):
            kind = non_finite_kind(value)
            if kind is None:
                keep.append(float(value))
                continue
            entry = invalid.setdefault(joint, {"count": 0, "by_kind": {}, "indices": []})
            entry["count"] += 1
            entry["by_kind"][kind] = entry["by_kind"].get(kind, 0) + 1
            entry["indices"].append(index)
        valid[joint] = keep
    return valid, invalid


def coverage_problems(readings, expected, min_samples):
    """{"missing": [...], "too_few_samples": {joint: n}} for M07: an expected joint that never appeared, or appeared fewer than `min_samples`
    times in the window, means the window did not measure it. Empty lists/dicts mean complete coverage; anything else is a FAIL of the
    measurement (never a PASS, never a WARN). Only FINITE readings count (P27): a joint whose readings were all NaN has no valid reading, and
    49 finite readings plus one NaN is 49, fewer than 50. `readings` may be raw; the count is the same whether or not split_finite ran first."""
    valid, _ = split_finite(readings)
    missing = [j for j in expected if not valid.get(j)]
    short = {j: len(valid[j]) for j in expected if valid.get(j) and len(valid[j]) < min_samples}
    return {"missing": missing, "too_few_samples": short}


def rest_rows(readings, limits, margin):
    """[{joint, min, max, lower, upper, to_lower, to_upper, nearest, distance, within_margin}] for every joint with limits that was read.
    `readings` must already be finite (split_finite). `distance` is the smaller of (lowest reading - lower) and (upper - highest reading);
    negative means the joint was read past a limit."""
    rows = []
    for joint in sorted(readings):
        if joint not in limits or not readings[joint]:
            continue
        values = readings[joint]
        low, high = min(values), max(values)
        lower, upper = limits[joint]
        to_lower, to_upper = low - lower, upper - high
        nearest = "lower" if to_lower <= to_upper else "upper"
        distance = min(to_lower, to_upper)
        rows.append({"joint": joint, "readings": len(values), "min_rad": low, "max_rad": high, "lower_limit_rad": lower, "upper_limit_rad": upper,
                     "distance_to_lower_rad": to_lower, "distance_to_upper_rad": to_upper, "nearest_limit": nearest,
                     "distance_to_nearest_limit_rad": distance, "within_margin": bool(distance < margin)})
    return rows


def classify(rows, margin):
    """(failing, warned). A row within the margin of a limit is `failing`, except a joint of AT_UPSTREAM_LIMIT that is within the margin of its INHERITED
    upstream limit (INHERITED_LIMIT_RAD, the zero end where its rest pose sits by construction), which is `warned` while it is no more than `margin`
    BEYOND that limit (distance >= -margin; Alex's rule, point 4). Within the margin of the OTHER end of its range (a joint4 at 2.4435, a finger at
    0.7854), or more than `margin` past the inherited one, a listed joint is failing like any other."""
    failing, warned = [], []
    for r in rows:
        if not r["within_margin"]:
            continue
        nearest_value = r["lower_limit_rad"] if r["nearest_limit"] == "lower" else r["upper_limit_rad"]
        if r["joint"] in AT_UPSTREAM_LIMIT and abs(nearest_value - INHERITED_LIMIT_RAD) < 1e-9 and r["distance_to_nearest_limit_rad"] >= -margin:
            warned.append(r)
        else:
            failing.append(r)
    return failing, warned


def evaluate_m07(raw_readings, expected, limits, margin, min_samples, not_measured=()):
    """The whole M07 verdict from raw `/arm/joint_states` positions, as a pure function (the launch test records what it returns; P27).

    Order, and why: (1) split off the non-finite readings, (2) count coverage on the finite ones only, (3) take extrema and distances on the finite
    ones only, (4) classify. The result is FAIL when any expected joint was not covered by `min_samples` finite readings, when ANY reading of any
    joint was non-finite (a NaN or an infinity is a defect in the physics output whatever the other readings say, so it is never averaged away
    into a PASS or a WARN), when no joint with limits was read, or when a joint is failing under classify(); else WARN when a joint of
    AT_UPSTREAM_LIMIT is within the margin of its limit; else PASS.
    Returns {"result", "rows", "failing_joints", "warned_joints", "coverage", "invalid_readings", "raw_samples_per_joint",
    "valid_samples_per_joint", "problems"}."""
    valid, invalid = split_finite(raw_readings)
    coverage = coverage_problems(valid, expected, min_samples)
    # The joints M07 lists as not measured (`not_measured`, its NOT_MEASURED: lift_joint) are taken out BEFORE the margin is judged (owner
    # decision, 1 Nov 2026: since issue #17 lift_joint is on /arm/joint_states, resting on its lower stop by construction). Only the margin
    # judgement: their non-finite readings above still FAIL, and every other joint is judged exactly as before.
    excluded = sorted(j for j in valid if j in set(not_measured))
    rows = rest_rows({j: v for j, v in valid.items() if j not in set(not_measured)}, limits, margin)
    failing_rows, warned_rows = classify(rows, margin)
    incomplete = bool(coverage["missing"] or coverage["too_few_samples"])
    problems = []
    if invalid:
        problems.append("non-finite readings (FAIL): " + "; ".join(
            f"{j} {e['count']} ({', '.join(f'{k} x{n}' for k, n in sorted(e['by_kind'].items()))})" for j, e in sorted(invalid.items())))
    if incomplete:
        problems.append("incomplete joint-state coverage by finite readings (FAIL): missing " + ", ".join(coverage["missing"]) +
                        "; too few finite samples " + ", ".join(f"{j} ({n} of {min_samples})" for j, n in coverage["too_few_samples"].items()))
    if not rows:
        problems.append("no joint with limits has a finite reading (FAIL)")
    failing = [r["joint"] for r in failing_rows]
    warned = {r["joint"]: AT_UPSTREAM_LIMIT[r["joint"]] for r in warned_rows}
    if failing:
        problems.append("joints within the margin of a limit (FAIL): " + ", ".join(failing))
    if invalid or incomplete or not rows or failing:
        result = "FAIL"
    else:
        result = "WARN" if warned else "PASS"
        if warned:
            problems.append("joints at an upstream limit at rest (WARN): " + ", ".join(sorted(warned)))
    return {"result": result, "rows": rows, "failing_joints": failing, "warned_joints": warned, "coverage": coverage,
            "not_measured_excluded_from_margin": excluded,
            "invalid_readings": invalid, "raw_samples_per_joint": {j: len(raw_readings.get(j, [])) for j in expected},
            "valid_samples_per_joint": {j: len(valid.get(j, [])) for j in expected}, "problems": problems}


def stack_alive(proc_info):
    """Active phase, last test: no process of the stack died while the tests were running (launch_guard.assert_no_process_died)."""
    description_scripts()
    import launch_guard
    launch_guard.assert_no_process_died(proc_info)


def stack_healthy(proc_info, expected, simulator=False, sidecar=None):
    """After the launch: every process in `expected` started and every process exited cleanly, allowing move_group's known SIGSEGV at stop and,
    with the simulator, its documented shutdown exits (launch_guard.py has the list and the reasons). With the simulator, also that no `gz sim`
    carrying this test's own GZ_PARTITION survived the teardown (P16): looked at FIRST, so that a failed health check does not leave the
    server running and unrecorded; the finding goes into the sidecar `sidecar` = (check id, slug) names, if it exists."""
    description_scripts()
    import launch_guard
    report = None
    if simulator:
        report = launch_guard.gz_server_shutdown_report(os.environ["GZ_PARTITION"])
        if sidecar:
            import evidence_common as ev
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory("openamrobot_upperbody_description")
            ev.amend_sidecar(sidecar[0], sidecar[1], "gz_server_after_shutdown", report, fail=bool(report["survivors"]),
                             directory=ev.evidence_dir(share))
    record = None
    if sidecar:
        import evidence_common as ev
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("openamrobot_upperbody_description")

        def record(records):        # P24: each process's exit code, in the sidecar of the check whose launch it was; never changes the result
            ev.amend_sidecar(sidecar[0], sidecar[1], "process_exits", records, directory=ev.evidence_dir(share))
    try:
        launch_guard.assert_launch_healthy(proc_info, expected=expected, allowed=launch_guard.SIM_ALLOWED if simulator else launch_guard.MOVE_GROUP_ALLOWED,
                                           record=record)
    finally:
        if report is not None:
            assert not report["survivors"], launch_guard.gz_survivor_message(report)
