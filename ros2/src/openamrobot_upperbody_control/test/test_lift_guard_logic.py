# SPDX-License-Identifier: MIT
"""The lift guard's decision (openamrobot_upperbody_control/lift_guard.py, issue #44): unit tests with no ROS runtime.

The node in a running stack is covered by the launch tests the issue assigns to the black-box side; these pin the decision itself, every
condition and every way an input can be missing, stale or malformed, all of which must block the lift."""
import math
import os

import pytest

from openamrobot_upperbody_control import lift_guard as lg

HERE = os.path.dirname(os.path.abspath(__file__))
# The repository's SRDF, read from the source tree (this package does not depend on the moveit package; the file is the one move_group uses).
REPO_SRDF = os.path.join(HERE, os.pardir, os.pardir, "openamrobot_upperbody_moveit", "config", "openamrobot.srdf")

PARK = {side: {j: (0.1 if j.endswith("joint4") else 0.0) for j in lg.ARM_JOINTS[side]} for side in lg.SIDES}
HOME = {side: {j: 0.0 for j in lg.ARM_JOINTS[side]} for side in lg.SIDES}
T0 = 100.0


def srdf(states=None, groups=None):
    """A minimal SRDF: the two arm groups (or `groups`), and `states` as {name: {side: {joint: value}}} (default: park)."""
    groups = groups if groups is not None else {side: lg.ARM_JOINTS[side] for side in lg.SIDES}
    states = states if states is not None else {"park": PARK}
    parts = ['<robot name="openamrobot">']
    for side, joints in groups.items():
        parts.append(f'<group name="{lg.SRDF_GROUPS[side]}">' + "".join(f'<joint name="{j}"/>' for j in joints) + "</group>")
    for name, sides in states.items():
        for side, pose in sides.items():
            parts.append(f'<group_state name="{name}" group="{lg.SRDF_GROUPS[side]}">'
                         + "".join(f'<joint name="{j}" value="{v}"/>' for j, v in pose.items()) + "</group_state>")
    parts.append("</robot>")
    return "".join(parts)


def logic(config=None, poses=None):
    return lg.LiftGuardLogic(poses or {side: (PARK[side],) for side in lg.SIDES}, config)


def feed_arms(guard, now, pose=PARK, reference=None, velocity=0.0, sides=lg.SIDES):
    """Joint states and controller states for `sides`, at `pose`; the controllers' reference at `reference` (default: the pose)."""
    reference = reference or pose
    for side in sides:
        joints = lg.ARM_JOINTS[side]
        guard.on_joint_state(joints, [pose[side][j] for j in joints], now)
        guard.on_controller_state(side, joints, [reference[side][j] for j in joints], [velocity] * len(joints), now)


def feed_odom(guard, now, linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0)):
    guard.on_odometry(linear, angular, now)


def all_good(guard=None, now=T0):
    """A guard whose every input says 'safe', the base stopped since well before `now`."""
    guard = guard or logic()
    feed_odom(guard, now - 2.0 * lg.DEFAULTS["base_stopped_hold_s"])
    feed_odom(guard, now)
    feed_arms(guard, now)
    return guard


# --- the SRDF ---------------------------------------------------------------------------------------------------------------------------
def test_park_is_read_from_the_repository_srdf_exactly_as_the_issue_states_it():
    with open(REPO_SRDF) as f:
        poses = lg.stowed_poses_from_srdf(f.read(), ["park"])
    for side in lg.SIDES:
        assert poses[side] == (PARK[side],), "park: upstream's home with joint4 at 0.1 rad (openamrobot.srdf), and nothing else"


def test_the_srdf_is_refused_rather_than_guessed_from():
    good = {"park": PARK}
    cases = {
        "a state that is not there": (srdf(good), ["carry"]),
        "a state that leaves a joint out": (srdf({"park": {"left": {j: 0.0 for j in lg.ARM_JOINTS["left"][:-1]}, "right": PARK["right"]}}), ["park"]),
        "a value that is not a number": (srdf({"park": {"left": {**PARK["left"], "openarm_left_joint1": "x"}, "right": PARK["right"]}}), ["park"]),
        "a value that is not finite": (srdf({"park": {"left": {**PARK["left"], "openarm_left_joint1": "nan"}, "right": PARK["right"]}}), ["park"]),
        "a group with other joints": (srdf(good, {"left": lg.ARM_JOINTS["left"][:-1], "right": lg.ARM_JOINTS["right"]}), ["park"]),
        "a group missing": (srdf(good, {"left": lg.ARM_JOINTS["left"]}), ["park"]),
        "not XML": ("<robot", ["park"]),
    }
    for label, (text, states) in cases.items():
        with pytest.raises(lg.GuardConfigError):
            lg.stowed_poses_from_srdf(text, states)
            pytest.fail(label)


def test_every_named_stowed_state_is_a_pose_the_lift_may_move_in():
    carry = {side: {**PARK[side], lg.ARM_JOINTS[side][1]: 0.3} for side in lg.SIDES}
    poses = lg.stowed_poses_from_srdf(srdf({"park": PARK, "carry": carry}), ["park", "carry"])
    guard = lg.LiftGuardLogic(poses, {"stowed_states": ("park", "carry")})
    all_good(guard)
    feed_arms(guard, T0, pose=carry)
    assert guard.evaluate(T0) == ()


# --- the settings -----------------------------------------------------------------------------------------------------------------------
def test_the_defaults_are_the_provisional_values_the_interface_states():
    assert lg.validate_config({}) == {"stowed_states": ("park",), "stowed_tolerance": 0.01, "reference_velocity_max": 0.001,
                                      "base_linear_speed_max": 0.01, "base_angular_speed_max": 0.01, "base_stopped_hold_s": 0.5,
                                      "state_timeout_s": 0.5}


@pytest.mark.parametrize("bad", [{"stowed_tolerance": 0.0}, {"state_timeout_s": -1.0}, {"base_stopped_hold_s": math.nan},
                                 {"base_linear_speed_max": math.inf}, {"reference_velocity_max": True}, {"stowed_tolerance": "0.01"},
                                 {"stowed_states": ()}, {"stowed_states": ("",)}, {"unknown_setting": 1.0}])
def test_a_setting_that_cannot_be_right_is_refused(bad):
    with pytest.raises(lg.GuardConfigError):
        lg.validate_config(bad)


def test_a_missing_stowed_pose_for_an_arm_is_refused():
    with pytest.raises(lg.GuardConfigError):
        lg.LiftGuardLogic({"left": (PARK["left"],), "right": ()})


# --- the decision: allowed --------------------------------------------------------------------------------------------------------------
def test_arms_in_park_and_the_base_stopped_long_enough_allow_the_lift():
    assert all_good().evaluate(T0) == ()


def test_on_the_tolerance_is_inside_it():
    guard = all_good()
    tol = lg.DEFAULTS["stowed_tolerance"]
    pose = {side: {j: v + (tol if j.endswith("joint1") else 0.0) for j, v in PARK[side].items()} for side in lg.SIDES}
    feed_arms(guard, T0, pose=pose)
    assert guard.evaluate(T0) == ()


# --- the decision: missing, stale, malformed --------------------------------------------------------------------------------------------
def test_nothing_received_blocks_with_missing_arm_state_and_missing_odometry():
    assert logic().evaluate(T0) == (lg.ARM_STATE_MISSING, lg.ODOM_MISSING)


def test_one_arm_joint_never_received_blocks():
    guard = logic()
    all_good(guard)
    guard._joints.pop("openarm_right_joint7")
    assert guard.evaluate(T0) == (lg.ARM_STATE_MISSING,)


def test_freshness_is_per_joint_so_other_joints_on_the_same_topic_do_not_keep_the_arm_fresh():
    """Mock: /joint_states also carries the base stub's wheel joints (moveit.launch.py:83). When the arm joints stop, the topic does not."""
    guard = all_good()
    later = T0 + 2.0 * lg.DEFAULTS["state_timeout_s"]
    guard.on_joint_state(["fl_wheel_joint", "fr_wheel_joint"], [0.0, 0.0], later)
    for side in lg.SIDES:
        joints = lg.ARM_JOINTS[side]
        guard.on_controller_state(side, joints, [PARK[side][j] for j in joints], [0.0] * 7, later)
    feed_odom(guard, later)
    assert guard.evaluate(later) == (lg.ARM_STATE_STALE,)


def test_a_stale_controller_state_blocks_even_with_fresh_joints():
    guard = all_good()
    later = T0 + 2.0 * lg.DEFAULTS["state_timeout_s"]
    for side in lg.SIDES:
        joints = lg.ARM_JOINTS[side]
        guard.on_joint_state(joints, [PARK[side][j] for j in joints], later)
    feed_odom(guard, later)
    assert guard.evaluate(later) == (lg.ARM_STATE_STALE,)


def test_a_clock_that_goes_backwards_counts_as_stale():
    guard = all_good()
    assert guard.evaluate(T0 - 1.0) == (lg.ARM_STATE_STALE, lg.ODOM_STALE)


def test_stale_odometry_blocks():
    guard = all_good()
    later = T0 + 2.0 * lg.DEFAULTS["state_timeout_s"]
    feed_arms(guard, later)
    assert guard.evaluate(later) == (lg.ODOM_STALE,)


@pytest.mark.parametrize("names, positions, velocities", [
    (lg.ARM_JOINTS["left"][:-1], [0.0] * 6, [0.0] * 6),          # a joint not covered
    (lg.ARM_JOINTS["left"], [0.0] * 6, [0.0] * 7),               # positions short
    (lg.ARM_JOINTS["left"], [0.0] * 7, []),                      # no reference velocities
])
def test_a_malformed_controller_state_blocks_as_missing(names, positions, velocities):
    guard = all_good()
    guard.on_controller_state("left", names, positions, velocities, T0)
    assert guard.evaluate(T0) == (lg.ARM_STATE_MISSING,)


# --- the decision: arms -----------------------------------------------------------------------------------------------------------------
def test_an_arm_out_of_park_blocks_and_names_the_arm():
    guard = all_good()
    out = {"left": {**PARK["left"], "openarm_left_joint2": 0.5}, "right": PARK["right"]}
    feed_arms(guard, T0, pose=out, reference=PARK)
    assert guard.evaluate(T0) == (lg.ARM_NOT_STOWED["left"],)


def test_the_mock_start_pose_home_is_not_stowed():
    """Upstream starts the mock arms at all zeros, and park has joint4 at 0.1 rad: the lift stays blocked until the arms reach park."""
    guard = all_good()
    feed_arms(guard, T0, pose=HOME)
    assert guard.evaluate(T0) == (lg.ARM_NOT_STOWED["left"], lg.ARM_NOT_STOWED["right"],
                                  lg.ARM_TRAJECTORY_ACTIVE["left"], lg.ARM_TRAJECTORY_ACTIVE["right"])


def test_a_non_finite_joint_position_is_not_stowed():
    guard = all_good()
    guard.on_joint_state(["openarm_right_joint3"], [math.nan], T0)
    assert guard.evaluate(T0) == (lg.ARM_NOT_STOWED["right"],)


def test_a_reference_away_from_park_is_an_active_trajectory_even_with_the_arm_still_in_park():
    """The first instants of a trajectory: the arm still reads park, the controller's reference has started to move (topic or action)."""
    guard = all_good()
    away = {"left": PARK["left"], "right": {**PARK["right"], "openarm_right_joint1": 0.05}}
    feed_arms(guard, T0, reference=away)
    assert guard.evaluate(T0) == (lg.ARM_TRAJECTORY_ACTIVE["right"],)


def test_a_moving_reference_is_an_active_trajectory_even_inside_the_tolerance():
    guard = all_good()
    feed_arms(guard, T0, velocity=2.0 * lg.DEFAULTS["reference_velocity_max"], sides=("left",))
    assert guard.evaluate(T0) == (lg.ARM_TRAJECTORY_ACTIVE["left"],)


@pytest.mark.parametrize("status", [1, 2, 3,          # ACCEPTED, EXECUTING, CANCELING
                                    0,                # UNKNOWN: "status has not been properly set", not terminal
                                    7, 127, -1])      # values action_msgs/GoalStatus does not define
def test_any_goal_status_but_a_terminal_one_is_an_active_trajectory(status):
    guard = all_good()
    guard.on_goal_statuses("left", [4, status])
    assert guard.evaluate(T0) == (lg.ARM_TRAJECTORY_ACTIVE["left"],)


@pytest.mark.parametrize("statuses", [[4], [5], [6], [4, 5, 6], []])
def test_only_terminal_goals_or_no_goals_do_not_block(statuses):
    """SUCCEEDED, CANCELED and ABORTED are the only states in which a goal can no longer move the arm. An empty list: no goal."""
    guard = all_good()
    guard.on_goal_statuses("right", statuses)
    assert guard.evaluate(T0) == ()


def test_the_terminal_set_is_exactly_succeeded_canceled_aborted():
    assert lg.TERMINAL_GOAL_STATUSES == frozenset((4, 5, 6))


# --- the decision: base -----------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("linear, angular", [((0.2, 0.0, 0.0), (0.0, 0.0, 0.0)),                     # Karthik's 0.2 m/s
                                             ((0.0, 0.0, 0.0), (0.0, 0.0, 0.3)),                     # turning on the spot
                                             ((lg.DEFAULTS["base_linear_speed_max"], 0.0, 0.0), (0.0, 0.0, 0.0)),   # the threshold is strict
                                             ((0.008, 0.008, 0.0), (0.0, 0.0, 0.0)),                 # the norm, not each axis
                                             ((math.nan, 0.0, 0.0), (0.0, 0.0, 0.0))])
def test_a_moving_base_blocks(linear, angular):
    guard = all_good()
    feed_odom(guard, T0, linear, angular)
    assert guard.evaluate(T0) == (lg.BASE_MOVING,)


def test_the_base_must_stay_stopped_for_the_hold_time_and_moving_restarts_it():
    hold = lg.DEFAULTS["base_stopped_hold_s"]
    guard = logic()
    feed_arms(guard, T0)
    feed_odom(guard, T0)
    assert guard.evaluate(T0) == (lg.BASE_MOVING,), "stopped, but not for long enough yet"
    t = T0 + hold
    feed_arms(guard, t)
    feed_odom(guard, t)
    assert guard.evaluate(t) == ()
    feed_odom(guard, t + 0.1, linear=(0.2, 0.0, 0.0))
    feed_odom(guard, t + 0.2)
    feed_arms(guard, t + 0.2)
    assert guard.evaluate(t + 0.2) == (lg.BASE_MOVING,), "a movement restarts the hold"


# --- the switch, the order, the status --------------------------------------------------------------------------------------------------
def test_a_failed_switch_is_reported_and_retried_rather_than_read_as_unsafe():
    guard = all_good()
    guard.switch_failed = True
    assert guard.evaluate(T0) == (lg.SWITCH_FAILED,)
    assert lg.wants_active((lg.SWITCH_FAILED,))
    assert not lg.wants_active((lg.SWITCH_FAILED, lg.BASE_MOVING))
    assert lg.wants_active(())


@pytest.mark.parametrize("codes, listed, pending, expected", [
    ((), "inactive", None, "active"),                      # safe, inactive, nothing pending: activate
    ((), "active", None, None),                            # already active
    ((), None, None, None),                                # unknown state: never activated
    ((), "unconfigured", None, None),
    ((), "inactive", "active", None),                      # an activation is already pending
    ((), "inactive", "inactive", None),                    # an activation waits for any pending request
    ((lg.SWITCH_FAILED,), "inactive", None, "active"),     # a failed switch is retried
    ((lg.BASE_MOVING,), "active", None, "inactive"),       # unsafe and active: deactivate at once
    ((lg.BASE_MOVING,), "inactive", "active", "inactive"),  # unsafe while an activation is pending: deactivate over it
    ((lg.BASE_MOVING,), "active", "inactive", None),       # a deactivation is already pending
    ((lg.BASE_MOVING,), "inactive", None, None),           # already inactive
    ((lg.BASE_MOVING,), None, None, None),                 # unknown: nothing to deactivate yet (it is spawned inactive)
])
def test_the_switch_asked_for(codes, listed, pending, expected):
    assert lg.switch_to_request(codes, listed, pending) == expected


def test_codes_come_in_one_fixed_order():
    guard = logic()
    feed_arms(guard, T0, pose=HOME)
    guard.on_goal_statuses("left", [2])
    feed_odom(guard, T0, linear=(1.0, 0.0, 0.0))
    guard.switch_failed = True
    codes = guard.evaluate(T0)
    assert codes == tuple(c for c in lg.CODE_ORDER if c in codes)
    assert codes == (lg.ARM_NOT_STOWED["left"], lg.ARM_NOT_STOWED["right"], lg.ARM_TRAJECTORY_ACTIVE["left"],
                     lg.ARM_TRAJECTORY_ACTIVE["right"], lg.BASE_MOVING, lg.SWITCH_FAILED)


def test_ok_only_when_nothing_blocks_and_lift_controller_is_listed_active():
    assert lg.build_status((), "active") == (lg.LEVEL_OK, "lift_guard", "lift allowed",
                                             [("lift_controller", "active"), ("blocking", "")])


@pytest.mark.parametrize("listed", ["inactive", None, "unconfigured", "finalized"])
def test_nothing_blocking_but_not_listed_active_is_warn_activating_with_an_empty_blocking(listed):
    """Accepted on #44: switching on is WARN, blocking empty, lift_controller "activating"."""
    level, _, _, values = lg.build_status((), listed)
    assert (level, values) == (lg.LEVEL_WARN, [("lift_controller", "activating"), ("blocking", "")])


@pytest.mark.parametrize("listed, reported", [("active", "active"), ("inactive", "inactive"), (None, "inactive")])
def test_error_always_means_blocked_and_always_names_a_code(listed, reported):
    """Accepted on #44: ERROR always means blocked. An outside activation the guard is undoing shows as ERROR with lift_controller active."""
    level, _, message, values = lg.build_status((lg.BASE_MOVING, lg.ODOM_STALE), listed)
    assert level == lg.LEVEL_ERROR and values == [("lift_controller", reported), ("blocking", "BASE_MOVING,ODOM_STALE")]
    assert "BASE_MOVING" in message


@pytest.mark.parametrize("codes", [(), (lg.BASE_MOVING,), (lg.SWITCH_FAILED,), tuple(lg.CODE_ORDER)])
@pytest.mark.parametrize("listed", ["active", "inactive", None])
def test_every_status_is_one_of_the_three_agreed_shapes(codes, listed):
    level, name, _, values = lg.build_status(codes, listed)
    state, blocking = dict(values)["lift_controller"], dict(values)["blocking"]
    assert name == "lift_guard"
    assert (level == lg.LEVEL_ERROR) == bool(blocking), "ERROR iff something blocks"
    if level == lg.LEVEL_OK:
        assert state == "active" and blocking == ""
    if level == lg.LEVEL_WARN:
        assert state == "activating" and blocking == ""


# --- the controller manager's answers ---------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("requested_after, requested_while, sent, switching_now, current", [
    (3, False, 3, False, True),     # requested after the last switch, nothing pending: current
    (2, False, 3, False, False),    # requested before the last switch was sent: may predate it (on_list after on_switch)
    (3, True, 3, False, False),     # requested while a switch was pending
    (3, False, 3, True, False),     # arrives while a switch is pending
    (0, False, 0, False, True),     # no switch yet
])
def test_a_list_answer_is_applied_only_when_no_switch_can_have_overtaken_it(requested_after, requested_while, sent, switching_now, current):
    assert lg.list_answer_is_current(requested_after, requested_while, sent, switching_now) is current


def test_switching_on_turns_into_switch_failed_after_the_timeout():
    since, overdue = lg.activation_overdue((), "inactive", None, 10.0, 2.0)
    assert (since, overdue) == (10.0, False), "the activating state starts now"
    assert lg.activation_overdue((), "inactive", since, 12.0, 2.0) == (10.0, False), "on the timeout is not past it"
    assert lg.activation_overdue((), "inactive", since, 12.01, 2.0) == (10.0, True)
    assert lg.activation_overdue((), "active", since, 50.0, 2.0) == (None, False), "active ends it"
    assert lg.activation_overdue((lg.BASE_MOVING,), "inactive", since, 50.0, 2.0) == (None, False), "a block ends it"


@pytest.mark.parametrize("listed", [None, "unconfigured"])
def test_no_switch_failed_while_lift_controller_is_not_loaded_yet(listed):
    """Nothing can be switched before the controller manager lists lift_controller (its spawner is still loading it): no clock, no
    SWITCH_FAILED, however long; still WARN, and the message says what it waits for."""
    assert lg.activation_overdue((), listed, None, 10.0, 2.0) == (None, False)
    assert lg.activation_overdue((), listed, 10.0, 99.0, 2.0) == (None, False)
    level, _, message, values = lg.build_status((), listed)
    assert (level, values) == (lg.LEVEL_WARN, [("lift_controller", "activating"), ("blocking", "")])
    assert "waiting for lift_controller to be loaded" in message
