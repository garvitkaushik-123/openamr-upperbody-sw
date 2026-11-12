# SPDX-License-Identifier: MIT
"""The simulated lift guard's decision (issue #44): may lift_controller be active? No ROS here, so it is unit-tested without a runtime.

What it mirrors. The robot's interlock (firmware CTL-017) lets the lift move only with the base stopped and both arms stowed. In simulation
nothing enforced that (baseline on main 2ca7392, issue #44): the lift moved with an arm out, with the base driving, and with no arm state at
all. The guard node (scripts/lift_guard.py) keeps lift_controller INACTIVE unless every condition below holds, and deactivates it as soon as
one fails. An inactive JointTrajectoryController rejects action goals and ignores its joint_trajectory topic, so every command path is
covered, including the ones that bypass MoveIt.

What it is NOT. It does not replace the robot-side interlock, it is not safety evidence, and it claims nothing about a guard process that
hangs (only one that exits: on SIGINT or SIGTERM the node deactivates lift_controller itself, and on any other exit the launch does). It runs
only in the simulated profiles, mock and gazebo.

The decision, per evaluation, is a tuple of blocking codes, in CODE_ORDER; empty means the lift may move:
  ARM_STATE_MISSING / ARM_STATE_STALE   an arm joint, or an arm controller's controller_state, never received / older than state_timeout_s
                                        (also: a controller_state that does not cover the arm's joints, or has no reference velocities)
  ARM_NOT_STOWED_{LEFT,RIGHT}           the arm's joints are not all within stowed_tolerance of one of the stowed states
  ARM_TRAJECTORY_ACTIVE_{LEFT,RIGHT}    the arm controller's reference is not at a stowed state, or moves faster than reference_velocity_max,
                                        or its follow_joint_trajectory action has a goal in any status but SUCCEEDED, CANCELED or
                                        ABORTED (UNKNOWN and undefined values included)
  ODOM_MISSING / ODOM_STALE             no odometry yet / older than state_timeout_s
  BASE_MOVING                           the base's speed is not below its thresholds, or has not been for base_stopped_hold_s
  SWITCH_FAILED                         the last controller switch the guard asked for failed (set by the node)

Times are the guard's own clock at RECEIPT, never a header stamp: in Gazebo the merged /joint_states keeps advancing its stamp with frozen
positions (issue #44, baseline), so a stamp says nothing about whether the data is current. Every comparison fails closed: a NaN, a clock
that goes backwards, a missing joint or a malformed message blocks the lift.
"""
import math
import xml.etree.ElementTree as ET

SIDES = ("left", "right")
# The arm joints of each side: the SRDF groups left_arm and right_arm (openamrobot.srdf), which stowed_poses_from_srdf checks against.
ARM_JOINTS = {side: tuple(f"openarm_{side}_joint{n}" for n in range(1, 8)) for side in SIDES}
SRDF_GROUPS = {"left": "left_arm", "right": "right_arm"}

ARM_STATE_MISSING = "ARM_STATE_MISSING"
ARM_STATE_STALE = "ARM_STATE_STALE"
ARM_NOT_STOWED = {"left": "ARM_NOT_STOWED_LEFT", "right": "ARM_NOT_STOWED_RIGHT"}
ARM_TRAJECTORY_ACTIVE = {"left": "ARM_TRAJECTORY_ACTIVE_LEFT", "right": "ARM_TRAJECTORY_ACTIVE_RIGHT"}
ODOM_MISSING = "ODOM_MISSING"
ODOM_STALE = "ODOM_STALE"
BASE_MOVING = "BASE_MOVING"
SWITCH_FAILED = "SWITCH_FAILED"
CODE_ORDER = (ARM_STATE_MISSING, ARM_STATE_STALE, ARM_NOT_STOWED["left"], ARM_NOT_STOWED["right"],
              ARM_TRAJECTORY_ACTIVE["left"], ARM_TRAJECTORY_ACTIVE["right"], ODOM_MISSING, ODOM_STALE, BASE_MOVING, SWITCH_FAILED)

# action_msgs/msg/GoalStatus: the terminal states, the only ones in which a goal can no longer move the arm. Every other value blocks:
# ACCEPTED (1), EXECUTING (2) and CANCELING (3), UNKNOWN (0, "status has not been properly set") and any value the message does not define.
GOAL_STATUS_SUCCEEDED, GOAL_STATUS_CANCELED, GOAL_STATUS_ABORTED = 4, 5, 6
TERMINAL_GOAL_STATUSES = frozenset((GOAL_STATUS_SUCCEEDED, GOAL_STATUS_CANCELED, GOAL_STATUS_ABORTED))

# diagnostic_msgs/msg/DiagnosticStatus levels used here.
LEVEL_OK, LEVEL_WARN, LEVEL_ERROR = 0, 1, 2
STATUS_NAME = "lift_guard"

# Provisional values, NOT firmware values (issue #44): the firmware design (#20) sets the base threshold, the hold time and the timeout,
# and the stowed and carry definitions are still to come. They are parameters of the node so that they can be set without a code change.
DEFAULTS = {
    "stowed_states": ("park",),
    "stowed_tolerance": 0.01,             # rad
    "reference_velocity_max": 0.001,      # rad/s
    "base_linear_speed_max": 0.01,        # m/s
    "base_angular_speed_max": 0.01,       # rad/s
    "base_stopped_hold_s": 0.5,
    "state_timeout_s": 0.5,
}


class GuardConfigError(ValueError):
    """The guard cannot be configured as asked. Raised instead of guessing: the node refuses to start, and lift_controller stays inactive."""


def validate_config(config):
    """A full config dict (DEFAULTS' keys) with every number finite and positive, and at least one stowed state. Returns a new dict."""
    out = dict(DEFAULTS)
    unknown = set(config) - set(DEFAULTS)
    if unknown:
        raise GuardConfigError(f"unknown settings: {', '.join(sorted(unknown))}")
    out.update(config)
    out["stowed_states"] = tuple(out["stowed_states"])
    if not out["stowed_states"] or not all(isinstance(s, str) and s for s in out["stowed_states"]):
        raise GuardConfigError(f"stowed_states must name at least one SRDF group state, got {out['stowed_states']!r}")
    for key in DEFAULTS:
        if key == "stowed_states":
            continue
        value = out[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise GuardConfigError(f"{key} must be a finite number > 0, got {value!r}")
        out[key] = float(value)
    return out


def stowed_poses_from_srdf(srdf_xml, state_names):
    """{side: ({joint: position}, ...)}: one pose per name in `state_names`, read from the SRDF's group states of left_arm and right_arm.

    Refuses (GuardConfigError) rather than guess: a group that is not exactly ARM_JOINTS of its side, a state missing for a group, a state that
    does not set every joint of the group, or a value that is not a finite number."""
    try:
        root = ET.fromstring(srdf_xml)
    except ET.ParseError as err:
        raise GuardConfigError(f"the SRDF does not parse: {err}") from err
    for side, group_name in SRDF_GROUPS.items():
        groups = [g for g in root.findall("group") if g.get("name") == group_name]
        if len(groups) != 1:
            raise GuardConfigError(f"the SRDF has {len(groups)} groups named {group_name}, expected 1")
        if any(child.tag != "joint" for child in groups[0]):
            raise GuardConfigError(f"SRDF group {group_name} is not a plain list of joints; the guard reads only <joint> members")
        joints = tuple(j.get("name") for j in groups[0].findall("joint"))
        if joints != ARM_JOINTS[side]:
            raise GuardConfigError(f"SRDF group {group_name} has joints {joints}, the guard expects {ARM_JOINTS[side]}")
    poses = {side: [] for side in SIDES}
    for state in state_names:
        for side, group_name in SRDF_GROUPS.items():
            matches = [s for s in root.findall("group_state") if s.get("name") == state and s.get("group") == group_name]
            if len(matches) != 1:
                raise GuardConfigError(f"the SRDF has {len(matches)} group_state {state!r} for group {group_name}, expected 1")
            pose = {}
            for joint in matches[0].findall("joint"):
                try:
                    value = float(joint.get("value"))
                except (TypeError, ValueError) as err:
                    raise GuardConfigError(f"group_state {state!r}/{group_name}: {joint.get('name')} has value "
                                           f"{joint.get('value')!r}") from err
                if not math.isfinite(value):
                    raise GuardConfigError(f"group_state {state!r}/{group_name}: {joint.get('name')} is {value!r}")
                pose[joint.get("name")] = value
            if set(pose) != set(ARM_JOINTS[side]):
                raise GuardConfigError(f"group_state {state!r}/{group_name} sets {sorted(pose)}, expected {list(ARM_JOINTS[side])}")
            poses[side].append(pose)
    return {side: tuple(poses[side]) for side in SIDES}


def _within(values, pose, joints, tolerance):
    """True if every joint's value is within `tolerance` of `pose`. A NaN is never within (the comparison is False)."""
    return all(abs(values[j] - pose[j]) <= tolerance for j in joints)


class LiftGuardLogic:
    """The inputs the guard has seen, and the decision from them. The node feeds it; it never talks to ROS.

    `now` everywhere is the guard's clock in seconds (float), at receipt."""

    def __init__(self, stowed_poses, config=None):
        self.config = validate_config(config or {})
        for side in SIDES:
            if not stowed_poses.get(side):
                raise GuardConfigError(f"no stowed pose for the {side} arm")
        self.stowed_poses = {side: tuple(stowed_poses[side]) for side in SIDES}
        self._arm_joint_names = frozenset(j for side in SIDES for j in ARM_JOINTS[side])
        self._joints = {}                                  # joint -> (position, received)
        self._controller = {side: None for side in SIDES}  # side -> (reference dict, velocities ok, received) or "malformed"
        self._goal_statuses = {side: frozenset() for side in SIDES}
        self._odom_received = None
        self._stopped_since = None
        self.switch_failed = False

    # --- inputs ---------------------------------------------------------------------------------------------------------------------------
    def on_joint_state(self, names, positions, now):
        """A sensor_msgs/JointState. Only the arm joints are kept; any other joint (wheels, lift, grippers) is ignored. A name without a
        position (a malformed message) is not updated, so it goes stale."""
        for name, position in zip(names, positions):
            if name in self._arm_joint_names:
                self._joints[name] = (float(position), now)

    def on_controller_state(self, side, joint_names, reference_positions, reference_velocities, now):
        """An arm controller's control_msgs/JointTrajectoryControllerState. It must cover the side's joints, with a reference position and
        velocity for each (the arm controllers have a velocity state interface, so JTC sizes reference.velocities): otherwise it is kept as
        malformed, which blocks the lift as ARM_STATE_MISSING."""
        joint_names, positions, velocities = list(joint_names), list(reference_positions), list(reference_velocities)
        if (len(positions) != len(joint_names) or len(velocities) != len(joint_names)
                or not set(ARM_JOINTS[side]) <= set(joint_names)):
            self._controller[side] = ("malformed", now)
            return
        index = {name: i for i, name in enumerate(joint_names)}
        reference = {j: float(positions[index[j]]) for j in ARM_JOINTS[side]}
        velocities_ok = all(abs(float(velocities[index[j]])) <= self.config["reference_velocity_max"] for j in ARM_JOINTS[side])
        self._controller[side] = (reference, velocities_ok, now)

    def on_goal_statuses(self, side, statuses):
        """The status codes of every goal in the arm's follow_joint_trajectory/_action/status (action_msgs/GoalStatusArray). This topic only
        publishes on a change, so it has no freshness: no message yet means no goal yet. Any goal not in a terminal status (SUCCEEDED,
        CANCELED, ABORTED) blocks, including UNKNOWN and values the message does not define."""
        self._goal_statuses[side] = frozenset(int(s) for s in statuses)

    def on_odometry(self, linear, angular, now):
        """nav_msgs/Odometry twist: `linear` and `angular` are (x, y, z). The base counts as stopped when the norm of each is strictly below
        its threshold; it must stay so for base_stopped_hold_s before the lift may move. A NaN is not stopped."""
        linear_speed = math.sqrt(sum(float(v) * float(v) for v in linear))
        angular_speed = math.sqrt(sum(float(v) * float(v) for v in angular))
        stopped = linear_speed < self.config["base_linear_speed_max"] and angular_speed < self.config["base_angular_speed_max"]
        if not stopped:
            self._stopped_since = None
        elif self._stopped_since is None:
            self._stopped_since = now
        self._odom_received = now

    # --- the decision ---------------------------------------------------------------------------------------------------------------------
    def _stale(self, received, now):
        age = now - received
        return not (0.0 <= age <= self.config["state_timeout_s"])       # a clock that went backwards counts as stale, as does a NaN

    def evaluate(self, now):
        """The blocking codes at `now`, in CODE_ORDER. Empty: the lift may move."""
        codes = set()
        tolerance = self.config["stowed_tolerance"]
        for side in SIDES:
            joints = ARM_JOINTS[side]
            controller = self._controller[side]
            if any(j not in self._joints for j in joints) or controller is None or controller[0] == "malformed":
                codes.add(ARM_STATE_MISSING)
            elif any(self._stale(self._joints[j][1], now) for j in joints) or self._stale(controller[-1], now):
                codes.add(ARM_STATE_STALE)
            else:
                positions = {j: self._joints[j][0] for j in joints}
                if not any(_within(positions, pose, joints, tolerance) for pose in self.stowed_poses[side]):
                    codes.add(ARM_NOT_STOWED[side])
                reference, velocities_ok, _ = controller
                if not velocities_ok or not any(_within(reference, pose, joints, tolerance) for pose in self.stowed_poses[side]):
                    codes.add(ARM_TRAJECTORY_ACTIVE[side])
            if self._goal_statuses[side] - TERMINAL_GOAL_STATUSES:
                codes.add(ARM_TRAJECTORY_ACTIVE[side])
        if self._odom_received is None:
            codes.add(ODOM_MISSING)
        elif self._stale(self._odom_received, now):
            codes.add(ODOM_STALE)
        elif self._stopped_since is None or not (now - self._stopped_since >= self.config["base_stopped_hold_s"]):
            codes.add(BASE_MOVING)
        if self.switch_failed:
            codes.add(SWITCH_FAILED)
        return tuple(code for code in CODE_ORDER if code in codes)


def wants_active(codes):
    """Whether the guard should ask for lift_controller to be active: no blocking code other than SWITCH_FAILED (a failed switch is retried,
    it does not by itself say the lift is unsafe; it stays in the status until a switch succeeds)."""
    return all(code == SWITCH_FAILED for code in codes)


def switch_to_request(codes, controller_state, pending):
    """The switch to ask the controller manager for now: "active", "inactive" or None (nothing).

    `controller_state` is lift_controller's state as last listed (None if unknown), `pending` the target of a switch already asked for and
    not answered (None if none). A deactivation is asked at once whenever the controller is, or is about to be, active, even over a pending
    activation; an activation is asked only from a listed "inactive" with nothing pending. An unknown state is never activated."""
    if not wants_active(codes):
        if (controller_state == "active" or pending == "active") and pending != "inactive":
            return "inactive"
        return None
    if controller_state == "inactive" and pending is None:
        return "active"
    return None


def list_answer_is_current(requested_after_switch, requested_while_switching, switches_sent, switching_now):
    """Whether a list_controllers answer may update lift_controller's known state (issue #44, review of #53).

    An answer requested before the last switch was sent, or while a switch was pending, or that arrives while one is pending, may describe the
    state before that switch; applied after the switch's own answer it would overwrite newer state (seen as OK published while the
    controller was inactive). `requested_after_switch` is the number of switches sent when the list was requested, `switches_sent` the number
    sent now."""
    return requested_after_switch == switches_sent and not requested_while_switching and not switching_now


def activation_overdue(codes, controller_state, activating_since, now, timeout_s):
    """(activating_since, overdue) for the WARN "activating" state: nothing blocks but lift_controller is not listed active.

    `activating_since` is when that state began (None when not in it). Overdue once it has lasted longer than `timeout_s`: the node then
    reports SWITCH_FAILED (ERROR) instead of staying in WARN, e.g. when the switch service is not there to answer.

    The clock runs only while lift_controller is listed "inactive", i.e. while an activation can be asked. Before the controller manager
    lists it at all (its spawner is still loading it; seen in the mock stack, about 3 s after the arms were parked) nothing can be switched,
    so nothing has failed: it stays WARN, as #60's P7 also counts WARN only once the controller has been seen."""
    if codes or controller_state != "inactive":
        return None, False
    since = now if activating_since is None else activating_since
    return since, now - since > timeout_s


def build_status(codes, controller_state):
    """The content of /lift_guard/status (diagnostic_msgs/DiagnosticStatus) as plain data: (level, name, message, [(key, value), ...]).

    `controller_state` is lift_controller's state as the controller manager last listed it ("active", "inactive", ...), or None when it is
    not known. Three levels, as accepted on issue #44:
      OK     nothing blocks and lift_controller is listed active: the lift may move.
      WARN   nothing blocks but lift_controller is not listed active yet: switching on. `blocking` is empty, `lift_controller` "activating".
      ERROR  something blocks: always blocked, and `blocking` always names at least one code. `lift_controller` is "active" if the controller
             manager lists it active (an outside activation the guard is undoing), else "inactive"."""
    listed_active = controller_state == "active"
    if codes:
        level, message = LEVEL_ERROR, "lift blocked: " + ", ".join(codes)
        state = "active" if listed_active else "inactive"
    elif listed_active:
        level, message, state = LEVEL_OK, "lift allowed", "active"
    elif controller_state == "inactive":
        level, message, state = LEVEL_WARN, "lift allowed by the guard; switching lift_controller on", "activating"
    else:
        level, message, state = LEVEL_WARN, "lift allowed by the guard; waiting for lift_controller to be loaded", "activating"
    return level, STATUS_NAME, message, [("lift_controller", state), ("blocking", ",".join(codes))]
