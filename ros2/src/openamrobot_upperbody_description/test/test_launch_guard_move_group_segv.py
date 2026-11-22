# SPDX-License-Identifier: MIT
"""upperbody-sw #48: move_group's -11 is allowed only as the teardown crash it was documented as.

The teardown text is the one move_group printed in every A7 run looked at (7 of 7 in hosted run 42527494905, 1 of 1 locally): the SIGINT handler
line, then a backtrace through MoveItCpp::~MoveItCpp, then "Segmentation fault". A -11 without that shape fails; nothing else changes."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "scripts"))
import launch_guard as lg  # noqa: E402

TEARDOWN = """[INFO] [1794284838.593330053] [rclcpp]: signal_handler(SIGINT/SIGTERM)
Stack trace (most recent call last):
#9    Object "/opt/ros/jazzy/lib/libmoveit_cpp.so.2.12.4", at 0x7fb333e55ad0, in moveit_cpp::MoveItCpp::~MoveItCpp()
#7    Object "/opt/ros/jazzy/lib/libmoveit_trajectory_execution_manager.so.2.12.4", in trajectory_execution_manager::TrajectoryExecutionManager::~TrajectoryExecutionManager()
#0    Object "/opt/ros/jazzy/lib/librclcpp.so", at 0x7fb333903ba9, in rclcpp::CallbackGroup::~CallbackGroup()
Segmentation fault (Address not mapped to object [0x7fb32c0c1da0])
"""
MID_RUN = """[INFO] [1.0] [move_group]: You can start planning now!
Stack trace (most recent call last):
#3    Object "/opt/ros/jazzy/lib/libmoveit_planning_scene_monitor.so.2.12.4", in planning_scene_monitor::CurrentStateMonitor::jointStateCallback
Segmentation fault (Address not mapped to object [0x10])
"""
OTHER_PATH = """[INFO] [2.0] [rclcpp]: signal_handler(SIGINT/SIGTERM)
Stack trace (most recent call last):
#3    Object "/opt/ros/jazzy/lib/libmoveit_planning_scene_monitor.so.2.12.4", in planning_scene_monitor::CurrentStateMonitor::jointStateCallback
Segmentation fault (Address not mapped to object [0x10])
"""


class Event:
    def __init__(self, name, code=None, text=b""):
        self.action = type("A", (), {"process_details": {"name": name}})()
        self.process_name, self.returncode, self.text = name, code, text


class ProcInfo:
    def __init__(self, codes):
        self.events = {n: Event(n, c) for n, c in codes.items()}

    def processes(self):
        return [e.action for e in self.events.values()]

    def process_names(self):
        return list(self.events)

    def __getitem__(self, key):
        return self.events[key if isinstance(key, str) else key.process_details["name"]]


class Output:
    def __init__(self, texts):
        self.texts = texts

    def __getitem__(self, name):
        text = self.texts[name]
        half = len(text) // 2         # output arrives in pieces; the check must read them joined
        return [Event(name, text=text[:half].encode()), Event(name, text=text[half:].encode())]


def test_the_observed_teardown_crash_is_recognised():
    assert lg.segv_is_teardown(TEARDOWN) == (True, "teardown")


@pytest.mark.parametrize("text, why", [(MID_RUN, "no 'signal_handler"), (OTHER_PATH, "does not go through"), ("", "no 'signal_handler"),
                                       ("Segmentation fault\n" + TEARDOWN, "comes before the SIGINT")])
def test_anything_else_is_not_teardown(text, why):
    ok, reason = lg.segv_is_teardown(text)
    assert not ok and why in reason


def test_a_move_group_crash_while_running_fails_the_shutdown_check():
    info = ProcInfo({"move_group-8": -11, "gazebo-1": -2})
    with pytest.raises(AssertionError, match="move_group-8 exited with -11"):
        lg.assert_launch_healthy(info, expected=["move_group"], allowed=lg.SIM_ALLOWED, proc_output=Output({"move_group-8": MID_RUN}))


def test_the_teardown_crash_still_passes_and_is_still_recorded():
    seen = []
    info = ProcInfo({"move_group-8": -11, "gazebo-1": -2})
    lg.assert_launch_healthy(info, expected=["move_group"], allowed=lg.SIM_ALLOWED, record=seen.append,
                             proc_output=Output({"move_group-8": TEARDOWN}))
    assert {"process": "move_group-8", "exit_code": -11} in seen[0]


def test_a_move_group_with_no_captured_output_and_minus_11_fails():
    with pytest.raises(AssertionError, match="not stopped by the launch"):
        lg.assert_launch_healthy(ProcInfo({"move_group-2": -11}), expected=["move_group"], allowed=lg.MOVE_GROUP_ALLOWED, proc_output=Output({}))


def test_a_clean_move_group_needs_no_output_and_the_allowance_list_is_unchanged():
    lg.assert_launch_healthy(ProcInfo({"move_group-2": 0}), expected=["move_group"], allowed=lg.MOVE_GROUP_ALLOWED, proc_output=Output({}))
    assert lg.MOVE_GROUP_ALLOWED == {"move_group": [lg.SIGSEGV]}
