# SPDX-License-Identifier: MIT
"""How the lift guard's helper processes end on SIGINT/SIGTERM (issue #44): exit 0, as every process of a launch must (P24, nothing is
allow-listed). The same contract as openamrobot_upperbody_moveit's joint_state_clamp.run.

A shutdown reaches a spinning rclpy process in one of three ways, all of them a clean stop: KeyboardInterrupt; ExternalShutdownException
(rclpy raises it out of a spin for every shutdown, SIGTERM included); or a RuntimeError raised by the executor when the signal lands between
the wait set reporting work and the take. The third is only a shutdown if the context is no longer ok: the same error with a live context is
a defect and is re-raised. Measured in the mock stack (issue #44): park_arms.py, interrupted by a launch teardown while it waited, died with
-2 (KeyboardInterrupt) and failed L01's exit-code check.

exit_process: how these processes end, once main() has its exit code. They do not destroy their node or shut rclpy down first. With
rmw_fastrtps (Jazzy's default) destroying the last node destroys the DDS participant, and that can deadlock: the main thread waits in
DomainParticipant::set_listener (rmw_fastrtps_shared_cpp::destroy_participant) for a listener callback that a discovery thread is running
and that waits on a lock the main thread holds. It holds the GIL, so neither a Python watchdog nor SIGTERM can end it; launch escalates to
SIGKILL after 15 s and the exit code is -9. Seen in the stack: lift_guard_odom_stub.py in #60's L04 and base_joint_states_stub.py in M01
(1 in 6 launch runs), and reproduced with the stub alone, 1 in 160 SIGINTs under load (gdb backtrace on #53). The process is ending:
os._exit leaves the participant to the kernel, and DDS peers drop it at its lease.
"""
import os
import sys

import rclpy
from rclpy.executors import ExternalShutdownException


def is_clean_stop(error, context_ok=rclpy.ok):
    """Whether `error`, raised while a process waited on rclpy, is a shutdown (a clean stop) rather than a defect."""
    if isinstance(error, (KeyboardInterrupt, ExternalShutdownException)):
        return True
    return isinstance(error, RuntimeError) and not context_ok()


def spin_until_shutdown(node, spin=rclpy.spin, context_ok=rclpy.ok):
    """Spin `node` until the context is shut down; 0 when a shutdown ended the spin, anything else raises."""
    try:
        spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError) as error:
        if not is_clean_stop(error, context_ok):
            raise
    return 0


def exit_process(code):
    """End the process with `code` now, without destroying the node or the DDS participant (see the module docstring)."""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.flush()
        except (OSError, ValueError):
            pass
    os._exit(code)
