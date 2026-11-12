# SPDX-License-Identifier: MIT
"""How the lift guard's helper processes end on SIGINT/SIGTERM (issue #44): exit 0, as every process of a launch must (P24, nothing is
allow-listed). The same contract as openamrobot_upperbody_moveit's joint_state_clamp.run.

A shutdown reaches a spinning rclpy process in one of three ways, all of them a clean stop: KeyboardInterrupt; ExternalShutdownException
(rclpy raises it out of a spin for every shutdown, SIGTERM included); or a RuntimeError raised by the executor when the signal lands between
the wait set reporting work and the take. The third is only a shutdown if the context is no longer ok: the same error with a live context is
a defect and is re-raised. Measured in the mock stack (issue #44): park_arms.py, interrupted by a launch teardown while it waited, died with
-2 (KeyboardInterrupt) and failed L01's exit-code check.
"""
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
