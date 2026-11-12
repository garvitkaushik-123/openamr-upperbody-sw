#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Start a ROS node many times, many at once in one domain under CPU load, SIGINT it, and count the starts that hang or exit non-zero.

    teardown_stress.py [--runs 400] [--batch 16] [--load 3] [--run-for 3.0] [--grace 5.0] [-- COMMAND ...]

The reproduction of the teardown hang of base_joint_states_stub.py (M01, P24): `destroy_node()` looping in Fast DDS's
DomainParticipant::set_listener with the GIL held, which SIGTERM cannot interrupt. Each batch starts `--batch` copies of COMMAND (by default the
installed base_joint_states_stub.py) in one private ROS domain on localhost, with `--load` busy loops on the side, sends SIGINT after
`--run-for` seconds, and waits `--grace` seconds. A copy still alive then is a hang (killed, and counted); any exit code but 0 is a failure.
Before the fix: about 1 hang in 160 to 400 starts. Exit status 0 only when every start exited 0 in time.
"""
import argparse
import os
import random
import signal
import subprocess
import sys
import time


def default_command():
    from ament_index_python.packages import get_package_prefix
    return [os.path.join(get_package_prefix("openamrobot_upperbody_bringup"), "lib", "openamrobot_upperbody_bringup", "base_joint_states_stub.py")]


def run_batch(command, size, run_for, grace, env):
    processes = [subprocess.Popen(command, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, start_new_session=True)
                 for _ in range(size)]
    time.sleep(run_for)
    for p in processes:
        p.send_signal(signal.SIGINT)
    deadline = time.monotonic() + grace
    results = []
    for p in processes:
        try:
            code = p.wait(timeout=max(0.0, deadline - time.monotonic()))
            hung = False
        except subprocess.TimeoutExpired:
            p.kill()
            code = p.wait()
            hung = True
        err = p.stderr.read().decode(errors="replace")
        p.stderr.close()
        results.append((p.pid, hung, code, err))
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--runs", type=int, default=400, help="total starts (default 400)")
    parser.add_argument("--batch", type=int, default=16, help="copies started at once in one domain (default 16)")
    parser.add_argument("--load", type=int, default=3, help="busy loops on the side, like `yes > /dev/null` (default 3)")
    parser.add_argument("--run-for", type=float, default=3.0, help="seconds before the SIGINT (default 3.0)")
    parser.add_argument("--grace", type=float, default=5.0, help="seconds a copy has to exit after the SIGINT (default 5.0)")
    parser.add_argument("command", nargs="*", help="the node to start (default: the installed base_joint_states_stub.py)")
    args = parser.parse_args()
    command = args.command or default_command()

    env = dict(os.environ, ROS_DOMAIN_ID=str(random.randint(101, 200)), ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    load = [subprocess.Popen(["yes"], stdout=subprocess.DEVNULL) for _ in range(args.load)]
    started = hangs = failures = 0
    try:
        while started < args.runs:
            size = min(args.batch, args.runs - started)
            for pid, hung, code, err in run_batch(command, size, args.run_for, args.grace, env):
                started += 1
                if hung:
                    hangs += 1
                    print(f"HANG: pid {pid} still alive {args.grace:g} s after SIGINT (killed, exit {code})", flush=True)
                elif code != 0:
                    failures += 1
                    print(f"FAIL: pid {pid} exited {code}:\n{err[-1500:]}", flush=True)
            print(f"{started}/{args.runs} started, {hangs} hung, {failures} exited non-zero", flush=True)
    finally:
        for p in load:
            p.kill()
            p.wait()
    print(f"RESULT: {started} starts of {' '.join(command)} (domain {env['ROS_DOMAIN_ID']}, batch {args.batch}, load {args.load}): "
          f"{hangs} hangs, {failures} non-zero exits")
    return 0 if hangs == 0 and failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
