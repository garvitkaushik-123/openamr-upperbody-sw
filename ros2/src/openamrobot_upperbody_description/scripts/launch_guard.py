# SPDX-License-Identifier: MIT
"""What every launch test must check after the launch has stopped: that the processes it needed started, and that none of them died.

`launch_testing` runs the test methods while the launch is up and, unless the module says otherwise, never looks at how the processes ended:
a process of the launch that dies of a signal while the test passes, or one that cannot be started at all, is invisible to the test and so to
the evidence (demonstrated with canaries, P12A task 0: both ended as a passing test and a `0 FAIL` report). So each launch test module ends with

    @launch_testing.post_shutdown_test()
    class TestShutdown(unittest.TestCase):
        def test_processes_started_and_exited_cleanly(self, proc_info):
            assert_launch_healthy(proc_info, expected=["move_group", ...], allowed={"move_group": [SIGSEGV]})

`expected` lists the name prefix of every process the test cannot do without: a process that never started is not in `proc_info` at all (demonstrated with
a canary, an ExecuteProcess whose executable does not exist: the launch carried on with its other processes and the test passed), so only a list kept next to
the launch description can notice it is missing. A process that DIES during the tests is a different failure, caught by the last active test
(`assert_no_process_died`) before the shutdown allowances below can hide it. `allowed` lists the non-zero exit codes a process may end with, per
process, each one a known and documented behaviour of the installed software and nothing broader. `test_h01_harness_guards.py` fails any launch
test module that does not call this, so a new launch test cannot be written without it.
"""
import json
import os
import signal
import time

from launch_testing.asserts import EXIT_OK, assertExitCodes

SIGSEGV = -11       # move_group, moveit_py: observed on every stop of MoveIt 2.12.4 (see the moveit package README)
SIGINT = -2         # a process the launch stopped with SIGINT and that did not install a handler (a ruby wrapper, `sleep`)
SIGTERM = -15       # the same process, when it did not die of the SIGINT within the launch's timeout and the launch escalated
SIGABRT = -6        # ros_gz_bridge's parameter_bridge, once in a while, at shutdown: see SIM_ALLOWED

# Non-zero exits that are known behaviours of the installed software at shutdown, and nothing broader. Each is a finding, not an approval:
#   gazebo             -2   the `ruby` wrapper of `gz sim` dies of the SIGINT that stops the launch (the server is stopped through /server_control),
#                      -15  or, when it outlives the launch's SIGINT timeout, of the SIGTERM the launch then sends (seen in one G02 run, P12A)
#   parameter_bridge   -6   ros2 ros_gz_bridge aborts on SIGINT with "corrupted double-linked list" in some runs (found in the log of one G01
#                           run in P12A: before this guard nothing looked at how the processes ended). Heap corruption is not benign. It is
#                           allowed here only so that a shutdown abort of platform-sw's bridge does not make G01/G02/M02 flaky, and it is
#                           a finding to take to platform-sw, not an approval
#   joint_state_publisher  1  the stock node raises "RuntimeError: Unable to convert call argument '0' to Python object" from rclpy's take_message when
#                           the SIGINT of the shutdown arrives (seen once in G02, P12A): a race in the stock node and rclpy, exit code 1; same status
MOVE_GROUP_ALLOWED = {"move_group": [SIGSEGV]}
SIM_ALLOWED = {"gazebo": [SIGINT, SIGTERM], "parameter_bridge": [SIGABRT], "joint_state_publisher": [1], **MOVE_GROUP_ALLOWED}


def assert_no_process_died(proc_info):
    """For an ACTIVE test, the last one of a launch test: in the active phase `proc_info` holds an event for every process that has started, and only the ones that have
    exited carry a returncode, so a process with a non-zero code died while the tests were running, not at shutdown. No allowance applies: the shutdown behaviours listed above
    happen after the launch is stopped, and the allowances of the post-shutdown check would otherwise hide a crash in the middle of a test."""
    # the active proc_info holds an event per process that has STARTED: a ProcessExited (with a returncode) once it ended, a ProcessStarted while it runs
    died = {name: getattr(proc_info[name], "returncode", None) for name in proc_info.process_names()}
    died = {name: code for name, code in died.items() if code not in (None, EXIT_OK)}
    assert not died, f"processes of the launch died while the tests were running: {died}"


def process_exit_records(proc_info):
    """[{"process": name, "exit_code": code}] for every process the launch started, in name order (P24). `exit_code` is the returncode of the
    process's last event: an int once it has exited (negative for a signal: -11 is SIGSEGV), None when the framework saw no exit. This is what a crash
    like `parameter_bridge-4 exited with code -11` is as DATA: before P24 it was a line on the console and a failed test name in the report."""
    records = []
    for action in proc_info.processes():              # by action, not by name: proc_info["spawner-1"] is a substring match and would also find "spawner-10"
        code = getattr(proc_info[action], "returncode", None)
        records.append({"process": action.process_details["name"], "exit_code": code if isinstance(code, int) else None})
    return sorted(records, key=lambda r: r["process"])


# upperbody-sw #48: move_group's -11 is allowed ONLY as the teardown crash it was documented as. In every A7 run looked at (7 of 7 in hosted run
# 42527494905, 1 of 1 in the local M02 reproduction of 10 Nov 2026) the process first logs rclcpp's `signal_handler(SIGINT/SIGTERM)` (the launch
# stopping it) and the backtrace then runs through `moveit_cpp::MoveItCpp::~MoveItCpp()`. A -11 without the signal line before it, or with a backtrace
# that does not go through that destructor, is a crash while running and fails the check, whatever the allowance says.
TEARDOWN_SIGNAL_LINE = "signal_handler(SIGINT/SIGTERM)"
TEARDOWN_DESTRUCTOR = "MoveItCpp::~MoveItCpp"
BACKTRACE_MARK = "Segmentation fault"


def segv_is_teardown(output_text):
    """(ok, reason) for one process's captured output, the stdout and stderr in the order they arrived. Pure."""
    signal_at = output_text.find(TEARDOWN_SIGNAL_LINE)
    if signal_at < 0:
        return False, f"no '{TEARDOWN_SIGNAL_LINE}' line: it was not stopped by the launch when it crashed"
    crash_at = output_text.find(BACKTRACE_MARK)
    if crash_at >= 0 and crash_at < signal_at:
        return False, "the segmentation fault comes before the SIGINT handler line: a crash while running"
    if crash_at >= 0 and TEARDOWN_DESTRUCTOR not in output_text[signal_at:crash_at + len(BACKTRACE_MARK)]:
        return False, f"the backtrace after the SIGINT does not go through {TEARDOWN_DESTRUCTOR}"
    return True, "teardown"


def process_output_text(proc_output, name):
    """All captured output of the process called `name` (an IoHandler as launch_testing passes to a post-shutdown test), decoded."""
    try:
        events = proc_output[name]
    except (KeyError, IndexError):        # a process that printed nothing has no entry
        return ""
    return "".join(e.text.decode(errors="replace") if isinstance(e.text, bytes) else str(e.text) for e in events)


def assert_move_group_segv_is_teardown(proc_info, proc_output):
    """Fail for every move_group whose exit code is -11 and whose own output does not show the teardown crash (segv_is_teardown)."""
    bad = []
    for record in process_exit_records(proc_info):
        if record["process"].startswith("move_group") and record["exit_code"] == SIGSEGV:
            ok, reason = segv_is_teardown(process_output_text(proc_output, record["process"]))
            if not ok:
                bad.append(f"{record['process']} exited with -11: {reason}")
    assert not bad, "move_group crashed outside teardown: " + "; ".join(bad)


def assert_launch_healthy(proc_info, expected, allowed=None, record=None, proc_output=None):
    """Fail if a process in `expected` (name prefixes) never started, or if any process that did start exited with a code that is neither 0 nor
    listed for it in `allowed` ({name prefix: [exit codes]}).

    `record(records)` (P24), when given, is called with `process_exit_records(proc_info)` BEFORE anything is asserted, so a launch that fails
    this check still leaves its exit codes in the evidence; it is how G01, G02, M01, M02 and M07 put them into their sidecars. It records and
    judges nothing: no allowance was added by it.

    `proc_output` (upperbody-sw #48), when given, narrows move_group's SIGSEGV allowance to the teardown crash (assert_move_group_segv_is_teardown)."""
    allowed = allowed or {}
    if record is not None:
        record(process_exit_records(proc_info))
    if proc_output is not None and any(SIGSEGV in codes for prefix, codes in allowed.items() if prefix.startswith("move_group")):
        assert_move_group_segv_is_teardown(proc_info, proc_output)
    started = list(proc_info.process_names())
    never_started = [prefix for prefix in expected if not any(name.startswith(prefix) for name in started)]
    assert not never_started, f"processes that never started: {never_started} (the launch started {started})"
    assert started, "the launch started no process at all"
    for name in started:
        codes = [EXIT_OK]
        for prefix, extra in allowed.items():
            if name.startswith(prefix):
                codes += list(extra)
        assertExitCodes(proc_info, allowable_exit_codes=codes, process=name)


# --- the simulator that outlives its launch (P16 task 2) ------------------------------------------------------------------------------------
# `gz sim` is started by a ruby wrapper that forks the server. When the launch stops, the wrapper dies of the SIGINT (SIM_ALLOWED above) and the
# server is supposed to be stopped through /server_control (sim.launch.py); in about 2 of 8 A7 runs, and in the first G02 run of P16, it was not:
# the server ended up a child of init, still running, still holding its transport partition. Nothing above sees it: the wrapper is the process the
# launch owns, and it has exited with a code the guard allows. So the check looks for the server itself, and keys on the one thing that is this
# run's and nobody else's: the GZ_PARTITION the test put in its environment before the launch, which every process of the launch inherits (a
# test that does not set one has nothing this function may look for, and it refuses). A `gz sim` of another run, of another test or of the
# person at the keyboard has another partition (or none) and is never matched.
GZ_SERVER_GRACE_S = 10.0     # wall seconds a server that was asked to stop may take to go before it counts as having survived
REAP_TERM_WAIT_S = 3.0
HELPER_REPORT_WAIT_S = 3.0   # a helper that wrote {"state": "started"} and has not finished: how long its final report is waited for

# P17/P18: sim.launch.py stops the server through a detached helper that asks it to stop, watches it, and signals it if it does not go (see that
# file's header). A server that had to be signalled DID NOT STOP ON REQUEST, and that is recorded in `report["forced"]` and surfaces as the WARN
# check W04 in the A7 report (it was a FAIL in P17; P18 decided that a hung server which the launch then ended is a finding, not a failure of the
# run). What did NOT change, and must not: this function judges what is still running in /proc AFTER the helper has had its time to act (it
# waits up to `grace_s`, longer than the helper's 9 s), and any `gz sim` of the run's partition still alive then is a survivor and a FAIL. A server
# that survives its own force-stop is still an orphan.
def gz_stop_report_path(partition):
    """The helper's report for `partition`: `gz_stop_<partition>.json` in the evidence directory (evidence_common.evidence_dir, the one every
    sidecar goes to; a report is part of a run's evidence and evidence is never kept in a temp directory). sim.launch.py's gz_stop_report_path
    resolves it the same way and hands it to the helper (test_sim_launch_shutdown.py pins that they name one file)."""
    import evidence_common
    return os.path.join(evidence_common.evidence_dir(evidence_common.share_dir()), f"gz_stop_{partition}.json")


def read_gz_stop_report(partition, wait_s=HELPER_REPORT_WAIT_S, poll_s=0.1):
    """The stop helper's report of this run, or None when no helper ran (a launch without the stop handler, which is the orphan canary). A
    report still in state `started` is waited for up to `wait_s`: the helper is mid-watch and its final word is what matters."""
    path = gz_stop_report_path(partition)
    deadline = time.monotonic() + wait_s
    while True:
        try:
            with open(path) as f:
                report = json.load(f)
        except (OSError, ValueError):
            return None
        if report.get("state") != "started" or time.monotonic() >= deadline:
            return report
        time.sleep(poll_s)


def _pid_dirs(proc_root):
    return sorted(int(name) for name in os.listdir(proc_root) if name.isdigit())


def _read_bytes(path):
    try:
        with open(path, "rb") as f:
            return f.read()
    except OSError:     # gone in between, or another user's: not ours to look at
        return b""


def _state_and_ppid(proc_root, pid):
    """(state letter, ppid) from /proc/<pid>/stat; the command name in it may contain spaces and parentheses, hence the rpartition."""
    raw = _read_bytes(os.path.join(proc_root, str(pid), "stat")).decode("utf-8", "replace")
    after = raw.rpartition(")")[2].split()
    return (after[0], int(after[1])) if len(after) >= 2 else ("?", -1)


def is_gz_sim_command(tokens):
    """`gz sim ...` (the forked server, whose argv[0] is `gz`) or `ruby .../gz sim ...` (the wrapper): `sim` as an argument and a `gz` in the first
    two. `gz service ...` (the shutdown request), `gz topic ...` (a probe) and a test whose command line merely mentions gz are not simulators."""
    return "sim" in tokens[1:4] and any(os.path.basename(t) == "gz" for t in tokens[:2])


def gz_sim_processes_of(partition, proc_root="/proc", exclude_pids=()):
    """[{pid, ppid, state, cmdline}] of the live `gz sim` processes whose environment says GZ_PARTITION=<partition>. Zombies are not running
    anything and are left out. `partition` must be non-empty."""
    if not partition:
        raise ValueError("an empty GZ_PARTITION would match every simulator of the machine, not this run's")
    wanted = b"GZ_PARTITION=" + partition.encode()
    found = []
    for pid in _pid_dirs(proc_root):
        if pid in exclude_pids:
            continue
        environ = _read_bytes(os.path.join(proc_root, str(pid), "environ"))
        if wanted not in environ.split(b"\0"):
            continue
        # the forked server rewrites its argv into ONE string ("gz sim -r -s ..."), so split on whitespace as well as on NUL
        tokens = _read_bytes(os.path.join(proc_root, str(pid), "cmdline")).decode("utf-8", "replace").replace("\0", " ").split()
        if not is_gz_sim_command(tokens):
            continue
        state, ppid = _state_and_ppid(proc_root, pid)
        if state in ("Z", "X"):
            continue
        found.append({"pid": pid, "ppid": ppid, "state": state, "cmdline": " ".join(tokens)[:300]})
    return found


def gz_server_shutdown_report(partition, grace_s=GZ_SERVER_GRACE_S, reap=True, proc_root="/proc", poll_s=0.25):
    """After the launch under test has been torn down: wait up to `grace_s` for every `gz sim` of this run (`partition`) to be gone, and say what
    is left. A simulator that was asked to stop takes a moment to do so, so a server seen in the first instant is not yet a finding; one that is
    still there at the end of the grace period is. It is judged FIRST: the verdict in the returned dict (`survivors`, non-empty or not) is fixed
    before anything is signalled. Only then, with `reap` true, the survivors are stopped (SIGTERM, then SIGKILL after REAP_TERM_WAIT_S) so a
    failed run does not leave a simulator eating three cores under the next one; what was done to them is in `reaped`. Never the other way
    round: a remedy applied before the assertion would turn this check into a clean-up that cannot fail."""
    started = time.monotonic()
    survivors = gz_sim_processes_of(partition, proc_root)
    while survivors and time.monotonic() - started < grace_s:
        time.sleep(poll_s)
        survivors = gz_sim_processes_of(partition, proc_root)
    report = {"partition": partition, "grace_s": grace_s, "waited_s": round(time.monotonic() - started, 2),
              "survivors": survivors, "reaped": [], "forced": []}
    if proc_root == "/proc":       # the helper's report is a fact about the real machine, not about a fake /proc tree
        helper = read_gz_stop_report(partition)
        report["stop_helper"] = helper
        # a server the helper had to signal and that is gone by now is not a survivor (it is not running); it is recorded, and W04 reports it
        report["forced"] = ([{"pid": pid, "signals": helper.get("signals"), "gone": helper.get("gone")} for pid in helper.get("survivors", [])]
                            if helper and helper.get("state") == "force_stopped" else [])
        try:
            os.unlink(gz_stop_report_path(partition))
        except OSError:
            pass
    if survivors and reap:
        for proc in survivors:
            report["reaped"].append({"pid": proc["pid"], "signal": _stop(proc["pid"], proc_root)})
    return report


def _stop(pid, proc_root):
    """SIGTERM, and SIGKILL if the process is still there REAP_TERM_WAIT_S later (`gz sim` ignores a plain SIGTERM sent to its wrapper; the server
    itself does die of it only sometimes, observed). Returns the strongest signal that was needed."""
    for sig, wait in ((signal.SIGTERM, REAP_TERM_WAIT_S), (signal.SIGKILL, 2.0)):
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            return signal.Signals(sig).name if sig == signal.SIGTERM else "SIGTERM"
        except PermissionError:
            return "not permitted"
        end = time.monotonic() + wait
        while time.monotonic() < end:
            if _state_and_ppid(proc_root, pid)[0] in ("Z", "X", "?"):
                return signal.Signals(sig).name
            time.sleep(0.1)
    return "SIGKILL (still listed)"


def gz_survivor_message(report):
    forced = report.get("forced") or []
    note = f" (the stop helper had also had to signal {len(forced)} server(s) of this run: {forced})" if forced else ""
    return (f"{len(report['survivors'])} gz sim process(es) of this run (GZ_PARTITION={report['partition']}) survived the launch's teardown by "
            f"{report['waited_s']} s: {[(p['pid'], p['ppid']) for p in report['survivors']]} (pid, ppid); stopped afterwards with "
            f"{report['reaped']}{note}. The launch's own process guard cannot see this: the ruby wrapper it owns had exited with an allowed code")


def assert_no_gz_server_survives(partition, **kwargs):
    """Fail if a `gz sim` of this run (`partition`) is still alive after the grace period. Returns the report when none is."""
    report = gz_server_shutdown_report(partition, **kwargs)
    assert not report["survivors"], gz_survivor_message(report)
    return report
