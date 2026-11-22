#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Test counts for the Model tests workflow (.github/workflows/model-tests.yml), and its zero-executed-test gate.

    count_ci_tests.py --workspace WS --label A6 --evidence-glob 'evidence/*_dev.json' --exit-status RC [--summary FILE] [--guard-exit-log FILE] PKG [PKG ...]

Reads the JUnit XML that `colcon test` leaves in <WS>/build/<PKG>/test_results/<PKG>/*.xml and counts <testcase> elements, not the
`tests` attributes of the suites, so the numbers are the cases themselves:

  discovered   every test case
  skipped      a case with a <skipped> child (an OPENAMR_A7-gated M01/M02 under A6, a pytest.skip with its issue)
  executed     discovered minus skipped: the gate. Zero executed is a failure, whether nothing was found or everything skipped
  failed       an executed case with a <failure> or <error> child (reported; verify_a6.sh and run_a7_regression.sh decide the job)

`colcon test-result` prints a larger total because it also counts each registered CTest test once (the CTest wrapper around a pytest or
launch_testing file); that number is printed as `ctest entries` and `colcon total = discovered + ctest entries`, so the two can be compared.
The evidence report's check results are summarised next to the counts. This is CI bookkeeping only: it is no evidence of anything beyond the
A6 or A7 run it counts, and it changes no exit status of the verifiers (the workflow passes theirs through separately).

Exit status: 1 if no test case executed, 2 if a result file cannot be read, else 0.
"""
import argparse
import collections
import glob
import json
import os
import re
import sys
import xml.etree.ElementTree as ET

ADD_TEST = re.compile(r"^\s*add_test\(", re.MULTILINE)


def count_cases(paths):
    """Counter of discovered, skipped, executed and failed test cases over JUnit XML files (testsuite or testsuites root)."""
    counts = collections.Counter(files=0, discovered=0, skipped=0, failed=0)
    for path in paths:
        root = ET.parse(path).getroot()
        counts["files"] += 1
        for case in root.iter("testcase"):
            counts["discovered"] += 1
            if case.find("skipped") is not None:
                counts["skipped"] += 1
            elif case.find("failure") is not None or case.find("error") is not None:
                counts["failed"] += 1
    counts["executed"] = counts["discovered"] - counts["skipped"]
    return counts


def result_files(workspace, package):
    return sorted(glob.glob(os.path.join(workspace, "build", package, "test_results", package, "*.xml")))


def ctest_entries(workspace, package):
    """Registered CTest tests of a package: what `colcon test-result` counts on top of the test cases."""
    total = 0
    for path in glob.glob(os.path.join(workspace, "build", package, "**", "CTestTestfile.cmake"), recursive=True):
        with open(path) as f:
            total += len(ADD_TEST.findall(f.read()))
    return total


def evidence_lines(workspace, pattern):
    lines = []
    for report in sorted(glob.glob(os.path.join(workspace, pattern))):
        with open(report) as f:
            data = json.load(f)
        results = collections.Counter(c["result"] for c in data["checks"])
        lines.append(f"{os.path.basename(report)}: git {data['git_hash']}, evidence_class {data['evidence_class']}, "
                     + ", ".join(f"{k} {v}" for k, v in sorted(results.items())))
        lines += [f"- {c['id']} {c['result']}" for c in data["checks"] if c["result"] != "PASS"]
    return lines or [f"no evidence report matches {pattern}"]


# The lift guard's exit outcomes (ros2/src/openamrobot_upperbody_control/scripts/lift_guard.py): "confirmed" is verified; these two exit 0 as a
# simulator-teardown exception and are UNVERIFIED (owner decision on upperbody-sw #53, 22 Nov 2026); everything else is a failure.
GUARD_VERIFIED = ("confirmed",)
GUARD_UNVERIFIED = ("manager_gone", "unanswered")


def guard_exit_lines(path):
    """Summary lines for the lift guard's exits, read from the file the guard appends to ($OPENAMR_GUARD_EXIT_LOG, one JSON line per exit).
    UNVERIFIED exits are counted apart from the confirmed ones and are a WARN: they never join a PASS count. A missing or empty file is
    said, not read as zero unverified exits."""
    if not os.path.isfile(path):
        return [f"lift guard exits: no log at {path}: nothing recorded, none counted as confirmed"]
    with open(path) as f:
        rows = [json.loads(line) for line in f if line.strip()]
    if not rows:
        return [f"lift guard exits: the log {path} is empty: nothing recorded, none counted as confirmed"]
    counts = collections.Counter(row["outcome"] for row in rows)
    confirmed = sum(counts[o] for o in GUARD_VERIFIED)
    unverified = sum(counts[o] for o in GUARD_UNVERIFIED)
    failed = len(rows) - confirmed - unverified
    lines = [f"lift guard exits: {len(rows)}: {confirmed} confirmed, **{unverified} UNVERIFIED (WARN, not PASS)**, {failed} failed"]
    lines += [f"- {outcome}: {count}" for outcome, count in sorted(counts.items()) if outcome not in GUARD_VERIFIED]
    return lines


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--workspace", default=".")
    parser.add_argument("--label", required=True, help="A6 or A7, printed in the heading")
    parser.add_argument("--evidence-glob", required=True, help="relative to the workspace, e.g. 'evidence/*_dev.json'")
    parser.add_argument("--exit-status", default="not run", help="the verifier's exit status, printed only")
    parser.add_argument("--summary", help="append the lines to this file too (the job summary)")
    parser.add_argument("--guard-exit-log", help="the lift guard's exit log (OPENAMR_GUARD_EXIT_LOG): its exits are counted apart, "
                                                 "UNVERIFIED ones as WARN, never as PASS")
    parser.add_argument("packages", nargs="+")
    ns = parser.parse_args(argv)

    total = collections.Counter()
    lines = [f"### {ns.label}: test cases per package (discovered / skipped / executed / failed; ctest entries)"]
    try:
        for package in ns.packages:
            counts = count_cases(result_files(ns.workspace, package))
            ctest = ctest_entries(ns.workspace, package)
            total.update(counts)
            total["ctest"] += ctest
            lines.append(f"- {package}: {counts['discovered']} / {counts['skipped']} / {counts['executed']} / {counts['failed']}; "
                         f"{ctest} ctest entries, {counts['files']} result files")
        evidence = evidence_lines(ns.workspace, ns.evidence_glob)
        guard = guard_exit_lines(os.path.join(ns.workspace, ns.guard_exit_log)) if ns.guard_exit_log else []
    except (OSError, ET.ParseError, KeyError, ValueError) as exc:
        print(f"count_ci_tests: cannot read the results: {exc}", file=sys.stderr)
        return 2
    lines.append(f"**Total: {total['discovered']} discovered, {total['skipped']} skipped, {total['executed']} executed, "
                 f"{total['failed']} failed** (colcon total = discovered + {total['ctest']} ctest entries = "
                 f"{total['discovered'] + total['ctest']})")
    lines += evidence
    lines += guard
    lines.append(f"verifier exit status: {ns.exit_status}")
    lines.append(f"CI results are {ns.label} evidence only: not hardware, physical acceptance or safety evidence.")
    print("\n".join(lines))
    if ns.summary:
        with open(ns.summary, "a") as f:
            f.write("\n".join(lines) + "\n")
    if total["executed"] == 0:
        print(f"count_ci_tests: zero test cases executed ({total['discovered']} discovered, {total['skipped']} skipped): "
              "a suite that runs no test fails", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
