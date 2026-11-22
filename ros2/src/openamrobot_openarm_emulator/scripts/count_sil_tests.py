#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Counts of the OpenArm emulator tests from the JUnit XML that `colcon test` leaves, with BLOCKED kept apart from passed.

    count_sil_tests.py RESULTS_DIR [--summary FILE]

Every <testcase> is one of
  passed    ran and did not fail
  failed    a <failure> or <error> child
  BLOCKED   skipped with a message that says BLOCKED: the test could not run here (no vcan interface, no driver)
  skipped   skipped for another stated reason

A BLOCKED test is never counted as passed. Exit status: 1 a test failed or none ran; 3 none failed and at least one is
BLOCKED; 0 otherwise. CI bookkeeping only.
"""
import argparse
import collections
import glob
import os
import sys
import xml.etree.ElementTree as ET


def classify(case):
    if case.find("failure") is not None or case.find("error") is not None:
        return "failed", ""
    skipped = case.find("skipped")
    if skipped is not None:
        text = " ".join(filter(None, [skipped.get("message"), skipped.text])).strip()
        return ("BLOCKED" if "BLOCKED" in text else "skipped"), text
    return "passed", ""


def count(results_dir):
    """-> ({file: Counter}, [(file, test, text)] of BLOCKED and skipped cases)."""
    per_file, notes = collections.OrderedDict(), []
    for path in sorted(glob.glob(os.path.join(results_dir, "*.xml"))):
        counts = collections.Counter(passed=0, failed=0, BLOCKED=0, skipped=0)
        for case in ET.parse(path).getroot().iter("testcase"):
            kind, text = classify(case)
            counts[kind] += 1
            if kind in ("BLOCKED", "skipped"):
                notes.append((os.path.basename(path), case.get("name"), text.splitlines()[-1] if text else ""))
        per_file[os.path.basename(path)] = counts
    return per_file, notes


def exit_status(total):
    if total["failed"] or not total["passed"]:
        return 1
    return 3 if total["BLOCKED"] else 0


def render(per_file, notes):
    total = collections.Counter()
    lines = ["| result file | passed | failed | BLOCKED | skipped |", "|---|---|---|---|---|"]
    for name, counts in per_file.items():
        total.update(counts)
        lines.append(f"| {name} | {counts['passed']} | {counts['failed']} | {counts['BLOCKED']} | {counts['skipped']} |")
    lines.append(f"| **total** | {total['passed']} | {total['failed']} | {total['BLOCKED']} | {total['skipped']} |")
    if notes:
        lines += ["", "Not run:"] + [f"- {name} {test}: {text}" for name, test, text in notes]
    return total, "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("results_dir")
    parser.add_argument("--summary")
    args = parser.parse_args(argv)
    per_file, notes = count(args.results_dir)
    total, text = render(per_file, notes)
    print(text)
    if args.summary:
        with open(args.summary, "a") as f:
            f.write("### OpenArm emulator tests\n\n" + text + "\n")
    status = exit_status(total)
    print(f"exit status {status} (0 pass, 1 a test failed or none ran, 3 BLOCKED)")
    return status


if __name__ == "__main__":
    sys.exit(main())
