#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Every skip, xfail and importorskip names a tracking issue on the same statement (shared rule, openAMRobot/.github AGENTS.md, Tests).

    check_skip_tracking.py [--root DIR] [PATH ...]

Scans the Python test files under PATH (default: ros2 and tools, relative to --root, default: the repository) with `ast` and lists every
statement that skips or expects a failure without an issue reference (`#<number>`, as in `openAMRobot/openamr-upperbody-sw#16`) in the lines
the statement spans or in a comment at the end of its last line. A skip is not a pass, and a skip nobody tracks is a skip nobody reads.

Recognised: `pytest.skip`, `pytest.importorskip`, `pytest.xfail`, `pytest.mark.skip/skipif/xfail`, `self.skipTest`, `unittest.skip`,
`unittest.skipIf`, `unittest.skipUnless` and `unittest.expectedFailure` (the last has no reason to carry the number: it needs a comment).

Exit status: 0 nothing untracked, 1 at least one untracked skip, 2 a file cannot be parsed. No ROS is needed; this checks the text of the tests,
not their results.
"""
import argparse
import ast
import os
import re
import sys

TRACKING = re.compile(r"#\d+")
SKIP_NAMES = {"skip", "skipif", "skipunless", "skiptest", "importorskip", "xfail", "expectedfailure"}
DEFAULT_PATHS = ("ros2", "tools")


def _name(node):
    """Last attribute or name of a call's function or of a decorator: `pytest.mark.skipif` -> `skipif`."""
    if isinstance(node, ast.Call):
        node = node.func
    if isinstance(node, ast.Attribute):
        return node.attr
    if isinstance(node, ast.Name):
        return node.id
    return ""


def untracked(path):
    """[(line, text)] of the skips in one Python file that name no issue. Raises SyntaxError for a file that does not parse."""
    with open(path, encoding="utf-8") as handle:
        source = handle.read()
    lines = source.splitlines()
    found = []
    for node in ast.walk(ast.parse(source, filename=path)):
        if not isinstance(node, ast.Call) or _name(node).lower() not in SKIP_NAMES:
            continue
        if isinstance(node.func, ast.Name) and node.func.id == "skip":
            continue  # a local helper called skip(); pytest.skip and unittest.skip are attributes
        span = "\n".join(lines[node.lineno - 1:node.end_lineno])
        if not TRACKING.search(span):
            found.append((node.lineno, lines[node.lineno - 1].strip()))
    for node in ast.walk(ast.parse(source, filename=path)):  # bare decorators: @unittest.expectedFailure, @pytest.mark.skip
        for decorator in getattr(node, "decorator_list", []):
            if not isinstance(decorator, ast.Call) and _name(decorator).lower() in SKIP_NAMES:
                span = "\n".join(lines[decorator.lineno - 1:decorator.end_lineno])
                if not TRACKING.search(span):
                    found.append((decorator.lineno, lines[decorator.lineno - 1].strip()))
    return sorted(set(found))


def python_files(root, paths):
    for path in paths:
        base = os.path.join(root, path)
        if os.path.isfile(base):
            yield base
            continue
        for directory, names, files in os.walk(base):
            names[:] = [n for n in names if n not in (".git", "build", "install", "log", "__pycache__")]
            for name in sorted(files):
                if name.endswith(".py"):
                    yield os.path.join(directory, name)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--root", default=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    parser.add_argument("paths", nargs="*", default=list(DEFAULT_PATHS))
    args = parser.parse_args(argv)
    bad, status = 0, 0
    for path in python_files(args.root, args.paths):
        if os.path.abspath(path) == os.path.abspath(__file__):
            continue
        try:
            for line, text in untracked(path):
                print(f"{os.path.relpath(path, args.root)}:{line}: skip without a tracking issue: {text}")
                bad += 1
        except SyntaxError as error:
            print(f"{path}: cannot parse: {error}", file=sys.stderr)
            status = 2
    print(f"{bad} skip(s) without a tracking issue")
    return status or (1 if bad else 0)


if __name__ == "__main__":
    sys.exit(main())
