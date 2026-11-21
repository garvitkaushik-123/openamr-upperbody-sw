# SPDX-License-Identifier: MIT
"""The organisation's merge-marker scan, run here before CI does (P27, run 39575711096).

Where the scan lives: NOT in this repository. `.github/workflows/repository-quality.yml` calls
`openAMRobot/.github/.github/workflows/repository-quality-reusable.yml@main`, whose step "Reject unresolved merge markers" is

    git grep -nE '^(<<<<<<<|=======|>>>>>>>)' -- ':!*.patch' ':!*.diff' ':!*.md' ':!**/*.md'

A line that merely STARTS with seven `=` matches, so pytest's summary separator ("============ 5 passed in 0.09s ============") in a committed log
turned the check red. We cannot change that workflow; this test runs its exact command over the tracked files, so the next such line fails here, and it
shows with a canary (a throw-away git repository) what the scan catches, what it wrongly catches, and what the narrower pattern the organisation could
adopt (`^(<{7}|={7}|>{7})( |$)`: exactly seven characters, then a space or the end of the line) catches and leaves alone. It changes nothing about the
organisation's scan, and it does not weaken ours: real markers fail under both patterns.
"""
import os
import subprocess

import pytest

ORG_PATTERN = r"^(<<<<<<<|=======|>>>>>>>)"
NARROW_PATTERN = r"^(<{7}|={7}|>{7})( |$)"
EXCLUDES = [":!*.patch", ":!*.diff", ":!*.md", ":!**/*.md"]
HERE = os.path.dirname(os.path.abspath(__file__))
SEPARATOR = "=" * 30 + " 5 passed in 0.09s " + "=" * 31


def git_grep(repo, pattern):
    """(exit status, output): 0 = matches found, 1 = none."""
    result = subprocess.run(["git", "grep", "-nE", pattern, "--", *EXCLUDES], cwd=repo, capture_output=True, text=True)
    assert result.returncode in (0, 1), result.stderr
    return result.returncode, result.stdout


def toplevel():
    result = subprocess.run(["git", "rev-parse", "--show-toplevel"], cwd=HERE, capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 else None


def test_no_tracked_file_trips_the_organisations_merge_marker_scan():
    repo = toplevel()
    if repo is None:
        pytest.skip("not a git checkout: the organisation's scan runs over a checkout")  # tracking: openAMRobot/openamr-upperbody-sw#16
    status, output = git_grep(repo, ORG_PATTERN)
    assert status == 1, "the organisation's `Reject unresolved merge markers` step would fail on:\n" + output


@pytest.fixture
def repo(tmp_path):
    def run(*args):
        subprocess.run(["git", *args], cwd=tmp_path, check=True, capture_output=True)
    run("init", "-q")
    run("config", "user.email", "canary@example.invalid")
    run("config", "user.name", "canary")
    return tmp_path, run


def commit(run, tmp_path, name, text):
    (tmp_path / name).parent.mkdir(parents=True, exist_ok=True)
    (tmp_path / name).write_text(text)
    run("add", name)


@pytest.mark.parametrize("marker", ["<<<<<<< HEAD", "=======", ">>>>>>> feature", "<<<<<<<", ">>>>>>>"])
def test_canary_a_real_conflict_marker_is_caught_by_both_patterns(repo, marker):
    tmp_path, run = repo
    commit(run, tmp_path, "evidence/x/result.txt", f"keep\n{marker}\nkeep\n")
    assert git_grep(tmp_path, ORG_PATTERN)[0] == 0
    assert git_grep(tmp_path, NARROW_PATTERN)[0] == 0


def test_canary_pytest_separator_trips_the_organisations_pattern_and_not_the_narrow_one(repo):
    tmp_path, run = repo
    commit(run, tmp_path, "evidence/x/pytest_output.txt", f"collected 5 items\n{SEPARATOR}\n")
    assert git_grep(tmp_path, ORG_PATTERN)[0] == 0           # why run 39575711096 was red
    assert git_grep(tmp_path, NARROW_PATTERN)[0] == 1        # exactly seven, then a space or the end of the line: a long run does not match


def test_canary_the_same_text_in_a_markdown_file_is_outside_the_scan(repo):
    tmp_path, run = repo
    commit(run, tmp_path, "evidence/x/pytest_output.md", f"collected 5 items\n{SEPARATOR}\n")
    assert git_grep(tmp_path, ORG_PATTERN)[0] == 1


def test_canary_a_real_marker_in_a_markdown_file_is_also_outside_the_organisations_scan(repo):
    """Said plainly: the scan skips *.md, so the rename of the P24 log does not make it a place where a conflict is caught. A conflict in a markdown
    file is not detected by this scan before or after P27."""
    tmp_path, run = repo
    commit(run, tmp_path, "docs/x.md", "<<<<<<< HEAD\nsomething\n=======\nother\n>>>>>>> branch\n")
    assert git_grep(tmp_path, ORG_PATTERN)[0] == 1
