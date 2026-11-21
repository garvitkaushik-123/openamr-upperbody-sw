# SPDX-License-Identifier: MIT
"""Revision guard: datums.yaml content must not change without `revision` changing (ADR-002: revision feeds config_id).

Compares the working-tree file with the same path at HEAD~. Skips cleanly (with the reason) outside a git checkout, in
a checkout with no parent commit, or when the file did not exist at HEAD~. Content means the parsed YAML, so comments
and whitespace do not count as a change.
"""
import os
import subprocess

import pytest
import yaml

from conftest import DATUMS_FILE


def revision_problem(old_text, new_text):
    """None if fine; otherwise why the change is not allowed."""
    old, new = yaml.safe_load(old_text), yaml.safe_load(new_text)
    if old == new:
        return None
    if old.get("revision") == new.get("revision"):
        return f"datums.yaml content changed but revision is still {new.get('revision')!r}: bump it"
    return None


def _git(cwd, *args):
    return subprocess.run(["git", "-C", cwd, *args], capture_output=True, text=True)


def check_against_parent(datums_path):
    """('ok'|'fail'|'skip', message) for the working-tree file at `datums_path` versus HEAD~."""
    path = os.path.realpath(datums_path)
    cwd = os.path.dirname(path)
    try:
        top = _git(cwd, "rev-parse", "--show-toplevel")
    except FileNotFoundError:
        return "skip", "git is not installed"
    if top.returncode != 0:
        return "skip", "not inside a git checkout"
    rel = os.path.relpath(path, os.path.realpath(top.stdout.strip()))
    if _git(cwd, "rev-parse", "--verify", "--quiet", "HEAD~").returncode != 0:
        return "skip", "no HEAD~ (single-commit or shallow checkout)"
    old = _git(cwd, "show", f"HEAD~:{rel}")
    if old.returncode != 0:
        return "skip", f"{rel} does not exist at HEAD~ (new file)"
    with open(path) as f:
        problem = revision_problem(old.stdout, f.read())
    return ("fail", problem) if problem else ("ok", "datums.yaml unchanged or revision bumped vs HEAD~")


def test_revision_guard_against_head_parent():
    status, message = check_against_parent(DATUMS_FILE)
    if status == "skip":
        pytest.skip(message)  # tracking: openAMRobot/openamr-upperbody-sw#16
    assert status == "ok", message


# --- the guard itself, on throw-away repositories ------------------------------------------------------------

BASE = 'schema_version: 1\nrevision: "2026-10-08.a"\ndatums:\n  x: {value: 1.0}\n'


def _commit(repo, text, msg):
    (repo / "config").mkdir(exist_ok=True)
    (repo / "config" / "datums.yaml").write_text(text)
    subprocess.run(["git", "-C", str(repo), "add", "-A"], check=True)
    subprocess.run(["git", "-C", str(repo), "-c", "user.name=t", "-c", "user.email=t@example.invalid",
                    "commit", "-q", "-m", msg], check=True)


@pytest.fixture
def repo(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    return tmp_path


@pytest.mark.parametrize("new_text,expected", [
    (BASE.replace("1.0", "2.0"), "fail"),                                              # value changed, same revision
    (BASE.replace("1.0", "2.0").replace(".a", ".b"), "ok"),                            # value changed, revision bumped
    (BASE.replace("x: {", "y: {"), "fail"),                                            # datum renamed
    (BASE + "pending_reference: {x: S4}\n", "fail"),                                   # key added
    ("# a comment\n" + BASE, "ok"),                                                    # comment only
    (BASE.replace("{value: 1.0}", "{ value: 1.0 }"), "ok"),                            # whitespace only
    (BASE, "ok"),                                                                      # unchanged
])
def test_guard_in_a_git_checkout(repo, new_text, expected):
    _commit(repo, BASE, "first")
    _commit(repo, new_text if new_text != BASE else BASE + "\n", "second")             # HEAD~ = BASE
    status, message = check_against_parent(str(repo / "config" / "datums.yaml"))
    assert status == expected, message


def test_guard_skips_with_a_single_commit(repo):
    _commit(repo, BASE, "first")
    assert check_against_parent(str(repo / "config" / "datums.yaml"))[0] == "skip"


def test_guard_skips_when_the_file_is_new_at_head(repo):
    _commit(repo, "x: 1\n", "first")
    (repo / "x.txt").write_text("y")
    _commit(repo, "x: 1\n", "second")
    (repo / "config").mkdir(exist_ok=True)
    other = repo / "config" / "other.yaml"
    other.write_text(BASE)
    assert check_against_parent(str(other))[0] == "skip"


def test_guard_skips_outside_a_git_checkout(tmp_path):
    f = tmp_path / "datums.yaml"
    f.write_text(BASE)
    status, message = check_against_parent(str(f))
    assert status == "skip" and "git" in message
