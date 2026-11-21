# SPDX-License-Identifier: MIT
"""C02: every pin in config/upstream_pins.yaml agrees with its upstream checkout when there is one.

The openarm_description pin is the fallback that lets a reviewer with no `src/upstream/` compute the same `config_id` as
the author (C01). That only holds while the pin equals the commit a real checkout is at, so each pin is compared with its
checkout whenever both exist: equal is PASS, different is FAIL naming both hashes, a checkout with local changes is FAIL.
With no checkout for a pin that pin is not compared and the report lists it; with no checkout for any pin the result is
SKIP, with the reason, and that is the only legitimate SKIP in this tree.

The file iterates by entry (P09), not by naming openarm_description: a pin nothing checks is a pin that drifts, and
openarm_ros2 sat in the file as a 7-character hash that nothing compared. Every pin is a quoted full 40-hex string, so
the all-digit-YAML-number case and prefix comparison do not exist; both are still FAILs, because a file can regress.

The check compares the pin with the checkout, not with upstream's latest commit: holding both behind upstream on purpose
(they are pinned to the manipulation baseline) is not drift.

The three outcomes are demonstrated on synthetic checkouts in tmp_path, so they run whether or not the real checkout is
there; the test that records C02's own result for the evidence report runs against the real workspace.
"""
import os
import re
import subprocess

import pytest

import evidence_common as ev
from conftest import SHARE, write_sidecar

WORKSPACE = ev.workspace_root(SHARE)
CHECK_NAME = "upstream pin file agrees with the upstream checkout"


def _checkout(tmp_path):
    upstream = tmp_path / "src" / "upstream" / "openarm_description"
    upstream.mkdir(parents=True)
    git = lambda *a: subprocess.run(["git", "-C", str(upstream), *a], check=True, capture_output=True)  # noqa: E731
    git("init", "-q")
    git("-c", "user.name=t", "-c", "user.email=t@t", "commit", "-q", "--allow-empty", "-m", "x")
    return upstream, ev.git_head(str(upstream))


def _pins(tmp_path, commit, **others):
    """A pin file with openarm_description at `commit` (written as given: pass quotes to quote it) and any other pins."""
    path = tmp_path / "pins.yaml"
    lines = "".join(f"  {name}: {value}\n" for name, value in others.items())
    path.write_text(f"schema_version: 1\npins:\n  openarm_description: {commit}\n{lines}")
    return str(path)


def _second_checkout(tmp_path, name="openarm_ros2"):
    repo = tmp_path / "src" / "upstream" / name
    repo.mkdir(parents=True)
    git = lambda *a: subprocess.run(["git", "-C", str(repo), *a], check=True, capture_output=True)  # noqa: E731
    git("init", "-q")
    git("-c", "user.name=t", "-c", "user.email=t@t", "commit", "-q", "--allow-empty", "-m", "y")
    return repo, ev.git_head(str(repo))


def test_c02_the_real_pin_file_agrees_with_every_real_checkout_or_says_why_it_skips():
    result, details = ev.pins_vs_checkouts(WORKSPACE)
    write_sidecar("C02", "upstream_pin", CHECK_NAME, result, details)
    if result == "SKIP":
        pytest.skip(details["reason"])  # tracking: openAMRobot/openamr-upperbody-sw#16
    assert result == "PASS", details["reason"]
    for name in details["compared"]:
        assert details["pins"][name]["pin"] == details["pins"][name]["checkout_head"]


def test_c02_the_real_pin_file_is_normalised_every_pin_a_quoted_40_hex_string():
    """Read as text: after yaml.safe_load a quoted and an unquoted hash both look like a str (unless all digits)."""
    lines = [l for l in open(os.path.join(SHARE, "config", "upstream_pins.yaml")).read().splitlines()
             if re.match(r"^\s+\w+:\s", l) and not l.lstrip().startswith("#") and "schema_version" not in l]
    pins = ev.all_pins()
    assert pins and len(lines) == len(pins)
    for line in lines:
        assert re.fullmatch(r'\s+\w+: "[0-9a-f]{40}"', line), f"not a quoted full hash: {line!r}"
    assert all(isinstance(v, str) and ev.HEX40_RE.match(v) for v in pins.values())


def test_c02_the_real_pin_file_checks_openarm_ros2_too():
    """The pin that used to be a 7-character hash nothing checked is now iterated like any other."""
    assert "openarm_ros2" in ev.all_pins()
    _, details = ev.pins_vs_checkouts(WORKSPACE)
    assert "openarm_ros2" in details["pins"]


def test_c02_equal_pin_and_checkout_pass(tmp_path):
    _, head = _checkout(tmp_path)
    result, details = ev.pin_vs_checkout(str(tmp_path), pins_file=_pins(tmp_path, head))
    assert result == "PASS" and details["pin"] == details["checkout_head"] == head


def test_c02_a_wrong_pin_fails_and_names_both_hashes(tmp_path):
    _, head = _checkout(tmp_path)
    wrong = "deadbeef" * 5
    result, details = ev.pin_vs_checkout(str(tmp_path), pins_file=_pins(tmp_path, wrong))
    assert result == "FAIL"
    assert wrong in details["reason"] and head in details["reason"]
    assert (details["pin"], details["checkout_head"]) == (wrong, head)


def test_c02_an_unquoted_all_digit_pin_is_not_silently_compared_as_a_number(tmp_path):
    _, head = _checkout(tmp_path)
    result, details = ev.pin_vs_checkout(str(tmp_path), pins_file=_pins(tmp_path, "0" * 40))   # YAML reads this as 0
    assert result == "FAIL" and "not a string" in details["reason"] and details["checkout_head"] == head


def test_c02_a_wrong_pin_fails_against_the_real_checkout_too(tmp_path):
    real = ev.upstream_checkout(WORKSPACE, "openarm_description")
    if ev.git_head(real) is None:
        pytest.skip(f"no real checkout at {real}; the synthetic test above covers the same path")  # tracking: openAMRobot/openamr-upperbody-sw#16
    wrong = "f" * 40
    result, details = ev.pin_vs_checkout(WORKSPACE, pins_file=_pins(tmp_path, wrong))
    assert result == "FAIL" and wrong in details["reason"] and ev.git_head(real) in details["reason"]


def test_c02_an_absent_checkout_skips_and_says_it_is_the_only_legitimate_skip(tmp_path):
    result, details = ev.pin_vs_checkout(str(tmp_path), pins_file=_pins(tmp_path, "a" * 40))
    assert result == "SKIP"
    assert "no git checkout of openarm_description" in details["reason"]
    assert "only legitimate SKIP" in details["reason"]


def test_c02_a_directory_that_is_not_a_git_checkout_is_absent_not_a_pass(tmp_path):
    (tmp_path / "src" / "upstream" / "openarm_description").mkdir(parents=True)
    result, _ = ev.pin_vs_checkout(str(tmp_path), pins_file=_pins(tmp_path, "a" * 40))
    assert result == "SKIP"


def test_c02_a_dirty_checkout_fails_even_when_head_equals_the_pin(tmp_path):
    upstream, head = _checkout(tmp_path)
    (upstream / "local_edit.txt").write_text("x")
    result, details = ev.pin_vs_checkout(str(tmp_path), pins_file=_pins(tmp_path, head))
    assert result == "FAIL" and "local changes" in details["reason"]


def test_c02_a_checkout_with_no_pin_in_the_file_fails(tmp_path):
    _, head = _checkout(tmp_path)
    other = tmp_path / "other.yaml"
    other.write_text("schema_version: 1\npins:\n  something_else: abc\n")
    result, details = ev.pin_vs_checkout(str(tmp_path), pins_file=str(other))
    assert result == "FAIL" and "no pin for openarm_description" in details["reason"] and details["checkout_head"] == head


def test_c02_a_pin_held_behind_the_upstream_tip_is_not_drift(tmp_path):
    # the checkout is NOT advanced to a newer commit: a later upstream commit that nobody checked out changes nothing
    upstream, head = _checkout(tmp_path)
    subprocess.run(["git", "-C", str(upstream), "-c", "user.name=t", "-c", "user.email=t@t", "commit", "-q",
                    "--allow-empty", "-m", "newer"], check=True, capture_output=True)
    newer = ev.git_head(str(upstream))
    subprocess.run(["git", "-C", str(upstream), "checkout", "-q", head], check=True, capture_output=True)
    result, _ = ev.pin_vs_checkout(str(tmp_path), pins_file=_pins(tmp_path, head))
    assert head != newer and result == "PASS"


# --- every pin, not just openarm_description (P09) ------------------------------------------------------------------

def test_c02_two_clean_checkouts_both_equal_their_pins_pass_and_both_are_compared(tmp_path):
    _, head = _checkout(tmp_path)
    _, ros2_head = _second_checkout(tmp_path)
    result, details = ev.pins_vs_checkouts(str(tmp_path), pins_file=_pins(tmp_path, f'"{head}"', openarm_ros2=f'"{ros2_head}"'))
    assert result == "PASS" and sorted(details["compared"]) == ["openarm_description", "openarm_ros2"]
    assert details["not_compared"] == []


def test_c02_a_second_pin_that_drifted_fails_naming_that_pin_and_both_hashes(tmp_path):
    _, head = _checkout(tmp_path)
    _, ros2_head = _second_checkout(tmp_path)
    stale = "4e837e1" + "0" * 33
    result, details = ev.pins_vs_checkouts(str(tmp_path), pins_file=_pins(tmp_path, f'"{head}"', openarm_ros2=f'"{stale}"'))
    assert result == "FAIL"
    assert "openarm_ros2" in details["reason"] and stale in details["reason"] and ros2_head in details["reason"]
    assert details["pins"]["openarm_description"]["result"] == "PASS"


def test_c02_a_dirty_second_checkout_fails(tmp_path):
    _, head = _checkout(tmp_path)
    repo, ros2_head = _second_checkout(tmp_path)
    (repo / "edit.txt").write_text("x")
    result, details = ev.pins_vs_checkouts(str(tmp_path), pins_file=_pins(tmp_path, f'"{head}"', openarm_ros2=f'"{ros2_head}"'))
    assert result == "FAIL" and "openarm_ros2" in details["reason"] and "local changes" in details["reason"]


def test_c02_a_pin_without_a_checkout_is_listed_as_not_compared_not_silently_passed(tmp_path):
    _, head = _checkout(tmp_path)
    result, details = ev.pins_vs_checkouts(str(tmp_path), pins_file=_pins(tmp_path, f'"{head}"', openarm_ros2='"' + "a" * 40 + '"'))
    assert result == "PASS"
    assert details["compared"] == ["openarm_description"] and details["not_compared"] == ["openarm_ros2"]


def test_c02_no_checkout_for_any_pin_is_skip_and_names_each_pin(tmp_path):
    result, details = ev.pins_vs_checkouts(str(tmp_path), pins_file=_pins(tmp_path, '"' + "a" * 40 + '"', openarm_ros2='"' + "b" * 40 + '"'))
    assert result == "SKIP" and "openarm_description" in details["reason"] and "openarm_ros2" in details["reason"]


def test_c02_a_short_pin_fails_whether_or_not_a_checkout_exists(tmp_path):
    """The 7-character hash openarm_ros2 used to be: prefixes cannot be compared exactly and the fallback id needs a full one."""
    result, details = ev.pins_vs_checkouts(str(tmp_path), pins_file=_pins(tmp_path, '"' + "a" * 40 + '"', openarm_ros2="4e837e1"))
    assert result == "FAIL" and "openarm_ros2" in details["reason"] and "not a full 40-hex" in details["reason"]
    repo, ros2_head = _second_checkout(tmp_path)
    result, details = ev.pins_vs_checkouts(str(tmp_path), pins_file=_pins(tmp_path, '"' + "a" * 40 + '"', openarm_ros2=f'"{ros2_head[:7]}"'))
    assert result == "FAIL" and "not a full 40-hex" in details["reason"]


@pytest.mark.parametrize("bad", ['"' + "A" * 40 + '"', '"' + "a" * 41 + '"', '"' + "g" * 40 + '"', '""'])
def test_c02_a_malformed_pin_fails(tmp_path, bad):
    result, details = ev.pins_vs_checkouts(str(tmp_path), pins_file=_pins(tmp_path, bad))
    assert result == "FAIL" and "not a full 40-hex" in details["reason"]


def test_c02_an_unquoted_all_digit_second_pin_is_a_number_and_fails(tmp_path):
    result, details = ev.pins_vs_checkouts(str(tmp_path), pins_file=_pins(tmp_path, '"' + "a" * 40 + '"', openarm_ros2="1" * 40))
    assert result == "FAIL" and "not a string" in details["reason"] and "openarm_ros2" in details["reason"]


def test_c02_a_required_pin_missing_from_the_file_fails_when_its_checkout_exists(tmp_path):
    _checkout(tmp_path)
    only_other = tmp_path / "other.yaml"
    only_other.write_text('schema_version: 1\npins:\n  openarm_ros2: "' + "a" * 40 + '"\n')
    result, details = ev.pins_vs_checkouts(str(tmp_path), pins_file=str(only_other))
    assert result == "FAIL" and "no pin for openarm_description" in details["reason"]


def test_c02_an_empty_pin_file_does_not_crash_it_just_has_nothing_to_compare(tmp_path):
    empty = tmp_path / "empty.yaml"
    empty.write_text("")
    result, details = ev.pins_vs_checkouts(str(tmp_path), pins_file=str(empty))
    assert result == "SKIP" and details["not_compared"] == ["openarm_description"]
