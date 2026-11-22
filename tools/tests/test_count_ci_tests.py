# SPDX-License-Identifier: MIT
"""tools/count_ci_tests.py on small workspaces of its own: the zero-executed-test gate of the Model tests workflow."""
import json

import count_ci_tests as cct

PKG = "pkg_a"


def case(name, outcome=None):
    child = {"skipped": '<skipped message="gated"/>', "failure": '<failure message="x"/>', "error": '<error message="x"/>'}.get(outcome, "")
    return f'<testcase classname="t" name="{name}">{child}</testcase>'


def workspace(tmp_path, files=None, ctest=0, results=("PASS", "WARN")):
    """A workspace with build/pkg_a/test_results/pkg_a/<name>.xml and a CTestTestfile.cmake with `ctest` add_test lines."""
    results_dir = tmp_path / "build" / PKG / "test_results" / PKG
    results_dir.mkdir(parents=True)
    for name, text in (files or {}).items():
        (results_dir / name).write_text(text)
    (tmp_path / "build" / PKG / "CTestTestfile.cmake").write_text(
        "".join(f'add_test(t{i} "/usr/bin/python3")\nset_tests_properties(t{i} PROPERTIES TIMEOUT 60)\n' for i in range(ctest)))
    (tmp_path / "evidence").mkdir()
    (tmp_path / "evidence" / "20261030T000000Z_dev.json").write_text(json.dumps(
        {"git_hash": "a" * 40, "evidence_class": "static+mock", "checks": [{"id": f"C{i}", "result": r} for i, r in enumerate(results)]}))
    return tmp_path


def suite(*cases, attrs=""):
    return f'<testsuite name="s" {attrs}>{"".join(cases)}</testsuite>'


def run(ws, capsys, summary=None):
    argv = ["--workspace", str(ws), "--label", "A6", "--evidence-glob", "evidence/*_dev.json", "--exit-status", "0", PKG]
    if summary:
        argv[0:0] = ["--summary", str(summary)]
    rc = cct.main(argv)
    out = capsys.readouterr()
    return rc, out.out, out.err


def test_no_result_file_is_zero_executed_and_fails(tmp_path, capsys):
    rc, out, err = run(workspace(tmp_path), capsys)
    assert rc == 1
    assert "0 discovered, 0 skipped, 0 executed" in out and "zero test cases executed" in err


def test_a_suite_with_no_testcase_fails(tmp_path, capsys):
    rc, _, err = run(workspace(tmp_path, {"empty.xml": suite(attrs='tests="0"')}), capsys)
    assert rc == 1 and "zero test cases executed" in err


def test_all_skipped_is_zero_executed_and_fails_whatever_the_tests_attribute_says(tmp_path, capsys):
    """The case the first version let through: it summed `tests` (5) and rejected only 0."""
    xml = suite(*(case(f"t{i}", "skipped") for i in range(5)), attrs='tests="5" skipped="5" failures="0" errors="0"')
    rc, out, err = run(workspace(tmp_path, {"skipped.xml": xml}), capsys)
    assert rc == 1
    assert "5 discovered, 5 skipped, 0 executed" in out and "zero test cases executed (5 discovered, 5 skipped)" in err


def test_mixed_counts_each_outcome_and_passes(tmp_path, capsys):
    files = {"a.xml": suite(case("p1"), case("p2"), case("s1", "skipped"), case("f1", "failure")),
             "b.xml": f'<?xml version="1.0"?><testsuites>{suite(case("p3"), case("e1", "error"))}{suite(case("s2", "skipped"))}</testsuites>'}
    rc, out, _ = run(workspace(tmp_path, files, ctest=2), capsys)
    assert rc == 0
    assert f"- {PKG}: 7 / 2 / 5 / 2; 2 ctest entries, 2 result files" in out
    assert "**Total: 7 discovered, 2 skipped, 5 executed, 2 failed** (colcon total = discovered + 2 ctest entries = 9)" in out


def test_counts_cases_not_the_tests_attribute(tmp_path, capsys):
    xml = suite(case("p1"), case("s1", "skipped"), attrs='tests="40" skipped="0"')
    rc, out, _ = run(workspace(tmp_path, {"a.xml": xml}), capsys)
    assert rc == 0 and "2 discovered, 1 skipped, 1 executed" in out


def test_a_failed_case_is_reported_but_the_verifier_decides_the_job(tmp_path, capsys):
    rc, out, _ = run(workspace(tmp_path, {"a.xml": suite(case("f1", "failure"))}), capsys)
    assert rc == 0 and "1 discovered, 0 skipped, 1 executed, 1 failed" in out


def test_an_unreadable_result_file_is_exit_2(tmp_path, capsys):
    rc, _, err = run(workspace(tmp_path, {"bad.xml": "<testsuite"}), capsys)
    assert rc == 2 and "cannot read the results" in err


def test_evidence_and_summary_file(tmp_path, capsys):
    summary = tmp_path / "summary.md"
    rc, out, _ = run(workspace(tmp_path, {"a.xml": suite(case("p1"))}), capsys, summary=summary)
    assert rc == 0
    assert "20261030T000000Z_dev.json: git " + "a" * 40 + ", evidence_class static+mock, PASS 1, WARN 1" in out and "- C1 WARN" in out
    assert summary.read_text().strip() == out.strip()


# --- the lift guard's exits: UNVERIFIED counted apart, never as PASS (owner decision on upperbody-sw #53) --------------------------------
def guard_run(ws, capsys, log):
    argv = ["--workspace", str(ws), "--label", "A7", "--evidence-glob", "evidence/*_dev.json", "--exit-status", "0",
            "--guard-exit-log", log, PKG]
    rc = cct.main(argv)
    out = capsys.readouterr()
    return rc, out.out


def exits(*outcomes):
    return "".join(json.dumps({"outcome": o}) + "\n" for o in outcomes)


def test_guard_exits_are_counted_apart_and_unverified_is_a_warn(tmp_path, capsys):
    ws = workspace(tmp_path, {"a.xml": suite(case("t1"))})
    (ws / "guard_exits.jsonl").write_text(exits("confirmed", "confirmed", "manager_gone", "manager_gone", "manager_gone", "unanswered"))
    rc, out = guard_run(ws, capsys, "guard_exits.jsonl")
    assert rc == 0
    assert "lift guard exits: 6: 2 confirmed, **4 UNVERIFIED (WARN, not PASS)**, 0 failed" in out
    assert "- manager_gone: 3" in out and "- unanswered: 1" in out


def test_a_failed_or_stop_unanswered_exit_is_counted_as_failed_never_as_unverified(tmp_path, capsys):
    ws = workspace(tmp_path, {"a.xml": suite(case("t1"))})
    (ws / "guard_exits.jsonl").write_text(exits("confirmed", "stop_unanswered", "failed"))
    _, out = guard_run(ws, capsys, "guard_exits.jsonl")
    assert "3: 1 confirmed, **0 UNVERIFIED (WARN, not PASS)**, 2 failed" in out


def test_a_missing_or_empty_guard_log_is_said_not_read_as_zero_unverified(tmp_path, capsys):
    ws = workspace(tmp_path, {"a.xml": suite(case("t1"))})
    _, out = guard_run(ws, capsys, "guard_exits.jsonl")
    assert "no log at" in out and "none counted as confirmed" in out
    (ws / "guard_exits.jsonl").write_text("")
    _, out = guard_run(ws, capsys, "guard_exits.jsonl")
    assert "is empty" in out and "UNVERIFIED" not in out


def test_without_the_option_the_summary_has_no_guard_section(tmp_path, capsys):
    ws = workspace(tmp_path, {"a.xml": suite(case("t1"))})
    _, out, _ = run(ws, capsys)
    assert "lift guard exits" not in out
