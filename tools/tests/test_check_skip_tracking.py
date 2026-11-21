# SPDX-License-Identifier: MIT
"""tools/check_skip_tracking.py on small files of its own: a skip, xfail or importorskip names an issue or the check fails."""
import check_skip_tracking as cst

TRACKED = "openAMRobot/openamr-upperbody-sw#16"


def write(tmp_path, body, name="test_x.py"):
    path = tmp_path / name
    path.write_text(body)
    return path


def lines(tmp_path, body):
    return [line for line, _ in cst.untracked(write(tmp_path, body))]


def test_a_bare_skip_is_reported(tmp_path):
    assert lines(tmp_path, 'import pytest\n\ndef test_a():\n    pytest.skip("no checkout")\n') == [4]


def test_a_skip_that_names_an_issue_in_its_reason_passes(tmp_path):
    assert lines(tmp_path, f'import pytest\n\ndef test_a():\n    pytest.skip("no checkout (tracking: {TRACKED})")\n') == []


def test_a_trailing_comment_on_the_last_line_counts(tmp_path):
    body = f'import pytest\n\ndef test_a():\n    pytest.skip("a"\n                "b")  # tracking: {TRACKED}\n'
    assert lines(tmp_path, body) == []


def test_importorskip_skipif_skiptest_and_xfail_are_all_checked(tmp_path):
    body = ("import pytest, unittest\n"
            "np = pytest.importorskip('numpy')\n"                                  # 2
            "@pytest.mark.skipif(True, reason='x')\n"                              # 3
            "def test_a():\n    pass\n"
            "class T(unittest.TestCase):\n"
            "    @unittest.skipUnless(False, 'y')\n"                               # 7
            "    def test_b(self):\n        self.skipTest('z')\n"                  # 9
            "@pytest.mark.xfail\n"                                                 # 10
            "def test_c():\n    pass\n")
    assert lines(tmp_path, body) == [2, 3, 7, 9, 10]


def test_a_bare_expected_failure_decorator_is_reported_and_a_commented_one_passes(tmp_path):
    body = ("import unittest\n"
            "class T(unittest.TestCase):\n"
            "    @unittest.expectedFailure\n"                                      # 3
            "    def test_a(self):\n        pass\n"
            "    @unittest.expectedFailure  # tracking: openAMRobot/openamr-upperbody-sw#16\n"
            "    def test_b(self):\n        pass\n")
    assert lines(tmp_path, body) == [3]


def test_code_that_merely_mentions_skip_is_not_a_skip(tmp_path):
    assert lines(tmp_path, 'def skip_list():\n    return ["skip"]\n\nx = skip_list()\n') == []


def test_main_exit_status_and_report(tmp_path, capsys):
    write(tmp_path, 'import pytest\n\ndef test_a():\n    pytest.skip("x")\n')
    assert cst.main(["--root", str(tmp_path), "."]) == 1
    assert "test_x.py:4: skip without a tracking issue" in capsys.readouterr().out
    (tmp_path / "test_x.py").write_text(f'import pytest\n\ndef test_a():\n    pytest.skip("x {TRACKED}")\n')
    assert cst.main(["--root", str(tmp_path), "."]) == 0


def test_main_reports_a_file_that_does_not_parse(tmp_path, capsys):
    write(tmp_path, "def broken(:\n")
    assert cst.main(["--root", str(tmp_path), "."]) == 2
    assert "cannot parse" in capsys.readouterr().err
