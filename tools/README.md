# tools

Scripts that are not part of a ROS package.

## Two prototype checks (P22)

Both are **run by hand**. Neither is called by `verify_a6.sh`, `run_a7_regression.sh`, a workflow or any other repository's CI, and neither
touches a file it reads. Python 3 with PyYAML (`python3-yaml`) is enough; no ROS is needed.

### `check_decision_currency.py`: does a document cite the revision the register records?

```bash
git clone https://github.com/openAMRobot/.github /tmp/harness        # or any checkout; the commit it prints is the one that was read
python3 tools/check_decision_currency.py --register /tmp/harness/decisions.yaml \
        --design-root /path/to/design/10_architecture                  # or --repo-only
python3 tools/check_decision_currency.py --register /tmp/harness/decisions.yaml --canary
python3 tools/check_decision_currency.py --register /tmp/harness/decisions.yaml --discover
```

`docs/decision_citations.yaml` lists (document, decision ID, revision cited), each with a verbatim `quote` so the list cannot drift from the text. The
check compares the revision with the `source.document` the register records for that decision. States: `CURRENT`, `STALE`, `AHEAD`, `SUPERSEDED`,
`HISTORICAL` (kept on purpose, with a reason), `UNKNOWN_*`, `QUOTE_*`, `UNLISTED` (a living document names a hyphenated decision ID next to a revision and
the list has no entry). Exit 0 clean, 1 a failure, 2 unreadable input, 3 design documents not checked.

What it does **not** prove: that the value a document states is the register's value (the register's own `tools/check_decisions.py` scans for
contradicting phrases), or anything about a citation that names no revision. Citations that name an item number and no register ID
(`P-03 rev19.0 item 19 D2`) are not covered.

`--canary` plants faults in temporary copies (a stale citation, an unknown decision, a quote that is not in the document, a list that disagrees with the
quote, an unlisted citation, and a real document of this repository with one citation re-dated) and exits 0 only if every one is caught.

### `check_evidence_backing.py`: does every count and rate resolve to a file under `evidence/`?

```bash
python3 tools/check_evidence_backing.py                  # docs/pr/PR14_BODY.md and the newest committed A6 and A7 reports
python3 tools/check_evidence_backing.py -v               # the backed claims too
python3 tools/check_evidence_backing.py --report evidence/reports/<new>.json
python3 tools/check_evidence_backing.py --skeleton       # ledger entries for what nothing backs yet
python3 tools/check_evidence_backing.py --canary
```

A claim is a verdict count (`25 PASS / 3 WARN / 0 FAIL`), a rate (`2 of 36`, `about 1 in 13`), a count (`826 tests`) or a percentage. It is `BACKED` (a
committed report has it, or `docs/pr/claim_backing.yaml` says how to derive it from committed reports), `RECORDED` (a written record under `evidence/`
says it: weaker, and labelled so), `UNBACKED` with an acknowledged reason, `OBSERVED-NOT-RECORDED` (seen, never recorded, and labelled so in the text), or `UNBACKED` without a reason, which fails. The table rows of
`evidence/reports/README.md` are read for one thing: the report files named in a row must have the verdict counts the row states.

What it does **not** prove: that a number is right (a record written by us is only as good as we were), and it does not see numbers that are not counts of
the nouns it knows. Exit 0 clean, 1 a failure, 2 unreadable input.

### Tests

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest tools/tests -q -p no:cacheprovider
```

They use small registers and trees of their own; they do not read the real `decisions.yaml`.

### Wiring them in, if someone wants to (not done)

Run `--canary` first in the same job (a check that cannot fail is not a check) and pin the register to a harness commit (the same SHA as the caller's
`harness_ref`). Neither tool should be added to another repository's workflow. Since P23 the currency check is clean on the real list (0 STALE): the
`datums.yaml` citation that P22 left red was re-cited with the revision bump to `2026-10-20.a`.

Since P24 an A7 report carries `process_exits` (the exit code of every process of every launch), and the ledger kind `launch_exits` derives "N of M launches with a `parameter_bridge` ended in -11" from it: BACKED for runs since P24, nothing for earlier reports.

The backing check has a fourth status besides BACKED, RECORDED and UNBACKED: `OBSERVED-NOT-RECORDED`, for a number someone saw (a console line) and nothing
recorded. It is allowed only while the text on the claim's line says OBSERVED-NOT-RECORDED.

## `compare_option_b_wrapper.sh`

The byte-level comparison of option B's wrapper against `upstream/main` that the PR body cites.

## `count_ci_tests.py`: the test counts and zero-executed gate of the Model tests workflow

Called by both jobs of `.github/workflows/model-tests.yml` after `verify_a6.sh` or `run_a7_regression.sh`; it changes neither script's exit status.
It counts `<testcase>` elements in `build/<pkg>/test_results/<pkg>/*.xml` as discovered, skipped, executed (discovered minus skipped) and failed, and
exits 1 when nothing executed (no result file, no case, or every case skipped), 2 when a result file cannot be read. `colcon test-result`'s total is
discovered plus one entry per registered CTest test; the script prints both. `tests/test_count_ci_tests.py` covers the empty, all-skipped and mixed
cases, and each job runs it before counting. CI bookkeeping only, not evidence of anything beyond the run it counts.

## P-03 item 24 equipment (1 Nov 2026)

Both need the built workspace (`source /opt/ros/jazzy/setup.bash && source <ws>/install/setup.bash`). Neither is called by `verify_a6.sh`
or a workflow; the export is also run by a test of the description package (`test_item24_docs_model_values.py`).

### `report_equipment_collisions.py`: REPORT ONLY, the moving equipment against both arms

```bash
python3 tools/report_equipment_collisions.py [--json out.json]
```

bp000 and bp050, the tablet as the model draws it (portrait on a fixed mount, the only position since P-03 rev20.4 item 28; the
landscape-plus-portrait variant and its assumed pivot are gone), lift stops L1000 / L1175 / L1350, the default state and every named arm
pose of the SRDF. Every (moving equipment link, arm link) pair is checked with the SRDF's list ignored (`acm_scene`). It moves nothing and
decides nothing. A control (the flag on its rod, touching by design) shows the check sees a contact. Exit 0 whatever it finds, 2 when it could not run.

### `export_docs_model_values.py`: the URDF, written as the docs model's CHEST-JSON and LIFT-JSON

```bash
python3 tools/export_docs_model_values.py [--urdf expanded.urdf] [--base-plate bp000]
```

Reads the expanded URDF (poses, box and cylinder sizes) and prints the two blocks in the format of openamrobot-docs
`docs/assets/roadmap/model/model.js`. Not exported, because the URDF does not carry them: `second_estop.glb_node`, `second_estop.glb_axis`,
`drawn.head_light.window` and LIFT-JSON `corner_x` (the rod's corner bracket). Every exported value is read from the URDF.
`test_item24_docs_model_values.py` compares the output with a verbatim fixture of the blocks (P-03 rev20.4 item 28: docs branch
`roadmap/2026-11-08-owner-decisions`, not yet merged, on `8d743e2`), 0.5 mm tolerance; `--model-js <model.js>` compares with a live file (exit 1 on a difference).

### `v01_rear_captures/`: three rear views for V01

`capture_rear.py <outdir>` takes one RViz shot from behind (and a little to the left) per lift stop at bp000, so the second E-stop on the
moving adapter plate (button up) and the stop sensors on the static column's rear face are in view (P-03 rev20.4 item 28). It reuses the P19 script `evidence/renders/rviz/capture_rviz.py`
unchanged, pointed at this directory (`capture.launch.py` with `rear_capture.rviz`; `xgrab.py` runs the P19 grabber). The a7 job runs it after
the six P19 views and uploads both with the artifact `v01-rviz-captures` (rear views under `rear/`). Pictures for a person, not evidence.

### `report_hand_clearance.py`: REPORT ONLY, hand to base top cover at L1000, rest pose

```bash
python3 tools/report_hand_clearance.py [--json out.json]
```

bp000 and bp050, lift at 0 (L1000), every arm and finger joint at 0 (the stack's initial state: SRDF `home` + `open`). For each hand (the gripper
links with collision geometry) it reports the minimum distance to the base top cover twice: MoveIt's distance to base_link's collision box (top
0.295 m, what planning sees; SRDF not consulted) and the exact triangle distance to the visual cover mesh `base_covers.stl` (0 when the
meshes cross; vertex-face and edge-edge otherwise), with the height gap as a certified lower bound. Exit 0 whatever it finds.
