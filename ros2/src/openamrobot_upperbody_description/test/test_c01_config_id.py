# SPDX-License-Identifier: MIT
"""C01: the configuration id (ARCHITECTURE.md s7) has the format the openamrobot-interfaces C9 contract requires, and
changes when, and only when, one of its inputs changes.

Format: `sha256:` + 64 lowercase hex. A 12-character truncation (the original s7) fails the C9 validator, so a test here
turns red if anyone brings it back. When openamrobot-interfaces is checked out in the workspace, the pattern is also read
from ITS schema (every field the C9 contract requires a configuration hash in) and the id is validated against that, so
a change on their side turns our test red instead of leaving us silently different. Without that checkout those two
tests SKIP with the reason, and C01's evidence says so. (That the package pin equals a present upstream checkout is C02.)
"""
import json
import os
import re
import subprocess
import sys

import jsonschema
import pytest

import evidence_common as ev
import print_config_id as pci
from conftest import BASELINE, SCRIPTS, SHARE, UPPERBODY, expand_xml, load_yaml, write_sidecar

WORKSPACE = ev.workspace_root(SHARE)
C9_SCHEMA = os.path.join(WORKSPACE, "src", "openamrobot-interfaces", "schemas", "json", "c9-lerobot-dataset.schema.json")
# where C9 requires a configuration hash: the schema path of each field, as a list of keys
C9_HASH_FIELDS = [
    ["$defs", "openAMRobot", "properties", "provenance", "properties", "robot_configuration_hash"],
    ["$defs", "cameraStream", "properties", "configuration_hash"],
    ["$defs", "stateStream", "properties", "configuration_hash"],
    ["$defs", "actionStream", "properties", "configuration_hash"],
]
COMMIT = "0123456789abcdef0123456789abcdef01234567"


def _commit():
    return ev.upstream_commit(WORKSPACE)


MATRIX = ev.expansion_matrix(UPPERBODY)
OTHER_PIN = "89abcdef0123456789abcdef0123456789abcdef"


def _row(**overrides):
    return {**BASELINE, "arm_package": "openarm", "hardware_profile": "mock", **overrides}


def _pins_with(commit=COMMIT, **others):
    """{name: commit} for every pin of upstream_pins.yaml, openarm_description at `commit`, the others at OTHER_PIN."""
    pins = {name: OTHER_PIN for name in ev.all_pins()}
    pins["openarm_description"] = commit
    pins.update(others)
    return pins


def _id(commit=COMMIT, planar="false", datums_file=None, pins=None, **row):
    return pci.config_id_of(SHARE, pins or _pins_with(commit), _row(**row), planar, datums_file)


# --- format -------------------------------------------------------------------------------------------------------

def test_c01_every_config_id_in_the_matrix_is_sha256_plus_64_lowercase_hex():
    pins, _ = pci.pins_of(WORKSPACE)
    ids = {ev.row_label(r): pci.config_id_of(SHARE, pins, r) for r in MATRIX}
    assert len(ids) == 14
    bad = {k: v for k, v in ids.items() if not ev.SHA256_RE.match(v)}
    assert not bad, bad
    assert all(len(v) == len("sha256:") + 64 for v in ids.values())
    assert len(set(ids.values())) == len(ids), "two different expansions share an id"


def test_c01_the_pattern_itself_rejects_the_old_twelve_character_form():
    full = _id()
    assert ev.SHA256_RE.match(full)
    assert not ev.SHA256_RE.match(full[:7 + 12])                       # sha256:<12 hex>
    assert not ev.SHA256_RE.match(full.split(":")[1])                  # no prefix
    assert not ev.SHA256_RE.match(full.upper().replace("SHA256", "sha256"))   # uppercase hex


def _resolve(schema, ref):
    assert ref.startswith("#/"), f"only local refs are followed: {ref}"
    node = schema
    for part in ref[2:].split("/"):
        node = node[part]
    return node


def _c9_hash_definitions():
    """[(field path, resolved JSON-schema node)] for every field C9 requires a configuration hash in."""
    with open(C9_SCHEMA) as f:
        schema = json.load(f)
    out = []
    for path in C9_HASH_FIELDS:
        node = schema
        for key in path:
            node = node[key]
        while "$ref" in node:
            node = _resolve(schema, node["$ref"])
        out.append(("/".join(path), node))
    return out


needs_c9 = pytest.mark.skipif(not os.path.isfile(C9_SCHEMA),
                              reason=f"openamrobot-interfaces is not in the workspace ({C9_SCHEMA} missing)")  # tracking: openAMRobot/openamr-upperbody-sw#16


@needs_c9
def test_c01_the_id_validates_against_the_pattern_in_the_c9_schema():
    definitions = _c9_hash_definitions()
    assert len(definitions) == len(C9_HASH_FIELDS)
    cid = pci.config_id_of(SHARE, pci.pins_of(WORKSPACE)[0], _row())
    for path, node in definitions:
        jsonschema.validate(cid, node)                                 # type + pattern, as their validator sees it
        assert re.search(node["pattern"], cid), path


@needs_c9
def test_c01_the_c9_validator_would_reject_the_old_twelve_character_id():
    cid = _id()
    for path, node in _c9_hash_definitions():
        with pytest.raises(jsonschema.ValidationError):
            jsonschema.validate(cid[:7 + 12], node)


@needs_c9
def test_c01_our_pattern_is_the_one_c9_publishes():
    # if they change their definition this fails, and says which side moved
    patterns = {node["pattern"] for _, node in _c9_hash_definitions()}
    assert patterns == {ev.SHA256_RE.pattern}, f"C9 now requires {patterns}; evidence_common.SHA256_RE is {ev.SHA256_RE.pattern}"


# --- inputs ---------------------------------------------------------------------------------------------------------

def test_c01_the_same_inputs_give_the_same_id_and_each_input_changes_it():
    base = _id()
    assert _id() == base
    assert _id(base_plate="bp050") != base                             # through the URDF, and as a member
    assert _id(head_pitch="hp15") != base
    assert _id(base_tilt="bt15") != base
    assert _id(arm_package="so101") != base
    assert _id(hardware_profile="gazebo") != base                      # an input in its own right, not only via the URDF
    assert _id(commit="f" * 40) != base                                # the pinned upstream commit
    assert _id(pins=_pins_with(openarm_ros2="e" * 40)) != base          # EVERY pin is an input, not only openarm_description
    assert _id(planar="true") != base                                  # through the URDF content


def test_c01_each_configuration_id_is_a_member_in_its_own_right():
    """Not only through the expanded URDF: two rows that differed only in a member would collide if the member were dropped.
    (`column` cannot be varied through the xacro while tl3_400 has no supplier data, so the function is called directly.)"""
    urdf = expand_xml()
    base = pci.config_id(urdf, _row(), "rev", _pins_with())
    for key, other in (("column", "tl3_400"), ("base_plate", "bp050"), ("head_pitch", "hp35"), ("base_tilt", "bt05"),
                       ("arm_package", "none"), ("hardware_profile", "real")):
        assert pci.config_id(urdf, _row(**{key: other}), "rev", _pins_with()) != base, key


def test_c01_the_lift_stop_is_a_calibration_id_and_not_an_input():
    """ADR-010 had the three lift stops as configuration IDs. Option B's file calls them calibration IDs, which they are: a
    joint position with a calibration, not a build variant. Nothing here may take one, and the id must not move with it."""
    import inspect
    assert "stop" not in str(inspect.signature(pci.config_id)) and "stop" not in str(inspect.signature(pci.config_id_of))
    assert not [k for k in ev.ROW_KEYS if "lift" in k or "stop" in k or k == "mast_id"]  # decision-allow: CONFIG-IDS negative test: mast_id must be absent or rejected
    out = _run("column:=dold_v4_350", "base_plate:=bp000", "head_pitch:=hp30", "base_tilt:=bt10", "arm_package:=openarm",
               "hardware_profile:=mock", "lift_stop:=L1350")
    assert out.returncode != 0 and "unrecognised argument" in out.stderr


def test_c01_the_input_set_is_the_one_the_prompt_names_and_mast_id_is_gone():
    """column x base_plate x head_pitch x base_tilt x arm_package x hardware_profile x datums revision x upstream pins (+ the
    canonical URDF). The member list is asserted from the serialisation, so the documented set cannot drift from the real one."""
    import hashlib
    urdf = expand_xml()
    members = {"canonical_urdf": pci.canonical_urdf(urdf), **_row(), "datums_revision": "r", "upstream_pins": dict(sorted(_pins_with().items()))}
    assert sorted(members) == sorted(["canonical_urdf", "column", "base_plate", "head_pitch", "base_tilt", "arm_package",
                                      "hardware_profile", "datums_revision", "upstream_pins"])
    assert "mast_id" not in members and "openarm_description_commit" not in members  # decision-allow: CONFIG-IDS negative test: mast_id must be absent or rejected
    payload = json.dumps(members, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    assert pci.config_id(urdf, _row(), "r", _pins_with()) == "sha256:" + hashlib.sha256(payload).hexdigest()


def test_c01_an_id_recorded_before_p15_is_not_comparable_with_one_computed_now():
    """The format is unchanged and still valid against C9, so no validator can tell. This test is where the break is stated:
    the old inputs (mast_id, openarm_description_commit) hash to a different value from the new ones for the same sources."""  # decision-allow: CONFIG-IDS negative test: mast_id must be absent or rejected
    import hashlib
    urdf = expand_xml()
    old = {"canonical_urdf": pci.canonical_urdf(urdf), "mast_id": "mast_1350", "arm_package": "openarm", "hardware_profile": "mock",  # decision-allow: CONFIG-IDS negative test: mast_id must be absent or rejected
           "datums_revision": "r", "openarm_description_commit": COMMIT}
    old_id = "sha256:" + hashlib.sha256(json.dumps(old, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode()).hexdigest()
    new_id = pci.config_id(urdf, _row(), "r", _pins_with())
    assert ev.SHA256_RE.match(old_id) and ev.SHA256_RE.match(new_id) and old_id != new_id


def test_c01_datums_revision_is_an_input(tmp_path):
    datums = load_yaml(os.path.join(SHARE, "config", "datums.yaml"))
    with open(os.path.join(SHARE, "config", "datums.yaml")) as f:
        text = f.read()
    original = _id()
    same = tmp_path / "same.yaml"
    same.write_text(text)
    assert _id(datums_file=str(same)) == original                       # same content, other path: same id
    bumped = tmp_path / "bumped.yaml"
    bumped.write_text(text.replace(f'revision: "{datums["revision"]}"', 'revision: "9999-01-01.z"'))
    assert bumped.read_text() != text
    assert _id(datums_file=str(bumped)) != original                     # only the revision differs, the numbers do not


def test_c01_comments_whitespace_and_the_autogenerated_path_do_not_change_the_id():
    urdf = expand_xml()
    args = (_row(), "rev", _pins_with())
    base = pci.config_id(urdf, *args)
    with_banner = urdf.replace("<robot ", "<!-- banner naming /some/other/path -->\n<robot ", 1)
    spaced = urdf.replace("><", ">\n   <")
    renamed = urdf.replace('<link name="base_link">', '<link name="base_link2">', 1)
    assert len({urdf, with_banner, spaced, renamed}) == 4                 # each edit really changed the text
    assert pci.config_id(with_banner, *args) == base
    assert pci.config_id(spaced, *args) == base
    assert pci.config_id(renamed, *args) != base                          # a content change is not normalised away


def test_c01_the_expanded_urdf_carries_no_machine_specific_path():
    # an absolute path inside the URDF would make the id differ between checkouts of the same commit
    canonical = pci.canonical_urdf(expand_xml())
    assert os.path.expanduser("~") not in canonical
    assert WORKSPACE not in canonical


def test_c01_the_docstring_names_the_members_the_serialisation_has():
    doc = pci.__doc__
    for member in ("canonical_urdf", "column", "base_plate", "head_pitch", "base_tilt", "arm_package", "hardware_profile",
                   "datums_revision", "upstream_pins"):
        assert member in doc, member
    assert "NOT an input" in doc and "lift stop" in doc and "CALIBRATION ID" in doc


# --- the command ----------------------------------------------------------------------------------------------------

def _run(*args):
    return subprocess.run([sys.executable, os.path.join(SCRIPTS, "print_config_id.py"), *args], capture_output=True, text=True)


def test_c01_command_prints_exactly_the_full_id_and_matches_the_function():
    result = _run("column:=dold_v4_350", "base_plate:=bp000", "head_pitch:=hp30", "base_tilt:=bt10", "arm_package:=openarm",
                  "hardware_profile:=mock", "--upstream-commit", COMMIT)
    assert result.returncode == 0, result.stderr
    # the command reads every OTHER pin from the workspace, so the library call does too
    pins, _ = pci.pins_of(WORKSPACE)
    pins["openarm_description"] = COMMIT
    assert result.stdout.strip() == pci.config_id_of(SHARE, pins, _row())
    assert ev.SHA256_RE.match(result.stdout.strip()) and result.stdout.count("\n") == 1


def test_c01_command_labels_the_short_form_as_a_prefix():
    result = _run("column:=dold_v4_350", "base_plate:=bp000", "head_pitch:=hp30", "base_tilt:=bt10", "arm_package:=openarm",
                  "hardware_profile:=mock", "--upstream-commit", COMMIT, "--explain")
    assert result.returncode == 0, result.stderr
    full = result.stdout.strip().split(":")[1]
    assert f"prefix (12 hex, for log lines only, not an id): {full[:12]}" in result.stderr


@pytest.mark.parametrize("args,needle", [
    (["column:=dold_v4_350", "base_plate:=bp000", "head_pitch:=hp30", "base_tilt:=bt10", "arm_package:=openarm"],
     "missing: hardware_profile:=..."),
    (["column:=dold_v4_350", "base_plate:=bp100", "head_pitch:=hp30", "base_tilt:=bt10", "arm_package:=openarm",
      "hardware_profile:=mock"], "Accepted values"),
    (["column:=dold_v4_350", "base_plate:=bp000", "head_pitch:=hp30", "base_tilt:=bt10", "arm_package:=openarm",
      "hardware_profile:=mock", "bogus:=1"], "unrecognised argument"),
    (["mast_id:=mast_1350", "base_plate:=bp000", "head_pitch:=hp30", "base_tilt:=bt10", "arm_package:=openarm",  # decision-allow: CONFIG-IDS negative test: mast_id must be absent or rejected
      "hardware_profile:=mock", "column:=dold_v4_350"], "unrecognised argument"),
])
def test_c01_command_fails_loudly_on_bad_input(args, needle):
    result = _run(*args, "--upstream-commit", COMMIT)
    assert result.returncode != 0
    assert needle in result.stderr and not result.stdout.strip()


# --- the pinned upstream commit -------------------------------------------------------------------------------------

def test_c01_the_pinned_upstream_commit_is_a_full_hash_and_matches_the_baseline_pin():
    commit = _commit()
    assert re.fullmatch(r"[0-9a-f]{40}", commit)
    pinned = ev.pinned_commit_in_baseline(WORKSPACE)
    if pinned is None:
        pytest.skip("openamrobot-manipulation's baseline .repos is not in the workspace")  # tracking: openAMRobot/openamr-upperbody-sw#16
    assert commit == pinned, f"checkout is at {commit}, openarm_v2.dev.repos pins {pinned}"


def _checkout(tmp_path):
    upstream = tmp_path / "src" / "upstream" / "openarm_description"
    upstream.mkdir(parents=True)
    git = lambda *a: subprocess.run(["git", "-C", str(upstream), *a], check=True, capture_output=True)  # noqa: E731
    git("init", "-q")
    git("-c", "user.name=t", "-c", "user.email=t@t", "commit", "-q", "--allow-empty", "-m", "x")
    return upstream


def _pins(tmp_path, commit=COMMIT):
    path = tmp_path / "pins.yaml"
    path.write_text(f"schema_version: 1\npins:\n  openarm_description: {commit}\n")
    return str(path)


def test_c01_a_git_checkout_wins_over_the_package_pin(tmp_path):
    upstream = _checkout(tmp_path)
    commit, source = ev.upstream_commit_and_source(str(tmp_path), pins_file=_pins(tmp_path))
    assert source == ev.GIT_SOURCE and commit != COMMIT and re.fullmatch(r"[0-9a-f]{40}", commit)
    assert commit == ev.git_head(str(upstream))


def test_c01_a_dirty_checkout_is_an_error_and_is_not_replaced_by_the_pin(tmp_path):
    upstream = _checkout(tmp_path)
    (upstream / "local_edit.txt").write_text("x")
    with pytest.raises(ev.EvidenceError, match="local changes"):
        ev.upstream_commit(str(tmp_path), pins_file=_pins(tmp_path))


def test_c01_without_a_checkout_the_package_pin_is_used_and_labelled(tmp_path):
    assert ev.upstream_commit_and_source(str(tmp_path), pins_file=_pins(tmp_path)) == (COMMIT, ev.METADATA_SOURCE)


def test_c01_no_checkout_and_no_pin_is_an_error_not_a_guess(tmp_path):
    with pytest.raises(ev.EvidenceError, match="no git checkout"):
        ev.upstream_commit(str(tmp_path), pins_file=str(tmp_path / "missing.yaml"))
    other = tmp_path / "other.yaml"
    other.write_text("schema_version: 1\npins:\n  something_else: abc\n")
    with pytest.raises(ev.EvidenceError, match="no git checkout"):
        ev.upstream_commit(str(tmp_path), pins_file=str(other))


def test_c01_every_package_pin_is_a_full_hash_and_equals_the_baseline_pin():
    """The fallback constants may not rot against the manipulation baseline. Their agreement with a present checkout is
    C02: here the comparison would pass vacuously with no checkout, because the pin is then what `_commit()` returns."""
    pins = ev.all_pins()
    assert "openarm_description" in pins
    for name, pin in pins.items():
        assert isinstance(pin, str) and re.fullmatch(r"[0-9a-f]{40}", pin), f"{name}: {pin!r}"
        baseline = ev.pinned_commit_in_baseline(WORKSPACE, name)
        if baseline is not None:
            assert pin == baseline, f"{name}: pinned {pin}, openarm_v2.dev.repos pins {baseline}"


def test_c01_the_fallback_gives_the_same_id_as_the_checkout(tmp_path):
    """An empty workspace has no checkout, so the package pin is used; the id it gives is the checkout's id."""
    commit, source = ev.upstream_commit_and_source(str(tmp_path))
    assert source == ev.METADATA_SOURCE
    assert _id(commit=commit) == _id(commit=_commit())
    pins, sources = pci.pins_of(str(tmp_path))
    assert set(pins) == set(ev.all_pins()) and set(sources.values()) == {ev.METADATA_SOURCE}


def test_c01_records_its_inputs_and_where_the_c9_pattern_came_from():
    commit = _commit()
    pinned = ev.pinned_commit_in_baseline(WORKSPACE)
    if os.path.isfile(C9_SCHEMA):
        c9 = {"checked": True, "schema": os.path.relpath(C9_SCHEMA, WORKSPACE),
              "pattern": next(iter({n["pattern"] for _, n in _c9_hash_definitions()})),
              "fields": [p for p, _ in _c9_hash_definitions()]}
    else:
        c9 = {"checked": False, "skipped_because": "openamrobot-interfaces is not in the workspace"}
    write_sidecar("C01", "config_id", "config id is sha256:<64 hex>, accepted by the C9 pattern", "PASS", {
        "format": "sha256:<64 lowercase hex>", "pattern": ev.SHA256_RE.pattern,
        "inputs": ["canonical expanded URDF (C14N, no comments, whitespace-stripped)", "column", "base_plate", "head_pitch",
                   "base_tilt", "arm_package", "hardware_profile", "datums.revision",
                   "upstream pins (every entry of config/upstream_pins.yaml, full hashes)"],
        "not_inputs": ["lift stop L1000/L1175/L1350: a calibration ID", "mast_id: deleted with the mast"],  # decision-allow: CONFIG-IDS negative test: mast_id must be absent or rejected
        "before_p15": ["canonical expanded URDF", "mast_id", "arm_package", "hardware_profile", "datums.revision",  # decision-allow: CONFIG-IDS negative test: mast_id must be absent or rejected
                       "openarm_description commit (full hash)"],
        "meaning_changed": "the format is unchanged and still valid against C9; an id recorded before P15 is not comparable",
        "enable_planar_base": "not a separate input: it changes the expanded URDF",
        "openarm_description_commit": commit,
        "matches_baseline_pin": None if pinned is None else commit == pinned,
        "c9_schema": c9,
    })
