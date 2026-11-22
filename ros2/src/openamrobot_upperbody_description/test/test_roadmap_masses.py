# SPDX-License-Identifier: MIT
"""Upper-body masses from the roadmap model, link by link (owner decision 8 Nov 2026, P-03 rev20.5 item 29: the roadmap model
is the reference for mass and placement, one source, every part counted once on the link where it sits).

config/roadmap_mass_export.yaml is a verbatim copy of openamrobot-docs docs/assets/roadmap/model/mass_export.yaml, pinned in
config/roadmap_mass_export.source.yaml. Every upper-body link with mass takes its export entry (roadmap_inertial.xacro): the
static DOLD share and the stop sensors on lift_column_base_link, the moving DOLD share and the carriage parts on
lift_carriage_link, the Getac, the head light, the ZED Mini with its bracket and the wrist cameras on their links. The OpenArm
links keep the upstream masses, which are the ARM-001 line. The base group is checked in openamr-platform-sw."""
import hashlib
import os

import pytest

from conftest import SHARE, expand, load_yaml

EXPORT_FILE = os.path.join(SHARE, "config", "roadmap_mass_export.yaml")
EXPORT = load_yaml(EXPORT_FILE)
PIN = load_yaml(os.path.join(SHARE, "config", "roadmap_mass_export.source.yaml"))
UPPER = {link: entry for g, v in EXPORT["groups"].items() if g not in ("base", "openarm") for link, entry in v["links"].items()}
CARRIAGE_BOM = {"MEC-003", "MEC-004", "UI-002", "LFT-004", "LFT-005", "LFT-006", "MEC-018:moving"}   # plus the second E-stop
COLUMN_BOM = {"MEC-018", "LFT-001", "LFT-003"}


def inertials(root):
    out = {}
    for e in root.findall("link"):
        i = e.find("inertial")
        if i is not None:
            o = i.find("origin")
            t = i.find("inertia")
            out[e.get("name")] = {"mass": float(i.find("mass").get("value")),
                                  "com": [float(v) for v in o.get("xyz").split()] if o is not None else [0.0, 0.0, 0.0],
                                  "diag": [float(t.get(k)) for k in ("ixx", "iyy", "izz")]}
    return out


def test_export_copy_is_the_pinned_one():
    with open(EXPORT_FILE, "rb") as f:
        assert hashlib.sha256(f.read()).hexdigest() == PIN["sha256"], "roadmap_mass_export.yaml differs from the pinned docs export"
    assert len(PIN["commit"]) == 40


def test_every_upper_body_link_takes_the_export():
    got = inertials(expand(arm_package="openarm"))
    for link, want in UPPER.items():
        assert link in got, f"{link} has no inertial: its parts ({[i['bom'] for i in want['items']]}) would be massless"
        assert got[link]["mass"] == pytest.approx(want["mass_kg"], abs=1e-6), link
        assert got[link]["com"] == pytest.approx(want["com_m"], abs=1e-6), link
        assert got[link]["diag"] == pytest.approx([want["inertia_kgm2"][k] for k in ("ixx", "iyy", "izz")], abs=1e-9), link


def test_no_upper_body_link_carries_a_mass_outside_the_export():
    got = inertials(expand(arm_package="openarm"))
    base_links = set(EXPORT["groups"]["base"]["links"]) | {"base_link", "camera_link"}
    platform = {n for n in got if n.endswith(("_caster_link", "_wheel_link")) or n in ("lidar_link",)}
    extra = {n for n in got if not n.startswith("openarm_") and n not in UPPER and n not in base_links and n not in platform}
    assert not extra, f"links with a hand-written mass: {sorted(extra)}"


def test_openarm_links_are_the_arm_001_line():
    got = inertials(expand(arm_package="openarm"))
    arm = sum(v["mass"] for n, v in got.items() if n.startswith("openarm_"))
    items = EXPORT["groups"]["openarm"]["links"]["openarm"]["items"]
    want = sum(i["mass_kg"] for i in items if i["bom"] == "ARM-001")
    assert arm == pytest.approx(want, abs=0.01)
    assert not any(i["bom"] != "ARM-001" for i in items), "only ARM-001 is on the upstream arm links"


def test_every_part_counted_once_on_its_link():
    seen = {}
    for group in EXPORT["groups"].values():
        for link, entry in group["links"].items():
            for it in entry["items"]:
                key = (it["bom"], it["component"])
                assert key not in seen, f"{key} is on {seen.get(key)} and on {link}"
                seen[key] = link
            assert sum(i["mass_kg"] for i in entry["items"]) == pytest.approx(entry["mass_kg"], abs=0.002), link
    for (bom, comp), link in seen.items():
        if bom in CARRIAGE_BOM or comp == "second_estop":
            assert link == "lift_carriage_link", f"{bom} {comp} rides the carriage but sits on {link}"
        if bom in COLUMN_BOM:
            assert link == "lift_column_base_link", f"{bom} {comp} is on the static column but sits on {link}"
    assert seen.get(("ARM-011", "head_camera_bracket")) == "head_camera_link", "the ZED Mini bracket rides with the camera"
    dold = [link for (bom, _), link in seen.items() if bom.startswith("MEC-018")]
    assert sorted(dold) == ["lift_carriage_link", "lift_column_base_link"], "the DOLD column: static and moving share, once each"
