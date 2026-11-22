# SPDX-License-Identifier: MIT
"""T10: modelled mass, reported by owner. FAIL only if the modelled total exceeds `gross_mass_limit`.

It REPORTS what the description models; it does not claim the robot weighs that. The battery, hub motors, electronics
and the lift column's structure beyond its catalogue mass are not modelled, so the total is a lower bound on nothing in
particular and says nothing about the gross mass of the machine. The report says so in words.

Owner of a link = the `owner` of its frame_contract.yaml entry, with option B's and our own links split into lift, openarm and
sensors by name (option B declares the arm base links, but their mass is the upstream arm's).
A massed link the contract does not list (or an owner this file does not know) is `unclassified`: still counted in the
total, and a WARN, so a new link cannot hide from the report.

After the total the report also prints the `gross_mass_limit` it compared against, the F2S vehicle estimate, and
"per-link masses still placeholders" if and only if some group is `placeholder` in config/mass_provenance.yaml, plus a
separate line naming the `external_nominal` groups. The map has three states over all four groups (placeholder,
external_nominal, verified); it replaces P09's tripwire, which only guarded the absence of such a map. Where a group draws its
mass from a datum (the lift, `lift_mass_dold_v4_350`) the map must agree with the datum's status, so the clause cannot go quiet over a
group that is still a placeholder, and disappears by itself on the day every group is real.
"""
import fnmatch
import os
import warnings
import pytest
import xml.etree.ElementTree as ET

from conftest import MATRIX, SHARE, expand, load_datums, load_yaml, write_sidecar

CONTRACT = load_yaml(os.path.join(SHARE, "config", "frame_contract.yaml"))
CONTRACT_OWNER = {e["name"]: e["owner"] for e in CONTRACT["entries"] if e["kind"] == "link"}

GROUPS = [  # key, label, what the number is (its provenance STATE is read from config/mass_provenance.yaml, not written here)
    ("base", "base (platform-sw)", "platform-sw CAD export"),
    ("lift", "lift (ours)", "catalogue mass (lift_mass_dold_v4_350) on a placeholder box"),
    ("openarm", "openarm_* (upstream)", "upstream nominal, arms + grippers, no body (ADR-008)"),
    ("sensors", "sensors (ours)", "ZED Mini with its mount and the wrist cameras, from the roadmap export"),
    ("equipment", "equipment (item 24)", "supplier masses (getac_tablet_mass, head_light_mass) on boxes"),
    ("equipment_no_mass", "equipment, no mass", "item 24 parts with no stated mass carry no <inertial>"),
    ("unclassified", "unclassified", "massed links not in the frame contract"),
]
MASS_GROUPS = ("base", "lift", "openarm", "sensors", "equipment", "equipment_no_mass")  # the groups the provenance map must cover
NOT_MODELLED = ["battery", "hub motors", "electronics", "the lift column's structure beyond its catalogue mass"]
SENSOR_LINKS = ["head_camera_*", "*_wrist_camera_*"]
# P-03 item 24 equipment (upper_body_equipment.xacro). Massed ones (tablet, head light) report in `equipment`; the others carry no
# <inertial> (no stated mass, provenance group `equipment_no_mass`) and would report in `equipment` too the day they get one.
EQUIPMENT_LINKS = ["getac_tablet_link", "head_light_link", "second_estop_link", "lift_stop_sensor_*_link", "lift_flag_rod_link",
                   "lift_flag_link", "head_camera_bracket_link"]
# Group -> the datums whose value is a link mass in that group (combined_robot.xacro: lift_mass). A group draws a mass from a
# datum only if it is listed here and its provenance entry names the datum; test_t10_mass_datums_are_the_ones_the_xacro_reads
# keeps the list honest. The null datum of the fallback column (lift_mass_tl3_400) is not listed: it is never a link mass.
MASS_DATUMS = {"lift": ["lift_mass_dold_v4_350"]}
PROVENANCE_FILE = os.path.join(SHARE, "config", "mass_provenance.yaml")
PROVENANCE_STATES = ("placeholder", "external_nominal", "verified")
F2S_ESTIMATE = "f2s_vehicle_mass_estimate"
PLACEHOLDER_CLAUSE = "per-link masses still placeholders"


def load_provenance():
    """{group: entry} from config/mass_provenance.yaml (a fresh parse; tests may mutate it)."""
    return load_yaml(PROVENANCE_FILE)["groups"]


def groups_in_state(provenance, state):
    return [group for group in MASS_GROUPS if provenance[group]["state"] == state]


def provenance_problems(provenance, datums):
    """What is wrong with a provenance map, as messages (empty: nothing). Three states over exactly the four mass groups; a
    `verified` group names its document and a group in another state does not claim one; and a group that draws its mass from a
    datum agrees with that datum's status, in both directions that matter: a `placeholder` datum is a placeholder group (the
    caveat may not go quiet over it) and a `verified` datum is a verified group (a stale map may not keep a caveat that is
    no longer true), while a `provisional` datum can only be an external figure."""
    problems = []
    if set(provenance) != set(MASS_GROUPS):
        problems.append(f"groups {sorted(provenance)} are not exactly {sorted(MASS_GROUPS)}")
    for group, entry in provenance.items():
        state = entry.get("state")
        if state not in PROVENANCE_STATES:
            problems.append(f"{group}: state {state!r} is not one of {', '.join(PROVENANCE_STATES)}")
        if not str(entry.get("source") or "").strip():
            problems.append(f"{group}: no `source` saying where the figure comes from")
        if state == "verified" and not str(entry.get("document") or "").strip():
            problems.append(f"{group}: `verified` needs a `document` (measured, with a document)")
        if state in ("placeholder", "external_nominal") and entry.get("document"):
            problems.append(f"{group}: a {state} group cannot name a `document`; that is what `verified` means")
        datum = entry.get("datum")
        if datum:
            status = datums[datum]["status"]
            expected = {"placeholder": ("placeholder",), "provisional": ("external_nominal",), "verified": ("verified",)}[status]
            if state not in expected:
                problems.append(f"{group}: datum {datum} has status {status} but the group's state is {state} "
                                f"(expected {' or '.join(expected)})")
    return problems


def owner_group(link_name, contract_owner=None):
    owner = CONTRACT_OWNER.get(link_name) if contract_owner is None else contract_owner
    if owner is None:
        return "unclassified"
    if owner == "platform-sw":
        return "base"
    if owner.startswith("upstream:") or fnmatch.fnmatchcase(link_name, "openarm_*"):
        return "openarm"      # option B's mount frames carry the upstream arm's base link (same name, upstream's mass)
    if owner.startswith("A6") or owner.startswith("Alex (option B"):
        if owner.startswith("A6") and any(fnmatch.fnmatchcase(link_name, pat) for pat in EQUIPMENT_LINKS):
            return "equipment"
        if fnmatch.fnmatchcase(link_name, "lift_*"):
            return "lift"
        if any(fnmatch.fnmatchcase(link_name, pat) for pat in SENSOR_LINKS):
            return "sensors"
    return "unclassified"


def mass_by_group(root):
    """({group: kg}, {group: [link names]}) over every link with an <inertial>."""
    kg = {key: 0.0 for key, _, _ in GROUPS}
    links = {key: [] for key, _, _ in GROUPS}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        group = owner_group(link.get("name"))
        kg[group] += float(inertial.find("mass").get("value"))
        links[group].append(link.get("name"))
    return kg, links


def report_lines(label, kg, links, datums, base_link_mass, provenance):
    """The T10 report. `datums` is the parsed datums.yaml `datums` map and `provenance` the parsed mass_provenance.yaml `groups`:
    the limit, the F2S estimate, their sources, each row's provenance state and the placeholder clause all come from them."""
    total = sum(kg.values())
    limit, f2s = datums["gross_mass_limit"], datums[F2S_ESTIMATE]
    placeholders, external = groups_in_state(provenance, "placeholder"), groups_in_state(provenance, "external_nominal")
    lines = [f"Modelled mass at {label}"]
    for key, name, what in GROUPS:
        if key == "unclassified" and not kg[key]:
            continue
        state = f"[{provenance[key]['state']}]" if key in provenance else ""
        lines.append(f"  {name:<22} {kg[key]:>7.2f} kg   {state:<18} {what}")
    lines += [f"  {'modelled total':<22} {total:>7.2f} kg   vs gross_mass_limit {limit['value']:g} kg",
              f"  gross_mass_limit {limit['value']:g} kg [{limit['source']}, {limit['status']}]: {limit['ref']}",
              "  (F2S vehicle estimate " + f"{f2s['value']:g} kg" + (f" \u2014 {PLACEHOLDER_CLAUSE}" if placeholders else "") + ")",
              f"  F2S vehicle estimate [{f2s['source']}, {f2s['status']}]: {f2s['ref']}"]
    if placeholders:
        lines.append(f"  placeholder groups (no source, chosen so the model expands): {', '.join(placeholders)}")
    if external:
        lines.append(f"  external_nominal groups (another owner's figure, not verified by us): {', '.join(external)}")
    lines += ["Not modelled (all F2S): " + ", ".join(NOT_MODELLED) + ".",
              "  The lift row above is a catalogue mass on a placeholder box, not the structure. The total is",
              "  therefore what the description contains, not what the robot weighs.",
              f"  W01 (retired): base_link ({base_link_mass:.2f} kg of the base figure) is no longer the root, so KDL sees it."]
    return lines


def test_t10_mass_report_and_gross_mass_limit():
    datums = load_datums()["datums"]
    limit = float(datums["gross_mass_limit"]["value"])
    release = expand()                                             # baseline / openarm / mock
    kg, links = mass_by_group(release)
    base_link = float(release.find("link[@name='base_link']/inertial/mass").get("value"))
    provenance = load_provenance()
    lines = report_lines("the baseline / openarm", kg, links, datums, base_link, provenance)
    problems = provenance_problems(provenance, datums)

    totals, exceeded = {}, []
    for base_plate, arm_package in MATRIX:
        group_kg, _ = mass_by_group(expand(base_plate=base_plate, arm_package=arm_package))
        totals[f"{base_plate}/{arm_package}"] = round(sum(group_kg.values()), 3)
        if sum(group_kg.values()) > limit:
            exceeded.append(f"{base_plate}/{arm_package}: {sum(group_kg.values()):.2f} kg > {limit:g} kg")

    unclassified = links["unclassified"]
    density = density_findings(datums)
    density_warn = [r for r in density if "problem" in r or not r["within"]]
    result = "FAIL" if exceeded or problems else ("WARN" if unclassified or density_warn else "PASS")
    write_sidecar("T10", "mass_report", "modelled mass by owner vs gross_mass_limit", result, {
        "report_expansion": "dold_v4_350/bp000/hp30/bt10/openarm/mock",
        "kg_by_owner": {key: round(value, 3) for key, value in kg.items() if value or key != "unclassified"},
        "links_by_owner": {key: sorted(names) for key, names in links.items() if names},
        "modelled_total_kg": round(sum(kg.values()), 3),
        "gross_mass_limit_kg": limit,
        "gross_mass_limit_source": {key: datums["gross_mass_limit"][key] for key in ("source", "status", "ref")},
        "f2s_vehicle_estimate_kg": float(datums[F2S_ESTIMATE]["value"]),
        "f2s_vehicle_estimate_source": {key: datums[F2S_ESTIMATE][key] for key in ("source", "status", "ref")},
        "mass_provenance": {group: {"state": entry["state"], "source": entry["source"]} for group, entry in provenance.items()},
        "per_link_masses_still_placeholders": bool(groups_in_state(provenance, "placeholder")),
        "placeholder_groups": groups_in_state(provenance, "placeholder"),
        "external_nominal_groups": groups_in_state(provenance, "external_nominal"),
        "verified_groups": groups_in_state(provenance, "verified"),
        "provenance_problems": problems,
        "modelled_total_kg_by_expansion": totals,
        "density_plausibility": {"scope": "mass datums of ours only (MASS_DATUMS); a band is declared per datum by its author",
                                 "rows": density, "outside_band": [r["datum"] for r in density_warn]},
        "not_modelled": NOT_MODELLED,
        "base_link_inertia_not_seen_by_kdl": {"id": "W01", "retired_as_a_warn": True, "base_link_mass_kg": round(base_link, 3),
                                              "now": "base_link is a non-root link (root: base_footprint), KDL sees its inertia"},
        "report": lines,
    })
    print("\n" + "\n".join(lines))
    if unclassified:
        warnings.warn(f"T10 WARN: massed links with no owner in frame_contract.yaml: {', '.join(unclassified)}",
                      UserWarning, stacklevel=1)
    assert not problems, "\n" + "\n".join(problems)
    assert not exceeded, "\n" + "\n".join(exceeded)


def test_t10_every_massed_link_in_the_matrix_has_an_owner_group():
    unclassified = set()
    for base_plate, arm_package in MATRIX:
        unclassified |= set(mass_by_group(expand(base_plate=base_plate, arm_package=arm_package))[1]["unclassified"])
    assert not unclassified, f"massed links with no owner in frame_contract.yaml: {sorted(unclassified)}"


def test_t10_the_groups_partition_the_total():
    root = expand()
    kg, links = mass_by_group(root)
    every = [l.get("name") for l in root.findall("link") if l.find("inertial") is not None]
    assert sorted(n for names in links.values() for n in names) == sorted(every)
    assert abs(sum(kg.values()) - sum(float(m.get("value")) for m in root.iter("mass"))) < 1e-9


def test_t10_the_upstream_arms_are_the_known_nominal_mass():
    # ADR-008 / S1: arms + grippers 12.26 kg from the upstream xacro; this is what "upstream nominal" means
    kg, _ = mass_by_group(expand())
    assert abs(kg["openarm"] - 12.26) < 0.01


# --- plausibility of OUR masses: the implied density against the band the datum declares (V01-F2's rule, P15) -----------------
# V01 found a 3.0 kg placeholder for 1.24 m of aluminium extrusion that nothing questioned. The rule is not a global density band
# (upstream arm link2 implies 297 kg/m3 over its bounding box, below that mast's 337: a box round an articulated link is mostly
# air, and we have no standing to judge upstream's inertials). It is scoped to the mass datums THIS repository declares, each with
# the band its author declared about the thing it describes, and a figure outside the band is a WARN, never a derived value.

def bounding_volume_m3(datum, datums):
    """Volume of the bounding box a mass datum's `density_band.bounding_box` names. One entry per datum we own."""
    from conftest import UPPERBODY
    if datum == "lift_mass_dold_v4_350":
        x, y = datums["lift_column_section"]["value"]
        return x * y * UPPERBODY["column"]["dold_v4_350"]["retracted_length"]
    raise KeyError(f"no bounding volume is defined for the mass datum {datum}: say what it is a mass of")


def density_findings(datums, mass_datums=None):
    """[{datum, mass_kg, volume_m3, density_kg_m3, band, within, status}] for every mass datum of ours (value not null)."""
    rows = []
    for group, names in (mass_datums or MASS_DATUMS).items():
        for name in names:
            entry = datums[name]
            band = entry.get("density_band")
            if band is None:
                rows.append({"datum": name, "problem": "declares no density_band"})
                continue
            if entry["value"] is None:
                continue
            volume = bounding_volume_m3(name, datums)
            density = float(entry["value"]) / volume
            rows.append({"datum": name, "group": group, "mass_kg": float(entry["value"]), "volume_m3": round(volume, 6),
                         "density_kg_m3": round(density, 1), "band": [band["min"], band["max"]],
                         "within": band["min"] <= density <= band["max"], "status": entry["status"],
                         "bounding_box": band["bounding_box"]})
    return rows


def test_t10_every_mass_of_ours_declares_a_density_band_and_sits_inside_it():
    datums = load_datums()["datums"]
    rows = density_findings(datums)
    assert rows and not [r for r in rows if "problem" in r], rows
    for r in rows:
        assert r["band"][0] < r["band"][1] and r["within"], f"{r['datum']}: {r['density_kg_m3']} kg/m3 outside {r['band']} over {r['bounding_box']}"
    lift = next(r for r in rows if r["datum"] == "lift_mass_dold_v4_350")
    # P28: 11.7 kg over the SOURCED outer section 0.240 x 0.060 x 0.530 m (it was 383.4 over an invented 0.240 x 0.240); the band's upper bound was
    # re-fitted from 1200 to 1600 for that, see the datum's reason
    assert abs(lift["density_kg_m3"] - 1533.0) < 0.1, lift


def test_t10_the_check_is_scoped_to_our_masses_not_upstreams():
    """Upstream's arm masses are nominal figures from another owner: the check never reads them (no datum, no band)."""
    assert set(MASS_DATUMS) == {"lift"} and all(not n.startswith("openarm") for names in MASS_DATUMS.values() for n in names)


def test_t10_the_density_check_flags_the_v01_f2_case_and_a_missing_band():
    """The mast of A6: 3.0 kg over 0.060 x 0.120 x 1.2365 m is 337 kg/m3. Against a band declared for an extrusion it is flagged."""
    datums = load_datums()["datums"]
    datums["lift_mass_dold_v4_350"]["value"] = 0.5                 # a column of 0.5 kg over 0.24 x 0.06 x 0.53 m: 66 kg/m3
    (row,) = density_findings(datums)
    assert not row["within"] and row["density_kg_m3"] < 100
    datums["lift_mass_dold_v4_350"]["value"] = 25.0                # and 25 kg: 3276 kg/m3, denser than solid aluminium
    (row,) = density_findings(datums)
    assert not row["within"] and row["density_kg_m3"] > 2700
    datums = load_datums()["datums"]
    del datums["lift_mass_dold_v4_350"]["density_band"]
    (row,) = density_findings(datums)
    assert row["problem"] == "declares no density_band"


def test_t10_a_null_datum_is_skipped_by_the_density_check_not_judged():
    datums = load_datums()["datums"]
    assert datums["lift_mass_tl3_400"]["value"] is None
    # lift_mass_tl3_400 declares no band (it has no value to judge) and is skipped before that is looked at
    rows = density_findings(datums, {"lift": ["lift_mass_dold_v4_350"]})
    assert [r["datum"] for r in rows] == ["lift_mass_dold_v4_350"]
    assert density_findings(datums, {"lift": ["lift_mass_tl3_400"]}) == [{"datum": "lift_mass_tl3_400", "problem": "declares no density_band"}]


# --- the limit and the grouping on synthetic input: they must be able to fail ------------------------------------------

def _urdf(*masses):
    return ET.fromstring("<robot>" + "".join(
        f"<link name='{n}'><inertial><mass value='{m}'/></inertial></link>" for n, m in masses) + "</robot>")


def test_t10_grouping_follows_the_contract_owner_and_splits_our_links_by_name():
    assert owner_group("base_link") == "base"
    assert owner_group("openarm_left_link1") == "openarm"
    assert owner_group("lift_column_base_link") == "lift"
    assert owner_group("openarm_left_base_link") == "openarm"          # option B's name, upstream's mass
    assert owner_group("head_camera_link") == "sensors"                # option B's frame, in our sensors group
    assert owner_group("left_wrist_camera_link") == "sensors"
    assert owner_group("getac_tablet_link") == "equipment" and owner_group("head_light_link") == "equipment"
    assert owner_group("lift_stop_sensor_l1000_link") == "equipment" and owner_group("lift_flag_link") == "equipment"
    assert owner_group("lift_carriage_link") == "lift"                 # option B's lift_* links stay in the lift group
    assert owner_group("some_new_link") == "unclassified"                        # not in the contract
    assert owner_group("odd_link", contract_owner="A6 (openamr-upperbody-sw)") == "unclassified"
    assert owner_group("odd_link", contract_owner="Alex (option B, openamrobot_upperbody.xacro)") == "unclassified"
    assert owner_group("odd_link", contract_owner="somebody else") == "unclassified"


def test_t10_a_total_over_the_limit_is_what_fails():
    kg, _ = mass_by_group(_urdf(("base_link", 100.0), ("lift_column_base_link", 25.0)))
    assert sum(kg.values()) > 120.0
    kg, _ = mass_by_group(_urdf(("base_link", 100.0), ("lift_column_base_link", 19.0)))
    assert not sum(kg.values()) > 120.0
    kg, links = mass_by_group(_urdf(("mystery", 1.0)))
    assert kg["unclassified"] == 1.0 and links["unclassified"] == ["mystery"]


# --- the limit, the F2S estimate and the placeholder clause --------------------------------------------------------

def _report(datums=None, provenance=None):
    datums = datums or load_datums()["datums"]
    provenance = provenance or load_provenance()
    kg, links = mass_by_group(expand())
    return report_lines("the baseline / openarm", kg, links, datums, 4.18, provenance), datums


def _provenance(**states):
    """The real map with some groups' states replaced; `verified` gets a document so the map stays valid."""
    provenance = load_provenance()
    for group, state in states.items():
        provenance[group]["state"] = state
        provenance[group].pop("document", None)
        if state == "verified":
            provenance[group]["document"] = "weighing record no. X, rev 2"
    return provenance


def _datums_with_lift_mass(status):
    datums = load_datums()["datums"]
    datums["lift_mass_dold_v4_350"]["status"] = status
    return datums


def test_t10_prints_the_limit_with_its_source_and_the_f2s_estimate_with_its_source():
    lines, datums = _report()
    text = "\n".join(lines)
    assert "vs gross_mass_limit 120 kg" in text
    limit = datums["gross_mass_limit"]
    assert f"gross_mass_limit 120 kg [{limit['source']}, {limit['status']}]: {limit['ref']}" in text
    f2s = datums[F2S_ESTIMATE]
    assert f2s["value"] == 93.0 and f2s["source"] == "f2s"          # the datum, not a constant in the reporter
    assert f"(F2S vehicle estimate 93 kg" in text
    assert f"F2S vehicle estimate [{f2s['source']}, {f2s['status']}]: {f2s['ref']}" in text


def test_t10_the_f2s_estimate_is_what_datums_yaml_says_not_a_constant_in_the_reporter():
    datums = load_datums()["datums"]
    datums[F2S_ESTIMATE].update(value=87.5, ref="a different document")
    text = "\n".join(_report(datums)[0])
    assert "(F2S vehicle estimate 87.5 kg" in text and "a different document" in text and "93 kg" not in text


# --- the provenance map: three states over four groups, and the caveat reads off it ---------------------------------------

def test_t10_mass_provenance_the_real_map_is_valid_and_says_what_the_design_says():
    provenance, datums = load_provenance(), load_datums()["datums"]
    assert provenance_problems(provenance, datums) == []
    assert {g: provenance[g]["state"] for g in MASS_GROUPS} == {
        "base": "external_nominal", "lift": "external_nominal", "openarm": "external_nominal", "sensors": "external_nominal",
        "equipment": "external_nominal", "equipment_no_mass": "placeholder"}
    assert not groups_in_state(provenance, "verified"), "nothing is measured yet; a verified group needs its document first"


def test_t10_with_the_current_tree_the_caveat_and_the_external_nominal_line_appear():
    lines, _ = _report()
    assert "  (F2S vehicle estimate 93 kg \u2014 per-link masses still placeholders)" in lines
    assert any(line.startswith("  placeholder groups") and line.endswith(": equipment_no_mass") for line in lines)
    assert any(line.startswith("  external_nominal groups") and line.endswith("base, lift, openarm, sensors, equipment") for line in lines)
    row = {name: line for line in lines for _, name, _ in GROUPS if line.startswith(f"  {name} ")}
    assert "[external_nominal]" in row["base (platform-sw)"] and "[external_nominal]" in row["lift (ours)"]
    assert "[external_nominal]" in row["openarm_* (upstream)"] and "[external_nominal]" in row["sensors (ours)"]
    assert "[external_nominal]" in row["equipment (item 24)"] and "[placeholder]" in row["equipment, no mass"]


def test_t10_the_caveat_is_present_when_any_one_group_is_a_placeholder_and_absent_when_none_is():
    for lone in MASS_GROUPS:
        states = {g: "external_nominal" for g in MASS_GROUPS}
        states[lone] = "placeholder"
        lines = _report(provenance=_provenance(**states))[0]
        assert PLACEHOLDER_CLAUSE in "\n".join(lines), f"{lone} alone is a placeholder and the caveat must stay"
        assert any(line.endswith(f"expands): {lone}") for line in lines)
    quiet = "\n".join(_report(provenance=_provenance(**{g: "external_nominal" for g in MASS_GROUPS}))[0])
    assert PLACEHOLDER_CLAUSE not in quiet and "placeholder groups" not in quiet
    assert "  (F2S vehicle estimate 93 kg)" in quiet                   # the estimate itself is still printed


def test_t10_the_external_nominal_line_is_its_own_line_and_follows_the_map():
    all_verified = "\n".join(_report(provenance=_provenance(**{g: "verified" for g in MASS_GROUPS}))[0])
    assert PLACEHOLDER_CLAUSE not in all_verified and "external_nominal groups" not in all_verified
    only_arms = _report(provenance=_provenance(base="verified", lift="verified", sensors="verified", equipment="verified",
                                               equipment_no_mass="verified"))[0]
    assert any(line.endswith("not verified by us): openarm") for line in only_arms)
    assert PLACEHOLDER_CLAUSE not in "\n".join(only_arms)


def test_t10_the_caveat_follows_the_map_back_and_forth():
    states = {g: "verified" for g in MASS_GROUPS}
    assert PLACEHOLDER_CLAUSE not in "\n".join(_report(provenance=_provenance(**states))[0])
    states["sensors"] = "placeholder"
    assert PLACEHOLDER_CLAUSE in "\n".join(_report(provenance=_provenance(**states))[0])
    states["sensors"] = "external_nominal"                                # only `placeholder` triggers it
    assert PLACEHOLDER_CLAUSE not in "\n".join(_report(provenance=_provenance(**states))[0])


def test_t10_a_map_that_is_not_three_states_over_the_four_groups_is_a_problem():
    datums = load_datums()["datums"]
    provenance = load_provenance()
    del provenance["sensors"]
    assert any("not exactly" in m for m in provenance_problems(provenance, datums))
    provenance = load_provenance()
    provenance["base"]["state"] = "provisional"                           # a datum status is not a provenance state
    assert any("base: state 'provisional'" in m for m in provenance_problems(provenance, datums))
    provenance = load_provenance()
    provenance["openarm"]["source"] = ""
    assert any("openarm: no `source`" in m for m in provenance_problems(provenance, datums))


def test_t10_verified_needs_a_document_and_nothing_else_may_claim_one():
    datums = _datums_with_lift_mass("verified")
    provenance = _provenance(lift="verified")
    assert provenance_problems(provenance, datums) == []
    del provenance["lift"]["document"]
    assert any("lift: `verified` needs a `document`" in m for m in provenance_problems(provenance, datums))
    provenance = load_provenance()
    provenance["base"]["document"] = "a drawing"
    assert any("base: a external_nominal group cannot name a `document`" in m or "base: a external_nominal" in m
               for m in provenance_problems(provenance, load_datums()["datums"]))


def test_t10_the_map_has_to_agree_with_the_datum_a_group_draws_its_mass_from():
    """P09's tripwire, in its place: the day lift_mass_dold_v4_350 becomes verified the map cannot be left saying
    `external_nominal` (the caveat would outlive its cause), and a map that says `verified` over a provisional datum is the lie
    that would silence it. A catalogue figure (provisional) is another owner's number: external_nominal, not placeholder."""
    assert provenance_problems(load_provenance(), _datums_with_lift_mass("provisional")) == []
    stale = provenance_problems(load_provenance(), _datums_with_lift_mass("verified"))
    assert stale and "datum lift_mass_dold_v4_350 has status verified but the group's state is external_nominal" in stale[0]
    over_claimed = provenance_problems(_provenance(lift="verified"), _datums_with_lift_mass("provisional"))
    assert over_claimed and "has status provisional but the group's state is verified" in over_claimed[0]
    # a datum that went back to placeholder (the catalogue figure withdrawn) makes the group a placeholder again
    assert provenance_problems(load_provenance(), _datums_with_lift_mass("placeholder"))
    assert provenance_problems(_provenance(lift="placeholder"), _datums_with_lift_mass("placeholder")) == []


def test_t10_mass_datums_are_the_ones_the_xacro_reads():
    # MASS_DATUMS is the one hand-kept list the map's datum link depends on: every entry must be a real datum that the xacro reads
    # for a link mass, and the map must name it. A datum renamed or dropped from combined_robot.xacro fails here instead of
    # silently breaking the agreement check.
    from test_t13_datum_references import referenced_in_package
    datums, referenced, provenance = load_datums()["datums"], referenced_in_package(), load_provenance()
    for group, names in MASS_DATUMS.items():
        assert group in {key for key, _, _ in GROUPS}
        assert [provenance[group].get("datum")] == names, f"{group}'s provenance entry must name the datum {names}"
        for name in names:
            assert name in datums and name in referenced, f"{group}: {name} is not a datum the xacro reads"
    assert {g for g, entry in provenance.items() if entry.get("datum")} == set(MASS_DATUMS)
    root = expand(roadmap_masses="false")   # the datum path: the lift datum on one box
    kg, links = mass_by_group(root)
    lift_mass = float(datums["lift_mass_dold_v4_350"]["value"])
    assert lift_mass == 11.7
    assert any(abs(float(l.find("inertial/mass").get("value")) - lift_mass) < 1e-9
               for l in root.findall("link") if l.get("name") in links["lift"]), "no lift link carries lift_mass"
    # the default (roadmap export): the same 11.7 kg, split once into the static and the moving share
    export = load_yaml(os.path.join(SHARE, "config", "roadmap_mass_export.yaml"))
    dold = [i["mass_kg"] for g in export["groups"].values() for e in g["links"].values() for i in e["items"] if i["bom"].startswith("MEC-018")]
    assert len(dold) == 2 and sum(dold) == pytest.approx(lift_mass, abs=0.002)
