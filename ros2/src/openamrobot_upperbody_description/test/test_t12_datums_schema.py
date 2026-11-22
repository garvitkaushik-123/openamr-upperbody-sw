# SPDX-License-Identifier: MIT
"""T12: datums.yaml validates against datums.schema.json, and the schema rejects what it must reject."""
import os

import pytest
from jsonschema import Draft202012Validator

from conftest import FIXTURES, load_datums, load_schema, load_yaml


def _errors(doc):
    return list(Draft202012Validator(load_schema()).iter_errors(doc))


def _failing_datums(doc):
    """Names of datums that have at least one schema error."""
    out = set()
    for e in _errors(doc):
        path = list(e.absolute_path)
        if len(path) >= 2 and path[0] == "datums":
            out.add(path[1])
    return out


def test_t12_schema_is_a_valid_2020_12_schema():
    schema = load_schema()
    assert schema["$schema"] == "https://json-schema.org/draft/2020-12/schema"
    Draft202012Validator.check_schema(schema)


def test_t12_datums_yaml_validates():
    errors = _errors(load_datums())
    assert not errors, "\n".join(f"{'/'.join(map(str, e.absolute_path))}: {e.message}" for e in errors)


def test_t12_schema_allows_report_only_and_pending_reference():
    doc = load_datums()
    assert "report_only" in doc and "pending_reference" in doc, "datums.yaml should carry both top-level maps"
    assert not _errors(doc)
    # and neither is mandatory
    for key in ("report_only", "pending_reference"):
        trimmed = load_datums()
        del trimmed[key]
        assert not _errors(trimmed), f"{key} must be optional"


def test_t12_broken_fixture_is_rejected():
    doc = load_yaml(os.path.join(FIXTURES, "datums_broken.yaml"))
    errors = _errors(doc)
    assert errors, "datums_broken.yaml validated, so the schema is not enforcing anything"
    # ... and for the reason each datum was written to break, not for some accidental one
    assert _failing_datums(doc) == set(doc["datums"]), (
        f"expected every datum in the fixture to fail, got {sorted(_failing_datums(doc))}")
    assert any(list(e.absolute_path)[:1] == ["pending_reference"] for e in errors), "bad pending step not caught"


# One mutation per rule, applied to a copy of the real file, so each rule is shown to bite on its own.
def _verified_derived(d):
    d["datums"]["lift_mass_dold_v4_350"].update(source="derived", status="verified")


def _verified_upstream(d):
    d["datums"]["lift_mass_dold_v4_350"].update(source="upstream", status="verified")


def _verified_placeholder_source(d):
    d["datums"]["lift_mass_dold_v4_350"].update(source="placeholder", status="verified")


def _verified_empty_ref(d):
    d["datums"]["gross_mass_limit"]["ref"] = ""


def _verified_blank_ref(d):
    d["datums"]["gross_mass_limit"]["ref"] = "   "


def _verified_missing_ref(d):
    del d["datums"]["gross_mass_limit"]["ref"]


def _placeholder_source_provisional(d):
    d["datums"]["lift_mass_dold_v4_350"].update(source="placeholder", status="provisional")


def _placeholder_source_verified_with_ref(d):
    d["datums"]["lift_mass_dold_v4_350"].update(source="placeholder", status="verified")


def _bad_source(d):
    d["datums"]["lift_mass_dold_v4_350"]["source"] = "guessed"


def _bad_status(d):
    d["datums"]["lift_mass_dold_v4_350"]["status"] = "approved"


def _missing(field):
    def mutate(d):
        del d["datums"]["lift_mass_dold_v4_350"][field]
    mutate.__name__ = f"missing_{field}"
    return mutate


def _unknown_field(d):
    d["datums"]["lift_mass_dold_v4_350"]["verified_by"] = "me"


def _bad_pending_step(d):
    d["pending_reference"]["left_wrist_camera_pose"] = "later"


def _pending_not_a_map(d):
    d["pending_reference"] = ["left_wrist_camera_pose"]


def _report_only_duplicate(d):
    d["report_only"].append(d["report_only"][0])


def _unknown_top_level(d):
    d["report_also"] = []


def _pose_missing_rpy(d):
    d["datums"]["left_wrist_camera_pose"]["value"] = {"xyz": [0.0, 0.0, 0.2]}


def _bad_revision(d):
    d["revision"] = "v2"


def _null_with_a_supplier_source(d):
    d["datums"]["lift_mass_tl3_400"]["source"] = "supplier"          # a null value is placeholder, source and status


def _null_with_a_provisional_status(d):
    d["datums"]["lift_mass_tl3_400"]["status"] = "provisional"


def _dimension_list_without_an_axis_convention(d):
    del d["datums"]["lift_column_section"]["axis_convention"]


def _axis_convention_with_an_unknown_role(d):
    d["datums"]["lift_column_section"]["axis_convention"]["x"] = "forwards"


def _axis_convention_with_an_unknown_axis(d):
    d["datums"]["lift_column_section"]["axis_convention"]["w"] = "lateral"


def _density_band_without_a_reason(d):
    del d["datums"]["lift_mass_dold_v4_350"]["density_band"]["reason"]


def _density_band_with_an_inverted_range_is_not_a_schema_matter_but_a_zero_is(d):
    d["datums"]["lift_mass_dold_v4_350"]["density_band"]["min"] = 0


MUTATIONS = [
    _verified_derived, _verified_upstream, _verified_placeholder_source, _verified_empty_ref, _verified_blank_ref,
    _verified_missing_ref, _placeholder_source_provisional, _placeholder_source_verified_with_ref, _bad_source,
    _bad_status, _missing("value"), _missing("source"), _missing("status"), _missing("ref"), _missing("owner"),
    _unknown_field, _bad_pending_step, _pending_not_a_map, _report_only_duplicate, _unknown_top_level,
    _pose_missing_rpy, _bad_revision, _null_with_a_supplier_source, _null_with_a_provisional_status,
    _dimension_list_without_an_axis_convention, _axis_convention_with_an_unknown_role, _axis_convention_with_an_unknown_axis,
    _density_band_without_a_reason, _density_band_with_an_inverted_range_is_not_a_schema_matter_but_a_zero_is,
]


@pytest.mark.parametrize("mutate", MUTATIONS, ids=lambda m: m.__name__.strip("_"))
def test_t12_each_rule_rejects_its_violation(mutate):
    doc = load_datums()
    assert not _errors(doc)
    mutate(doc)
    assert _errors(doc)


@pytest.mark.parametrize("source", ["measured", "contractor_m01", "f2s", "supplier", "plan"])
def test_t12_verified_is_accepted_with_an_evidenced_source_and_a_ref(source):
    doc = load_datums()
    doc["datums"]["lift_mass_dold_v4_350"].update(source=source, status="verified", ref="drawing no. X, rev 2")
    assert not _errors(doc)



# --- the two rules V01 found and P15 implements -------------------------------------------------------------------------------

def test_t12_a_null_datum_is_a_placeholder_and_is_accepted_only_as_one():
    doc = load_datums()
    assert doc["datums"]["lift_mass_tl3_400"]["value"] is None
    assert not _errors(doc)


def _consumer_inertia(section, tmp_path):
    """The (ixx, iyy, izz) of lift_column_base_link in an expansion whose lift_column_section is `section`."""
    import xml.etree.ElementTree as ET

    import yaml

    from conftest import expand_xml
    doc = load_datums()
    doc["datums"]["lift_column_section"]["value"] = list(section)
    doc["revision"] = "2099-01-01.a"
    path = tmp_path / "datums_section.yaml"
    path.write_text(yaml.safe_dump(doc))
    root = ET.fromstring(expand_xml(datums_file=str(path), roadmap_masses="false"))   # the datum box (roadmap export off)
    inertia = root.find("link[@name='lift_column_base_link']/inertial/inertia")
    mass = float(root.find("link[@name='lift_column_base_link']/inertial/mass").get("value"))
    return mass, *(float(inertia.get(k)) for k in ("ixx", "iyy", "izz"))


def _lift_column_section_assertion(tmp_path):
    """x = fore-aft is the first entry, y = lateral the second. A box of x by y by h about the frame's own axes has
    ixx = m (y^2 + h^2)/12, iyy = m (x^2 + h^2)/12 and izz = m (x^2 + y^2)/12, so an asymmetric section tells the two apart:
    a swapped assignment in the xacro would exchange ixx and iyy."""
    from conftest import UPPERBODY
    h = UPPERBODY["column"]["dold_v4_350"]["retracted_length"]
    x, y = 0.200, 0.320
    mass, ixx, iyy, izz = _consumer_inertia((x, y), tmp_path)
    assert ixx == pytest.approx(mass * (y * y + h * h) / 12), "the second entry is not the lateral side"
    assert iyy == pytest.approx(mass * (x * x + h * h) / 12), "the first entry is not the fore-aft side"
    assert izz == pytest.approx(mass * (x * x + y * y) / 12)


def _expansion_with(datum, value, tmp_path):
    """The combined expansion with one datum replaced (the revision bumped so the guard is not the thing under test)."""
    import xml.etree.ElementTree as ET

    import yaml

    from conftest import expand_xml
    doc = load_datums()
    doc["datums"][datum]["value"] = list(value)
    doc["revision"] = "2099-01-01.a"
    path = tmp_path / f"datums_{datum}.yaml"
    path.write_text(yaml.safe_dump(doc))
    return ET.fromstring(expand_xml(datums_file=str(path)))


def _box(root, link, kind):
    return tuple(float(v) for v in root.find(f"link[@name='{link}']/{kind}/geometry/box").get("size").split())


def _lift_inner_section_assertion(tmp_path):
    """x fore-aft first, y lateral second, z the stage's length, on the box of the moving stage (the carriage's box). P-03 rev20.4 item 28 (DOLD
    STEP, 8 Nov 2026) gave the length, so the datum is a triple; before it the length was the stroke."""
    x, y, z = 0.200, 0.040, 0.410
    root = _expansion_with("lift_inner_section", (x, y, z), tmp_path)
    for kind in ("visual", "collision"):
        assert _box(root, "lift_carriage_link", kind) == pytest.approx((x, y, z)), f"the {kind} of the moving stage does not follow lift_inner_section"
    assert _box(root, "lift_column_base_link", "collision")[:2] != pytest.approx((x, y)), "the fixed stage must not follow the inner section"


def _adapter_plate_section_assertion(tmp_path):
    """x fore-aft first, y lateral second, thickness from geometry.adapter_plate_thickness; the rear edge stays adapter_plate_rear_extension behind
    the column's rear face (P-03 rev20.4 item 28: 65 mm; it was flush), so the plate's centre moves with its length."""
    from conftest import UPPERBODY
    x, y = 0.300, 0.080
    root = _expansion_with("adapter_plate_section", (x, y), tmp_path)
    t = UPPERBODY["geometry"]["adapter_plate_thickness"]
    for kind in ("visual", "collision"):
        assert _box(root, "adapter_plate_link", kind) == pytest.approx((x, y, t))
    origin = [float(v) for v in root.find("joint[@name='adapter_plate_joint']/origin").get("xyz").split()]
    column_x = load_datums()["datums"]["lift_column_section"]["value"][0]
    extension = load_datums()["datums"]["adapter_plate_rear_extension"]["value"]
    assert origin == pytest.approx([-column_x / 2 - extension + x / 2, 0.0, t / 2])


def _equipment_box_assertion(datum, link, collision):
    """P-03 item 24: x fore-aft (depth), y lateral, z vertical, on the named box of the equipment link, visual and collision. An
    asymmetric triple tells the three axes apart, so a swapped assignment in upper_body_equipment.xacro fails here."""
    def assertion(tmp_path):
        value = (0.031, 0.057, 0.083)
        root = _expansion_with(datum, value, tmp_path)
        box = root.find(f"link[@name='{link}']/collision[@name='{collision}']/geometry/box")
        assert box is not None, f"{link}: no collision {collision}"
        assert tuple(float(v) for v in box.get("size").split()) == pytest.approx(value), f"{datum}: the collision of {link} does not follow it"
        # the visuals are unnamed (P25); the one at the collision's origin is its twin
        origin = root.find(f"link[@name='{link}']/collision[@name='{collision}']/origin").get("xyz")
        twins = [v for v in root.findall(f"link[@name='{link}']/visual") if v.find("origin").get("xyz") == origin]
        assert len(twins) == 1 and tuple(float(v) for v in twins[0].find("geometry/box").get("size").split()) == pytest.approx(value), \
            f"{datum}: the visual of {link} does not follow it"
    return assertion


def _head_camera_bracket_post_section_assertion(tmp_path):
    """x fore-aft first, y lateral second, on the mount post's box; its height is derived (lens line minus profile top), not this datum."""
    x, y = 0.031, 0.057
    root = _expansion_with("head_camera_bracket_post_section", (x, y), tmp_path)
    box = root.find("link[@name='head_camera_bracket_link']/collision[@name='placeholder_head_camera_bracket_post']/geometry/box")
    assert tuple(float(v) for v in box.get("size").split())[:2] == pytest.approx((x, y))


# one entry per dimension list in datums.yaml: how the xacro consumes it, asserted against an expansion. A list with no entry here
# FAILS (test_t12_every_dimension_list_has_its_assignment_asserted): a pair of dimensions whose axes are only a note in prose is
# the V01-F1 defect (the 90 degree mast profile, wrong for six steps and not seen by one test).
AXIS_ASSERTIONS = {"lift_column_section": _lift_column_section_assertion, "lift_inner_section": _lift_inner_section_assertion,
                   "adapter_plate_section": _adapter_plate_section_assertion,
                   "getac_tablet_size": _equipment_box_assertion("getac_tablet_size", "getac_tablet_link", "placeholder_getac_tablet"),
                   "head_light_size": _equipment_box_assertion("head_light_size", "head_light_link", "placeholder_head_light"),
                   "second_estop_head_size": _equipment_box_assertion("second_estop_head_size", "second_estop_link",
                                                                      "placeholder_second_estop_head"),
                   "second_estop_collar_size": _equipment_box_assertion("second_estop_collar_size", "second_estop_link",
                                                                        "placeholder_second_estop_collar"),
                   "second_estop_contact_block_size": _equipment_box_assertion("second_estop_contact_block_size", "second_estop_link",
                                                                               "placeholder_second_estop_contact_block"),
                   "second_estop_housing_size": _equipment_box_assertion("second_estop_housing_size", "second_estop_link",
                                                                         "placeholder_second_estop_housing"),
                   "lift_flag_size": _equipment_box_assertion("lift_flag_size", "lift_flag_link", "placeholder_lift_flag"),
                   "head_camera_size": _equipment_box_assertion("head_camera_size", "head_camera_link", "placeholder_head_camera_body"),
                   "head_camera_bracket_foot_size": _equipment_box_assertion("head_camera_bracket_foot_size", "head_camera_bracket_link",
                                                                             "placeholder_head_camera_bracket_foot"),
                   "head_camera_bracket_post_section": _head_camera_bracket_post_section_assertion}
AXES_BY_LENGTH = {2: ("x", "y"), 3: ("x", "y", "z")}


def test_t12_every_dimension_list_declares_its_axis_convention():
    datums = load_datums()["datums"]
    lists = {n: d for n, d in datums.items() if isinstance(d["value"], list)}
    assert lists, "no dimension list in datums.yaml: the rule would be vacuous"
    for name, d in lists.items():
        assert len(d["value"]) in AXES_BY_LENGTH, f"{name}: a dimension list has two or three entries"
        convention = d.get("axis_convention")
        assert convention and tuple(convention) == AXES_BY_LENGTH[len(d["value"])], f"{name}: axis_convention {convention!r}"
        assert len(set(convention.values())) == len(convention), f"{name}: two entries on the same axis"


def test_t12_every_dimension_list_has_its_assignment_asserted(tmp_path):
    datums = load_datums()["datums"]
    lists = sorted(n for n, d in datums.items() if isinstance(d["value"], list))
    assert sorted(AXIS_ASSERTIONS) == lists, f"asserted: {sorted(AXIS_ASSERTIONS)}, in datums.yaml: {lists}"
    for name in lists:
        AXIS_ASSERTIONS[name](tmp_path)


def test_t12_the_lift_section_declares_fore_aft_then_lateral():
    assert load_datums()["datums"]["lift_column_section"]["axis_convention"] == {"x": "fore_aft", "y": "lateral"}
