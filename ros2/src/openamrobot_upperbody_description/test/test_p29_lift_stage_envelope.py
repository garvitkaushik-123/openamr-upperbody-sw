# SPDX-License-Identifier: MIT
"""P29: quantify the current lift-stage placeholder collision-box envelope.

This test calculates intersections between the solid-box collision geometries
of the fixed and moving lift stages. It does NOT establish physical collision,
hollow-housing clearance, wall thickness, or minimum mechanical engagement.

Those require the DOLD supplier STEP geometry to be available and modelled.
"""
import pytest

from conftest import LIFT_STROKE, expand, load_datums


@pytest.fixture(scope="module")
def model():
    root = expand(base_plate="bp000", arm_package="openarm")
    datums = load_datums()["datums"]

    def box(link):
        element = root.find(f"link[@name='{link}']/collision")
        size = tuple(
            float(value)
            for value in element.find("geometry/box").get("size").split()
        )
        origin = tuple(
            float(value)
            for value in element.find("origin").get("xyz").split()
        )
        return size, origin

    joint = root.find("joint[@name='lift_joint']")
    joint_origin = tuple(
        float(value)
        for value in joint.find("origin").get("xyz").split()
    )
    joint_limit = joint.find("limit")

    return {
        "outer_size": box("lift_column_base_link")[0],
        "outer_origin": box("lift_column_base_link")[1],
        "inner_size": box("lift_carriage_link")[0],
        "inner_origin": box("lift_carriage_link")[1],
        "joint_origin": joint_origin,
        "lower": float(joint_limit.get("lower")),
        "upper": float(joint_limit.get("upper")),
        "outer_datum": datums["lift_column_section"]["value"],
        "inner_datum": datums["lift_inner_section"]["value"],
        "static_top_below_column_top": datums[
            "lift_static_profile_top_below_column_top"
        ]["value"],
        "stroke": LIFT_STROKE,
    }


def _z_intersection(outer, inner, lift):
    outer_size, outer_origin = outer
    inner_size, inner_origin, joint_origin = inner

    outer_min = outer_origin[2] - outer_size[2] / 2
    outer_max = outer_origin[2] + outer_size[2] / 2

    inner_center = joint_origin[2] + lift + inner_origin[2]
    inner_min = inner_center - inner_size[2] / 2
    inner_max = inner_center + inner_size[2] / 2

    return max(0.0, min(outer_max, inner_max) - max(outer_min, inner_min))


def test_p29_stage_sections_match_the_provisional_supplier_datums(model):
    assert model["outer_size"][:2] == pytest.approx(model["outer_datum"])
    assert model["inner_size"] == pytest.approx(model["inner_datum"])

    # Both placeholder boxes are centred on the same column axis.
    assert model["outer_origin"][:2] == pytest.approx((0.0, 0.0))
    assert model["inner_origin"][:2] == pytest.approx((0.0, 0.0))

    # Nominal cross-section margins, not verified physical clearances.
    margin_x = (model["outer_size"][0] - model["inner_size"][0]) / 2
    margin_y = (model["outer_size"][1] - model["inner_size"][1]) / 2
    assert margin_x == pytest.approx(0.01525)
    assert margin_y == pytest.approx(0.00335)


def test_p29_joint_travel_matches_the_configured_stroke(model):
    assert model["lower"] == pytest.approx(0.0)
    assert model["upper"] == pytest.approx(model["stroke"])
    assert model["joint_origin"][2] == pytest.approx(0.530)


@pytest.mark.parametrize(
    "lift, expected_intersection",
    [
        (0.000, 0.445),
        (0.175, 0.270),
        (0.350, 0.095),
    ],
)
def test_p29_reports_solid_box_z_intersection(model, lift, expected_intersection):
    outer = (model["outer_size"], model["outer_origin"])
    inner = (
        model["inner_size"],
        model["inner_origin"],
        model["joint_origin"],
    )

    intersection = _z_intersection(outer, inner, lift)
    assert intersection == pytest.approx(expected_intersection)


def test_p29_does_not_claim_mechanical_clearance(model):
    """The outer collision is a solid box; supplier wall geometry is not modelled."""
    assert model["static_top_below_column_top"] == pytest.approx(0.025)
    assert model["inner_size"][1] < model["outer_size"][1]
    # A positive box intersection is expected in this placeholder model.
    # It is not evidence that the real telescoping stages physically interfere.
    assert _z_intersection(
        (model["outer_size"], model["outer_origin"]),
        (model["inner_size"], model["inner_origin"], model["joint_origin"]),
        model["upper"],
    ) > 0.0
