# SPDX-License-Identifier: MIT
# Copyright (c) 2026 OpenAMRobot (Botshare LTD)
"""Build the description for every configuration combination (no ROS graph).

Option B values (P-03 revision 18.9 item 18): arm mount point at column axis
+0.180 m and y +/-0.031 m; shoulder z 1.000 m at lift 0 and 1.350 m at lift
0.350 m; head camera at or below 1.700 m at full lift. Default column axis x
+0.060 m at bp000 and +0.110 m at bp050, so J1_A x is +0.240 m and +0.290 m
(P-03 rev19.0 item 19).
"""

import math
import xml.etree.ElementTree as ET

import pytest
import xacro

from upperbody_model import (AXES, CONFIG, EXPECTED_FRAMES, buildable,
                             combinations, expand, frame_poses)

TOL = 1e-9
# Default column axis x and OpenArm J1_A x per base plate (P-03 rev19.0 item 19).
COLUMN_AXIS_X = {'bp000': 0.060, 'bp050': 0.110}
J1_A_X = {'bp000': 0.240, 'bp050': 0.290}
ALL = list(combinations())
BUILDABLE = [c for c in ALL if buildable(c)]
PENDING = [c for c in ALL if not buildable(c)]


def _id(combo):
    return '-'.join(combo.values())


def test_configuration_axes_are_option_b():
    assert AXES['column'] == ['dold_v4_350', 'tl3_400']
    assert AXES['base_plate'] == ['bp000', 'bp050']
    assert AXES['head_pitch'] == ['hp15', 'hp20', 'hp25', 'hp30', 'hp35']
    assert AXES['base_tilt'] == ['bt05', 'bt10', 'bt15']
    assert len(ALL) == 2 * 2 * 5 * 3
    assert BUILDABLE, 'no buildable combination: the test would be empty'


@pytest.mark.parametrize('combo', BUILDABLE, ids=_id)
def test_combination(combo):
    urdf = expand(**combo)
    root = ET.fromstring(urdf)
    links = [link.get('name') for link in root.findall('link')]
    assert len(links) == len(set(links)), 'duplicate link names'
    assert sorted(links) == sorted(EXPECTED_FRAMES)
    assert 'openarm_body_link0' not in links

    stroke = CONFIG['column'][combo['column']]['stroke']
    lift = root.find("joint[@name='lift_joint']")
    assert lift.get('type') == 'prismatic'
    assert float(lift.find('limit').get('lower')) == 0.0
    assert math.isclose(float(lift.find('limit').get('upper')), 0.350)

    column_x = COLUMN_AXIS_X[combo['base_plate']]
    pitch = math.radians(CONFIG['head_pitch'][combo['head_pitch']]['pitch_down_deg'])
    for q, shoulder_z in ((0.0, 1.000), (stroke, 1.350)):
        poses = frame_poses(urdf, {'lift_joint': q})
        # Every frame is reachable from the root, so TF has no missing frame.
        assert sorted(poses) == sorted(EXPECTED_FRAMES)
        axis_x = poses['lift_column_base_link'][1][0]
        assert abs(axis_x - column_x) < TOL
        for side, y in (('left', 0.031), ('right', -0.031)):
            r, t = poses[f'openarm_{side}_base_link']
            assert abs(t[0] - (axis_x + 0.180)) < TOL, (side, t)
            assert abs(t[0] - J1_A_X[combo['base_plate']]) < TOL, (side, t)
            assert abs(t[1] - y) < TOL, (side, t)
            assert abs(t[2] - shoulder_z) < TOL, (side, q, t)
            assert all(abs(r[i][j] - (i == j)) < TOL for i in range(3) for j in range(3))
        r, cam = poses['head_camera_link']
        assert abs(cam[0] - (axis_x + 0.180 + 0.060)) < TOL   # P-03 rev20.4 item 28 (8 Nov 2026); was 0.042 (P-03 rev20.0 item 18)
        assert abs(cam[1]) < TOL
        assert abs(cam[2] - (shoulder_z + 0.223)) < TOL
        assert cam[2] <= CONFIG['geometry']['max_assembled_height']
        # Optical axis (camera x) points forward and down by the pitch.
        assert abs(r[2][0] - (-math.sin(pitch))) < TOL
        assert abs(r[0][0] - math.cos(pitch)) < TOL


def test_arm_profile_spans_844_to_1184_at_retracted_lift():
    poses = frame_poses(expand(column='dold_v4_350', base_plate='bp000',
                               head_pitch='hp30', base_tilt='bt10'))
    z = poses['arm_profile_link'][1][2]
    half = CONFIG['geometry']['arm_profile']['length'] / 2
    assert abs((z - half) - 0.844) < TOL
    assert abs((z + half) - 1.184) < TOL


def test_column_axis_x_bp000_argument_shifts_both_positions():
    for bp, offset in (('bp000', 0.0), ('bp050', 0.050)):
        poses = frame_poses(expand(column_axis_x_bp000='0.1', column='dold_v4_350',
                                   base_plate=bp, head_pitch='hp30', base_tilt='bt10'))
        assert abs(poses['openarm_left_base_link'][1][0] - (0.1 + offset + 0.180)) < TOL


@pytest.mark.parametrize('combo', PENDING, ids=_id)
def test_pending_column_is_refused_not_guessed(combo):
    with pytest.raises(xacro.XacroException, match='pending supplier data'):
        expand(**combo)


@pytest.mark.parametrize('axis,value', [
    ('base_plate', 'bp100'), ('base_plate', 'bp150'), ('base_plate', 'bp200'),
    ('head_pitch', 'hp40'), ('base_tilt', 'bt00'), ('column', 'mast_1350'),
])
def test_removed_or_unknown_ids_are_rejected(axis, value):
    combo = {'column': 'dold_v4_350', 'base_plate': 'bp000',
             'head_pitch': 'hp30', 'base_tilt': 'bt10', axis: value}
    with pytest.raises(xacro.XacroException, match=f"unknown {axis} '{value}'"):
        expand(**combo)
