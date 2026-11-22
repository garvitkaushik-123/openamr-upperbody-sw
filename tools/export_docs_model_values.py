#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Print the P-03 item 24 equipment values of the expanded URDF in the format of the docs model's CHEST-JSON and LIFT-JSON blocks.

    source /opt/ros/jazzy/setup.bash && source <workspace>/install/setup.bash
    python3 tools/export_docs_model_values.py [--urdf expanded.urdf] [--base-plate bp000] [--model-js <openamrobot-docs>/docs/assets/roadmap/model/model.js]

The values are read from the expanded URDF (link poses, box and cylinder sizes), in the docs model's frames: x from the column axis, z of
the chest above the column top at lift 0 (L1000), z of the stop sensors above the column base. So the output is what the robot description
draws, written the way openamrobot-docs docs/assets/roadmap/model/model.js writes it, and test_item24_docs_model_values.py compares it with
the model.js blocks the datums came from (tolerance 0.5 mm).

HEAD-CAMERA (model.js has no JSON block for it) is printed in model.js's own terms: ZED_X, ZED_ABOVE_SH, MOUNT_ABOVE_COL, the default head pitch,
the ZED box and the mount; docs_head_camera() reads the same values from model.js source.

Not exported (the URDF does not carry them): CHEST-JSON second_estop.glb_node and glb_axis (the docs model's handle on the base E-stop mesh
of the platform GLB), drawn.head_light.window (the diffuser window, not modelled); LIFT-JSON corner_x (the rod's corner bracket, not modelled).
Every exported value is read from the URDF (since P-03 rev20.4 item 28 none comes from a datum: the second E-stop is measured from the adapter
plate top, no longer from the static profile top).
"""
import argparse
import json
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

NOT_EXPORTED = {"CHEST-JSON": ["second_estop.glb_node", "second_estop.glb_axis", "drawn.head_light.window"], "LIFT-JSON": ["corner_x"],
                "HEAD-CAMERA": []}
BLOCKS = ("CHEST-JSON", "LIFT-JSON", "HEAD-CAMERA")
# model.js has no JSON block for the head camera: its values are M.DIMS entries and literals of upperBody() and M.build(). HEAD-CAMERA is this
# tool's name for them, in model.js's own terms: ZED_X (lens line from the column axis), ZED_ABOVE_SH, MOUNT_ABOVE_COL, the default head pitch,
# the ZED box (size and x about the lens line) and the mount ("head_camera_bracket": set-back behind the lens, foot box, post section).
HEAD_CAMERA_PATTERNS = {
    "ZED_X": r"\bZED_X: (-?[\d.]+)",
    "ZED_ABOVE_SH": r"\bZED_ABOVE_SH: (-?[\d.]+)",
    "MOUNT_ABOVE_COL": r"\bMOUNT_ABOVE_COL: (-?[\d.]+)",
    "pitch_deg": r"o\.headPitch === undefined \? (-?[\d.]+)",
    "body": r"box\(zed, ([\d.]+), ([\d.]+), ([\d.]+), mats\.black, (-?[\d.]+), 0, 0\)",
    "foot": r"box\(br, ([\d.]+), ([\d.]+), ([\d.]+), mats\.alu, D\.ZED_X - ([\d.]+), 0, y \+ ([\d.]+)\)",
    "post": r"box\(br, ([\d.]+), ([\d.]+), m - y, mats\.aluDark, D\.ZED_X - ([\d.]+), 0, \(y \+ m\) / 2\)",
}


def docs_head_camera(js_text):
    """The HEAD-CAMERA values of a model.js text (or of the source lines of the fixture), in the export's format. Raises if a pattern is missing
    or the mount's two boxes disagree on its set-back (the format would then not describe the drawing)."""
    import re
    found = {}
    for key, pattern in HEAD_CAMERA_PATTERNS.items():
        m = re.search(pattern, js_text)
        if not m:
            raise ValueError(f"HEAD-CAMERA {key}: pattern not found in model.js")
        found[key] = [float(g) for g in m.groups()]
    foot, post = found["foot"], found["post"]
    if foot[3] != post[2] or abs(foot[4] - foot[2] / 2) > 1e-12:
        raise ValueError("HEAD-CAMERA: the mount's foot and post are not drawn as this format assumes (same set-back, foot on the profile top)")
    return {"ZED_X": found["ZED_X"][0], "ZED_ABOVE_SH": found["ZED_ABOVE_SH"][0], "MOUNT_ABOVE_COL": found["MOUNT_ABOVE_COL"][0],
            "pitch_deg": found["pitch_deg"][0],
            "body": {"size": found["body"][:3], "x": found["body"][3]},
            "bracket": {"back": foot[3], "foot": foot[:3], "post": post[:2]}}
FROM_DATUM = {}
STOPS = ["L1000", "L1175", "L1350"]


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array(
        [[1, 0, 0], [0, cr, -sr], [0, sr, cr]])


def _origin(element):
    t = np.eye(4)
    o = element.find("origin") if element is not None else None
    if o is not None:
        t[:3, :3] = _rpy(*[float(v) for v in o.get("rpy", "0 0 0").split()])
        t[:3, 3] = [float(v) for v in o.get("xyz", "0 0 0").split()]
    return t


class Urdf:
    def __init__(self, root):
        self.root = root
        self.by_child = {j.find("child").get("link"): j for j in root.findall("joint")}

    def pose(self, base, link):
        """Pose of `link` in `base` at zero joint positions (fixed joints only on the path)."""
        t = np.eye(4)
        while link != base:
            joint = self.by_child[link]
            if joint.get("type") != "fixed":
                raise ValueError(f"{joint.get('name')} on the path {base} -> {link} is {joint.get('type')}")
            t = _origin(joint) @ t
            link = joint.find("parent").get("link")
        return t

    def shape(self, link, name=None):
        """(pose of the collision in the link, size list) of the link's collision `name` (or its only collision)."""
        collisions = self.root.findall(f"link[@name='{link}']/collision")
        if name is not None:
            collisions = [c for c in collisions if c.get("name") == name]
        assert len(collisions) == 1, f"{link}: {len(collisions)} collisions named {name!r}"
        c = collisions[0]
        g = c.find("geometry")
        if g.find("box") is not None:
            size = [float(v) for v in g.find("box").get("size").split()]
        else:
            size = [2 * float(g.find("cylinder").get("radius")), float(g.find("cylinder").get("length"))]
        return _origin(c), size


def r(x):
    return round(float(x), 6)


def _span(centre, size):
    return [r(centre - size / 2), r(centre + size / 2)]


def export(root, datums):
    u = Urdf(root)
    _, column = u.shape("lift_column_base_link")                   # outer stage box: [x, y, fixed length]
    sec_x = column[0]

    # ---- CHEST-JSON (frames: lift_carriage_link = column axis at the column top, lift 0)
    plate = u.pose("lift_carriage_link", "adapter_plate_link")
    _, plate_box = u.shape("adapter_plate_link")
    plate_top = plate[2, 3] + plate_box[2] / 2
    profile = u.pose("lift_carriage_link", "arm_profile_link")
    _, profile_box = u.shape("arm_profile_link")
    front = profile[0, 3] + profile_box[0] / 2
    drawn = {}
    for item in ("head_light", "getac_tablet"):
        pose = u.pose("lift_carriage_link", f"{item}_link")
        local, size = u.shape(f"{item}_link")
        centre = pose @ local
        assert abs(centre[0, 3] - size[0] / 2 - front) < 1e-9, f"{item}: rear face is not on the arm-profile front face"
        drawn[item] = {"z": r(centre[2, 3]), "size": [r(s) for s in size]}
    tablet = drawn["getac_tablet"]["size"]
    drawn["getac_tablet"]["orientation"] = "portrait" if tablet[2] > tablet[1] else "landscape"
    order = sorted(drawn, key=lambda k: drawn[k]["z"])
    tablet_bottom = drawn["getac_tablet"]["z"] - drawn["getac_tablet"]["size"][2] / 2
    light_top = drawn["head_light"]["z"] + drawn["head_light"]["size"][2] / 2
    estop_joint = u.by_child["second_estop_link"]
    estop = u.pose("lift_carriage_link", "second_estop_link")
    on_plate = estop_joint.find("parent").get("link") == "adapter_plate_link" and estop[2, 0] > 1 - 1e-9   # button axis (+x) up
    assert on_plate and abs(estop[0, 3]) < 1e-9 and abs(estop[1, 3]) < 1e-9, "the second E-stop is not on the plate top above the column axis"
    chest = {"second_estop": {"position": "adapter_top", "panel_above_plate": r(estop[2, 3] - plate_top)},
             "order": order, "drawn": drawn, "gap_light_tablet": r(tablet_bottom - light_top)}

    # ---- LIFT-JSON (x from the column axis; sensor z above the column base; flag, rod and plate in the carriage frame at lift 0)
    sensors = [u.pose("lift_column_base_link", f"lift_stop_sensor_{s.lower()}_link") for s in STOPS]
    s0 = sensors[0]
    _, (d, length) = u.shape("lift_stop_sensor_l1000_link", "placeholder_lift_stop_sensor")
    face_leg, face_leg_size = u.shape("lift_stop_sensor_l1000_link", "placeholder_lift_sensor_bracket_face_leg")
    sensor_leg, sensor_leg_size = u.shape("lift_stop_sensor_l1000_link", "placeholder_lift_sensor_bracket_sensor_leg")
    flag = u.pose("lift_carriage_link", "lift_flag_link")
    _, flag_size = u.shape("lift_flag_link")
    rod = u.pose("lift_carriage_link", "lift_flag_rod_link")
    rod_local, rod_size = u.shape("lift_flag_rod_link")
    underside = plate[2, 3] - plate_box[2] / 2
    rear_x, axis_x, face_y = -sec_x / 2, s0[0, 3], s0[1, 3]
    assert abs((s0[:3, :3] @ np.array([0, 1, 0]))[1] - 1) < 1e-9, "the sensor frame is turned: its y is not the robot's y"
    assert abs(axis_x + face_leg[0, 3] + face_leg_size[0] / 2 - rear_x) < 1e-9, "the bracket's face leg is not on the column rear face"
    rod_x, rod_y = _span(rod[0, 3], rod_size[0]), _span(rod[1, 3], rod_size[1])
    lift = {"face": "rear" if axis_x < rear_x else ("left" if face_y > 0 else "right"),
            "z": [r(p[2, 3]) for p in sensors], "stops": STOPS,
            "sensor": {"d": r(d), "len": r(length)},
            "rear_x": r(rear_x), "axis_x": r(axis_x), "face_y": r(face_y),
            "gap": r(flag[1, 3] - flag_size[1] / 2 - face_y),
            "flag": [r(s) for s in flag_size],
            "flag_x": _span(flag[0, 3], flag_size[0]), "flag_y": _span(flag[1, 3], flag_size[1]),
            "flag_below_plate": r(underside - flag[2, 3]),
            "rod": {"w": r(rod_size[0]), "len": r(rod_size[2])},
            "rod_x": rod_x, "rod_y": rod_y,
            "rod_clear": r(axis_x - d / 2 - rod_x[1]),
            "bracket": {"leg": r(sensor_leg_size[0]), "t": r(sensor_leg_size[1]), "at": r(rear_x - axis_x)},
            "bracket_h": r(face_leg_size[2]),
            "plate_rear_x": r(plate[0, 3] - plate_box[0] / 2)}
    assert abs(rod[2, 3] + rod_local[2, 3] + rod_size[2] / 2 - underside) < 1e-9, "the rod does not hang from the plate underside"
    assert abs(underside) < 1e-9, "the adapter plate does not rest on the carriage frame (the column top at lift 0)"
    # ---- HEAD-CAMERA (lift_carriage_link frame, lift 0; option B's lens frame head_camera_link carries the body; hp from the expansion)
    lens = u.pose("lift_carriage_link", "head_camera_link")
    mount = u.pose("lift_carriage_link", "openarm_left_base_link")
    body_local, body = u.shape("head_camera_link", "placeholder_head_camera_body")
    assert abs(body_local[1, 3]) < 1e-12 and abs(body_local[2, 3]) < 1e-12, "the ZED body is not centred on the lens line"
    bracket = u.pose("lift_carriage_link", "head_camera_bracket_link")
    _, foot = u.shape("head_camera_bracket_link", "placeholder_head_camera_bracket_foot")
    _, post = u.shape("head_camera_bracket_link", "placeholder_head_camera_bracket_post")
    profile_top = profile[2, 3] + profile_box[2] / 2
    assert abs(bracket[2, 3] - profile_top) < 1e-9, "the mount foot is not on the arm-profile top"
    assert abs(bracket[2, 3] + post[2] - lens[2, 3]) < 1e-9, "the mount post does not reach the lens line"
    head = {"ZED_X": r(lens[0, 3]), "ZED_ABOVE_SH": r(lens[2, 3] - mount[2, 3]), "MOUNT_ABOVE_COL": r(mount[2, 3]),
            "pitch_deg": r(np.degrees(np.arctan2(-lens[2, 0], lens[0, 0]))),
            "body": {"size": [r(s) for s in body], "x": r(body_local[0, 3])},
            "bracket": {"back": r(lens[0, 3] - bracket[0, 3]), "foot": [r(s) for s in foot], "post": [r(post[0]), r(post[1])]}}
    return {"CHEST-JSON": chest, "LIFT-JSON": lift, "HEAD-CAMERA": head}


def expand(base_plate):
    import xacro
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory("openamrobot_upperbody_description")
    return xacro.process_file(os.path.join(share, "urdf", "combined_robot.xacro"),
                              mappings={"base_plate": base_plate, "arm_package": "none"}).toxml()


def load_datums(path=None):
    import yaml
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory("openamrobot_upperbody_description"), "config", "datums.yaml")
    with open(path) as f:
        return yaml.safe_load(f)["datums"]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--urdf", help="an expanded URDF (default: expand the installed combined_robot.xacro)")
    parser.add_argument("--base-plate", default="bp000", choices=("bp000", "bp050"), help="the values do not depend on it")
    parser.add_argument("--datums", help="datums.yaml (default: the installed one)")
    parser.add_argument("--model-js", help="also compare with the blocks of this model.js (0.5 mm); exit 1 on a difference")
    args = parser.parse_args(argv)
    text = open(args.urdf).read() if args.urdf else expand(args.base_plate)
    values = export(ET.fromstring(text), load_datums(args.datums))
    for block in BLOCKS:
        print(f"/*{block}*/{json.dumps(values[block])}/*{block}-END*/")
    print("// not exported (not in the URDF): " + ", ".join(f"{b} {k}" for b, keys in NOT_EXPORTED.items() for k in keys), file=sys.stderr)
    if FROM_DATUM:
        print("// from a datum, not the URDF: " + ", ".join(f"{k} <- {v}" for k, v in FROM_DATUM.items()), file=sys.stderr)
    if args.model_js:
        import re
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "ros2", "src",
                                        "openamrobot_upperbody_description", "test"))
        from test_item24_docs_model_values import differences
        with open(args.model_js) as f:
            text = f.read()
        problems = []
        for block in ("CHEST-JSON", "LIFT-JSON"):
            match = re.search(rf"/\*{block}\*/(.*?)/\*{block}-END\*/", text)
            if not match:
                problems.append(f"{block}: not in {args.model_js}")
                continue
            problems += [f"{block} {p}" for p in differences(json.loads(match.group(1)), values[block], NOT_EXPORTED[block])]
        problems += [f"HEAD-CAMERA {p}" for p in differences(docs_head_camera(text), values["HEAD-CAMERA"], NOT_EXPORTED["HEAD-CAMERA"])]
        print("// model.js comparison (0.5 mm): " + ("; ".join(problems) if problems else "equal"), file=sys.stderr)
        return 1 if problems else 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
