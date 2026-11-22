#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Diagnosis of the A7 M02 lift_joint excursion (upperbody-sw #48): one M02 stack, one spawn height, everything recorded.

    source /opt/ros/jazzy/setup.bash && source <ws>/install/setup.bash
    python3 a7_lift_settle_probe.py --spawn-z 0.15 --out probe_015.json
    python3 a7_lift_settle_probe.py --summarise probe_015.json probe_0005.json

It starts `moveit.launch.py sim:=gazebo headless:=true physics_engine:=<Bullet Featherstone>` exactly as M02 does, with only `spawn_z`
changed, on its own ROS_DOMAIN_ID and GZ_PARTITION, and records for lift_joint (prismatic, METRES):
  - ground truth: lift_carriage_link's z in the model frame (Gazebo /world/<w>/dynamic_pose/info) minus the joint origin, so q_gt = 0 at L1000;
  - raw: lift_joint on /arm/joint_states (joint_state_broadcaster) and on /joint_states (the merged topic the clamp reads);
  - clamped: lift_joint on /joint_states_clamped (what move_group plans from);
  - controller feedback: /lift_controller/controller_state (feedback, reference, error), once the controller is active;
  - the scenario phase each sample falls in: settle (spawn to all controllers active), start_gate, park, lift_L1175, lift_L1000, poses.
Then the clamp's own counters (/joint_state_clamp/get_stats). Nothing is changed in the stack: no epsilon, no tolerance, no limit.
Simulation only: it says how the model settles in this simulator and nothing about the DOLD column.
"""
import argparse
import json
import math
import os
import random
import signal
import subprocess
import sys
import threading
import time

EPSILON_M = 1e-5               # the clamp's epsilon (joint_state_clamp.EPSILON); repeated here only for the summary, never used to gate
LIFT_JOINT = "lift_joint"
ENGINE = "gz-physics-bullet-featherstone-plugin"
CONTROLLERS = ("joint_state_broadcaster", "left_arm_controller", "right_arm_controller", "left_gripper_controller",
               "right_gripper_controller", "lift_controller")


def lift_joint_origin_z(urdf_xml):
    """z of lift_carriage_link in the frame of the model root at q = 0: the sum of the fixed z offsets from the root to lift_joint's child.
    Returns (z, x) so a caller can check the carriage sits on the column axis."""
    import xml.etree.ElementTree as ET
    root = ET.fromstring(urdf_xml)
    by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
    link, x, z = "lift_carriage_link", 0.0, 0.0
    while link in by_child:
        joint = by_child[link]
        origin = joint.find("origin")
        if origin is not None and origin.get("xyz"):
            ox, _, oz = (float(v) for v in origin.get("xyz").split())
            if origin.get("rpy") and any(abs(float(v)) > 1e-12 for v in origin.get("rpy").split()):
                raise ValueError(f"{joint.get('name')} has a rotation: the simple z sum does not apply")
            x, z = x + ox, z + oz
        link = joint.find("parent").get("link")
    return z, x


def summarise(samples, epsilon=EPSILON_M):
    """Pure: the per-phase extremes of every lift_joint source and the readings past the lower limit by more than `epsilon`.
    `samples` is a list of {"t", "source", "value", "phase", ...}. Units are metres."""
    phases, beyond = {}, []
    for s in samples:
        row = phases.setdefault(s["phase"], {}).setdefault(s["source"], {"n": 0, "min": math.inf, "max": -math.inf})
        row["n"] += 1
        row["min"] = min(row["min"], s["value"])
        row["max"] = max(row["max"], s["value"])
        if s["source"] in ("raw_arm", "raw_merged", "clamped", "ground_truth") and -s["value"] > epsilon:
            beyond.append(s)
    first = {}
    for s in beyond:
        first.setdefault(s["source"], s)
    return {"per_phase": phases, "beyond_epsilon_count": {src: sum(1 for s in beyond if s["source"] == src) for src in first},
            "first_beyond_epsilon": first, "worst": {src: min(s["value"] for s in beyond if s["source"] == src) for src in first}}


def ground_truth_from_poses(poses, joint_origin_z, model="openamrobot", carriage="lift_carriage_link"):
    """Lift joint position from one Gazebo world-frame pose message.

    poses: (name, x, z) per entry. The carriage world z minus the model root world z minus the URDF
    root-to-joint origin z is the joint position; the spawn height and any settling of the base cancel.
    Without the model pose in the same message there is no ground truth (never a world-z guess).
    Returns (model_z, ground_truth, carriage_x); entries are None when absent.
    """
    model_z = carriage_z = carriage_x = None
    for name, x, z in poses:
        if name == model:
            model_z = z
        elif name == carriage:
            carriage_z, carriage_x = z, x
    if model_z is None or carriage_z is None:
        return model_z, None, carriage_x
    return model_z, carriage_z - model_z - joint_origin_z, carriage_x


class Recorder:
    def __init__(self, node, joint_origin_z, world):
        from control_msgs.msg import JointTrajectoryControllerState
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        self.node, self.z0, self.lock = node, joint_origin_z, threading.Lock()
        self.samples, self.phase, self.t0 = [], "settle", time.monotonic()
        for topic, source in (("/arm/joint_states", "raw_arm"), ("/joint_states", "raw_merged"), ("/joint_states_clamped", "clamped")):
            node.create_subscription(JointState, topic, lambda m, s=source: self.on_joint_state(m, s), qos_profile_sensor_data)
        node.create_subscription(JointTrajectoryControllerState, "/lift_controller/controller_state", self.on_controller, 10)
        self.gz_node = None
        try:
            from gz.msgs10.pose_v_pb2 import Pose_V
            from gz.transport13 import Node as GzNode
            self.gz_node = GzNode()
            self.gz_node.subscribe(Pose_V, f"/world/{world}/dynamic_pose/info", self.on_poses)
        except ImportError as error:  # the gz python bindings need their vendor lib dirs on LD_LIBRARY_PATH
            print(f"probe: no ground truth ({error})", file=sys.stderr)

    def add(self, source, value, stamp=None, **extra):
        with self.lock:
            self.samples.append({"t": round(time.monotonic() - self.t0, 4), "stamp": stamp, "source": source, "value": value,
                                 "phase": self.phase, **extra})

    def on_joint_state(self, msg, source):
        if LIFT_JOINT in msg.name:
            i = msg.name.index(LIFT_JOINT)
            if i < len(msg.position):
                self.add(source, msg.position[i], msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def on_controller(self, msg):
        if msg.feedback.positions:
            self.add("controller_feedback", msg.feedback.positions[0], msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                     reference=msg.reference.positions[0] if msg.reference.positions else None,
                     error=msg.error.positions[0] if msg.error.positions else None)

    def on_poses(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9      # gz.msgs.Time: sec and nsec
        model_z, ground_truth, x = ground_truth_from_poses(
            ((p.name, p.position.x, p.position.z) for p in msg.pose), self.z0)
        if model_z is not None:
            self.add("model_z", model_z, stamp)
        if ground_truth is not None:
            self.add("ground_truth", ground_truth, stamp, x=x)


def controller_states(node, timeout=2.0):
    import rclpy
    from controller_manager_msgs.srv import ListControllers
    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            return {}
        future = client.call_async(ListControllers.Request())
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        return {c.name: c.state for c in future.result().controller} if future.done() and future.result() else {}
    finally:
        node.destroy_client(client)


def run(spawn_z, out, scenario=True, engine=ENGINE):
    os.environ["ROS_DOMAIN_ID"] = str(random.randint(201, 230))
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ["GZ_PARTITION"] = f"probe_{os.getpid()}"
    import rclpy
    import xacro
    from ament_index_python.packages import get_package_share_directory
    from openamrobot_upperbody_moveit.move_group_client import MoveGroupClient
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "test"))
    import a7_support as a7

    urdf = xacro.process_file(os.path.join(get_package_share_directory("openamrobot_upperbody_description"), "urdf", "combined_robot.xacro"),
                              mappings={"hardware_profile": "gazebo"}).toxml()
    z0, x0 = lift_joint_origin_z(urdf)
    log = open(out + ".launch.log", "w")
    launch = subprocess.Popen(["ros2", "launch", "openamrobot_upperbody_bringup", "moveit.launch.py", "sim:=gazebo", "rviz:=false",
                               "headless:=true", f"physics_engine:={engine}", f"gazebo_spawn_z:={spawn_z}"],
                              stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node("a7_lift_settle_probe", parameter_overrides=[])
    recorder = Recorder(node, z0, "empty")
    client = MoveGroupClient(node)
    record = {"spawn_z_m": float(spawn_z), "engine": engine, "lift_joint_origin_z_m": z0, "carriage_axis_x_m": x0, "units": "m",
              "events": {}, "moves": []}
    try:
        t0 = recorder.t0
        deadline = time.monotonic() + 240
        while time.monotonic() < deadline:
            client.spin_until(lambda: False, 0.5)
            states = controller_states(node)
            for name, state in states.items():
                if state == "active" and f"{name}_active" not in record["events"]:
                    record["events"][f"{name}_active"] = round(time.monotonic() - t0, 3)
            if states and all(states.get(c) == "active" for c in CONTROLLERS):
                break
        record["controllers"] = controller_states(node)
        if scenario and client.wait_for_server(60):
            recorder.phase = "start_gate"
            inside, start = a7.start_gate(client, 60.0)
            record["start_gate"] = start
            recorder.phase = "park"
            row, failure = a7.park_both_arms(client)
            record["moves"].append(row)
            for stop in ("L1175", "L1000"):
                recorder.phase = f"lift_{stop}"
                rows, failures = a7.move_lift(client, [stop])
                record["moves"] += rows
            recorder.phase = "poses"
            rows, failures = a7.run_poses(client, a7.named_pose_plan(), 240.0)
            record["moves"] += rows
        recorder.phase = "end"
        client.settle(2.0)
        record["clamp"] = client.clamp_stats()
    finally:
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(60)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
        subprocess.run(["pkill", "-f", f"GZ_PARTITION={os.environ['GZ_PARTITION']}"], check=False)
        record["launch_returncode"] = launch.returncode
        with recorder.lock:
            record["samples"] = list(recorder.samples)
        record["summary"] = summarise(record["samples"])
        with open(out, "w") as f:
            json.dump(record, f, indent=1, default=str)
        node.destroy_node()
        rclpy.shutdown()
    return record


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--spawn-z", default="0.15")
    parser.add_argument("--out", default="probe.json")
    parser.add_argument("--engine", default=ENGINE, help="gz-physics plugin (default: Bullet Featherstone, what M02 passes)")
    parser.add_argument("--settle-only", action="store_true", help="record the settling only, no MoveIt scenario")
    parser.add_argument("--summarise", nargs="*", help="print the summaries of earlier probe files and exit")
    args = parser.parse_args(argv)
    if args.summarise:
        for path in args.summarise:
            data = json.load(open(path))
            print(json.dumps({"file": path, "spawn_z_m": data["spawn_z_m"], "events": data["events"], "clamp": data.get("clamp"),
                              "summary": data["summary"]}, indent=1, default=str))
        return 0
    record = run(args.spawn_z, args.out, scenario=not args.settle_only, engine=args.engine)
    print(json.dumps({"spawn_z_m": record["spawn_z_m"], "events": record["events"], "summary": record["summary"]}, indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
