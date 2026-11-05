#!/usr/bin/env python3
"""Exercise real ROS 2 launch, DDS, TF, services and map export without hardware.

Examples (after sourcing ROS and the colcon workspace):
  python3 tests/ros_smoke.py --mode toy --output /results/ros_toy
  python3 tests/make_test_bag.py --output /results/test_bag
  python3 tests/ros_smoke.py --mode bag --bag-path /results/test_bag --output /results/ros_bag

The tests are bounded by wall time and work while ROS /clock is paused. They
never command motion. Their subprocess is isolated into its own process group.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
import yaml


class Observer(Node):
    def __init__(self):
        super().__init__("tp_slam_transport_smoke")
        self.status = None
        self.map = None
        self.pose = None
        self.done = False
        self.maps_received = 0
        self.poses_received = 0
        self.cmd_vel_received = 0
        self.stale_map_tf_received = 0
        self.invalid_pose_received = 0
        persistent = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, persistent)
        self.create_subscription(String, "/slam/status", self.on_status, persistent)
        self.create_subscription(String, "/toy/done", self.on_done, persistent)
        self.create_subscription(PoseWithCovarianceStamped, "/slam_pose", self.on_pose, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, qos_profile_sensor_data)
        self.create_subscription(TFMessage, "/tf", self.on_tf, qos_profile_sensor_data)
        self.service_clients = {name: self.create_client(Trigger, "/slam/" + name)
                        for name in ("pause", "resume", "save_map", "reset")}

    def on_map(self, message):
        self.map = message
        self.maps_received += 1

    def on_status(self, message):
        self.status = json.loads(message.data)

    def on_done(self, message):
        self.done = message.data == "complete"

    def on_pose(self, message):
        self.pose = message
        self.poses_received += 1
        pose = message.pose.pose
        values = (pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                  *message.pose.covariance)
        if not all(math.isfinite(value) for value in values):
            self.invalid_pose_received += 1

    def on_cmd_vel(self, message):
        del message
        self.cmd_vel_received += 1

    def on_tf(self, message):
        for transform in message.transforms:
            if (transform.header.frame_id == "map" and transform.child_frame_id == "odom"
                    and abs(transform.transform.translation.x) > 50):
                self.stale_map_tf_received += 1


def wait_until(node, predicate, deadline, description, process):
    while time.monotonic() < deadline:
        if predicate():
            return
        if process.poll() is not None:
            raise AssertionError(f"Launch exited ({process.returncode}) while waiting for {description}")
        rclpy.spin_once(node, timeout_sec=min(0.05, max(0, deadline - time.monotonic())))
        if node.status and node.status.get("state") == "ERROR":
            raise AssertionError(f"SLAM entered ERROR while waiting for {description}: {node.status}")
    raise AssertionError(f"Timeout waiting for {description}; last status={node.status}")


def call(node, name, expected_success, deadline, process):
    client = node.service_clients[name]
    wait_until(node, client.service_is_ready, deadline, name + " service", process)
    future = client.call_async(Trigger.Request())
    wait_until(node, future.done, deadline, name + " response", process)
    response = future.result()
    if response is None or response.success != expected_success:
        raise AssertionError(f"{name}: expected success={expected_success}, got {response}")
    return {"service": name, "success": response.success, "message": response.message}


def validate_export(output):
    metadata_path, pgm_path = output / "map.yaml", output / "map.pgm"
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf8"))
    if metadata["image"] != pgm_path.name:
        raise AssertionError("Map YAML image path does not match exported PGM")
    if not metadata["resolution"] > 0 or len(metadata["origin"]) != 3:
        raise AssertionError("Invalid map metadata")
    if not 0 < metadata["free_thresh"] < metadata["occupied_thresh"] < 1:
        raise AssertionError("Invalid map thresholds")
    magic, dimensions, maximum, pixels = pgm_path.read_bytes().split(b"\n", 3)
    width, height = (int(value) for value in dimensions.split())
    if magic != b"P5" or maximum != b"255" or len(pixels) != width * height:
        raise AssertionError("Malformed binary PGM")
    values, counts = np.unique(np.frombuffer(pixels, dtype=np.uint8), return_counts=True)
    histogram = {str(int(value)): int(count) for value, count in zip(values, counts)}
    if histogram.get("0", 0) < 50 or histogram.get("254", 0) < 500:
        raise AssertionError(f"Export has too little occupied/free evidence: {histogram}")
    if histogram.get("128", 0) == 0:
        raise AssertionError("Unknown space was not preserved in export")
    return {"width": width, "height": height, "resolution": metadata["resolution"],
            "pixel_histogram": histogram}


def run(args, summary):
    output = args.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    command = ["ros2", "launch", "tp_slam", "slam.launch.py", f"mode:={args.mode}",
               "rviz:=false", "toy_rate:=10", "scan_queue_size:=100",
               f"odom_source:={args.odom_source}",
               f"output_path:={output / 'map'}"]
    expected = None
    if args.mode == "bag":
        if args.bag_path is None:
            raise ValueError("--mode bag requires --bag-path")
        bag = args.bag_path.expanduser().resolve()
        expectations_path = bag.parent / (bag.name + ".json")
        expected = json.loads(expectations_path.read_text(encoding="utf8"))
        command += [f"bag_path:={bag}", "bag_rate:=2"]
    summary["launch_command"] = command
    log = (output / "launch.log").open("w", encoding="utf8")
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    summary["launch_pid"] = process.pid
    node = None
    saved_hash = None
    try:
        rclpy.init(args=[])
        node = Observer()
        deadline = time.monotonic() + args.timeout
        wait_until(node, lambda: all(client.service_is_ready() for client in node.service_clients.values()),
                   deadline, "service discovery", process)
        summary["services"] = [call(node, "pause", True, deadline, process),
                               call(node, "pause", False, deadline, process),
                               call(node, "resume", True, deadline, process),
                               call(node, "resume", False, deadline, process)]
        if args.mode == "toy":
            wait_until(node, lambda: node.done, deadline, "toy completion", process)
            wait_until(node, lambda: node.status and node.status.get("processed_scans", 0) >= 118
                       and node.status.get("queued_scans") == 0,
                       deadline, "last toy scans to drain", process)
        else:
            wait_until(node, lambda: node.status and node.status.get("processed_scans", 0)
                       >= expected["minimum_processed_scans"] and node.pose is not None
                       and abs(node.pose.header.stamp.sec + node.pose.header.stamp.nanosec * 1e-9
                               - expected["last_scan_stamp"]) < 1e-6
                       and node.status.get("queued_scans") == 0,
                       deadline, "bag final scan and queue drain", process)
        summary["final_status"] = dict(node.status)
        if node.status.get("state") != "RUNNING":
            raise AssertionError(f"Expected a running filter after mapping: {node.status}")
        if node.invalid_pose_received:
            raise AssertionError("Received a pose or covariance containing NaN/inf")
        if not node.pose or node.poses_received < 100:
            raise AssertionError(f"Too few pose messages: {node.poses_received}")
        pose = node.pose.pose.pose.position
        if abs(pose.x) > 10 or abs(pose.y) > 10:
            raise AssertionError(f"Estimated final pose lies outside the toy world: {pose}")
        summary["final_pose"] = {"x": pose.x, "y": pose.y}
        summary["services"].append(call(node, "save_map", True, deadline, process))
        wait_until(node, lambda: node.map is not None and sum(value >= 0 for value in node.map.data) >= 500,
                   deadline, "populated map", process)
        summary["map_observed_cells"] = sum(value >= 0 for value in node.map.data)
        summary["map_export"] = validate_export(output)
        summary["poses_received"], summary["maps_received"] = node.poses_received, node.maps_received
        summary["cmd_vel_received"] = node.cmd_vel_received
        summary["stale_map_tf_received"] = node.stale_map_tf_received
        if node.cmd_vel_received:
            raise AssertionError("Bag playback unexpectedly republished cmd_vel")
        if node.stale_map_tf_received:
            raise AssertionError("Bag TF filter republished the recorded map->odom x=100 transform")
        saved_hash = hashlib.sha256((output / "map.pgm").read_bytes()).hexdigest()
        previous_maps = node.maps_received
        summary["services"].append(call(node, "reset", True, deadline, process))
        wait_until(node, lambda: node.status and node.status.get("state") == "WAITING"
                   and node.status.get("processed_scans") == 0 and node.maps_received > previous_maps
                   and node.map is not None and all(value == -1 for value in node.map.data),
                   deadline, "reset WAITING and empty map", process)
        summary["reset_verified"] = True
        summary["reset_status"] = dict(node.status)
    finally:
        if process.poll() is None:
            try:
                # ROS launch propagates SIGINT to its children. Signalling their
                # group too would deliver a second interrupt during node cleanup.
                process.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                summary["forced_cleanup"] = True
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=5)
        summary["launch_returncode"] = process.returncode
        log.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if saved_hash is not None:
            after = hashlib.sha256((output / "map.pgm").read_bytes()).hexdigest()
            if after != saved_hash:
                raise AssertionError("Shutdown overwrote the saved map after reset")
    launch_log = (output / "launch.log").read_text(encoding="utf8")
    for error_marker in ("process has died", "Traceback (most recent call last)"):
        if error_marker in launch_log:
            raise AssertionError(f"Child process error in launch.log: {error_marker}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("toy", "bag"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--bag-path", type=Path)
    parser.add_argument("--odom-source", choices=("topic", "tf"), default="topic")
    parser.add_argument("--timeout", type=float, default=45.0)
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    summary = {"mode": args.mode, "odom_source": args.odom_source,
               "passed": False, "forced_cleanup": False}
    started = time.monotonic()
    try:
        run(args, summary)
        summary["passed"] = True
    except Exception as exc:
        summary["error"] = f"{type(exc).__name__}: {exc}"
        summary["traceback"] = traceback.format_exc()
    summary["wall_seconds"] = time.monotonic() - started
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf8")
    print(json.dumps(summary, indent=2))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
