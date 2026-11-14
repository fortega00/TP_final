#!/usr/bin/env python3
"""Generate an actual rosbag2 transport fixture from the independent toy.

Run in a sourced ROS 2 Humble environment with this package installed.  The bag
contains deliberately stale map->odom TF and an unwanted cmd_vel topic: the
launcher must filter both while preserving odometry and the LiDAR mount.
"""

import argparse
import json
from pathlib import Path

import rosbag2_py
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from slam.ros.ros_node import pose_message
from slam.toy.world import ToyConfig, generate_samples


def set_stamp(stamp, value):
    stamp.sec, stamp.nanosec = divmod(int(round(value * 1_000_000_000)), 1_000_000_000)


def generate_bag(output: Path, seed: int):
    output = output.expanduser().resolve()
    if output.exists():
        raise ValueError(f"Fixture output already exists: {output}; choose a new path")
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(output), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                           output_serialization_format="cdr"))
    topics = {"/scan": "sensor_msgs/msg/LaserScan", "/odom": "nav_msgs/msg/Odometry",
              "/tf": "tf2_msgs/msg/TFMessage", "/tf_static": "tf2_msgs/msg/TFMessage",
              "/cmd_vel": "geometry_msgs/msg/Twist"}
    for name, typename in topics.items():
        writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=typename,
                                                   serialization_format="cdr"))

    def write(topic, message, timestamp):
        writer.write(topic, serialize_message(message), int(round(timestamp * 1_000_000_000)))

    mount = TransformStamped()
    mount.header.frame_id, mount.child_frame_id = "base_footprint", "laser_frame"
    mount.transform.rotation.w = 1.0
    # Static transforms are valid at every timestamp; retain their conventional stamp 0.
    write("/tf_static", TFMessage(transforms=[mount]), 10.0)
    samples = list(generate_samples(ToyConfig(), seed=seed))
    final_odom = None
    for index, sample in enumerate(samples):
        timestamp = 10.0 + sample.scan.stamp
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id = "odom", "base_footprint"
        set_stamp(odom.header.stamp, timestamp)
        odom.pose.pose = pose_message(sample.odometry)
        write("/odom", odom, timestamp)
        final_odom = odom
        dynamic = TransformStamped()
        dynamic.header = odom.header
        dynamic.child_frame_id = "base_footprint"
        dynamic.transform.translation.x, dynamic.transform.translation.y = sample.odometry.x, sample.odometry.y
        dynamic.transform.rotation = odom.pose.pose.orientation
        stale = TransformStamped()
        stale.header.frame_id, stale.child_frame_id = "map", "odom"
        set_stamp(stale.header.stamp, timestamp)
        stale.transform.translation.x, stale.transform.rotation.w = 100.0, 1.0
        write("/tf", TFMessage(transforms=[dynamic, stale]), timestamp)
        scan = LaserScan()
        scan.header.frame_id = "laser_frame"
        set_stamp(scan.header.stamp, timestamp)
        scan.angle_min, scan.angle_increment = sample.scan.angle_min, sample.scan.angle_increment
        scan.angle_max = scan.angle_min + (len(sample.scan.ranges) - 1) * scan.angle_increment
        scan.range_min, scan.range_max = sample.scan.range_min, sample.scan.range_max
        scan.scan_time, scan.time_increment = 0.25, 0.0
        scan.ranges = sample.scan.ranges.astype("float32").tolist()
        write("/scan", scan, timestamp)
        unwanted = Twist()
        unwanted.linear.x = 99.0
        write("/cmd_vel", unwanted, timestamp)
    final_stamp = 10.0 + samples[-1].scan.stamp
    set_stamp(final_odom.header.stamp, final_stamp + 0.05)
    write("/odom", final_odom, final_stamp + 0.05)
    # Destructor closes SQLite and writes metadata before another process reads it.
    del writer
    metadata = {"seed": seed, "scans": len(samples),
                "minimum_processed_scans": len(samples) - 1,
                "first_scan_stamp": 10.0 + samples[0].scan.stamp,
                "last_scan_stamp": final_stamp,
                "duration_seconds": samples[-1].scan.stamp - samples[0].scan.stamp + 0.05,
                "injected_stale_map_to_odom_x": 100.0,
                "injected_cmd_vel_linear_x": 99.0,
                "topics": topics}
    metadata_path = output.parent / (output.name + ".json")
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf8")
    return metadata_path, metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()
    path, metadata = generate_bag(args.output, args.seed)
    print(json.dumps({"bag": str(args.output.resolve()), "expectations": str(path), **metadata}, indent=2))


if __name__ == "__main__":
    main()
