#!/usr/bin/env python3
"""Convierte un bag de ROS 2 en el .npz que leen los scripts de replay.

Necesita un entorno con ROS (rosbag2_py). Ejemplo:

    python scripts/bag_to_npz.py bag_aula bag_aula.npz --base-frame base_footprint

El .npz tiene `metadata` (stamp, angle_min, angle_increment, range_min,
range_max por scan), `ranges`, `odometry` (x, y, yaw interpolados al instante de
cada scan) y `laser` (dónde está montado el lidar respecto de la base). Con
`--truth-topic` agrega `truth`, la pose real del simulador en cada scan.

Los scans sin odometría antes y después se descartan: no se extrapola.
"""
import argparse
import math
from pathlib import Path
import numpy as np
import yaml

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from rosidl_runtime_py.utilities import get_message

from slam.core.geometry import Pose2D
from slam.core.odometry import OdometryBuffer


def stamp(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def odometry_pose(msg) -> Pose2D:
    p = msg.pose.pose
    return Pose2D(p.position.x, p.position.y, yaw_of(p.orientation))


def static_mount(transforms: dict, base_frame: str, laser_frame: str) -> Pose2D:
    """Compone los TF estáticos desde la base hasta el lidar."""
    chain, frame = [], laser_frame
    while frame != base_frame:
        if frame not in transforms:
            raise ValueError(f"No static transform chain from {base_frame} to {laser_frame}; "
                             f"static frames in the bag: {sorted(transforms)}")
        parent, pose = transforms[frame]
        chain.append(pose)
        frame = parent
    mount = Pose2D()
    for pose in reversed(chain):
        mount = mount.compose(pose)
    return mount


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("bag", type=Path, help="carpeta del bag")
    p.add_argument("output", type=Path, help=".npz de salida")
    p.add_argument("--scan-topic", default="/scan")
    p.add_argument("--odom-topic", default="/odom")
    p.add_argument("--truth-topic", help="tópico Odometry con la pose real (solo simulación)")
    p.add_argument("--base-frame", required=True,
                   help="frame de la base del robot, el child_frame_id de la odometría")
    args = p.parse_args()

    info = yaml.safe_load((args.bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(args.bag), storage_id=info["storage_identifier"]),
                ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    wanted = [args.scan_topic, args.odom_topic, "/tf_static"]
    if args.truth_topic:
        wanted.append(args.truth_topic)
    missing = [t for t in wanted if t not in types]
    if missing:
        raise SystemExit(f"Topics missing from the bag: {missing}. It has: {sorted(types)}")
    reader.set_filter(StorageFilter(topics=wanted))

    scans, statics = [], {}
    odometry, truth = OdometryBuffer(history_seconds=1e9), OdometryBuffer(history_seconds=1e9)
    odometry_children = set()
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        msg = deserialize_message(raw, types[topic])
        if topic == args.scan_topic:
            scans.append(msg)
        elif topic == args.odom_topic:
            odometry.append(stamp(msg.header), odometry_pose(msg))
            odometry_children.add(msg.child_frame_id)
        elif topic == args.truth_topic:
            truth.append(stamp(msg.header), odometry_pose(msg))
        else:
            for tf in msg.transforms:
                t = tf.transform
                statics[tf.child_frame_id] = (tf.header.frame_id,
                                              Pose2D(t.translation.x, t.translation.y, yaw_of(t.rotation)))
    if not scans:
        raise SystemExit("The bag has no scans")
    # La odometría tiene que describir la base. Si su child_frame_id es otro frame
    # (por ejemplo calc_base en el simulador), las poses no son las de la base.
    if odometry_children != {args.base_frame}:
        raise SystemExit(f"Odometry child_frame_id is {sorted(odometry_children)}, "
                         f"not --base-frame {args.base_frame}")
    laser_frame = scans[0].header.frame_id
    mount = static_mount(statics, args.base_frame, laser_frame)

    first = scans[0]
    rows, ranges, odom_rows, truth_rows, dropped = [], [], [], [], 0
    for scan in scans:
        if (scan.angle_min, scan.angle_increment, len(scan.ranges)) != (
                first.angle_min, first.angle_increment, len(first.ranges)):
            raise SystemExit("Scan geometry changes inside the bag")
        t = stamp(scan.header)
        pose = odometry.at(t)
        real = truth.at(t) if args.truth_topic else None
        if pose is None or (args.truth_topic and real is None):
            dropped += 1
            continue
        rows.append([t, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max])
        ranges.append(scan.ranges)
        odom_rows.append([pose.x, pose.y, pose.yaw])
        if real is not None:
            truth_rows.append([real.x, real.y, real.yaw])
    if not rows:
        raise SystemExit("No scan has odometry before and after it")
    arrays = dict(metadata=np.array(rows), ranges=np.array(ranges, dtype=np.float32),
                  odometry=np.array(odom_rows), laser=np.array([mount.x, mount.y, mount.yaw]))
    if args.truth_topic:
        arrays["truth"] = np.array(truth_rows)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output, **arrays)
    xy = arrays["odometry"][:, :2]
    print(f"{args.output}: {len(rows)} scans of {len(first.ranges)} rays, {dropped} dropped without odometry, "
          f"{rows[-1][0] - rows[0][0]:.0f} s, odometry path "
          f"{np.linalg.norm(np.diff(xy, axis=0), axis=1).sum():.1f} m, "
          f"laser frame {laser_frame} at ({mount.x:.3f}, {mount.y:.3f}) m, "
          f"yaw {math.degrees(mount.yaw):.0f} deg from {args.base_frame}")


if __name__ == "__main__":
    main()
