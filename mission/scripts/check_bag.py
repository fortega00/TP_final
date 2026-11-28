#!/usr/bin/env python3
"""Pasa la visión de la misión por un bag de la cámara del robot, sin navegación.

Para cada imagen hace lo mismo que `mission_node`: detecta conos rojos, los
ubica con la profundidad y los confirma con el filtro de Bayes (decisiones 016
y 017). En lugar del mapa usa el frame de la odometría del bag (`/odom` y los
TF estáticos hasta la cámara), porque el bag no tiene localización. Un cono no
se mueve: si sus posiciones medidas desde distintos lugares quedan juntas, la
cadena detector, profundidad, calibración y montaje de la cámara está bien.

Salidas en `--output`:
- `summary.json`: imágenes con detección, detecciones ubicadas o descartadas
  (sin profundidad, tamaño fuera de rango) y, por cono candidato con al menos
  dos detecciones, posición, dispersión de las mediciones, tamaño medido,
  probabilidad final y máxima, e imagen en la que el filtro lo descartó.
- `detections.csv`: una fila por detección.
- `positions.png`: recorrido del robot y conos medidos, vistos desde arriba.
- `samples/`: imágenes de muestra con los recuadros (verde ubicado, amarillo
  descartado) y la distancia y el tamaño medidos.

Necesita ROS (rosbag2_py) para leer el bag, OpenCV y matplotlib. Con la imagen
del curso, desde la raíz del repo:

    docker run --rm -u "$(id -u):$(id -g)" -v "$PWD":/ws -v <bag>:/bag:ro -w /ws/mission \\
      yahboom_rosmaster:latest bash -c 'source /opt/ros/humble/setup.bash && \\
      PYTHONPATH=../slam:../nav:.:$PYTHONPATH python3 scripts/check_bag.py /bag --output artifacts/check_bag'
"""

import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

from mission.core.config import MissionConfig
from mission.core.cone_locator import ConeTracker, blob_depth, camera_position_yaw
from mission.core.detector import detect_cones
from mission.ros.conversions import depth_in_meters, intrinsics_from_k, match_size, transform_matrix

# Diferencia máxima entre una imagen de color y su profundidad (s). El limitador
# de la cámara las publica de a pares con el mismo instante o casi.
PAIR_TOLERANCE = 0.05


def stamp(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def image_array(msg) -> np.ndarray:
    """Imagen de ROS a arreglo de NumPy, sin cv_bridge: color en BGR, profundidad tal cual."""
    if msg.encoding in ("rgb8", "bgr8"):
        image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        image = image.reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else image.copy()
    dtype = {"32FC1": np.float32, "16UC1": np.uint16, "mono16": np.uint16}.get(msg.encoding)
    if dtype is None:
        raise ValueError(f"Unsupported image encoding {msg.encoding!r}")
    size = np.dtype(dtype).itemsize
    return np.frombuffer(msg.data, dtype).reshape(msg.height, msg.step // size)[:, :msg.width]


def read_bag(path: Path, color_topic: str, depth_topic: str, info_topic: str):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(path), storage_id="sqlite3"), ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for topic in (color_topic, depth_topic, info_topic, "/odom", "/tf_static"):
        if topic not in types:
            raise SystemExit(f"Topic {topic} is not in the bag; topics: {sorted(types)}")
    wanted = {color_topic, depth_topic, info_topic, "/odom", "/tf_static"}
    colors, depths, odometry, static, info = [], [], [], {}, None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in wanted or (topic == info_topic and info is not None):
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == color_topic:
            colors.append((stamp(msg.header), msg.header.frame_id, image_array(msg)))
        elif topic == depth_topic:
            depths.append((stamp(msg.header), depth_in_meters(image_array(msg), msg.encoding)))
        elif topic == info_topic:
            info = msg
        elif topic == "/odom":
            p = msg.pose.pose
            odometry.append((stamp(msg.header), msg.child_frame_id, p.position.x, p.position.y,
                             yaw_of(p.orientation)))
        else:
            for tr in msg.transforms:
                t, q = tr.transform.translation, tr.transform.rotation
                static[tr.child_frame_id] = (tr.header.frame_id,
                                             transform_matrix((t.x, t.y, t.z), (q.x, q.y, q.z, q.w)))
    return colors, depths, odometry, static, info


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def static_chain(static: dict, base_frame: str, frame: str) -> np.ndarray:
    """base_T_frame componiendo los TF estáticos del bag."""
    matrix = np.eye(4)
    while frame != base_frame:
        if frame not in static:
            raise SystemExit(f"No static transform chain from {base_frame}; static frames: {sorted(static)}")
        parent, parent_T_frame = static[frame]
        matrix = parent_T_frame @ matrix
        frame = parent
    return matrix


def odometry_at(odometry, t: float):
    """odom_T_base interpolada al instante t, o None si t queda fuera de la odometría."""
    times = np.array([o[0] for o in odometry])
    if not times[0] <= t <= times[-1]:
        return None
    x = np.interp(t, times, [o[2] for o in odometry])
    y = np.interp(t, times, [o[3] for o in odometry])
    yaw = np.interp(t, times, np.unwrap([o[4] for o in odometry]))
    matrix = np.eye(4)
    matrix[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    matrix[:2, 3] = x, y
    return matrix


def nearest_depth(depths, t: float):
    best = min(depths, key=lambda d: abs(d[0] - t), default=None)
    return best[1] if best is not None and abs(best[0] - t) <= PAIR_TOLERANCE else None


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag", type=Path, help="carpeta del bag")
    parser.add_argument("--config", default="config/real.yaml", help="YAML de mission")
    parser.add_argument("--output", type=Path, default=Path("check_bag"))
    parser.add_argument("--color-topic", default="/mission/cam/color/image_raw")
    parser.add_argument("--depth-topic", default="/mission/cam/depth/image_raw")
    parser.add_argument("--info-topic", default="/mission/cam/color/camera_info")
    parser.add_argument("--samples", type=int, default=16, help="imágenes de muestra a guardar")
    args = parser.parse_args()

    config = MissionConfig.load_yaml(args.config)
    colors, depths, odometry, static, info = read_bag(args.bag, args.color_topic, args.depth_topic,
                                                      args.info_topic)
    if not colors or not odometry or info is None:
        raise SystemExit("The bag needs color images, camera_info and /odom")
    intrinsics = intrinsics_from_k(info.k, info.width)
    base_frame = odometry[0][1]
    base_T_camera = static_chain(static, base_frame, colors[0][1])
    tracker = ConeTracker(config.locator)
    # Todos los candidatos que hubo, también los que el filtro descartó en el camino:
    # id -> {"track", "max_probability", "dropped_at"} (imagen en la que se descartó).
    history = {}
    rows, no_pair, no_odometry = [], 0, 0
    path = []
    for index, (t, _, image) in enumerate(colors):
        depth = nearest_depth(depths, t)
        odom_T_base = odometry_at(odometry, t)
        if depth is None:
            no_pair += 1
            continue
        if odom_T_base is None:
            no_odometry += 1
            continue
        odom_T_camera = odom_T_base @ base_T_camera
        path.append(odom_T_base[:2, 3])
        meters = match_size(depth, image.shape)
        points = []
        for detection in detect_cones(image, config.detector):
            row = measure_row(index, t - colors[0][0], detection, meters, intrinsics, odom_T_camera,
                              config.locator)
            rows.append(row)
            if row["status"] == "located":
                points.append((row["x"], row["y"]))
        camera_xy, camera_yaw = camera_position_yaw(odom_T_camera)
        tracker.update(points, camera_xy, camera_yaw, intrinsics.horizontal_fov)
        alive = {id(track) for track in tracker.tracks}
        for track in tracker.tracks:
            entry = history.setdefault(id(track), {"track": track, "max_probability": 0.0, "dropped_at": None})
            entry["max_probability"] = max(entry["max_probability"], track.probability)
        for key, entry in history.items():
            if key not in alive and entry["dropped_at"] is None:
                entry["dropped_at"] = index

    args.output.mkdir(parents=True, exist_ok=True)
    summary = summarize(colors, rows, history, config, intrinsics, base_T_camera, no_pair, no_odometry)
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
    with (args.output / "detections.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]) if rows else ["frame"])
        writer.writeheader()
        writer.writerows(rows)
    plot_positions(args.output / "positions.png", np.array(path), rows, summary["tracks"])
    save_samples(args.output / "samples", colors, rows, args.samples)
    print(json.dumps({k: v for k, v in summary.items() if k != "tracks"}, indent=2, ensure_ascii=False))
    for track in summary["tracks"]:
        print(track)


def measure_row(index, t, detection, depth, intrinsics, odom_T_camera, config) -> dict:
    """Lo mismo que `cone_locator.measure`, guardando por qué se descarta una detección."""
    x, y, w, h = detection.box
    row = {"frame": index, "t": round(t, 3), "box_x": x, "box_y": y, "box_w": w, "box_h": h,
           "area": detection.area, "distance": "", "width": "", "height": "", "x": "", "y": "",
           "status": "no_depth"}
    distance = blob_depth(depth, detection.box, config)
    if distance is None:
        return row
    width, height = w * distance / intrinsics.fx, h * distance / intrinsics.fy
    row.update(distance=round(distance, 3), width=round(width, 3), height=round(height, 3))
    if not (config.width_min <= width <= config.width_max and config.height_min <= height <= config.height_max):
        row["status"] = "size"
        return row
    u, v = detection.center
    point = np.array([(u - intrinsics.cx) * distance / intrinsics.fx,
                      (v - intrinsics.cy) * distance / intrinsics.fy, distance, 1.0])
    px, py = (odom_T_camera @ point)[:2]
    row.update(x=round(float(px), 3), y=round(float(py), 3), status="located")
    return row


def summarize(colors, rows, history, config, intrinsics, base_T_camera, no_pair, no_odometry) -> dict:
    located = [r for r in rows if r["status"] == "located"]
    tracks = []
    for entry in sorted(history.values(), key=lambda e: -e["track"].hits):
        track = entry["track"]
        if track.hits < 2:
            continue
        near = [r for r in located if math.hypot(r["x"] - track.x, r["y"] - track.y)
                < config.locator.associate_distance]
        if not near:
            continue
        xy = np.array([(r["x"], r["y"]) for r in near])
        spread = np.hypot(*(xy - xy.mean(axis=0)).T)
        tracks.append({
            "x": round(track.x, 3), "y": round(track.y, 3), "hits": track.hits,
            "probability": round(track.probability, 3),
            "max_probability": round(entry["max_probability"], 3),
            "dropped_at_image": entry["dropped_at"],
            "measurements": len(near),
            "spread_median_m": round(float(np.median(spread)), 3),
            "spread_p95_m": round(float(np.percentile(spread, 95)), 3),
            "distance_range_m": [round(min(r["distance"] for r in near), 2),
                                 round(max(r["distance"] for r in near), 2)],
            "width_median_m": round(float(np.median([r["width"] for r in near])), 3),
            "height_median_m": round(float(np.median([r["height"] for r in near])), 3),
        })
    frames_with_detection = len({r["frame"] for r in rows})
    return {
        "images": len(colors),
        "images_without_depth_pair": no_pair,
        "images_without_odometry": no_odometry,
        "images_with_detection": frames_with_detection,
        "detections": len(rows),
        "detections_by_status": {s: sum(r["status"] == s for r in rows) for s in ("located", "no_depth", "size")},
        "camera": {"width": intrinsics.width, "fx": round(intrinsics.fx, 2),
                   "horizontal_fov_rad": round(intrinsics.horizontal_fov, 3),
                   "base_T_camera_xyz_m": [round(float(v), 3) for v in base_T_camera[:3, 3]]},
        "locator": {k: getattr(config.locator, k) for k in
                    ("min_depth", "max_depth", "width_min", "width_max", "height_min", "height_max",
                     "associate_distance", "confirm_probability")},
        "tracks": tracks,
    }


def plot_positions(path_png: Path, path: np.ndarray, rows, tracks):
    fig, ax = plt.subplots(figsize=(7, 7))
    if len(path):
        ax.plot(path[:, 0], path[:, 1], "-", color="0.6", lw=1, label="robot (odometría)")
    located = [r for r in rows if r["status"] == "located"]
    if located:
        ax.scatter([r["x"] for r in located], [r["y"] for r in located], s=8, c="tab:red", alpha=0.5,
                   label="cono medido en una imagen")
    for track in tracks:
        confirmed = track["max_probability"] >= 0.97
        ax.scatter(track["x"], track["y"], s=120, marker="x", c="tab:green" if confirmed else "black")
        ax.annotate(f"{track['measurements']} med.\n{track['spread_median_m'] * 100:.0f} cm",
                    (track["x"], track["y"]), xytext=(6, 6), textcoords="offset points", fontsize=8)
    ax.set_aspect("equal")
    ax.set_xlabel("x odom (m)")
    ax.set_ylabel("y odom (m)")
    ax.set_title("Conos medidos (x verde: candidato confirmado)")
    ax.legend(loc="best", fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path_png, dpi=120)
    plt.close(fig)


def save_samples(folder: Path, colors, rows, count: int):
    folder.mkdir(parents=True, exist_ok=True)
    frames = sorted({r["frame"] for r in rows})
    if not frames or count <= 0:
        return
    chosen = [frames[int(i)] for i in np.linspace(0, len(frames) - 1, min(count, len(frames)))]
    for frame in chosen:
        image = colors[frame][2].copy()
        for r in (r for r in rows if r["frame"] == frame):
            x, y, w, h = r["box_x"], r["box_y"], r["box_w"], r["box_h"]
            color = (0, 255, 0) if r["status"] == "located" else (0, 255, 255)
            cv2.rectangle(image, (x, y), (x + w, y + h), color, 1)
            label = r["status"] if r["distance"] == "" else f"{r['distance']:.2f}m {r['width']:.2f}x{r['height']:.2f}"
            cv2.putText(image, label, (x, max(10, y - 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
        cv2.imwrite(str(folder / f"frame_{frame:04d}.png"), image)


if __name__ == "__main__":
    main()
