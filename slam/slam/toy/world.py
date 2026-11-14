"""Deterministic planar office benchmark, independent of ROS and of the SLAM map.

The simulator ray-casts analytic segments rather than the estimated grid.  Ground
truth is used only by the evaluator; the filter receives noisy odometry and scans.
This is an algorithm smoke test, not a substitute for Gazebo or real-robot trials.
"""
from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
import csv
import json
import math
from pathlib import Path
import time
from typing import Iterator

import numpy as np
import yaml

from ..core.geometry import Pose2D
from ..core.types import Scan


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class ToyConfig:
    translation_step: float = 0.28
    rotation_step: float = math.pi / 12
    beams: int = 180
    range_max: float = 8.0
    range_min: float = 0.12
    sensor_noise: float = 0.015
    dropout: float = 0.005
    odom_translation_noise: float = 0.004
    odom_rotation_noise: float = 0.003
    odom_scale: float = 1.002
    sample_period: float = 0.25
    robot_radius: float = 0.22
    safety_margin: float = 0.05

    def __post_init__(self) -> None:
        if self.translation_step <= 0 or self.rotation_step <= 0:
            raise ValueError("Toy step sizes must be positive")
        if self.beams < 8 or self.range_max <= self.range_min or self.range_min < 0:
            raise ValueError("Invalid toy LiDAR configuration")
        if not 0 <= self.dropout < 1 or self.sample_period <= 0:
            raise ValueError("Invalid dropout/sample period")
        for name in ("sensor_noise", "odom_translation_noise", "odom_rotation_noise",
                     "robot_radius", "safety_margin"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if not math.isfinite(self.odom_scale) or self.odom_scale <= 0:
            raise ValueError("odom_scale must be positive")


@dataclass(frozen=True)
class OfficeWorld:
    """9 x 7 metre room, two offices connected by a two metre doorway."""

    segments: np.ndarray
    bounds: tuple[float, float, float, float] = (-2.0, 7.0, -3.5, 3.5)

    @classmethod
    def standard(cls) -> "OfficeWorld":
        # Zero-width analytic surfaces. Rasterization marks their containing cells.
        return cls(np.array([
            [-2., -3.5, 7., -3.5], [7., -3.5, 7., 3.5],
            [7., 3.5, -2., 3.5], [-2., 3.5, -2., -3.5],
            [2.5, -3.5, 2.5, -1.], [2.5, 1., 2.5, 3.5],
            # Asymmetric short wall helps disambiguate the two sides.
            [5.8, 1., 7., 1.],
        ], dtype=float))

    def raycast(self, pose: Pose2D, bearings: np.ndarray, range_max: float) -> np.ndarray:
        angles = np.asarray(bearings) + pose.yaw
        direction = np.column_stack((np.cos(angles), np.sin(angles)))
        start = self.segments[:, :2]
        vector = self.segments[:, 2:] - start
        offset = start - np.array([pose.x, pose.y])
        denom = (direction[:, None, 0] * vector[None, :, 1]
                 - direction[:, None, 1] * vector[None, :, 0])
        cross_offset_vector = offset[:, 0] * vector[:, 1] - offset[:, 1] * vector[:, 0]
        cross_offset_direction = (offset[None, :, 0] * direction[:, None, 1]
                                  - offset[None, :, 1] * direction[:, None, 0])
        with np.errstate(divide="ignore", invalid="ignore"):
            distance = cross_offset_vector[None, :] / denom
            along_wall = cross_offset_direction / denom
        valid = ((np.abs(denom) > 1e-12) & (distance >= 0)
                 & (along_wall >= 0) & (along_wall <= 1))
        nearest = np.min(np.where(valid, distance, np.inf), axis=1)
        return np.where(nearest < range_max, nearest, np.inf)

    def trajectory(self, config: ToyConfig) -> list[Pose2D]:
        # Every translation is collision free; turns are sampled in place.  Start
        # and end use the same full pose, making loop error directly measurable.
        waypoints = [(0., 0.), (5., 0.), (5., 2.), (4., 2.), (4., 0.),
                     (1., 0.), (1., -2.), (0., -2.), (0., 0.)]
        poses = [Pose2D(0., 0., 0.)]
        for target_x, target_y in waypoints[1:]:
            last = poses[-1]
            dx, dy = target_x - last.x, target_y - last.y
            yaw = math.atan2(dy, dx)
            turn = _wrap(yaw - last.yaw)
            nturn = max(1, math.ceil(abs(turn) / config.rotation_step))
            for index in range(1, nturn + 1):
                poses.append(Pose2D(last.x, last.y, _wrap(last.yaw + turn * index / nturn)))
            nsteps = max(1, math.ceil(math.hypot(dx, dy) / config.translation_step))
            for index in range(1, nsteps + 1):
                poses.append(Pose2D(last.x + dx * index / nsteps,
                                    last.y + dy * index / nsteps, yaw))
        last = poses[-1]
        turn = _wrap(-last.yaw)
        for index in range(1, max(1, math.ceil(abs(turn) / config.rotation_step)) + 1):
            steps = max(1, math.ceil(abs(turn) / config.rotation_step))
            poses.append(Pose2D(last.x, last.y, _wrap(last.yaw + turn * index / steps)))
        return poses

    def rasterize(self, map_config) -> np.ndarray:
        """Occupancy in ROS orientation [row y, column x], independently generated."""
        resolution = map_config.resolution
        width = math.ceil(map_config.width / resolution)
        height = math.ceil(map_config.height / resolution)
        result = np.zeros((height, width), dtype=np.int8)
        x = map_config.origin_x + (np.arange(width) + .5) * resolution
        y = map_config.origin_y + (np.arange(height) + .5) * resolution
        min_x, max_x, min_y, max_y = self.bounds
        interior = ((x[None, :] >= min_x) & (x[None, :] <= max_x)
                    & (y[:, None] >= min_y) & (y[:, None] <= max_y))
        result[~interior] = -1
        for x1, y1, x2, y2 in self.segments:
            count = math.ceil(math.hypot(x2-x1, y2-y1) / (resolution / 4)) + 1
            cols = np.floor((np.linspace(x1, x2, count) - map_config.origin_x) / resolution).astype(int)
            rows = np.floor((np.linspace(y1, y2, count) - map_config.origin_y) / resolution).astype(int)
            inside = (cols >= 0) & (rows >= 0) & (cols < width) & (rows < height)
            result[rows[inside], cols[inside]] = 100
        return result


@dataclass(frozen=True)
class ToySample:
    truth: Pose2D
    odometry: Pose2D
    scan: Scan


def generate_samples(config: ToyConfig = ToyConfig(), seed: int = 7,
                     world: OfficeWorld | None = None) -> Iterator[ToySample]:
    """Separate RNG streams keep odometry identical across sensor-noise ablations."""
    world = world or OfficeWorld.standard()
    sensor_rng, odom_rng = [np.random.default_rng(s) for s in np.random.SeedSequence(seed).spawn(2)]
    poses = world.trajectory(config)
    odom = Pose2D(0., 0., 0.)
    bearings = np.linspace(-math.pi, math.pi, config.beams, endpoint=False)
    for index, truth in enumerate(poses):
        if index:
            previous = poses[index - 1]
            dx, dy = truth.x - previous.x, truth.y - previous.y
            c, s = math.cos(previous.yaw), math.sin(previous.yaw)
            local_x, local_y = c * dx + s * dy, -s * dx + c * dy
            rotation = _wrap(truth.yaw - previous.yaw)
            distance = math.hypot(dx, dy)
            local_x = local_x * config.odom_scale + odom_rng.normal(0, config.odom_translation_noise * math.sqrt(distance))
            local_y += odom_rng.normal(0, config.odom_translation_noise * math.sqrt(distance))
            rotation += odom_rng.normal(0, config.odom_rotation_noise * math.sqrt(distance + abs(rotation)))
            c, s = math.cos(odom.yaw), math.sin(odom.yaw)
            odom = Pose2D(odom.x + c * local_x - s * local_y,
                          odom.y + s * local_x + c * local_y, _wrap(odom.yaw + rotation))
        ranges = world.raycast(truth, bearings, config.range_max)
        finite = np.isfinite(ranges)
        ranges[finite] += sensor_rng.normal(0, config.sensor_noise, finite.sum())
        ranges[(ranges < config.range_min) | (ranges >= config.range_max)] = np.inf
        ranges[sensor_rng.random(config.beams) < config.dropout] = np.nan
        yield ToySample(truth, odom, Scan(
            ranges=ranges, angle_min=-math.pi, angle_increment=2*math.pi/config.beams,
            range_min=config.range_min, range_max=config.range_max,
            stamp=(index + 1) * config.sample_period,
        ))


def _dilate(mask: np.ndarray, radius_cells: int, circular: bool = False) -> np.ndarray:
    result = np.zeros_like(mask, dtype=bool)
    height, width = mask.shape
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if circular and dx*dx + dy*dy > radius_cells*radius_cells:
                continue
            src_y = slice(max(0, -dy), min(height, height-dy))
            src_x = slice(max(0, -dx), min(width, width-dx))
            dst_y = slice(max(0, dy), min(height, height+dy))
            dst_x = slice(max(0, dx), min(width, width+dx))
            result[dst_y, dst_x] |= mask[src_y, src_x]
    return result


def _nearest_distances(points: np.ndarray, targets: np.ndarray) -> np.ndarray:
    if len(points) == 0 or len(targets) == 0:
        return np.array([], dtype=float)
    # Bounded batches avoid allocating the full point-to-point distance matrix.
    return np.concatenate([np.sqrt(np.min(np.sum((points[i:i+256, None, :]
                            - targets[None, :, :]) ** 2, axis=2), axis=1))
                           for i in range(0, len(points), 256)])


def evaluate_map(occupancy: np.ndarray, ground_truth: np.ndarray, map_config,
                 route: list[Pose2D], robot_radius: float = .22,
                 safety_margin: float = .05) -> dict:
    occupied = occupancy >= 65
    truth_occupied = ground_truth >= 65
    # One-cell Chebyshev tolerance is explicitly resolution-dependent.
    matched_est = occupied & _dilate(truth_occupied, 1)
    matched_truth = truth_occupied & _dilate(occupied, 1)
    est_count, truth_count = int(occupied.sum()), int(truth_occupied.sum())
    distances = _nearest_distances(np.argwhere(occupied), np.argwhere(truth_occupied)) * map_config.resolution
    truth_free = ground_truth == 0
    known = occupancy >= 0
    free = (occupancy >= 0) & (occupancy <= 25)
    blocked = ~free  # Unknown and uncertain cells are obstacles for this check.
    radius_cells = math.ceil((robot_radius + safety_margin) / map_config.resolution)
    blocked = _dilate(blocked, radius_cells, circular=True)
    blocked[:radius_cells, :] = blocked[-radius_cells:, :] = True
    blocked[:, :radius_cells] = blocked[:, -radius_cells:] = True
    height, width = occupancy.shape

    def cell(pose):
        return (math.floor((pose.y-map_config.origin_y)/map_config.resolution),
                math.floor((pose.x-map_config.origin_x)/map_config.resolution))

    reachable = np.zeros_like(blocked)
    start = cell(route[0])
    if 0 <= start[0] < height and 0 <= start[1] < width and not blocked[start]:
        queue = deque([start])
        reachable[start] = True
        while queue:
            row, col = queue.popleft()
            for y, x in ((row-1, col), (row+1, col), (row, col-1), (row, col+1)):
                if 0 <= y < height and 0 <= x < width and not blocked[y, x] and not reachable[y, x]:
                    reachable[y, x] = True
                    queue.append((y, x))
    # Check all translated route samples, so doors and narrow paths matter.
    route_cells = sorted(set(cell(pose) for pose in route))
    goals = [reachable[y, x] if 0 <= y < height and 0 <= x < width else False for y, x in route_cells]
    precision = int(matched_est.sum()) / max(1, est_count)
    recall = int(matched_truth.sum()) / max(1, truth_count)
    intersection = int((occupied & truth_occupied).sum())
    union = int((occupied | truth_occupied).sum())
    return {
        "occupied_iou_exact": intersection / max(1, union),
        "occupied_precision_1cell": precision,
        "occupied_recall_1cell": recall,
        "occupied_f1_1cell": 2*precision*recall/max(1e-12, precision+recall),
        "occupied_cells": est_count,
        "ground_truth_occupied_cells": truth_count,
        "wall_distance_mean_m": float(distances.mean()) if len(distances) else None,
        "wall_distance_p95_m": float(np.percentile(distances, 95)) if len(distances) else None,
        "known_fraction_inside_room": float((known & truth_free).sum() / max(1, truth_free.sum())),
        "confident_free_fraction_inside_room": float((free & truth_free).sum() / max(1, truth_free.sum())),
        "footprint_radius_m": robot_radius,
        "footprint_safety_margin_m": safety_margin,
        "route_cells_reachable_fraction": float(np.mean(goals)) if goals else 0.,
        "reachable_free_cells": int(reachable.sum()),
        "metric_notes": "Saved PGM/YAML trinary map; unknown/uncertain blocked. Exact occupied IoU includes all GT walls and all predicted obstacles. Precision/recall use 1-cell Chebyshev wall tolerance; coverage denominator is all GT free interior; 4-neighbour connectivity after circular inflation.",
    }


def _write_pgm(path: Path, occupancy: np.ndarray) -> None:
    # 128 falls between the YAML thresholds; 205 would be read as free at .25.
    pixels = np.full(occupancy.shape, 128, dtype=np.uint8)
    pixels[(occupancy >= 0) & (occupancy <= 25)] = 254
    pixels[occupancy >= 65] = 0
    with path.open("wb") as stream:
        stream.write(f"P5\n{pixels.shape[1]} {pixels.shape[0]}\n255\n".encode("ascii"))
        stream.write(np.flipud(pixels).tobytes())


def load_trinary_map(metadata_path: str | Path) -> tuple[np.ndarray, dict]:
    """Read the actual PGM/YAML artifact with map-server threshold semantics.

    This small P5 reader intentionally supports our 8-bit export format. Evaluating
    the saved artifact catches origin/row-order/unknown errors hidden by in-memory
    probability arrays. Values strictly between thresholds remain unknown.
    """
    metadata_path = Path(metadata_path)
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    with (metadata_path.parent / metadata["image"]).open("rb") as stream:
        def header_line():
            while True:
                line = stream.readline()
                if not line:
                    raise ValueError("Truncated PGM header")
                line = line.split(b"#", 1)[0].strip()
                if line:
                    return line

        if header_line() != b"P5":
            raise ValueError("Only binary P5 PGM is supported by the toy evaluator")
        width, height = map(int, header_line().split())
        if header_line() != b"255" or width <= 0 or height <= 0:
            raise ValueError("Expected an 8-bit nonempty PGM")
        pixels = np.frombuffer(stream.read(), dtype=np.uint8)
        if pixels.size != width * height:
            raise ValueError("Unexpected PGM payload size")
    pixels = np.flipud(pixels.reshape(height, width))
    probabilities = pixels / 255.0 if metadata.get("negate", 0) else 1 - pixels / 255.0
    occupancy = np.full(pixels.shape, -1, dtype=np.int8)
    occupancy[probabilities < metadata["free_thresh"]] = 0
    occupancy[probabilities > metadata["occupied_thresh"]] = 100
    return occupancy, metadata


def _write_overview(path: Path, occupancy: np.ndarray, map_config, trajectory: list[dict]) -> None:
    # Display cells as runs to keep the vector artifact compact and inspectable.
    height, width = occupancy.shape
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="960" height="960" viewBox="0 0 {width} {height}">',
             f'<title>Toy FastSLAM: map, ground truth (green), estimated (red), odometry (blue)</title>',
             f'<rect width="{width}" height="{height}" fill="#ddd"/>']
    for row in range(height):
        for selected, color in (((occupancy[row] >= 0) & (occupancy[row] <= 25), "#fff"),
                                (occupancy[row] >= 65, "#222")):
            edges = np.diff(np.r_[False, selected, False].astype(int))
            for start, end in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
                parts.append(f'<rect x="{start}" y="{height-row-1}" width="{end-start}" height="1" fill="{color}"/>')
    for prefix, color in (("truth", "#13944c"), ("odom", "#2266dd"), ("estimate", "#dc2525")):
        points = " ".join(f'{(p[prefix+"_x"]-map_config.origin_x)/map_config.resolution:.2f},'
                          f'{height-(p[prefix+"_y"]-map_config.origin_y)/map_config.resolution:.2f}'
                          for p in trajectory)
        parts.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width=".75"/>')
    parts.append("</svg>\n")
    path.write_text("\n".join(parts), encoding="utf-8")


def run_benchmark(slam_config, output: str | Path, toy_config: ToyConfig = ToyConfig(),
                  seed: int = 7, max_scans: int | None = None) -> dict:
    from ..core.filter import FastSLAM
    from ..core.io import save_map

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    world = OfficeWorld.standard()
    x_min, x_max, y_min, y_max = world.bounds
    if not (slam_config.map.origin_x <= x_min
            and slam_config.map.origin_y <= y_min
            and slam_config.map.origin_x + slam_config.map.width > x_max
            and slam_config.map.origin_y + slam_config.map.height > y_max):
        raise ValueError("Toy map must contain the complete office x=[-2,7], y=[-3.5,3.5]")
    slam = FastSLAM(slam_config)
    trajectory = []
    samples = []
    began = time.perf_counter()
    for index, sample in enumerate(generate_samples(toy_config, seed, world)):
        if max_scans is not None and index >= max_scans:
            break
        processed = slam.process(sample.scan, sample.odometry)
        estimate = slam.best_particle.pose
        row = {"stamp": sample.scan.stamp, "processed": int(processed)}
        for prefix, pose in (("truth", sample.truth), ("odom", sample.odometry), ("estimate", estimate)):
            row.update({prefix+"_x": pose.x, prefix+"_y": pose.y, prefix+"_yaw": pose.yaw})
        trajectory.append(row)
        samples.append(sample)
    runtime = time.perf_counter() - began
    if not trajectory:
        raise ValueError("Benchmark needs at least one scan")
    grid = slam.best_particle.grid
    save_map(grid, output / "map", occupied_thresh=slam_config.sensor.occupied_threshold)
    occupancy, _ = load_trinary_map(output / "map.yaml")
    truth = world.rasterize(slam_config.map)
    metrics = evaluate_map(occupancy, truth, slam_config.map, [sample.truth for sample in samples],
                           toy_config.robot_radius, toy_config.safety_margin)
    estimates = np.array([[p["estimate_x"], p["estimate_y"], p["estimate_yaw"]] for p in trajectory])
    truths = np.array([[p["truth_x"], p["truth_y"], p["truth_yaw"]] for p in trajectory])
    odometry = np.array([[p["odom_x"], p["odom_y"], p["odom_yaw"]] for p in trajectory])
    errors, odom_errors = estimates - truths, odometry - truths
    yaw_errors = (errors[:, 2] + math.pi) % (2*math.pi) - math.pi
    metrics.update({
        "seed": seed, "scans": len(samples), "processed_scans": sum(p["processed"] for p in trajectory),
        "loop_completed": len(samples) == len(world.trajectory(toy_config)),
        "resolution_m": slam_config.map.resolution, "particles": slam_config.filter.particles,
        "position_rmse_m": float(np.sqrt(np.mean(np.sum(errors[:, :2]**2, axis=1)))),
        "yaw_rmse_rad": float(np.sqrt(np.mean(yaw_errors**2))),
        "odometry_position_rmse_m": float(np.sqrt(np.mean(np.sum(odom_errors[:, :2]**2, axis=1)))),
        "final_position_error_m": float(np.linalg.norm(errors[-1, :2])),
        "final_yaw_error_rad": float(abs(yaw_errors[-1])),
        "runtime_seconds": runtime, "seconds_per_input_scan": runtime/len(samples),
        "particle_map_bytes": int(sum(p.grid.log_odds.nbytes + p.grid.observed.nbytes for p in slam.particles)),
        "toy_config": asdict(toy_config), "slam_config": slam_config.to_dict(),
        "diagnostics": slam.diagnostics,
        "validation_scope": "Synthetic planar office only; not Gazebo, rosbag or physical RosMaster validation.",
    })
    # Thresholds qualify only this toy scenario; retain the individual measurements.
    metrics["toy_acceptance_passed"] = bool(
        metrics["loop_completed"] and metrics["occupied_precision_1cell"] >= .80
        and metrics["occupied_recall_1cell"] >= .65
        and metrics["position_rmse_m"] <= .25
        and metrics["route_cells_reachable_fraction"] >= .95
    )
    _write_pgm(output / "ground_truth.pgm", truth)
    (output / "ground_truth.yaml").write_text(
        f"image: ground_truth.pgm\nmode: trinary\nresolution: {slam_config.map.resolution}\n"
        f"origin: [{slam_config.map.origin_x}, {slam_config.map.origin_y}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n", encoding="utf-8")
    with (output / "trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(trajectory[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(trajectory)
    _write_overview(output / "overview.svg", occupancy, slam_config.map, trajectory)
    (output / "metrics.json").write_text(json.dumps(metrics, indent=2, allow_nan=False, default=_json_default) + "\n", encoding="utf-8")
    return metrics


def _json_default(value):
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if hasattr(value, "value"):
        return value.value
    raise TypeError(f"Not serializable: {type(value).__name__}")
