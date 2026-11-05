"""Original inverse range sensor and likelihood-field measurement model.

No scan matcher is applied: the proposal is the odometry motion model. The
likelihood field is queried directly around beam endpoints, without scipy.
"""

from typing import Protocol, TYPE_CHECKING
import math
import numpy as np

from .config import SensorConfig
from .geometry import Pose2D
from .types import Scan

if TYPE_CHECKING:
    from .grid import OccupancyGrid


def prepare_scan(scan: Scan, config: SensorConfig, max_beams=None):
    """Return (ranges, angles, is_hit); +inf clears, invalid readings do nothing.

    Physical out-of-range finite readings are discarded. A valid finite reading
    beyond the configured usable limit clears only to that limit.
    """
    limit = min(scan.range_max, config.max_range)
    minimum = max(scan.range_min, config.min_range)
    if minimum >= limit:
        return np.empty(0), np.empty(0), np.empty(0, dtype=bool)
    values = scan.ranges
    valid = ((np.isfinite(values) & (values > minimum) & (values <= scan.range_max))
             | np.isposinf(values))
    indices = np.flatnonzero(valid)
    count = config.max_beams if max_beams is None else max_beams
    if len(indices) > count:
        indices = indices[np.linspace(0, len(indices) - 1, count, dtype=int)]
    readings = values[indices]
    hits = np.isfinite(readings) & (readings < limit)
    return np.minimum(readings, limit), scan.angle_min + indices * scan.angle_increment, hits


class MeasurementModel(Protocol):
    def log_likelihood(self, grid: "OccupancyGrid", pose: Pose2D, scan: Scan) -> float: ...
    def update(self, grid: "OccupancyGrid", pose: Pose2D, scan: Scan) -> None: ...


class LaserSensorModel:
    def __init__(self, config: SensorConfig):
        self.config = config
        self._offset_cache = {}

    def log_likelihood(self, grid: "OccupancyGrid", pose: Pose2D, scan: Scan) -> float:
        cfg = self.config
        occupied = grid.observed & (grid.log_odds >= math.log(cfg.occupied_threshold /
                                                            (1 - cfg.occupied_threshold)))
        if not occupied.any():
            return 0.0  # Bootstrap: an empty map cannot discriminate poses.
        ranges, angles, hits = prepare_scan(scan, cfg, cfg.likelihood_beams)
        if not hits.any():
            return 0.0  # Endpoint model uses finite obstacle returns only.
        sensor_pose = pose.compose(scan.sensor_pose)
        angles = angles[hits] + sensor_pose.yaw
        ranges = ranges[hits]
        x = sensor_pose.x + ranges * np.cos(angles)
        y = sensor_pose.y + ranges * np.sin(angles)
        cols, rows = grid.world_to_grid(x, y)
        resolution = grid.config.resolution
        key = (resolution, cfg.likelihood_max_dist)
        if key not in self._offset_cache:
            radius = int(math.ceil(cfg.likelihood_max_dist / resolution))
            yy, xx = np.mgrid[-radius:radius + 1, -radius:radius + 1]
            mask = (xx * xx + yy * yy) * resolution ** 2 <= cfg.likelihood_max_dist ** 2
            self._offset_cache[key] = (xx[mask], yy[mask])
        dx, dy = self._offset_cache[key]
        neighbor_cols = cols[:, None] + dx
        neighbor_rows = rows[:, None] + dy
        valid = ((neighbor_cols >= 0) & (neighbor_cols < grid.width) &
                 (neighbor_rows >= 0) & (neighbor_rows < grid.height))
        values = occupied[np.clip(neighbor_rows, 0, grid.height - 1),
                          np.clip(neighbor_cols, 0, grid.width - 1)] & valid
        centers_x = grid.config.origin_x + (neighbor_cols + 0.5) * resolution
        centers_y = grid.config.origin_y + (neighbor_rows + 0.5) * resolution
        distances2 = (centers_x - x[:, None]) ** 2 + (centers_y - y[:, None]) ** 2
        nearest2 = np.minimum(np.min(np.where(values, distances2, np.inf), axis=1),
                              cfg.likelihood_max_dist ** 2)
        # Queries outside the map have no supporting obstacle, even if one is near its boundary.
        outside = (cols < 0) | (cols >= grid.width) | (rows < 0) | (rows >= grid.height)
        nearest2[outside] = cfg.likelihood_max_dist ** 2
        probabilities = (cfg.z_hit * np.exp(-nearest2 / (2 * cfg.sigma_hit ** 2))
                         + cfg.z_rand / min(scan.range_max, cfg.max_range))
        return float(cfg.likelihood_temperature * np.log(probabilities).sum())

    def update(self, grid: "OccupancyGrid", pose: Pose2D, scan: Scan) -> None:
        grid.integrate(pose, scan, self.config)
