"""Per-particle finite occupancy grid with log-odds inverse-sensor updates."""

import math
import numpy as np

from .config import MapConfig, SensorConfig
from .geometry import Pose2D
from .types import Scan
from .sensor import prepare_scan


class OccupancyGrid:
    def __init__(self, config: MapConfig):
        self.config = config
        self.width = int(math.ceil(config.width / config.resolution))
        self.height = int(math.ceil(config.height / config.resolution))
        self.prior_log_odds = math.log(config.prior / (1 - config.prior))
        self.log_odds = np.full((self.height, self.width), self.prior_log_odds, dtype=np.float32)
        self.observed = np.zeros((self.height, self.width), dtype=bool)
        self.clipped_rays = 0

    def clone(self):
        result = object.__new__(OccupancyGrid)
        result.config = self.config
        result.width, result.height = self.width, self.height
        result.prior_log_odds = self.prior_log_odds
        result.log_odds = self.log_odds.copy()
        result.observed = self.observed.copy()
        result.clipped_rays = self.clipped_rays
        return result

    def world_to_grid(self, x, y):
        col = np.floor((np.asarray(x) - self.config.origin_x) / self.config.resolution).astype(int)
        row = np.floor((np.asarray(y) - self.config.origin_y) / self.config.resolution).astype(int)
        if col.ndim == 0:
            return int(col), int(row)
        return col, row

    def contains(self, x: float, y: float) -> bool:
        col, row = self.world_to_grid(x, y)
        return 0 <= col < self.width and 0 <= row < self.height

    def occupancy(self):
        values = np.rint(100.0 / (1.0 + np.exp(-self.log_odds))).astype(np.int8)
        values[~self.observed] = -1
        return values

    def integrate(self, pose: Pose2D, scan: Scan, config: SensorConfig):
        ranges, angles, hits = prepare_scan(scan, config)
        if not len(ranges):
            return
        sensor_pose = pose.compose(scan.sensor_pose)
        if not self.contains(sensor_pose.x, sensor_pose.y):
            # An out-of-grid sensor is not clipped onto the border; resize/reset instead.
            self.clipped_rays += len(ranges)
            return
        angles = angles + sensor_pose.yaw
        cosines, sines = np.cos(angles), np.sin(angles)
        half = config.hit_thickness / 2
        # Half-cell sampling produces contiguous rays. Cells are updated once per
        # scan, so angular oversampling cannot multiply confidence arbitrarily.
        step = self.config.resolution / 2
        steps = np.arange(0.0, float(ranges.max()) + half + step, step)
        positions = np.broadcast_to(steps, (len(ranges), len(steps)))
        free_mask = positions < (ranges - np.where(hits, half, 0))[:, None]
        xs = sensor_pose.x + positions * cosines[:, None]
        ys = sensor_pose.y + positions * sines[:, None]
        cols, rows = self.world_to_grid(xs, ys)
        inbounds = (cols >= 0) & (cols < self.width) & (rows >= 0) & (rows < self.height)
        free_indices = np.unique((rows * self.width + cols)[free_mask & inbounds])

        # Explicit endpoint samples ensure even a thickness smaller than the
        # sampling step still marks a hit; no occupancy is created at a clipped border.
        hit_ranges = ranges[hits]
        offsets = np.unique(np.append(np.linspace(-half, half,
                                                   max(2, int(math.ceil(2 * half / step)) + 1)), 0.0))
        hit_positions = np.maximum(0.0, hit_ranges[:, None] + offsets)
        hx = sensor_pose.x + hit_positions * cosines[hits, None]
        hy = sensor_pose.y + hit_positions * sines[hits, None]
        hc, hr = self.world_to_grid(hx, hy)
        hit_valid = (hc >= 0) & (hc < self.width) & (hr >= 0) & (hr < self.height)
        # If the measured endpoint lies outside, its uncertainty band must not
        # fabricate an occupied wall just inside the map either.
        ec, er = self.world_to_grid(sensor_pose.x + hit_ranges * cosines[hits],
                                    sensor_pose.y + hit_ranges * sines[hits])
        endpoint_valid = (ec >= 0) & (ec < self.width) & (er >= 0) & (er < self.height)
        hit_valid &= endpoint_valid[:, None]
        hit_indices = np.unique((hr * self.width + hc)[hit_valid])
        free_indices = np.setdiff1d(free_indices, hit_indices, assume_unique=True)
        log_odds = self.log_odds.ravel()
        observed = self.observed.ravel()
        free_increment = math.log(config.p_free / (1 - config.p_free)) - self.prior_log_odds
        hit_increment = math.log(config.p_hit / (1 - config.p_hit)) - self.prior_log_odds
        log_odds[free_indices] = np.clip(log_odds[free_indices] + free_increment,
                                        config.log_odds_min, config.log_odds_max)
        log_odds[hit_indices] = np.clip(log_odds[hit_indices] + hit_increment,
                                       config.log_odds_min, config.log_odds_max)
        observed[free_indices] = True
        observed[hit_indices] = True
        endx = sensor_pose.x + ranges * cosines
        endy = sensor_pose.y + ranges * sines
        endc, endr = self.world_to_grid(endx, endy)
        self.clipped_rays += int(np.sum((endc < 0) | (endc >= self.width) |
                                        (endr < 0) | (endr >= self.height)))
