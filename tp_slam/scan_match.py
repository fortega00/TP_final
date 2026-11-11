"""Scan-matching proposal: place each particle where the scan fits its own map.

The odometry proposal scatters particles around the odometry increment and lets
the weights discard the bad ones. The lidar is far more precise than the wheels,
so this proposal uses the scan first (the idea behind GMapping / FastSLAM 2.0):

1. guess = previous pose composed with the odometry increment, without noise;
2. every pose of a small window around the guess is scored by how close the
   scan endpoints fall to occupied cells of the particle's map, plus a Gaussian
   prior that keeps the answer near the guess where the scan cannot decide
   (a featureless corridor);
3. the best pose is returned with a small Gaussian sample to keep diversity.

While the map has too few occupied cells there is nothing to match against, so
the plain odometry proposal is used.

Simplification: the exact importance weight of this proposal integrates the
likelihood over the window. Here the correction is 0, so the filter weights
each particle by the scan likelihood at its matched pose.
"""

import math
import numpy as np
from scipy.ndimage import distance_transform_edt

from .config import SlamConfig
from .geometry import Pose2D, wrap_angle
from .motion import OdometryProposal, SE2OdometryModel
from .sensor import prepare_scan


class ScanMatchProposal:
    def __init__(self, config: SlamConfig):
        self.sensor = config.sensor
        self.config = c = config.scan_match
        self.odometry = OdometryProposal(SE2OdometryModel(config.motion))
        offsets_xy = np.arange(-c.window_xy, c.window_xy + 1e-9, c.step_xy)
        offsets_yaw = np.arange(-c.window_yaw, c.window_yaw + 1e-9, c.step_yaw)
        self.dx, self.dy, self.dyaw = (a.ravel() for a in np.meshgrid(
            offsets_xy, offsets_xy, offsets_yaw, indexing="ij"))
        self.log_prior = (-(self.dx ** 2 + self.dy ** 2) / (2 * c.prior_xy ** 2)
                          - self.dyaw ** 2 / (2 * c.prior_yaw ** 2))

    def propose(self, pose: Pose2D, odometry_delta: Pose2D, scan, grid, rng: np.random.Generator):
        c, sensor = self.config, self.sensor
        threshold = math.log(sensor.occupied_threshold / (1 - sensor.occupied_threshold))
        occupied = grid.observed & (grid.log_odds >= threshold)
        ranges, angles, hits = prepare_scan(scan, sensor, c.beams)
        if occupied.sum() < c.min_occupied_cells or not hits.any():
            return self.odometry.propose(pose, odometry_delta, scan, grid, rng)
        ranges, angles = ranges[hits], angles[hits]
        guess = pose.compose(odometry_delta)

        # Distance to the nearest occupied cell, only in the patch the scan can reach.
        resolution = grid.config.resolution
        max_range = min(scan.range_max, sensor.max_range)
        reach = int(math.ceil((max_range + c.window_xy) / resolution)) + 1
        col, row = (int(v) for v in grid.world_to_grid(guess.x, guess.y))
        col0, col1 = max(col - reach, 0), min(col + reach + 1, grid.width)
        row0, row1 = max(row - reach, 0), min(row + reach + 1, grid.height)
        patch = occupied[row0:row1, col0:col1]
        if not patch.any():
            return self.odometry.propose(pose, odometry_delta, scan, grid, rng)
        distance = np.minimum(distance_transform_edt(~patch) * resolution, sensor.likelihood_max_dist)

        # Scan endpoints for every candidate pose: arrays of shape (candidates, beams).
        yaw = guess.yaw + self.dyaw
        mount = scan.sensor_pose
        sensor_x = guess.x + self.dx + np.cos(yaw) * mount.x - np.sin(yaw) * mount.y
        sensor_y = guess.y + self.dy + np.sin(yaw) * mount.x + np.cos(yaw) * mount.y
        beam = yaw[:, None] + mount.yaw + angles[None, :]
        cols = np.floor((sensor_x[:, None] + ranges * np.cos(beam) - grid.config.origin_x)
                        / resolution).astype(int) - col0
        rows = np.floor((sensor_y[:, None] + ranges * np.sin(beam) - grid.config.origin_y)
                        / resolution).astype(int) - row0
        inside = (cols >= 0) & (cols < patch.shape[1]) & (rows >= 0) & (rows < patch.shape[0])
        d = np.full(cols.shape, sensor.likelihood_max_dist)
        d[inside] = distance[rows[inside], cols[inside]]

        # Same mixture as the likelihood field: the uniform term makes the score
        # saturate for endpoints far from any known wall (newly seen areas), so
        # they cannot drag the pose. Neighbouring beams are correlated, so the
        # mean over beams is scaled as if only `effective_beams` were independent.
        probability = sensor.z_hit * np.exp(-d ** 2 / (2 * c.sigma ** 2)) + sensor.z_rand / max_range
        score = c.effective_beams * np.log(probability).mean(axis=1) + self.log_prior
        best = int(np.argmax(score))
        noise = rng.normal(size=3)
        return Pose2D(guess.x + self.dx[best] + noise[0] * c.sample_xy,
                      guess.y + self.dy[best] + noise[1] * c.sample_xy,
                      wrap_angle(yaw[best] + noise[2] * c.sample_yaw)), 0.0
