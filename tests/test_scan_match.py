"""Scan-matching proposal: places a particle where the scan fits its own map."""

from dataclasses import replace
import math
import unittest

import numpy as np

from tp_slam.config import FilterConfig, MapConfig, MotionConfig, ScanMatchConfig, SlamConfig
from tp_slam.filter import FastSLAM
from tp_slam.geometry import Pose2D
from tp_slam.grid import OccupancyGrid
from tp_slam.scan_match import ScanMatchProposal
from tp_slam.types import Scan

# Rectangular room centred on the origin. Walls sit on cell centres (5 cm cells), so
# the map has no half-cell bias and the expected accuracy is one search step.
HALF_X, HALF_Y = 2.025, 1.525


def room_scan(pose=Pose2D(), beams=360):
    """Exact ranges a lidar at ``pose`` would measure inside the room."""
    angles = np.arange(beams) * (2 * math.pi / beams)
    world = angles + pose.yaw
    c, s = np.cos(world), np.sin(world)
    with np.errstate(divide="ignore"):
        tx = np.where(c > 0, (HALF_X - pose.x) / c, np.where(c < 0, (-HALF_X - pose.x) / c, np.inf))
        ty = np.where(s > 0, (HALF_Y - pose.y) / s, np.where(s < 0, (-HALF_Y - pose.y) / s, np.inf))
    return Scan(np.minimum(tx, ty), 0.0, 2 * math.pi / beams, 0.1, 8.0, 0.0, Pose2D())


def config(**scan_match):
    return SlamConfig(map=MapConfig(resolution=0.05, width=8, height=8, origin_x=-4, origin_y=-4),
                      motion=MotionConfig(0, 0, 0, 0), filter=FilterConfig(particles=2),
                      scan_match=ScanMatchConfig(enabled=True, sample_xy=0.0, sample_yaw=0.0, **scan_match))


def mapped_room(cfg):
    grid = OccupancyGrid(cfg.map)
    for _ in range(3):
        grid.integrate(Pose2D(), room_scan(), cfg.sensor)
    return grid


class ScanMatchTests(unittest.TestCase):
    def test_corrects_a_wrong_odometry_increment(self):
        cfg = config()
        grid = mapped_room(cfg)
        true_pose = Pose2D(0.30, -0.20, 0.10)
        wrong_delta = Pose2D(0.36, -0.15, 0.13)  # odometry is off by 6 cm, 5 cm and 0.03 rad
        pose, correction = ScanMatchProposal(cfg).propose(Pose2D(), wrong_delta, room_scan(true_pose),
                                                          grid, np.random.default_rng(0))
        self.assertLess(math.hypot(pose.x - true_pose.x, pose.y - true_pose.y), 0.03)
        self.assertLess(abs(pose.yaw - true_pose.yaw), 0.011)
        self.assertEqual(correction, 0.0)

    def test_keeps_a_correct_odometry_increment(self):
        cfg = config()
        true_pose = Pose2D(0.30, -0.20, 0.10)
        pose, _ = ScanMatchProposal(cfg).propose(Pose2D(), true_pose, room_scan(true_pose),
                                                 mapped_room(cfg), np.random.default_rng(0))
        self.assertLess(math.hypot(pose.x - true_pose.x, pose.y - true_pose.y), 0.03)

    def test_empty_map_uses_the_odometry_increment(self):
        cfg = config()
        delta = Pose2D(0.36, -0.15, 0.13)
        pose, _ = ScanMatchProposal(cfg).propose(Pose2D(), delta, room_scan(), OccupancyGrid(cfg.map),
                                                 np.random.default_rng(0))
        self.assertAlmostEqual(pose.x, delta.x)
        self.assertAlmostEqual(pose.y, delta.y)
        self.assertAlmostEqual(pose.yaw, delta.yaw)

    def test_filter_selects_the_proposal_from_config(self):
        self.assertIsInstance(FastSLAM(config()).proposal, ScanMatchProposal)
        self.assertNotIsInstance(FastSLAM(SlamConfig()).proposal, ScanMatchProposal)

    def test_invalid_settings_are_rejected(self):
        base = config()
        for bad in (dict(enabled=1), dict(sigma=0.0), dict(step_xy=0.0), dict(window_xy=-0.1), dict(beams=0)):
            with self.assertRaises(ValueError):
                SlamConfig(scan_match=replace(base.scan_match, **bad))


if __name__ == "__main__":
    unittest.main()
