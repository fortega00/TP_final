"""Equivalencia numérica del muestreo compilado y del camino NumPy."""
import unittest
from unittest.mock import patch

import numpy as np

from slam.core.acceleration import _lookup_numpy, integrate_scan, lookup_scores, nearest_squared, warmup
from slam.core.config import MapConfig, SensorConfig
from slam.core.geometry import Pose2D
from slam.core.grid import OccupancyGrid
from slam.core.types import Scan
from slam.core.sensor import LaserSensorModel


class AccelerationTests(unittest.TestCase):
    def test_lookup_preserves_boundary_and_outside_cells(self):
        warmup()
        field = np.arange(35, dtype=float).reshape(5, 7)
        x = np.array([-0.1, 0., .05, .35, .399999999999, .4], dtype=float)
        y = np.array([0., .1, .15, .2, .3, .4], dtype=float)
        beam_x = np.array([[0., .05, -.05, .3], [.1, -.1, .5, -.6]])
        beam_y = np.array([[0., -.1, .2, .05], [.2, .1, -.2, .6]])
        ids = np.array([0, 1, 0, 1, 0, 1], dtype=np.int64)
        args = x, y, beam_x, beam_y, ids, -.1, -.1, .05, 1, 1, field, -99.
        np.testing.assert_array_equal(lookup_scores(*args), _lookup_numpy(*args))

    def test_lookup_matches_numpy_for_random_queries(self):
        rng = np.random.default_rng(831)
        x, y = rng.normal(size=(2, 83))
        beam_x, beam_y = rng.normal(size=(2, 13, 61))
        ids = rng.integers(0, 13, size=83, dtype=np.int64)
        field = rng.normal(size=(81, 65))
        args = x, y, beam_x, beam_y, ids, -2., -2., .05, 0, 0, field, -3.
        np.testing.assert_array_equal(lookup_scores(*args), _lookup_numpy(*args))

    @unittest.skipIf(integrate_scan is None, "Numba unavailable")
    def test_grid_jit_is_bitwise_equal_with_overlaps_invalid_returns_and_clipping(self):
        rng = np.random.default_rng(63)
        for dtype in (np.float32, np.float64):
            cfg = SensorConfig(no_return_is_free=True)
            grid = OccupancyGrid(MapConfig(width=4., height=4., origin_x=-2., origin_y=-2.))
            reference = grid.clone()
            for index in range(15):
                values = rng.uniform(.05, 9., 180).astype(dtype)
                values[:6] = [np.nan, np.inf, -np.inf, 0., -1., 12.]
                scan = Scan(values, -np.pi, 2*np.pi/180, .1, 10., float(index), Pose2D(.12, -.03, 2.))
                pose = Pose2D(*rng.uniform(-1., 1., 3))
                grid.integrate(pose, scan, cfg)
                with patch('slam.core.grid.integrate_scan', None):
                    reference.integrate(pose, scan, cfg)
                np.testing.assert_array_equal(grid.observed, reference.observed)
                np.testing.assert_array_equal(grid.log_odds, reference.log_odds)
                self.assertEqual(grid.clipped_rays, reference.clipped_rays)

    @unittest.skipIf(nearest_squared is None, "Numba unavailable")
    def test_likelihood_matches_numpy_with_boundary_and_outside_endpoints(self):
        rng = np.random.default_rng(872)
        cfg = SensorConfig()
        grid = OccupancyGrid(MapConfig(width=4., height=4., origin_x=-2., origin_y=-2.))
        grid.observed[:] = True
        grid.log_odds[rng.random(grid.log_odds.shape) < .2] = 3.
        sensor = LaserSensorModel(cfg)
        for index in range(12):
            values = rng.uniform(.2, 5., 180)
            scan = Scan(values, -np.pi, 2*np.pi/180, .1, 8., float(index))
            pose = Pose2D(*rng.uniform(-2., 2., 3))
            actual = sensor.log_likelihood(grid, pose, scan)
            with patch('slam.core.sensor.nearest_squared', None):
                expected = sensor.log_likelihood(grid, pose, scan)
            self.assertAlmostEqual(actual, expected, places=12)


if __name__ == "__main__":
    unittest.main()
