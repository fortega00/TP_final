"""Tests de la geometría del simulador y de su evaluación, sin depender del SLAM."""
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from slam.core.geometry import Pose2D
from slam.toy.world import OfficeWorld, ToyConfig, evaluate_map, generate_samples, load_trinary_map
from slam.core.config import MapConfig


class ToyTests(unittest.TestCase):
    def test_analytic_raycast_sees_doorway_and_wall(self):
        world = OfficeWorld.standard()
        # Por la puerta (y=0) el rayo llega a la pared exterior; un metro más arriba pega en el tabique.
        self.assertAlmostEqual(world.raycast(Pose2D(0, 0, 0), np.array([0.]), 10.)[0], 7.)
        self.assertAlmostEqual(world.raycast(Pose2D(0, 2, 0), np.array([0.]), 10.)[0], 2.5)

    def test_route_is_closed_collision_free_and_respects_step(self):
        config = ToyConfig()
        world = OfficeWorld.standard()
        route = world.trajectory(config)
        self.assertEqual(route[0], route[-1])
        for previous, pose in zip(route, route[1:]):
            self.assertLessEqual(math.hypot(pose.x-previous.x, pose.y-previous.y), config.translation_step + 1e-10)
            distances = world.raycast(pose, np.linspace(-math.pi, math.pi, 360), 10.)
            self.assertGreater(np.min(distances), config.robot_radius + config.safety_margin)

    def test_noise_streams_reproducible_and_separate(self):
        first = list(generate_samples(ToyConfig(), seed=7))
        repeated = list(generate_samples(ToyConfig(), seed=7))
        noisy = list(generate_samples(ToyConfig(sensor_noise=.1), seed=7))
        self.assertEqual(len(first), len(repeated))
        for a, b, c in zip(first, repeated, noisy):
            self.assertEqual(a.odometry, b.odometry)
            self.assertEqual(a.odometry, c.odometry)
            np.testing.assert_array_equal(a.scan.ranges, b.scan.ranges)

    def test_ground_truth_is_perfect_and_door_stays_connected(self):
        config = MapConfig()
        world = OfficeWorld.standard()
        truth = world.rasterize(config)
        metrics = evaluate_map(truth, truth, config, world.trajectory(ToyConfig()))
        self.assertEqual(metrics["occupied_iou_exact"], 1.)
        self.assertEqual(metrics["occupied_precision_1cell"], 1.)
        self.assertEqual(metrics["occupied_recall_1cell"], 1.)
        self.assertEqual(metrics["route_cells_reachable_fraction"], 1.)

    def test_unknown_space_does_not_become_navigable(self):
        config = MapConfig()
        truth = OfficeWorld.standard().rasterize(config)
        unknown = np.full(truth.shape, -1, dtype=np.int8)
        metrics = evaluate_map(unknown, truth, config, [Pose2D(0, 0, 0), Pose2D(5, 0, 0)])
        self.assertEqual(metrics["route_cells_reachable_fraction"], 0.)
        self.assertEqual(metrics["known_fraction_inside_room"], 0.)
        self.assertEqual(metrics["occupied_iou_exact"], 0.)

    def test_saved_artifact_preserves_unknown_thresholds_and_row_order(self):
        from slam.core.grid import OccupancyGrid
        from slam.core.io import save_map

        config = MapConfig(resolution=.1, width=.4, height=.2, origin_x=-.2, origin_y=-.1)
        grid = OccupancyGrid(config)
        probabilities = np.array([[.10, .249, .251, .50], [.649, .651, .90, .10]])
        grid.log_odds[:] = np.log(probabilities / (1-probabilities))
        grid.observed[:] = True
        grid.observed[1, 3] = False
        expected = np.array([[0, 0, -1, -1], [-1, 100, 100, -1]], dtype=np.int8)
        with tempfile.TemporaryDirectory() as directory:
            _, metadata_path = save_map(grid, Path(directory) / "test")
            loaded, metadata = load_trinary_map(metadata_path)
            np.testing.assert_array_equal(loaded, expected)
            self.assertEqual(metadata["origin"], [-.2, -.1, 0.])
            self.assertEqual(metadata["resolution"], .1)


if __name__ == "__main__":
    unittest.main()
