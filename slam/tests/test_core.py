"""Propiedades del algoritmo que no dependen de ROS ni del hardware (unittest estándar)."""

from dataclasses import replace
import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np
import yaml

from slam.core.config import SlamConfig, MapConfig, SensorConfig, MotionConfig, FilterConfig, UpdateConfig
from slam.core.filter import FastSLAM, normalize_log_weights, systematic_resample
from slam.core.geometry import Pose2D
from slam.core.grid import OccupancyGrid
from slam.core.io import save_map
from slam.core.sensor import LaserSensorModel, prepare_scan
from slam.core.state_machine import SlamState
from slam.core.types import Scan


def scan(ranges=(1.0,), stamp=0.0, sensor_pose=Pose2D()):
    return Scan(np.array(ranges), 0.0, 0.01, 0.1, 5.0, stamp, sensor_pose)


def small_config(**kwargs):
    base = SlamConfig(map=MapConfig(resolution=0.1, width=8, height=8, origin_x=-4, origin_y=-4),
                      motion=MotionConfig(0, 0, 0, 0), filter=FilterConfig(particles=4),
                      sensor=SensorConfig(hit_thickness=0.04))
    return replace(base, **kwargs)


class GeometryTests(unittest.TestCase):
    def test_se2_composition_inverse_and_relative_lateral_reverse(self):
        initial = Pose2D(2, 3, math.pi / 2)
        delta = Pose2D(-1, 0.5, -0.4)
        result = initial.compose(delta)
        self.assertAlmostEqual(result.x, 1.5)
        self.assertAlmostEqual(result.y, 2)
        recovered = initial.between(result)
        np.testing.assert_allclose([recovered.x, recovered.y, recovered.yaw], [-1, 0.5, -0.4], atol=1e-12)
        identity = initial.compose(initial.inverse())
        np.testing.assert_allclose([identity.x, identity.y, identity.yaw], [0, 0, 0], atol=1e-12)


class GridTests(unittest.TestCase):
    def test_unknown_is_not_prior_probability(self):
        grid = OccupancyGrid(MapConfig())
        self.assertTrue(np.all(grid.occupancy() == -1))
        grid.observed[0, 0] = True
        self.assertEqual(grid.occupancy()[0, 0], 50)

    def test_logodds_prior_subtraction_and_saturation(self):
        grid = OccupancyGrid(MapConfig(resolution=0.1, width=8, height=8,
                                       origin_x=-4, origin_y=-4, prior=0.6))
        sensor = SensorConfig(hit_thickness=0.04)
        grid.integrate(Pose2D(), scan(), sensor)
        cx, cy = grid.world_to_grid(1.0, 0)
        fx, fy = grid.world_to_grid(0.5, 0)
        self.assertAlmostEqual(float(grid.log_odds[cy, cx]), math.log(0.7 / 0.3), places=6)
        self.assertAlmostEqual(float(grid.log_odds[fy, fx]), math.log(0.35 / 0.65), places=6)
        for _ in range(30):
            grid.integrate(Pose2D(), scan(), sensor)
        self.assertEqual(grid.log_odds[cy, cx], sensor.log_odds_max)
        self.assertEqual(grid.log_odds[fy, fx], sensor.log_odds_min)

    def test_nan_negative_inf_zero_invalid_and_positive_inf_no_obstacle(self):
        cfg = small_config()
        grid = OccupancyGrid(cfg.map)
        grid.integrate(Pose2D(), scan((float("nan"), float("-inf"), 0, -1, 0.05, 6.0)), cfg.sensor)
        self.assertFalse(grid.observed.any())
        # Un lidar real devuelve +inf cuando el rayo no vuelve: por defecto no aporta información.
        grid.integrate(Pose2D(), scan((float("inf"),)), cfg.sensor)
        self.assertFalse(grid.observed.any())
        # Un lidar simulado devuelve +inf solo cuando no hay nada hasta su alcance.
        grid.integrate(Pose2D(), scan((float("inf"),)), replace(cfg.sensor, no_return_is_free=True))
        self.assertTrue(grid.observed.any())
        self.assertFalse((grid.log_odds > 0).any())

    def test_maximum_range_and_configured_truncation_are_not_hits(self):
        sensor = replace(SensorConfig(), max_range=2.0)
        ranges, _, hits = prepare_scan(scan((5, 3, 1, float("inf"))), sensor)
        np.testing.assert_allclose(ranges, [2, 2, 1])
        np.testing.assert_array_equal(hits, [False, False, True])
        ranges, _, hits = prepare_scan(scan((5, 3, 1, float("inf"))), replace(sensor, no_return_is_free=True))
        np.testing.assert_allclose(ranges, [2, 2, 1, 2])
        np.testing.assert_array_equal(hits, [False, False, True, False])

    def test_no_return_flag_must_be_boolean(self):
        with self.assertRaises(ValueError):
            SlamConfig(sensor=SensorConfig(no_return_is_free=1))

    def test_hits_at_clipped_endpoints_do_not_make_boundary_walls(self):
        grid = OccupancyGrid(MapConfig(resolution=0.1, width=2, height=2, origin_x=-1, origin_y=-1))
        grid.integrate(Pose2D(), scan((4.0,)), SensorConfig())
        self.assertTrue(grid.observed.any())
        self.assertFalse((grid.log_odds > 0).any())
        self.assertEqual(grid.clipped_rays, 1)

    def test_sensor_offset_and_yaw_are_applied(self):
        cfg = small_config()
        grid = OccupancyGrid(cfg.map)
        grid.integrate(Pose2D(), scan(sensor_pose=Pose2D(0.5, 0, math.pi / 2)), cfg.sensor)
        col, row = grid.world_to_grid(0.5, 1.0)
        self.assertGreater(grid.log_odds[row, col], 0)
        wrong_col, wrong_row = grid.world_to_grid(1.0, 0)
        self.assertEqual(grid.occupancy()[wrong_row, wrong_col], -1)

    def test_oversampled_duplicate_beams_count_once_per_scan(self):
        cfg = small_config()
        grid1, grid2 = OccupancyGrid(cfg.map), OccupancyGrid(cfg.map)
        grid1.integrate(Pose2D(), scan(), cfg.sensor)
        duplicate = Scan(np.ones(40), 0, 1e-12, 0.1, 5, 0)
        grid2.integrate(Pose2D(), duplicate, cfg.sensor)
        col, row = grid1.world_to_grid(1.0, 0)
        self.assertEqual(grid1.log_odds[row, col], grid2.log_odds[row, col])

    def test_clone_is_independent(self):
        grid = OccupancyGrid(small_config().map)
        cloned = grid.clone()
        cloned.log_odds[1, 1] = 3
        cloned.observed[1, 1] = True
        self.assertFalse(grid.observed[1, 1])
        self.assertEqual(grid.log_odds[1, 1], 0)


class FilterTests(unittest.TestCase):
    def test_log_weights_do_not_underflow(self):
        weights = normalize_log_weights([-100000, -100001, -100002])
        self.assertAlmostEqual(float(np.exp(weights).sum()), 1)
        np.testing.assert_allclose(np.exp(weights), [0.66524096, 0.24472847, 0.09003057])

    def test_systematic_resampling_uses_posterior_and_is_deterministic(self):
        result = systematic_resample([0, 0, 1, 0], np.random.default_rng(7))
        np.testing.assert_array_equal(result, [2, 2, 2, 2])
        a = systematic_resample([0.1, 0.2, 0.3, 0.4], np.random.default_rng(7))
        b = systematic_resample([0.1, 0.2, 0.3, 0.4], np.random.default_rng(7))
        np.testing.assert_array_equal(a, b)

    def test_accumulated_motion_does_not_disappear_on_skipped_scans(self):
        slam = FastSLAM(small_config())
        self.assertTrue(slam.process(scan(stamp=0), Pose2D(30, 40, 1)))
        anchor = Pose2D(30, 40, 1)
        for i in (1, 2):
            self.assertFalse(slam.process(scan(stamp=i * 0.1), anchor.compose(Pose2D(i * 0.03, 0, 0))))
        self.assertTrue(slam.process(scan(stamp=0.3), anchor.compose(Pose2D(0.09, 0, 0))))
        self.assertAlmostEqual(slam.best_particle.pose.x, 0.09)
        self.assertAlmostEqual(slam.best_particle.pose.y, 0)
        self.assertEqual(slam.diagnostics["skipped_motion"], 2)

    def test_pause_resume_accumulates_odometry_and_reset_restarts_seed(self):
        slam = FastSLAM(small_config())
        slam.process(scan(stamp=0), Pose2D())
        slam.pause()
        self.assertEqual(slam.state, SlamState.PAUSED)
        self.assertFalse(slam.process(scan(stamp=1), Pose2D(0.5, 0, 0)))
        slam.resume()
        slam.process(scan(stamp=2), Pose2D(0.6, 0, 0))
        self.assertAlmostEqual(slam.best_particle.pose.x, 0.6)
        slam.stop()
        self.assertFalse(slam.process(scan(stamp=3), Pose2D(0.7, 0, 0)))
        slam.reset()
        self.assertEqual(slam.state, SlamState.WAITING)
        self.assertFalse(slam.best_particle.grid.observed.any())
        self.assertIsNone(slam.last_stamp)

    def test_duplicate_and_backward_timestamps_are_rejected(self):
        slam = FastSLAM(small_config())
        slam.process(scan(stamp=2), Pose2D())
        self.assertFalse(slam.process(scan(stamp=2), Pose2D(1, 0, 0)))
        self.assertFalse(slam.process(scan(stamp=1), Pose2D(2, 0, 0)))
        self.assertEqual(slam.diagnostics["rejected_stamp"], 2)
        self.assertEqual(len(slam.best_particle.trajectory), 1)

    def test_measurement_scores_old_maps_then_updates_and_resampling_clones(self):
        cfg = small_config()

        class StrongMeasurement(LaserSensorModel):
            def __init__(self):
                super().__init__(cfg.sensor)
                self.calls = 0

            def log_likelihood(self, grid, pose, reading):
                self.assert_empty = not grid.observed.any()
                if not self.assert_empty:
                    raise AssertionError("Likelihood was evaluated after map update")
                self.calls += 1
                return 100.0 if self.calls == 1 else -100.0

        sensor = StrongMeasurement()
        slam = FastSLAM(cfg, measurement_model=sensor)
        slam.process(scan(), Pose2D())
        self.assertEqual(sensor.calls, 4)
        self.assertEqual(slam.diagnostics["resamples"], 1)
        self.assertEqual(len({id(p.grid.log_odds) for p in slam.particles}), 4)
        slam.particles[0].grid.log_odds.fill(0)
        self.assertTrue(slam.particles[1].grid.log_odds.any())
        self.assertTrue(slam.best_particle.grid.log_odds.any())

    def test_particle_yaw_statistics_are_circular(self):
        slam = FastSLAM(small_config())
        for i, particle in enumerate(slam.particles):
            particle.pose = Pose2D(0, 0, math.pi - 0.01 if i % 2 else -math.pi + 0.01)
        slam._posterior_statistics(np.full(4, 0.25))
        pose, covariance = slam.estimate()
        self.assertAlmostEqual(abs(pose.yaw), math.pi)
        self.assertLess(covariance[2, 2], 0.001)

    def test_custom_proposal_importance_correction_is_applied(self):
        cfg = small_config()

        class TestProposal:
            index = 0

            def propose(self, pose, odometry_delta, reading, grid, rng):
                self.index += 1
                return pose, math.log(4) if self.index == 4 else 0.0

        slam = FastSLAM(cfg, proposal=TestProposal())
        slam.process(scan(), Pose2D())
        weights = np.exp([p.log_weight for p in slam.particles])
        np.testing.assert_allclose(weights, np.array([1, 1, 1, 4]) / 7)
        self.assertAlmostEqual(math.exp(slam.best_particle.log_weight), 4 / 7)

    def test_seed_reset_reproduces_the_same_noisy_posterior(self):
        cfg = small_config(motion=MotionConfig(0.01, 0.001, 0.01, 0.001))
        slam = FastSLAM(cfg)

        def advance():
            slam.process(scan(stamp=0), Pose2D())
            slam.process(scan(stamp=1), Pose2D(0.1, 0.05, 0.03))
            return [(p.pose, p.log_weight, p.grid.log_odds.copy()) for p in slam.particles]

        first = advance()
        slam.reset()
        second = advance()
        for (a_pose, a_weight, a_map), (b_pose, b_weight, b_map) in zip(first, second):
            self.assertEqual(a_pose, b_pose)
            self.assertEqual(a_weight, b_weight)
            np.testing.assert_array_equal(a_map, b_map)

    def test_outside_all_particles_enters_error_until_reset(self):
        slam = FastSLAM(small_config())
        slam.process(scan(), Pose2D())
        with self.assertRaises(ValueError):
            slam.process(scan(stamp=1), Pose2D(20, 0, 0))
        self.assertEqual(slam.state, SlamState.ERROR)
        slam.reset()
        self.assertEqual(slam.state, SlamState.WAITING)

    def test_likelihood_prefers_pose_aligned_with_existing_wall(self):
        cfg = small_config()
        grid = OccupancyGrid(cfg.map)
        model = LaserSensorModel(cfg.sensor)
        model.update(grid, Pose2D(), scan())
        aligned = model.log_likelihood(grid, Pose2D(), scan())
        wrong = model.log_likelihood(grid, Pose2D(0.5, 0, 0), scan())
        self.assertGreater(aligned, wrong)

    def test_diagnostics_are_json_serializable(self):
        slam = FastSLAM(small_config())
        slam.process(scan(), Pose2D())
        json.dumps(slam.diagnostics)


class ExportAndConfigTests(unittest.TestCase):
    def test_export_flips_y_and_preserves_unknown(self):
        grid = OccupancyGrid(MapConfig(resolution=1, width=2, height=2, origin_x=-1, origin_y=-1))
        grid.observed[0, 0], grid.log_odds[0, 0] = True, 4
        grid.observed[1, 1], grid.log_odds[1, 1] = True, -4
        with tempfile.TemporaryDirectory() as directory:
            pgm, metadata = save_map(grid, Path(directory) / "map")
            data = pgm.read_bytes().split(b"\n", 3)[3]
            np.testing.assert_array_equal(np.frombuffer(data, dtype=np.uint8).reshape(2, 2),
                                          [[128, 254], [0, 128]])
            settings = yaml.safe_load(metadata.read_text())
            self.assertEqual(settings["origin"], [-1, -1, 0])
            unknown_probability = (255 - 128) / 255
            self.assertLess(settings["free_thresh"], unknown_probability)
            self.assertGreater(settings["occupied_thresh"], unknown_probability)

    def test_config_roundtrip_and_unknown_parameters_fail(self):
        cfg = SlamConfig()
        self.assertEqual(SlamConfig.from_dict(cfg.to_dict()), cfg)
        with self.assertRaises(ValueError):
            SlamConfig.from_dict({"sensor": {"typo": 1}})
        with self.assertRaises(ValueError):
            SlamConfig.from_dict({"typo": {}})
        with self.assertRaises(ValueError):
            SlamConfig.from_dict({"map": {"resolution": 0}})
        with self.assertRaises(ValueError):
            SlamConfig.from_dict({"filter": {"particles": 2.5}})


if __name__ == "__main__":
    unittest.main()
