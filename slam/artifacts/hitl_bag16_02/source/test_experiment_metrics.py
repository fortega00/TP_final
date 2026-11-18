"""Casos analíticos para las convenciones de las métricas experimentales."""
import importlib.util
from pathlib import Path
import unittest
import tempfile
import numpy as np

path = Path(__file__).resolve().parents[1] / 'scripts/experiment_metrics.py'
spec = importlib.util.spec_from_file_location('experiment_metrics', path)
metrics = importlib.util.module_from_spec(spec)
spec.loader.exec_module(metrics)


class ExperimentMetricsTests(unittest.TestCase):
    def test_laser_mount_and_base_rotation(self):
        points = metrics.laser_endpoints([2., 3., np.pi/2], [.2, 0., np.pi/2],
                                         np.array([1.]), np.array([0.]))
        np.testing.assert_allclose(points, [[1., 3.2]])

    def test_inflation_is_metric_and_unknown_blocks_footprint(self):
        disk = metrics.metric_disk(.22, .05)
        self.assertTrue(disk[5, 9])
        self.assertFalse(disk[5, 10])
        free = np.ones((25, 25), bool)
        free[12, 12] = False
        nav = metrics.navigable_cells(free, .05, .22)
        self.assertFalse(nav[12, 16])
        self.assertTrue(nav[12, 17])
        self.assertFalse(nav[0, 12])

    def test_yaw_wrap(self):
        error = metrics.angular_error(np.deg2rad(-179), np.deg2rad(179))
        self.assertAlmostEqual(float(np.rad2deg(error)), 2.)

    def test_endpoint_metric_reads_input_mount(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder)/'input.npz'
            np.savez(source, metadata=[[1., 0., 1., .1, 8.]], ranges=[[1.]],
                     laser=[.2, 0., 0.])
            samples = {'paths': {'slam': [[1., 0., 0., 0.]]},
                       'configuration': {'sensor': {'min_range': .1, 'max_range': 8.}}}
            occupied = np.zeros((1, 2), bool)
            occupied[0, 1] = True
            result = metrics.endpoint_consistency(samples, occupied,
                     {'resolution': 1., 'origin': [-.3, -.5, 0.]}, source)
            self.assertAlmostEqual(result['distance_m']['median'], 0., places=7)
            self.assertEqual(result['fraction_within_10cm'], 1.)

    def test_truth_anchor_and_yaw_are_consistent(self):
        samples = {'paths': {'slam': [[1., 2., 3., 0.]]}, 'configuration': {
                   'filter': {'initial_x': 2., 'initial_y': 3., 'initial_yaw': 0.}}}
        data = {'metadata': np.array([[1.], [2.]]),
                'truth': np.array([[10., 20., np.pi/2], [10., 21., np.pi/2]])}
        truth = metrics.aligned_truth(samples, data, np.array([1., 2.]))
        np.testing.assert_allclose(truth, [[2., 3., 0.], [3., 3., 0.]], atol=1e-12)


if __name__ == '__main__':
    unittest.main()
