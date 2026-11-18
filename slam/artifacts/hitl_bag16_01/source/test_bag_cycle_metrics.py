"""Casos analíticos de métricas sin referencia independiente de pose."""
import importlib.util
from pathlib import Path
import sys
import unittest

import numpy as np

scripts = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(scripts))
spec = importlib.util.spec_from_file_location('bag_cycle', scripts / 'analyze_bag_cycle.py')
metrics = importlib.util.module_from_spec(spec)
spec.loader.exec_module(metrics)


class BagCycleMetricsTests(unittest.TestCase):
    def test_interpolation_wraps_yaw_and_rejects_extrapolation(self):
        path = [[10., 0., 0., np.deg2rad(179)], [12., 2., 4., np.deg2rad(-179)]]
        row = metrics.interpolate_path(path, [11.])[0]
        np.testing.assert_allclose(row, [11., 1., 2., np.pi])
        with self.assertRaises(ValueError):
            metrics.interpolate_path(path, [9.])

    def test_unknown_barrier_and_start_component(self):
        pixels = np.full((20, 20), 254, dtype=np.uint8)
        pixels[:, 10] = 128
        meta = {'origin': [0., 0., 0.], 'resolution': .05}
        result = metrics.connectivity(pixels, meta, [[.2, .5], [.8, .5]], [.2, .5], radius=.1)
        self.assertEqual(result['connected'], 1)
        self.assertEqual(result['failed_indices'], [1])
        self.assertEqual(result['admissible_checkpoints'], 2)
        self.assertEqual(result['connected_fraction_of_admissible'], .5)
        self.assertEqual(result['disconnected_admissible_indices'], [1])
        blocked = metrics.connectivity(pixels, meta, [[.2, .5]], [.5, .5], radius=.1)
        self.assertFalse(blocked['start_is_navigable'])
        self.assertEqual(blocked['connected'], 0)

    def test_interior_seed_is_shared_by_every_map(self):
        left = np.full((40, 40), 254, dtype=np.uint8)
        right = left.copy()
        left[20, 20] = 128
        right[20, 10] = 128
        meta = {'origin': [0., 0., 0.], 'resolution': .05}
        maps = {'left': (meta, left), 'right': (meta, right)}
        points = np.array([[.5, 1.], [1., 1.], [1.5, 1.]])
        self.assertEqual(metrics.common_interior_seed(maps, points), 2)
        self.assertIsNone(metrics.common_interior_seed(maps, points[:2]))


if __name__ == '__main__':
    unittest.main()
