"""Regresiones del reloj y de los scans que habilitan navegación."""

from pathlib import Path
import unittest

from slam.core.types import Scan

from nav.core.config import NavConfig
from nav.core.scan_watchdog import ScanWatchdog

CONFIGS = Path(__file__).resolve().parents[1] / "config"


def make_scan(stamp, ranges=(1.0,)):
    return Scan(ranges, 0, 0.01, 0.05, 12.0, stamp)


def watchdog(profile, simulated):
    cfg = NavConfig.load_yaml(CONFIGS / f"{profile}.yaml")
    return ScanWatchdog(cfg.navigator.scan_timeout, cfg.sensor.min_range, simulated)


class ScanWatchdogTests(unittest.TestCase):
    def test_real_time_uses_local_reception_despite_offset_and_jitter(self):
        guard = watchdog("real", False)
        self.assertFalse(guard.ready(16.0))
        self.assertTrue(guard.accept(make_scan(1700000000.0), 16.0))
        # Gap de tres periodos nominales (dos pérdidas) más jitter.
        self.assertTrue(guard.ready(16.48))
        self.assertTrue(guard.accept(make_scan(1700000000.41), 16.48))
        self.assertTrue(guard.ready(16.95))
        self.assertFalse(guard.ready(17.0))

    def test_invalid_duplicate_and_reordered_scans_do_not_refresh_the_timeout(self):
        guard = watchdog("real", False)
        self.assertTrue(guard.accept(make_scan(30.0), 10.0))
        for scan in (make_scan(30.0), make_scan(29.9),
                     make_scan(30.3, (0, 0.10, 12.1, float("nan"), float("inf")))):
            with self.subTest(stamp=scan.stamp, ranges=scan.ranges):
                self.assertFalse(guard.accept(scan, 10.48))
        self.assertFalse(guard.ready(10.5))
        self.assertEqual(guard.silence(10.5), 0.5)

    def test_simulated_time_rejects_delayed_scans_and_ages_the_capture(self):
        guard = watchdog("cafe", True)
        self.assertFalse(guard.accept(make_scan(12.5), 13.0))
        self.assertTrue(guard.accept(make_scan(12.8), 13.0))
        self.assertTrue(guard.ready(13.24))
        self.assertFalse(guard.ready(13.26))
        self.assertFalse(guard.ready(12.0))
        self.assertTrue(guard.clock_rewound)
        self.assertFalse(guard.accept(make_scan(14.0), 14.0))


if __name__ == '__main__':
    unittest.main()
