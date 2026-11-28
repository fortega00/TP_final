"""Limitador de frecuencia de la cámara, sin ROS."""

import unittest

from mission.core.throttle import PairedThrottle, RateGate


def frames(rate, duration, offset=0.0):
    """Tiempos de una cámara a `rate` Hz durante `duration` segundos."""
    return [offset + i / rate for i in range(int(round(duration * rate)))]


class RateGateTests(unittest.TestCase):
    def test_average_rate_matches_request(self):
        gate = RateGate(1 / 3)
        passed = [t for t in frames(30, 10) if gate.accept(t)]
        self.assertEqual(len(passed), 30)
        gaps = [b - a for a, b in zip(passed, passed[1:])]
        self.assertAlmostEqual(min(gaps), 1 / 3, places=6)
        self.assertAlmostEqual(max(gaps), 1 / 3, places=6)

    def test_jitter_does_not_lower_the_rate(self):
        # Cuadros a 30 Hz con +-5 ms de jitter: pedir 3 Hz tiene que dar 3 Hz, no 2,7.
        gate = RateGate(1 / 3)
        stamps = [t + (0.005 if i % 2 else -0.005) for i, t in enumerate(frames(30, 20))]
        passed = sum(gate.accept(t) for t in stamps)
        self.assertEqual(passed, 60)

    def test_slow_input_passes_everything(self):
        gate = RateGate(1 / 3)
        self.assertTrue(all(gate.accept(t) for t in frames(2, 5)))

    def test_gap_restarts_the_schedule(self):
        gate = RateGate(1.0)
        self.assertTrue(gate.accept(0.0))
        self.assertTrue(gate.accept(10.0))
        self.assertFalse(gate.accept(10.5))
        self.assertTrue(gate.accept(11.0))

    def test_time_going_backwards_restarts(self):
        gate = RateGate(1.0)
        self.assertTrue(gate.accept(100.0))
        self.assertFalse(gate.accept(100.5))
        self.assertTrue(gate.accept(0.0))

    def test_rejects_nonpositive_period(self):
        with self.assertRaises(ValueError):
            RateGate(0.0)


class PairedThrottleTests(unittest.TestCase):
    def run_streams(self, throttle, color, depth):
        """Mezcla los dos flujos en orden de llegada y junta lo publicado."""
        events = sorted([(t, 0, f"c{t:.4f}") for t in color] + [(t, 1, f"d{t:.4f}") for t in depth])
        out = []
        for t, kind, message in events:
            handler = throttle.on_primary if kind == 0 else throttle.on_secondary
            out.extend(handler(t, message))
        return out

    def test_each_color_gets_the_nearest_depth(self):
        throttle = PairedThrottle(1 / 3, tolerance=0.02)
        # Profundidad corrida 10 ms respecto del color, las dos a 30 Hz.
        out = self.run_streams(throttle, frames(30, 10), frames(30, 10, offset=0.010))
        colors = [m for c, m in out if c == "primary"]
        depths = [m for c, m in out if c == "secondary"]
        self.assertEqual(len(colors), 30)
        self.assertEqual(len(depths), 30)
        for color, depth in zip(colors, depths):
            self.assertAlmostEqual(float(depth[1:]) - float(color[1:]), 0.010, places=4)
        self.assertEqual(throttle.stats.unpaired, 0)

    def test_depth_arriving_first_is_used(self):
        throttle = PairedThrottle(1.0, tolerance=0.02)
        self.assertEqual(throttle.on_secondary(4.995, "d"), [])
        self.assertEqual(throttle.on_primary(5.0, "c"), [("primary", "c"), ("secondary", "d")])

    def test_depth_arriving_later_is_used(self):
        throttle = PairedThrottle(1.0, tolerance=0.02)
        self.assertEqual(throttle.on_primary(5.0, "c"), [("primary", "c")])
        self.assertEqual(throttle.on_secondary(4.0, "old"), [])
        self.assertEqual(throttle.on_secondary(5.01, "d"), [("secondary", "d")])
        self.assertEqual(throttle.on_secondary(5.02, "d2"), [])

    def test_missing_depth_is_counted(self):
        throttle = PairedThrottle(1.0, tolerance=0.02)
        throttle.on_primary(5.0, "c")
        self.assertEqual(throttle.on_secondary(5.5, "late"), [])
        self.assertEqual(throttle.stats.unpaired, 1)
        self.assertEqual(throttle.stats.paired, 0)

    def test_same_depth_is_not_published_twice(self):
        # Con tolerancia grande, un cuadro de profundidad no puede acompañar dos colores.
        throttle = PairedThrottle(0.1, tolerance=0.5)
        throttle.on_secondary(1.0, "d")
        self.assertEqual(throttle.on_primary(1.0, "c1"), [("primary", "c1"), ("secondary", "d")])
        self.assertEqual(throttle.on_primary(1.1, "c2"), [("primary", "c2")])

    def test_without_depth_only_color_is_throttled(self):
        throttle = PairedThrottle(1 / 3, tolerance=0.02)
        out = self.run_streams(throttle, frames(30, 10), [])
        self.assertEqual([c for c, _ in out], ["primary"] * 30)


if __name__ == "__main__":
    unittest.main()
