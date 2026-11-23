"""Carga del mapa y localización Monte Carlo sobre una sala sintética."""

from dataclasses import replace
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from slam.core.config import MotionConfig, ScanMatchConfig
from slam.core.geometry import Pose2D
from slam.core.types import Scan

from nav.core.config import LocalizationConfig, NavConfig
from nav.core.localization import MonteCarloLocalization
from nav.core.map import free_cells, load_map, occupied_cells
from nav.core.obstacles import ObstacleLayer, ObstacleSnapshot

RESOLUTION = 0.05
# Sala de 6 x 4 m con una columna cuadrada de 0,5 m adentro. La columna rompe la
# simetría: sin ella, la sala se ve igual girada 180 grados.
ROOM_X, ROOM_Y = (-3.0, 3.0), (-2.0, 2.0)
PILLAR_X, PILLAR_Y = (1.0, 1.5), (0.5, 1.0)


def write_map(folder: Path) -> Path:
    """Escribe la sala como PGM + YAML, con un borde de celdas desconocidas."""
    margin = 0.5
    xs = np.arange(ROOM_X[0] - margin, ROOM_X[1] + margin, RESOLUTION) + RESOLUTION / 2
    ys = np.arange(ROOM_Y[0] - margin, ROOM_Y[1] + margin, RESOLUTION) + RESOLUTION / 2
    x, y = np.meshgrid(xs, ys)
    inside = (x > ROOM_X[0]) & (x < ROOM_X[1]) & (y > ROOM_Y[0]) & (y < ROOM_Y[1])
    wall = ((x > ROOM_X[0] - RESOLUTION) & (x < ROOM_X[1] + RESOLUTION) &
            (y > ROOM_Y[0] - RESOLUTION) & (y < ROOM_Y[1] + RESOLUTION) & ~inside)
    pillar = (x > PILLAR_X[0]) & (x < PILLAR_X[1]) & (y > PILLAR_Y[0]) & (y < PILLAR_Y[1])
    pixels = np.full(x.shape, 205, dtype=np.uint8)   # desconocido
    pixels[inside] = 254                              # libre
    pixels[wall | pillar] = 0                         # ocupado
    (folder / "room.pgm").write_bytes(
        f"P5\n{x.shape[1]} {x.shape[0]}\n255\n".encode() + np.flipud(pixels).tobytes())
    (folder / "room.yaml").write_text(
        f"image: room.pgm\nmode: trinary\nresolution: {RESOLUTION}\n"
        f"origin: [{ROOM_X[0] - margin}, {ROOM_Y[0] - margin}, 0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    return folder / "room.yaml"


def _box_distance(px, py, c, s, x_limits, y_limits):
    """Distancia de cada rayo (origen px, py; dirección c, s) hasta entrar a una caja."""
    with np.errstate(divide="ignore", invalid="ignore"):
        tx0, tx1 = (x_limits[0] - px) / c, (x_limits[1] - px) / c
        ty0, ty1 = (y_limits[0] - py) / s, (y_limits[1] - py) / s
    enter = np.maximum(np.minimum(tx0, tx1), np.minimum(ty0, ty1))
    leave = np.minimum(np.maximum(tx0, tx1), np.maximum(ty0, ty1))
    return np.where((enter <= leave) & (enter > 0), enter, np.inf)


def room_scan(pose: Pose2D, stamp=0.0, beams=360) -> Scan:
    """Rangos exactos que mediría un lidar en `pose`: paredes de la sala y columna."""
    angles = np.arange(beams) * (2 * math.pi / beams)
    c, s = np.cos(angles + pose.yaw), np.sin(angles + pose.yaw)
    with np.errstate(divide="ignore"):
        tx = np.where(c > 0, (ROOM_X[1] - pose.x) / c, (ROOM_X[0] - pose.x) / c)
        ty = np.where(s > 0, (ROOM_Y[1] - pose.y) / s, (ROOM_Y[0] - pose.y) / s)
    walls = np.minimum(np.abs(tx), np.abs(ty))
    pillar = _box_distance(pose.x, pose.y, c, s, PILLAR_X, PILLAR_Y)
    return Scan(np.minimum(walls, pillar), 0.0, 2 * math.pi / beams, 0.1, 8.0, stamp, Pose2D())


def error(estimate: Pose2D, truth: Pose2D):
    return (math.hypot(estimate.x - truth.x, estimate.y - truth.y),
            abs(math.atan2(math.sin(estimate.yaw - truth.yaw), math.cos(estimate.yaw - truth.yaw))))


class MapTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.grid = load_map(write_map(Path(self.folder.name)))

    def tearDown(self):
        self.folder.cleanup()

    def cell(self, mask, x, y):
        col, row = self.grid.world_to_grid(x, y)
        return bool(mask[row, col])

    def test_cells_are_classified_and_oriented(self):
        occupied, free = occupied_cells(self.grid), free_cells(self.grid)
        # Centro de la sala: libre. La columna está arriba a la derecha (y positiva):
        # si la imagen no se invirtiera en vertical quedaría en y negativa.
        self.assertTrue(self.cell(free, 0.0, 0.0))
        self.assertTrue(self.cell(occupied, 1.25, 0.75))
        self.assertTrue(self.cell(free, 1.25, -0.75))
        # Pared izquierda ocupada; más afuera, desconocido (ni libre ni ocupado).
        self.assertTrue(self.cell(occupied, -3.025, 0.0))
        self.assertFalse(self.cell(occupied, -3.3, 0.0))
        self.assertFalse(self.cell(free, -3.3, 0.0))

    def test_rejects_negated_map(self):
        path = Path(self.folder.name) / "room.yaml"
        path.write_text(path.read_text().replace("negate: 0", "negate: 1"))
        with self.assertRaises(ValueError):
            load_map(path)


class LocalizationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.folder = tempfile.TemporaryDirectory()
        cls.grid = load_map(write_map(Path(cls.folder.name)))

    @classmethod
    def tearDownClass(cls):
        cls.folder.cleanup()

    def filter(self, **overrides):
        config = NavConfig(motion=MotionConfig(alpha1=0.02, alpha2=0.002, alpha3=0.02, alpha4=0.02))
        scan_match = overrides.pop("scan_match", False)
        config = replace(config, scan_match=ScanMatchConfig(enabled=scan_match),
                         localization=replace(LocalizationConfig(), **overrides))
        return MonteCarloLocalization(self.grid, config)

    def test_burst_converges_from_a_rough_click(self):
        truth = Pose2D(-1.0, -0.5, 0.4)
        mcl = self.filter()
        # El click cae a 36 cm y 12 grados de la pose real.
        mcl.initialize(Pose2D(-0.7, -0.3, 0.6), std_xy=0.5, std_yaw=0.26)
        self.assertFalse(mcl.converged)
        self.assertTrue(mcl.converge(room_scan(truth)))
        distance, angle = error(mcl.estimate()[0], truth)
        self.assertLess(distance, 0.05)
        self.assertLess(angle, math.radians(2))

    def drive(self, mcl, scale=1.15, steps=40):
        """Avanza en línea recta con una odometría que mide 15 % de más.

        Devuelve (error del filtro, error de la odometría sola) al final.
        """
        truth = Pose2D(-2.0, -1.0, 0.0)
        mcl.initialize(truth, std_xy=0.05, std_yaw=0.02)
        mcl.converge(room_scan(truth))
        odom = Pose2D()
        mcl.process(room_scan(truth), odom)
        step = Pose2D(0.1, 0.0, 0.0)
        for i in range(steps):
            truth = truth.compose(step)
            odom = odom.compose(Pose2D(step.x * scale, 0.0, 0.0))
            self.assertTrue(mcl.process(room_scan(truth, stamp=i + 1.0), odom))
        dead_reckoning = Pose2D(-2.0, -1.0, 0.0).compose(odom)
        return error(mcl.estimate()[0], truth)[0], error(dead_reckoning, truth)[0]

    def test_tracking_corrects_odometry_drift(self):
        filter_error, odometry_error = self.drive(self.filter())
        self.assertGreater(odometry_error, 0.5)   # la odometría sola termina a 60 cm
        self.assertLess(filter_error, 0.08)

    def test_tracking_with_scan_matching(self):
        filter_error, _ = self.drive(self.filter(scan_match=True))
        self.assertLess(filter_error, 0.08)

    def test_standing_still_does_not_update(self):
        truth = Pose2D(0.0, 0.0, 0.0)
        mcl = self.filter()
        mcl.initialize(truth, std_xy=0.2, std_yaw=0.1)
        mcl.process(room_scan(truth), Pose2D())
        before = mcl.poses.copy()
        self.assertFalse(mcl.process(room_scan(truth, stamp=1.0), Pose2D()))
        np.testing.assert_array_equal(mcl.poses, before)
        self.assertEqual(mcl.diagnostics["skipped_motion"], 1)

    def kidnap(self, **overrides):
        """Localiza al robot en un lugar y después le muestra scans de otro."""
        start = Pose2D(-2.0, -1.0, 0.0)
        mcl = self.filter(**overrides)
        mcl.initialize(start, std_xy=0.05, std_yaw=0.02)
        mcl.converge(room_scan(start))
        odom = Pose2D()
        mcl.process(room_scan(start), odom)
        elsewhere = Pose2D(2.0, -1.2, 2.0)
        for i in range(5):
            odom = odom.compose(Pose2D(0.1, 0.0, 0.0))
            mcl.process(room_scan(elsewhere, stamp=i + 1.0), odom)
        return mcl.diagnostics["injected_particles"]

    def test_injection_reacts_when_the_scan_stops_matching(self):
        self.assertGreater(self.kidnap(), 0)
        self.assertEqual(self.kidnap(injection_enabled=False), 0)

    def test_lost_when_the_estimate_does_not_explain_the_scan(self):
        # Una nube cerrada en una pose equivocada (acá, girada 90 grados) pasa por
        # convergida. Lo que la delata es que el scan no cae sobre las paredes.
        truth = Pose2D(-1.0, -0.5, 0.4)
        scan = room_scan(truth)
        mcl = self.filter()
        mcl.initialize(truth, 0.001, 0.001)
        self.assertFalse(mcl.lost(scan))
        mcl.initialize(Pose2D(truth.x, truth.y, truth.yaw + math.pi / 2), 0.001, 0.001)
        self.assertTrue(mcl.converged)
        self.assertTrue(mcl.lost(scan))

    def test_right_after_the_click_the_lost_check_is_stricter(self):
        # Justo después del click no debería haber obstáculos nuevos pegados al
        # robot, así que se tolera menos scan sin explicar que en marcha.
        truth = Pose2D(-1.0, -0.5, 0.4)
        scan = room_scan(truth)
        mcl = self.filter(lost_fraction=0.99, click_lost_fraction=0.15)
        mcl.initialize(Pose2D(truth.x, truth.y, truth.yaw + math.pi / 2), 0.001, 0.001)
        self.assertFalse(mcl.lost(scan))
        self.assertTrue(mcl.lost(scan, after_click=True))
        mcl.initialize(truth, 0.001, 0.001)
        self.assertFalse(mcl.lost(scan, after_click=True))

    def test_confirmed_obstacle_explains_returns_only_during_tracking(self):
        truth = Pose2D()
        scan = room_scan(truth, stamp=10.0)
        angles = np.arange(len(scan.ranges)) * scan.angle_increment
        box = _box_distance(0.0, 0.0, np.cos(angles), np.sin(angles), (0.55, 0.80), (-0.85, 0.85))
        scan = replace(scan, ranges=np.minimum(scan.ranges, box))
        mcl = self.filter()
        mcl.initialize(truth, 0.0, 0.0)
        layer = ObstacleLayer(self.grid, mcl.config.sensor, mcl.config.obstacles)
        for stamp in (9.6, 9.7, 9.8):
            layer.observe(truth, replace(scan, stamp=stamp))
        confirmed = ObstacleSnapshot(9.8, layer.points())

        self.assertTrue(mcl.lost(scan))
        self.assertFalse(mcl.lost(scan, obstacles=confirmed))
        self.assertGreater(mcl.diagnostics['static_unexplained_fraction'], 0.25)
        self.assertLess(mcl.diagnostics['unexplained_fraction'], 0.25)

        for stamp in (9.0, scan.stamp, 10.1):
            with self.subTest(stamp=stamp):
                self.assertTrue(mcl.lost(scan, obstacles=replace(confirmed, stamp=stamp)))
        self.assertTrue(mcl.lost(scan, after_click=True, obstacles=confirmed))

        require_more_static = self.filter(min_static_match_fraction=0.80)
        require_more_static.initialize(truth, 0.0, 0.0)
        self.assertTrue(require_more_static.lost(scan, obstacles=confirmed))

    def test_process_requires_an_initial_pose(self):
        with self.assertRaises(RuntimeError):
            self.filter().process(room_scan(Pose2D()), Pose2D())


class ConfigTests(unittest.TestCase):
    def test_unknown_group_and_bad_values_raise(self):
        with self.assertRaises(ValueError):
            NavConfig.from_dict({"mission": {}})
        with self.assertRaises(ValueError):
            NavConfig.from_dict({"localization": {"particles": 0}})
        with self.assertRaises(ValueError):
            NavConfig.from_dict({"localization": {"injection_alpha_slow": 0.5}})


if __name__ == "__main__":
    unittest.main()
