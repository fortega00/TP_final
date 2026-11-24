"""Ubicación del cono, filtro de Bayes y pose de aproximación. Sin ROS."""

import math
import unittest

import numpy as np

from slam.core.geometry import Pose2D
from nav.core.config import PlannerConfig
from nav.core.map import free_cells
from nav.core.planner import Costmap, plan

from mission.core.cone_locator import (ConeTracker, Intrinsics, approach_pose, blob_depth,
                                       camera_position_yaw, line_of_sight, measure)
from mission.core.config import LocatorConfig, MissionConfig
from mission.core.detector import Detection
from mission.core.exploration import main_component
from test_exploration import room

CAMERA = Intrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0, width=640)


def camera_pose(x, y, yaw, height=0.25):
    """map_T_camera de una cámara mirando hacia adelante del robot, a `height` m del piso.

    Frame óptico: z hacia adelante, x a la derecha, y hacia abajo.
    """
    base = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])   # columnas: x, y, z ópticos
    rotation = np.array([[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0, 0, 1]])
    transform = np.eye(4)
    transform[:3, :3] = rotation @ base
    transform[:3, 3] = (x, y, height)
    return transform


def detection(u, v, w=40, h=60):
    return Detection(center=(u, v), bottom=(u, v + h / 2), box=(int(u - w / 2), int(v - h / 2), w, h),
                     area=w * h, solidity=0.9, taper=0.5)


def depth_image(value=2.0):
    return np.full((480, 640), value, dtype=np.float32)


class MeasureTests(unittest.TestCase):
    def setUp(self):
        self.config = LocatorConfig()

    def test_straight_ahead(self):
        x, y = measure(detection(320, 240), depth_image(2.0), CAMERA, camera_pose(1.0, 1.0, 0.0), self.config)
        self.assertAlmostEqual(x, 3.0, places=3)
        self.assertAlmostEqual(y, 1.0, places=3)

    def test_to_the_right_of_the_image_is_to_the_right_of_the_robot(self):
        # 100 px a la derecha a 2 m: 100 * 2 / 500 = 0,4 m hacia -y del robot.
        x, y = measure(detection(420, 240), depth_image(2.0), CAMERA, camera_pose(0.0, 0.0, 0.0), self.config)
        self.assertAlmostEqual(x, 2.0, places=3)
        self.assertAlmostEqual(y, -0.4, places=3)

    def test_follows_the_robot_heading(self):
        x, y = measure(detection(320, 240), depth_image(2.0), CAMERA, camera_pose(0.0, 0.0, math.pi / 2), self.config)
        self.assertAlmostEqual(x, 0.0, places=3)
        self.assertAlmostEqual(y, 2.0, places=3)

    def test_wrong_real_size_is_rejected(self):
        # A 2 m, 40 x 60 px son 0,16 x 0,24 m: un cono. 6 x 8 px son 2 cm: no.
        pose = camera_pose(0.0, 0.0, 0.0)
        self.assertIsNotNone(measure(detection(320, 240), depth_image(2.0), CAMERA, pose, self.config))
        self.assertIsNone(measure(detection(320, 240, w=6, h=8), depth_image(2.0), CAMERA, pose, self.config))
        self.assertIsNone(measure(detection(320, 240, w=200, h=300), depth_image(2.0), CAMERA, pose, self.config))

    def test_invalid_depth(self):
        pose = camera_pose(0.0, 0.0, 0.0)
        for value in (0.0, np.nan, 9.0):
            self.assertIsNone(measure(detection(320, 240), depth_image(value), CAMERA, pose, self.config))

    def test_depth_uses_the_middle_of_the_blob(self):
        # Bordes con fondo lejano, centro con el cono: manda el centro.
        depth = depth_image(3.5)
        depth[225:255, 310:330] = 1.5
        self.assertAlmostEqual(blob_depth(depth, (300, 210, 40, 60), self.config), 1.5)

    def test_camera_position_and_heading(self):
        (x, y), yaw = camera_position_yaw(camera_pose(1.0, 2.0, 0.7))
        self.assertEqual((x, y), (1.0, 2.0))
        self.assertAlmostEqual(yaw, 0.7)

    def test_horizontal_field_of_view(self):
        self.assertAlmostEqual(CAMERA.horizontal_fov, 2 * math.atan(640 / 1000))


class TrackerTests(unittest.TestCase):
    def setUp(self):
        self.config = LocatorConfig()
        self.tracker = ConeTracker(self.config)
        self.fov = CAMERA.horizontal_fov

    def see(self, points, camera=(0.0, 0.0), yaw=0.0):
        self.tracker.update(points, camera, yaw, self.fov)

    def test_one_detection_is_only_a_candidate(self):
        self.see([(2.0, 0.0)])
        self.assertIsNone(self.tracker.confirmed())
        self.assertEqual(self.tracker.candidate(), (2.0, 0.0))

    def test_two_consistent_detections_confirm(self):
        # Previa 0,2; acierto 0,7 y falsa alarma 0,05: 0,78 con una, 0,98 con dos.
        self.see([(2.0, 0.0)])
        self.see([(2.1, 0.1)])
        x, y = self.tracker.confirmed()
        self.assertAlmostEqual(x, 2.05)
        self.assertAlmostEqual(y, 0.05)
        self.assertIsNone(self.tracker.candidate())

    def test_missing_it_while_looking_drops_it(self):
        self.see([(2.0, 0.0)])
        for _ in range(4):
            self.see([])                       # está enfrente, a 2 m, y no se ve
        self.assertEqual(self.tracker.tracks, [])

    def test_no_evidence_when_it_is_out_of_view(self):
        self.see([(2.0, 0.0)])
        for _ in range(10):
            self.see([], yaw=math.pi)          # mirando para el otro lado
        self.assertEqual(len(self.tracker.tracks), 1)
        self.see([], camera=(-5.0, 0.0))       # a 7 m: más lejos que max_depth
        self.assertEqual(len(self.tracker.tracks), 1)

    def test_no_evidence_behind_a_wall(self):
        grid = room(6.0, 4.0, walls=[(2.9, 0.4, 3.1, 3.6)])
        tracker = ConeTracker(self.config, blocked=~free_cells(grid), grid=grid)
        tracker.update([(4.0, 2.0)], (2.0, 2.0), 0.0, self.fov)    # visto (por un hueco que ya no se ve)
        for _ in range(10):
            tracker.update([], (2.0, 2.0), 0.0, self.fov)
        self.assertEqual(len(tracker.tracks), 1)

    def test_far_detections_are_different_cones(self):
        self.see([(2.0, 0.0), (2.0, 1.0)])
        self.assertEqual(len(self.tracker.tracks), 2)

    def test_a_false_alarm_does_not_confirm_a_cone(self):
        # Una detección suelta y después varias imágenes mirándolo sin verlo.
        self.see([(2.0, 0.0)])
        self.see([])
        self.see([(2.0, 0.0)])
        self.assertIsNone(self.tracker.confirmed())


class ApproachTests(unittest.TestCase):
    def setUp(self):
        self.config = LocatorConfig()

    def plan(self, grid, robot):
        costmap = Costmap(grid, PlannerConfig())
        reachable = main_component(costmap.traversable, costmap.cell(*robot))
        return costmap, reachable

    def test_open_room_stops_in_front_of_the_cone(self):
        grid = room(7.0, 5.0)
        costmap, reachable = self.plan(grid, (1.0, 2.5))
        cone = (4.0, 2.5)
        goal = approach_pose(cone, Pose2D(1.0, 2.5, 0.0), costmap, reachable, self.config)
        distance = math.hypot(goal.x - cone[0], goal.y - cone[1])
        # Con lugar libre alrededor, el anillo más cercano.
        self.assertAlmostEqual(distance, self.config.approach_distance, places=2)
        self.assertLess(goal.x, cone[0])                       # del lado del robot
        self.assertAlmostEqual(math.atan2(cone[1] - goal.y, cone[0] - goal.x), goal.yaw)
        row, col = costmap.cell(goal.x, goal.y)
        self.assertTrue(reachable[row, col])

    def test_cone_seen_through_a_hole_is_reached_around_the_wall(self):
        # Pared en x = 4,5 con un hueco de 15 cm en y = 1,5 (se ve el cono, no se pasa)
        # y una puerta de 1 m arriba. El cono está detrás del hueco.
        grid = room(9.0, 5.0, walls=[(4.45, 0.4, 4.55, 1.42), (4.45, 1.58, 4.55, 3.5)])
        costmap, reachable = self.plan(grid, (2.0, 1.5))
        cone = (4.9, 1.5)
        self.assertTrue(line_of_sight(~free_cells(grid), grid, (2.0, 1.5), cone))   # se ve por el hueco
        goal = approach_pose(cone, Pose2D(2.0, 1.5, 0.0), costmap, reachable, self.config)
        self.assertGreater(goal.x, 4.55)                       # del lado del cono, no contra la pared
        # El A* del navegador de la etapa B llega por la puerta (y > 3,5), no por el hueco.
        path = plan(costmap, (2.0, 1.5), (goal.x, goal.y))
        self.assertGreater(max(y for _, y in path), 3.5)

    def test_cone_in_a_narrow_aisle_is_approached_from_its_mouth(self):
        # Pasillo de 0,8 m de ancho y 1,4 m de fondo (como entre dos bancos), con
        # el cono adentro. Con la holgura de 0,5 m no se para adentro del pasillo.
        grid = room(7.0, 6.0, walls=[(2.9, 0.4, 3.0, 1.8), (3.8, 0.4, 3.9, 1.8)])
        costmap, reachable = self.plan(grid, (1.0, 4.5))
        cone = (3.45, 1.0)
        goal = approach_pose(cone, Pose2D(1.0, 4.5, 0.0), costmap, reachable, self.config, clearance=0.5)
        self.assertGreater(goal.y, 1.8)                        # afuera del pasillo
        self.assertLessEqual(math.hypot(goal.x - cone[0], goal.y - cone[1]), self.config.approach_max_distance + 0.01)
        # Sin holgura se mete en el pasillo, a la distancia mínima.
        inside = approach_pose(cone, Pose2D(1.0, 4.5, 0.0), costmap, reachable, self.config)
        self.assertLess(inside.y, 1.8)

    def test_unreachable_cone(self):
        # Cono en una sala cerrada sin puerta.
        grid = room(9.0, 5.0, walls=[(4.45, 0.4, 4.55, 4.6)])
        costmap, reachable = self.plan(grid, (2.0, 2.5))
        self.assertIsNone(approach_pose((6.5, 2.5), Pose2D(2.0, 2.5, 0.0), costmap, reachable, self.config))


class LocatorConfigTests(unittest.TestCase):
    def test_invalid_values_are_rejected(self):
        for bad in ({"min_depth": 5.0}, {"hit_rate": 1.0}, {"false_alarm_rate": 0.9},
                    {"prior": 0.99}, {"approach_distance": 2.0}, {"approach_angles": 2}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                MissionConfig.from_dict({"locator": bad})


if __name__ == "__main__":
    unittest.main()
