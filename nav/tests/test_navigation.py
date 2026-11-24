"""Planificador, seguidor y máquina de estados sobre un mapa sintético."""

from concurrent.futures import Future
from dataclasses import replace
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock

import numpy as np

from slam.core.geometry import Pose2D, wrap_angle
from slam.core.types import Scan

from nav.core.config import NavConfig, NavigatorConfig, PlannerConfig
from nav.core.follower import PurePursuit
from nav.core.map import load_map, occupied_cells
from nav.core.navigator import Navigator, State, safety_stop
from nav.core.obstacles import ObstacleLayer
from nav.core.planner import Costmap, PlanningError, plan

RESOLUTION = 0.05


def write_map(folder: Path) -> Path:
    """Dos salas de 4 x 4 m unidas por una puerta de 1 m, con borde desconocido.

    La pared divisoria está en x = 0 y la puerta entre y = 1 e y = 2, o sea
    corrida hacia arriba: el camino recto entre los centros de las salas choca
    contra la pared.
    """
    xs = np.arange(-4.5, 4.5, RESOLUTION) + RESOLUTION / 2
    ys = np.arange(-2.5, 2.5, RESOLUTION) + RESOLUTION / 2
    x, y = np.meshgrid(xs, ys)
    inside = (np.abs(x) < 4.0) & (np.abs(y) < 2.0)
    outer = (np.abs(x) < 4.0 + RESOLUTION) & (np.abs(y) < 2.0 + RESOLUTION) & ~inside
    divider = (np.abs(x) < RESOLUTION) & inside & ~((y > 1.0) & (y < 2.0))
    pixels = np.full(x.shape, 205, dtype=np.uint8)
    pixels[inside] = 254
    pixels[outer | divider] = 0
    (folder / "rooms.pgm").write_bytes(
        f"P5\n{x.shape[1]} {x.shape[0]}\n255\n".encode() + np.flipud(pixels).tobytes())
    (folder / "rooms.yaml").write_text(
        f"image: rooms.pgm\nmode: trinary\nresolution: {RESOLUTION}\norigin: [-4.5, -2.5, 0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    return folder / "rooms.yaml"


def path_length(path):
    return float(np.hypot(*np.diff(np.array(path), axis=0).T).sum())


class Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.folder = tempfile.TemporaryDirectory()
        cls.grid = load_map(write_map(Path(cls.folder.name)))
        cls.config = NavConfig()
        cls.costmap = Costmap(cls.grid, cls.config.planner)

    @classmethod
    def tearDownClass(cls):
        cls.folder.cleanup()


class PlannerTests(Base):
    def test_path_goes_through_the_door_and_keeps_the_hard_margin(self):
        path = plan(self.costmap, (-2.0, -1.0), (2.0, -1.0))
        points = np.array(path)
        self.assertEqual(path[-1], (2.0, -1.0))
        # Cruza la pared divisoria por la puerta (y entre 1 y 2).
        crossing = points[np.argmin(np.abs(points[:, 0]))]
        self.assertTrue(1.0 < crossing[1] < 2.0)
        # Ningún punto queda a menos de radio + margen de un obstáculo.
        limit = self.config.planner.robot_radius + self.config.planner.safety_margin
        for x, y in path[:-1]:
            self.assertGreater(self.costmap.clearance[self.costmap.cell(x, y)], limit)

    def test_clearance_cost_centres_the_path_in_the_door(self):
        def door_offset(weight):
            costmap = Costmap(self.grid, replace(PlannerConfig(), clearance_weight=weight))
            points = np.array(plan(costmap, (-2.0, -1.0), (2.0, -1.0)))
            return abs(points[np.argmin(np.abs(points[:, 0]))][1] - 1.5)
        # Sin costo de cercanía el camino más corto roza el marco de abajo de la
        # puerta. Con costo pasa cerca del centro (y = 1,5).
        self.assertGreater(door_offset(0.0), 0.2)
        self.assertLess(door_offset(5.0), 0.1)

    def test_unknown_space_is_not_traversable(self):
        # Celda desconocida a 35 cm de la pared más cercana: más que la inflación,
        # así que solo deja de ser transitable por ser desconocida.
        self.assertFalse(self.costmap.traversable[self.costmap.cell(4.4, 0.0)])
        self.assertTrue(self.costmap.traversable[self.costmap.cell(2.0, 0.0)])

    def test_goal_in_wall_unknown_or_outside_is_rejected(self):
        for goal in ((0.0, 0.0), (4.3, 0.0), (9.0, 0.0), (3.9, 0.0)):
            with self.assertRaises(PlanningError):
                plan(self.costmap, (-2.0, -1.0), goal)

    def test_closed_room_has_no_path(self):
        grid = load_map(write_map(Path(self.folder.name)))
        col, _ = grid.world_to_grid(0.0, 0.0)
        grid.log_odds[:, col - 1:col + 2] = 4.0          # cierra la puerta
        with self.assertRaises(PlanningError):
            plan(Costmap(grid, self.config.planner), (-2.0, -1.0), (2.0, -1.0))

    def test_start_next_to_a_wall_snaps_to_free_space(self):
        # A 10 cm de la pared de abajo: dentro de la zona inflada.
        path = plan(self.costmap, (-2.0, -1.9), (-2.0, 0.0))
        self.assertLess(math.hypot(path[0][0] + 2.0, path[0][1] + 1.9), 0.30)
        self.assertEqual(path[-1], (-2.0, 0.0))


def simulate(navigator, pose, seconds, dt=0.05, start=0.0, blocked=lambda t: False,
             localization=lambda t: "LOCALIZED", on_step=None):
    """Mueve un robot ideal con las velocidades que pide el navegador."""
    t = start
    for _ in range(int(seconds / dt)):
        v, w = navigator.update(t, pose, localization(t), blocked(t))
        pose = Pose2D(pose.x + v * math.cos(pose.yaw) * dt, pose.y + v * math.sin(pose.yaw) * dt,
                      wrap_angle(pose.yaw + w * dt))
        t += dt
        if on_step is not None:
            on_step(t, pose, v, w)
    return pose, t


class NavigatorTests(Base):
    def navigator(self):
        return Navigator(self.costmap, self.config)

    def test_reaches_goal_position_and_angle_without_touching_walls(self):
        nav = self.navigator()
        goal = Pose2D(2.0, -1.0, math.pi / 2)
        start = Pose2D(-2.0, -1.0, math.pi)       # arranca mirando para el otro lado
        nav.update(0.0, start, "LOCALIZED", False)
        nav.set_goal(goal, 0.0)
        closest = []
        limits = self.config.follower

        def check(t, pose, v, w):
            closest.append(self.costmap.clearance[self.costmap.cell(pose.x, pose.y)])
            self.assertLessEqual(abs(v), limits.max_linear + 1e-9)
            self.assertLessEqual(abs(w), limits.max_angular + 1e-9)

        pose, _ = simulate(nav, start, 120.0, on_step=check)
        self.assertEqual(nav.state, State.IDLE)
        self.assertLess(math.hypot(pose.x - goal.x, pose.y - goal.y), limits.goal_tolerance_xy)
        self.assertLess(abs(wrap_angle(pose.yaw - goal.yaw)), limits.goal_tolerance_yaw)
        self.assertGreater(min(closest), self.config.planner.robot_radius)

    def test_goal_on_the_robot_only_turns_in_place(self):
        # Objetivo en la misma posición con otro ángulo: girar en el lugar. Antes
        # pasaba por ALIGNING hacia un camino de largo cero, cuya dirección sale de
        # milímetros de error, y el robot giraba hacia cualquier lado primero.
        nav = self.navigator()
        start = Pose2D(-2.0, -1.0, 0.0)
        nav.update(0.0, start, "LOCALIZED", False)
        nav.set_goal(Pose2D(-1.99, -1.01, math.pi / 2), 0.0)
        states, turned = set(), [0.0]

        def check(t, pose, v, w):
            states.add(nav.state)
            turned[0] += abs(w) * 0.05
            self.assertEqual(v, 0.0)

        pose, _ = simulate(nav, start, 30.0, on_step=check)
        self.assertEqual(nav.state, State.IDLE)
        self.assertNotIn(State.ALIGNING, states)
        self.assertNotIn(State.FOLLOWING, states)
        self.assertLess(abs(wrap_angle(pose.yaw - math.pi / 2)), self.config.follower.goal_tolerance_yaw)
        self.assertLess(turned[0], math.pi / 2 + 0.2)        # un solo giro de 90 grados

    def test_goal_before_localization_is_stored(self):
        nav = self.navigator()
        nav.set_goal(Pose2D(-1.0, 0.0, 0.0), 0.0)
        self.assertEqual(nav.update(0.0, None, "WAITING_INITIAL_POSE", False), (0.0, 0.0))
        self.assertEqual(nav.state, State.WAIT_INITIAL_POSE)
        nav.update(0.1, Pose2D(-2.0, -1.0, 0.0), "NOT_CONVERGED", False)
        self.assertEqual(nav.state, State.LOCALIZING)
        nav.update(0.2, Pose2D(-2.0, -1.0, 0.0), "LOCALIZED", False)
        self.assertEqual(nav.state, State.PLANNING)

    def test_new_goal_while_moving_replans(self):
        nav = self.navigator()
        start = Pose2D(-2.0, -1.0, 0.0)
        nav.update(0.0, start, "LOCALIZED", False)
        nav.set_goal(Pose2D(2.0, -1.0, 0.0), 0.0)
        pose, t = simulate(nav, start, 8.0)
        self.assertEqual(nav.state, State.FOLLOWING)
        second = Pose2D(-3.0, 1.0, 0.0)
        nav.set_goal(second, t)
        self.assertEqual(nav.state, State.PLANNING)
        pose, _ = simulate(nav, pose, 90.0, start=t)
        self.assertEqual(nav.state, State.IDLE)
        self.assertLess(math.hypot(pose.x - second.x, pose.y - second.y), 0.08)

    def test_waits_for_a_background_plan_with_zero_velocity(self):
        result = Future()
        executor = Mock()
        executor.submit.return_value = result
        nav = Navigator(self.costmap, self.config, planning_executor=executor)
        pose = Pose2D(-2, -1, 0)
        goal = Pose2D(-1, -1, 0)
        nav.update(0.0, pose, "LOCALIZED", False)
        nav.set_goal(goal, 0.0)

        for now in (0.0, 0.2, 0.4, 0.6, 1.0):
            self.assertEqual(nav.update(now, pose, "LOCALIZED", False), (0.0, 0.0))
            self.assertEqual(nav.state, State.PLANNING)
        executor.submit.assert_called_once()
        calculate, *args = executor.submit.call_args.args
        result.set_result(calculate(*args))

        self.assertEqual(nav.update(1.1, pose, "LOCALIZED", False), (0.0, 0.0))
        self.assertEqual(nav.state, State.ALIGNING)
        self.assertEqual(nav.path[-1], (goal.x, goal.y))

    def test_ignores_a_running_plan_when_the_goal_is_canceled_or_replaced(self):
        for lose_scan in (True, False):
            with self.subTest(lose_scan=lose_scan):
                old_result, new_result = Future(), Future()
                executor = Mock()
                executor.submit.side_effect = [old_result, new_result]
                nav = Navigator(self.costmap, self.config, planning_executor=executor)
                pose = Pose2D(-2, -1, 0)
                nav.update(0.0, pose, "LOCALIZED", False)
                nav.set_goal(Pose2D(-1, -1, 0), 0.0)
                nav.update(0.0, pose, "LOCALIZED", False)
                old_result.set_running_or_notify_cancel()
                if lose_scan:
                    nav.wait_for_scan(0.5)
                else:
                    nav.set_goal(Pose2D(-3, -1, 0), 0.5)
                old_result.set_result([(-2, -1), (-1, -1)])

                self.assertEqual(nav.update(0.6, pose, "LOCALIZED", False), (0.0, 0.0))
                self.assertEqual(nav.path, [])
                if lose_scan:
                    self.assertEqual(nav.state, State.WAIT_SCAN)
                    self.assertIsNone(nav.goal)
                else:
                    self.assertEqual(nav.state, State.PLANNING)
                    new_result.set_result([(-2, -1), (-3, -1)])
                    nav.update(0.7, pose, "LOCALIZED", False)
                    self.assertEqual(nav.path[-1], (-3, -1))

    def test_lidar_loss_cancels_a_moving_goal_even_when_localized(self):
        goal = Pose2D(-1.0, -1.0, 0.0)
        for yaw, motion in ((0.0, 0), (math.pi, 1)):
            with self.subTest(yaw=yaw):
                navigator = self.navigator()
                pose = Pose2D(-2.0, -1.0, yaw)
                navigator.set_goal(goal, 0.0)
                for t in (0.0, 0.1, 0.2, 0.3):
                    command = navigator.update(t, pose, "LOCALIZED", False)
                self.assertNotEqual(command[motion], 0.0)
                self.assertTrue(navigator.path)

                navigator.wait_for_scan(0.4)
                self.assertEqual(navigator.update(0.4, pose, "LOCALIZED", False), (0.0, 0.0))
                self.assertEqual(navigator.state, State.WAIT_SCAN)
                self.assertIsNone(navigator.goal)
                self.assertEqual(navigator.path, [])

                navigator.scan_available(0.5)
                self.assertEqual(navigator.update(0.5, pose, "LOCALIZED", False), (0.0, 0.0))
                self.assertEqual(navigator.state, State.IDLE)
                navigator.set_goal(goal, 0.6)
                self.assertEqual(navigator.state, State.PLANNING)

    def test_unreachable_goal_is_rejected_and_robot_stays(self):
        nav = self.navigator()
        start = Pose2D(-2.0, -1.0, 0.0)
        nav.update(0.0, start, "LOCALIZED", False)
        nav.set_goal(Pose2D(0.0, 0.0, 0.0), 0.0)          # adentro de la pared
        pose, _ = simulate(nav, start, 2.0)
        self.assertEqual(nav.state, State.IDLE)
        self.assertEqual((pose.x, pose.y), (start.x, start.y))

    def test_obstacle_ahead_stops_and_resumes(self):
        nav = self.navigator()
        start = Pose2D(-2.0, -1.0, 0.0)
        nav.update(0.0, start, "LOCALIZED", False)
        nav.set_goal(Pose2D(2.0, -1.0, 0.0), 0.0)
        speeds = []
        # Algo aparece delante entre los 8 y los 12 segundos.
        pose, t = simulate(nav, start, 12.0, blocked=lambda t: 8.0 <= t < 12.0,
                           on_step=lambda t, p, v, w: speeds.append((t, v, w)))
        self.assertEqual(nav.state, State.BLOCKED)
        self.assertTrue(all(v == 0 and w == 0 for time, v, w in speeds if time > 8.2))
        pose, _ = simulate(nav, pose, 110.0, start=t)
        self.assertEqual(nav.state, State.IDLE)
        self.assertLess(math.hypot(pose.x - 2.0, pose.y + 1.0), 0.08)

    def test_obstacle_that_never_clears_drops_the_goal(self):
        config = replace(self.config, navigator=replace(NavigatorConfig(), blocked_timeout=5.0))
        nav = Navigator(self.costmap, config)
        start = Pose2D(-2.0, -1.0, 0.0)
        nav.update(0.0, start, "LOCALIZED", False)
        nav.set_goal(Pose2D(2.0, -1.0, 0.0), 0.0)
        simulate(nav, start, 20.0, blocked=lambda t: t >= 8.0)
        self.assertEqual(nav.state, State.IDLE)
        self.assertIsNone(nav.goal)

    def test_losing_localization_stops_then_replans(self):
        nav = self.navigator()
        start = Pose2D(-2.0, -1.0, 0.0)
        nav.update(0.0, start, "LOCALIZED", False)
        nav.set_goal(Pose2D(2.0, -1.0, 0.0), 0.0)
        speeds = []
        status = lambda t: "NOT_CONVERGED" if 8.0 <= t < 11.0 else "LOCALIZED"
        pose, t = simulate(nav, start, 10.0, localization=status,
                           on_step=lambda t, p, v, w: speeds.append((t, v, w)))
        self.assertEqual(nav.state, State.LOST)
        self.assertTrue(all(v == 0 and w == 0 for time, v, w in speeds if time > 8.2))
        pose, _ = simulate(nav, pose, 110.0, start=t, localization=status)
        self.assertEqual(nav.state, State.IDLE)
        self.assertLess(math.hypot(pose.x - 2.0, pose.y + 1.0), 0.08)

    def test_localization_lost_for_too_long_asks_for_a_new_initial_pose(self):
        nav = self.navigator()
        start = Pose2D(-2.0, -1.0, 0.0)
        nav.update(0.0, start, "LOCALIZED", False)
        nav.set_goal(Pose2D(2.0, -1.0, 0.0), 0.0)
        simulate(nav, start, 25.0, localization=lambda t: "NOT_CONVERGED" if t >= 8.0 else "LOCALIZED")
        self.assertEqual(nav.state, State.WAIT_INITIAL_POSE)

    def test_new_initial_pose_while_moving_keeps_the_goal(self):
        nav = self.navigator()
        start = Pose2D(-2.0, -1.0, 0.0)
        nav.update(0.0, start, "LOCALIZED", False)
        goal = Pose2D(2.0, -1.0, 0.0)
        nav.set_goal(goal, 0.0)
        pose, t = simulate(nav, start, 8.0)
        nav.on_reinitialized(t)
        self.assertEqual(nav.state, State.LOCALIZING)
        self.assertEqual(nav.update(t, pose, "LOCALIZED", False), (0.0, 0.0))
        self.assertEqual(nav.state, State.PLANNING)
        pose, _ = simulate(nav, pose, 110.0, start=t)
        self.assertLess(math.hypot(pose.x - goal.x, pose.y - goal.y), 0.08)


def lidar(grid, pose, discs, stamp, beams=180, max_range=8.0):
    """Scan que mediría un lidar en `pose`: paredes del mapa más discos (x, y, radio)."""
    angles = np.arange(beams) * (2 * math.pi / beams)
    steps = np.arange(0.05, max_range, RESOLUTION / 2)
    x = pose.x + steps * np.cos(pose.yaw + angles)[:, None]
    y = pose.y + steps * np.sin(pose.yaw + angles)[:, None]
    cols, rows = grid.world_to_grid(x, y)
    inside = (cols >= 0) & (cols < grid.width) & (rows >= 0) & (rows < grid.height)
    hit = ~inside
    hit[inside] = occupied_cells(grid)[rows[inside], cols[inside]]
    for cx, cy, radius in discs:
        hit |= np.hypot(x - cx, y - cy) < radius
    ranges = np.where(hit.any(axis=1), steps[hit.argmax(axis=1)], np.inf)
    return Scan(ranges, 0.0, 2 * math.pi / beams, 0.1, max_range, stamp)


class ObstacleTests(Base):
    """Obstáculos que no están en el mapa: el robot solo los conoce por el lidar."""

    def drive(self, nav, pose, seconds, discs, dt=0.05):
        """Robot ideal con lidar. `discs(t)` da los obstáculos presentes en cada instante.

        Devuelve la pose final y la menor distancia entre el centro del robot y
        el borde de un obstáculo.
        """
        closest, t = math.inf, 0.0
        for step in range(int(seconds / dt)):
            present = discs(t)
            scan = lidar(self.grid, pose, present, t)
            if step % 3 == 0:                      # el lidar real da unos 7 scans por segundo
                nav.observe(pose, scan)
            v, w = nav.update(t, pose, "LOCALIZED", safety_stop(scan, self.config))
            pose = Pose2D(pose.x + v * math.cos(pose.yaw) * dt, pose.y + v * math.sin(pose.yaw) * dt,
                          wrap_angle(pose.yaw + w * dt))
            t += dt
            for cx, cy, radius in present:
                closest = min(closest, math.hypot(pose.x - cx, pose.y - cy) - radius)
        return pose, closest

    def start(self, start, goal, **navigator):
        config = replace(self.config, navigator=replace(NavigatorConfig(), **navigator))
        nav = Navigator(self.costmap, config)
        nav.update(0.0, start, "LOCALIZED", False)
        nav.set_goal(goal, 0.0)
        return nav

    def test_walls_seen_with_a_pose_error_are_not_new_obstacles(self):
        # La pose estimada tiene 10 cm y 2 grados de error: las paredes caen corridas.
        truth = Pose2D(-2.0, 0.0, 0.3)
        estimate = Pose2D(-1.9, 0.05, 0.3 + math.radians(2))
        layer = ObstacleLayer(self.grid, self.config.sensor, self.config.obstacles)
        for i in range(10):
            layer.observe(estimate, lidar(self.grid, truth, [], float(i)))
        self.assertFalse(layer.cells().any())
        # Con una caja a 1 m, sí aparece, y aparece donde la ve la pose estimada.
        for i in range(10):
            layer.observe(estimate, lidar(self.grid, truth, [(-1.0, 0.0, 0.15)], 10.0 + i))
        rows, cols = np.nonzero(layer.cells())
        self.assertGreater(len(rows), 0)
        x, y = self.costmap.point(rows, cols)
        self.assertLess(np.hypot(x + 1.0, y).max(), 0.15 + 0.20)

    def test_a_new_initial_pose_forgets_the_obstacles(self):
        nav = Navigator(self.costmap, self.config)
        pose = Pose2D(-2.0, 0.0, 0.0)
        for i in range(10):
            nav.observe(pose, lidar(self.grid, pose, [(-1.0, 0.0, 0.15)], float(i)))
        self.assertTrue(nav.obstacles.cells().any())
        nav.on_reinitialized(10.0)
        self.assertFalse(nav.obstacles.cells().any())

    def test_obstacle_on_the_path_is_avoided(self):
        # Una caja de 30 cm en medio del camino recto, con lugar para rodearla.
        start, goal = Pose2D(-3.5, -1.0, 0.0), Pose2D(-0.6, -1.0, 0.0)
        nav = self.start(start, goal)
        pose, closest = self.drive(nav, start, 120.0, lambda t: [(-2.0, -1.0, 0.15)])
        self.assertEqual(nav.state, State.IDLE)
        self.assertLess(math.hypot(pose.x - goal.x, pose.y - goal.y), self.config.follower.goal_tolerance_xy)
        self.assertGreater(closest, self.config.planner.robot_radius)
        self.assertTrue(any("goal reached" in text for _, text in nav.events))

    def test_without_obstacles_the_walls_do_not_block_the_path(self):
        start, goal = Pose2D(-2.0, -1.0, 0.0), Pose2D(2.0, -1.0, 0.0)
        nav = self.start(start, goal)
        self.drive(nav, start, 120.0, lambda t: [])
        self.assertEqual(nav.state, State.IDLE)
        self.assertEqual(sum("path blocked" in text for _, text in nav.events), 0)
        self.assertTrue(any("goal reached" in text for _, text in nav.events))

    def test_obstacle_that_closes_the_only_door_drops_the_goal(self):
        start, goal = Pose2D(-2.0, 1.5, 0.0), Pose2D(2.0, 1.5, 0.0)
        nav = self.start(start, goal, blocked_timeout=10.0)
        pose, closest = self.drive(nav, start, 60.0, lambda t: [(0.0, 1.5, 0.5)])
        self.assertEqual(nav.state, State.IDLE)
        self.assertIsNone(nav.goal)
        self.assertLess(pose.x, 0.0)
        self.assertGreater(closest, self.config.planner.robot_radius)
        self.assertTrue(any("did not clear" in text for _, text in nav.events))

    def test_obstacle_that_leaves_is_forgotten(self):
        # La puerta está tapada los primeros 20 segundos y después se despeja.
        start, goal = Pose2D(-2.0, 1.5, 0.0), Pose2D(2.0, 1.5, 0.0)
        nav = self.start(start, goal, blocked_timeout=60.0)
        pose, _ = self.drive(nav, start, 150.0, lambda t: [(0.0, 1.5, 0.5)] if t < 20.0 else [])
        self.assertEqual(nav.state, State.IDLE)
        self.assertLess(math.hypot(pose.x - goal.x, pose.y - goal.y), self.config.follower.goal_tolerance_xy)


class CafeSafetyTests(Base):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.cafe = NavConfig.load_yaml(Path(__file__).resolve().parents[1] / "config" / "cafe.yaml")
        # Extrínseca del Café registrada en hitl_cafe_01/report/D_analysis.log.
        cls.mount = Pose2D(0.043, 0.0, math.pi)

    def scan(self, distance, angle=-math.pi):
        return Scan(np.array([distance]), angle, 0.01, 0.1, 8.0, 0.0, self.mount)

    def test_cafe_stops_before_front_returns_enter_the_chassis_filter(self):
        # 0,26 m del láser son 0,303 m del centro. Debe frenar mientras el
        # retorno todavía supera min_range, antes de entrar en la zona ciega.
        for distance in (0.26, 0.30, 0.35):
            with self.subTest(distance=distance):
                self.assertTrue(safety_stop(self.scan(distance), self.cafe))

    def test_cafe_still_rejects_chassis_and_invalid_returns(self):
        for distance in (0.14, 0.18, 0.20, 0.25, 0.0, -1.0, np.nan, np.inf):
            with self.subTest(distance=distance):
                self.assertFalse(safety_stop(self.scan(distance), self.cafe))

    def test_cafe_only_stops_for_returns_inside_the_front_rectangle(self):
        # Fuera por delante, por detrás y por el costado, respectivamente.
        for distance, angle in ((0.50, -math.pi), (0.30, 0.0), (0.30, -math.pi / 2)):
            with self.subTest(distance=distance, angle=angle):
                self.assertFalse(safety_stop(self.scan(distance, angle), self.cafe))

    def test_cafe_stops_without_waiting_for_obstacle_confirmation(self):
        nav = Navigator(Costmap(self.grid, self.cafe.planner), self.cafe)
        pose = Pose2D(-2.0, -1.0, 0.0)
        nav.update(0.0, pose, "LOCALIZED", False)
        nav.set_goal(Pose2D(-1.0, -1.0, 0.0), 0.0)
        nav.update(0.1, pose, "LOCALIZED", False)
        nav.update(0.2, pose, "LOCALIZED", False)
        self.assertEqual(nav.state, State.FOLLOWING)
        self.assertFalse(nav.obstacles.cells().any())
        command = nav.update(0.3, pose, "LOCALIZED", safety_stop(self.scan(0.26), self.cafe))
        self.assertEqual(command, (0.0, 0.0))
        self.assertEqual(nav.state, State.BLOCKED)


class FollowerTests(Base):
    def test_straight_path_ahead_gives_no_turn(self):
        follower = PurePursuit([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)], self.config.follower)
        v, w = follower.command(Pose2D(0.0, 0.0, 0.0))
        self.assertAlmostEqual(v, self.config.follower.max_linear)
        self.assertAlmostEqual(w, 0.0)

    def test_path_to_the_left_turns_left(self):
        follower = PurePursuit([(0.0, 0.0), (0.5, 0.5), (1.0, 1.0)], self.config.follower)
        self.assertGreater(follower.command(Pose2D(0.0, 0.0, 0.0))[1], 0.0)


if __name__ == "__main__":
    unittest.main()
