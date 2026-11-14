"""Planificador, seguidor y máquina de estados sobre un mapa sintético."""

from dataclasses import replace
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from slam.core.geometry import Pose2D, wrap_angle

from nav.core.config import NavConfig, NavigatorConfig, PlannerConfig
from nav.core.follower import PurePursuit
from nav.core.map import load_map
from nav.core.navigator import Navigator, State
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
