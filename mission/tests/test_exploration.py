"""Exploración por puntos de observación, sobre mapas sintéticos y el mapa del aula."""

import math
from pathlib import Path
import time
import unittest

import numpy as np

from slam.core.config import MapConfig
from slam.core.grid import OccupancyGrid
from nav.core.config import NavConfig, PlannerConfig
from nav.core.map import free_cells
from nav.core.planner import Costmap

from mission.core.config import ExplorationConfig, MissionConfig
from mission.core.exploration import (CoverageTracker, ExplorationError, Visibility, clutter_blocks,
                                      main_component, order_tour, path_distances, plan_viewpoints,
                                      prepare_map, search_targets, tour_length, wall_cells)
from nav.core.map import occupied_cells

RESOLUTION = 0.05
CLASSROOM = Path(__file__).resolve().parents[2] / "sesiones/20261116/mapa/intento3_corregido/map.yaml"


def make_grid(width, height, free, walls=()):
    """Grilla de width x height m con origen en (0, 0).

    `free(x, y)` dice qué celdas son libres (el resto es desconocido) y cada
    elemento de `walls` es un rectángulo (x0, y0, x1, y1) ocupado.
    """
    grid = OccupancyGrid(MapConfig(resolution=RESOLUTION, width=width - RESOLUTION / 2,
                                   height=height - RESOLUTION / 2, origin_x=0.0, origin_y=0.0))
    xs = (np.arange(grid.width) + 0.5) * RESOLUTION
    ys = (np.arange(grid.height) + 0.5) * RESOLUTION
    x, y = np.meshgrid(xs, ys)
    is_free = free(x, y)
    occupied = np.zeros_like(is_free)
    for x0, y0, x1, y1 in walls:
        occupied |= (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
    grid.log_odds[is_free] = -4.0
    grid.log_odds[occupied] = 4.0
    grid.observed[:] = is_free | occupied
    return grid


def room(width, height, margin=0.5, walls=()):
    """Sala rectangular rodeada de pared, con borde exterior desconocido."""
    inside = lambda x, y: (x > margin) & (x < width - margin) & (y > margin) & (y < height - margin)
    border = [(margin - 0.1, margin - 0.1, width - margin + 0.1, margin),
              (margin - 0.1, height - margin, width - margin + 0.1, height - margin + 0.1),
              (margin - 0.1, margin - 0.1, margin, height - margin + 0.1),
              (width - margin, margin - 0.1, width - margin + 0.1, height - margin + 0.1)]
    return make_grid(width, height, inside, border + list(walls))


def two_rooms():
    """Dos salas de 4 x 4 m unidas por una puerta de 1 m en x = 4,5."""
    return room(9.0, 5.0, walls=[(4.45, 0.4, 4.55, 3.0), (4.45, 4.0, 4.55, 4.6)])


def cell_of(grid, x, y):
    col, row = grid.world_to_grid(x, y)
    return row * grid.width + col


class VisibilityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.grid = two_rooms()
        cls.config = ExplorationConfig(view_range=3.0, view_min_range=0.3)
        cls.visibility = Visibility(cls.grid, cls.config)

    def test_sees_free_cells_in_range_and_not_beyond(self):
        seen = set(self.visibility.visible(2.5, 2.5).tolist())
        self.assertIn(cell_of(self.grid, 4.0, 2.5), seen)     # 1,5 m
        self.assertIn(cell_of(self.grid, 1.0, 1.0), seen)     # esquina, 2,1 m
        self.assertNotIn(cell_of(self.grid, 2.6, 2.5), seen)  # 0,1 m: más cerca que el mínimo

    def test_walls_block_the_view(self):
        seen = set(self.visibility.visible(3.5, 1.5).tolist())
        # Del otro lado de la pared divisoria, a 2 m pero tapado.
        self.assertNotIn(cell_of(self.grid, 5.5, 1.5), seen)
        # Por la puerta sí se ve.
        through_door = set(self.visibility.visible(3.5, 3.5).tolist())
        self.assertIn(cell_of(self.grid, 5.5, 3.5), through_door)

    def test_range_limit(self):
        seen = set(self.visibility.visible(1.0, 2.5).tolist())
        self.assertIn(cell_of(self.grid, 3.9, 2.5), seen)
        self.assertNotIn(cell_of(self.grid, 4.2, 2.5), seen)

    def test_field_of_view_wedge(self):
        ahead = set(self.visibility.visible(2.5, 2.5, yaw=0.0).tolist())
        self.assertIn(cell_of(self.grid, 4.0, 2.5), ahead)
        self.assertNotIn(cell_of(self.grid, 1.0, 2.5), ahead)   # atrás
        self.assertNotIn(cell_of(self.grid, 2.5, 4.0), ahead)   # al costado, 90 grados

    def test_never_sees_occupied_or_unknown(self):
        seen = self.visibility.visible(2.5, 2.5)
        self.assertTrue(free_cells(self.grid).ravel()[seen].all())


class PlanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.grid = two_rooms()
        cls.costmap = Costmap(cls.grid, PlannerConfig())
        cls.config = ExplorationConfig(coverage_target=0.95)
        cls.plan = plan_viewpoints(cls.grid, cls.costmap, cls.config, start=(1.5, 1.5))

    def test_covers_both_rooms(self):
        self.assertGreaterEqual(self.plan.coverage, 0.95)
        xs = [x for x, _ in self.plan.viewpoints]
        self.assertTrue(any(x < 4.45 for x in xs) and any(x > 4.55 for x in xs))

    def test_viewpoints_are_traversable(self):
        for x, y in self.plan.viewpoints:
            row, col = self.costmap.cell(x, y)
            self.assertTrue(self.costmap.traversable[row, col])

    def test_few_points_for_two_small_rooms(self):
        self.assertLessEqual(len(self.plan.viewpoints), 6)

    def test_tour_starts_near_the_start(self):
        first = self.plan.viewpoints[0]
        self.assertLess(first[0], 4.45)

    def test_deterministic(self):
        again = plan_viewpoints(self.grid, self.costmap, self.config, start=(1.5, 1.5))
        self.assertEqual(again.viewpoints, self.plan.viewpoints)

    def test_one_point_is_enough_for_a_small_open_room(self):
        # Sala de 2 x 2 m: un punto ve todo menos el disco de 0,3 m donde la
        # profundidad no mide (7 %). Otro punto agregaría menos que min_gain.
        small = room(3.0, 3.0)
        plan = plan_viewpoints(small, Costmap(small, PlannerConfig()), self.config, start=(1.5, 1.5))
        self.assertEqual(len(plan.viewpoints), 1)
        self.assertGreater(plan.coverage, 0.90)

    def test_targets_skip_isolated_free_patches(self):
        # Una mancha libre suelta, separada de la sala por zona desconocida.
        grid = make_grid(6.0, 4.0, lambda x, y: ((x > 0.5) & (x < 3.5) & (y > 0.5) & (y < 3.5))
                         | ((x > 4.5) & (x < 5.5) & (y > 1.5) & (y < 2.5)))
        costmap = Costmap(grid, PlannerConfig())
        plan = plan_viewpoints(grid, costmap, self.config, start=(2.0, 2.0))
        self.assertFalse(plan.targets.ravel()[cell_of(grid, 5.0, 2.0)])
        self.assertTrue(plan.targets.ravel()[cell_of(grid, 2.0, 2.0)])

    def test_start_far_from_free_space_is_rejected(self):
        with self.assertRaises(ExplorationError):
            plan_viewpoints(self.grid, self.costmap, self.config, start=(0.1, 0.1))


class ManualPlanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.grid = two_rooms()
        cls.costmap = Costmap(cls.grid, PlannerConfig())

    def test_uses_given_points_in_order(self):
        config = ExplorationConfig(viewpoints=((6.5, 2.5), (2.5, 2.5)))
        plan = plan_viewpoints(self.grid, self.costmap, config, start=(1.5, 1.5))
        self.assertTrue(plan.manual)
        self.assertEqual(plan.viewpoints, [(6.5, 2.5), (2.5, 2.5)])
        self.assertGreater(plan.coverage, 0.8)

    def test_point_near_a_wall_is_moved_to_free_space(self):
        config = ExplorationConfig(viewpoints=((2.5, 0.65),))
        x, y = plan_viewpoints(self.grid, self.costmap, config).viewpoints[0]
        row, col = self.costmap.cell(x, y)
        self.assertTrue(self.costmap.traversable[row, col])
        self.assertLess(math.hypot(x - 2.5, y - 0.65), 0.3)

    def test_point_outside_the_room_is_rejected(self):
        config = ExplorationConfig(viewpoints=((0.1, 0.1),))
        with self.assertRaises(ExplorationError):
            plan_viewpoints(self.grid, self.costmap, config)


def table_legs(x0, y0, x1, y1, step=0.5, size=0.05):
    """Patas de 5 cm cada `step` metros en un rectángulo: lo que el lidar ve de unas mesas."""
    legs = []
    for x in np.arange(x0, x1 + 1e-9, step):
        for y in np.arange(y0, y1 + 1e-9, step):
            legs.append((x, y, x + size, y + size))
    return legs


class ClutterTests(unittest.TestCase):
    """Detección automática de mesas y bancos a partir de las patas del mapa."""

    @classmethod
    def setUpClass(cls):
        # Sala de 8 x 6 m (paredes en x = 0,5 y x = 7,5) con un grupo de mesas
        # (patas cada 0,5 m) en el medio, una pata suelta, una pared interna
        # con una puerta de 1,4 m y otro grupo de patas a 0,7 m de la pared
        # izquierda: el pasillo entre ese grupo y la pared no se tiene que tapar.
        legs = table_legs(3.0, 2.0, 5.0, 3.5) + table_legs(1.2, 4.0, 2.2, 5.0)
        door_wall = [(6.45, 0.4, 6.55, 1.9), (6.45, 3.3, 6.55, 5.6)]
        cls.grid = room(8.0, 6.0, walls=legs + [(1.5, 1.0, 1.55, 1.05)] + door_wall)
        cls.config = ExplorationConfig(clutter_radius=0.6, viewpoint_clearance=0.5, coverage_target=0.95)
        cls.marked, cls.costmap = prepare_map(cls.grid, PlannerConfig(), cls.config)

    def occupied_at(self, x, y):
        return bool(occupied_cells(self.marked).ravel()[cell_of(self.marked, x, y)])

    def test_legs_merge_into_a_block(self):
        # Entre patas, en el medio del grupo: el lidar lo ve libre, la detección lo cierra.
        self.assertFalse(bool(occupied_cells(self.grid).ravel()[cell_of(self.grid, 4.25, 2.75)]))
        self.assertTrue(self.occupied_at(4.25, 2.75))

    def test_open_floor_and_isolated_leg_stay_as_they_are(self):
        self.assertFalse(self.occupied_at(1.5, 2.5))
        self.assertFalse(self.occupied_at(1.7, 1.0))   # al lado de la pata suelta

    def test_corridor_between_furniture_and_wall_stays_open(self):
        # El grupo de la izquierda está a 0,7 m de la pared: con radio 0,6 se
        # uniría a ella si la pared entrara en el cierre.
        self.assertFalse(self.occupied_at(0.85, 4.5))
        row, col = self.costmap.cell(0.85, 4.5)
        self.assertTrue(self.costmap.traversable[row, col])

    def test_walls_are_told_apart_from_furniture(self):
        walls = wall_cells(self.grid)
        self.assertTrue(walls.ravel()[cell_of(self.grid, 0.45, 3.0)])     # pared exterior
        self.assertFalse(walls.ravel()[cell_of(self.grid, 3.02, 2.02)])   # pata

    def test_a_door_wider_than_two_radii_stays_open(self):
        self.assertFalse(self.occupied_at(6.5, 2.6))

    def test_the_room_itself_is_not_filled(self):
        # La sala está encerrada por paredes: no es un "hueco chico".
        self.assertGreater(np.count_nonzero(free_cells(self.marked)), 0.6 * np.count_nonzero(free_cells(self.grid)))

    def test_viewpoints_keep_their_clearance(self):
        plan = plan_viewpoints(self.marked, self.costmap, self.config, start=(1.0, 2.5))
        for x, y in plan.viewpoints:
            row, col = self.costmap.cell(x, y)
            self.assertGreaterEqual(self.costmap.clearance[row, col], 0.5)

    def test_path_distance_goes_around_the_block(self):
        reachable = main_component(self.costmap.traversable)
        a, b = (2.4, 2.75), (5.9, 2.75)
        distance = path_distances(self.costmap, reachable, [a, b])[0, 1]
        self.assertGreater(distance, 4.0)                 # en línea recta son 3,5 m
        self.assertAlmostEqual(path_distances(self.costmap, reachable, [a, a])[0, 1], 0.0)

    def test_too_much_clearance_is_rejected(self):
        config = ExplorationConfig(viewpoint_clearance=10.0)
        with self.assertRaises(ExplorationError):
            plan_viewpoints(self.grid, Costmap(self.grid, PlannerConfig()), config)

    def test_disabled_by_default(self):
        marked, _ = prepare_map(self.grid, PlannerConfig(), ExplorationConfig())
        self.assertIs(marked, self.grid)

    def test_original_map_is_not_modified(self):
        self.assertFalse(bool(occupied_cells(self.grid).ravel()[cell_of(self.grid, 4.25, 2.75)]))

    def test_clutter_blocks_on_a_bare_mask(self):
        mask = np.zeros((40, 40), dtype=bool)
        mask[10:30:6, 10:30:6] = True          # patas separadas 6 celdas
        # El centro de cuatro patas queda a 3 x raíz de 2 = 4,24 celdas de cada
        # una: un radio de 5 lo cubre (con 4 no).
        blocks = clutter_blocks(mask, radius_cells=5, max_hole_cells=100)
        self.assertTrue(blocks[13, 13])         # entre cuatro patas
        self.assertFalse(blocks[2, 2])          # lejos


class TourTests(unittest.TestCase):
    def test_points_on_a_line_are_visited_in_order(self):
        points = [(3.0, 0.0), (1.0, 0.0), (4.0, 0.0), (2.0, 0.0)]
        order = order_tour(points, (0.0, 0.0))
        self.assertEqual([points[i][0] for i in order], [1.0, 2.0, 3.0, 4.0])

    def test_two_opt_is_never_worse_than_nearest_neighbor(self):
        rng = np.random.default_rng(3)
        for _ in range(20):
            points = [tuple(p) for p in rng.uniform(0, 10, size=(9, 2))]
            start = (0.0, 0.0)
            order = order_tour(points, start)
            self.assertEqual(sorted(order), list(range(9)))
            # Vecino más cercano puro, sin 2-opt.
            remaining, position, greedy = list(range(9)), np.array(start), []
            while remaining:
                nearest = min(remaining, key=lambda i: np.hypot(*(np.array(points[i]) - position)))
                remaining.remove(nearest)
                greedy.append(nearest)
                position = np.array(points[nearest])
            self.assertLessEqual(tour_length(start, points, order),
                                 tour_length(start, points, greedy) + 1e-9)

    def test_empty(self):
        self.assertEqual(order_tour([], (0.0, 0.0)), [])

    def test_uses_given_distances_instead_of_straight_lines(self):
        # A está más cerca en línea recta, pero por el camino real B está más cerca.
        points = [(1.0, 0.0), (3.0, 0.0)]
        distances = np.array([[0.0, 10.0, 3.0], [10.0, 0.0, 2.0], [3.0, 2.0, 0.0]])
        self.assertEqual(order_tour(points, (0.0, 0.0)), [0, 1])
        self.assertEqual(order_tour(points, (0.0, 0.0), distances), [1, 0])
        self.assertAlmostEqual(tour_length((0.0, 0.0), points, [1, 0], distances), 5.0)


class CoverageTrackerTests(unittest.TestCase):
    def setUp(self):
        self.grid = room(5.0, 5.0)
        self.config = ExplorationConfig()
        visibility = Visibility(self.grid, self.config)
        costmap = Costmap(self.grid, PlannerConfig())
        targets = search_targets(self.grid, costmap.traversable)
        self.tracker = CoverageTracker(visibility, targets)

    def test_a_full_turn_needs_several_headings(self):
        headings = self.tracker.headings_to_check(2.5, 2.5, 0.0)
        # 60 grados de campo de visión: hacen falta unas 6 direcciones para la vuelta completa.
        self.assertGreaterEqual(len(headings), 5)
        self.assertLessEqual(len(headings), 8)

    def test_after_looking_there_is_nothing_left(self):
        for heading in self.tracker.headings_to_check(2.5, 2.5, 0.0):
            self.tracker.observe(2.5, 2.5, heading)
        self.assertEqual(self.tracker.headings_to_check(2.5, 2.5, 0.0), [])
        self.assertGreater(self.tracker.seen_fraction(), 0.9)

    def test_only_turns_toward_what_was_not_seen(self):
        # Ya miró hacia +x y hacia -x: no tiene que volver a mirar ahí.
        self.tracker.observe(2.5, 2.5, 0.0)
        self.tracker.observe(2.5, 2.5, math.pi)
        for heading in self.tracker.headings_to_check(2.5, 2.5, 0.0):
            self.assertGreater(abs(math.sin(heading)), 0.3)

    def test_first_heading_is_the_closest_turn(self):
        headings = self.tracker.headings_to_check(2.5, 2.5, 1.0)
        turns = [abs(math.remainder(h - 1.0, 2 * math.pi)) for h in headings]
        self.assertEqual(turns[0], min(turns))


class ConfigTests(unittest.TestCase):
    def test_defaults_are_valid(self):
        MissionConfig()

    def test_yaml_profile_loads(self):
        config = MissionConfig.load_yaml(Path(__file__).resolve().parents[1] / "config/real.yaml").exploration
        self.assertGreater(config.clutter_radius, 0)
        self.assertGreater(config.viewpoint_clearance, 0)
        self.assertEqual(config.viewpoints, ())

    def test_viewpoints_from_yaml(self):
        config = MissionConfig.from_dict({"exploration": {"viewpoints": [[1, 2], [3.5, -1]]}})
        self.assertEqual(config.exploration.viewpoints, ((1, 2), (3.5, -1)))

    def test_invalid_values_are_rejected(self):
        for bad in ({"exploration": {"view_range": 0}},
                    {"exploration": {"view_min_range": 4.0}},
                    {"exploration": {"coverage_target": 1.5}},
                    {"exploration": {"rays": 4}},
                    {"exploration": {"viewpoints": [[1, 2, 3]]}},
                    {"exploration": {"viewpoints": "1,2"}},
                    {"exploration": {"clutter_radius": -0.1}},
                    {"exploration": {"viewpoint_clearance": -1}},
                    {"exploration": {"clutter_max_hole": 0}},
                    {"exploration": {"unknown_field": 1}},
                    {"planner": {}}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                MissionConfig.from_dict(bad)


def box(grid, x0, y0, x1, y1):
    """Máscara de las celdas con centro dentro de un rectángulo (para medir regiones)."""
    c = grid.config
    xs = c.origin_x + (np.arange(grid.width) + 0.5) * c.resolution
    ys = c.origin_y + (np.arange(grid.height) + 0.5) * c.resolution
    return ((ys >= y0) & (ys <= y1))[:, None] & ((xs >= x0) & (xs <= x1))[None, :]


@unittest.skipUnless(CLASSROOM.is_file(), "classroom map not available")
class ClassroomTests(unittest.TestCase):
    """El mapa del aula del 16/11 con config/real.yaml y el planner del robot real."""

    @classmethod
    def setUpClass(cls):
        from nav.core.map import load_map
        root = Path(__file__).resolve().parents[2]
        cls.config = MissionConfig.load_yaml(root / "mission/config/real.yaml").exploration
        nav = NavConfig.load_yaml(root / "nav/config/real.yaml")
        cls.original = load_map(CLASSROOM)
        started = time.perf_counter()
        cls.grid, cls.costmap = prepare_map(cls.original, nav.planner, cls.config)
        cls.plan = plan_viewpoints(cls.grid, cls.costmap, cls.config, start=(0.0, 0.0))
        cls.seconds = time.perf_counter() - started
        cls.reachable = main_component(cls.costmap.traversable)

    def test_plan_reaches_the_target(self):
        self.assertGreaterEqual(self.plan.coverage, self.config.coverage_target)
        self.assertGreater(self.plan.area(self.plan.targets), 75.0)

    def test_reasonable_size_and_time(self):
        self.assertLessEqual(len(self.plan.viewpoints), 14)
        self.assertLess(self.plan.length, 45.0)
        self.assertLess(self.seconds, 10.0)

    def test_viewpoints_are_traversable_with_clearance(self):
        for x, y in self.plan.viewpoints:
            row, col = self.costmap.cell(x, y)
            self.assertTrue(self.costmap.traversable[row, col])
            self.assertGreaterEqual(self.costmap.clearance[row, col], self.config.viewpoint_clearance)

    def test_table_blocks_are_solid(self):
        # Medio de los dos bloques de mesas: el lidar ve piso entre las patas.
        for x, y in ((-5.0, 5.0), (-5.0, 1.3)):
            index = cell_of(self.grid, x, y)
            self.assertTrue(occupied_cells(self.grid).ravel()[index], (x, y))

    def test_robot_cannot_get_between_the_benches(self):
        # Franja de los bancos de abajo, debajo de la ruta principal.
        benches = box(self.grid, -7.5, -3.4, -2.5, -1.9)
        self.assertLess(np.count_nonzero(self.reachable & benches) * RESOLUTION ** 2, 0.6)

    def test_corridors_stay_open(self):
        # Pasillo central entre los bloques de mesas y pasillo entre el bloque
        # de arriba y la pared izquierda.
        for region in ((-5.5, 2.7, -4.0, 3.7), (-9.0, 5.0, -8.6, 6.5)):
            corridor = box(self.grid, *region)
            self.assertGreater(np.count_nonzero(self.reachable & corridor) / np.count_nonzero(corridor),
                               0.5, region)

    def test_central_corridor_is_covered(self):
        corridor = box(self.grid, -5.5, 2.5, -4.0, 3.9) & self.plan.targets
        seen = np.count_nonzero(corridor & self.plan.covered) / np.count_nonzero(corridor)
        self.assertGreater(seen, 0.95)


if __name__ == "__main__":
    unittest.main()
