"""Mapa de costos y planificador A*. Sin ROS.

El mapa de costos sale del mapa estático:

- obstáculo: celda ocupada o desconocida. Por lo desconocido no se pasa;
- inflación dura: toda celda a menos de (radio del robot + margen) de un
  obstáculo es intransitable. Así el robot se puede tratar como un punto;
- costo de cercanía: desde el borde de la inflación, un costo extra que decae
  con la distancia. Hace que el camino se centre en los pasillos pero no le
  prohíbe pasar por una puerta.

Decisiones y alternativas en docs/decisiones/006.
"""

import heapq
import math
import numpy as np
from scipy.ndimage import distance_transform_edt

from slam.core.grid import OccupancyGrid

from .config import PlannerConfig
from .map import free_cells


class PlanningError(Exception):
    """No se pudo planificar. El mensaje dice por qué."""


class Costmap:
    def __init__(self, grid: OccupancyGrid, config: PlannerConfig):
        self.grid = grid
        self.config = config
        obstacle = ~free_cells(grid)
        # Distancia de cada celda al obstáculo más cercano, en metros.
        self.clearance = distance_transform_edt(~obstacle) * grid.config.resolution
        inflation = config.robot_radius + config.safety_margin
        self.traversable = self.clearance > inflation
        # Multiplicador del costo de pisar cada celda: 1 lejos de todo, hasta
        # 1 + clearance_weight pegado a la zona inflada.
        closeness = np.clip(1.0 - (self.clearance - inflation) / config.clearance_distance, 0.0, 1.0)
        self.cost = 1.0 + config.clearance_weight * closeness

    def cell(self, x: float, y: float):
        """(fila, columna) de un punto del mapa, o error si cae afuera."""
        col, row = self.grid.world_to_grid(x, y)
        if not (0 <= col < self.grid.width and 0 <= row < self.grid.height):
            raise PlanningError(f"Point ({x:.2f}, {y:.2f}) is outside the map")
        return row, col

    def point(self, row: int, col: int):
        """Centro de una celda, en metros."""
        c = self.grid.config
        return (c.origin_x + (col + 0.5) * c.resolution, c.origin_y + (row + 0.5) * c.resolution)

    def nearest_traversable(self, row: int, col: int, max_distance: float):
        """Celda transitable más cercana a (row, col), o None si está más lejos que max_distance."""
        radius = int(math.ceil(max_distance / self.grid.config.resolution))
        r0, r1 = max(row - radius, 0), min(row + radius + 1, self.grid.height)
        c0, c1 = max(col - radius, 0), min(col + radius + 1, self.grid.width)
        rows, cols = np.nonzero(self.traversable[r0:r1, c0:c1])
        if not len(rows):
            return None
        distance = np.hypot(rows + r0 - row, cols + c0 - col) * self.grid.config.resolution
        best = int(np.argmin(distance))
        if distance[best] > max_distance:
            return None
        return int(rows[best] + r0), int(cols[best] + c0)


# Los 8 vecinos: (cambio de fila, cambio de columna, largo del paso en celdas).
NEIGHBORS = [(dr, dc, math.hypot(dr, dc)) for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc]


def plan(costmap: Costmap, start, goal):
    """Camino de `start` a `goal`, como lista de puntos (x, y) en metros.

    A* sobre la grilla de 8 vecinos. El costo de un paso es su largo por el
    multiplicador de la celda de llegada. La heurística es la distancia en línea
    recta: nunca sobreestima porque el multiplicador es al menos 1, entonces el
    camino encontrado es el de menor costo.

    Lanza PlanningError si el objetivo no es transitable o no hay camino.
    """
    start_cell, goal_cell = costmap.cell(*start), costmap.cell(*goal)
    if not costmap.traversable[goal_cell]:
        raise PlanningError("The goal is inside or too close to an obstacle or unknown space")
    if not costmap.traversable[start_cell]:
        # El robot está pegado a una pared (o la pose tiene un error chico): el
        # camino arranca en la celda transitable más cercana.
        snapped = costmap.nearest_traversable(*start_cell, costmap.config.start_snap_distance)
        if snapped is None:
            raise PlanningError("The robot is inside an obstacle and no free cell is near")
        start_cell = snapped
    traversable, cost = costmap.traversable, costmap.cost
    height, width = traversable.shape
    best = {start_cell: 0.0}
    parent = {}
    # Cola de prioridad ordenada por f = costo acumulado + heurística.
    queue = [(math.hypot(goal_cell[0] - start_cell[0], goal_cell[1] - start_cell[1]), 0.0, start_cell)]
    while queue:
        _, g, current = heapq.heappop(queue)
        if current == goal_cell:
            break
        if g > best[current]:
            continue                    # entrada vieja: ya se llegó más barato
        row, col = current
        for dr, dc, length in NEIGHBORS:
            r, c = row + dr, col + dc
            if not (0 <= r < height and 0 <= c < width) or not traversable[r, c]:
                continue
            # Una diagonal no puede cortar la esquina de un obstáculo.
            if dr and dc and not (traversable[row, c] and traversable[r, col]):
                continue
            candidate = g + length * cost[r, c]
            if candidate < best.get((r, c), math.inf):
                best[(r, c)] = candidate
                parent[(r, c)] = current
                heapq.heappush(queue, (candidate + math.hypot(goal_cell[0] - r, goal_cell[1] - c),
                                       candidate, (r, c)))
    else:
        raise PlanningError("No path to the goal")
    cells = [goal_cell]
    while cells[-1] != start_cell:
        cells.append(parent[cells[-1]])
    path = [costmap.point(*cell) for cell in reversed(cells)]
    # El último punto es el objetivo exacto, no el centro de su celda.
    path[-1] = (float(goal[0]), float(goal[1]))
    return path
