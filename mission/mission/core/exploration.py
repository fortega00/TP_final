"""Exploración sobre el mapa conocido con puntos de observación. Sin ROS.

El mapa del aula ya existe (etapa A), así que explorar no es descubrir la sala
sino **mirarla toda con la cámara** buscando conos. La cámara ve solo hacia
adelante, unos 60 grados, y detecta un cono con confianza hasta unos 3 m. El
plan es:

1. Objetivo: las celdas libres del aula, que es donde puede haber un cono.
2. Candidatos: puntos transitables (según el mapa de costos de la etapa B, así
   el navegador puede llegar) sobre una grilla de `viewpoint_spacing`.
3. Desde cada candidato se calcula qué celdas objetivo se ven en 360 grados:
   rayos que se cortan en la primera pared o zona desconocida.
4. Se eligen puntos de forma codiciosa (greedy): primero el que más área ve,
   después el que más área *nueva* agrega, hasta cubrir `coverage_target` de
   lo que se puede ver desde algún lugar, o hasta que el próximo agregue menos
   de `min_gain`. Es la aproximación clásica al problema de cubrir un
   conjunto, que es NP-difícil.
5. Se ordenan en un recorrido corto: vecino más cercano y después 2-opt, con
   distancias por la zona transitable (Dijkstra), no en línea recta: con
   bloques de mesas en el medio la diferencia es grande.

Durante la misión, `CoverageTracker` registra lo que la cámara vio de verdad,
con su campo de visión, y en cada punto propone girar solo hacia donde queda
algo sin ver (decisión 014).

El lidar ve solo las patas de mesas y bancos, así que el mapa los muestra casi
libres. Se corrige de forma automática, a partir del mismo mapa
(`prepare_map`):
- las paredes se separan de los muebles: una pared toca el exterior
  desconocido del aula, un mueble está adentro;
- `clutter_radius`: los muebles amontonados se unen en bloques (cierre
  morfológico) y se marcan como ocupados. Las paredes no entran en el cierre,
  así un pasillo entre un mueble y la pared no se tapa;
- `viewpoint_clearance`: el robot no se para a mirar en lugares angostos,
  como entre bancos; su piso se sigue buscando, mirándolo desde afuera.

Simplificaciones, documentadas en docs/decisiones/014:
- La visibilidad es 2D, sobre el plano del lidar: las patas de mesas y sillas
  del mapa tapan la vista aunque la cámara, más alta, vea por encima o al costado.
  El efecto es conservador: puede agregar algún punto de más.
- Las distancias del recorrido no tienen el costo de cercanía del A*, así que
  el camino real del navegador puede ser algo más largo.
- La cámara se supone en el centro del robot; está a unos centímetros.
"""

from dataclasses import dataclass
import math

import numpy as np
from scipy import ndimage, sparse
from scipy.sparse import csgraph

from slam.core.geometry import wrap_angle
from slam.core.grid import OccupancyGrid
from nav.core.config import PlannerConfig
from nav.core.map import free_cells
from nav.core.planner import Costmap

from .config import ExplorationConfig


class ExplorationError(Exception):
    """No se pudo armar el plan de exploración. El mensaje dice por qué."""


EIGHT = np.ones((3, 3), dtype=bool)   # vecindad de 8 celdas


def wall_cells(grid: OccupancyGrid) -> np.ndarray:
    """Obstáculos que son paredes: los que tocan el exterior desconocido del aula.

    El exterior es lo desconocido conectado con el borde de la grilla (fuera
    del aula). Lo desconocido de adentro, como detrás de una pata, no cuenta.
    Un obstáculo cuya componente conexa llega a menos de dos celdas del
    exterior es pared; el resto (patas, bordes de mesas y bancos) es mueble.
    """
    occupied = grid.observed & (grid.log_odds > grid.prior_log_odds)
    unknown = ~grid.observed
    labels, _ = ndimage.label(unknown, structure=EIGHT)
    border = np.unique(np.concatenate([labels[0], labels[-1], labels[:, 0], labels[:, -1]]))
    exterior = np.isin(labels, border[border > 0])
    near_exterior = ndimage.binary_dilation(exterior, structure=EIGHT, iterations=2)
    labels, _ = ndimage.label(occupied, structure=EIGHT)
    walls = np.unique(labels[near_exterior & occupied])
    return np.isin(labels, walls[walls > 0])


def clutter_blocks(furniture: np.ndarray, radius_cells: float, max_hole_cells: float) -> np.ndarray:
    """Une en bloques los muebles amontonados, como las patas de un grupo de mesas.

    Cierre morfológico con un disco de `radius_cells`: dilatar y después
    erosionar. Dos obstáculos a menos de dos radios quedan unidos; uno aislado
    queda igual y un paso más ancho no se cierra. Después se rellenan los
    huecos encerrados de menos de `max_hole_cells` (el interior de un bloque).
    """
    n = int(round(radius_cells))
    y, x = np.mgrid[-n:n + 1, -n:n + 1]
    closed = ndimage.binary_closing(furniture, structure=x * x + y * y <= n * n, border_value=0)
    holes = ndimage.binary_fill_holes(closed) & ~closed
    labels, count = ndimage.label(holes)
    if count:
        sizes = ndimage.sum(holes, labels, range(1, count + 1))
        closed |= np.isin(labels, np.flatnonzero(sizes < max_hole_cells) + 1)
    return closed | furniture


def prepare_map(grid: OccupancyGrid, planner: PlannerConfig, config: ExplorationConfig):
    """Mapa con mesas y bancos marcados como ocupados, y su mapa de costos.

    Con `clutter_radius` en 0 devuelve el mapa sin cambios. El mapa original no
    se modifica. El mapa de costos usa el grupo `planner` del navegador, así
    los puntos de observación son lugares adonde puede llegar.
    """
    if config.clutter_radius <= 0:
        return grid, Costmap(grid, planner)
    occupied = grid.observed & (grid.log_odds > grid.prior_log_odds)
    furniture = occupied & ~wall_cells(grid)
    cells = grid.config.resolution
    blocks = clutter_blocks(furniture, config.clutter_radius / cells, config.clutter_max_hole / cells ** 2)
    marked = grid.clone()
    marked.log_odds[blocks] = max(float(grid.log_odds.max()), 1.0)
    marked.observed[blocks] = True
    return marked, Costmap(marked, planner)


def _nearest(allowed: np.ndarray, row: int, col: int, max_cells: float):
    """Celda de `allowed` más cercana a (row, col), o None si está a más de max_cells."""
    radius = int(math.ceil(max_cells))
    r0, r1 = max(row - radius, 0), min(row + radius + 1, allowed.shape[0])
    c0, c1 = max(col - radius, 0), min(col + radius + 1, allowed.shape[1])
    rows, cols = np.nonzero(allowed[r0:r1, c0:c1])
    if not len(rows):
        return None
    distance = np.hypot(rows + r0 - row, cols + c0 - col)
    best = int(np.argmin(distance))
    if distance[best] > max_cells:
        return None
    return int(rows[best] + r0), int(cols[best] + c0)


class Visibility:
    """Qué celdas ve la cámara desde un punto, en 2D.

    Las paredes y lo desconocido tapan la vista. Cada rayo se muestrea cada
    media celda y se corta en la primera celda bloqueada; lo que está antes es
    visible si queda entre `view_min_range` y `view_range`.
    """

    def __init__(self, grid: OccupancyGrid, config: ExplorationConfig):
        self.grid = grid
        self.config = config
        self.resolution = grid.config.resolution
        self.blocked = ~free_cells(grid)
        step = self.resolution / 2
        self.distances = np.arange(step / 2, config.view_range, step)
        self.in_range = self.distances >= config.view_min_range
        self.all_angles = np.linspace(-math.pi, math.pi, config.rays, endpoint=False)
        # Rayos de un cono de visión, con la misma densidad angular que los de 360 grados.
        count = max(2, int(math.ceil(config.rays * config.camera_fov / (2 * math.pi))) + 1)
        self.fov_offsets = np.linspace(-config.camera_fov / 2, config.camera_fov / 2, count)

    def cast(self, x: float, y: float, angles) -> np.ndarray:
        """Índices planos de las celdas visibles, uno por muestra de cada rayo.

        Devuelve una matriz (rayos, muestras) con -1 donde la muestra no se ve.
        """
        angles = np.asarray(angles, dtype=float)
        c = self.grid.config
        xs = x + np.cos(angles)[:, None] * self.distances
        ys = y + np.sin(angles)[:, None] * self.distances
        cols = np.floor((xs - c.origin_x) / c.resolution).astype(int)
        rows = np.floor((ys - c.origin_y) / c.resolution).astype(int)
        inside = (cols >= 0) & (cols < self.grid.width) & (rows >= 0) & (rows < self.grid.height)
        blocked = np.ones(cols.shape, dtype=bool)
        blocked[inside] = self.blocked[rows[inside], cols[inside]]
        # Una vez que el rayo tocó algo bloqueado, todo lo que sigue queda tapado.
        occluded = np.logical_or.accumulate(blocked, axis=1)
        visible = ~occluded & self.in_range
        return np.where(visible, rows * self.grid.width + cols, -1)

    def visible(self, x: float, y: float, yaw: float = None) -> np.ndarray:
        """Celdas visibles (índices planos, sin repetir). Sin `yaw`, en 360 grados."""
        angles = self.all_angles if yaw is None else yaw + self.fov_offsets
        cells = self.cast(x, y, angles)
        return np.unique(cells[cells >= 0])


@dataclass
class ExplorationPlan:
    viewpoints: list            # [(x, y), ...] en el orden en que se recorren
    targets: np.ndarray         # máscara de celdas donde se busca
    coverable: np.ndarray       # máscara de celdas objetivo visibles desde algún candidato
    covered: np.ndarray         # máscara de celdas objetivo vistas desde los puntos elegidos
    manual: bool                # True si los puntos vinieron de la configuración
    resolution: float
    length: float = 0.0         # recorrido desde el inicio, por la zona transitable (m)

    def area(self, mask) -> float:
        return float(np.count_nonzero(mask)) * self.resolution ** 2

    @property
    def coverage(self) -> float:
        """Fracción de lo visible que cubren los puntos elegidos."""
        total = np.count_nonzero(self.coverable)
        return float(np.count_nonzero(self.covered & self.coverable)) / total if total else 0.0

    def summary(self) -> dict:
        return {"viewpoints": len(self.viewpoints), "manual": self.manual,
                "target_m2": round(self.area(self.targets), 2),
                "coverable_m2": round(self.area(self.coverable), 2),
                "covered_m2": round(self.area(self.covered & self.coverable), 2),
                "coverage": round(self.coverage, 4), "tour_m": round(self.length, 2)}


def main_component(mask: np.ndarray, seed=None) -> np.ndarray:
    """Componente conexa de `mask` que contiene `seed` (fila, columna), o la más grande."""
    labels, count = ndimage.label(mask, structure=np.ones((3, 3), dtype=bool))
    if count == 0:
        raise ExplorationError("The map has no traversable cells")
    if seed is not None and labels[seed] > 0:
        return labels == labels[seed]
    sizes = ndimage.sum(mask, labels, range(1, count + 1))
    return labels == int(np.argmax(sizes)) + 1


def search_targets(grid: OccupancyGrid, reachable: np.ndarray) -> np.ndarray:
    """Celdas libres conectadas con la zona transitable donde está el robot.

    Descarta manchas libres sueltas (ruido del mapa, lo que se ve por una puerta
    abierta hacia afuera) que no tocan la zona adonde el robot puede ir.
    """
    free = free_cells(grid)
    labels, count = ndimage.label(free, structure=np.ones((3, 3), dtype=bool))
    keep = np.unique(labels[reachable & free])
    return np.isin(labels, keep[keep > 0])


def plan_viewpoints(grid: OccupancyGrid, costmap: Costmap, config: ExplorationConfig,
                    start=None) -> ExplorationPlan:
    """Puntos de observación que cubren el aula, ordenados desde `start` (x, y).

    `grid` y `costmap` salen de `prepare_map`. Si `config.viewpoints` tiene
    puntos, se usan esos, en ese orden, después de llevar cada uno a la celda
    permitida más cercana. Si no, se calculan.
    """
    seed = None
    if start is not None:
        row, col = costmap.cell(*start)
        if not costmap.traversable[row, col]:
            snapped = costmap.nearest_traversable(row, col, config.snap_distance)
            if snapped is None:
                raise ExplorationError(f"Start ({start[0]:.2f}, {start[1]:.2f}) is not near a traversable cell")
            row, col = snapped
        seed = (row, col)
    reachable = main_component(costmap.traversable, seed)
    targets = search_targets(grid, reachable)
    # Donde el robot puede pararse a mirar: alcanzable y con al menos
    # viewpoint_clearance de holgura a los obstáculos.
    allowed = reachable & (costmap.clearance >= config.viewpoint_clearance)
    if not allowed.any():
        raise ExplorationError("No reachable cell has the required viewpoint clearance")
    visibility = Visibility(grid, config)
    if config.viewpoints:
        return _manual_plan(grid, costmap, config, visibility, allowed, targets, reachable, start)

    # Candidatos: celdas permitidas sobre una grilla de viewpoint_spacing.
    k = max(1, int(round(config.viewpoint_spacing / grid.config.resolution)))
    lattice = np.zeros_like(allowed)
    lattice[k // 2::k, k // 2::k] = True
    rows, cols = np.nonzero(allowed & lattice)
    if not len(rows):
        raise ExplorationError("No viewpoint candidates: viewpoint_spacing is larger than the free area")
    points = [costmap.point(r, c) for r, c in zip(rows, cols)]
    flat_targets = targets.ravel()
    seen_by = []
    for x, y in points:
        cells = visibility.visible(x, y)
        seen_by.append(cells[flat_targets[cells]])
    coverable = np.zeros(grid.width * grid.height, dtype=bool)
    for cells in seen_by:
        coverable[cells] = True
    total = int(coverable.sum())
    if total == 0:
        raise ExplorationError("No target cell is visible from any reachable point")

    # Cobertura codiciosa: cada vez, el candidato que más celdas nuevas agrega.
    covered = np.zeros_like(coverable)
    min_gain = config.min_gain / grid.config.resolution ** 2
    chosen = []
    while covered.sum() < config.coverage_target * total:
        gains = np.array([np.count_nonzero(~covered[cells]) for cells in seen_by])
        best = int(np.argmax(gains))
        if gains[best] < min_gain:
            break
        chosen.append(points[best])
        covered[seen_by[best]] = True
    origin = start if start is not None else chosen[0]
    distances = path_distances(costmap, reachable, [origin] + chosen)
    order = order_tour(chosen, origin, distances)
    shape = (grid.height, grid.width)
    return ExplorationPlan([chosen[i] for i in order], targets, coverable.reshape(shape),
                           covered.reshape(shape), False, grid.config.resolution,
                           tour_length(origin, chosen, order, distances))


def _manual_plan(grid, costmap, config, visibility, allowed, targets, reachable, start):
    """Plan con los puntos fijados a mano en la configuración, en su orden."""
    points = []
    for x, y in config.viewpoints:
        row, col = costmap.cell(x, y)
        if not allowed[row, col]:
            snapped = _nearest(allowed, row, col, config.snap_distance / grid.config.resolution)
            if snapped is None:
                raise ExplorationError(f"Viewpoint ({x:.2f}, {y:.2f}) is not reachable "
                                       "or is too close to an obstacle")
            row, col = snapped
            x, y = costmap.point(row, col)
        points.append((float(x), float(y)))
    covered = np.zeros(grid.width * grid.height, dtype=bool)
    for x, y in points:
        covered[visibility.visible(x, y)] = True
    covered = covered.reshape(grid.height, grid.width) & targets
    origin = start if start is not None else points[0]
    length = tour_length(origin, points, range(len(points)),
                         path_distances(costmap, reachable, [origin] + points))
    # Sin candidatos automáticos no se sabe qué es visible desde algún lugar:
    # la cobertura se informa sobre todas las celdas objetivo.
    return ExplorationPlan(points, targets, targets.copy(), covered, True, grid.config.resolution, length)


def path_distances(costmap: Costmap, reachable: np.ndarray, points) -> np.ndarray:
    """Distancias por la zona transitable entre todos los pares de `points` (m).

    Dijkstra sobre la grilla de 8 vecinos, sin el costo de cercanía del A*:
    alcanza para ordenar el recorrido. Con mesas grandes en el medio, la línea
    recta entre dos puntos puede ser mucho más corta que el camino real. Un
    punto fuera de `reachable` se lleva a la celda alcanzable más cercana; si
    dos puntos no se conectan, la distancia es infinita.
    """
    height, width = reachable.shape
    index = -np.ones(reachable.shape, dtype=np.int64)
    rows, cols = np.nonzero(reachable)
    index[rows, cols] = np.arange(len(rows))
    sources, targets, weights = [], [], []
    for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):
        r0, r1 = max(0, -dr), height - max(0, dr)
        c0, c1 = max(0, -dc), width - max(0, dc)
        a = index[r0:r1, c0:c1]
        b = index[r0 + dr:r1 + dr, c0 + dc:c1 + dc]
        both = (a >= 0) & (b >= 0)
        sources.append(a[both])
        targets.append(b[both])
        weights.append(np.full(int(both.sum()), math.hypot(dr, dc)))
    graph = sparse.coo_matrix((np.concatenate(weights), (np.concatenate(sources), np.concatenate(targets))),
                              shape=(len(rows), len(rows))).tocsr()
    nodes = []
    for x, y in points:
        row, col = costmap.cell(x, y)
        if not reachable[row, col]:
            snapped = _nearest(reachable, row, col, max(height, width))
            row, col = snapped
        nodes.append(index[row, col])
    found = csgraph.dijkstra(graph, directed=False, indices=nodes)
    return found[:, nodes] * costmap.grid.config.resolution


def tour_length(start, points, order, distances=None) -> float:
    """Largo de start -> points[order[0]] -> points[order[1]] -> ...

    Con `distances` (matriz de `path_distances` sobre [start] + points) usa
    esas distancias; si no, la línea recta.
    """
    if not len(order):
        return 0.0
    if distances is not None:
        stops = [0] + [i + 1 for i in order]
        return float(sum(distances[a, b] for a, b in zip(stops, stops[1:])))
    path = np.array([start] + [points[i] for i in order], dtype=float)
    return float(np.hypot(*np.diff(path, axis=0).T).sum())


def order_tour(points, start, distances=None) -> list:
    """Orden de visita corto desde `start`: vecino más cercano y después 2-opt.

    Es un camino abierto (no vuelve al inicio). 2-opt invierte tramos del
    recorrido mientras eso lo acorte; elimina los cruces que deja el vecino
    más cercano. `distances` es opcional, como en `tour_length`.
    """
    n = len(points)
    if n == 0:
        return []
    if distances is None:
        stops = np.asarray([start] + list(points), dtype=float)
        distances = np.hypot(*(stops[:, None, :] - stops[None, :, :]).transpose(2, 0, 1))
    remaining = list(range(n))
    order, current = [], 0
    while remaining:
        nearest = min(remaining, key=lambda i: distances[current, i + 1])
        remaining.remove(nearest)
        order.append(nearest)
        current = nearest + 1
    improved = True
    while improved:
        improved = False
        for i in range(n - 1):
            for j in range(i + 1, n):
                candidate = order[:i] + order[i:j + 1][::-1] + order[j + 1:]
                if (tour_length(start, points, candidate, distances)
                        < tour_length(start, points, order, distances) - 1e-9):
                    order, improved = candidate, True
    return order


class CoverageTracker:
    """Registra qué celdas objetivo ya vio la cámara y propone hacia dónde girar."""

    def __init__(self, visibility: Visibility, targets: np.ndarray):
        self.visibility = visibility
        self.targets = targets.ravel().copy()
        self.seen = np.zeros_like(self.targets)
        self.resolution = visibility.resolution

    def observe(self, x: float, y: float, yaw: float):
        """La cámara miró desde (x, y) hacia `yaw`: lo que entra en su campo queda visto."""
        cells = self.visibility.visible(x, y, yaw)
        self.seen[cells] = True

    def seen_fraction(self) -> float:
        total = np.count_nonzero(self.targets)
        return float(np.count_nonzero(self.seen & self.targets)) / total if total else 1.0

    def headings_to_check(self, x: float, y: float, yaw: float) -> list:
        """Direcciones hacia donde girar en (x, y) para ver lo que falta.

        Codicioso: entre `heading_candidates` direcciones, la que muestra más
        área objetivo todavía no vista, y así hasta que la próxima agregue menos
        de `heading_min_gain`. Se devuelven ordenadas para girar poco: cada una
        es la más cercana a la anterior, empezando por `yaw`.
        """
        config = self.visibility.config
        candidates = np.linspace(-math.pi, math.pi, config.heading_candidates, endpoint=False)
        unseen = self.targets & ~self.seen
        views = [self.visibility.visible(x, y, heading) for heading in candidates]
        views = [cells[unseen[cells]] for cells in views]
        pending = np.zeros_like(unseen)
        min_gain = config.heading_min_gain / self.resolution ** 2
        chosen = []
        while True:
            gains = np.array([np.count_nonzero(~pending[cells]) for cells in views])
            best = int(np.argmax(gains))
            if gains[best] < min_gain:
                break
            chosen.append(float(candidates[best]))
            pending[views[best]] = True
        ordered, current = [], yaw
        while chosen:
            nearest = min(chosen, key=lambda h: abs(wrap_angle(h - current)))
            chosen.remove(nearest)
            ordered.append(nearest)
            current = nearest
        return ordered
