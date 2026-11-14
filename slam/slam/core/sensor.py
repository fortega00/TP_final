"""Preparación del scan y modelo de medición por campo de verosimilitud.

El modelo de medición responde: si el robot estuviera en esta pose y el mapa
fuera este, qué tan probable es haber medido este scan. Esa probabilidad es el
peso de la partícula. Sin ROS.
"""

from typing import Protocol, TYPE_CHECKING
import math
import numpy as np

from .config import SensorConfig
from .geometry import Pose2D
from .types import Scan

if TYPE_CHECKING:
    from .grid import OccupancyGrid


def prepare_scan(scan: Scan, config: SensorConfig, max_beams=None):
    """Filtra un scan y devuelve (rangos, ángulos, es_impacto).

    - NaN, -inf, lecturas muy cercanas o más allá del alcance físico del sensor
      se descartan.
    - Una lectura finita más allá del alcance útil configurado solo libera
      espacio hasta ese límite, sin marcar obstáculo.
    - +inf (el rayo no volvió) se descarta, salvo con config.no_return_is_free.
      En el lidar real un +inf aparece con vidrio o superficies oscuras, así que
      no garantiza espacio libre. En Gazebo sí. Ver docs/decisiones/001.
    """
    limit = min(scan.range_max, config.max_range)
    minimum = max(scan.range_min, config.min_range)
    if minimum >= limit:
        return np.empty(0), np.empty(0), np.empty(0, dtype=bool)
    values = scan.ranges
    valid = np.isfinite(values) & (values > minimum) & (values <= scan.range_max)
    if config.no_return_is_free:
        valid |= np.isposinf(values)
    indices = np.flatnonzero(valid)
    # Submuestreo parejo de rayos: menos cómputo, y rayos vecinos aportan casi lo mismo.
    count = config.max_beams if max_beams is None else max_beams
    if len(indices) > count:
        indices = indices[np.linspace(0, len(indices) - 1, count, dtype=int)]
    readings = values[indices]
    hits = np.isfinite(readings) & (readings < limit)
    return np.minimum(readings, limit), scan.angle_min + indices * scan.angle_increment, hits


class MeasurementModel(Protocol):
    """Interfaz que usa el filtro: pesar un scan y dibujarlo en el mapa."""

    def log_likelihood(self, grid: "OccupancyGrid", pose: Pose2D, scan: Scan) -> float: ...
    def update(self, grid: "OccupancyGrid", pose: Pose2D, scan: Scan) -> None: ...


class LaserSensorModel:
    def __init__(self, config: SensorConfig):
        self.config = config
        self._offset_cache = {}

    def log_likelihood(self, grid: "OccupancyGrid", pose: Pose2D, scan: Scan) -> float:
        """Log-probabilidad del scan visto desde `pose` sobre el mapa `grid`.

        Campo de verosimilitud: para cada rayo se calcula dónde termina y la
        distancia d de ese punto a la celda ocupada más cercana. La probabilidad
        del rayo es una mezcla:

            p = z_hit * exp(-d^2 / (2 sigma_hit^2)) + z_rand / alcance

        La gaussiana premia los puntos que caen sobre una pared conocida. El
        término uniforme evita que un solo rayo inexplicable (algo nuevo, una
        persona) lleve el peso a cero. El peso del scan es la suma de los
        logaritmos, o sea que supone rayos independientes.
        """
        cfg = self.config
        occupied = grid.observed & (grid.log_odds >= math.log(cfg.occupied_threshold /
                                                            (1 - cfg.occupied_threshold)))
        if not occupied.any():
            return 0.0  # Con el mapa vacío ninguna pose es mejor que otra.
        ranges, angles, hits = prepare_scan(scan, cfg, cfg.likelihood_beams)
        if not hits.any():
            return 0.0  # El modelo solo usa rayos que pegaron en algo.
        sensor_pose = pose.compose(scan.sensor_pose)
        angles = angles[hits] + sensor_pose.yaw
        ranges = ranges[hits]
        x = sensor_pose.x + ranges * np.cos(angles)
        y = sensor_pose.y + ranges * np.sin(angles)
        cols, rows = grid.world_to_grid(x, y)
        resolution = grid.config.resolution
        # Vecindario circular de celdas alrededor de cada punto, hasta likelihood_max_dist.
        # Se calcula una vez y se guarda.
        key = (resolution, cfg.likelihood_max_dist)
        if key not in self._offset_cache:
            radius = int(math.ceil(cfg.likelihood_max_dist / resolution))
            yy, xx = np.mgrid[-radius:radius + 1, -radius:radius + 1]
            mask = (xx * xx + yy * yy) * resolution ** 2 <= cfg.likelihood_max_dist ** 2
            self._offset_cache[key] = (xx[mask], yy[mask])
        dx, dy = self._offset_cache[key]
        neighbor_cols = cols[:, None] + dx
        neighbor_rows = rows[:, None] + dy
        valid = ((neighbor_cols >= 0) & (neighbor_cols < grid.width) &
                 (neighbor_rows >= 0) & (neighbor_rows < grid.height))
        values = occupied[np.clip(neighbor_rows, 0, grid.height - 1),
                          np.clip(neighbor_cols, 0, grid.width - 1)] & valid
        centers_x = grid.config.origin_x + (neighbor_cols + 0.5) * resolution
        centers_y = grid.config.origin_y + (neighbor_rows + 0.5) * resolution
        # Distancia al cuadrado a la celda ocupada más cercana, saturada en max_dist.
        distances2 = (centers_x - x[:, None]) ** 2 + (centers_y - y[:, None]) ** 2
        nearest2 = np.minimum(np.min(np.where(values, distances2, np.inf), axis=1),
                              cfg.likelihood_max_dist ** 2)
        # Un punto fuera del mapa no tiene pared que lo respalde, aunque haya una cerca del borde.
        outside = (cols < 0) | (cols >= grid.width) | (rows < 0) | (rows >= grid.height)
        nearest2[outside] = cfg.likelihood_max_dist ** 2
        probabilities = (cfg.z_hit * np.exp(-nearest2 / (2 * cfg.sigma_hit ** 2))
                         + cfg.z_rand / min(scan.range_max, cfg.max_range))
        # temperature < 1 aplana los pesos: compensa que los rayos no son realmente independientes.
        return float(cfg.likelihood_temperature * np.log(probabilities).sum())

    def update(self, grid: "OccupancyGrid", pose: Pose2D, scan: Scan) -> None:
        """Dibuja el scan en el mapa (modelo inverso, en grid.py)."""
        grid.integrate(pose, scan, self.config)
