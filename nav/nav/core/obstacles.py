"""Obstáculos que no están en el mapa. Sin ROS.

El mapa estático no se toca. Los obstáculos nuevos van en una capa aparte: una
grilla de ocupación como la del SLAM, que arranca vacía y se actualiza con cada
scan desde la pose estimada.

- Un impacto del lidar suma evidencia de ocupado en su celda.
- Un rayo que atraviesa una celda suma evidencia de libre. Así un obstáculo que
  se fue deja de bloquear.
- Una celda cuenta como obstáculo nuevo solo si está lejos de toda pared del
  mapa. Cerca de una pared, el impacto es esa pared vista con algo de error.

Decisiones, alternativas y datos en docs/decisiones/008.
"""

from dataclasses import dataclass, replace
from functools import cached_property
import math
import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree

from slam.core.config import SensorConfig
from slam.core.geometry import Pose2D
from slam.core.grid import OccupancyGrid
from slam.core.types import Scan

from .config import ObstacleConfig
from .map import occupied_cells


@dataclass(frozen=True)
class ObstacleSnapshot:
    """Centros ocupados ya confirmados, con el sello del último scan integrado."""

    stamp: float
    points: np.ndarray

    @cached_property
    def tree(self):
        """Índice disperso, se construye al necesitar la primera consulta."""
        return cKDTree(self.points)


class ObstacleLayer:
    def __init__(self, static: OccupancyGrid, sensor: SensorConfig, config: ObstacleConfig):
        # Más allá de max_range un rayo solo libera hasta ese límite, no marca.
        self.sensor = replace(sensor, max_range=config.max_range)
        self.grid = OccupancyGrid(static.config)
        wall_distance = distance_transform_edt(~occupied_cells(static)) * static.config.resolution
        self.away_from_walls = wall_distance > config.wall_distance
        # Evidencia que deja hits_to_mark impactos seguidos (medio impacto de margen).
        hit = math.log(sensor.p_hit / (1 - sensor.p_hit))
        self.threshold = (config.hits_to_mark - 0.5) * hit
        if self.threshold >= sensor.log_odds_max:
            raise ValueError("obstacles.hits_to_mark is too high for sensor.log_odds_max")
        self._points = None

    def observe(self, pose: Pose2D, scan: Scan):
        """Integra un scan tomado desde `pose` (pose de la base en el mapa)."""
        self.grid.integrate(pose, scan, self.sensor)
        self._points = None

    def cells(self) -> np.ndarray:
        """Máscara de celdas con un obstáculo que no está en el mapa."""
        return self.away_from_walls & (self.grid.log_odds > self.threshold)

    def points(self) -> np.ndarray:
        """Centros ocupados en metros, compartidos por el control y ROS."""
        if self._points is None:
            rows, cols = np.nonzero(self.cells())
            c = self.grid.config
            self._points = np.column_stack((c.origin_x + (cols + 0.5) * c.resolution,
                                            c.origin_y + (rows + 0.5) * c.resolution))
        return self._points
