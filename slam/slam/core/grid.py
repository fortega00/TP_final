"""Grilla de ocupación en log-odds y modelo inverso del sensor.

Cada celda guarda qué tan probable es que esté ocupada, en log-odds:
l = log(p / (1 - p)). Con l = 0 la celda es desconocida (p = 0,5), positivo es
ocupada y negativo es libre. La ventaja de los log-odds es que integrar una
lectura nueva es una suma: l += log-odds de esa lectura (filtro de Bayes
binario). Sin ROS.
"""

import math
import numpy as np

from .config import MapConfig, SensorConfig
from .geometry import Pose2D
from .types import Scan
from .sensor import prepare_scan


class OccupancyGrid:
    """Mapa de una partícula. Fila 0 es la y mínima; `origin` es la esquina inferior izquierda."""

    def __init__(self, config: MapConfig):
        self.config = config
        self.width = int(math.ceil(config.width / config.resolution))
        self.height = int(math.ceil(config.height / config.resolution))
        self.prior_log_odds = math.log(config.prior / (1 - config.prior))
        self.log_odds = np.full((self.height, self.width), self.prior_log_odds, dtype=np.float32)
        # Marca qué celdas recibieron al menos una lectura; el resto es "desconocido".
        self.observed = np.zeros((self.height, self.width), dtype=bool)
        self.clipped_rays = 0

    def clone(self):
        result = object.__new__(OccupancyGrid)
        result.config = self.config
        result.width, result.height = self.width, self.height
        result.prior_log_odds = self.prior_log_odds
        result.log_odds = self.log_odds.copy()
        result.observed = self.observed.copy()
        result.clipped_rays = self.clipped_rays
        return result

    def world_to_grid(self, x, y):
        """Pasa de metros en el frame del mapa a índices (columna, fila)."""
        col = np.floor((np.asarray(x) - self.config.origin_x) / self.config.resolution).astype(int)
        row = np.floor((np.asarray(y) - self.config.origin_y) / self.config.resolution).astype(int)
        if col.ndim == 0:
            return int(col), int(row)
        return col, row

    def contains(self, x: float, y: float) -> bool:
        col, row = self.world_to_grid(x, y)
        return 0 <= col < self.width and 0 <= row < self.height

    def occupancy(self):
        """Mapa en el formato de ROS: 0 a 100 de probabilidad de ocupación, -1 desconocido."""
        values = np.rint(100.0 / (1.0 + np.exp(-self.log_odds))).astype(np.int8)
        values[~self.observed] = -1
        return values

    def integrate(self, pose: Pose2D, scan: Scan, config: SensorConfig):
        """Modelo inverso del sensor: dibuja un scan en la grilla desde una pose conocida.

        Para cada rayo, las celdas entre el sensor y el punto de impacto son
        evidencia de espacio libre, y las celdas alrededor del impacto son
        evidencia de obstáculo. Lo que queda detrás del impacto no se toca.
        """
        ranges, angles, hits = prepare_scan(scan, config)
        if not len(ranges):
            return
        sensor_pose = pose.compose(scan.sensor_pose)
        if not self.contains(sensor_pose.x, sensor_pose.y):
            # Si el sensor está fuera del mapa no se recorta contra el borde: hay que
            # agrandar el mapa y reiniciar.
            self.clipped_rays += len(ranges)
            return
        angles = angles + sensor_pose.yaw
        cosines, sines = np.cos(angles), np.sin(angles)
        half = config.hit_thickness / 2
        # Tramo libre: se muestrea cada rayo cada media celda, así no quedan celdas
        # salteadas. Cada celda se actualiza una sola vez por scan (np.unique), para
        # que varios rayos sobre la misma celda no multipliquen la confianza.
        step = self.config.resolution / 2
        steps = np.arange(0.0, float(ranges.max()) + half + step, step)
        positions = np.broadcast_to(steps, (len(ranges), len(steps)))
        # Libre hasta el impacto menos medio espesor. Un rayo sin impacto libera todo el tramo.
        free_mask = positions < (ranges - np.where(hits, half, 0))[:, None]
        xs = sensor_pose.x + positions * cosines[:, None]
        ys = sensor_pose.y + positions * sines[:, None]
        cols, rows = self.world_to_grid(xs, ys)
        inbounds = (cols >= 0) & (cols < self.width) & (rows >= 0) & (rows < self.height)
        free_indices = np.unique((rows * self.width + cols)[free_mask & inbounds])

        # Impactos: se marca un segmento de largo hit_thickness centrado en el punto
        # medido. Se muestrea aparte para que un espesor menor al paso igual marque
        # al menos una celda.
        hit_ranges = ranges[hits]
        offsets = np.unique(np.append(np.linspace(-half, half,
                                                   max(2, int(math.ceil(2 * half / step)) + 1)), 0.0))
        hit_positions = np.maximum(0.0, hit_ranges[:, None] + offsets)
        hx = sensor_pose.x + hit_positions * cosines[hits, None]
        hy = sensor_pose.y + hit_positions * sines[hits, None]
        hc, hr = self.world_to_grid(hx, hy)
        hit_valid = (hc >= 0) & (hc < self.width) & (hr >= 0) & (hr < self.height)
        # Si el punto medido cae fuera del mapa, su franja de espesor tampoco debe
        # inventar una pared justo adentro del borde.
        ec, er = self.world_to_grid(sensor_pose.x + hit_ranges * cosines[hits],
                                    sensor_pose.y + hit_ranges * sines[hits])
        endpoint_valid = (ec >= 0) & (ec < self.width) & (er >= 0) & (er < self.height)
        hit_valid &= endpoint_valid[:, None]
        hit_indices = np.unique((hr * self.width + hc)[hit_valid])
        # Una celda con impacto no cuenta además como libre en el mismo scan.
        free_indices = np.setdiff1d(free_indices, hit_indices, assume_unique=True)
        log_odds = self.log_odds.ravel()
        observed = self.observed.ravel()
        # Actualización de Bayes en log-odds: l += logit(p de la lectura) - logit(prior).
        # El recorte a [log_odds_min, log_odds_max] evita que una celda quede tan
        # segura que nunca más pueda corregirse.
        free_increment = math.log(config.p_free / (1 - config.p_free)) - self.prior_log_odds
        hit_increment = math.log(config.p_hit / (1 - config.p_hit)) - self.prior_log_odds
        log_odds[free_indices] = np.clip(log_odds[free_indices] + free_increment,
                                        config.log_odds_min, config.log_odds_max)
        log_odds[hit_indices] = np.clip(log_odds[hit_indices] + hit_increment,
                                       config.log_odds_min, config.log_odds_max)
        observed[free_indices] = True
        observed[hit_indices] = True
        # Diagnóstico: cuántos rayos terminaron fuera del mapa.
        endx = sensor_pose.x + ranges * cosines
        endy = sensor_pose.y + ranges * sines
        endc, endr = self.world_to_grid(endx, endy)
        self.clipped_rays += int(np.sum((endc < 0) | (endc >= self.width) |
                                        (endr < 0) | (endr >= self.height)))
