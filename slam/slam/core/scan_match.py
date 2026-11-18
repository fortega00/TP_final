"""Propuesta con scan matching: ubica cada partícula donde el scan encaja en su mapa.

La propuesta de odometría dispersa las partículas alrededor de lo que dicen las
ruedas y deja que los pesos descarten las malas. El lidar es mucho más preciso
que las ruedas, entonces esta propuesta usa primero el scan (la idea de GMapping
y FastSLAM 2.0):

1. estimación inicial = pose anterior compuesta con el incremento de odometría,
   sin ruido;
2. se prueban todas las poses de una ventana chica alrededor de esa estimación.
   Cada una recibe un puntaje: qué tan cerca caen los puntos del scan de las
   celdas ocupadas del mapa de la partícula, más un prior gaussiano que mantiene
   la respuesta cerca de la odometría cuando el scan no alcanza para decidir
   (un pasillo sin detalles);
3. se devuelve la mejor pose con un ruido gaussiano chico, para que las
   partículas no queden todas iguales.

Mientras el mapa tiene muy pocas celdas ocupadas no hay contra qué comparar y se
usa la propuesta de odometría.

Simplificación: el peso exacto de esta propuesta integra la verosimilitud sobre
toda la ventana. Acá la corrección es 0 y el filtro pesa cada partícula con la
verosimilitud del scan en su pose ajustada. Ver docs/decisiones/002. Sin ROS.
"""

import math
import numpy as np
from scipy.ndimage import distance_transform_edt

from .acceleration import BACKEND, lookup_scores, warmup
from .config import SlamConfig
from .geometry import Pose2D, wrap_angle
from .motion import OdometryProposal, SE2OdometryModel
from .sensor import prepare_scan


class ScanMatchProposal:
    def __init__(self, config: SlamConfig):
        self.acceleration_backend = BACKEND
        warmup()
        self.sensor = config.sensor
        self.config = c = config.scan_match
        self.odometry = OdometryProposal(SE2OdometryModel(config.motion))
        # Ventana de búsqueda: todas las combinaciones de (dx, dy, dyaw). Con los
        # valores por defecto son 9 x 9 x 13 = 1053 candidatas. Se arma una sola vez.
        offsets_xy = np.arange(-c.window_xy, c.window_xy + 1e-9, c.step_xy)
        offsets_yaw = np.arange(-c.window_yaw, c.window_yaw + 1e-9, c.step_yaw)
        self.dx, self.dy, self.dyaw = (a.ravel() for a in np.meshgrid(
            offsets_xy, offsets_xy, offsets_yaw, indexing="ij"))
        # Las 81 traslaciones repiten los mismos ángulos. Evaluar trigonometría
        # una vez por orientación conserva las candidatas y el orden original.
        self._yaw_offsets, self._yaw_indices = np.unique(self.dyaw, return_inverse=True)
        # Prior gaussiano centrado en la odometría: penaliza alejarse de ella.
        self.log_prior = (-(self.dx ** 2 + self.dy ** 2) / (2 * c.prior_xy ** 2)
                          - self.dyaw ** 2 / (2 * c.prior_yaw ** 2))

    def propose(self, pose: Pose2D, odometry_delta: Pose2D, scan, grid, rng: np.random.Generator):
        c, sensor = self.config, self.sensor
        threshold = math.log(sensor.occupied_threshold / (1 - sensor.occupied_threshold))
        occupied = grid.observed & (grid.log_odds >= threshold)
        ranges, angles, hits = prepare_scan(scan, sensor, c.beams)
        # Mapa todavía casi vacío: no hay contra qué comparar, se usa la odometría.
        if occupied.sum() < c.min_occupied_cells or not hits.any():
            return self.odometry.propose(pose, odometry_delta, scan, grid, rng)
        ranges, angles = ranges[hits], angles[hits]
        guess = pose.compose(odometry_delta)

        # Campo de distancias: para cada celda, la distancia a la celda ocupada más
        # cercana. Se calcula solo en el recorte del mapa que el scan puede alcanzar.
        resolution = grid.config.resolution
        max_range = min(scan.range_max, sensor.max_range)
        reach = int(math.ceil((max_range + c.window_xy) / resolution)) + 1
        col, row = (int(v) for v in grid.world_to_grid(guess.x, guess.y))
        col0, col1 = max(col - reach, 0), min(col + reach + 1, grid.width)
        row0, row1 = max(row - reach, 0), min(row + reach + 1, grid.height)
        patch = occupied[row0:row1, col0:col1]
        if not patch.any():
            return self.odometry.propose(pose, odometry_delta, scan, grid, rng)
        distance = np.minimum(distance_transform_edt(~patch) * resolution, sensor.likelihood_max_dist)
        # Todas las candidatas consultan el mismo campo. Evaluar la mezcla y su
        # logaritmo por celda evita repetir exp/log por cada candidata y haz.
        log_probability = np.log(sensor.z_hit * np.exp(-distance ** 2 / (2 * c.sigma ** 2))
                                 + sensor.z_rand / max_range)
        log_outside = math.log(sensor.z_hit * math.exp(-sensor.likelihood_max_dist ** 2 /
                                                       (2 * c.sigma ** 2))
                               + sensor.z_rand / max_range)

        # Puntos donde termina cada rayo para cada pose candidata, todo vectorizado:
        # arreglos de forma (candidatas, rayos).
        yaw = guess.yaw + self.dyaw
        mount = scan.sensor_pose
        sensor_x = guess.x + self.dx + np.cos(yaw) * mount.x - np.sin(yaw) * mount.y
        sensor_y = guess.y + self.dy + np.sin(yaw) * mount.x + np.cos(yaw) * mount.y
        beam = (guess.yaw + self._yaw_offsets)[:, None] + mount.yaw + angles[None, :]
        beam_x, beam_y = ranges * np.cos(beam), ranges * np.sin(beam)
        scores = lookup_scores(sensor_x, sensor_y, beam_x, beam_y, self._yaw_indices,
                               grid.config.origin_x, grid.config.origin_y, resolution,
                               col0, row0, log_probability, log_outside)

        # Misma mezcla que el campo de verosimilitud. El término uniforme hace que
        # el puntaje sature para los puntos lejos de toda pared conocida (zonas
        # recién vistas), así no arrastran la pose. Los rayos vecinos están
        # correlacionados, entonces el promedio se escala como si solo
        # `effective_beams` rayos fueran independientes.
        score = c.effective_beams * scores.mean(axis=1) + self.log_prior
        # Mejor candidata, más un ruido chico para mantener diversidad entre partículas.
        best = int(np.argmax(score))
        noise = rng.normal(size=3)
        return Pose2D(guess.x + self.dx[best] + noise[0] * c.sample_xy,
                      guess.y + self.dy[best] + noise[1] * c.sample_xy,
                      wrap_angle(yaw[best] + noise[2] * c.sample_yaw)), 0.0
