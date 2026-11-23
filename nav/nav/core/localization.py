"""Localización Monte Carlo (MCL): filtro de partículas sobre un mapa conocido.

Cada partícula es una hipótesis de la pose del robot (x, y, yaw) en el frame del
mapa. A diferencia del FastSLAM de la etapa A, el mapa es uno solo y no cambia,
entonces las partículas no llevan grilla propia y se guardan como un arreglo.

Un paso del filtro (`process`):

1. predicción: cada partícula aplica el incremento de odometría con ruido;
2. peso: qué tan bien explica el scan cada partícula (campo de verosimilitud);
3. estimación: media ponderada y dispersión de la nube;
4. remuestreo, solo si los pesos degeneraron, con inyección de partículas al azar.

Modificaciones sobre el filtro básico, ver docs/decisiones/004:

- remuestreo solo con tamaño efectivo de muestra (ESS) bajo;
- compuerta de movimiento: con el robot quieto no se actualiza;
- ráfaga inicial (`converge`): después de un /initialpose se itera con el robot
  quieto hasta que la nube se cierra;
- inyección de partículas al azar cuando el scan deja de coincidir con el mapa;
- opcional: corregir el incremento de odometría con scan matching.

Sin ROS.
"""

import math
from time import perf_counter
import numpy as np
from scipy.ndimage import distance_transform_edt

from slam.core.filter import normalize_log_weights, systematic_resample
from slam.core.geometry import Pose2D
from slam.core.grid import OccupancyGrid
from slam.core.motion import SE2OdometryModel
from slam.core.scan_match import ScanMatchProposal
from slam.core.sensor import prepare_scan
from slam.core.types import Scan

from .config import NavConfig
from .map import free_cells, occupied_cells
from .obstacles import ObstacleSnapshot

# Log-verosimilitud de una partícula cuyo sensor quedó fuera del mapa: muere al remuestrear.
OUTSIDE_LOG_LIKELIHOOD = -1e6


class MonteCarloLocalization:
    def __init__(self, grid: OccupancyGrid, config: NavConfig):
        self.grid = grid
        self.config = config
        self.rng = np.random.default_rng(config.localization.seed)
        self.motion = SE2OdometryModel(config.motion)
        self.matcher = (ScanMatchProposal(config.slam_config())
                        if config.scan_match.enabled else None)
        occupied = occupied_cells(grid)
        if not occupied.any():
            raise ValueError("The map has no occupied cells to localize against")
        # Campo de distancias: para cada celda, la distancia a la pared más cercana.
        # El mapa no cambia, entonces se calcula una sola vez.
        self.distance = distance_transform_edt(~occupied) * grid.config.resolution
        self.free_rows, self.free_cols = np.nonzero(free_cells(grid))
        if not len(self.free_rows):
            raise ValueError("The map has no free cells")
        self.poses = None
        self.diagnostics = {"processed_scans": 0, "skipped_motion": 0, "invalid_scans": 0,
                            "resamples": 0, "injected_particles": 0, "burst_iterations": 0,
                            "ess": 0.0, "scan_score": 0.0, "unexplained_fraction": 0.0,
                            "static_unexplained_fraction": 0.0, "lost_check_seconds": 0.0,
                            "update_seconds": 0.0}

    # ----- Inicialización -----

    def initialize(self, pose: Pose2D, std_xy: float, std_yaw: float):
        """Arranca (o reinicia) el filtro con una nube gaussiana alrededor de `pose`.

        Es lo que corresponde a un /initialpose: el usuario marca una pose
        aproximada y la nube cubre el error de ese click.
        """
        if std_xy < 0 or std_yaw < 0:
            raise ValueError("Initial standard deviations must be nonnegative")
        n = self.config.localization.particles
        noise = self.rng.normal(size=(n, 3))
        self.poses = np.column_stack([pose.x + noise[:, 0] * std_xy,
                                      pose.y + noise[:, 1] * std_xy,
                                      pose.yaw + noise[:, 2] * std_yaw])
        self.log_weights = np.full(n, -math.log(n))
        self._last_odom = None
        self._score_slow = self._score_fast = None
        self._update_estimate(np.exp(self.log_weights))

    def converge(self, scan: Scan) -> bool:
        """Ráfaga inicial: cierra la nube con el robot quieto. Devuelve si convergió.

        La compuerta de movimiento impide actualizar con el robot quieto, pero
        justo después del click el robot está quieto y la nube es ancha. El
        lidar ve 360 grados, así que un solo scan alcanza para ubicarse. Cada
        iteración pesa las partículas con ese scan, remuestrea y les suma un
        ruido chico para explorar alrededor de las mejores.

        La ráfaga termina cuando la nube es chica y además una iteración más ya
        no mueve la estimación. Sin la segunda condición terminaría en la
        primera iteración: con la nube ancha casi todo el peso cae en una sola
        partícula, la dispersión ponderada da cero y parece convergida, pero es
        una sola hipótesis a varios centímetros de la pose real.

        Reusar el mismo scan cuenta la misma evidencia varias veces, por eso
        solo se hace acá y con un tope de iteraciones.
        """
        self._require_initialized()
        c = self.config.localization
        previous = None
        for iteration in range(c.burst_max_iterations):
            log_likelihood, beam_score = self._weigh(scan)
            if log_likelihood is None:
                raise ValueError("The scan has no valid returns to localize with")
            # Después de cada remuestreo los pesos son parejos, así que el peso
            # nuevo es directamente la verosimilitud normalizada.
            self.log_weights = normalize_log_weights(log_likelihood)
            weights = np.exp(self.log_weights)
            self._update_estimate(weights)
            self.diagnostics["burst_iterations"] = iteration + 1
            if previous is not None:
                moved = previous.between(self._estimate)
                stable = (math.hypot(moved.x, moved.y) < c.burst_stable_xy
                          and abs(moved.yaw) < c.burst_stable_yaw)
            previous = self._estimate
            # Puntaje del scan en la pose estimada: promedio ponderado por los pesos.
            self.diagnostics["scan_score"] = float(np.sum(weights * beam_score))
            if self.converged and iteration > 0 and stable:
                self._score_slow = self._score_fast = float(beam_score.mean())
                return True
            self.poses = self.poses[systematic_resample(weights, self.rng)]
            noise = self.rng.normal(size=self.poses.shape)
            self.poses += noise * [c.burst_jitter_xy, c.burst_jitter_xy, c.burst_jitter_yaw]
            self.log_weights = np.full(len(self.poses), -math.log(len(self.poses)))
        return False

    # ----- Paso del filtro -----

    def process(self, scan: Scan, odom_pose: Pose2D) -> bool:
        """Un paso del filtro con un scan y la odometría de ese instante.

        Devuelve True si el scan se usó. La primera odometría solo fija la
        referencia para medir incrementos.
        """
        self._require_initialized()
        if self._last_odom is None:
            self._last_odom = odom_pose
            return False
        if not prepare_scan(scan, self.config.sensor)[2].any():
            self.diagnostics["invalid_scans"] += 1
            return False
        delta = self._last_odom.between(odom_pose)
        c = self.config.localization
        # Compuerta de movimiento: con el robot quieto, scans repetidos no traen
        # información nueva y vuelven al filtro demasiado seguro de sí mismo.
        if math.hypot(delta.x, delta.y) < c.min_translation and abs(delta.yaw) < c.min_rotation:
            self.diagnostics["skipped_motion"] += 1
            return False
        started = perf_counter()
        self._predict(delta, scan)
        log_likelihood, beam_score = self._weigh(scan)
        self.log_weights = normalize_log_weights(self.log_weights + log_likelihood)
        weights = np.exp(self.log_weights)
        self._update_estimate(weights)
        self.diagnostics["scan_score"] = float(np.sum(weights * beam_score))
        self._resample(weights, float(beam_score.mean()))
        self._last_odom = odom_pose
        self.diagnostics["processed_scans"] += 1
        self.diagnostics["update_seconds"] = float(perf_counter() - started)
        return True

    def _predict(self, delta: Pose2D, scan: Scan):
        """Paso 1: mover cada partícula con el incremento de odometría más ruido.

        Con scan matching activado y la nube convergida, primero se corrige el
        incremento: se busca alrededor de (estimación anterior + odometría) la
        pose donde el scan mejor encaja en el mapa, y se usa el movimiento hasta
        esa pose en lugar del que dicen las ruedas. Se hace una sola vez por
        scan, no por partícula, porque todas comparten el mismo mapa.
        """
        if self.matcher is not None and self.converged:
            matched, _ = self.matcher.propose(self._estimate, delta, scan, self.grid, self.rng)
            delta = self._estimate.between(matched)
        for i, (x, y, yaw) in enumerate(self.poses):
            moved = self.motion.sample(Pose2D(x, y, yaw), delta, self.rng)
            self.poses[i] = moved.x, moved.y, moved.yaw

    def _weigh(self, scan: Scan):
        """Paso 2: log-verosimilitud del scan para cada partícula.

        Mismo campo de verosimilitud que `slam.core.sensor`, calculado para
        todas las partículas a la vez. Para cada rayo se ubica dónde termina y
        se mira la distancia d a la pared más cercana del mapa:

            p = z_hit * exp(-d^2 / (2 sigma_hit^2)) + z_rand / alcance

        Devuelve (log-verosimilitud por partícula, puntaje medio por rayo). El
        puntaje por rayo es la media geométrica de p: no depende de cuántos
        rayos tiene el scan y sirve para decidir la inyección.
        """
        sensor = self.config.sensor
        ranges, angles, hits = prepare_scan(scan, sensor, sensor.likelihood_beams)
        if not hits.any():
            return None, None
        ranges, angles = ranges[hits], angles[hits]
        x, y, yaw = self.poses.T
        mount = scan.sensor_pose
        sensor_x = x + np.cos(yaw) * mount.x - np.sin(yaw) * mount.y
        sensor_y = y + np.sin(yaw) * mount.x + np.cos(yaw) * mount.y
        # Extremos de los rayos: arreglos de forma (partículas, rayos).
        beam = yaw[:, None] + mount.yaw + angles[None, :]
        cols, rows = self.grid.world_to_grid(sensor_x[:, None] + ranges * np.cos(beam),
                                             sensor_y[:, None] + ranges * np.sin(beam))
        inside = (cols >= 0) & (cols < self.grid.width) & (rows >= 0) & (rows < self.grid.height)
        # Un extremo fuera del mapa no tiene pared que lo respalde.
        d = np.full(cols.shape, sensor.likelihood_max_dist)
        d[inside] = np.minimum(self.distance[rows[inside], cols[inside]], sensor.likelihood_max_dist)
        max_range = min(scan.range_max, sensor.max_range)
        log_p = np.log(sensor.z_hit * np.exp(-d ** 2 / (2 * sensor.sigma_hit ** 2))
                       + sensor.z_rand / max_range)
        log_likelihood = sensor.likelihood_temperature * log_p.sum(axis=1)
        beam_score = np.exp(log_p.mean(axis=1))
        sensor_col, sensor_row = self.grid.world_to_grid(sensor_x, sensor_y)
        outside = ((sensor_col < 0) | (sensor_col >= self.grid.width) |
                   (sensor_row < 0) | (sensor_row >= self.grid.height))
        if outside.all():
            raise ValueError("All particles left the map")
        log_likelihood[outside] = OUTSIDE_LOG_LIKELIHOOD
        beam_score[outside] = 0.0
        return log_likelihood, beam_score

    def _update_estimate(self, weights):
        """Paso 3: pose media ponderada y dispersión de la nube.

        El ángulo se promedia con seno y coseno (media circular).
        """
        x, y = np.sum(self.poses[:, :2] * weights[:, None], axis=0)
        yaw = math.atan2(float(np.sum(weights * np.sin(self.poses[:, 2]))),
                         float(np.sum(weights * np.cos(self.poses[:, 2]))))
        residuals = self.poses - [x, y, yaw]
        residuals[:, 2] = np.arctan2(np.sin(residuals[:, 2]), np.cos(residuals[:, 2]))
        self._estimate = Pose2D(float(x), float(y), yaw)
        self._covariance = (residuals * weights[:, None]).T @ residuals
        self.std_xy = math.sqrt(self._covariance[0, 0] + self._covariance[1, 1])
        self.std_yaw = math.sqrt(self._covariance[2, 2])

    def _resample(self, weights, score: float):
        """Paso 4: remuestreo con ESS bajo, más inyección de partículas al azar.

        Inyección (MCL aumentado): se llevan dos promedios del puntaje del scan,
        uno lento y uno rápido. Si el rápido cae por debajo del lento, el scan
        dejó de coincidir con el mapa (mal click inicial, robot movido a mano) y
        una fracción 1 - rápido/lento de las partículas se reemplaza por poses
        al azar en el espacio libre, para tener hipótesis en otros lugares.
        """
        c = self.config.localization
        n = len(self.poses)
        if self._score_slow is None:  # primer scan sin ráfaga inicial
            self._score_slow = self._score_fast = score
        self._score_slow +=c.injection_alpha_slow * (score - self._score_slow)
        self._score_fast += c.injection_alpha_fast * (score - self._score_fast)
        injected = 0
        if c.injection_enabled:
            fraction = max(0.0, 1.0 - self._score_fast / self._score_slow)
            injected = int(n * min(c.injection_max_fraction, fraction))
        ess = float(1.0 / np.sum(weights ** 2))
        self.diagnostics["ess"] = ess
        if ess >= c.resample_ess_ratio * n and not injected:
            return
        self.poses = self.poses[systematic_resample(weights, self.rng)]
        self.log_weights = np.full(n, -math.log(n))
        self.diagnostics["resamples"] += 1
        if injected:
            self.poses[self.rng.choice(n, injected, replace=False)] = self._random_free_poses(injected)
            self.diagnostics["injected_particles"] += injected
            # Después de inyectar se igualan los promedios, para no seguir
            # inyectando por la misma caída.
            self._score_slow = self._score_fast = score

    def _random_free_poses(self, count: int):
        """Poses uniformes sobre las celdas libres del mapa, con yaw uniforme."""
        cells = self.rng.integers(len(self.free_rows), size=count)
        offsets = self.rng.random(size=(count, 2))
        config = self.grid.config
        return np.column_stack([
            config.origin_x + (self.free_cols[cells] + offsets[:, 0]) * config.resolution,
            config.origin_y + (self.free_rows[cells] + offsets[:, 1]) * config.resolution,
            self.rng.uniform(-math.pi, math.pi, size=count)])

    # ----- Salidas -----

    def estimate(self):
        """Pose estimada y covarianza 3 x 3 de (x, y, yaw)."""
        self._require_initialized()
        return self._estimate, self._covariance.copy()

    @property
    def converged(self) -> bool:
        """La nube está concentrada: la estimación es confiable."""
        c = self.config.localization
        return self.std_xy < c.converged_std_xy and self.std_yaw < c.converged_std_yaw

    def lost(self, scan: Scan, after_click: bool = False, *,
             obstacles: ObstacleSnapshot | None = None) -> bool:
        """Comprueba los retornos contra el mapa fijo y la ocupación dinámica previa.

        Las paredes deben seguir explicando parte del scan: la capa dinámica se
        construyó con la pose estimada. El click se valida solo con el mapa fijo.
        """
        started = perf_counter()
        c = self.config.localization
        cols, rows, inside = self._scan_cells(scan)
        unexplained = self._wall_distances_at(cols, rows, inside) > c.unexplained_distance
        count = int(np.count_nonzero(unexplained))
        static_fraction = count / len(unexplained)
        self.diagnostics["static_unexplained_fraction"] = static_fraction
        if (not after_click and obstacles is not None and len(obstacles.points)
                and 0 < scan.stamp - obstacles.stamp <= self.config.navigator.scan_timeout
                and 0 < static_fraction <= 1 - c.min_static_match_fraction):
            candidates = unexplained & inside
            g = self.grid.config
            points = np.column_stack((g.origin_x + (cols[candidates] + 0.5) * g.resolution,
                                      g.origin_y + (rows[candidates] + 0.5) * g.resolution))
            distances, _ = obstacles.tree.query(points)
            count -= int(np.count_nonzero(distances <= c.unexplained_distance))
        fraction = count / len(unexplained)
        self.diagnostics["unexplained_fraction"] = fraction
        self.diagnostics["lost_check_seconds"] = perf_counter() - started
        return fraction > (c.click_lost_fraction if after_click else c.lost_fraction)

    def unexplained_fraction(self, scan: Scan) -> float:
        """Fracción de los puntos del scan que cae lejos de toda pared del mapa.

        Con la pose correcta casi todos los puntos caen sobre paredes, salvo los
        de obstáculos que no están en el mapa. Con una pose equivocada, una parte
        grande del scan queda sin explicar. Es una fracción y no la distancia
        media para que un obstáculo cercano, que tapa pocos rayos pero queda muy
        lejos de las paredes, no alcance para declarar perdido.
        """
        return float((self._wall_distances(scan) > self.config.localization.unexplained_distance).mean())

    def scan_residual(self, scan: Scan) -> float:
        """Distancia media de los extremos del scan a la pared más cercana del mapa.

        Se calcula con la pose estimada. Mide qué tan bien esa pose explica el
        scan y no necesita la pose real, así que sirve también en el robot real.
        Cada distancia se satura en `likelihood_max_dist` para que un obstáculo
        que no está en el mapa no domine el promedio.
        """
        return float(np.minimum(self._wall_distances(scan), self.config.sensor.likelihood_max_dist).mean())

    def _scan_cells(self, scan: Scan):
        """Proyecta los impactos a celdas desde la pose estimada."""
        self._require_initialized()
        ranges, angles, hits = prepare_scan(scan, self.config.sensor)
        if not hits.any():
            raise ValueError("The scan has no valid returns")
        sensor_pose = self._estimate.compose(scan.sensor_pose)
        beam = sensor_pose.yaw + angles[hits]
        cols, rows = self.grid.world_to_grid(sensor_pose.x + ranges[hits] * np.cos(beam),
                                             sensor_pose.y + ranges[hits] * np.sin(beam))
        inside = (cols >= 0) & (cols < self.grid.width) & (rows >= 0) & (rows < self.grid.height)
        return cols, rows, inside

    def _wall_distances(self, scan: Scan) -> np.ndarray:
        return self._wall_distances_at(*self._scan_cells(scan))

    def _wall_distances_at(self, cols, rows, inside) -> np.ndarray:
        """Un punto fuera del mapa cuenta como infinitamente lejos de las paredes."""
        d = np.full(cols.shape, np.inf)
        d[inside] = self.distance[rows[inside], cols[inside]]
        return d

    def _require_initialized(self):
        if self.poses is None:
            raise RuntimeError("Call initialize() with an initial pose first")
