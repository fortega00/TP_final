"""FastSLAM-style RBPF with one conditional occupancy map per trajectory."""

from dataclasses import dataclass, field
import math
from time import perf_counter
import numpy as np

from .config import SlamConfig
from .geometry import Pose2D, wrap_angle
from .grid import OccupancyGrid
from .motion import OdometryProposal, SE2OdometryModel
from .scan_match import ScanMatchProposal
from .sensor import LaserSensorModel, prepare_scan
from .state_machine import SlamState, StateMachine
from .types import Scan


@dataclass
class Particle:
    pose: Pose2D
    grid: OccupancyGrid
    log_weight: float
    trajectory: list = field(default_factory=list)

    def clone(self):
        return Particle(self.pose, self.grid.clone(), self.log_weight, self.trajectory.copy())


def normalize_log_weights(log_weights):
    values = np.asarray(log_weights, dtype=float)
    if values.ndim != 1 or not len(values) or not np.isfinite(values).all():
        raise ValueError("Particle log-weights must be finite")
    maximum = float(values.max())
    values = values - maximum
    return values - math.log(float(np.exp(values).sum()))


def systematic_resample(weights, rng):
    values = np.asarray(weights, dtype=float)
    if values.ndim != 1 or not len(values) or not np.isfinite(values).all() or (values < 0).any():
        raise ValueError("Resampling requires finite nonnegative weights")
    total = float(values.sum())
    if total <= 0:
        raise ValueError("Resampling requires positive total weight")
    cumulative = np.cumsum(values / total)
    cumulative[-1] = 1.0
    positions = (rng.random() + np.arange(len(values))) / len(values)
    return np.searchsorted(cumulative, positions, side="right")


class FastSLAM:
    """Online RBPF; dependency injection keeps future proposals/sensors separate.

    ``process`` receives odometry interpolated to the scan's timestamp. First
    odometry anchors only the coordinate origin; the configured initial pose is
    the map-frame pose of the first scan. No ground truth is consumed.
    """

    def __init__(self, config=None, proposal=None, measurement_model=None):
        self.config = config or SlamConfig()
        self.proposal = proposal or (
            ScanMatchProposal(self.config) if self.config.scan_match.enabled
            else OdometryProposal(SE2OdometryModel(self.config.motion)))
        self.measurement_model = measurement_model or LaserSensorModel(self.config.sensor)
        self.machine = StateMachine()
        self.reset()

    @property
    def state(self):
        return self.machine.state

    @property
    def best_particle(self):
        # This is a detached snapshot selected before resampling, not an average
        # grid and not the arbitrarily first equally weighted resampled particle.
        return self._best_particle

    def reset(self):
        self.machine.reset()
        self.rng = np.random.default_rng(self.config.filter.seed)
        c = self.config.filter
        self.particles = []
        bounds = OccupancyGrid(self.config.map)
        for _ in range(c.particles):
            error = self.rng.normal(size=3)
            pose = Pose2D(c.initial_x + error[0] * c.initial_std_xy,
                          c.initial_y + error[1] * c.initial_std_xy,
                          wrap_angle(c.initial_yaw + error[2] * c.initial_std_yaw))
            if not bounds.contains(pose.x, pose.y):
                raise ValueError("Initial particle lies outside the configured map")
            self.particles.append(Particle(pose, OccupancyGrid(self.config.map),
                                           -math.log(c.particles)))
        self._best_particle = self.particles[0].clone()
        self._estimate_pose = self._best_particle.pose
        self._estimate_cov = np.diag([c.initial_std_xy ** 2, c.initial_std_xy ** 2,
                                      c.initial_std_yaw ** 2])
        self._last_odom = None
        self.last_stamp = None
        self.last_received_stamp = None
        self.diagnostics = {"state": self.state.value, "processed_scans": 0,
                            "skipped_motion": 0, "rejected_stamp": 0,
                            "invalid_scans": 0, "resamples": 0,
                            "ess": float(c.particles), "update_seconds": 0.0,
                            "clipped_rays": 0, "out_of_bounds_particles": 0}

    def pause(self):
        self.machine.pause()
        self.diagnostics["state"] = self.state.value

    def resume(self):
        self.machine.resume()
        self.diagnostics["state"] = self.state.value

    def stop(self):
        self.machine.stop()
        self.diagnostics["state"] = self.state.value

    def estimate(self):
        return self._estimate_pose, self._estimate_cov.copy()

    def _posterior_statistics(self, weights):
        points = np.array([[p.pose.x, p.pose.y, p.pose.yaw] for p in self.particles])
        x, y = np.sum(points[:, :2] * weights[:, None], axis=0)
        yaw = math.atan2(float(np.sum(weights * np.sin(points[:, 2]))),
                         float(np.sum(weights * np.cos(points[:, 2]))))
        residuals = points - np.array([x, y, yaw])
        residuals[:, 2] = np.arctan2(np.sin(residuals[:, 2]), np.cos(residuals[:, 2]))
        self._estimate_pose = Pose2D(float(x), float(y), yaw)
        self._estimate_cov = (residuals * weights[:, None]).T @ residuals

    def process(self, scan: Scan, odom_pose: Pose2D) -> bool:
        """Un paso completo de FastSLAM con un scan y la odometría de ese instante.

        Devuelve True si el scan se integró. Los cinco pasos son los del
        algoritmo: predecir, pesar, actualizar mapas, estimar y remuestrear.
        """
        delta = self._gate(scan, odom_pose)
        if delta is None:
            return False
        started = perf_counter()
        try:
            if self.state == SlamState.WAITING:
                self.machine.start()
            corrections = self._predict(delta, scan)
            log_weights = self._weigh(scan, corrections)
            self._update_maps(scan, log_weights)
            probabilities = np.exp(log_weights)
            self._update_estimate(log_weights, probabilities)
            self._resample_if_needed(probabilities)
            self._last_odom = odom_pose
            self.last_stamp = scan.stamp
            self.diagnostics["processed_scans"] += 1
            self.diagnostics["clipped_rays"] = self._best_particle.grid.clipped_rays
            self.diagnostics["state"] = self.state.value
            self.diagnostics["update_seconds"] = float(perf_counter() - started)
            return True
        except Exception:
            self.machine.fail()
            self.diagnostics["state"] = self.state.value
            raise

    def _gate(self, scan, odom_pose):
        """Decide si este scan se procesa. Devuelve el incremento de odometría o None.

        Se descarta si el filtro está pausado, si el scan llegó desordenado, si
        no tiene lecturas válidas o si el robot casi no se movió. Integrar scans
        con el robot quieto repite la misma evidencia y vuelve al filtro
        demasiado seguro de sí mismo.
        """
        if self.state in (SlamState.PAUSED, SlamState.STOPPED, SlamState.ERROR):
            return None
        if self.last_received_stamp is not None and scan.stamp <= self.last_received_stamp:
            self.diagnostics["rejected_stamp"] += 1
            return None
        self.last_received_stamp = scan.stamp
        if not len(prepare_scan(scan, self.config.sensor)[0]):
            self.diagnostics["invalid_scans"] += 1
            return None
        # El primer scan no tiene odometría anterior: el incremento es cero.
        delta = Pose2D() if self._last_odom is None else self._last_odom.between(odom_pose)
        if self._last_odom is not None:
            u = self.config.update
            if (math.hypot(delta.x, delta.y) < u.min_translation and
                    abs(delta.yaw) < u.min_rotation and
                    scan.stamp - self.last_stamp < u.max_interval):
                self.diagnostics["skipped_motion"] += 1
                return None
        return delta

    def _predict(self, delta, scan):
        """Paso 1, predicción: cada partícula muestrea su nueva pose de la propuesta.

        La propuesta es la odometría con ruido (FastSLAM 1.0) o scan matching
        contra el mapa de la propia partícula. Devuelve la corrección de peso
        que informa la propuesta para cada partícula.
        """
        corrections = []
        for particle in self.particles:
            particle.pose, correction = self.proposal.propose(
                particle.pose, delta, scan, particle.grid, self.rng)
            corrections.append(correction)
        return corrections

    def _weigh(self, scan, corrections):
        """Paso 2, peso: qué tan bien explica el scan cada partícula con su mapa.

        El scan se compara con el mapa anterior, m[t-1], que todavía no contiene
        este scan. Si se comparara con un mapa que ya lo incluye, todas las
        partículas coincidirían consigo mismas y el peso no diría nada.
        Devuelve los log-pesos normalizados.
        """
        log_weights = []
        out_of_bounds = 0
        for particle, correction in zip(self.particles, corrections):
            sensor_pose = particle.pose.compose(scan.sensor_pose)
            inbounds = particle.grid.contains(sensor_pose.x, sensor_pose.y)
            if not inbounds:
                out_of_bounds += 1
            # Una partícula fuera del mapa recibe un peso casi nulo y muere al remuestrear.
            score = (self.measurement_model.log_likelihood(particle.grid, particle.pose, scan)
                     if inbounds else -1e6)
            log_weights.append(particle.log_weight + score + correction)
        self.diagnostics["out_of_bounds_particles"] = out_of_bounds
        if out_of_bounds == len(self.particles):
            raise ValueError("All particles left the configured map; enlarge bounds and reset")
        return normalize_log_weights(log_weights)

    def _update_maps(self, scan, log_weights):
        """Paso 3, mapeo: cada partícula dibuja el scan en su mapa desde su pose.

        Es mapeo con pose conocida: dentro de una partícula la trayectoria se
        toma como cierta. Esa es la idea de Rao-Blackwell en FastSLAM.
        """
        for particle, log_weight in zip(self.particles, log_weights):
            particle.log_weight = float(log_weight)
            self.measurement_model.update(particle.grid, particle.pose, scan)
            particle.trajectory.append((float(scan.stamp), particle.pose))

    def _update_estimate(self, log_weights, probabilities):
        """Paso 4, estimación: pose media ponderada y copia de la mejor partícula.

        La mejor partícula se copia antes de remuestrear, porque después todas
        quedan con el mismo peso y ya no se puede elegir.
        """
        self._posterior_statistics(probabilities)
        self._best_particle = self.particles[int(np.argmax(log_weights))].clone()

    def _resample_if_needed(self, probabilities):
        """Paso 5, remuestreo, solo si el tamaño efectivo de muestra (ESS) es bajo.

        ESS = 1 / suma(w^2). Vale N con pesos iguales y 1 si una sola partícula
        concentra todo el peso. Remuestrear siempre destruye hipótesis sin
        necesidad, por eso se hace solo cuando ESS cae por debajo de una
        fracción de N.
        """
        ess = float(1.0 / np.sum(probabilities ** 2))
        self.diagnostics["ess"] = ess
        if ess < self.config.filter.resample_ess_ratio * len(self.particles):
            indices = systematic_resample(probabilities, self.rng)
            # clone() copia el mapa: dos copias de la misma partícula no comparten grilla.
            self.particles = [self.particles[int(index)].clone() for index in indices]
            for particle in self.particles:
                particle.log_weight = -math.log(len(self.particles))
            self.diagnostics["resamples"] += 1

    def save_map(self, path):
        from .io import save_map
        if not self.diagnostics["processed_scans"]:
            raise RuntimeError("No map has been built yet")
        return save_map(self.best_particle.grid, path,
                        occupied_thresh=self.config.sensor.occupied_threshold)
