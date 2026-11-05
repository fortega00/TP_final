"""FastSLAM-style RBPF with one conditional occupancy map per trajectory."""

from dataclasses import dataclass, field
import math
from time import perf_counter
import numpy as np

from .config import SlamConfig
from .geometry import Pose2D, wrap_angle
from .grid import OccupancyGrid
from .motion import OdometryProposal, SE2OdometryModel
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
        self.proposal = proposal or OdometryProposal(SE2OdometryModel(self.config.motion))
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
        if self.state in (SlamState.PAUSED, SlamState.STOPPED, SlamState.ERROR):
            return False
        if self.last_received_stamp is not None and scan.stamp <= self.last_received_stamp:
            self.diagnostics["rejected_stamp"] += 1
            return False
        self.last_received_stamp = scan.stamp
        if not len(prepare_scan(scan, self.config.sensor)[0]):
            self.diagnostics["invalid_scans"] += 1
            return False
        delta = Pose2D() if self._last_odom is None else self._last_odom.between(odom_pose)
        if self._last_odom is not None:
            u = self.config.update
            if (math.hypot(delta.x, delta.y) < u.min_translation and
                    abs(delta.yaw) < u.min_rotation and
                    scan.stamp - self.last_stamp < u.max_interval):
                self.diagnostics["skipped_motion"] += 1
                return False
        started = perf_counter()
        try:
            if self.state == SlamState.WAITING:
                self.machine.start()
            weights = []
            out_of_bounds = 0
            for particle in self.particles:
                pose, correction = self.proposal.propose(particle.pose, delta, scan,
                                                           particle.grid, self.rng)
                particle.pose = pose
                sensor_pose = pose.compose(scan.sensor_pose)
                inbounds = particle.grid.contains(sensor_pose.x, sensor_pose.y)
                if not inbounds:
                    out_of_bounds += 1
                # Importance scoring MUST see m[t-1], never a map containing z[t].
                score = (self.measurement_model.log_likelihood(particle.grid, pose, scan)
                         if inbounds else -1e6)
                weights.append(particle.log_weight + score + correction)
            self.diagnostics["out_of_bounds_particles"] = out_of_bounds
            if out_of_bounds == len(self.particles):
                raise ValueError("All particles left the configured map; enlarge bounds and reset")
            normalized = normalize_log_weights(weights)
            probabilities = np.exp(normalized)
            for particle, weight in zip(self.particles, normalized):
                particle.log_weight = float(weight)
                self.measurement_model.update(particle.grid, particle.pose, scan)
                particle.trajectory.append((float(scan.stamp), particle.pose))
            self._posterior_statistics(probabilities)
            best = int(np.argmax(normalized))
            self._best_particle = self.particles[best].clone()
            ess = float(1.0 / np.sum(probabilities ** 2))
            self.diagnostics["ess"] = ess
            if ess < self.config.filter.resample_ess_ratio * len(self.particles):
                indices = systematic_resample(probabilities, self.rng)
                self.particles = [self.particles[int(index)].clone() for index in indices]
                for particle in self.particles:
                    particle.log_weight = -math.log(len(self.particles))
                self.diagnostics["resamples"] += 1
            self._last_odom = odom_pose
            self.last_stamp = scan.stamp
            self.diagnostics["processed_scans"] += 1
            self.diagnostics["clipped_rays"] = self._best_particle.grid.clipped_rays
            self.diagnostics["out_of_bounds_particles"] = out_of_bounds
            self.diagnostics["state"] = self.state.value
            self.diagnostics["update_seconds"] = float(perf_counter() - started)
            return True
        except Exception:
            self.machine.fail()
            self.diagnostics["state"] = self.state.value
            raise

    def save_map(self, path):
        from .io import save_map
        if not self.diagnostics["processed_scans"]:
            raise RuntimeError("No map has been built yet")
        return save_map(self.best_particle.grid, path,
                        occupied_thresh=self.config.sensor.occupied_threshold)
