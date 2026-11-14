"""Validated configuration shared by the CLI and ROS adapter."""

from dataclasses import asdict, dataclass, field, fields
import math
from pathlib import Path
from typing import Mapping
import yaml


@dataclass(frozen=True)
class MapConfig:
    resolution: float = 0.05
    width: float = 16.0
    height: float = 16.0
    origin_x: float = -8.0
    origin_y: float = -8.0
    prior: float = 0.5


@dataclass(frozen=True)
class SensorConfig:
    p_hit: float = 0.70
    p_free: float = 0.35
    log_odds_min: float = -4.0
    log_odds_max: float = 4.0
    hit_thickness: float = 0.03
    min_range: float = 0.12
    max_range: float = 8.0
    max_beams: int = 180
    likelihood_beams: int = 60
    sigma_hit: float = 0.10
    z_hit: float = 0.95
    z_rand: float = 0.05
    likelihood_max_dist: float = 0.60
    likelihood_temperature: float = 1.0
    occupied_threshold: float = 0.65
    # +inf means "no echo". On a real lidar that is no information (glass, dark or
    # grazing surfaces); in Gazebo it means the beam is empty up to range_max.
    no_return_is_free: bool = False


@dataclass(frozen=True)
class MotionConfig:
    # Variance coefficients for translation/rotation of a relative SE(2) increment.
    alpha1: float = 0.0001
    alpha2: float = 0.00001
    alpha3: float = 0.0001
    alpha4: float = 0.0001


@dataclass(frozen=True)
class FilterConfig:
    particles: int = 24
    seed: int = 7
    resample_ess_ratio: float = 0.5
    initial_x: float = 0.0
    initial_y: float = 0.0
    initial_yaw: float = 0.0
    initial_std_xy: float = 0.0
    initial_std_yaw: float = 0.0


@dataclass(frozen=True)
class UpdateConfig:
    min_translation: float = 0.08
    min_rotation: float = 0.06
    max_interval: float = 1.0


@dataclass(frozen=True)
class ScanMatchConfig:
    """Scan-matching proposal (slam/core/scan_match.py). Disabled: odometry proposal."""
    enabled: bool = False
    window_xy: float = 0.10       # search half-width around the odometry guess, m
    step_xy: float = 0.025
    window_yaw: float = 0.06      # rad
    step_yaw: float = 0.01
    sigma: float = 0.05           # endpoint-to-wall distance that counts as a fit, m (one cell)
    prior_xy: float = 0.05        # std of the pull towards the odometry guess, m
    prior_yaw: float = 0.03       # rad
    sample_xy: float = 0.01       # std of the sample around the matched pose, m
    sample_yaw: float = 0.005     # rad
    beams: int = 180
    effective_beams: float = 20.0
    min_occupied_cells: int = 200


@dataclass(frozen=True)
class SlamConfig:
    map: MapConfig = field(default_factory=MapConfig)
    sensor: SensorConfig = field(default_factory=SensorConfig)
    motion: MotionConfig = field(default_factory=MotionConfig)
    filter: FilterConfig = field(default_factory=FilterConfig)
    update: UpdateConfig = field(default_factory=UpdateConfig)
    scan_match: ScanMatchConfig = field(default_factory=ScanMatchConfig)

    def __post_init__(self):
        for group in (self.map, self.sensor, self.motion, self.filter, self.update, self.scan_match):
            for f in fields(group):
                value = getattr(group, f.name)
                if not isinstance(value, (float, int)) or not math.isfinite(value):
                    raise ValueError(f"Configuration {type(group).__name__}.{f.name} must be finite")
        m, s, f, u = self.map, self.sensor, self.filter, self.update
        if min(m.resolution, m.width, m.height) <= 0 or not 0 < m.prior < 1:
            raise ValueError("Map dimensions must be positive and prior in (0,1)")
        if max(math.ceil(m.width / m.resolution), math.ceil(m.height / m.resolution)) > 10000:
            raise ValueError("Map dimensions exceed 10000 cells per axis")
        if not 0 < s.p_free < m.prior < s.p_hit < 1:
            raise ValueError("Require 0 < p_free < map.prior < p_hit < 1")
        if not s.log_odds_min < math.log(m.prior / (1 - m.prior)) < s.log_odds_max:
            raise ValueError("Log-odds bounds must contain the prior")
        if not 0 <= s.min_range < s.max_range or s.hit_thickness <= 0:
            raise ValueError("Invalid sensor range/thickness")
        for key, value in (("particles", f.particles), ("seed", f.seed),
                           ("max_beams", s.max_beams), ("likelihood_beams", s.likelihood_beams)):
            if not isinstance(value, int) or isinstance(value, bool) or value < (0 if key == "seed" else 1):
                raise ValueError(f"{key} must be a {'nonnegative' if key == 'seed' else 'positive'} integer")
        if not 0 < f.resample_ess_ratio <= 1 or min(f.initial_std_xy, f.initial_std_yaw) < 0:
            raise ValueError("Invalid filter ESS ratio or initial standard deviation")
        if min(s.sigma_hit, s.z_rand, s.likelihood_max_dist, s.likelihood_temperature) <= 0:
            raise ValueError("Likelihood sigma, random weight, distance and temperature must be positive")
        if s.z_hit < 0 or not math.isclose(s.z_hit + s.z_rand, 1.0, abs_tol=1e-6):
            raise ValueError("Likelihood mixture weights must sum to 1")
        if not isinstance(s.no_return_is_free, bool):
            raise ValueError("sensor.no_return_is_free must be true or false")
        if not 0.5 < s.occupied_threshold < 1:
            raise ValueError("occupied_threshold must be in (0.5,1)")
        if min(self.motion.alpha1, self.motion.alpha2, self.motion.alpha3, self.motion.alpha4) < 0:
            raise ValueError("Motion variance coefficients must be nonnegative")
        if min(u.min_translation, u.min_rotation) < 0 or u.max_interval <= 0:
            raise ValueError("Invalid update thresholds")
        k = self.scan_match
        if not isinstance(k.enabled, bool):
            raise ValueError("scan_match.enabled must be true or false")
        if min(k.window_xy, k.window_yaw, k.sample_xy, k.sample_yaw) < 0:
            raise ValueError("Scan-match windows and sample deviations must be nonnegative")
        if min(k.step_xy, k.step_yaw, k.sigma, k.prior_xy, k.prior_yaw, k.effective_beams) <= 0:
            raise ValueError("Scan-match steps, sigma, priors and effective_beams must be positive")
        for key, value in (("beams", k.beams), ("min_occupied_cells", k.min_occupied_cells)):
            if not isinstance(value, int) or isinstance(value, bool) or value < 1:
                raise ValueError(f"scan_match.{key} must be a positive integer")

    @classmethod
    def from_dict(cls, data: Mapping) -> "SlamConfig":
        if not isinstance(data, Mapping):
            raise ValueError("Configuration must be a mapping")
        # Accept a plain core config or a ROS parameter-file envelope.
        if "/**" in data:
            data = data["/**"]["ros__parameters"]
        elif "ros__parameters" in data:
            data = data["ros__parameters"]
        groups = {"map": MapConfig, "sensor": SensorConfig, "motion": MotionConfig,
                  "filter": FilterConfig, "update": UpdateConfig,
                  "scan_match": ScanMatchConfig}
        unknown = set(data) - set(groups)
        if unknown:
            raise ValueError(f"Unknown core configuration groups: {sorted(unknown)}")
        try:
            return cls(**{name: typ(**data.get(name, {})) for name, typ in groups.items()})
        except TypeError as exc:
            raise ValueError(f"Unknown or invalid configuration field: {exc}") from exc

    @classmethod
    def load_yaml(cls, path) -> "SlamConfig":
        with Path(path).open(encoding="utf8") as handle:
            return cls.from_dict(yaml.safe_load(handle) or {})

    def to_dict(self) -> dict:
        return asdict(self)
