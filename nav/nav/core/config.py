"""Parámetros de la etapa B, agrupados y validados. Sin ROS.

Los grupos `sensor`, `motion` y `scan_match` son los mismos de `slam.core`: la
localización usa el mismo modelo de sensor y de movimiento que construyó el
mapa. El grupo `localization` es propio de esta etapa. Un valor inválido lanza
un error al cargar, no se corrige en silencio.
"""

from dataclasses import dataclass, field, fields
import math
from pathlib import Path
from typing import Mapping
import yaml

from slam.core.config import MotionConfig, ScanMatchConfig, SensorConfig, SlamConfig


@dataclass(frozen=True)
class LocalizationConfig:
    """Filtro de partículas sobre mapa conocido (Monte Carlo Localization)."""
    particles: int = 1000
    seed: int = 7
    # Remuestrear solo cuando el tamaño efectivo de muestra cae bajo esta fracción de N.
    resample_ess_ratio: float = 0.5
    # Compuerta de movimiento: un scan se usa solo si el robot avanzó o giró esto.
    min_translation: float = 0.08
    min_rotation: float = 0.06
    # La nube se considera convergida por debajo de estos desvíos (m, rad).
    converged_std_xy: float = 0.10
    converged_std_yaw: float = 0.10
    # Perdido: más de esta fracción del scan cae lejos de toda pared del mapa, a más
    # de unexplained_distance (m). Una nube cerrada en una pose equivocada pasa por
    # convergida (docs/decisiones/004).
    unexplained_distance: float = 0.20
    lost_fraction: float = 0.25
    # Umbral más estricto para la ráfaga del click: ahí un falso encaje corrido a lo
    # largo de un pasillo deja 21 a 25 % sin explicar y una pose buena menos de 11 %.
    click_lost_fraction: float = 0.15
    # Ráfaga inicial con el robot quieto, después de cada /initialpose.
    burst_max_iterations: int = 30
    # La ráfaga termina cuando una iteración más mueve la estimación menos que esto.
    burst_stable_xy: float = 0.01
    burst_stable_yaw: float = 0.01
    burst_jitter_xy: float = 0.03
    burst_jitter_yaw: float = 0.02
    # Inyección de partículas al azar (MCL aumentado).
    injection_enabled: bool = True
    injection_alpha_slow: float = 0.001
    injection_alpha_fast: float = 0.1
    injection_max_fraction: float = 0.2


@dataclass(frozen=True)
class PlannerConfig:
    """Mapa de costos y A*. Distancias en metros."""
    # Inflación dura: ninguna celda a menos de robot_radius + safety_margin de un
    # obstáculo es transitable.
    robot_radius: float = 0.17
    safety_margin: float = 0.05
    # Costo de cercanía: desde el borde de la inflación decae linealmente hasta
    # cero en clearance_distance. Con peso 0 el camino es el más corto.
    clearance_distance: float = 0.50
    clearance_weight: float = 5.0
    # Si el robot arranca dentro de la zona inflada, el camino empieza en la celda
    # transitable más cercana, hasta esta distancia.
    start_snap_distance: float = 0.30


@dataclass(frozen=True)
class FollowerConfig:
    """Pure pursuit y giros en el lugar. m, rad, m/s, rad/s."""
    lookahead: float = 0.40
    max_linear: float = 0.20
    max_angular: float = 0.80
    # Cerca del objetivo la velocidad baja proporcional a la distancia, con este piso.
    min_linear: float = 0.05
    slowdown_distance: float = 0.50
    # Giro en el lugar: empieza con un error de orientación mayor que align_start y
    # termina por debajo de align_done.
    align_start: float = 0.80
    align_done: float = 0.15
    rotate_gain: float = 1.5
    min_angular: float = 0.20
    goal_tolerance_xy: float = 0.08
    goal_tolerance_yaw: float = 0.05


@dataclass(frozen=True)
class NavigatorConfig:
    """Máquina de estados y parada de seguridad."""
    # Parada de seguridad: frena si el lidar ve algo dentro de un rectángulo delante
    # del robot, de este largo (desde el centro) y este semiancho.
    stop_distance: float = 0.25
    stop_half_width: float = 0.20
    # Tiempos máximos, en segundos, parado por obstáculo o sin localización.
    blocked_timeout: float = 15.0
    # Parado por un obstáculo, cada cuánto se vuelve a planificar.
    retry_period: float = 2.0
    lost_timeout: float = 10.0
    # Si la pose no se actualiza en este tiempo, el robot frena.
    pose_timeout: float = 1.0
    # Silencio máximo entre scans utilizables. Independiente de la pose.
    scan_timeout: float = 0.50


@dataclass(frozen=True)
class ObstacleConfig:
    """Obstáculos que no están en el mapa (docs/decisiones/008)."""
    # El lidar solo agrega y libera obstáculos hasta esta distancia del robot (m).
    max_range: float = 3.0
    # Un impacto a menos de esto de una pared del mapa es esa pared, no un obstáculo nuevo (m).
    wall_distance: float = 0.20
    # Scans que tienen que ver una celda ocupada para que cuente como obstáculo.
    hits_to_mark: int = 3


@dataclass(frozen=True)
class NavConfig:
    sensor: SensorConfig = field(default_factory=SensorConfig)
    motion: MotionConfig = field(default_factory=MotionConfig)
    scan_match: ScanMatchConfig = field(default_factory=ScanMatchConfig)
    localization: LocalizationConfig = field(default_factory=LocalizationConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    follower: FollowerConfig = field(default_factory=FollowerConfig)
    navigator: NavigatorConfig = field(default_factory=NavigatorConfig)
    obstacles: ObstacleConfig = field(default_factory=ObstacleConfig)

    def __post_init__(self):
        # SlamConfig valida los tres grupos compartidos con sus propias reglas.
        self.slam_config()
        c = self.localization
        for f in fields(c):
            value = getattr(c, f.name)
            if not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError(f"localization.{f.name} must be finite")
        for name in ("particles", "burst_max_iterations"):
            value = getattr(c, name)
            if not isinstance(value, int) or isinstance(value, bool) or value < 1:
                raise ValueError(f"localization.{name} must be a positive integer")
        if not isinstance(c.seed, int) or isinstance(c.seed, bool) or c.seed < 0:
            raise ValueError("localization.seed must be a nonnegative integer")
        if not isinstance(c.injection_enabled, bool):
            raise ValueError("localization.injection_enabled must be true or false")
        if not 0 < c.resample_ess_ratio <= 1:
            raise ValueError("localization.resample_ess_ratio must be in (0, 1]")
        if min(c.burst_stable_xy, c.burst_stable_yaw) <= 0:
            raise ValueError("Burst stability thresholds must be positive")
        if min(c.min_translation, c.min_rotation, c.burst_jitter_xy, c.burst_jitter_yaw) < 0:
            raise ValueError("Motion gate and burst jitter must be nonnegative")
        if min(c.converged_std_xy, c.converged_std_yaw) <= 0:
            raise ValueError("Convergence thresholds must be positive")
        if c.unexplained_distance <= 0 or not 0 < c.click_lost_fraction <= c.lost_fraction < 1:
            raise ValueError("Require unexplained_distance > 0 and 0 < click_lost_fraction <= lost_fraction < 1")
        if not 0 < c.injection_alpha_slow < c.injection_alpha_fast <= 1:
            raise ValueError("Require 0 < injection_alpha_slow < injection_alpha_fast <= 1")
        if not 0 < c.injection_max_fraction <= 1:
            raise ValueError("localization.injection_max_fraction must be in (0, 1]")
        for name, group, may_be_zero in (("planner", self.planner, ("safety_margin", "clearance_weight")),
                                         ("follower", self.follower, ()),
                                         ("navigator", self.navigator, ()),
                                         ("obstacles", self.obstacles, ())):
            for f in fields(group):
                value = getattr(group, f.name)
                if not isinstance(value, (float, int)) or isinstance(value, bool) or not math.isfinite(value):
                    raise ValueError(f"{name}.{f.name} must be a finite number")
                if value < 0 or (value == 0 and f.name not in may_be_zero):
                    raise ValueError(f"{name}.{f.name} must be positive")
        if not isinstance(self.obstacles.hits_to_mark, int):
            raise ValueError("obstacles.hits_to_mark must be an integer")
        if self.follower.align_done >= self.follower.align_start:
            raise ValueError("follower.align_done must be smaller than follower.align_start")
        if self.follower.min_linear > self.follower.max_linear:
            raise ValueError("follower.min_linear must not exceed follower.max_linear")

    def slam_config(self) -> SlamConfig:
        """Los grupos compartidos en el formato que esperan las clases de `slam.core`."""
        return SlamConfig(sensor=self.sensor, motion=self.motion, scan_match=self.scan_match)

    @classmethod
    def from_dict(cls, data: Mapping) -> "NavConfig":
        if not isinstance(data, Mapping):
            raise ValueError("Configuration must be a mapping")
        groups = {"sensor": SensorConfig, "motion": MotionConfig,
                  "scan_match": ScanMatchConfig, "localization": LocalizationConfig,
                  "planner": PlannerConfig, "follower": FollowerConfig,
                  "navigator": NavigatorConfig, "obstacles": ObstacleConfig}
        unknown = set(data) - set(groups)
        if unknown:
            raise ValueError(f"Unknown configuration groups: {sorted(unknown)}")
        try:
            return cls(**{name: typ(**data.get(name, {})) for name, typ in groups.items()})
        except TypeError as exc:
            raise ValueError(f"Unknown or invalid configuration field: {exc}") from exc

    @classmethod
    def load_yaml(cls, path) -> "NavConfig":
        with Path(path).open(encoding="utf8") as handle:
            return cls.from_dict(yaml.safe_load(handle) or {})
