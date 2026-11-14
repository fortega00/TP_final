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
class NavConfig:
    sensor: SensorConfig = field(default_factory=SensorConfig)
    motion: MotionConfig = field(default_factory=MotionConfig)
    scan_match: ScanMatchConfig = field(default_factory=ScanMatchConfig)
    localization: LocalizationConfig = field(default_factory=LocalizationConfig)

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
        if not 0 < c.injection_alpha_slow < c.injection_alpha_fast <= 1:
            raise ValueError("Require 0 < injection_alpha_slow < injection_alpha_fast <= 1")
        if not 0 < c.injection_max_fraction <= 1:
            raise ValueError("localization.injection_max_fraction must be in (0, 1]")

    def slam_config(self) -> SlamConfig:
        """Los grupos compartidos en el formato que esperan las clases de `slam.core`."""
        return SlamConfig(sensor=self.sensor, motion=self.motion, scan_match=self.scan_match)

    @classmethod
    def from_dict(cls, data: Mapping) -> "NavConfig":
        if not isinstance(data, Mapping):
            raise ValueError("Configuration must be a mapping")
        groups = {"sensor": SensorConfig, "motion": MotionConfig,
                  "scan_match": ScanMatchConfig, "localization": LocalizationConfig}
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
