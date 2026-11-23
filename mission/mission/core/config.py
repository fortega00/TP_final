"""Parámetros de la etapa C, agrupados y validados. Sin ROS.

Mismo criterio que `nav.core.config`: un valor inválido lanza un error al
cargar, no se corrige en silencio.

El mapa de costos no se configura acá: los puntos de observación tienen que ser
lugares adonde el navegador de la etapa B pueda llevar al robot, así que se
calculan con el grupo `planner` del YAML de `nav` que use el navegador. Tener
una copia propia permitiría que las dos diverjan.
"""

from dataclasses import dataclass, field, fields
import math
from pathlib import Path
from typing import Mapping
import yaml


@dataclass(frozen=True)
class ExplorationConfig:
    """Puntos de observación y cobertura de la cámara. Distancias en m, ángulos en rad."""
    # Separación de la grilla de candidatos a punto de observación.
    viewpoint_spacing: float = 0.50
    # Campo de visión horizontal de la cámara. La Astra tiene unos 60 grados.
    camera_fov: float = 1.0
    # Hasta dónde se espera detectar un cono con confianza, y desde dónde mide
    # la cámara de profundidad.
    view_range: float = 3.0
    view_min_range: float = 0.30
    # Se agregan puntos hasta ver esta fracción de lo que se puede ver desde
    # algún lugar transitable...
    coverage_target: float = 0.95
    # ...o hasta que el próximo punto agregue menos que esta área nueva (m²).
    min_gain: float = 0.50
    # Rayos para calcular qué se ve desde un punto, en 360 grados.
    rays: int = 360
    # Puntos fijados a mano, como [[x, y], ...] en el frame del mapa. Si hay,
    # reemplazan a los automáticos, en ese orden (respaldo para el robot real).
    viewpoints: tuple = ()
    # Un punto fijado a mano (o el inicio) que cae en zona no transitable se
    # mueve a la celda transitable más cercana, hasta esta distancia.
    snap_distance: float = 0.30
    # En cada punto, el robot gira hacia una dirección solo si la cámara vería
    # ahí al menos esta área todavía no vista (m²).
    heading_min_gain: float = 0.05
    # Direcciones candidatas para girar en cada punto, repartidas en 360 grados.
    heading_candidates: int = 36
    # Mesas y bancos: el lidar ve solo las patas y el mapa los muestra casi
    # libres. Detección automática a partir del mapa: los obstáculos interiores
    # (no las paredes) a menos de 2 x clutter_radius entre sí se unen en un
    # bloque, y los huecos encerrados de menos de clutter_max_hole (m²) se
    # rellenan. Un paso más ancho que 2 x clutter_radius no se cierra, ni el
    # que queda entre un mueble y una pared. 0 lo desactiva.
    clutter_radius: float = 0.0
    clutter_max_hole: float = 2.0
    # El robot no se para a mirar a menos de esta distancia de un obstáculo (m).
    # Excluye lugares angostos, como entre bancos; su piso se sigue buscando
    # desde afuera. 0 lo desactiva.
    viewpoint_clearance: float = 0.0


def _numbers(item, count) -> bool:
    """`item` es una lista o tupla de `count` números finitos."""
    return (isinstance(item, (tuple, list)) and len(item) == count
            and all(isinstance(v, (float, int)) and not isinstance(v, bool) and math.isfinite(v)
                    for v in item))


@dataclass(frozen=True)
class MissionConfig:
    exploration: ExplorationConfig = field(default_factory=ExplorationConfig)

    def __post_init__(self):
        e = self.exploration
        for f in fields(e):
            if f.name == "viewpoints":
                continue
            value = getattr(e, f.name)
            if not isinstance(value, (float, int)) or isinstance(value, bool) or not math.isfinite(value):
                raise ValueError(f"exploration.{f.name} must be a finite number")
            # Las dos opciones automáticas se desactivan con 0.
            may_be_zero = f.name in ("clutter_radius", "viewpoint_clearance")
            if value < 0 or (value == 0 and not may_be_zero):
                raise ValueError(f"exploration.{f.name} must be positive")
        for name in ("rays", "heading_candidates"):
            if not isinstance(getattr(e, name), int) or getattr(e, name) < 8:
                raise ValueError(f"exploration.{name} must be an integer of at least 8")
        if not e.camera_fov < 2 * math.pi:
            raise ValueError("exploration.camera_fov must be less than 2 pi")
        if not e.view_min_range < e.view_range:
            raise ValueError("exploration.view_min_range must be smaller than view_range")
        if not e.coverage_target <= 1:
            raise ValueError("exploration.coverage_target must be in (0, 1]")
        for point in e.viewpoints:
            if not _numbers(point, 2):
                raise ValueError("exploration.viewpoints must be a list of [x, y] pairs")

    @classmethod
    def from_dict(cls, data: Mapping) -> "MissionConfig":
        if not isinstance(data, Mapping):
            raise ValueError("Configuration must be a mapping")
        groups = {"exploration": ExplorationConfig}
        unknown = set(data) - set(groups)
        if unknown:
            raise ValueError(f"Unknown configuration groups: {sorted(unknown)}")
        values = {name: dict(data.get(name) or {}) for name in groups}
        # La lista del YAML pasa a tupla: la configuración es inmutable.
        if "viewpoints" in values["exploration"]:
            items = values["exploration"]["viewpoints"] or []
            if not isinstance(items, (list, tuple)):
                raise ValueError("exploration.viewpoints must be a list of [x, y] pairs")
            values["exploration"]["viewpoints"] = tuple(
                tuple(p) if isinstance(p, (list, tuple)) else p for p in items)
        try:
            return cls(**{name: typ(**values[name]) for name, typ in groups.items()})
        except TypeError as exc:
            raise ValueError(f"Unknown or invalid configuration field: {exc}") from exc

    @classmethod
    def load_yaml(cls, path) -> "MissionConfig":
        with Path(path).open(encoding="utf8") as handle:
            return cls.from_dict(yaml.safe_load(handle) or {})
