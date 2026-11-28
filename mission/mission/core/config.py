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


@dataclass(frozen=True)
class DetectorConfig:
    """Detector de conos rojos (decisión 016). Valores de OpenCV: tono 0-179, saturación y brillo 0-255.

    Supuestos hasta tener imágenes del aula (`bag_conos`): se ajustan en el paso 14.
    """
    # El rojo está en los dos extremos del tono: hasta hue_low_max o desde hue_high_min.
    # El naranja (unos 10 a 25) queda afuera.
    hue_low_max: int = 8
    hue_high_min: int = 170
    sat_min: int = 120
    val_min: int = 70
    # Lado del elemento de la apertura y el cierre morfológicos (px, impar).
    kernel: int = 5
    # Mancha mínima: menos que esto es ruido o un cono demasiado lejos (px²).
    min_area: int = 80
    # Alto / ancho del rectángulo que encierra la mancha.
    aspect_min: float = 0.6
    aspect_max: float = 4.0
    # Área de la mancha / área de su envolvente convexa: un cono es convexo.
    solidity_min: float = 0.75
    # Ancho del tercio de arriba / ancho del tercio de abajo: un cono se angosta.
    taper_max: float = 0.85


@dataclass(frozen=True)
class LocatorConfig:
    """Ubicación, confirmación y aproximación a un cono (decisión 017). Distancias en m."""
    # Profundidad válida de la cámara. Fuera de este rango no se ubica el cono.
    min_depth: float = 0.30
    max_depth: float = 4.0
    # Tamaño real aceptado de un cono (ancho y alto de la mancha, en m). Supuestos
    # hasta medir los conos del aula en la visita 1.
    width_min: float = 0.08
    width_max: float = 0.50
    height_min: float = 0.10
    height_max: float = 0.70
    # Dos detecciones a menos de esto son el mismo cono.
    associate_distance: float = 0.40
    # Filtro de Bayes binario: probabilidad de que el detector vea un cono que
    # está en su campo de visión, y de que vea uno donde no hay. Se miden en bag_conos.
    hit_rate: float = 0.70
    false_alarm_rate: float = 0.05
    # Probabilidad inicial de que una detección nueva sea un cono, umbral para
    # confirmarlo y umbral para olvidarlo.
    prior: float = 0.20
    confirm_probability: float = 0.97
    drop_probability: float = 0.05
    # Pose de aproximación: lo más cerca posible desde approach_distance, en
    # anillos cada 15 cm, hasta approach_max_distance (ver approach_pose).
    approach_distance: float = 0.45
    approach_max_distance: float = 1.50
    approach_angles: int = 24


@dataclass(frozen=True)
class MissionStateConfig:
    """Máquina de estados de la misión (decisión 015). Tiempos en s, distancias en m."""
    # Un objetivo cuenta como alcanzado si el navegador volvió a IDLE con el
    # robot a menos de esto del objetivo (posición y ángulo).
    reach_tolerance_xy: float = 0.30
    reach_tolerance_yaw: float = 0.35
    # Si el navegador sigue en IDLE este tiempo después de mandar un objetivo y
    # el robot no está ahí, el objetivo se considera rechazado.
    ack_timeout: float = 3.0
    # Tiempo máximo para un objetivo, sin contar el tiempo con la localización perdida.
    goal_timeout: float = 120.0
    # Parado mirando un candidato, cuánto se espera a que se confirme como cono.
    confirm_timeout: float = 4.0
    # Intentos de llegar a un cono confirmado antes de darlo por inalcanzable.
    approach_attempts: int = 2
    # Un cono confirmado a menos de esto de uno inalcanzable es el mismo.
    same_cone_distance: float = 0.50


def _numbers(item, count) -> bool:
    """`item` es una lista o tupla de `count` números finitos."""
    return (isinstance(item, (tuple, list)) and len(item) == count
            and all(isinstance(v, (float, int)) and not isinstance(v, bool) and math.isfinite(v)
                    for v in item))


@dataclass(frozen=True)
class MissionConfig:
    exploration: ExplorationConfig = field(default_factory=ExplorationConfig)
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    locator: LocatorConfig = field(default_factory=LocatorConfig)
    mission: MissionStateConfig = field(default_factory=MissionStateConfig)

    def __post_init__(self):
        loc = self.locator
        for f in fields(loc):
            value = getattr(loc, f.name)
            if not isinstance(value, (float, int)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"locator.{f.name} must be a positive number")
        for low, high in (("min_depth", "max_depth"), ("width_min", "width_max"),
                          ("height_min", "height_max"), ("approach_distance", "approach_max_distance")):
            if getattr(loc, low) >= getattr(loc, high):
                raise ValueError(f"locator.{low} must be smaller than locator.{high}")
        for name in ("hit_rate", "false_alarm_rate", "prior", "confirm_probability", "drop_probability"):
            if not getattr(loc, name) < 1:
                raise ValueError(f"locator.{name} must be in (0, 1)")
        if loc.false_alarm_rate >= loc.hit_rate:
            raise ValueError("locator.false_alarm_rate must be smaller than locator.hit_rate")
        if not loc.drop_probability < loc.prior < loc.confirm_probability:
            raise ValueError("Require locator.drop_probability < locator.prior < locator.confirm_probability")
        if not isinstance(loc.approach_angles, int) or loc.approach_angles < 4:
            raise ValueError("locator.approach_angles must be an integer of at least 4")
        d = self.detector
        for name in ("hue_low_max", "hue_high_min", "sat_min", "val_min", "kernel", "min_area"):
            value = getattr(d, name)
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError(f"detector.{name} must be a nonnegative integer")
        if not 0 <= d.hue_low_max < d.hue_high_min <= 179:
            raise ValueError("Require 0 <= detector.hue_low_max < detector.hue_high_min <= 179")
        if d.sat_min > 255 or d.val_min > 255:
            raise ValueError("detector.sat_min and detector.val_min must be at most 255")
        if d.kernel < 1 or d.kernel % 2 == 0:
            raise ValueError("detector.kernel must be a positive odd integer")
        for name in ("aspect_min", "aspect_max", "solidity_min", "taper_max"):
            value = getattr(d, name)
            if not isinstance(value, (float, int)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"detector.{name} must be a positive number")
        if d.aspect_min >= d.aspect_max:
            raise ValueError("detector.aspect_min must be smaller than detector.aspect_max")
        if d.solidity_min > 1:
            raise ValueError("detector.solidity_min must be at most 1")
        m = self.mission
        for f in fields(m):
            value = getattr(m, f.name)
            if not isinstance(value, (float, int)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"mission.{f.name} must be a positive number")
        if not isinstance(m.approach_attempts, int):
            raise ValueError("mission.approach_attempts must be an integer")
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
        groups = {"exploration": ExplorationConfig, "detector": DetectorConfig,
                  "locator": LocatorConfig, "mission": MissionStateConfig}
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
