"""Ubicación de conos en el mapa, confirmación con filtro de Bayes y pose de aproximación. Sin ROS.

Decisión 017. Tres partes:

1. `measure`: de una detección (píxeles) a un punto (x, y) del mapa. La
   cámara de profundidad da la distancia en el centro de la mancha; con la
   calibración de la cámara (`Intrinsics`) el píxel pasa a un punto 3D en el
   frame óptico (z hacia adelante, x a la derecha, y hacia abajo), y con la
   pose de la cámara en el mapa (`map_T_camera`, de TF) al mapa. Con la
   distancia también se mide el tamaño real de la mancha: lo que no tiene
   tamaño de cono se descarta.
2. `ConeTracker`: filtro de Bayes binario por cono candidato, en log-odds,
   como una celda de la grilla de ocupación de la etapa A. Si el detector lo
   ve, suma log(acierto / falsa alarma). Si estaba en el campo de visión, a
   buena distancia y sin pared en medio, pero no lo vio, suma
   log((1 - acierto) / (1 - falsa alarma)), que es negativo. Se confirma
   cuando la probabilidad pasa un umbral.
3. `approach_pose`: dónde pararse frente al cono. Un punto alcanzable a
   `approach_distance` o algo más, desde el que el robot podría avanzar en
   línea recta hasta el cono. No alcanza con que el cono se vea: por un hueco
   en una pared se ve pero no se pasa. Así se elige un punto del lado del cono
   y el A* del navegador rodea la pared.
"""

from dataclasses import dataclass
import math

import numpy as np

from slam.core.geometry import Pose2D
from nav.core.planner import Costmap

from .config import LocatorConfig
from .exploration import path_distances


@dataclass(frozen=True)
class Intrinsics:
    """Calibración de la cámara (de `camera_info`): focales y centro óptico en px."""
    fx: float
    fy: float
    cx: float
    cy: float
    width: int

    @property
    def horizontal_fov(self) -> float:
        return 2.0 * math.atan(self.width / (2.0 * self.fx))


def logit(p: float) -> float:
    return math.log(p / (1.0 - p))


def blob_depth(depth: np.ndarray, box, config: LocatorConfig):
    """Distancia (m) a la mancha: mediana de la profundidad válida en el centro de su rectángulo.

    Se usa la mitad central del rectángulo para no mezclar el fondo de los
    bordes. Cero, NaN o valores fuera de rango son lecturas inválidas. Si hay
    muy pocas válidas, None.
    """
    x, y, w, h = box
    patch = depth[y + h // 4:y + h - h // 4, x + w // 4:x + w - w // 4]
    valid = patch[np.isfinite(patch) & (patch >= config.min_depth) & (patch <= config.max_depth)]
    if valid.size < 5:
        return None
    return float(np.median(valid))


def camera_position_yaw(map_T_camera: np.ndarray):
    """Posición (x, y) de la cámara en el mapa y hacia dónde mira (yaw del eje óptico z)."""
    forward = map_T_camera[:3, 2]
    return (float(map_T_camera[0, 3]), float(map_T_camera[1, 3])), math.atan2(forward[1], forward[0])


def measure(detection, depth: np.ndarray, intrinsics: Intrinsics, map_T_camera: np.ndarray,
            config: LocatorConfig):
    """Punto (x, y) del cono en el mapa, o None si no hay profundidad o no tiene tamaño de cono."""
    distance = blob_depth(depth, detection.box, config)
    if distance is None:
        return None
    _, _, w, h = detection.box
    width, height = w * distance / intrinsics.fx, h * distance / intrinsics.fy
    if not (config.width_min <= width <= config.width_max and config.height_min <= height <= config.height_max):
        return None
    u, v = detection.center
    point = np.array([(u - intrinsics.cx) * distance / intrinsics.fx,
                      (v - intrinsics.cy) * distance / intrinsics.fy, distance, 1.0])
    x, y = (map_T_camera @ point)[:2]
    return float(x), float(y)


def line_of_sight(blocked: np.ndarray, grid, a, b, ignore_end: float = 0.10) -> bool:
    """True si el segmento de `a` a `b` no cruza celdas bloqueadas del mapa.

    Se muestrea cada media celda. Los últimos `ignore_end` metros antes de `b`
    no cuentan: `b` es el cono, que puede estar pegado a una pared o a un mueble.
    """
    c = grid.config
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    if length <= ignore_end:
        return True
    steps = np.arange(0.0, length - ignore_end, c.resolution / 2) / length
    xs = a[0] + steps * (b[0] - a[0])
    ys = a[1] + steps * (b[1] - a[1])
    cols = np.floor((xs - c.origin_x) / c.resolution).astype(int)
    rows = np.floor((ys - c.origin_y) / c.resolution).astype(int)
    inside = (cols >= 0) & (cols < grid.width) & (rows >= 0) & (rows < grid.height)
    if not inside.all():
        return False
    return not blocked[rows, cols].any()


@dataclass
class Track:
    """Un cono candidato: posición en el mapa y probabilidad de ser un cono, en log-odds."""
    x: float
    y: float
    log_odds: float
    hits: int = 1

    @property
    def probability(self) -> float:
        return 1.0 / (1.0 + math.exp(-self.log_odds))


class ConeTracker:
    """Conos candidatos en el mapa, confirmados con un filtro de Bayes binario."""

    # Recorte de los log-odds, como en la grilla de la etapa A: que nada quede tan
    # seguro que no se pueda corregir.
    LIMIT = 8.0
    # Solo cuenta como "debería haberlo visto" si está dentro de esta fracción del
    # campo de visión: en el borde de la imagen el cono puede salir cortado.
    FOV_MARGIN = 0.8

    def __init__(self, config: LocatorConfig, blocked=None, grid=None):
        self.config = config
        self.blocked, self.grid = blocked, grid
        self.hit = math.log(config.hit_rate / config.false_alarm_rate)
        self.miss = math.log((1 - config.hit_rate) / (1 - config.false_alarm_rate))
        self.start = logit(config.prior)
        self.confirm = logit(config.confirm_probability)
        self.drop = logit(config.drop_probability)
        self.tracks = []
        self.seen_now = []      # candidatos vistos en la última actualización

    def update(self, points, camera_xy, camera_yaw: float, fov: float):
        """Una imagen: `points` son las detecciones ya ubicadas en el mapa."""
        matched = set()
        self.seen_now = []
        for x, y in points:
            free = [t for t in self.tracks if id(t) not in matched]
            nearest = min(free, key=lambda t: math.hypot(t.x - x, t.y - y), default=None)
            if nearest is not None and math.hypot(nearest.x - x, nearest.y - y) < self.config.associate_distance:
                # Promedio de las posiciones medidas: la estimación mejora con cada detección.
                nearest.x += (x - nearest.x) / (nearest.hits + 1)
                nearest.y += (y - nearest.y) / (nearest.hits + 1)
                nearest.hits += 1
                nearest.log_odds = min(nearest.log_odds + self.hit, self.LIMIT)
                track = nearest
            else:
                track = Track(x, y, self.start + self.hit)
                self.tracks.append(track)
            matched.add(id(track))
            self.seen_now.append(track)
        for track in self.tracks:
            if id(track) not in matched and self._should_see(track, camera_xy, camera_yaw, fov):
                track.log_odds = max(track.log_odds + self.miss, -self.LIMIT)
        self.tracks = [t for t in self.tracks if t.log_odds > self.drop]

    def confirmed(self):
        """El cono confirmado más probable, como (x, y), o None."""
        best = max(self.tracks, key=lambda t: t.log_odds, default=None)
        return (best.x, best.y) if best is not None and best.log_odds >= self.confirm else None

    def candidate(self):
        """El candidato sin confirmar más probable de los vistos en la última imagen, o None."""
        pending = [t for t in self.seen_now if t.log_odds < self.confirm and t in self.tracks]
        best = max(pending, key=lambda t: t.log_odds, default=None)
        return (best.x, best.y) if best is not None else None

    def _should_see(self, track, camera_xy, camera_yaw, fov) -> bool:
        dx, dy = track.x - camera_xy[0], track.y - camera_xy[1]
        distance = math.hypot(dx, dy)
        if not self.config.min_depth <= distance <= self.config.max_depth:
            return False
        bearing = math.atan2(dy, dx) - camera_yaw
        if abs(math.atan2(math.sin(bearing), math.cos(bearing))) > self.FOV_MARGIN * fov / 2:
            return False
        # Detrás de una pared no se lo puede ver: no es evidencia en contra.
        if self.blocked is not None and not line_of_sight(self.blocked, self.grid, camera_xy, (track.x, track.y)):
            return False
        return True


def open_segment(costmap: Costmap, a, cone) -> bool:
    """True si el robot podría avanzar en línea recta de `a` hacia el cono.

    No alcanza con que se vea: por un hueco en una pared se ve el cono pero el
    robot no pasa. A lo largo del segmento la distancia a los obstáculos tiene
    que ser al menos el radio del robot, salvo en el último tramo, donde el
    robot ya no llega porque frena antes del cono.
    """
    grid, radius = costmap.grid, costmap.config.robot_radius
    c = grid.config
    length = math.hypot(cone[0] - a[0], cone[1] - a[1])
    stop = length - (radius + 0.10)
    if stop <= 0:
        return True
    steps = np.arange(0.0, stop, c.resolution / 2) / length
    xs = a[0] + steps * (cone[0] - a[0])
    ys = a[1] + steps * (cone[1] - a[1])
    cols = np.floor((xs - c.origin_x) / c.resolution).astype(int)
    rows = np.floor((ys - c.origin_y) / c.resolution).astype(int)
    inside = (cols >= 0) & (cols < grid.width) & (rows >= 0) & (rows < grid.height)
    return bool(inside.all() and (costmap.clearance[rows, cols] >= radius).all())


def approach_pose(cone, pose: Pose2D, costmap: Costmap, reachable: np.ndarray, config: LocatorConfig,
                  clearance: float = 0.0):
    """Pose para quedar frente al cono, o None si no hay ninguna.

    Candidatos en anillos alrededor del cono, cada 15 cm desde
    approach_distance hasta approach_max_distance. Valen los alcanzables desde
    el robot, con al menos `clearance` de holgura a los obstáculos (la misma
    regla que los puntos de observación: no pararse en pasillos angostos, como
    entre bancos) y desde los que podría avanzar en línea recta hasta el cono
    (`open_segment`). Se usa el anillo más cercano al cono que tenga alguno, y
    en él el de camino más corto desde el robot: un cono en un pasillo angosto
    se aproxima desde la boca del pasillo. El ángulo mira al cono.
    """
    for radius in np.arange(config.approach_distance, config.approach_max_distance + 1e-9, 0.15):
        goal = _best_on_ring(cone, radius, pose, costmap, reachable, config, clearance)
        if goal is not None:
            return goal
    return None


def _best_on_ring(cone, radius, pose, costmap, reachable, config, clearance):
    """El mejor candidato a `radius` del cono, o None."""
    grid = costmap.grid
    candidates = []
    for angle in np.linspace(-math.pi, math.pi, config.approach_angles, endpoint=False):
        x, y = cone[0] + radius * math.cos(angle), cone[1] + radius * math.sin(angle)
        col, row = grid.world_to_grid(x, y)
        if not (0 <= col < grid.width and 0 <= row < grid.height) or not reachable[row, col]:
            continue
        if costmap.clearance[row, col] < clearance:
            continue
        if open_segment(costmap, (x, y), cone):
            candidates.append((x, y))
    if not candidates:
        return None
    distances = path_distances(costmap, reachable, [(pose.x, pose.y)] + candidates)[0, 1:]
    best = int(np.argmin(distances))
    if not math.isfinite(distances[best]):
        return None
    x, y = candidates[best]
    return Pose2D(x, y, math.atan2(cone[1] - y, cone[0] - x))
