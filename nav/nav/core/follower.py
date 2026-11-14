"""Seguimiento del camino con pure pursuit y giros en el lugar. Sin ROS.

Pure pursuit: se elige un punto del camino a una distancia fija por delante del
robot (lookahead) y se calcula el arco de círculo que pasa por el robot y por
ese punto. Si el punto está en (x, y) en el frame del robot, a distancia L, la
curvatura del arco es

    kappa = 2 y / L^2

y la velocidad angular es w = v * kappa. Solo se manda avance y giro, sin
movimiento lateral. Decisiones y alternativas en docs/decisiones/007.
"""

import math
import numpy as np

from slam.core.geometry import Pose2D, wrap_angle

from .config import FollowerConfig


class PurePursuit:
    def __init__(self, path, config: FollowerConfig):
        self.path = np.asarray(path, dtype=float)
        if self.path.ndim != 2 or self.path.shape[1] != 2 or len(self.path) < 1:
            raise ValueError("A path needs at least one (x, y) point")
        self.config = config
        self.index = 0   # punto del camino más cercano al robot; solo avanza

    def distance_to_goal(self, pose: Pose2D) -> float:
        return math.hypot(*(self.path[-1] - [pose.x, pose.y]))

    def target(self, pose: Pose2D):
        """Punto de lookahead: el primero del camino a más de `lookahead` del robot."""
        position = np.array([pose.x, pose.y])
        distances = np.hypot(*(self.path[self.index:] - position).T)
        # El más cercano se busca desde el último índice, para no volver atrás
        # cuando el camino pasa dos veces cerca del mismo lugar.
        self.index += int(np.argmin(distances))
        ahead = np.hypot(*(self.path[self.index:] - position).T)
        beyond = np.flatnonzero(ahead >= self.config.lookahead)
        return self.path[self.index + beyond[0]] if len(beyond) else self.path[-1]

    def heading_error(self, pose: Pose2D) -> float:
        """Ángulo entre la orientación del robot y la dirección al punto de lookahead."""
        x, y = self.target(pose)
        return wrap_angle(math.atan2(y - pose.y, x - pose.x) - pose.yaw)

    def command(self, pose: Pose2D):
        """Velocidades (v, w) para seguir el camino."""
        c = self.config
        x, y = self.target(pose)
        # Punto de lookahead en el frame del robot.
        dx, dy = x - pose.x, y - pose.y
        local_x = math.cos(pose.yaw) * dx + math.sin(pose.yaw) * dy
        local_y = -math.sin(pose.yaw) * dx + math.cos(pose.yaw) * dy
        distance2 = local_x ** 2 + local_y ** 2
        if distance2 < 1e-9:
            return 0.0, 0.0
        curvature = 2.0 * local_y / distance2
        # Más lento cerca del objetivo, y más lento en curvas cerradas para no
        # superar la velocidad angular máxima.
        v = c.max_linear * min(1.0, self.distance_to_goal(pose) / c.slowdown_distance)
        v = max(v, c.min_linear)
        if abs(curvature) * v > c.max_angular:
            v = c.max_angular / abs(curvature)
        return v, v * curvature


def rotate_command(error: float, config: FollowerConfig) -> float:
    """Velocidad angular para girar en el lugar y anular `error` (control proporcional).

    Tiene un piso: con una velocidad muy chica el robot real no llega a moverse.
    """
    w = config.rotate_gain * error
    magnitude = min(max(abs(w), config.min_angular), config.max_angular)
    return math.copysign(magnitude, error)
