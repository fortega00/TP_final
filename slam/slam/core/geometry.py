"""Poses en el plano (x, y, yaw) y sus operaciones. Metros y radianes. Sin ROS.

Una pose se puede leer de dos formas: dónde está un objeto, o la transformación
que lleva del frame de ese objeto al frame donde está expresado.
"""

from dataclasses import dataclass
import math


def wrap_angle(angle: float) -> float:
    """Lleva un ángulo al rango (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.x, self.y, self.yaw)):
            raise ValueError("Pose coordinates must be finite")

    def compose(self, other: "Pose2D") -> "Pose2D":
        """Encadena dos transformaciones: `other` está expresada en el frame de `self`.

        Ejemplo: pose del robot en el mapa compuesta con el incremento de
        odometría (en el frame del robot) da la nueva pose en el mapa.
        """
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(self.x + c * other.x - s * other.y,
                      self.y + s * other.x + c * other.y,
                      wrap_angle(self.yaw + other.yaw))

    def inverse(self) -> "Pose2D":
        """Transformación inversa: compose(inverse) da la identidad."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(-c * self.x - s * self.y, s * self.x - c * self.y,
                      wrap_angle(-self.yaw))

    def between(self, other: "Pose2D") -> "Pose2D":
        """Movimiento relativo de `self` a `other`, visto desde `self`.

        Con dos poses de odometría consecutivas da el incremento (dx, dy, dyaw).
        """
        return self.inverse().compose(other)


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    return a.compose(b)


def inverse(a: Pose2D) -> Pose2D:
    return a.inverse()


def between(a: Pose2D, b: Pose2D) -> Pose2D:
    return a.between(b)
