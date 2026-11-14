"""Small, ROS-independent SE(2) primitives (metres, radians)."""

from dataclasses import dataclass
import math


def wrap_angle(angle: float) -> float:
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
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(self.x + c * other.x - s * other.y,
                      self.y + s * other.x + c * other.y,
                      wrap_angle(self.yaw + other.yaw))

    def inverse(self) -> "Pose2D":
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(-c * self.x - s * self.y, s * self.x - c * self.y,
                      wrap_angle(-self.yaw))

    def between(self, other: "Pose2D") -> "Pose2D":
        return self.inverse().compose(other)


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    return a.compose(b)


def inverse(a: Pose2D) -> Pose2D:
    return a.inverse()


def between(a: Pose2D, b: Pose2D) -> Pose2D:
    return a.between(b)
