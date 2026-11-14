"""Buffer de odometría con interpolación por tiempo. Sin ROS.

El scan y la odometría llegan a instantes distintos. Para saber dónde decía la
odometría que estaba el robot en el instante exacto del scan, se interpola entre
las dos muestras que lo rodean. Nunca se extrapola: si falta una muestra
posterior, se espera.
"""
from bisect import bisect_left
import math

from .geometry import Pose2D, wrap_angle


class OdometryBuffer:
    def __init__(self, history_seconds=30.0):
        if not math.isfinite(history_seconds) or history_seconds <= 0:
            raise ValueError("history_seconds must be positive")
        self.history_seconds = history_seconds
        self.samples = []

    def clear(self):
        self.samples.clear()

    def append(self, stamp, pose):
        if not all(math.isfinite(v) for v in (stamp, pose.x, pose.y, pose.yaw)):
            return False
        if self.samples and stamp <= self.samples[-1][0]:
            return False
        self.samples.append((stamp, pose))
        # Se conserva una muestra anterior a la ventana para poder interpolar en su borde.
        while len(self.samples) > 2 and self.samples[1][0] < stamp - self.history_seconds:
            self.samples.pop(0)
        return True

    def at(self, stamp):
        """Pose de odometría interpolada en `stamp`, o None si no hay muestras que lo rodeen."""
        if not self.samples or not math.isfinite(stamp):
            return None
        times = [sample[0] for sample in self.samples]
        i = bisect_left(times, stamp)
        if i < len(times) and abs(times[i] - stamp) < 1e-8:
            return self.samples[i][1]
        if i == 0 or i == len(times):
            return None
        t0, a = self.samples[i - 1]
        t1, b = self.samples[i]
        # Interpolación lineal. El ángulo se interpola por el camino corto (wrap_angle).
        fraction = (stamp - t0) / (t1 - t0)
        return Pose2D(a.x + fraction * (b.x - a.x),
                      a.y + fraction * (b.y - a.y),
                      wrap_angle(a.yaw + fraction * wrap_angle(b.yaw - a.yaw)))
