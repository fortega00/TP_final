"""Timestamp interpolation independent of ROS; never extrapolate robot motion."""
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
        # Keep one sample before the window, so its boundary can be interpolated.
        while len(self.samples) > 2 and self.samples[1][0] < stamp - self.history_seconds:
            self.samples.pop(0)
        return True

    def at(self, stamp):
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
        fraction = (stamp - t0) / (t1 - t0)
        return Pose2D(a.x + fraction * (b.x - a.x),
                      a.y + fraction * (b.y - a.y),
                      wrap_angle(a.yaw + fraction * wrap_angle(b.yaw - a.yaw)))
