"""Sensor input types; adapters must provide a scan-time odometry pose."""

from dataclasses import dataclass, field
import math
import numpy as np

from .geometry import Pose2D


@dataclass(frozen=True)
class Scan:
    ranges: np.ndarray
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    stamp: float
    sensor_pose: Pose2D = field(default_factory=Pose2D)

    def __post_init__(self):
        ranges = np.asarray(self.ranges, dtype=float)
        if ranges.ndim != 1 or len(ranges) == 0:
            raise ValueError("A scan requires a nonempty, one-dimensional ranges array")
        if not all(math.isfinite(v) for v in (self.angle_min, self.angle_increment,
                                               self.range_min, self.range_max, self.stamp)):
            raise ValueError("Scan metadata must be finite")
        if self.angle_increment == 0 or not 0 <= self.range_min < self.range_max:
            raise ValueError("Invalid scan angular increment or range limits")
        object.__setattr__(self, "ranges", ranges.copy())
