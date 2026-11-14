"""Holonomic relative-odometry proposal supporting forward, reverse and strafe."""

import math
from typing import Protocol
import numpy as np

from .config import MotionConfig
from .geometry import Pose2D, wrap_angle


class MotionModel(Protocol):
    def sample(self, pose: Pose2D, odometry_delta: Pose2D, rng: np.random.Generator) -> Pose2D: ...


class SE2OdometryModel:
    def __init__(self, config: MotionConfig):
        self.config = config

    def sample(self, pose: Pose2D, odometry_delta: Pose2D, rng: np.random.Generator) -> Pose2D:
        d = odometry_delta
        trans2, rot2 = d.x ** 2 + d.y ** 2, d.yaw ** 2
        c = self.config
        std_xy = math.sqrt(c.alpha1 * trans2 + c.alpha2 * rot2)
        std_yaw = math.sqrt(c.alpha3 * rot2 + c.alpha4 * trans2)
        error = rng.normal(size=3)
        noisy = Pose2D(d.x + error[0] * std_xy, d.y + error[1] * std_xy,
                       wrap_angle(d.yaw + error[2] * std_yaw))
        return pose.compose(noisy)


class Proposal(Protocol):
    def propose(self, pose: Pose2D, odometry_delta: Pose2D, scan, grid,
                rng: np.random.Generator):
        """Return (sampled_pose, log[p(x|u)/q(x|u,z,m)] correction)."""
        ...


class OdometryProposal:
    def __init__(self, motion_model: MotionModel):
        self.motion_model = motion_model

    def propose(self, pose: Pose2D, odometry_delta: Pose2D, scan, grid,
                rng: np.random.Generator):
        # q is the motion prior, hence the importance correction is exactly zero.
        return self.motion_model.sample(pose, odometry_delta, rng), 0.0
