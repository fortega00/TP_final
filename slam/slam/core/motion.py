"""Modelo de movimiento por odometría y la propuesta más simple del filtro.

La odometría da cuánto dice el robot que se movió entre dos scans: un incremento
(dx, dy, dyaw) en su propio frame. El robot es omnidireccional, por eso el
incremento tiene componente lateral. Las ruedas patinan, así que cada partícula
aplica el incremento con un ruido distinto. Sin ROS.
"""

import math
from typing import Protocol
import numpy as np

from .config import MotionConfig
from .geometry import Pose2D, wrap_angle


class MotionModel(Protocol):
    """Interfaz: muestrear una pose nueva a partir de la anterior y la odometría."""

    def sample(self, pose: Pose2D, odometry_delta: Pose2D, rng: np.random.Generator) -> Pose2D: ...


class SE2OdometryModel:
    def __init__(self, config: MotionConfig):
        self.config = config

    def sample(self, pose: Pose2D, odometry_delta: Pose2D, rng: np.random.Generator) -> Pose2D:
        """Aplica el incremento de odometría con ruido gaussiano.

        La varianza del ruido crece con el movimiento:
            var_xy  = alpha1 * traslación^2 + alpha2 * rotación^2
            var_yaw = alpha3 * rotación^2   + alpha4 * traslación^2
        Si el robot no se mueve, no hay ruido.
        """
        d = odometry_delta
        trans2, rot2 = d.x ** 2 + d.y ** 2, d.yaw ** 2
        c = self.config
        std_xy = math.sqrt(c.alpha1 * trans2 + c.alpha2 * rot2)
        std_yaw = math.sqrt(c.alpha3 * rot2 + c.alpha4 * trans2)
        error = rng.normal(size=3)
        noisy = Pose2D(d.x + error[0] * std_xy, d.y + error[1] * std_xy,
                       wrap_angle(d.yaw + error[2] * std_yaw))
        # Componer: el incremento está en el frame del robot y se pasa al frame del mapa.
        return pose.compose(noisy)


class Proposal(Protocol):
    def propose(self, pose: Pose2D, odometry_delta: Pose2D, scan, grid,
                rng: np.random.Generator):
        """Devuelve (pose muestreada, corrección del log-peso).

        La corrección es log[p(x|u) / q(x|u,z,m)]: compensa haber muestreado de
        una distribución q distinta del modelo de movimiento p.
        """
        ...


class OdometryProposal:
    """Propuesta de FastSLAM 1.0: la pose nueva sale solo del modelo de movimiento."""

    def __init__(self, motion_model: MotionModel):
        self.motion_model = motion_model

    def propose(self, pose: Pose2D, odometry_delta: Pose2D, scan, grid,
                rng: np.random.Generator):
        # Se muestrea del mismo modelo de movimiento (q = p), entonces la corrección es 0.
        return self.motion_model.sample(pose, odometry_delta, rng), 0.0
