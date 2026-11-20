"""Vigila la llegada de scans utilizables sin depender de ROS."""

import numpy as np

from slam.core.types import Scan


def valid_returns(scan: Scan, minimum_range: float) -> np.ndarray:
    """Retornos finitos en rango, con la máscara del chasis."""
    return (np.isfinite(scan.ranges)
            & (scan.ranges > max(scan.range_min, minimum_range))
            & (scan.ranges <= scan.range_max))


class ScanWatchdog:
    """En real compara recepciones; en simulación también admite comparar stamps."""

    def __init__(self, timeout: float, minimum_range: float, simulated: bool):
        self.timeout = timeout
        self.minimum_range = minimum_range
        self.simulated = simulated
        self.last_received = None
        self.last_stamp = None
        self.clock_rewound = False
        self._last_sim_time = None

    def _clock_valid(self, now: float) -> bool:
        if not self.simulated:
            return True
        if self._last_sim_time is not None and now < self._last_sim_time:
            self.clock_rewound = True
        self._last_sim_time = now
        return not self.clock_rewound

    def accept(self, scan: Scan, received: float) -> bool:
        if not self._clock_valid(received):
            return False
        if self.last_stamp is not None and scan.stamp <= self.last_stamp:
            return False
        # /clock y los stamps del simulador comparten tiempo. En real, nunca
        # se compara el reloj del robot que firmó el scan con el de la PC.
        if self.simulated and abs(received - scan.stamp) >= self.timeout:
            return False
        if not valid_returns(scan, self.minimum_range).any():
            return False
        self.last_received = received
        self.last_stamp = scan.stamp
        return True

    def silence(self, now: float) -> float | None:
        return None if self.last_received is None else now - self.last_received

    def ready(self, now: float) -> bool:
        if not self._clock_valid(now):
            return False
        silence = self.silence(now)
        if silence is None or not 0.0 <= silence < self.timeout:
            return False
        return not self.simulated or now - self.last_stamp < self.timeout
