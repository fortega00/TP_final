"""Baja la frecuencia de la cámara conservando pares color-profundidad. Sin ROS.

La cámara del robot publica color y profundidad sin comprimir, unos 45 MB/s a
30 Hz. Para grabar un bag o mandarlo por wifi alcanzan pocos cuadros por
segundo. Si se bajara la frecuencia de cada flujo por separado, el cuadro de
profundidad que pasa podría ser de otro instante que el de color, y para ubicar
un cono hace falta la profundidad del mismo cuadro donde se lo detectó.

Por eso el color manda: `RateGate` decide qué cuadros de color pasan, y
`PairedThrottle` acompaña cada uno con el cuadro de profundidad más cercano en
el tiempo, si hay uno a menos de `tolerance` segundos.

Los mensajes se tratan como objetos opacos: solo importa su tiempo.
"""

from collections import deque
from dataclasses import dataclass


class RateGate:
    """Deja pasar como mucho un mensaje cada `period` segundos, según su tiempo.

    El próximo turno se calcula sumando `period` al anterior y no al tiempo del
    último mensaje aceptado. Así la frecuencia promedio es la pedida aunque los
    cuadros lleguen con jitter. Si el tiempo retrocede (un bag que vuelve a
    empezar, un reloj simulado reiniciado), se vuelve a empezar.
    """

    def __init__(self, period: float):
        if not period > 0:
            raise ValueError("period must be positive")
        self.period = float(period)
        self.next_due = None
        self.last = None

    def accept(self, stamp: float) -> bool:
        stamp = float(stamp)
        if self.last is not None and stamp < self.last:
            self.next_due = None
        # Margen de 1 µs: un cuadro que cae justo en el turno no se pierde por redondeo.
        if self.next_due is not None and stamp < self.next_due - 1e-6:
            return False
        if self.next_due is None or stamp - self.next_due >= self.period:
            # Primer mensaje, o pasó más de un turno sin mensajes: el próximo
            # turno se cuenta desde este.
            self.next_due = stamp + self.period
        else:
            self.next_due += self.period
        self.last = stamp
        return True


@dataclass
class ThrottleStats:
    primary_received: int = 0
    primary_passed: int = 0
    secondary_received: int = 0
    paired: int = 0
    # Cuadros de color que pasaron sin profundidad a menos de `tolerance`.
    unpaired: int = 0


class PairedThrottle:
    """Color a baja frecuencia, cada cuadro con su profundidad más cercana.

    `on_primary` y `on_secondary` devuelven la lista de mensajes que hay que
    publicar, como pares (canal, mensaje), con canal "primary" o "secondary".

    La profundidad puede llegar antes o después que el color del mismo
    instante. Las que llegan antes se guardan en un buffer corto; si al pasar un
    color no hay una cercana, se espera la próxima hasta `tolerance`.
    """

    def __init__(self, period: float, tolerance: float, buffer_size: int = 8):
        if not tolerance >= 0:
            raise ValueError("tolerance must be nonnegative")
        if buffer_size < 1:
            raise ValueError("buffer_size must be positive")
        self.gate = RateGate(period)
        self.tolerance = float(tolerance)
        self.buffer = deque(maxlen=buffer_size)   # (tiempo, mensaje) de profundidad
        self.waiting_for = None                   # tiempo del color que espera profundidad
        self.last_secondary = None                # para no publicar dos veces la misma
        self.stats = ThrottleStats()

    def on_primary(self, stamp: float, message):
        self.stats.primary_received += 1
        if not self.gate.accept(stamp):
            return []
        self.stats.primary_passed += 1
        if self.waiting_for is not None:
            # El color anterior se quedó sin pareja.
            self.stats.unpaired += 1
        self.waiting_for = None
        out = [("primary", message)]
        match = self._nearest(stamp)
        if match is not None:
            out.append(self._pair(match))
        else:
            self.waiting_for = float(stamp)
        return out

    def on_secondary(self, stamp: float, message):
        self.stats.secondary_received += 1
        stamp = float(stamp)
        if self.buffer and stamp < self.buffer[-1][0]:
            self.buffer.clear()   # el tiempo retrocedió
        self.buffer.append((stamp, message))
        if self.waiting_for is None:
            return []
        if abs(stamp - self.waiting_for) <= self.tolerance:
            self.waiting_for = None
            return [self._pair((stamp, message))]
        if stamp > self.waiting_for + self.tolerance:
            self.waiting_for = None
            self.stats.unpaired += 1
        return []

    def _nearest(self, stamp):
        candidates = [item for item in self.buffer
                      if abs(item[0] - stamp) <= self.tolerance and item[0] != self.last_secondary]
        if not candidates:
            return None
        return min(candidates, key=lambda item: abs(item[0] - stamp))

    def _pair(self, item):
        self.last_secondary = item[0]
        self.stats.paired += 1
        return ("secondary", item[1])
