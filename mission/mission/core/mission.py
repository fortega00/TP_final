"""Máquina de estados de la misión de la etapa C. Sin ROS. Decisión 015.

Decide a qué objetivo mandar al navegador de la etapa B: puntos de observación
mientras explora, giros en cada punto, frenar a mirar algo rojo y, si se
confirma que es un cono, acercarse. El navegador planifica y maneja; la misión
solo elige objetivos.

En cada paso (`update`) recibe la pose, el estado del navegador (`/nav/state`),
un candidato visto por el detector (todavía sin confirmar) y un cono
confirmado. Devuelve un objetivo nuevo para publicar en `/goal_pose`, o None.

El navegador no tiene "cancelar": interrumpir es mandar otro objetivo. Girar
en el lugar, o frenar a mirar un candidato, es mandar un objetivo en la
posición actual con otro ángulo.

    WAIT_LOCALIZATION -> EXPLORE -> LOOK_AROUND -> EXPLORE (siguiente punto)
    EXPLORE / LOOK_AROUND -(candidato)-> CONFIRM -(no se confirma)-> lo que hacía
    EXPLORE / LOOK_AROUND / CONFIRM -(cono confirmado)-> APPROACH -> DONE
    APPROACH -(inalcanzable)-> EXPLORE;  EXPLORE sin puntos -> EXHAUSTED
"""

from enum import Enum
import math

from slam.core.geometry import Pose2D, wrap_angle

from .config import MissionStateConfig


class State(Enum):
    WAIT_LOCALIZATION = "WAIT_LOCALIZATION"   # el navegador todavía no está ubicado
    EXPLORE = "EXPLORE"                       # yendo a un punto de observación
    LOOK_AROUND = "LOOK_AROUND"               # girando en el punto hacia lo que falta ver
    CONFIRM = "CONFIRM"                       # parado mirando un candidato
    APPROACH = "APPROACH"                     # yendo al cono confirmado
    DONE = "DONE"                             # llegó al cono
    EXHAUSTED = "EXHAUSTED"                   # recorrió todos los puntos sin encontrar uno


# Estados del navegador en los que no se puede confiar en la pose: se espera.
NAV_NOT_READY = {"WAIT_SCAN", "WAIT_INITIAL_POSE", "LOCALIZING", "LOST"}


class Outcome(Enum):
    PENDING = "PENDING"
    REACHED = "REACHED"
    FAILED = "FAILED"


class GoalMonitor:
    """Sigue un objetivo mandado al navegador hasta saber cómo terminó.

    El navegador vuelve a IDLE tanto si llegó como si rechazó o abandonó el
    objetivo. La diferencia se mira en la geometría, no en el texto de los
    eventos: si el robot está cerca del objetivo, llegó. Para no confundir un
    IDLE viejo con el final, se exige haber visto al navegador moverse, o que
    pasen `ack_timeout` segundos. El tiempo con la localización perdida no cuenta.
    """

    def __init__(self, goal: Pose2D, now: float, config: MissionStateConfig):
        self.goal = goal
        self.config = config
        self.sent = now
        self.last = now
        self.paused = 0.0
        self.started = False

    def update(self, now: float, pose, nav_state: str) -> Outcome:
        step, self.last = now - self.last, now
        if nav_state in NAV_NOT_READY:
            self.paused += step
            return Outcome.PENDING
        if nav_state != "IDLE":
            self.started = True
        elapsed = now - self.sent - self.paused
        if elapsed > self.config.goal_timeout:
            return Outcome.FAILED
        if nav_state == "IDLE" and (self.started or elapsed > self.config.ack_timeout):
            return Outcome.REACHED if self.near(pose) else Outcome.FAILED
        return Outcome.PENDING

    def near(self, pose) -> bool:
        if pose is None:
            return False
        c = self.config
        return (math.hypot(pose.x - self.goal.x, pose.y - self.goal.y) < c.reach_tolerance_xy
                and abs(wrap_angle(pose.yaw - self.goal.yaw)) < c.reach_tolerance_yaw)


class Mission:
    """Misión completa. `viewpoints` en orden de visita, `tracker` es un
    `CoverageTracker` y `approach(cone, pose)` devuelve la pose a la que ir para
    quedar frente al cono, o None si no hay ninguna alcanzable."""

    # Tope de giros en un mismo punto, por si la cobertura no avanza.
    MAX_LOOKS = 12

    def __init__(self, viewpoints, tracker, approach, config: MissionStateConfig):
        self.pending = [tuple(v) for v in viewpoints]
        self.tracker = tracker
        self.approach = approach
        self.config = config
        self.state = State.WAIT_LOCALIZATION
        self.monitor = None
        self.viewpoint = None       # punto de observación actual
        self.looks = 0              # giros hechos en el punto actual
        self.resume = None          # qué retomar si el candidato no se confirma
        self.candidate = None
        self.confirm_since = None
        self.cone = None
        self.attempts = 0
        self.dismissed = []         # candidatos que no se confirmaron
        self.unreachable = []       # conos confirmados a los que no se pudo llegar
        self.events = []            # (tiempo, texto)

    # ----- Paso -----

    def update(self, now: float, pose, nav_state: str, candidate=None, cone=None):
        """Un paso. Devuelve el objetivo nuevo a publicar (Pose2D) o None."""
        ready = pose is not None and nav_state not in NAV_NOT_READY
        if ready:
            # Lo que la cámara mira ahora queda visto, también en movimiento.
            self.tracker.observe(pose.x, pose.y, pose.yaw)
        if self.state in (State.DONE, State.EXHAUSTED):
            return None
        if self.state == State.WAIT_LOCALIZATION:
            return self._next_viewpoint(now, pose) if ready else None

        # Un cono confirmado o un candidato interrumpen la exploración.
        searching = self.state in (State.EXPLORE, State.LOOK_AROUND, State.CONFIRM)
        if ready and searching and cone is not None and not self._near_any(cone, self.unreachable):
            return self._start_approach(now, pose, cone)
        if (ready and self.state in (State.EXPLORE, State.LOOK_AROUND) and candidate is not None
                and not self._near_any(candidate, self.dismissed + self.unreachable)):
            return self._start_confirm(now, pose, candidate)

        outcome = self.monitor.update(now, pose, nav_state)
        if self.state == State.CONFIRM:
            if outcome == Outcome.PENDING:
                return None
            # Ya está mirando el candidato: se espera a que se confirme.
            if self.confirm_since is None:
                self.confirm_since = now
            if now - self.confirm_since < self.config.confirm_timeout:
                return None
            self._log(now, f"candidate ({self.candidate[0]:.2f}, {self.candidate[1]:.2f}) not confirmed")
            self.dismissed.append(self.candidate)
            return self._resume(now, pose)
        if outcome == Outcome.PENDING:
            return None
        if self.state == State.EXPLORE:
            if outcome == Outcome.FAILED:
                self._log(now, f"viewpoint ({self.viewpoint[0]:.2f}, {self.viewpoint[1]:.2f}) "
                               "not reached, skipped")
                return self._next_viewpoint(now, pose)
            self.looks = 0
            return self._look(now, pose)
        if self.state == State.LOOK_AROUND:
            if outcome == Outcome.FAILED:
                return self._next_viewpoint(now, pose)
            return self._look(now, pose)
        if self.state == State.APPROACH:
            if outcome == Outcome.REACHED:
                self._log(now, f"reached the cone at ({self.cone[0]:.2f}, {self.cone[1]:.2f})")
                self._go(State.DONE, now)
                return None
            self.attempts += 1
            if self.attempts < self.config.approach_attempts:
                return self._send_approach(now, pose)
            return self._give_up_cone(now, pose)
        raise RuntimeError(f"Unhandled state {self.state}")

    # ----- Exploración -----

    def _next_viewpoint(self, now, pose):
        """Siguiente punto que todavía tenga algo para ver; si no queda ninguno, EXHAUSTED."""
        while self.pending:
            x, y = self.pending.pop(0)
            if not self.tracker.headings_to_check(x, y, pose.yaw if pose else 0.0):
                self._log(now, f"viewpoint ({x:.2f}, {y:.2f}) already seen, skipped")
                continue
            self.viewpoint = (x, y)
            self._go(State.EXPLORE, now)
            # Llegar mirando en la dirección de avance: el primer giro suele ser chico.
            yaw = math.atan2(y - pose.y, x - pose.x) if pose else 0.0
            return self._send(now, Pose2D(x, y, yaw))
        self._log(now, "all viewpoints visited, no cone found")
        self._go(State.EXHAUSTED, now)
        return None

    def _look(self, now, pose):
        """Próximo giro en el punto actual hacia lo que falta ver, o el siguiente punto."""
        headings = self.tracker.headings_to_check(pose.x, pose.y, pose.yaw)
        if not headings or self.looks >= self.MAX_LOOKS:
            return self._next_viewpoint(now, pose)
        self.looks += 1
        self._go(State.LOOK_AROUND, now)
        return self._send(now, Pose2D(pose.x, pose.y, headings[0]))

    # ----- Candidato y cono -----

    def _start_confirm(self, now, pose, candidate):
        self.resume = self.state
        self.candidate = tuple(candidate)
        self.confirm_since = None
        self._log(now, f"red candidate at ({candidate[0]:.2f}, {candidate[1]:.2f}), stopping to look")
        self._go(State.CONFIRM, now)
        yaw = math.atan2(candidate[1] - pose.y, candidate[0] - pose.x)
        return self._send(now, Pose2D(pose.x, pose.y, yaw))

    def _resume(self, now, pose):
        """Vuelve a lo que hacía antes del candidato."""
        if self.resume == State.LOOK_AROUND:
            return self._look(now, pose)
        x, y = self.viewpoint
        self._go(State.EXPLORE, now)
        return self._send(now, Pose2D(x, y, math.atan2(y - pose.y, x - pose.x)))

    def _start_approach(self, now, pose, cone):
        if self.state != State.CONFIRM:
            self.resume = self.state
        self.cone = tuple(cone)
        self.attempts = 0
        self._log(now, f"cone confirmed at ({cone[0]:.2f}, {cone[1]:.2f})")
        self._go(State.APPROACH, now)
        return self._send_approach(now, pose)

    def _send_approach(self, now, pose):
        goal = self.approach(self.cone, pose)
        if goal is None:
            return self._give_up_cone(now, pose)
        return self._send(now, goal)

    def _give_up_cone(self, now, pose):
        """El cono no se puede alcanzar (por ejemplo, visto por un hueco sin paso): seguir buscando."""
        self._log(now, f"cone at ({self.cone[0]:.2f}, {self.cone[1]:.2f}) unreachable, exploring again")
        self.unreachable.append(self.cone)
        self.cone = None
        if self.viewpoint is None:
            return self._next_viewpoint(now, pose)
        return self._resume(now, pose)

    # ----- Internas -----

    def _near_any(self, point, points) -> bool:
        return any(math.hypot(point[0] - p[0], point[1] - p[1]) < self.config.same_cone_distance
                   for p in points)

    def _send(self, now, goal: Pose2D):
        self.monitor = GoalMonitor(goal, now, self.config)
        return goal

    def _go(self, state: State, now: float):
        if state != self.state:
            self._log(now, f"{self.state.value} -> {state.value}")
        self.state = state

    def _log(self, now, text):
        self.events.append((now, text))
