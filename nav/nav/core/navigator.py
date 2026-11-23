"""Máquina de estados de la navegación. Sin ROS.

Decide qué hace el robot en cada instante a partir de: la pose estimada, el
estado de la localización, el objetivo pedido y lo que ve el lidar adelante.
Devuelve velocidades. Estados y alternativas en docs/decisiones/009.

Obstáculos que no están en el mapa (docs/decisiones/008): el lidar los anota en
una capa aparte. Si uno tapa el camino, el robot frena y planifica de nuevo con
esa capa sumada al mapa. Si no queda camino, espera a que se despeje.
"""

from concurrent.futures import Executor
from enum import Enum
import math
import time
import numpy as np

from slam.core.geometry import Pose2D, wrap_angle
from slam.core.types import Scan

from .config import NavConfig
from .follower import PurePursuit, rotate_command
from .obstacles import ObstacleLayer
from .planner import Costmap, PlanningError, plan
from .scan_watchdog import valid_returns


class State(Enum):
    WAIT_SCAN = "WAIT_SCAN"                   # sin datos utilizables del lidar
    WAIT_INITIAL_POSE = "WAIT_INITIAL_POSE"   # el filtro no arrancó o se perdió
    LOCALIZING = "LOCALIZING"                 # hay pose inicial, la nube no convergió
    IDLE = "IDLE"                             # ubicado, sin objetivo
    PLANNING = "PLANNING"
    ALIGNING = "ALIGNING"                     # gira hacia el camino
    FOLLOWING = "FOLLOWING"
    FINAL_ALIGN = "FINAL_ALIGN"               # gira al ángulo del objetivo
    BLOCKED = "BLOCKED"                       # parado por un obstáculo
    LOST = "LOST"                             # parado, la localización dejó de ser confiable


def safety_stop(scan: Scan, config: NavConfig) -> bool:
    """Parada de seguridad: el lidar ve algo dentro del rectángulo delante del robot."""
    ranges = np.asarray(scan.ranges)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    valid = valid_returns(scan, config.sensor.min_range)
    beam = angles[valid] + scan.sensor_pose.yaw
    x = scan.sensor_pose.x + ranges[valid] * np.cos(beam)
    y = scan.sensor_pose.y + ranges[valid] * np.sin(beam)
    c = config.navigator
    return bool(np.any((x > 0) & (x < c.stop_distance) & (np.abs(y) < c.stop_half_width)))


MOVING = (State.PLANNING, State.ALIGNING, State.FOLLOWING, State.FINAL_ALIGN, State.BLOCKED, State.LOST)


def find_route(static_costmap, obstacles, start, goal):
    """Devuelve el camino, o None si lo impiden solo obstáculos nuevos."""
    if not obstacles.any():
        return plan(static_costmap, start, goal)
    try:
        dynamic_costmap = Costmap(static_costmap.grid, static_costmap.config, obstacles)
        return plan(dynamic_costmap, start, goal)
    except PlanningError as exc:
        try:
            plan(static_costmap, start, goal)
        except PlanningError:
            raise exc from None
        return None


class Navigator:
    def __init__(self, costmap: Costmap, config: NavConfig, *,
                 planning_executor: Executor | None = None):
        self.costmap = costmap          # solo el mapa estático
        self.config = config
        self.obstacles = ObstacleLayer(costmap.grid, config.sensor, config.obstacles)
        self.state = State.WAIT_INITIAL_POSE
        self.goal = None        # objetivo vigente o guardado (Pose2D)
        self.follower = None
        self.path = []
        self.events = []        # (tiempo, texto): registro de transiciones
        self._since = 0.0       # instante en que se entró al estado actual
        self._safety = False        # BLOCKED por la parada de seguridad (y no por falta de camino)
        self._blocked_since = None  # desde cuándo un obstáculo no deja avanzar
        self._needs_initial_pose = False   # la localización no se recuperó: hace falta un click nuevo
        self._planning_executor = planning_executor
        self._pending_plan = None
        self._plan_requested_at = None

    # ----- Entradas -----

    def set_goal(self, goal: Pose2D, now: float):
        """Objetivo nuevo. Reemplaza al anterior, también con el robot en marcha."""
        self._discard_plan()
        self.goal = goal
        self._blocked_since = None
        self._log(now, f"goal ({goal.x:.2f}, {goal.y:.2f}, {math.degrees(goal.yaw):.0f} deg)")
        if self.state in (State.IDLE,) + MOVING and self.state != State.LOST:
            self._go(State.PLANNING, now)
        # En WAIT_INITIAL_POSE, LOCALIZING o LOST queda guardado y se planifica
        # cuando el robot esté ubicado.

    def observe(self, pose: Pose2D, scan: Scan):
        """Scan nuevo tomado desde `pose`: actualiza los obstáculos que no están en el mapa."""
        self.obstacles.observe(pose, scan)

    def wait_for_scan(self, now: float):
        """Cancela el viaje ante la falta de datos del lidar."""
        self.goal, self.path, self.follower = None, [], None
        self._blocked_since = None
        self._safety = False
        self._go(State.WAIT_SCAN, now)

    def scan_available(self, now: float):
        """Tras la espera, vuelve a comprobar la localización antes de planear."""
        if self.state == State.WAIT_SCAN:
            self._go(State.WAIT_INITIAL_POSE, now)

    def on_reinitialized(self, now: float):
        """Llegó un /initialpose nuevo. Con el robot en marcha: frenar y replanificar.

        El objetivo se conserva: el usuario estaba corrigiendo la pose, no
        cancelando el viaje.
        """
        self._log(now, "new initial pose")
        self._needs_initial_pose = False
        # Los obstáculos anotados con la pose anterior quedaron en lugares equivocados.
        self.obstacles = ObstacleLayer(self.costmap.grid, self.config.sensor, self.config.obstacles)
        # Desde WAIT_INITIAL_POSE el estado sigue al de la localización en update().
        if self.state not in (State.WAIT_INITIAL_POSE, State.WAIT_SCAN):
            self._go(State.LOCALIZING, now)

    def update(self, now: float, pose, localization: str, blocked: bool):
        """Un paso. Devuelve (v, w).

        `pose` es la pose estimada o None. `localization` es el estado del nodo
        de localización: WAITING_INITIAL_POSE, NOT_CONVERGED, LOCALIZED o LOST.
        `blocked` dice si el lidar ve algo en el rectángulo de seguridad.
        """
        if self.state == State.WAIT_SCAN:
            return 0.0, 0.0
        located = localization == "LOCALIZED" and pose is not None
        # Transiciones globales por el estado de la localización.
        if self._needs_initial_pose or localization in ("WAITING_INITIAL_POSE", "LOST"):
            if self.state != State.WAIT_INITIAL_POSE:
                self._go(State.WAIT_INITIAL_POSE, now)
            return 0.0, 0.0
        if self.state == State.WAIT_INITIAL_POSE:
            self._go(State.LOCALIZING, now)
        if self.state == State.LOCALIZING:
            if located:
                self._go(State.PLANNING if self.goal is not None else State.IDLE, now)
            return 0.0, 0.0
        if not located:
            # La nube se abrió con el robot ubicado: frenar y esperar.
            if self.state in (State.ALIGNING, State.FOLLOWING, State.FINAL_ALIGN, State.BLOCKED,
                              State.PLANNING):
                self._go(State.LOST, now)
            elif self.state == State.IDLE:
                self._go(State.LOCALIZING, now)
            if self.state == State.LOST and now - self._since > self.config.navigator.lost_timeout:
                self._log(now, "localization did not recover in time, send a new initial pose")
                self._needs_initial_pose = True
                self._go(State.WAIT_INITIAL_POSE, now)
            return 0.0, 0.0
        if self.state == State.LOST:
            self._go(State.PLANNING, now)

        if self.state == State.IDLE:
            return 0.0, 0.0
        if self.state == State.PLANNING:
            self._plan(now, pose)
            return 0.0, 0.0

        f = self.config.follower
        if self.state == State.BLOCKED:
            c = self.config.navigator
            if now - self._blocked_since > c.blocked_timeout:
                self._log(now, "obstacle did not clear in time, goal dropped")
                self.goal = None
                self._go(State.IDLE, now)
            elif self._safety:
                # Parada de seguridad: se retoma cuando se despeja, o se rodea el
                # obstáculo en cuanto la capa lo anotó sobre el camino.
                if not blocked or self._path_blocked():
                    self._go(State.PLANNING, now)
            elif now - self._since >= c.retry_period:
                # Sin camino por obstáculos nuevos: cada tanto se prueba de nuevo.
                self._go(State.PLANNING, now)
            return 0.0, 0.0
        if self.state in (State.ALIGNING, State.FOLLOWING) and self._path_blocked():
            self._log(now, "path blocked by an unmapped obstacle, replanning")
            self._go(State.PLANNING, now)
            return 0.0, 0.0
        if self.state == State.ALIGNING:
            error = self.follower.heading_error(pose)
            if abs(error) < f.align_done:
                self._go(State.FOLLOWING, now)
                return 0.0, 0.0
            return 0.0, rotate_command(error, f)
        if self.state == State.FOLLOWING:
            if self.follower.distance_to_goal(pose) < f.goal_tolerance_xy:
                self._go(State.FINAL_ALIGN, now)
                return 0.0, 0.0
            if blocked:
                # La parada de seguridad solo aplica al avanzar, no a los giros.
                self._block(now, safety=True)
                return 0.0, 0.0
            if abs(self.follower.heading_error(pose)) > f.align_start:
                self._go(State.ALIGNING, now)
                return 0.0, 0.0
            self._blocked_since = None      # está avanzando
            return self.follower.command(pose)
        if self.state == State.FINAL_ALIGN:
            error = wrap_angle(self.goal.yaw - pose.yaw)
            if abs(error) < f.goal_tolerance_yaw:
                self._log(now, f"goal reached, {100 * self.follower.distance_to_goal(pose):.0f} cm "
                               f"and {math.degrees(error):.1f} deg from it")
                self.goal = None
                self._go(State.IDLE, now)
                return 0.0, 0.0
            return 0.0, rotate_command(error, f)
        raise RuntimeError(f"Unhandled state {self.state}")

    # ----- Internas -----

    def _plan(self, now, pose):
        if self._planning_executor is not None:
            if self._pending_plan is None:
                self._plan_requested_at = time.monotonic()
                # La máscara es un array nuevo. El hilo solo lee el mapa estático
                # y esta captura de los obstáculos, mientras el lazo recibe scans.
                self._pending_plan = self._planning_executor.submit(
                    find_route, self.costmap, self.obstacles.cells(),
                    (pose.x, pose.y), (self.goal.x, self.goal.y))
                return
            if not self._pending_plan.done():
                return
            elapsed = time.monotonic() - self._plan_requested_at
            self._log(now, f"Planning result ready after {elapsed:.3f} s (wall time)")

        try:
            if self._pending_plan is None:
                path = find_route(self.costmap, self.obstacles.cells(),
                                  (pose.x, pose.y), (self.goal.x, self.goal.y))
            else:
                path = self._pending_plan.result()
        except PlanningError as exc:
            self._log(now, f"goal rejected: {exc}")
            self.goal, self.path, self.follower = None, [], None
            self._go(State.IDLE, now)
            return
        if path is None:
            self.path, self.follower = [], None
            self._block(now, safety=False)
            return
        self.path = path
        self.follower = PurePursuit(self.path, self.config.follower)
        self._go(State.ALIGNING, now)

    def _discard_plan(self):
        if self._pending_plan is not None:
            self._pending_plan.cancel()
            self._pending_plan = None
            self._plan_requested_at = None

    def _block(self, now, safety: bool):
        self._safety = safety
        if self._blocked_since is None:
            self._blocked_since = now
        self._go(State.BLOCKED, now)

    def _path_blocked(self) -> bool:
        """Algún obstáculo nuevo quedó a menos del radio inflado del camino que falta recorrer."""
        cells = self.obstacles.points()
        if not len(cells):
            return False
        ahead = self.follower.path[self.follower.index:]
        distance = np.hypot(*(ahead[:, None, :] - cells[None, :, :]).transpose(2, 0, 1))
        return bool(distance.min() <= self.config.planner.robot_radius + self.config.planner.safety_margin)

    def _go(self, state: State, now: float):
        if state != self.state:
            if self.state == State.PLANNING:
                self._discard_plan()
            self._log(now, f"{self.state.value} -> {state.value}")
        self.state = state
        self._since = now

    def _log(self, now, text):
        self.events.append((now, text))
