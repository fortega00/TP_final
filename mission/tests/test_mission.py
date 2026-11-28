"""Máquina de estados de la misión, contra un navegador falso. Sin ROS."""

import math
import unittest

from slam.core.geometry import Pose2D
from nav.core.config import PlannerConfig
from nav.core.planner import Costmap

from mission.core.config import ExplorationConfig, MissionStateConfig
from mission.core.exploration import CoverageTracker, Visibility, plan_viewpoints
from mission.core.mission import GoalMonitor, Mission, Outcome, State
from test_exploration import room


class FakeNavigator:
    """Imita al navegador de la etapa B: PLANNING, FOLLOWING unos pasos, y llega.

    `reject(goal)` decide si un objetivo se rechaza (vuelve a IDLE sin moverse).
    """

    def __init__(self, pose=Pose2D(1.0, 1.0, 0.0), steps=3, reject=lambda goal: False):
        self.pose = pose
        self.state = "IDLE"
        self.goal = None
        self.left = 0
        self.steps = steps
        self.reject = reject

    def send(self, goal):
        self.goal = goal
        if self.reject(goal):
            self.state, self.goal = "PLANNING", None
            self.left = -1          # el próximo paso vuelve a IDLE sin moverse
        else:
            self.state, self.left = "PLANNING", self.steps

    def step(self):
        if self.left == -1:
            self.state, self.left = "IDLE", 0
        elif self.goal is not None:
            self.left -= 1
            self.state = "FOLLOWING"
            if self.left <= 0:
                self.pose, self.goal, self.state = self.goal, None, "IDLE"


class FakeTracker:
    """Cobertura simplificada: cada punto pide una lista fija de direcciones.

    Una dirección queda vista cuando el robot mira hacia ella (a menos de 0,2
    rad) desde menos de 0,3 m del punto.
    """

    def __init__(self, needs):
        self.needs = {point: list(headings) for point, headings in needs.items()}
        self.observed = []

    def observe(self, x, y, yaw):
        for (px, py), headings in self.needs.items():
            if math.hypot(x - px, y - py) < 0.3:
                headings[:] = [h for h in headings if abs(math.remainder(h - yaw, 2 * math.pi)) > 0.2]

    def headings_to_check(self, x, y, yaw):
        for (px, py), headings in self.needs.items():
            if math.hypot(x - px, y - py) < 0.3:
                return list(headings)
        return []


def straight_approach(cone, pose):
    """Pose a 0,4 m del cono, del lado del robot, mirándolo."""
    angle = math.atan2(cone[1] - pose.y, cone[0] - pose.x)
    return Pose2D(cone[0] - 0.4 * math.cos(angle), cone[1] - 0.4 * math.sin(angle), angle)


def run(mission, nav, steps=200, dt=0.5, candidate_at=None, cone_at=None, nav_state=None):
    """Hace andar la misión con el navegador falso. Devuelve los objetivos mandados.

    `candidate_at(t, mission)` y `cone_at(t, mission)` dicen qué ve el detector en cada paso.
    """
    goals = []
    for i in range(steps):
        now = i * dt
        state = nav_state(now) if nav_state else nav.state
        goal = mission.update(now, nav.pose, state,
                              candidate=candidate_at(now, mission) if candidate_at else None,
                              cone=cone_at(now, mission) if cone_at else None)
        if goal is not None:
            goals.append(goal)
            nav.send(goal)
        if not nav_state or state not in ("LOST", "WAIT_INITIAL_POSE"):
            nav.step()
        if mission.state in (State.DONE, State.EXHAUSTED):
            break
    return goals


class GoalMonitorTests(unittest.TestCase):
    def setUp(self):
        self.config = MissionStateConfig()
        self.goal = Pose2D(2.0, 0.0, 0.0)

    def test_reached_after_moving(self):
        monitor = GoalMonitor(self.goal, 0.0, self.config)
        self.assertEqual(monitor.update(0.5, Pose2D(), "FOLLOWING"), Outcome.PENDING)
        self.assertEqual(monitor.update(1.0, Pose2D(2.05, 0.0, 0.1), "IDLE"), Outcome.REACHED)

    def test_idle_far_from_the_goal_is_a_failure(self):
        monitor = GoalMonitor(self.goal, 0.0, self.config)
        monitor.update(0.5, Pose2D(), "PLANNING")
        self.assertEqual(monitor.update(1.0, Pose2D(), "IDLE"), Outcome.FAILED)

    def test_old_idle_is_not_taken_as_the_end(self):
        # Justo después de mandar el objetivo el navegador todavía dice IDLE.
        monitor = GoalMonitor(self.goal, 0.0, self.config)
        self.assertEqual(monitor.update(0.5, Pose2D(), "IDLE"), Outcome.PENDING)
        self.assertEqual(monitor.update(self.config.ack_timeout + 0.5, Pose2D(), "IDLE"), Outcome.FAILED)

    def test_lost_localization_pauses_the_timeout(self):
        monitor = GoalMonitor(self.goal, 0.0, self.config)
        monitor.update(1.0, Pose2D(), "FOLLOWING")
        t = self.config.goal_timeout + 10
        self.assertEqual(monitor.update(t, Pose2D(), "LOST"), Outcome.PENDING)
        self.assertEqual(monitor.update(t + 1, Pose2D(), "FOLLOWING"), Outcome.PENDING)

    def test_timeout(self):
        monitor = GoalMonitor(self.goal, 0.0, self.config)
        monitor.update(1.0, Pose2D(), "FOLLOWING")
        self.assertEqual(monitor.update(self.config.goal_timeout + 2, Pose2D(), "FOLLOWING"), Outcome.FAILED)


class ExplorationFlowTests(unittest.TestCase):
    def setUp(self):
        self.points = [(3.0, 1.0), (5.0, 1.0)]
        self.tracker = FakeTracker({(3.0, 1.0): [math.pi / 2, -math.pi / 2], (5.0, 1.0): [0.0]})
        self.mission = Mission(self.points, self.tracker, straight_approach, MissionStateConfig())

    def test_waits_for_localization(self):
        nav = FakeNavigator()
        self.assertIsNone(self.mission.update(0.0, nav.pose, "WAIT_INITIAL_POSE"))
        self.assertIsNone(self.mission.update(0.5, None, "IDLE"))
        self.assertEqual(self.mission.state, State.WAIT_LOCALIZATION)

    def test_visits_every_viewpoint_and_looks_around(self):
        nav = FakeNavigator()
        goals = run(self.mission, nav)
        self.assertEqual(self.mission.state, State.EXHAUSTED)
        positions = [(round(g.x, 2), round(g.y, 2)) for g in goals]
        self.assertEqual(positions[0], (3.0, 1.0))
        self.assertIn((5.0, 1.0), positions)
        # En el primer punto gira hacia las dos direcciones pendientes.
        yaws = [round(g.yaw, 2) for g in goals if (round(g.x, 2), round(g.y, 2)) == (3.0, 1.0)]
        self.assertIn(round(math.pi / 2, 2), yaws)
        self.assertIn(round(-math.pi / 2, 2), yaws)

    def test_skips_a_viewpoint_already_seen(self):
        self.tracker.needs[(5.0, 1.0)] = []
        goals = run(self.mission, FakeNavigator())
        self.assertNotIn((5.0, 1.0), [(round(g.x, 2), round(g.y, 2)) for g in goals])
        self.assertTrue(any("already seen" in text for _, text in self.mission.events))

    def test_a_rejected_viewpoint_is_skipped(self):
        nav = FakeNavigator(reject=lambda goal: abs(goal.x - 3.0) < 0.01 and abs(goal.y - 1.0) < 0.01
                            and abs(goal.yaw) < 0.01)
        goals = run(self.mission, nav)
        self.assertEqual(self.mission.state, State.EXHAUSTED)
        self.assertTrue(any("not reached, skipped" in text for _, text in self.mission.events))
        self.assertIn((5.0, 1.0), [(round(g.x, 2), round(g.y, 2)) for g in goals])

    def test_lost_localization_waits_instead_of_failing(self):
        nav = FakeNavigator(steps=6)
        lost = lambda now: "LOST" if 1.0 <= now < 200.0 else nav.state
        run(self.mission, nav, steps=500, nav_state=lost)
        self.assertEqual(self.mission.state, State.EXHAUSTED)
        self.assertFalse(any("not reached" in text for _, text in self.mission.events))


class ConeFlowTests(unittest.TestCase):
    def setUp(self):
        self.points = [(3.0, 1.0), (5.0, 1.0)]
        self.tracker = FakeTracker({(3.0, 1.0): [math.pi / 2], (5.0, 1.0): [0.0]})
        self.mission = Mission(self.points, self.tracker, straight_approach, MissionStateConfig())

    def test_candidate_confirmed_goes_to_the_cone(self):
        cone = (4.0, 3.0)
        candidate_at = lambda now, m: cone if now >= 1.0 else None
        cone_at = lambda now, m: cone if m.state == State.CONFIRM and now >= 3.0 else None
        nav = FakeNavigator()
        run(self.mission, nav, candidate_at=candidate_at, cone_at=cone_at)
        self.assertEqual(self.mission.state, State.DONE)
        self.assertLess(math.hypot(nav.pose.x - cone[0], nav.pose.y - cone[1]), 0.45)
        # Antes de acercarse frenó a mirarlo: un objetivo en el lugar, apuntando al cono.
        states = [text for _, text in self.mission.events]
        self.assertTrue(any("CONFIRM" in s for s in states))

    def test_unconfirmed_candidate_resumes_exploration(self):
        candidate_at = lambda now, m: (4.0, 3.0) if 1.0 <= now < 1.6 else None
        run(self.mission, FakeNavigator(), candidate_at=candidate_at)
        self.assertEqual(self.mission.state, State.EXHAUSTED)
        self.assertTrue(any("not confirmed" in text for _, text in self.mission.events))
        self.assertEqual(self.mission.dismissed, [(4.0, 3.0)])

    def test_dismissed_candidate_does_not_stop_the_robot_again(self):
        seen = lambda now, m: (4.0, 3.0)        # el detector lo sigue viendo todo el tiempo
        run(self.mission, FakeNavigator(), candidate_at=seen)
        self.assertEqual(self.mission.state, State.EXHAUSTED)
        self.assertEqual(sum("stopping to look" in text for _, text in self.mission.events), 1)

    def test_unreachable_cone_is_remembered_and_exploration_continues(self):
        cone = (4.0, 3.0)
        mission = Mission(self.points, self.tracker, lambda c, p: None, MissionStateConfig())
        run(mission, FakeNavigator(), cone_at=lambda now, m: cone if now >= 1.0 else None)
        self.assertEqual(mission.state, State.EXHAUSTED)
        self.assertEqual(mission.unreachable, [cone])
        self.assertEqual(sum("cone confirmed" in text for _, text in mission.events), 1)

    def test_approach_is_retried_then_given_up(self):
        # El navegador rechaza todo objetivo cerca del cono: se intenta dos veces y se abandona.
        cone = (4.0, 3.0)
        rejects = lambda goal: math.hypot(goal.x - cone[0], goal.y - cone[1]) < 0.5
        mission = Mission(self.points, self.tracker, straight_approach,
                          MissionStateConfig(approach_attempts=2))
        goals = run(mission, FakeNavigator(reject=rejects), cone_at=lambda now, m: cone if now >= 1.0 else None)
        attempts = [g for g in goals if math.hypot(g.x - cone[0], g.y - cone[1]) < 0.5]
        self.assertEqual(len(attempts), 2)
        self.assertEqual(mission.unreachable, [cone])
        self.assertEqual(mission.state, State.EXHAUSTED)

    def test_cone_on_a_viewpoint_is_approached_not_hit(self):
        # El cono está justo sobre el segundo punto: el detector lo ve en el camino.
        cone = (5.0, 1.0)
        seen = lambda now, m: cone if m.viewpoint == (5.0, 1.0) and m.state == State.EXPLORE else None
        confirmed = lambda now, m: cone if m.state == State.CONFIRM else None
        nav = FakeNavigator()
        run(self.mission, nav, candidate_at=seen, cone_at=confirmed)
        self.assertEqual(self.mission.state, State.DONE)
        self.assertGreater(math.hypot(nav.pose.x - cone[0], nav.pose.y - cone[1]), 0.3)


class EndToEndTests(unittest.TestCase):
    """Plan real de puntos y cobertura real, con el navegador falso."""

    def test_explores_a_room_with_the_real_tracker(self):
        grid = room(8.0, 5.0)
        config = ExplorationConfig(coverage_target=0.95)
        costmap = Costmap(grid, PlannerConfig())
        plan = plan_viewpoints(grid, costmap, config, start=(1.0, 1.0))
        tracker = CoverageTracker(Visibility(grid, config), plan.coverable)
        mission = Mission(plan.viewpoints, tracker, straight_approach, MissionStateConfig())
        run(mission, FakeNavigator(pose=Pose2D(1.0, 1.0, 0.0)), steps=1000)
        self.assertEqual(mission.state, State.EXHAUSTED)
        self.assertGreater(tracker.seen_fraction(), 0.9)


if __name__ == "__main__":
    unittest.main()
