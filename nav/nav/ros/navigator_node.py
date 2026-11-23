"""Nodo ROS 2 de navegación. Solo conecta ROS con nav/core/navigator.py.

Entradas: `/nav/pose` y `/nav/localization_status` (del nodo de localización),
`/goal_pose` (2D Goal Pose de RViz), `/initialpose` (para saber que el usuario
corrigió la pose) y `/scan` (parada de seguridad y obstáculos que no están en el mapa).
Salidas: `/cmd_vel`, `/nav/plan`, `/nav/state` (JSON) y `/nav/obstacle_cells`.

Publica velocidades durante un viaje y un comando cero al detenerlo.
En espera no pisa al teleop.
"""
from concurrent.futures import ThreadPoolExecutor
import json
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import GridCells, Path as PathMsg
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from slam.core.geometry import Pose2D
from slam.core.types import Scan

from ..core.config import NavConfig
from ..core.map import load_map
from ..core.navigator import Navigator, State, safety_stop
from ..core.planner import Costmap
from ..core.scan_watchdog import ScanWatchdog
from .localization_node import pose_message, seconds, yaw_from_quaternion

ACTIVE = (State.ALIGNING, State.FOLLOWING, State.FINAL_ALIGN)
OBSTACLE_PUBLISH_PERIOD = 0.20  # segundos entre capturas publicadas


class NavigatorNode(Node):
    def __init__(self, planning_executor):
        super().__init__('navigator')
        defaults = {'map_file': '', 'config_file': '', 'scan_topic': '/scan',
                    'cmd_vel_topic': '/cmd_vel', 'map_frame': 'map',
                    'base_frame': 'base_footprint', 'rate': 10.0}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.options = {name: self.get_parameter(name).value for name in defaults}
        if not self.options['map_file'] or not self.options['config_file']:
            raise ValueError('map_file and config_file are required')
        self.config = NavConfig.load_yaml(self.options['config_file'])
        grid = load_map(self.options['map_file'], self.config.sensor)
        self.core = Navigator(Costmap(grid, self.config.planner), self.config,
                              planning_executor=planning_executor)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.scan_watchdog = ScanWatchdog(self.config.navigator.scan_timeout,
                                          self.config.sensor.min_range, self.use_sim_time)
        self.awaiting_recovery_goal = False
        self.pose = None
        self.pose_stamp = None
        self.localization = 'WAITING_INITIAL_POSE'
        self.blocked = False
        self.was_active = False
        self.shown_events = 0
        self.shown_path = None
        self.last_scan_callback = None
        self.last_obstacle_stamp = None

        persistent = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.velocity_pub = self.create_publisher(Twist, self.options['cmd_vel_topic'], 10)
        self.plan_pub = self.create_publisher(PathMsg, '/nav/plan', persistent)
        self.state_pub = self.create_publisher(String, '/nav/state', persistent)
        self.obstacles_pub = self.create_publisher(
            GridCells, '/nav/obstacle_cells', QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(PoseWithCovarianceStamped, '/nav/pose', self.on_pose, 10)
        self.create_subscription(String, '/nav/localization_status', self.on_status, persistent)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initial_pose, 10)
        self.create_subscription(LaserScan, self.options['scan_topic'], self.on_scan,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        control_clock = self.get_clock() if self.use_sim_time else Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / self.options['rate'], self.step, clock=control_clock)
        self.get_logger().info('Navigator ready; waiting for localization and /goal_pose')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ----- Entradas -----

    def on_pose(self, message):
        body = message.pose.pose
        self.pose = Pose2D(body.position.x, body.position.y, yaw_from_quaternion(body.orientation))
        self.pose_stamp = seconds(message.header.stamp)

    def on_status(self, message):
        self.localization = json.loads(message.data)['state']

    def on_goal(self, message):
        if message.header.frame_id != self.options['map_frame']:
            self.get_logger().error(f'/goal_pose must be in frame {self.options["map_frame"]}, '
                                    f'got "{message.header.frame_id}"')
            return
        now = self.now()
        if not self.check_scan(now, self.scan_time(now)):
            self.get_logger().warning('Goal rejected: waiting for usable lidar data')
            self.publish_navigation()
            return
        if self.awaiting_recovery_goal and (self.localization != 'LOCALIZED'
                                            or self.fresh_pose(now) is None):
            self.get_logger().warning('Goal rejected: waiting for a recent localized pose after lidar loss')
            return
        body = message.pose
        self.core.set_goal(Pose2D(body.position.x, body.position.y,
                                  yaw_from_quaternion(body.orientation)), now)
        self.awaiting_recovery_goal = False

    def on_initial_pose(self, message):
        del message
        # La pose anterior ya no vale: esperar la que publique la localización.
        self.pose = self.pose_stamp = None
        self.last_obstacle_stamp = None
        self.core.on_reinitialized(self.now())

    def on_scan(self, message):
        """Parada de seguridad y obstáculos que no están en el mapa."""
        callback_started = time.monotonic()
        now = self.now()
        received = self.scan_time(now)
        # Si volvió entre dos ticks luego de un silencio largo, el viaje igual
        # queda cancelado antes de aceptar el nuevo scan.
        if not self.check_scan(now, received):
            self.publish_navigation()
        self.last_scan_callback = callback_started
        if not message.ranges or not message.header.frame_id:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.options['base_frame'], message.header.frame_id, Time())
            t = transform.transform
            mount = Pose2D(t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation))
            scan = Scan(message.ranges, message.angle_min, message.angle_increment,
                        message.range_min, message.range_max, seconds(message.header.stamp),
                        mount)
        except (TransformException, ValueError):
            return
        if not self.scan_watchdog.accept(scan, received):
            return
        self.blocked = safety_stop(scan, self.config)
        # Los obstáculos se anotan solo con una pose confiable y reciente.
        pose = self.fresh_pose(now)
        if self.localization == 'LOCALIZED' and pose is not None:
            self.core.observe(pose, scan)
            if (self.last_obstacle_stamp is None
                    or scan.stamp - self.last_obstacle_stamp >= OBSTACLE_PUBLISH_PERIOD):
                self.publish_obstacles(message.header.stamp)
                self.last_obstacle_stamp = scan.stamp

    # ----- Lazo de control -----

    def scan_time(self, ros_now):
        return ros_now if self.use_sim_time else time.monotonic()

    def fresh_pose(self, now):
        if self.pose_stamp is None or now - self.pose_stamp > self.config.navigator.pose_timeout:
            return None
        return self.pose

    def check_scan(self, now, scan_now):
        """Frena antes de usar una velocidad o un objetivo sin un scan utilizable."""
        clock_was_rewound = self.scan_watchdog.clock_rewound
        if self.scan_watchdog.ready(scan_now):
            self.core.scan_available(now)
            return True
        clock_rewound = self.scan_watchdog.clock_rewound
        if clock_rewound and not clock_was_rewound:
            self.get_logger().warning('ROS time jumped backwards; restart navigation and localization')
        if self.core.state == State.WAIT_SCAN:
            return False

        had_goal = self.core.goal is not None
        if had_goal or self.was_active:
            self.velocity_pub.publish(Twist())
            self.was_active = False
        self.core.wait_for_scan(now)
        self.blocked = False
        silence = self.scan_watchdog.silence(scan_now)
        if silence is not None and not clock_rewound:
            self.awaiting_recovery_goal = True
            action = 'active goal canceled' if had_goal else 'no active goal'
            age = (f', {scan_now - self.scan_watchdog.last_stamp:.3f} s since scan capture'
                   if self.use_sim_time else '')
            callback_age = (f', {time.monotonic() - self.last_scan_callback:.3f} wall s '
                            'since previous scan callback' if self.last_scan_callback is not None else '')
            self.get_logger().warning(f'Lidar unavailable on {self.options["scan_topic"]}: '
                                      f'{max(0.0, silence):.3f} s since last usable reception{age}{callback_age} '
                                      f'(timeout {self.scan_watchdog.timeout:.2f} s); {action}')
        return False

    def step(self):
        now = self.now()
        if not self.check_scan(now, self.scan_time(now)):
            self.publish_navigation()
            return
        pose = self.fresh_pose(now)
        localization = self.localization if pose is not None or self.localization != 'LOCALIZED' \
            else 'NOT_CONVERGED'
        v, w = self.core.update(now, pose, localization, self.blocked)
        # Un cálculo que terminó después del plazo no puede publicar movimiento.
        now = self.now()
        if not self.check_scan(now, self.scan_time(now)):
            self.publish_navigation()
            return
        active = self.core.state in ACTIVE
        if active or self.was_active:
            command = Twist()
            command.linear.x, command.angular.z = float(v), float(w)
            self.velocity_pub.publish(command)
        self.was_active = active
        self.publish_navigation()

    def publish_navigation(self):
        for _, text in self.core.events[self.shown_events:]:
            self.get_logger().info(text)
        self.shown_events = len(self.core.events)
        if self.core.path is not self.shown_path:
            self.shown_path = self.core.path
            self.publish_plan()
        status = String()
        status.data = json.dumps(dict(state=self.core.state.value, blocked=self.blocked,
                                      localization=self.localization,
                                      events=[text for _, text in self.core.events[-5:]]))
        self.state_pub.publish(status)

    def publish_plan(self):
        message = PathMsg()
        message.header.frame_id = self.options['map_frame']
        message.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.core.path:
            point = PoseStamped()
            point.header = message.header
            point.pose = pose_message(Pose2D(x, y, 0.0))
            message.poses.append(point)
        self.plan_pub.publish(message)

    def publish_obstacles(self, stamp):
        message = GridCells()
        message.header.frame_id = self.options['map_frame']
        message.header.stamp = stamp
        resolution = self.core.costmap.grid.config.resolution
        message.cell_width = message.cell_height = resolution
        message.cells = [Point(x=float(x), y=float(y)) for x, y in self.core.obstacles.points()]
        self.obstacles_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    planning_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='nav-planner')
    try:
        node = NavigatorNode(planning_executor)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.velocity_pub.publish(Twist())   # frenar al salir
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        planning_executor.shutdown(cancel_futures=True)
