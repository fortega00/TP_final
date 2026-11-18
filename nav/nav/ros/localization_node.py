"""Nodo ROS 2 de localización. Solo conecta ROS con el filtro de nav/core/localization.py.

Entradas: `/scan`, la odometría y `/initialpose` (2D Pose Estimate de RViz).
Salidas:

- `/nav/pose`: pose del robot en el mapa, publicada con cada mensaje de
  odometría. Entre dos actualizaciones del filtro la pose avanza con la
  odometría: pose = corrección compuesta con odometría, donde la corrección
  (map -> odom) se recalcula en cada actualización;
- `/nav/particles` y `/nav/localization_status` (JSON) para ver y registrar;
- opcional: el TF `map -> odom` y el mapa en `/map`.

El nodo no arranca a estimar hasta recibir un `/initialpose`.
"""
from collections import deque
import json
import math
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from slam.core.geometry import Pose2D
from slam.core.odometry import OdometryBuffer
from slam.core.types import Scan

from ..core.config import NavConfig
from ..core.localization import MonteCarloLocalization
from ..core.map import load_map


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def pose_message(pose):
    message = Pose()
    message.position.x, message.position.y = float(pose.x), float(pose.y)
    message.orientation.z = math.sin(pose.yaw / 2)
    message.orientation.w = math.cos(pose.yaw / 2)
    return message


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization')
        defaults = {
            'map_file': '', 'config_file': '', 'scan_topic': '/scan', 'odom_topic': '/odom',
            'map_frame': 'map', 'odom_frame': 'odom', 'base_frame': 'base_footprint',
            # child_frame_id que traen los mensajes de odometría. Su pose se toma
            # como la pose de la base. Vacío: tiene que ser igual a base_frame.
            'odom_child_frame': '',
            'publish_tf': False, 'publish_map': False,
            'scan_wait_timeout': 2.0, 'max_particles_shown': 300,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.options = {name: self.get_parameter(name).value for name in defaults}
        if not self.options['map_file'] or not self.options['config_file']:
            raise ValueError('map_file and config_file are required')
        self.odom_child = self.options['odom_child_frame'] or self.options['base_frame']
        self.config = NavConfig.load_yaml(self.options['config_file'])
        self.grid = load_map(self.options['map_file'], self.config.sensor)
        self.core = MonteCarloLocalization(self.grid, self.config)
        self.odometry = OdometryBuffer(30.0)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.laser_mount = None          # dónde está el lidar respecto de la base
        self.pending = deque(maxlen=5)   # scans que esperan su odometría
        self.pending_initial = None      # /initialpose que espera un scan
        self.correction = None           # map -> odom, de la última actualización
        self.covariance = np.zeros((3, 3))
        self.state = 'WAITING_INITIAL_POSE'
        self.error = ''
        self.dropped_scans = 0

        persistent = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/nav/pose', 10)
        self.particles_pub = self.create_publisher(PoseArray, '/nav/particles', 10)
        self.status_pub = self.create_publisher(String, '/nav/localization_status', persistent)
        if self.options['publish_map']:
            self.map_pub = self.create_publisher(OccupancyGrid, '/map', persistent)
            self.publish_map()
        sensor_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, self.options['scan_topic'], self.on_scan, sensor_qos)
        self.create_subscription(Odometry, self.options['odom_topic'], self.on_odom, sensor_qos)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initial_pose, 10)
        # Timers con reloj de pared: siguen andando con el simulador o el bag en pausa.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.02, self.drain, clock=steady)
        self.create_timer(0.5, self.publish_status, clock=steady)
        self.get_logger().info(
            f'Localization with {self.config.localization.particles} particles on '
            f'{self.options["map_file"]}; waiting for /initialpose')

    # ----- Entradas -----

    def on_odom(self, message):
        if message.child_frame_id != self.odom_child:
            self.fail(f'Odometry child_frame_id is "{message.child_frame_id}", expected '
                      f'"{self.odom_child}"; set odom_child_frame or base_frame')
            return
        body = message.pose.pose
        pose = Pose2D(body.position.x, body.position.y, yaw_from_quaternion(body.orientation))
        if not self.odometry.append(seconds(message.header.stamp), pose):
            return
        if self.correction is not None:
            self.publish_pose(self.correction.compose(pose), message.header.stamp)

    def on_scan(self, message):
        if message.ranges:
            self.pending.append((message, time.monotonic()))

    def on_initial_pose(self, message):
        """Guarda el click de RViz. Se aplica con el próximo scan."""
        if message.header.frame_id != self.options['map_frame']:
            self.get_logger().error(f'/initialpose must be in frame {self.options["map_frame"]}, '
                                    f'got "{message.header.frame_id}"')
            return
        body = message.pose.pose
        cov = message.pose.covariance
        self.pending_initial = (Pose2D(body.position.x, body.position.y,
                                       yaw_from_quaternion(body.orientation)),
                                math.sqrt(max(cov[0], cov[7])), math.sqrt(cov[35]))
        self.error = ''
        self.get_logger().info('Initial pose received')

    # ----- Filtro -----

    def drain(self):
        """Procesa un scan de la cola con la odometría interpolada a su instante."""
        if self.error or not self.pending:
            return
        message, received = self.pending[0]
        try:
            if self.laser_mount is None:
                transform = self.tf_buffer.lookup_transform(
                    self.options['base_frame'], message.header.frame_id, Time())
                t = transform.transform
                self.laser_mount = Pose2D(t.translation.x, t.translation.y,
                                          yaw_from_quaternion(t.rotation))
            odom = self.odometry.at(seconds(message.header.stamp))
            if odom is None:
                raise ValueError('no odometry before and after the scan')
        except (TransformException, ValueError) as exc:
            # Todavía no llegó la odometría posterior al scan: esperar, no extrapolar.
            if time.monotonic() - received > self.options['scan_wait_timeout']:
                self.pending.popleft()
                self.dropped_scans += 1
                self.get_logger().warning(f'Discarding scan: {exc}', throttle_duration_sec=5.0)
            return
        self.pending.popleft()
        scan = Scan(np.asarray(message.ranges), message.angle_min, message.angle_increment,
                    message.range_min, message.range_max, seconds(message.header.stamp),
                    self.laser_mount)
        try:
            if self.pending_initial is not None:
                pose, std_xy, std_yaw = self.pending_initial
                self.pending_initial = None
                started = time.perf_counter()
                self.core.initialize(pose, std_xy, std_yaw)
                converged = self.core.converge(scan)
                self.state = self.state_for(scan)
                self.get_logger().info(
                    f'Initial burst: {"converged" if converged else "did NOT converge"} in '
                    f'{self.core.diagnostics["burst_iterations"]} iterations, '
                    f'{1000 * (time.perf_counter() - started):.0f} ms')
                self.core.process(scan, odom)      # fija la referencia de odometría
            elif self.core.poses is None or not self.core.process(scan, odom):
                return
            else:
                self.state = self.state_for(scan)
        except ValueError as exc:
            self.fail(f'{exc}; send a new /initialpose')
            return
        estimate, self.covariance = self.core.estimate()
        # map -> odom: la corrección que lleva la odometría al mapa.
        self.correction = estimate.compose(odom.inverse())
        self.publish_pose(estimate, message.header.stamp)
        self.publish_particles(message.header.stamp)

    def state_for(self, scan):
        """Estado que se informa después de una actualización del filtro."""
        if self.core.lost(scan):
            self.get_logger().warning(
                f'{100 * self.core.diagnostics["unexplained_fraction"]:.0f} % of the scan is far from every mapped wall: '
                'LOST, send a new /initialpose', throttle_duration_sec=5.0)
            return 'LOST'
        return 'LOCALIZED' if self.core.converged else 'NOT_CONVERGED'

    def fail(self, reason):
        if reason != self.error:
            self.get_logger().error(reason)
        self.error = reason
        self.state = 'LOST'
        self.correction = None
        self.core.poses = None

    # ----- Salidas -----

    def publish_pose(self, pose, stamp):
        message = PoseWithCovarianceStamped()
        message.header.frame_id, message.header.stamp = self.options['map_frame'], stamp
        message.pose.pose = pose_message(pose)
        for a, row in enumerate((0, 1, 5)):
            for b, col in enumerate((0, 1, 5)):
                message.pose.covariance[row * 6 + col] = float(self.covariance[a, b])
        self.pose_pub.publish(message)
        if self.options['publish_tf']:
            transform = TransformStamped()
            transform.header.frame_id = self.options['map_frame']
            transform.child_frame_id = self.options['odom_frame']
            transform.header.stamp = stamp
            transform.transform.translation.x = float(self.correction.x)
            transform.transform.translation.y = float(self.correction.y)
            transform.transform.rotation.z = math.sin(self.correction.yaw / 2)
            transform.transform.rotation.w = math.cos(self.correction.yaw / 2)
            self.tf_broadcaster.sendTransform(transform)

    def publish_particles(self, stamp):
        cloud = PoseArray()
        cloud.header.frame_id, cloud.header.stamp = self.options['map_frame'], stamp
        step = max(1, len(self.core.poses) // self.options['max_particles_shown'])
        cloud.poses = [pose_message(Pose2D(*row)) for row in self.core.poses[::step]]
        self.particles_pub.publish(cloud)

    def publish_map(self):
        message = OccupancyGrid()
        message.header.frame_id = self.options['map_frame']
        message.header.stamp = self.get_clock().now().to_msg()
        message.info.resolution = self.grid.config.resolution
        message.info.height, message.info.width = self.grid.log_odds.shape
        message.info.origin = pose_message(Pose2D(self.grid.config.origin_x,
                                                  self.grid.config.origin_y, 0))
        message.data = self.grid.occupancy().ravel().tolist()
        self.map_pub.publish(message)

    def publish_status(self):
        status = dict(self.core.diagnostics)
        status.update(state=self.state, error=self.error, dropped_scans=self.dropped_scans)
        if self.core.poses is not None:
            status.update(std_xy=self.core.std_xy, std_yaw=self.core.std_yaw)
        message = String()
        message.data = json.dumps(status)
        self.status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LocalizationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
