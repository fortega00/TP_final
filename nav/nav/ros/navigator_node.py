"""Nodo ROS 2 de navegación. Solo conecta ROS con nav/core/navigator.py.

Entradas: `/nav/pose` y `/nav/localization_status` (del nodo de localización),
`/goal_pose` (2D Goal Pose de RViz), `/initialpose` (para saber que el usuario
corrigió la pose) y `/scan` (parada de seguridad y obstáculos que no están en el mapa).
Salidas: `/cmd_vel`, `/nav/plan` y `/nav/state` (JSON).

Solo publica velocidades mientras está llevando al robot a un objetivo. Parado
no publica nada, así no pisa al teleop.
"""
import json

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path as PathMsg
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from slam.core.geometry import Pose2D
from slam.core.types import Scan

from ..core.config import NavConfig
from ..core.map import load_map
from ..core.navigator import Navigator, State, safety_stop
from ..core.planner import Costmap
from .localization_node import pose_message, seconds, yaw_from_quaternion

ACTIVE = (State.ALIGNING, State.FOLLOWING, State.FINAL_ALIGN)


class NavigatorNode(Node):
    def __init__(self):
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
        self.core = Navigator(Costmap(grid, self.config.planner), self.config)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.laser_mount = None
        self.pose = None
        self.pose_stamp = None
        self.localization = 'WAITING_INITIAL_POSE'
        self.blocked = False
        self.was_active = False
        self.shown_events = 0
        self.shown_path = None

        persistent = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.velocity_pub = self.create_publisher(Twist, self.options['cmd_vel_topic'], 10)
        self.plan_pub = self.create_publisher(PathMsg, '/nav/plan', persistent)
        self.state_pub = self.create_publisher(String, '/nav/state', persistent)
        self.create_subscription(PoseWithCovarianceStamped, '/nav/pose', self.on_pose, 10)
        self.create_subscription(String, '/nav/localization_status', self.on_status, persistent)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initial_pose, 10)
        self.create_subscription(LaserScan, self.options['scan_topic'], self.on_scan,
                                 QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(1.0 / self.options['rate'], self.step)
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
        body = message.pose
        self.core.set_goal(Pose2D(body.position.x, body.position.y,
                                  yaw_from_quaternion(body.orientation)), self.now())

    def on_initial_pose(self, message):
        del message
        # La pose anterior ya no vale: esperar la que publique la localización.
        self.pose = self.pose_stamp = None
        self.core.on_reinitialized(self.now())

    def on_scan(self, message):
        """Parada de seguridad y obstáculos que no están en el mapa."""
        if self.laser_mount is None:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.options['base_frame'], message.header.frame_id, Time())
            except TransformException:
                return
            t = transform.transform
            self.laser_mount = Pose2D(t.translation.x, t.translation.y,
                                      yaw_from_quaternion(t.rotation))
        if not message.ranges:
            return
        scan = Scan(np.asarray(message.ranges), message.angle_min, message.angle_increment,
                    message.range_min, message.range_max, seconds(message.header.stamp),
                    self.laser_mount)
        self.blocked = safety_stop(scan, self.config)
        # Los obstáculos se anotan solo con una pose confiable y reciente.
        if (self.localization == 'LOCALIZED' and self.pose_stamp is not None
                and self.now() - self.pose_stamp <= self.config.navigator.pose_timeout):
            self.core.observe(self.pose, scan)

    # ----- Lazo de control -----

    def step(self):
        now = self.now()
        pose = self.pose
        # Una pose vieja no sirve para manejar: se trata como si no hubiera.
        if self.pose_stamp is None or now - self.pose_stamp > self.config.navigator.pose_timeout:
            pose = None
        localization = self.localization if pose is not None or self.localization != 'LOCALIZED' \
            else 'NOT_CONVERGED'
        v, w = self.core.update(now, pose, localization, self.blocked)
        active = self.core.state in ACTIVE
        if active or self.was_active:
            command = Twist()
            command.linear.x, command.angular.z = float(v), float(w)
            self.velocity_pub.publish(command)
        self.was_active = active
        for stamp, text in self.core.events[self.shown_events:]:
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


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NavigatorNode()
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
