"""Nodo de la misión de la etapa C: explorar, detectar conos rojos e ir hasta uno.

Conecta `mission.core` con ROS; la lógica está en el núcleo.

Entradas: `/nav/pose` y `/nav/state` (etapa B), la imagen de color, la de
profundidad y el `camera_info` de la cámara, y TF hasta el frame de la cámara.
Salidas: `/goal_pose` (lo lee el navegador de la etapa B), `/mission/state`
(JSON), `/mission/markers` (puntos de observación y conos candidatos para RViz)
y `/mission/debug_image` (detecciones dibujadas sobre la imagen).

El plan de exploración se calcula al llegar la primera pose con el navegador
ubicado, desde donde está el robot. La cámara se procesa a `camera_rate` Hz,
cada imagen de color con la profundidad más cercana en el tiempo.
"""

import json

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from slam.core.geometry import Pose2D
from nav.core.config import NavConfig
from nav.core.map import free_cells, load_map
from nav.ros.localization_node import pose_message, seconds, yaw_from_quaternion

from ..core.config import MissionConfig
from ..core.cone_locator import ConeTracker, approach_pose, camera_position_yaw, measure
from ..core.detector import detect_cones
from ..core.exploration import (CoverageTracker, ExplorationError, Visibility, main_component,
                                plan_viewpoints, prepare_map)
from ..core.mission import NAV_NOT_READY, Mission
from ..core.throttle import PairedThrottle
from .conversions import depth_in_meters, intrinsics_from_k, match_size, transform_matrix


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission')
        defaults = {'map_file': '', 'config_file': '', 'nav_config_file': '', 'map_frame': 'map',
                    'color_topic': '/cam_1/color/image_raw', 'depth_topic': '/cam_1/depth/image_raw',
                    'camera_info_topic': '/cam_1/color/camera_info',
                    'camera_rate': 5.0, 'pair_tolerance': 0.05, 'rate': 5.0}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.options = {name: self.get_parameter(name).value for name in defaults}
        for name in ('map_file', 'config_file', 'nav_config_file'):
            if not self.options[name]:
                raise ValueError(f'{name} is required')
        self.config = MissionConfig.load_yaml(self.options['config_file'])
        nav_config = NavConfig.load_yaml(self.options['nav_config_file'])
        # Mismo mapa y mismo mapa de costos que el navegador, con mesas y bancos detectados.
        original = load_map(self.options['map_file'], nav_config.sensor)
        self.grid, self.costmap = prepare_map(original, nav_config.planner, self.config.exploration)
        self.cones = ConeTracker(self.config.locator, blocked=~free_cells(self.grid), grid=self.grid)
        self.mission = None
        self.viewpoints = []
        self.pose = None
        self.nav_state = 'WAIT_INITIAL_POSE'
        self.intrinsics = None
        self.waiting_color = None
        self.shown_events = 0
        self.plan_error = None
        self.throttle = PairedThrottle(1.0 / self.options['camera_rate'], self.options['pair_tolerance'])
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        persistent = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.state_pub = self.create_publisher(String, '/mission/state', persistent)
        self.markers_pub = self.create_publisher(MarkerArray, '/mission/markers', persistent)
        self.debug_pub = self.create_publisher(Image, '/mission/debug_image', 1)
        self.create_subscription(PoseWithCovarianceStamped, '/nav/pose', self.on_pose, 10)
        self.create_subscription(String, '/nav/state', self.on_nav_state, persistent)
        self.create_subscription(CameraInfo, self.options['camera_info_topic'], self.on_camera_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, self.options['color_topic'], self.on_color, qos_profile_sensor_data)
        self.create_subscription(Image, self.options['depth_topic'], self.on_depth, qos_profile_sensor_data)
        self.create_timer(1.0 / self.options['rate'], self.step)
        self.get_logger().info('Mission ready; waiting for localization')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ----- Entradas -----

    def on_pose(self, message):
        body = message.pose.pose
        self.pose = Pose2D(body.position.x, body.position.y, yaw_from_quaternion(body.orientation))

    def on_nav_state(self, message):
        self.nav_state = json.loads(message.data)['state']

    def on_camera_info(self, message):
        self.intrinsics = intrinsics_from_k(message.k, message.width)

    def on_color(self, message):
        self.handle(self.throttle.on_primary(seconds(message.header.stamp), message))

    def on_depth(self, message):
        self.handle(self.throttle.on_secondary(seconds(message.header.stamp), message))

    def handle(self, items):
        """Junta cada color que pasó el limitador con su profundidad y procesa el par."""
        for kind, message in items:
            if kind == 'primary':
                self.waiting_color = message
            elif self.waiting_color is not None:
                self.process(self.waiting_color, message)
                self.waiting_color = None

    def process(self, color, depth):
        """Detectar, ubicar en el mapa y actualizar el filtro de Bayes con un par de imágenes."""
        if self.intrinsics is None or self.mission is None or self.nav_state in NAV_NOT_READY:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.options['map_frame'], color.header.frame_id, Time.from_msg(color.header.stamp),
                timeout=Duration(seconds=0.1))
        except TransformException as exc:
            self.get_logger().warning(f'No camera pose: {exc}', throttle_duration_sec=5.0)
            return
        t = transform.transform
        map_T_camera = transform_matrix((t.translation.x, t.translation.y, t.translation.z),
                                        (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))
        image = self.bridge.imgmsg_to_cv2(color, desired_encoding='bgr8')
        meters = match_size(depth_in_meters(self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough'),
                                            depth.encoding), image.shape)
        detections = detect_cones(image, self.config.detector)
        points = []
        for detection in detections:
            point = measure(detection, meters, self.intrinsics, map_T_camera, self.config.locator)
            if point is not None:
                points.append(point)
            # Verde: ubicado en el mapa; amarillo: con forma de cono pero sin profundidad o sin tamaño de cono.
            x, y, w, h = detection.box
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0) if point else (0, 255, 255), 2)
        camera_xy, camera_yaw = camera_position_yaw(map_T_camera)
        self.cones.update(points, camera_xy, camera_yaw, self.intrinsics.horizontal_fov)
        if self.debug_pub.get_subscription_count():
            debug = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            debug.header = color.header
            self.debug_pub.publish(debug)

    # ----- Lazo -----

    def step(self):
        if self.pose is None:
            return
        if self.mission is None and not self.start_mission():
            return
        goal = self.mission.update(self.now(), self.pose, self.nav_state,
                                   candidate=self.cones.candidate(), cone=self.cones.confirmed())
        if goal is not None:
            self.publish_goal(goal)
        for _, text in self.mission.events[self.shown_events:]:
            self.get_logger().info(text)
        self.shown_events = len(self.mission.events)
        self.publish_state()
        self.publish_markers()

    def start_mission(self) -> bool:
        """Plan de exploración desde donde está el robot, cuando el navegador está ubicado."""
        if self.nav_state in NAV_NOT_READY:
            return False
        exploration = self.config.exploration
        start = (self.pose.x, self.pose.y)
        try:
            plan = plan_viewpoints(self.grid, self.costmap, exploration, start=start)
        except ExplorationError as exc:
            if str(exc) != self.plan_error:
                self.get_logger().error(f'Cannot plan the exploration: {exc}')
            self.plan_error = str(exc)
            return False
        reachable = main_component(self.costmap.traversable, self.costmap.cell(*start))
        tracker = CoverageTracker(Visibility(self.grid, exploration), plan.coverable)
        locator = self.config.locator
        self.mission = Mission(plan.viewpoints, tracker,
                               lambda cone, pose: approach_pose(cone, pose, self.costmap, reachable, locator),
                               self.config.mission)
        self.viewpoints = plan.viewpoints
        self.get_logger().info(f'Exploration plan: {json.dumps(plan.summary())}')
        return True

    # ----- Salidas -----

    def publish_goal(self, goal: Pose2D):
        message = PoseStamped()
        message.header.frame_id = self.options['map_frame']
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose = pose_message(goal)
        self.goal_pub.publish(message)

    def publish_state(self):
        m = self.mission
        status = dict(state=m.state.value, nav_state=self.nav_state,
                      viewpoints_left=len(m.pending), seen=round(m.tracker.seen_fraction(), 3),
                      cones=[dict(x=round(t.x, 2), y=round(t.y, 2), p=round(t.probability, 3))
                             for t in self.cones.tracks],
                      unreachable=m.unreachable, events=[text for _, text in m.events[-5:]])
        self.state_pub.publish(String(data=json.dumps(status)))

    def publish_markers(self):
        """Puntos de observación en azul; conos candidatos en rojo, más opacos cuanto más probables."""
        stamp = self.get_clock().now().to_msg()
        markers = [Marker(action=Marker.DELETEALL)]
        for index, (x, y) in enumerate(self.viewpoints):
            markers.append(self.marker('viewpoints', index, Marker.SPHERE, x, y, 0.15, (0.1, 0.4, 1.0, 0.8), stamp))
        for index, track in enumerate(self.cones.tracks):
            confirmed = track.log_odds >= self.cones.confirm
            color = (0.1, 0.9, 0.1, 1.0) if confirmed else (1.0, 0.1, 0.1, max(0.2, track.probability))
            markers.append(self.marker('cones', index, Marker.CYLINDER, track.x, track.y, 0.2, color, stamp))
        self.markers_pub.publish(MarkerArray(markers=markers))

    def marker(self, namespace, index, kind, x, y, size, rgba, stamp):
        marker = Marker(ns=namespace, id=index, type=kind, action=Marker.ADD)
        marker.header.frame_id, marker.header.stamp = self.options['map_frame'], stamp
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = float(x), float(y), size / 2
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = size
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (float(v) for v in rgba)
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
