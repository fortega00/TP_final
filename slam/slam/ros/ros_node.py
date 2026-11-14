"""ROS 2 adapter: timestamp alignment, TF, services and transport only.

The probabilistic algorithm lives in filter.py; this module never substitutes a
latest TF for a transform at the sensor acquisition timestamp.
"""
from collections import deque
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from ..core.config import SlamConfig
from ..core.filter import FastSLAM
from ..core.geometry import Pose2D
from ..core.odometry import OdometryBuffer
from .reference_paths import ReferencePaths
from ..core.types import Scan


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_from_quaternion(q):
    values = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(values)
    if not np.isfinite(norm) or norm < 1e-8:
        raise ValueError("Invalid quaternion")
    x, y, z, w = values / norm
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


def pose_from_transform(transform):
    return Pose2D(transform.translation.x, transform.translation.y,
                  yaw_from_quaternion(transform.rotation))


def pose_message(pose):
    message = Pose()
    message.position.x, message.position.y = float(pose.x), float(pose.y)
    message.orientation.z = math.sin(pose.yaw / 2)
    message.orientation.w = math.cos(pose.yaw / 2)
    return message


class SlamNode(Node):
    def __init__(self):
        super().__init__('slam')
        defaults = {
            'config_file': '', 'scan_topic': '/scan', 'odom_topic': '/odom',
            'odom_source': 'topic', 'map_frame': 'map', 'odom_frame': 'odom',
            'base_frame': 'base_footprint', 'output_path': './maps/slam',
            'publish_odom_tf': False, 'publish_period': 1.0,
            'tf_wait_timeout': 2.0, 'odom_history_seconds': 30.0,
            'scan_queue_size': 30, 'max_path_poses': 5000, 'mode': 'real',
            'ground_truth_topic': '',
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.options = {name: self.get_parameter(name).value for name in defaults}
        for name in ('map_frame', 'odom_frame', 'base_frame'):
            value = self.options[name]
            if not value or value.startswith('/'):
                raise ValueError(f'{name} must be a nonempty TF frame without leading slash')
        if len({self.options[k] for k in ('map_frame', 'odom_frame', 'base_frame')}) != 3:
            raise ValueError('Map, odometry and base frames must be distinct')
        if self.options['odom_source'] not in ('topic', 'tf'):
            raise ValueError('odom_source must be topic or tf')
        for name in ('publish_period', 'tf_wait_timeout', 'odom_history_seconds',
                     'scan_queue_size', 'max_path_poses'):
            if self.options[name] <= 0:
                raise ValueError(f'{name} must be positive')
        filename = self.options['config_file']
        self.config = SlamConfig.load_yaml(filename) if filename else SlamConfig()
        self.core = FastSLAM(self.config)
        self.odometry = OdometryBuffer(self.options['odom_history_seconds'])
        self.tf_buffer = Buffer(cache_time=Duration(seconds=self.options['odom_history_seconds']))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        persistent = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', persistent)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/slam_pose', 10)
        self.particles_pub = self.create_publisher(PoseArray, '/particles', 10)
        self.path_pub = self.create_publisher(PathMsg, '/path', persistent)
        self.status_pub = self.create_publisher(String, '/slam/status', persistent)
        self.path = PathMsg()
        self.path.header.frame_id = self.options['map_frame']
        self.references = ReferencePaths(self, persistent, pose_message, seconds,
                                         yaw_from_quaternion)
        self.pending = deque()
        self.last_scan_received = None
        self.last_integrated_message = None
        self.last_odom_at_scan = None
        self.map_to_odom = None
        self.last_correction_stamp = -math.inf
        self.last_map_publish = -math.inf
        self.dropped_scans = 0
        self.adapter_error = ''
        self.last_wait_reason = ''
        # The DDS queue must survive one expensive map update as well as the
        # explicit waiting queue; the default sensor profile holds just 5 scans.
        scan_qos = QoSProfile(depth=self.options['scan_queue_size'],
                              reliability=ReliabilityPolicy.BEST_EFFORT)
        odom_qos = QoSProfile(depth=max(100, self.options['scan_queue_size']),
                              reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, self.options['scan_topic'], self.on_scan, scan_qos)
        if self.options['odom_source'] == 'topic':
            self.create_subscription(Odometry, self.options['odom_topic'], self.on_odom,
                                     odom_qos)
        self.create_service(Trigger, '/slam/save_map', self.on_save)
        self.create_service(Trigger, '/slam/pause', self.on_pause)
        self.create_service(Trigger, '/slam/resume', self.on_resume)
        self.create_service(Trigger, '/slam/reset', self.on_reset)
        # Wall timer allows services/diagnostics and TF retries while /clock pauses.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.025, self.drain, clock=steady)
        self.create_timer(1.0, self.publish_status, clock=steady)
        self.get_logger().info(
            f'FastSLAM {self.config.filter.particles} particles; '
            f'{self.options["odom_source"]} odometry; scan={self.options["scan_topic"]}')

    def lookup(self, target, source, stamp):
        if target == source:
            return Pose2D()
        transform = self.tf_buffer.lookup_transform(target, source, Time.from_msg(stamp))
        return pose_from_transform(transform.transform)

    def on_odom(self, message):
        if self.adapter_error:
            return
        try:
            if message.header.frame_id != self.options['odom_frame']:
                raise ValueError('Odometry header.frame_id differs from odom_frame')
            if not message.child_frame_id:
                raise ValueError('Odometry child_frame_id is empty')
            body = message.pose.pose
            pose = Pose2D(body.position.x, body.position.y, yaw_from_quaternion(body.orientation))
            pose = pose.compose(self.lookup(message.child_frame_id, self.options['base_frame'],
                                            message.header.stamp))
            stamp = seconds(message.header.stamp)
            if self.odometry.samples and stamp < self.odometry.samples[-1][0] - 0.5:
                self.adapter_error = 'Odometry time moved backwards; call /slam/reset before a new run'
                self.publish_status()
                return
            if not self.odometry.append(stamp, pose):
                return
            if self.options['publish_odom_tf']:
                self.broadcast(self.options['odom_frame'], self.options['base_frame'],
                               pose, message.header.stamp)
            self.publish_correction(message.header.stamp)
        except (ValueError, TransformException) as exc:
            self.get_logger().warning(f'Odometry unavailable: {exc}', throttle_duration_sec=5.0)

    def on_scan(self, message):
        stamp = seconds(message.header.stamp)
        if not message.header.frame_id or not message.ranges:
            self.dropped_scans += 1
            return
        if self.last_scan_received is not None and stamp <= self.last_scan_received:
            if stamp < self.last_scan_received - 0.5:
                self.adapter_error = 'Scan time moved backwards; call /slam/reset before a new run'
            self.dropped_scans += 1
            return
        self.last_scan_received = stamp
        if len(self.pending) >= self.options['scan_queue_size']:
            self.pending.popleft()
            self.dropped_scans += 1
        self.pending.append((message, time.monotonic()))

    def drain(self):
        if self.adapter_error:
            return
        if self.options['odom_source'] == 'tf' and self.map_to_odom is not None:
            try:
                latest = self.tf_buffer.lookup_transform(self.options['odom_frame'],
                                                         self.options['base_frame'], Time())
                self.publish_correction(latest.header.stamp)
            except TransformException:
                pass
        # One scan per tick keeps the executor responsive to TF and services.
        if not self.pending:
            return
        message, received = self.pending[0]
        try:
            sensor = self.lookup(self.options['base_frame'], message.header.frame_id,
                                 message.header.stamp)
            if self.options['odom_source'] == 'tf':
                odom = self.lookup(self.options['odom_frame'], self.options['base_frame'],
                                   message.header.stamp)
            else:
                odom = self.odometry.at(seconds(message.header.stamp))
                if odom is None:
                    raise ValueError('No odometry samples bracketing scan timestamp')
        except (TransformException, ValueError) as exc:
            self.last_wait_reason = str(exc)
            if time.monotonic() - received > self.options['tf_wait_timeout']:
                self.pending.popleft()
                self.dropped_scans += 1
                self.get_logger().warning(f'Discarding scan: {exc}', throttle_duration_sec=5.0)
            return
        self.pending.popleft()
        self.last_wait_reason = ''
        try:
            scan = Scan(np.asarray(message.ranges), message.angle_min, message.angle_increment,
                        message.range_min, message.range_max, seconds(message.header.stamp), sensor)
            if not self.core.process(scan, odom):
                return
            self.last_integrated_message = message
            self.last_odom_at_scan = odom
            self.references.observe(message.header.stamp, odom)
            self.publish_update(message)
        except ValueError as exc:
            self.dropped_scans += 1
            if self.core.state.name == 'ERROR':
                self.adapter_error = str(exc) + '; call /slam/reset after correcting configuration'
            self.get_logger().warning(f'Invalid scan: {exc}', throttle_duration_sec=5.0)
        except Exception as exc:
            self.adapter_error = f'{type(exc).__name__}: {exc}; inspect inputs and call /slam/reset'
            self.get_logger().error(self.adapter_error)
            self.publish_status()

    def broadcast(self, parent, child, pose, stamp):
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = parent, child
        transform.header.stamp = stamp
        transform.transform.translation.x = float(pose.x)
        transform.transform.translation.y = float(pose.y)
        transform.transform.rotation.z = math.sin(pose.yaw / 2)
        transform.transform.rotation.w = math.cos(pose.yaw / 2)
        self.tf_broadcaster.sendTransform(transform)

    def publish_update(self, scan_message):
        stamp = scan_message.header.stamp
        best = self.core.best_particle
        self.map_to_odom = best.pose.compose(self.last_odom_at_scan.inverse())
        self.broadcast(self.options['map_frame'], self.options['odom_frame'], self.map_to_odom, stamp)
        self.last_correction_stamp = max(self.last_correction_stamp, seconds(stamp))
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id, pose.header.stamp = self.options['map_frame'], stamp
        pose.pose.pose = pose_message(best.pose)
        mean, covariance = self.core.estimate()
        difference = best.pose.between(mean)
        # Publish the second moment about the selected hypothesis, in map axes.
        residual = np.array([mean.x - best.pose.x, mean.y - best.pose.y, difference.yaw])
        covariance += np.outer(residual, residual)
        for a, row in enumerate((0, 1, 5)):
            for b, col in enumerate((0, 1, 5)):
                pose.pose.covariance[row * 6 + col] = float(covariance[a, b])
        self.pose_pub.publish(pose)
        cloud = PoseArray()
        cloud.header = pose.header
        cloud.poses = [pose_message(particle.pose) for particle in self.core.particles]
        self.particles_pub.publish(cloud)
        self.path.header.stamp = stamp
        self.path.poses = []
        for timestamp, historical_pose in best.trajectory[-self.options['max_path_poses']:]:
            point = PoseStamped()
            point.header.frame_id = self.options['map_frame']
            nanos = int(round(timestamp * 1e9))
            point.header.stamp.sec, point.header.stamp.nanosec = divmod(nanos, 1_000_000_000)
            point.pose = pose_message(historical_pose)
            self.path.poses.append(point)
        self.path_pub.publish(self.path)
        now = seconds(stamp)
        if now - self.last_map_publish >= self.options['publish_period']:
            self.publish_map(stamp)
            self.last_map_publish = now

    def publish_correction(self, stamp):
        # Keep the latest SLAM correction valid at actual odometry timestamps.
        # Only the TF transport uses this heartbeat; scoring still uses scan-time poses.
        timestamp = seconds(stamp)
        if self.map_to_odom is not None and timestamp > self.last_correction_stamp:
            self.broadcast(self.options['map_frame'], self.options['odom_frame'],
                           self.map_to_odom, stamp)
            self.last_correction_stamp = timestamp

    def publish_map(self, stamp):
        grid = self.core.best_particle.grid
        message = OccupancyGrid()
        message.header.frame_id, message.header.stamp = self.options['map_frame'], stamp
        message.info.map_load_time = stamp
        message.info.resolution = self.config.map.resolution
        message.info.height, message.info.width = grid.log_odds.shape
        message.info.origin = pose_message(Pose2D(self.config.map.origin_x,
                                                 self.config.map.origin_y, 0))
        message.data = grid.occupancy().ravel().tolist()
        self.map_pub.publish(message)

    def publish_status(self):
        status = dict(self.core.diagnostics)
        status.update(state='ERROR' if self.adapter_error else self.core.state.name,
                      adapter_error=self.adapter_error, waiting_for=self.last_wait_reason,
                      dropped_scans=self.dropped_scans, queued_scans=len(self.pending))
        message = String()
        message.data = json.dumps(status, allow_nan=False)
        self.status_pub.publish(message)

    def save(self):
        if self.last_integrated_message is None:
            raise ValueError('No integrated scan; map has not been created')
        output = Path(self.options['output_path']).expanduser()
        paths = self.core.save_map(output)
        metadata = {'config': self.config.to_dict(), 'ros': self.options,
                    'diagnostics': self.core.diagnostics,
                    'scan_stamp': seconds(self.last_integrated_message.header.stamp),
                    'dropped_scans': self.dropped_scans}
        paths[1].with_suffix('.json').write_text(json.dumps(metadata, indent=2) + '\n')
        if rclpy.ok():
            self.publish_map(self.last_integrated_message.header.stamp)
        return ', '.join(str(path) for path in paths)

    def on_save(self, request, response):
        del request
        try:
            response.message, response.success = self.save(), True
        except (OSError, ValueError) as exc:
            response.message, response.success = str(exc), False
        return response

    def on_pause(self, request, response):
        del request
        try:
            self.core.pause()
            response.success, response.message = True, 'Mapping paused'
        except (RuntimeError, ValueError) as exc:
            response.success, response.message = False, str(exc)
        self.publish_status()
        return response

    def on_resume(self, request, response):
        del request
        if self.adapter_error:
            response.success, response.message = False, self.adapter_error
        else:
            try:
                self.core.resume()
                response.success, response.message = True, 'Mapping resumed'
            except (RuntimeError, ValueError) as exc:
                response.success, response.message = False, str(exc)
        self.publish_status()
        return response

    def on_reset(self, request, response):
        del request
        self.core.reset()
        self.odometry.clear()
        self.pending.clear()
        self.path.poses.clear()
        self.path_pub.publish(self.path)
        self.last_scan_received = self.last_integrated_message = self.last_odom_at_scan = None
        self.references.reset()
        self.map_to_odom = None
        self.last_correction_stamp = -math.inf
        self.last_map_publish = -math.inf
        self.dropped_scans = 0
        self.adapter_error = self.last_wait_reason = ''
        self.publish_map(self.get_clock().now().to_msg())
        self.publish_status()
        response.success, response.message = True, 'New mapping session; previous map discarded'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SlamNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if node.last_integrated_message is not None:
                try:
                    # Saving to disk remains valid after a ROS context shutdown.
                    node.save()
                except Exception as exc:
                    node.get_logger().error(f'Could not save map on shutdown: {exc}')
            node.core.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
