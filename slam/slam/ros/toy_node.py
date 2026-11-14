"""Publish the independent toy simulator through real ROS 2 interfaces."""
import math

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock as ClockMsg
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from ..core.config import SlamConfig
from .ros_node import pose_message
from ..toy.world import ToyConfig, generate_samples


class ToyNode(Node):
    def __init__(self):
        super().__init__('toy_simulator')
        for name, default in {
            'config_file': '', 'rate': 10.0, 'scan_topic': '/scan', 'odom_topic': '/odom',
            'odom_frame': 'odom', 'base_frame': 'base_footprint', 'laser_frame': 'laser_frame',
        }.items():
            self.declare_parameter(name, default)
        get = lambda key: self.get_parameter(key).value
        config = SlamConfig.load_yaml(get('config_file')) if get('config_file') else SlamConfig()
        if get('rate') <= 0:
            raise ValueError('Toy rate must be positive')
        if (config.filter.initial_x, config.filter.initial_y, config.filter.initial_yaw) != (0, 0, 0):
            raise ValueError('Built-in toy starts at (0,0,0); use this initial pose in config')
        self.samples = iter(generate_samples(ToyConfig(range_max=config.sensor.max_range),
                                             seed=config.filter.seed))
        self.scan_pub = self.create_publisher(LaserScan, get('scan_topic'), qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, get('odom_topic'), qos_profile_sensor_data)
        self.truth_pub = self.create_publisher(PoseStamped, '/toy/ground_truth', 10)
        self.clock_pub = self.create_publisher(ClockMsg, '/clock', 10)
        persistent = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.done_pub = self.create_publisher(String, '/toy/done', persistent)
        self.dynamic = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = get('base_frame'), get('laser_frame')
        transform.transform.rotation.w = 1.0
        self.static.sendTransform(transform)
        self.odom_frame, self.base_frame, self.laser_frame = get('odom_frame'), get('base_frame'), get('laser_frame')
        self.last_clock = None
        self.done = False
        self.ticks = 0
        self.timer = self.create_timer(1.0 / get('rate'), self.tick,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
        # One wall second gives DDS discovery time; this node supplies /clock.
        self.startup_ticks = int(math.ceil(get('rate')))

    def tick(self):
        self.ticks += 1
        if self.ticks <= self.startup_ticks:
            return
        if self.done:
            self.clock_pub.publish(self.last_clock)
            return
        try:
            sample = next(self.samples)
        except StopIteration:
            self.done = True
            self.done_pub.publish(String(data='complete'))
            self.get_logger().info('Toy complete. Save with /slam/save_map or Ctrl+C.')
            return
        clock = ClockMsg()
        nanoseconds = int(round(sample.scan.stamp * 1e9))
        clock.clock.sec, clock.clock.nanosec = divmod(nanoseconds, 1_000_000_000)
        stamp = clock.clock
        self.last_clock = clock
        self.clock_pub.publish(clock)
        odom = Odometry()
        odom.header.frame_id, odom.header.stamp = self.odom_frame, stamp
        odom.child_frame_id = self.base_frame
        odom.pose.pose = pose_message(sample.odometry)
        self.odom_pub.publish(odom)
        transform = TransformStamped()
        transform.header, transform.child_frame_id = odom.header, self.base_frame
        transform.transform.translation.x = sample.odometry.x
        transform.transform.translation.y = sample.odometry.y
        transform.transform.rotation = odom.pose.pose.orientation
        self.dynamic.sendTransform(transform)
        scan = LaserScan()
        scan.header.frame_id, scan.header.stamp = self.laser_frame, stamp
        scan.angle_min, scan.angle_increment = sample.scan.angle_min, sample.scan.angle_increment
        scan.angle_max = scan.angle_min + (len(sample.scan.ranges) - 1) * scan.angle_increment
        scan.range_min, scan.range_max = sample.scan.range_min, sample.scan.range_max
        # Synthetic observations are instantaneous: no unmodelled rolling scan.
        scan.scan_time, scan.time_increment = 0.25, 0.0
        scan.ranges = sample.scan.ranges.astype('float32').tolist()
        self.scan_pub.publish(scan)
        truth = PoseStamped()
        truth.header.frame_id, truth.header.stamp = 'map', stamp
        truth.pose = pose_message(sample.truth)
        self.truth_pub.publish(truth)


def main(args=None):
    rclpy.init(args=args)
    node = ToyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
