"""Keep recorded sensor/odometry TF, discard previous map localization edges."""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from tf2_msgs.msg import TFMessage


class BagTfFilter(Node):
    def __init__(self):
        super().__init__('bag_tf_filter')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        static = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.dynamic_pub = self.create_publisher(TFMessage, '/tf', 100)
        self.static_pub = self.create_publisher(TFMessage, '/tf_static', static)
        self.static_edges = {}
        self.create_subscription(TFMessage, '/slam/bag_tf', self.on_dynamic,
                                 qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/slam/bag_tf_static', self.on_static, static)

    def allowed(self, transform):
        return (transform.header.frame_id.lstrip('/') != self.map_frame
                and transform.child_frame_id.lstrip('/') not in (self.map_frame, self.odom_frame))

    def on_dynamic(self, message):
        transforms = [transform for transform in message.transforms if self.allowed(transform)]
        if transforms:
            self.dynamic_pub.publish(TFMessage(transforms=transforms))

    def on_static(self, message):
        for transform in message.transforms:
            if self.allowed(transform):
                self.static_edges[transform.child_frame_id] = transform
        if self.static_edges:
            self.static_pub.publish(TFMessage(transforms=list(self.static_edges.values())))


def main(args=None):
    rclpy.init(args=args)
    node = BagTfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
