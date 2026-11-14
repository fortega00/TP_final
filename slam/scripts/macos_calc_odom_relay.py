"""Republica /calc_odom como /slam_odom con child_frame_id base_footprint (macOS, Gazebo Classic)."""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
rclpy.init()
node = Node('calc_relay', parameter_overrides=[Parameter('use_sim_time', value=True)])
pub = node.create_publisher(Odometry, '/slam_odom', 50)
def relay(m):
    m.child_frame_id = 'base_footprint'
    pub.publish(m)
node.create_subscription(Odometry, '/calc_odom', relay, 50)
rclpy.spin(node)
