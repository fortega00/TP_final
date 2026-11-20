"""Callbacks del navegador con reloj y publicadores controlados, sin nodos vivos."""

import json
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import Mock

from slam.core.geometry import Pose2D

from nav.core.navigator import Navigator, State
from nav.core.scan_watchdog import ScanWatchdog
from test_navigation import Base

try:
    from nav.ros.navigator_node import NavigatorNode, TransformException
except ImportError:
    NavigatorNode = None


@unittest.skipIf(NavigatorNode is None, "ROS 2 Python dependencies unavailable")
class NavigationTransportTests(Base):
    def adapter(self):
        core = Navigator(self.costmap, self.config)
        orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
        translation = SimpleNamespace(x=0.0, y=0.0)
        tf = SimpleNamespace(transform=SimpleNamespace(translation=translation, rotation=orientation))
        node = SimpleNamespace(core=core, config=self.config, ros_time=0.0, receipt=0.0,
                               options={'map_frame': 'map', 'base_frame': 'base_footprint',
                                        'scan_topic': '/scan'},
                               scan_watchdog=ScanWatchdog(0.50, 0.12, False),
                               use_sim_time=False,
                               awaiting_recovery_goal=False,
                               pose=Pose2D(-2.0, -1.0, 0.0), pose_stamp=0.0,
                               localization='LOCALIZED', blocked=False, was_active=False,
                               shown_events=0, shown_path=None,
                               velocity_pub=Mock(), state_pub=Mock(), publish_plan=Mock())
        node.get_logger = Mock(return_value=Mock())
        node.tf_buffer = SimpleNamespace(lookup_transform=Mock(return_value=tf))
        node.now = lambda: node.ros_time
        node.scan_time = lambda _: node.receipt
        for name in ('on_goal', 'on_scan', 'step', 'check_scan', 'fresh_pose', 'publish_navigation'):
            setattr(node, name, MethodType(getattr(NavigatorNode, name), node))
        goal = SimpleNamespace(header=SimpleNamespace(frame_id='map'),
                               pose=SimpleNamespace(position=SimpleNamespace(x=-1.0, y=-1.0),
                                                    orientation=orientation))
        return node, goal

    @staticmethod
    def scan(stamp):
        # El stamp vive en otro reloj, que va adelantado respecto al navegador.
        return SimpleNamespace(header=SimpleNamespace(frame_id='laser',
                                                      stamp=SimpleNamespace(sec=1700000000, nanosec=stamp)),
                               ranges=[5.0], angle_min=0.0, angle_increment=0.02,
                               range_min=0.05, range_max=12.0)

    def test_late_recovery_cancels_even_before_the_next_control_tick(self):
        node, goal = self.adapter()
        node.on_goal(goal)
        self.assertIsNone(node.core.goal)
        self.assertEqual(node.core.state, State.WAIT_SCAN)

        node.on_scan(self.scan(0))
        node.on_goal(goal)
        for t in (0.0, 0.1, 0.2, 0.3):
            node.ros_time = node.receipt = node.pose_stamp = t
            node.step()
        self.assertGreater(node.velocity_pub.publish.call_args.args[0].linear.x, 0.0)

        node.ros_time = node.receipt = node.pose_stamp = 0.4
        node.tf_buffer.lookup_transform.side_effect = TransformException('Missing laser mount')
        node.on_scan(self.scan(400000000))
        node.tf_buffer.lookup_transform.side_effect = None
        self.assertEqual(node.scan_watchdog.last_received, 0.0)

        node.ros_time = node.receipt = node.pose_stamp = 0.51
        node.on_scan(self.scan(510000000))
        zero = node.velocity_pub.publish.call_args.args[0]
        self.assertEqual((zero.linear.x, zero.angular.z), (0.0, 0.0))
        self.assertIsNone(node.core.goal)
        self.assertEqual(node.core.path, [])
        self.assertEqual(json.loads(node.state_pub.publish.call_args.args[0].data)['state'], 'WAIT_SCAN')

        node.step()
        self.assertEqual(node.core.state, State.IDLE)
        self.assertIsNone(node.core.goal)
        warnings = [call.args[0] for call in node.get_logger().warning.call_args_list]
        self.assertEqual(sum('Lidar unavailable' in warning for warning in warnings), 1)
        node.pose_stamp = None
        node.on_goal(goal)
        self.assertIsNone(node.core.goal)
        node.pose_stamp = node.now()
        node.on_goal(goal)
        self.assertIsNotNone(node.core.goal)
