"""Regresiones de cola/sincronización; requieren los tipos de ROS, no nodos vivos."""
from collections import Counter, deque
from types import SimpleNamespace
import time
import unittest
import numpy as np

try:
    from slam.ros.ros_node import SlamNode
    from nav_msgs.msg import Path
except ImportError:
    SlamNode = None

from slam.core.geometry import Pose2D
from slam.core.odometry import OdometryBuffer


@unittest.skipIf(SlamNode is None, "ROS 2 Python dependencies unavailable")
class TransportTests(unittest.TestCase):
    def adapter(self, stamp, received=None):
        message = SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(sec=int(stamp), nanosec=int((stamp-int(stamp))*1e9)),
            frame_id='laser'), ranges=[1., 2.])
        fake = SimpleNamespace(adapter_error='', last_wait_reason='',
                               options={'odom_source': 'topic', 'base_frame': 'base',
                                        'tf_wait_timeout': 2., 'scan_queue_size': 2},
                               pending=deque([(message, time.monotonic() if received is None else received)]),
                               dropped_scans=0, drop_reasons=Counter(), odometry=OdometryBuffer(),
                               last_scan_received=None, received_scans=0, callback_seconds={})
        fake.lookup = lambda *args: Pose2D()
        fake.drop_scan = lambda reason: SlamNode.drop_scan(fake, reason)
        fake.get_logger = lambda: SimpleNamespace(warning=lambda *args, **kw: None)
        return fake

    def test_scan_before_earliest_odometry_does_not_block_valid_successors(self):
        fake = self.adapter(1.)
        fake.odometry.append(2., Pose2D())
        SlamNode.drain(fake)
        self.assertFalse(fake.pending)
        self.assertEqual(fake.dropped_scans, 1)
        self.assertEqual(fake.drop_reasons, {'odometry_before_history': 1})

    def test_scan_waits_for_a_future_sample_instead_of_extrapolating(self):
        fake = self.adapter(3.)
        fake.odometry.append(1., Pose2D())
        fake.odometry.append(2., Pose2D())
        SlamNode.drain(fake)
        self.assertEqual(len(fake.pending), 1)
        self.assertEqual(fake.dropped_scans, 0)

    def test_future_timeout_has_a_specific_counter(self):
        fake = self.adapter(3., time.monotonic()-3.)
        fake.odometry.append(2., Pose2D())
        SlamNode.drain(fake)
        self.assertFalse(fake.pending)
        self.assertEqual(fake.drop_reasons, {'odometry_future_timeout': 1})

    def test_overflow_and_invalid_messages_have_distinct_counters(self):
        fake = self.adapter(1.)
        fake.pending.append(fake.pending[0])
        message = fake.pending[0][0]
        SlamNode.on_scan(fake, message)
        self.assertEqual(fake.drop_reasons, {'queue_overflow': 1})
        self.assertEqual(len(fake.pending), 2)
        message.header.frame_id = ''
        SlamNode.on_scan(fake, message)
        self.assertEqual(fake.drop_reasons['invalid_message'], 1)

    def test_path_cache_preserves_content_when_the_selected_ancestry_changes(self):
        first, second = Pose2D(), Pose2D(.2, .1, .05)
        particle = SimpleNamespace(pose=second, trajectory=[(1., first), (2., second)])
        publisher = SimpleNamespace(publish=lambda value: None)
        core = SimpleNamespace(best_particle=particle, particles=[particle],
                               estimate=lambda: (particle.pose, np.zeros((3, 3))))
        fake = SimpleNamespace(core=core, options={'map_frame': 'map', 'odom_frame': 'odom', 'max_path_poses': 20,
                                                 'publish_period': 1e9},
                               last_odom_at_scan=Pose2D(), last_correction_stamp=-np.inf,
                               last_map_publish=0., path=Path(), _path_message_cache={},
                               broadcast=lambda *args: None, pose_pub=publisher,
                               particles_pub=publisher, path_pub=publisher)
        message = SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(sec=2, nanosec=0)))
        # Los mensajes ROS necesitan su tipo real para asignar header.stamp.
        from builtin_interfaces.msg import Time
        message.header.stamp = Time(sec=2)
        SlamNode.publish_update(fake, message)
        prefix_message, old_tail = fake.path.poses
        particle.pose = Pose2D(.3, -.1, .1)
        particle.trajectory = [(1., first), (2., particle.pose)]
        SlamNode.publish_update(fake, message)
        self.assertIs(fake.path.poses[0], prefix_message)
        self.assertIsNot(fake.path.poses[1], old_tail)
        self.assertAlmostEqual(fake.path.poses[1].pose.position.x, .3)
        self.assertEqual(fake.path.poses[1].header.stamp.sec, 2)
        self.assertEqual(len(fake._path_message_cache), 2)


if __name__ == '__main__':
    unittest.main()
