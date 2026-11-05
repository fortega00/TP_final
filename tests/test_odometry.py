import math
import unittest

from tp_slam.geometry import Pose2D
from tp_slam.odometry import OdometryBuffer


class OdometryTests(unittest.TestCase):
    def test_shortest_angle_and_no_extrapolation(self):
        buffer = OdometryBuffer()
        buffer.append(1.0, Pose2D(0, 2, math.radians(179)))
        buffer.append(3.0, Pose2D(2, 4, math.radians(-179)))
        result = buffer.at(2.0)
        self.assertAlmostEqual(result.x, 1.0)
        self.assertAlmostEqual(result.y, 3.0)
        self.assertAlmostEqual(abs(result.yaw), math.pi)
        self.assertIsNone(buffer.at(0.5))
        self.assertIsNone(buffer.at(3.1))

    def test_old_duplicate_and_invalid_poses_do_not_modify_history(self):
        buffer = OdometryBuffer(1.0)
        self.assertTrue(buffer.append(2, Pose2D(0, 0, 0)))
        self.assertFalse(buffer.append(1, Pose2D(5, 0, 0)))
        self.assertFalse(buffer.append(2, Pose2D(5, 0, 0)))
        self.assertFalse(buffer.append(float('nan'), Pose2D(0, 0, 0)))
        self.assertEqual(len(buffer.samples), 1)
        buffer.append(4, Pose2D(1, 0, 0))
        buffer.append(5, Pose2D(2, 0, 0))
        self.assertIsNotNone(buffer.at(4.5))
