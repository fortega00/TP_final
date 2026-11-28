"""Conversiones de mensajes de ROS a lo que usa el núcleo. Sin ROS."""

import math
import unittest

import numpy as np

from mission.ros.conversions import depth_in_meters, intrinsics_from_k, match_size, transform_matrix
from test_cone_locator import camera_pose


def yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class TransformTests(unittest.TestCase):
    def test_identity(self):
        np.testing.assert_allclose(transform_matrix((0, 0, 0), (0, 0, 0, 1)), np.eye(4))

    def test_rotation_about_z_and_translation(self):
        matrix = transform_matrix((1.0, 2.0, 0.5), yaw_quaternion(math.pi / 2))
        np.testing.assert_allclose(matrix @ [1, 0, 0, 1], [1.0, 3.0, 0.5, 1.0], atol=1e-12)

    def test_unnormalized_quaternion(self):
        matrix = transform_matrix((0, 0, 0), tuple(2 * v for v in yaw_quaternion(0.3)))
        np.testing.assert_allclose(matrix[:3, :3] @ matrix[:3, :3].T, np.eye(3), atol=1e-12)

    def test_matches_an_optical_camera_frame(self):
        # El frame óptico de los tests del localizador, expresado como cuaternión.
        expected = camera_pose(1.0, 2.0, 0.4, height=0.25)
        rotation = expected[:3, :3]
        w = math.sqrt(1 + np.trace(rotation)) / 2
        q = ((rotation[2, 1] - rotation[1, 2]) / (4 * w), (rotation[0, 2] - rotation[2, 0]) / (4 * w),
             (rotation[1, 0] - rotation[0, 1]) / (4 * w), w)
        np.testing.assert_allclose(transform_matrix((1.0, 2.0, 0.25), q), expected, atol=1e-12)


class DepthTests(unittest.TestCase):
    def test_millimeters(self):
        depth = np.array([[0, 1500], [2000, 65535]], dtype=np.uint16)
        np.testing.assert_allclose(depth_in_meters(depth, "16UC1"), [[0.0, 1.5], [2.0, 65.535]])

    def test_meters_pass_through(self):
        depth = np.array([[1.25, np.nan]], dtype=np.float32)
        result = depth_in_meters(depth, "32FC1")
        self.assertEqual(result[0, 0], np.float32(1.25))
        self.assertTrue(np.isnan(result[0, 1]))

    def test_unknown_encoding(self):
        with self.assertRaises(ValueError):
            depth_in_meters(np.zeros((2, 2)), "rgb8")

    def test_match_size(self):
        depth = np.arange(4, dtype=np.float32).reshape(2, 2)
        bigger = match_size(depth, (4, 4, 3))
        self.assertEqual(bigger.shape, (4, 4))
        self.assertEqual(bigger[3, 3], 3.0)
        self.assertIs(match_size(depth, (2, 2)), depth)


class IntrinsicsTests(unittest.TestCase):
    def test_from_camera_info_k(self):
        intrinsics = intrinsics_from_k([525.0, 0, 319.5, 0, 526.0, 239.5, 0, 0, 1], 640)
        self.assertEqual((intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy, intrinsics.width),
                         (525.0, 526.0, 319.5, 239.5, 640))


if __name__ == "__main__":
    unittest.main()
