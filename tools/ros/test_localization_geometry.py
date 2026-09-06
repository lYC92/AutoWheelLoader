#!/usr/bin/env python3
"""Analytical ground/obstacle and articulated body cases, independent of ROS."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from localization_geometry import ground_plane, outside_body, range_mask


class GeometryTests(unittest.TestCase):
    def test_ground_and_obstacles_at_multiple_mounts_and_slopes(self):
        rng = np.random.default_rng(41)
        for height, slope in [(3.15, 0), (3.75, .2), (2.6, -.25)]:
            xy = rng.uniform(-12, 12, (2000, 2))
            ground = np.column_stack((xy, -height + slope*xy[:, 0] + rng.normal(0, .015, len(xy))))
            obstacles = ground[:150].copy()
            obstacles[:, 2] += 1.
            normal, offset = ground_plane(np.vstack((ground, obstacles)))
            self.assertLess(np.mean(abs(ground @ normal + offset)), .025)
            self.assertTrue(np.all(obstacles @ normal + offset > .18))

    def test_no_ground_does_not_misclassify_wall_or_line(self):
        rng = np.random.default_rng(3)
        wall = np.column_stack((np.full(1000, 4.), rng.uniform(-10, 10, 1000), rng.uniform(-4, 3, 1000)))
        self.assertIsNone(ground_plane(wall))
        line = np.column_stack((np.arange(100), np.zeros(100), np.full(100, -3.)))
        self.assertIsNone(ground_plane(line))
        self.assertIsNone(ground_plane(np.full((100, 3), np.nan)))
        self.assertIsNone(ground_plane(np.zeros((0, 3))))

    def test_bucket_follows_joint_pose_and_keeps_nearby_obstacle(self):
        box = [('bucket', np.eye(4), np.array([-.5, -1., -.3]), np.array([.5, 1., .3]))]
        for angle in [0, .7, -1.2]:
            pose = np.eye(4)
            pose[:3, :3] = Rotation.from_euler('y', angle).as_matrix()
            pose[:3, 3] = [4., 0, -1.]
            local = np.array([[0, 0, 0], [.5, 1, .3], [0, 1.3, 0], [2, 0, 0]])
            xyz = local @ pose[:3, :3].T + pose[:3, 3]
            np.testing.assert_array_equal(outside_body(xyz, box, {'bucket': pose}), [False, False, True, True])

    def test_invalid_and_range_returns(self):
        xyz = np.array([[4, 0, 0], [1, 0, 0], [60, 0, 0], [np.nan, 0, 0], [np.inf, 0, 0]])
        np.testing.assert_array_equal(range_mask(xyz, 3, 50), [True, False, False, False, False])


if __name__ == '__main__':
    unittest.main()
