"""Analytical geometry and conservation regressions; no ROS required."""
import math
import unittest
from heightfield_model import Material, SoilHeightfield3D, all_finite


def flat(capacity=100):
    soil = SoilHeightfield3D(domain_min_x_m=-2, domain_max_x_m=2,
        domain_min_y_m=-2, domain_max_y_m=2, cell_size_m=0.2,
        subgrid_step_m=0.01, bucket_width_m=1, bucket_capacity_m3=capacity,
        angle_of_repose_rad=math.radians(34),
        material=Material(1600, math.radians(34), 0, 0.3, 9.81, 0.15, 1, 0.6))
    soil.heights_m = [1.0]*len(soil.heights_m)
    soil.initial_volume_m3 = soil.terrain_volume_m3
    return soil


class SweepTests(unittest.TestCase):
    def test_rotated_translation_and_retrace(self):
        for angle in (0, math.pi/2, math.pi/4, -0.73):
            with self.subTest(angle=angle):
                soil = flat()
                start = (-0.5*math.cos(angle), -0.5*math.sin(angle), 0.5)
                end = (0.5*math.cos(angle), 0.5*math.sin(angle), 0.5)
                trace = soil.excavate_segment(start_xyz_m=start, end_xyz_m=end,
                    duration_s=1, start_yaw_rad=angle)
                self.assertTrue(all_finite(trace))
                self.assertAlmostEqual(soil.payload_volume_m3, 0.5, places=9)
                soil.begin_cutting_pass()
                soil.excavate_segment(start_xyz_m=end, end_xyz_m=start,
                    duration_s=1, start_yaw_rad=angle)
                self.assertAlmostEqual(soil.payload_volume_m3, 0.5, places=9)
                self.assertAlmostEqual(soil.volume_balance_error_m3, 0, places=10)

    def test_crossing_and_deeper_second_pass(self):
        soil = flat()
        soil.excavate_segment(start_xyz_m=(-0.75,0,0.5), end_xyz_m=(0.75,0,0.5), duration_s=1)
        soil.excavate_segment(start_xyz_m=(0,-0.75,0.5), end_xyz_m=(0,0.75,0.5),
            duration_s=1, start_yaw_rad=math.pi/2)
        # Two 1.5 x 1 strips intersect in a 1 x 1 square.
        self.assertAlmostEqual(soil.payload_volume_m3, 1.0, places=9)
        soil.excavate_segment(start_xyz_m=(0.75,0,0.25), end_xyz_m=(-0.75,0,0.25), duration_s=1)
        self.assertAlmostEqual(soil.payload_volume_m3, 1.375, places=9)
        self.assertAlmostEqual(soil.volume_balance_error_m3, 0, places=10)

    def test_rotation_sector(self):
        soil = flat()
        trace = soil.excavate_segment(start_xyz_m=(0,0,0.5), end_xyz_m=(0,0,0.5),
            duration_s=1, start_yaw_rad=0, end_yaw_rad=math.pi/2)
        self.assertLess(trace[0].bucket_torque_z_nm, 0)
        self.assertEqual(trace[0].bucket_torque_z_nm + trace[0].soil_reaction_torque_z_nm, 0)
        # Two radius-0.5 quarter sectors, excavated to depth 0.5.
        self.assertAlmostEqual(soil.payload_volume_m3, math.pi/16, delta=2e-5)
        self.assertLessEqual(soil.maximum_substep_m, 0.01+1e-12)
        self.assertAlmostEqual(soil.volume_balance_error_m3, 0, places=10)

    def test_capacity_deposit_and_stationary(self):
        soil = flat(0.13)
        soil.excavate_segment(start_xyz_m=(-0.5,0,0.5), end_xyz_m=(0.5,0,0.5), duration_s=1)
        self.assertAlmostEqual(soil.payload_volume_m3, 0.13, places=10)
        self.assertAlmostEqual(soil.unload_all(center_x_m=1, center_y_m=1), 0.13)
        self.assertAlmostEqual(soil.volume_balance_error_m3, 0, places=10)
        trace = soil.excavate_segment(start_xyz_m=(0,0,0.5), end_xyz_m=(0,0,0.5), duration_s=1)
        self.assertEqual(sum(t.swept_volume_m3 for t in trace), 0)
        with self.assertRaises(ValueError):
            soil.unload_all(center_x_m=5, center_y_m=0)

    def test_domain_clipping_and_invalid_input(self):
        soil = flat()
        soil.excavate_segment(start_xyz_m=(1.5,0,0.5), end_xyz_m=(2.5,0,0.5), duration_s=1)
        self.assertAlmostEqual(soil.payload_volume_m3, 0.25, places=10)
        for duration in (0, -1, math.nan):
            with self.assertRaises(ValueError):
                soil.excavate_segment(start_xyz_m=(0,0,0), end_xyz_m=(1,0,0), duration_s=duration)


if __name__ == "__main__":
    unittest.main()
