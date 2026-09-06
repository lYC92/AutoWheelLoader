#!/usr/bin/env python3
"""Analytical regression cases for initial-pose alignment and time matching."""
import numpy as np
from scipy.spatial.transform import Rotation
from evaluate_localization import compare
from filter_localization_cloud import select_points

points = np.array([[4., 0., -3.15], [4., 0., -1.], [1., 0., 0.],
                   [60., 0., 0.], [np.nan, 0., 0.]])
assert np.array_equal(select_points(points, -2.7, 3., 50.), [[4., 0., -1.]])
print("PASS  localization crop excludes ground, self range, far and invalid returns")

truth = []
for t in np.arange(0., 2.01, .02):
    p = np.eye(4)
    p[:3, 3] = [t, .5*t, 0]
    p[:3, :3] = Rotation.from_euler("z", .1*t).as_matrix()
    truth.append((t, p))
offset = np.eye(4)
offset[:3, :3] = Rotation.from_euler("z", .7).as_matrix()
offset[:3, 3] = [4, -2, 1]
estimates = []
for t in np.arange(.05, 1.96, .1):
    p = np.eye(4)
    p[:3, 3] = [t, .5*t, 0]
    p[:3, :3] = Rotation.from_euler("z", .1*t).as_matrix()
    estimates.append((t, np.linalg.inv(offset) @ p, .01))
rows = compare(estimates, truth)
assert len(rows) == len(estimates)
assert np.max(rows[:, 7:9]) < 1e-10, "interpolation/initial alignment changed the trajectory"
drifting = [(t, p.copy(), lag) for t, p, lag in estimates]
drifting[-1][1][:3, 3] += [1, 0, 0]
rows = compare(drifting, truth)
assert abs(rows[-1, 7]-1) < 1e-10, "alignment concealed later drift"
assert len(compare(estimates + [(3., np.eye(4), 0)], truth)) == len(estimates), "out-of-range truth was extrapolated"
print("PASS  initial alignment, interpolated SE(3) truth, drift detection, and timestamp coverage")
