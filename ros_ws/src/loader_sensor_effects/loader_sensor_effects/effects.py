"""Pure functions implementing nominal lidar scan effects.

The functions operate on plain numpy arrays so they can be unit-tested without
ROS.  All rotations use exact Rodrigues formulas; scan timing assumes a
constantly rotating multi-beam lidar whose columns are captured left to right
over one scan period.
"""

from __future__ import annotations

import numpy as np


def dropout_keep_mask(count: int, probability: float, seed: int) -> np.ndarray:
    """Return a boolean keep mask; False entries are dropped points."""
    if probability <= 0.0:
        return np.ones(count, dtype=bool)
    rng = np.random.default_rng(seed)
    return rng.random(count) >= probability


def rodrigues_rotate(points: np.ndarray, angle_vectors: np.ndarray) -> np.ndarray:
    """Rotate each point by its own axis-angle vector (exact Rodrigues)."""
    points = np.asarray(points, dtype=np.float64)
    angle_vectors = np.asarray(angle_vectors, dtype=np.float64)
    theta = np.linalg.norm(angle_vectors, axis=1)
    out = points.copy()
    moving = theta > 1e-12
    if not np.any(moving):
        return out
    axis = angle_vectors[moving] / theta[moving, None]
    angle = theta[moving, None]
    p = points[moving]
    cos = np.cos(angle)
    sin = np.sin(angle)
    out[moving] = (
        p * cos
        + np.cross(axis, p) * sin
        + axis * np.sum(axis * p, axis=1, keepdims=True) * (1.0 - cos)
    )
    return out


def scan_column_times(width: int, count: int, scan_period_s: float) -> np.ndarray:
    """Capture time of each point in a row-major organized cloud.

    Column 0 is captured at t=0 (the reference time), the last column at
    scan_period_s * (width - 1) / width.
    """
    columns = np.arange(count) % width
    return columns * (scan_period_s / width)


def apply_rotation_distortion(
    xyz: np.ndarray,
    width: int,
    scan_period_s: float,
    angular_velocity_rps: np.ndarray,
) -> np.ndarray:
    """Express points captured during a rotating scan in the start-of-scan frame.

    The sensor frame rotates with the vehicle at angular_velocity_rps while the
    beam sweeps.  A point captured at time t_i must be rotated by
    -omega * t_i to express it in the frame at t=0.
    """
    times = scan_column_times(width, xyz.shape[0], scan_period_s)
    angle_vectors = -np.asarray(angular_velocity_rps, dtype=np.float64)[None, :] * times[:, None]
    return rodrigues_rotate(xyz, angle_vectors)
