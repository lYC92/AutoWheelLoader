#!/usr/bin/env python3
"""Validate the nominal sensor effect channel and IMU noise model.

Live checks run against the sensors world:
  - the effect cloud mirrors the raw cloud at 10 Hz with the requested dropout
    ratio (dropped points appear as NaN, organized layout preserved);
  - stationary-vehicle rotation distortion stays within a millimetre band;
  - the IMU noise model produces the configured variance around gravity;
  - the URDF carries the requested sensor mount perturbations.

Unit checks cover dropout determinism and the Rodrigues rotation math.
"""

from __future__ import annotations

import math
import sys
import time
import xml.etree.ElementTree as ET
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2, PointField

from loader_sensor_effects.effects import dropout_keep_mask, rodrigues_rotate
from loader_sensor_effects.lidar_effects_node import cloud_dtype, LidarEffectsNode

DROPOUT_PROBABILITY = 0.10
RANDOM_SEED = 7
IMU_GYRO_STDDEV = 0.0017
IMU_ACCEL_STDDEV = 0.017


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def unit_checks() -> None:
    mask_a = dropout_keep_mask(1000, 0.3, seed=42)
    mask_b = dropout_keep_mask(1000, 0.3, seed=42)
    require(np.array_equal(mask_a, mask_b), "dropout mask is not deterministic under a fixed seed")
    require(
        abs(mask_a.mean() - 0.7) < 0.05,
        f"dropout ratio off target: keep={mask_a.mean():.3f}",
    )
    require(
        dropout_keep_mask(10, 0.0, seed=1).all(),
        "zero dropout probability must keep every point",
    )

    rotated = rodrigues_rotate(
        np.array([[1.0, 0.0, 0.0]]), np.array([[0.0, 0.0, math.pi / 2]])
    )
    require(
        np.allclose(rotated[0], [0.0, 1.0, 0.0], atol=1e-9),
        f"Rodrigues 90-degree z rotation wrong: {rotated[0]}",
    )
    identity = rodrigues_rotate(np.array([[1.0, 2.0, 3.0]]), np.zeros((1, 3)))
    require(np.allclose(identity[0], [1.0, 2.0, 3.0]), "zero rotation must be identity")
    print("PASS  effect unit checks: deterministic dropout, Rodrigues rotation")

    # A missing early return must not move later samples to an earlier column.
    # Two rows with padding also detect treating PointCloud2 as a flat buffer.
    cloud = PointCloud2(height=2, width=4, point_step=16, row_step=72, is_dense=False)
    cloud.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                    for i, name in enumerate(("x", "y", "z", "intensity"))]
    raw_data = bytearray([0xA5] * (cloud.row_step * cloud.height))
    points = np.ndarray((2, 4), dtype=cloud_dtype(cloud), buffer=raw_data, strides=(72, 16))
    points["x"], points["y"], points["z"], points["intensity"] = 1.0, 0.0, 0.0, 42.0
    points["x"][0, 1] = np.nan
    cloud.data = bytes(raw_data)
    output = []
    fake = SimpleNamespace(distortion_enabled=True, scan_period_s=0.1,
                           angular_velocity=np.array([0., 0., 1.]),
                           dropout_probability=0., random_seed=7, scan_sequence=0,
                           publisher=SimpleNamespace(publish=output.append))
    LidarEffectsNode.on_cloud(fake, cloud)
    out = output[0]
    actual = np.ndarray((2, 4), dtype=cloud_dtype(out), buffer=bytes(out.data), strides=(72, 16))
    require(np.allclose(actual["y"][1], -np.sin(np.arange(4)*0.025)), "second-row scan timing changed")
    require(abs(actual["y"][0, 2] + math.sin(0.05)) < 1e-7, "NaN shifted scan column timing")
    require(np.isnan(actual["x"][0, 1]), "invalid return was invented")
    require(np.all(actual["intensity"] == 42), "non-XYZ fields changed")
    require(bytes(out.data)[64:72] == bytes(raw_data)[64:72], "row padding changed")
    require(len(out.data) == out.row_step*out.height, "PointCloud2 data length is inconsistent")
    print("PASS  missing returns preserve scan timing; padded rows and intensity preserved")


def urdf_checks(urdf_path: str) -> None:
    root = ET.parse(urdf_path).getroot()
    origins = {}
    for joint in root.iter("joint"):
        origin = joint.find("origin")
        if origin is not None:
            origins[joint.get("name")] = (
                [float(v) for v in origin.get("xyz", "0 0 0").split()],
                [float(v) for v in origin.get("rpy", "0 0 0").split()],
            )
    lidar_xyz, _ = origins["lidar_mount_joint"]
    require(
        abs(lidar_xyz[0] - (-0.84)) < 1e-9 and abs(lidar_xyz[2] - 2.23) < 1e-9,
        f"lidar mount perturbation missing from URDF: {lidar_xyz}",
    )
    _, imu_rpy = origins["imu_mount_joint"]
    require(
        abs(imu_rpy[2] - 0.005) < 1e-9,
        f"IMU mount yaw perturbation missing from URDF: {imu_rpy}",
    )
    print("PASS  sensor mount perturbations baked into URDF origins")


class EffectHarness(Node):
    def __init__(self) -> None:
        super().__init__("sensor_effects_smoke_test")
        self.raw_clouds: list[PointCloud2] = []
        self.effect_clouds: list[PointCloud2] = []
        self.imu_messages: list[Imu] = []
        self.create_subscription(
            PointCloud2, "/loader/sensors/lidar/scan/points", self.raw_clouds.append, 10
        )
        self.create_subscription(
            PointCloud2,
            "/loader/sensors/lidar/scan/points_effect",
            self.effect_clouds.append,
            10,
        )
        self.create_subscription(Imu, "/loader/sensors/imu", self.imu_messages.append, 100)


def xyz_array(message: PointCloud2) -> np.ndarray:
    count = message.width * message.height
    points = np.frombuffer(bytes(message.data), dtype=cloud_dtype(message), count=count)
    return np.stack([points["x"], points["y"], points["z"]], axis=1).astype(np.float64)


def live_checks(node: EffectHarness) -> None:
    start = time.monotonic()
    deadline = start + 10.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if len(node.raw_clouds) >= 20 and len(node.effect_clouds) >= 20 and len(node.imu_messages) >= 400:
            break
    require(len(node.raw_clouds) >= 20, f"only {len(node.raw_clouds)} raw clouds")
    require(len(node.effect_clouds) >= 20, f"only {len(node.effect_clouds)} effect clouds")
    require(len(node.imu_messages) >= 400, f"only {len(node.imu_messages)} IMU messages")

    def stamp(message):
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
    raw_by_stamp = {stamp(m): m for m in node.raw_clouds}
    pairs = [(raw_by_stamp[stamp(m)], m) for m in node.effect_clouds if stamp(m) in raw_by_stamp]
    require(len(pairs) >= 10, "raw/effect clouds do not share source timestamps")
    raw, effect = pairs[-1]
    stamps = [stamp(m) for m in node.effect_clouds]
    require(all(b > a for a, b in zip(stamps, stamps[1:])), "effect timestamps are not increasing")
    frequency = (len(stamps)-1)*1e9/(stamps[-1]-stamps[0])
    raw_stamps = [stamp(m) for m in node.raw_clouds]
    raw_frequency = (len(raw_stamps)-1)*1e9/(raw_stamps[-1]-raw_stamps[0])
    print(f"INFO  source rate={raw_frequency:.2f} Hz, effect rate={frequency:.2f} Hz; "
          f"raw={len(raw_stamps)}, effect={len(stamps)}", flush=True)
    require(9.0 <= frequency <= 11.0, f"effect simulation-time frequency is {frequency:.2f} Hz")
    print(f"PASS  {len(pairs)} timestamp-matched raw/effect pairs; effect rate={frequency:.2f} Hz (simulation time)")
    require(
        effect.width == raw.width and effect.height == raw.height
        and effect.point_step == raw.point_step,
        "effect cloud does not preserve the organized layout",
    )
    require(
        effect.header.frame_id == raw.header.frame_id,
        "effect cloud changed frame_id",
    )

    raw_xyz = xyz_array(raw)
    effect_xyz = xyz_array(effect)
    raw_finite = np.isfinite(raw_xyz).all(axis=1)
    effect_finite = np.isfinite(effect_xyz).all(axis=1)
    require(raw_finite.sum() >= 1000, "raw cloud has too few finite points")

    dropped = raw_finite & ~effect_finite
    spurious = ~raw_finite & effect_finite
    require(not spurious.any(), "effect cloud invented points where raw had none")
    dropout_ratio = dropped.sum() / raw_finite.sum()
    require(
        abs(dropout_ratio - DROPOUT_PROBABILITY) < 0.04,
        f"dropout ratio {dropout_ratio:.3f} outside target {DROPOUT_PROBABILITY}",
    )

    kept = raw_finite & effect_finite
    displacement = np.linalg.norm(effect_xyz[kept] - raw_xyz[kept], axis=1)
    require(
        displacement.max() < 0.02,
        f"stationary rotation distortion too large: {displacement.max():.4f} m",
    )
    print(
        f"PASS  effect cloud live: dropout={dropout_ratio:.3f} "
        f"stationary distortion max={displacement.max():.5f} m layout/frame preserved"
    )

    accel_z = np.array([m.linear_acceleration.z for m in node.imu_messages])
    gyro_z = np.array([m.angular_velocity.z for m in node.imu_messages])
    accel_std = float(np.std(accel_z))
    gyro_std = float(np.std(gyro_z))
    require(
        abs(float(np.mean(accel_z)) - 9.80665) < 0.05,
        f"IMU accel z mean off gravity: {np.mean(accel_z):.4f}",
    )
    require(
        0.4 * IMU_ACCEL_STDDEV < accel_std < 3.0 * IMU_ACCEL_STDDEV,
        f"IMU accel noise stddev {accel_std:.5f} outside configured band",
    )
    require(
        0.4 * IMU_GYRO_STDDEV < gyro_std < 3.0 * IMU_GYRO_STDDEV,
        f"IMU gyro noise stddev {gyro_std:.6f} outside configured band",
    )
    print(
        f"PASS  IMU noise model: accel_z std={accel_std:.5f} m/s^2 "
        f"gyro_z std={gyro_std:.6f} rad/s around gravity mean"
    )


def main() -> int:
    urdf_path = sys.argv[1] if len(sys.argv) > 1 else ""
    try:
        unit_checks()
        if urdf_path:
            urdf_checks(urdf_path)
        rclpy.init()
        node = EffectHarness()
        try:
            live_checks(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()
    except (RuntimeError, KeyError) as error:
        print(f"FAIL sensor effects smoke: {error}", file=sys.stderr)
        return 1
    print("PASS  sensor effect channel and IMU noise model verified end to end.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
