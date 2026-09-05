#!/usr/bin/env python3
"""Lidar effect channel: dropout and scan motion distortion.

Subscribes to the ideal Gazebo point cloud and publishes the perturbed stream
that localization and perception algorithms must consume.  The ideal stream
stays available for ground-truth comparisons; effects are fully parameterized
and deterministic under a fixed seed.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2

from loader_sensor_effects.effects import (
    apply_rotation_distortion,
    dropout_keep_mask,
)

_FIELD_FORMATS = {
    1: "i1",  # INT8
    2: "u1",  # UINT8
    3: "i2",  # INT16
    4: "u2",  # UINT16
    5: "i4",  # INT32
    6: "u4",  # UINT32
    7: "f4",  # FLOAT32
    8: "f8",  # FLOAT64
}


def cloud_dtype(message: PointCloud2) -> np.dtype:
    byte_order = ">" if message.is_bigendian else "<"
    return np.dtype(
        {
            "names": [field.name for field in message.fields],
            "formats": [byte_order + _FIELD_FORMATS[field.datatype] for field in message.fields],
            "offsets": [field.offset for field in message.fields],
            "itemsize": message.point_step,
        }
    )


class LidarEffectsNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_effects_node")
        self.declare_parameter("input_topic", "/loader/sensors/lidar/scan/points")
        self.declare_parameter("output_topic", "/loader/sensors/lidar/scan/points_effect")
        self.declare_parameter("imu_topic", "/loader/sensors/imu")
        self.declare_parameter("dropout_probability", 0.0)
        self.declare_parameter("distortion_enabled", True)
        self.declare_parameter("random_seed", 0)
        self.declare_parameter("scan_period_s", 0.1)

        self.dropout_probability = float(self.get_parameter("dropout_probability").value)
        self.distortion_enabled = bool(self.get_parameter("distortion_enabled").value)
        self.random_seed = int(self.get_parameter("random_seed").value)
        self.scan_period_s = float(self.get_parameter("scan_period_s").value)

        self.angular_velocity = np.zeros(3)
        self.scan_sequence = 0

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.publisher = self.create_publisher(PointCloud2, output_topic, 10)
        self.create_subscription(PointCloud2, input_topic, self.on_cloud, 10)
        self.create_subscription(Imu, str(self.get_parameter("imu_topic").value), self.on_imu, 50)
        self.get_logger().info(
            f"Lidar effects ready: {input_topic} -> {output_topic} "
            f"(dropout={self.dropout_probability}, distortion={self.distortion_enabled}, "
            f"seed={self.random_seed})"
        )

    def on_imu(self, message: Imu) -> None:
        # Nominal approximation: gyro axes are aligned with the lidar axes.
        self.angular_velocity = np.array(
            [
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ]
        )

    def on_cloud(self, message: PointCloud2) -> None:
        count = message.width * message.height
        dtype = cloud_dtype(message)
        points = np.frombuffer(bytes(message.data), dtype=dtype, count=count).copy()

        xyz = np.stack([points["x"], points["y"], points["z"]], axis=1).astype(np.float64)
        finite = np.isfinite(xyz).all(axis=1)

        if self.distortion_enabled:
            distorted = xyz.copy()
            distorted[finite] = apply_rotation_distortion(
                xyz[finite], message.width, self.scan_period_s, self.angular_velocity
            )
            xyz = distorted

        keep = dropout_keep_mask(
            count, self.dropout_probability, self.random_seed + self.scan_sequence
        )
        self.scan_sequence += 1
        drop = ~keep & finite
        xyz[drop] = math.nan

        points["x"] = xyz[:, 0]
        points["y"] = xyz[:, 1]
        points["z"] = xyz[:, 2]

        out = PointCloud2()
        out.header = message.header
        out.fields = message.fields
        out.is_bigendian = message.is_bigendian
        out.point_step = message.point_step
        out.row_step = message.row_step
        out.height = message.height
        out.width = message.width
        out.is_dense = False
        out.data = points.tobytes()
        self.publisher.publish(out)


def main() -> None:
    rclpy.init()
    node = LidarEffectsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
