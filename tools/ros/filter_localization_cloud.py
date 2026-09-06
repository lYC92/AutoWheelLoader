#!/usr/bin/env python3
"""Nominal level-ground KISS baseline crop, downstream of sensor effects.

The rendered plane repeats a ray-aligned sampling pattern as the vehicle moves.
Exclude its returns for this first point-to-point registration baseline; this
fixed lidar-frame crop is not ground segmentation for slopes or rough terrain.
"""
import array
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from loader_sensor_effects.lidar_effects_node import cloud_dtype


def select_points(xyz, min_z, min_range, max_range):
    radius = np.linalg.norm(xyz, axis=1)
    return xyz[np.isfinite(xyz).all(axis=1) & (xyz[:, 2] >= min_z)
               & (radius > min_range) & (radius < max_range)]


class BaselineCrop(Node):
    def __init__(self):
        super().__init__("loader_localization_crop")
        self.min_z = self.declare_parameter("min_z", -2.7).value
        self.min_range = self.declare_parameter("min_range", 3.0).value
        self.max_range = self.declare_parameter("max_range", 50.0).value
        if not np.isfinite([self.min_z, self.min_range, self.max_range]).all() or not 0 <= self.min_range < self.max_range:
            raise ValueError("invalid localization crop bounds")
        self.publisher = self.create_publisher(PointCloud2, "/loader/localization/points", 10)
        self.create_subscription(PointCloud2, "/loader/sensors/lidar/scan/points_effect", self.on_cloud, 10)

    def on_cloud(self, message):
        points = np.ndarray((message.height, message.width), dtype=cloud_dtype(message),
                            buffer=message.data, strides=(message.row_step, message.point_step))
        xyz = np.stack([points[k].ravel() for k in ("x", "y", "z")], axis=1)
        xyz = select_points(xyz, self.min_z, self.min_range, self.max_range)
        if len(xyz) < 20:
            self.get_logger().warning("fewer than 20 usable localization returns", throttle_duration_sec=5.)
            return
        out = PointCloud2(header=message.header, height=1, width=len(xyz),
                         is_bigendian=False, point_step=12, row_step=12*len(xyz), is_dense=True)
        out.fields = [PointField(name=k, offset=4*i, datatype=PointField.FLOAT32, count=1)
                      for i, k in enumerate(("x", "y", "z"))]
        out.data = array.array("B", xyz.astype("<f4").tobytes())
        self.publisher.publish(out)


def main():
    rclpy.init()
    node = BaselineCrop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
