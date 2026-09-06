#!/usr/bin/env python3
"""Filter effect returns using scan-time articulated TF and ground fitting."""
import argparse
import array
from collections import deque
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import PointCloud2, PointField
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener, TransformException
from loader_sensor_effects.lidar_effects_node import cloud_dtype
from localization_geometry import ground_plane, outside_body, range_mask, visual_bounds


def select_points(xyz, min_z, min_range, max_range):
    """Original crop retained for explicit historical comparisons."""
    return xyz[range_mask(xyz, min_range, max_range) & (xyz[:, 2] >= min_z)]


class BaselineCrop(Node):
    def __init__(self, urdf=None):
        super().__init__("loader_localization_crop")
        self.min_z = self.declare_parameter("min_z", -2.7).value
        self.min_range = self.declare_parameter("min_range", 3.0).value
        self.max_range = self.declare_parameter("max_range", 50.0).value
        self.mode = self.declare_parameter('ground_mode', 'adaptive').value
        self.threshold = self.declare_parameter('ground_threshold', .18).value
        self.self_filter = self.declare_parameter('self_filter', True).value
        self.margin = self.declare_parameter('body_margin', .10).value
        if (not np.isfinite([self.min_z, self.min_range, self.max_range, self.threshold, self.margin]).all()
                or not 0 <= self.min_range < self.max_range or self.threshold <= 0 or self.margin < 0
                or self.mode not in ('fixed', 'adaptive')):
            raise ValueError('invalid localization filter configuration')
        if self.self_filter and not urdf:
            raise ValueError('--model-urdf is required when self_filter is enabled')
        self.bounds = visual_bounds(urdf) if self.self_filter else []
        self.links = sorted({b[0] for b in self.bounds})
        self.buffer = Buffer(cache_time=Duration(seconds=10.), node=self)
        self.listener = TransformListener(self.buffer, self)
        self.pending = deque()
        self.last_stamp = None
        self.publisher = self.create_publisher(PointCloud2, "/loader/localization/points", 10)
        self.create_subscription(PointCloud2, "/loader/sensors/lidar/scan/points_effect", self.on_cloud, 10)
        self.create_timer(.02, self.drain)
        self.get_logger().info(f'ground={self.mode}, self_filter={self.self_filter}, visual_boxes={len(self.bounds)}')

    def on_cloud(self, message):
        stamp = Time.from_msg(message.header.stamp).nanoseconds
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.pending.clear()
        self.last_stamp = stamp
        if len(self.pending) >= 5:
            self.pending.popleft()
            self.get_logger().warning('self-filter queue overflow: dropped oldest scan', throttle_duration_sec=5.)
        self.pending.append((message, time.monotonic()))
        self.drain()

    def drain(self):
        while self.pending:
            message, arrival = self.pending[0]
            transforms = {}
            try:
                for link in self.links:
                    tf = self.buffer.lookup_transform(message.header.frame_id, link, Time.from_msg(message.header.stamp)).transform
                    matrix = np.eye(4)
                    q, t = tf.rotation, tf.translation
                    matrix[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
                    matrix[:3, 3] = [t.x, t.y, t.z]
                    transforms[link] = matrix
            except TransformException:
                # Allow joint TF to arrive asynchronously. Never use latest TF.
                if time.monotonic()-arrival < .5:
                    return
                self.pending.popleft()
                self.get_logger().warning('missing scan-time body TF: dropped scan', throttle_duration_sec=5.)
                continue
            self.pending.popleft()
            self.publish_filtered(message, transforms)

    def publish_filtered(self, message, transforms):
        points = np.ndarray((message.height, message.width), dtype=cloud_dtype(message),
                            buffer=message.data, strides=(message.row_step, message.point_step))
        xyz = np.stack([points[k].ravel() for k in ("x", "y", "z")], axis=1)
        xyz = xyz[range_mask(xyz, self.min_range, self.max_range)]
        if self.bounds:
            xyz = xyz[outside_body(xyz, self.bounds, transforms, self.margin)]
        if self.mode == 'fixed':
            xyz = xyz[xyz[:, 2] >= self.min_z]
        else:
            plane = ground_plane(xyz, threshold=self.threshold)
            if plane is not None:
                normal, offset = plane
                xyz = xyz[xyz @ normal + offset > self.threshold]
            else:
                self.get_logger().warning('ground plane unsupported: retaining non-body returns', throttle_duration_sec=5.)
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
    parser = argparse.ArgumentParser()
    parser.add_argument('--model-urdf')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = BaselineCrop(args.model_urdf)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
