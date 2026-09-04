#!/usr/bin/env python3
"""Validate ROS 2 lidar, IMU, and simulation-clock output from Gazebo."""

from __future__ import annotations

import math
import struct
import sys
import time

import rclpy
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2


def stamp_ns(stamp: object) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class SensorHarness(Node):
    def __init__(self) -> None:
        super().__init__("loader_sensor_smoke_test")
        self.point_clouds: list[PointCloud2] = []
        self.imu_messages: list[Imu] = []
        self.clocks: list[Clock] = []
        self.create_subscription(
            PointCloud2,
            "/loader/sensors/lidar/scan/points",
            self.point_clouds.append,
            10,
        )
        self.create_subscription(Imu, "/loader/sensors/imu", self.imu_messages.append, 50)
        self.create_subscription(Clock, "/clock", self.clocks.append, 50)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def finite_xyz_count(message: PointCloud2, sample_limit: int = 4096) -> int:
    field_offsets = {field.name: field.offset for field in message.fields}
    require(all(axis in field_offsets for axis in ("x", "y", "z")), "point cloud lacks XYZ fields")
    count = 0
    sample_count = min(message.width * message.height, sample_limit)
    byte_order = ">" if message.is_bigendian else "<"
    for index in range(sample_count):
        row = index // message.width
        column = index % message.width
        base = row * message.row_step + column * message.point_step
        coordinates = [
            struct.unpack_from(byte_order + "f", message.data, base + field_offsets[axis])[0]
            for axis in ("x", "y", "z")
        ]
        if all(math.isfinite(value) for value in coordinates):
            count += 1
    return count


def monotonic_sim_stamps(messages: list[object]) -> bool:
    stamps = [stamp_ns(message.header.stamp) for message in messages]
    return all(current >= previous for previous, current in zip(stamps, stamps[1:]))


def main() -> int:
    rclpy.init()
    node = SensorHarness()
    try:
        start = time.monotonic()
        deadline = start + 8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            elapsed = time.monotonic() - start
            if elapsed >= 3.0 and len(node.point_clouds) >= 20 and len(node.imu_messages) >= 150:
                break

        elapsed = time.monotonic() - start
        require(len(node.point_clouds) >= 20, f"only {len(node.point_clouds)} lidar frames received")
        require(len(node.imu_messages) >= 150, f"only {len(node.imu_messages)} IMU frames received")
        require(len(node.clocks) >= 100, f"only {len(node.clocks)} clock messages received")

        cloud = node.point_clouds[-1]
        require(cloud.width == 1024, f"unexpected lidar width {cloud.width}")
        require(cloud.height == 32, f"unexpected lidar height {cloud.height}")
        require(cloud.point_step > 0 and len(cloud.data) > 0, "empty lidar data payload")
        require(bool(cloud.header.frame_id), "lidar frame_id is empty")
        require(
            "lidar_link" in cloud.header.frame_id,
            f"lidar frame does not preserve lidar_link: {cloud.header.frame_id}",
        )
        valid_points = finite_xyz_count(cloud)
        require(valid_points >= 50, f"only {valid_points} finite points in sampled cloud")

        require(monotonic_sim_stamps(node.point_clouds), "lidar timestamps are not monotonic")
        require(monotonic_sim_stamps(node.imu_messages), "IMU timestamps are not monotonic")
        require(stamp_ns(cloud.header.stamp) > 0, "lidar timestamp is zero")
        require(stamp_ns(node.imu_messages[-1].header.stamp) > 0, "IMU timestamp is zero")

        imu = node.imu_messages[-1]
        require(
            "imu_link" in imu.header.frame_id,
            f"IMU frame does not preserve imu_link: {imu.header.frame_id}",
        )
        imu_values = [
            imu.orientation.x,
            imu.orientation.y,
            imu.orientation.z,
            imu.orientation.w,
            imu.angular_velocity.x,
            imu.angular_velocity.y,
            imu.angular_velocity.z,
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z,
        ]
        require(all(math.isfinite(value) for value in imu_values), "non-finite IMU value")

        clock_values = [stamp_ns(message.clock) for message in node.clocks]
        require(
            all(current >= previous for previous, current in zip(clock_values, clock_values[1:])),
            "/clock is not monotonic",
        )

        lidar_rate = len(node.point_clouds) / elapsed
        imu_rate = len(node.imu_messages) / elapsed
        require(lidar_rate >= 7.0, f"lidar rate too low: {lidar_rate:.2f} Hz")
        require(imu_rate >= 60.0, f"IMU rate too low: {imu_rate:.2f} Hz")

        print(
            "PASS loader sensor ROS bridge smoke: "
            f"lidar={lidar_rate:.2f}Hz imu={imu_rate:.2f}Hz "
            f"cloud={cloud.width}x{cloud.height} sampled_finite={valid_points} "
            f"lidar_frame={cloud.header.frame_id} imu_frame={imu.header.frame_id}"
        )
        return 0
    except RuntimeError as error:
        print(f"FAIL loader sensor ROS bridge smoke: {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
