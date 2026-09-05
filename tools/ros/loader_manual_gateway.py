#!/usr/bin/env python3
"""Safe Foxglove teleoperation adapter for the loader simulation."""

from __future__ import annotations

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from loader_sim_msgs.msg import VehicleCommand
from rclpy.node import Node
from std_msgs.msg import Bool, String


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class LoaderManualGateway(Node):
    """Combine two dead-man Twist inputs into the loader command contract."""

    def __init__(self) -> None:
        super().__init__("loader_manual_gateway")
        self.declare_parameter("maximum_traction_torque_nm", 10_000.0)
        self.declare_parameter("maximum_articulation_angle_rad", 0.45)
        self.declare_parameter("input_timeout_s", 0.35)
        self.declare_parameter("command_rate_hz", 50.0)
        self.declare_parameter("start_enabled", True)

        self.maximum_traction_torque_nm = float(
            self.get_parameter("maximum_traction_torque_nm").value
        )
        self.maximum_articulation_angle_rad = float(
            self.get_parameter("maximum_articulation_angle_rad").value
        )
        self.input_timeout_s = float(self.get_parameter("input_timeout_s").value)
        command_rate_hz = float(self.get_parameter("command_rate_hz").value)
        self.enabled = bool(self.get_parameter("start_enabled").value)
        self.emergency_stop = False

        self.drive = Twist()
        self.hydraulics = Twist()
        self.last_drive_wall_s = -math.inf
        self.last_hydraulics_wall_s = -math.inf
        self.last_status_wall_s = -math.inf

        self.command_publisher = self.create_publisher(
            VehicleCommand, "/loader/command", 10
        )
        self.status_publisher = self.create_publisher(
            String, "/loader/manual/status", 10
        )
        self.create_subscription(Twist, "/loader/manual/drive", self.on_drive, 10)
        self.create_subscription(
            Twist, "/loader/manual/hydraulics", self.on_hydraulics, 10
        )
        self.create_subscription(Bool, "/loader/manual/enable", self.on_enable, 10)
        self.create_subscription(
            Bool, "/loader/manual/emergency_stop", self.on_emergency_stop, 10
        )
        self.create_timer(1.0 / command_rate_hz, self.publish_command)
        self.get_logger().info(
            "Manual gateway ready: hold Foxglove Teleop buttons to command the loader"
        )

    def on_drive(self, message: Twist) -> None:
        self.drive = message
        self.last_drive_wall_s = time.monotonic()

    def on_hydraulics(self, message: Twist) -> None:
        self.hydraulics = message
        self.last_hydraulics_wall_s = time.monotonic()

    def on_enable(self, message: Bool) -> None:
        self.enabled = bool(message.data)
        state = "enabled" if self.enabled else "disabled"
        self.get_logger().info(f"Manual control {state}")

    def on_emergency_stop(self, message: Bool) -> None:
        self.emergency_stop = bool(message.data)
        state = "ACTIVE" if self.emergency_stop else "released"
        self.get_logger().warning(f"Manual emergency stop {state}")

    def publish_command(self) -> None:
        wall_now_s = time.monotonic()
        drive_fresh = wall_now_s - self.last_drive_wall_s <= self.input_timeout_s
        hydraulics_fresh = (
            wall_now_s - self.last_hydraulics_wall_s <= self.input_timeout_s
        )

        drive_value = (
            clamp(float(self.drive.linear.x), -1.0, 1.0)
            if self.enabled and drive_fresh
            else 0.0
        )
        steering_value = (
            clamp(float(self.drive.angular.z), -1.0, 1.0)
            if self.enabled and drive_fresh
            else 0.0
        )
        lift_value = (
            clamp(float(self.hydraulics.linear.z), -1.0, 1.0)
            if self.enabled and hydraulics_fresh
            else 0.0
        )
        tilt_value = (
            clamp(float(self.hydraulics.angular.y), -1.0, 1.0)
            if self.enabled and hydraulics_fresh
            else 0.0
        )

        command = VehicleCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "base_link"
        if drive_value > 1.0e-3:
            command.gear = VehicleCommand.GEAR_FORWARD
        elif drive_value < -1.0e-3:
            command.gear = VehicleCommand.GEAR_REVERSE
        else:
            command.gear = VehicleCommand.GEAR_NEUTRAL
        command.traction_torque_nm = (
            drive_value * self.maximum_traction_torque_nm
        )
        command.brake_command = 0.0 if abs(drive_value) > 1.0e-3 else 1.0
        command.target_articulation_angle_rad = (
            steering_value * self.maximum_articulation_angle_rad
        )
        command.lift_valve_command = lift_value
        command.tilt_valve_command = tilt_value
        command.emergency_stop = self.emergency_stop or not self.enabled
        self.command_publisher.publish(command)

        if wall_now_s - self.last_status_wall_s >= 0.2:
            status = String()
            status.data = json.dumps(
                {
                    "enabled": self.enabled,
                    "emergency_stop": command.emergency_stop,
                    "drive_input_fresh": drive_fresh,
                    "hydraulic_input_fresh": hydraulics_fresh,
                    "gear": int(command.gear),
                    "traction_torque_nm": command.traction_torque_nm,
                    "target_articulation_angle_rad": (
                        command.target_articulation_angle_rad
                    ),
                    "lift_valve_command": command.lift_valve_command,
                    "tilt_valve_command": command.tilt_valve_command,
                },
                separators=(",", ":"),
            )
            self.status_publisher.publish(status)
            self.last_status_wall_s = wall_now_s


def main() -> None:
    rclpy.init()
    node = LoaderManualGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
