#!/usr/bin/env python3
"""Force-controlled driving scenario, independent of the soil contact benchmark."""
import math
import time
import rclpy
from loader_sim_msgs.msg import VehicleCommand, VehicleState
from rclpy.node import Node
from rclpy.parameter import Parameter


def main():
    rclpy.init()
    node = Node("loader_localization_scenario", parameter_overrides=[Parameter("use_sim_time", value=True)])
    states = []
    node.create_subscription(VehicleState, "/loader/state", states.append, 10)
    publisher = node.create_publisher(VehicleCommand, "/loader/command", 10)
    try:
        deadline = time.monotonic()+15
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if states and publisher.get_subscription_count() and node.get_clock().now().nanoseconds > 0:
                break
        if not states or not publisher.get_subscription_count():
            raise RuntimeError("vehicle control chain not ready")
        max_speed = 0.
        # Raise before travel. Holding valves by joint feedback avoids depending
        # on how far gravity lowered the bucket during sensor initialization.
        for label, duration, gear, torque in [
            ("prepare", 3., 0, 0.), ("reverse", 6., -1, -10000.),
            ("brake", 1., 0, 0.), ("forward", 4., 1, 10000.),
            ("stop", 1., 0, 0.),
        ]:
            end = node.get_clock().now().nanoseconds/1e9+duration
            wall_end = time.monotonic()+max(30, duration*10)
            next_publish = 0.
            while node.get_clock().now().nanoseconds/1e9 < end:
                if time.monotonic() > wall_end:
                    raise RuntimeError("simulation clock stalled")
                state = states[-1]
                joint = dict(zip(state.joint_state.name, state.joint_state.position))
                if time.monotonic() >= next_publish:
                    command = VehicleCommand()
                    command.header.stamp = node.get_clock().now().to_msg()
                    command.header.frame_id = "base_link"
                    command.gear, command.traction_torque_nm = gear, torque
                    command.brake_command = 1. if gear == 0 else 0.
                    command.lift_valve_command = max(-1., min(1., .15+3.*(.25-joint["lift_joint"])))
                    command.tilt_valve_command = max(-1., min(1., .08+3.*(.35-joint["bucket_tilt_joint"])))
                    publisher.publish(command)
                    next_publish = time.monotonic()+.02
                max_speed = max(max_speed, abs(state.longitudinal_speed_mps))
                rclpy.spin_once(node, timeout_sec=.01)
            print(f"PASS  localization maneuver: {label}", flush=True)
        if not math.isfinite(max_speed) or max_speed < .2:
            raise RuntimeError(f"vehicle did not move: max speed {max_speed}")
        print(f"PASS  localization driving scenario; peak wheel-derived speed={max_speed:.3f} m/s")
    finally:
        command = VehicleCommand()
        command.brake_command, command.emergency_stop = 1., True
        for _ in range(5):
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=.02)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
