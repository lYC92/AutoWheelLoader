#!/usr/bin/env python3
"""Closed-loop smoke test for the nominal loader Gazebo dynamics plugin."""

from __future__ import annotations

import math
import sys
import time
from collections.abc import Callable

import rclpy
from loader_sim_msgs.msg import VehicleCommand, VehicleState
from rclpy.node import Node


FAULT_COMMAND_TIMEOUT = 1 << 0
FAULT_COMMAND_SATURATED = 1 << 2


class DynamicsHarness(Node):
    def __init__(self) -> None:
        super().__init__("loader_dynamics_smoke_test")
        self.publisher = self.create_publisher(VehicleCommand, "/loader/command", 10)
        self.subscription = self.create_subscription(
            VehicleState, "/loader/state", self._on_state, 10
        )
        self.states: list[VehicleState] = []

    def _on_state(self, message: VehicleState) -> None:
        self.states.append(message)
        if len(self.states) > 1000:
            del self.states[:500]

    def publish_command(
        self,
        *,
        gear: int = VehicleCommand.GEAR_NEUTRAL,
        traction_torque_nm: float = 0.0,
        brake_command: float = 0.0,
        articulation_rad: float = 0.0,
        lift_valve: float = 0.0,
        tilt_valve: float = 0.0,
        emergency_stop: bool = False,
    ) -> None:
        command = VehicleCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.gear = gear
        command.traction_torque_nm = traction_torque_nm
        command.brake_command = brake_command
        command.target_articulation_angle_rad = articulation_rad
        command.lift_valve_command = lift_valve
        command.tilt_valve_command = tilt_valve
        command.emergency_stop = emergency_stop
        self.publisher.publish(command)


def spin_until(
    node: DynamicsHarness,
    predicate: Callable[[], bool],
    timeout_s: float,
    description: str,
) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise RuntimeError(f"timed out waiting for {description}")


def run_command_phase(
    node: DynamicsHarness,
    duration_s: float,
    **command: float | int | bool,
) -> list[VehicleState]:
    first_index = len(node.states)
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        node.publish_command(**command)
        rclpy.spin_once(node, timeout_sec=0.02)
    return node.states[first_index:]


def joint_position(state: VehicleState, joint_name: str) -> float:
    try:
        index = list(state.joint_state.name).index(joint_name)
    except ValueError as error:
        raise RuntimeError(f"state is missing joint {joint_name}") from error
    return state.joint_state.position[index]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def main() -> int:
    rclpy.init()
    node = DynamicsHarness()
    try:
        spin_until(
            node,
            lambda: bool(node.states),
            8.0,
            "the first /loader/state message",
        )
        initial_state = node.states[-1]
        require(
            bool(initial_state.fault_flags & FAULT_COMMAND_TIMEOUT),
            "initial state did not report command timeout",
        )
        require(initial_state.emergency_stop_active, "initial timeout did not activate stop")

        controlled_states = run_command_phase(
            node,
            2.5,
            articulation_rad=0.15,
            lift_valve=0.65,
            tilt_valve=0.50,
        )
        require(len(controlled_states) >= 20, "state feedback rate is too low")
        require(
            not bool(controlled_states[-1].fault_flags & FAULT_COMMAND_TIMEOUT),
            "command timeout remained active during continuous command publication",
        )

        expected_motion = {
            "articulation_joint": 0.01,
            "lift_joint": 0.01,
            "bucket_tilt_joint": 0.01,
        }
        motions: dict[str, float] = {}
        for joint_name, minimum_motion in expected_motion.items():
            start = joint_position(initial_state, joint_name)
            motion = max(abs(joint_position(state, joint_name) - start) for state in controlled_states)
            motions[joint_name] = motion
            require(
                motion >= minimum_motion,
                f"{joint_name} moved only {motion:.6f} rad",
            )

        lift_peak = max(abs(state.lift_cylinder_pressure_pa) for state in controlled_states)
        tilt_peak = max(abs(state.tilt_cylinder_pressure_pa) for state in controlled_states)
        require(lift_peak > 1.0e6, f"lift pressure response too small: {lift_peak:.1f} Pa")
        require(tilt_peak > 1.0e6, f"tilt pressure response too small: {tilt_peak:.1f} Pa")

        for state in controlled_states:
            scalar_values = [
                state.longitudinal_speed_mps,
                state.lift_cylinder_position_m,
                state.tilt_cylinder_position_m,
                state.lift_cylinder_pressure_pa,
                state.tilt_cylinder_pressure_pa,
                *state.wheel_speed_radps,
                *state.joint_state.position,
                *state.joint_state.velocity,
                *state.joint_state.effort,
            ]
            require(all(math.isfinite(value) for value in scalar_values), "non-finite state detected")

        saturated_states = run_command_phase(
            node,
            0.6,
            traction_torque_nm=1.0e9,
            articulation_rad=0.15,
        )
        require(
            any(state.fault_flags & FAULT_COMMAND_SATURATED for state in saturated_states),
            "out-of-range command did not set saturation fault",
        )

        stopped_states = run_command_phase(node, 0.6, emergency_stop=True)
        require(
            any(state.emergency_stop_active for state in stopped_states),
            "explicit emergency stop was not reflected in state",
        )
        require(
            any(not (state.fault_flags & FAULT_COMMAND_TIMEOUT) for state in stopped_states),
            "explicit emergency-stop phase was incorrectly classified only as timeout",
        )

        first_timeout_index = len(node.states)
        spin_until(
            node,
            lambda: any(
                state.fault_flags & FAULT_COMMAND_TIMEOUT
                for state in node.states[first_timeout_index:]
            ),
            2.0,
            "command watchdog timeout",
        )
        timeout_state = node.states[-1]
        require(timeout_state.emergency_stop_active, "watchdog timeout did not activate stop")

        print(
            "PASS loader dynamics closed-loop smoke: "
            f"states={len(node.states)} "
            f"articulation_motion={motions['articulation_joint']:.4f}rad "
            f"lift_motion={motions['lift_joint']:.4f}rad "
            f"tilt_motion={motions['bucket_tilt_joint']:.4f}rad "
            f"lift_peak={lift_peak / 1.0e6:.2f}MPa "
            f"tilt_peak={tilt_peak / 1.0e6:.2f}MPa"
        )
        return 0
    except RuntimeError as error:
        print(f"FAIL loader dynamics closed-loop smoke: {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
