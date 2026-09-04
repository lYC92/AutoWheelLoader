#!/usr/bin/env python3
"""Exercise the loader, ros2_control, and nominal Gazebo soil slice together."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import struct
import sys
import time

import rclpy
from loader_sim_msgs.msg import BucketInteraction, TerrainState, VehicleCommand, VehicleState
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2


class SoilCouplingHarness(Node):
    def __init__(self, observer_topic: str | None, use_sim_time_for_phases: bool) -> None:
        super().__init__(
            "loader_soil_coupling_smoke_test",
            parameter_overrides=[
                Parameter("use_sim_time", value=use_sim_time_for_phases),
            ],
        )
        self.use_sim_time_for_phases = use_sim_time_for_phases
        self.command_publisher = self.create_publisher(VehicleCommand, "/loader/command", 10)
        self.interactions: list[BucketInteraction] = []
        self.terrain_states: list[TerrainState] = []
        self.vehicle_states: list[VehicleState] = []
        self.observer_clouds: list[PointCloud2] = []
        self.create_subscription(
            BucketInteraction,
            "/loader/bucket_interaction",
            self.interactions.append,
            50,
        )
        self.create_subscription(TerrainState, "/loader/terrain_state", self.terrain_states.append, 50)
        self.create_subscription(VehicleState, "/loader/state", self.vehicle_states.append, 50)
        if observer_topic is not None:
            self.create_subscription(PointCloud2, observer_topic, self.observer_clouds.append, 10)

    def phase_time_s(self) -> float:
        if self.use_sim_time_for_phases:
            return self.get_clock().now().nanoseconds * 1.0e-9
        return time.monotonic()

    def publish_command(
        self,
        *,
        gear: int,
        traction_torque_nm: float,
        brake_command: float = 0.0,
        lift_valve: float = 0.0,
        tilt_valve: float = 0.0,
        emergency_stop: bool = False,
    ) -> None:
        command = VehicleCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.gear = gear
        command.traction_torque_nm = traction_torque_nm
        command.brake_command = brake_command
        command.target_articulation_angle_rad = 0.0
        command.lift_valve_command = lift_valve
        command.tilt_valve_command = tilt_valve
        command.emergency_stop = emergency_stop
        self.command_publisher.publish(command)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def joint_position(state: VehicleState, joint_name: str) -> float:
    try:
        index = list(state.joint_state.name).index(joint_name)
    except ValueError as error:
        raise RuntimeError(f"state is missing joint {joint_name}") from error
    return state.joint_state.position[index]


def point_xyz(message: PointCloud2) -> list[tuple[float, float, float]]:
    offsets = {field.name: field.offset for field in message.fields}
    require(all(axis in offsets for axis in ("x", "y", "z")), "observer cloud lacks XYZ")
    byte_order = ">" if message.is_bigendian else "<"
    points: list[tuple[float, float, float]] = []
    for index in range(message.width * message.height):
        row = index // message.width
        column = index % message.width
        base = row * message.row_step + column * message.point_step
        xyz = tuple(
            struct.unpack_from(byte_order + "f", message.data, base + offsets[axis])[0]
            for axis in ("x", "y", "z")
        )
        points.append(xyz)
    return points


def run_phase(
    node: SoilCouplingHarness,
    duration_s: float,
    *,
    gear: int,
    torque: float,
    brake: float = 0.0,
    lift_valve: float = 0.0,
    tilt_valve: float = 0.0,
    emergency_stop: bool = False,
) -> None:
    deadline = node.phase_time_s() + duration_s
    wall_deadline = time.monotonic() + max(60.0, 20.0 * duration_s)
    while node.phase_time_s() < deadline:
        if time.monotonic() >= wall_deadline:
            raise RuntimeError(
                f"simulation clock did not advance through a {duration_s:.1f}s phase"
            )
        node.publish_command(
            gear=gear,
            traction_torque_nm=torque,
            brake_command=brake,
            lift_valve=lift_valve,
            tilt_valve=tilt_valve,
            emergency_stop=emergency_stop,
        )
        rclpy.spin_once(node, timeout_sec=0.02)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--proxy-expectation", type=Path)
    parser.add_argument("--observer-topic")
    parser.add_argument(
        "--use-sim-time-for-phases",
        action="store_true",
        help="measure command phases in simulation time for an interactive GUI run",
    )
    args = parser.parse_args()
    rclpy.init()
    node = SoilCouplingHarness(args.observer_topic, args.use_sim_time_for_phases)
    try:
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if (
                node.interactions
                and node.terrain_states
                and node.vehicle_states
                and (args.observer_topic is None or len(node.observer_clouds) >= 3)
            ):
                break
        require(bool(node.interactions), "no BucketInteraction messages")
        require(bool(node.terrain_states), "no TerrainState messages")
        require(bool(node.vehicle_states), "no VehicleState messages")
        if args.observer_topic is not None:
            require(len(node.observer_clouds) >= 3, "no initial observer lidar point clouds")
            initial_observer_cloud = node.observer_clouds[-1]
        else:
            initial_observer_cloud = None

        initial_terrain = node.terrain_states[-1]
        require(initial_terrain.initial_volume_m3 > 1.0, "initial terrain volume is invalid")
        initial_profile = list(initial_terrain.height_profile_m)
        require(len(initial_profile) == 280, "terrain height profile has the wrong size")
        require(initial_terrain.cell_size_m == 0.05, "terrain cell size is incorrect")
        require(initial_terrain.slice_width_m == 2.7, "terrain slice width is incorrect")
        run_phase(
            node,
            1.0,
            gear=VehicleCommand.GEAR_NEUTRAL,
            torque=0.0,
        )
        first_interaction_index = len(node.interactions)
        first_state_index = len(node.vehicle_states)
        run_phase(
            node,
            6.0,
            gear=VehicleCommand.GEAR_FORWARD,
            torque=10000.0,
        )
        cutting = node.interactions[first_interaction_index:]
        driving = node.vehicle_states[first_state_index:]
        require(bool(cutting), "no soil feedback during drive phase")
        require(bool(driving), "no vehicle feedback during drive phase")

        maximum_wheel_speed = max(
            abs(speed)
            for state in driving
            for speed in state.wheel_speed_radps
        )
        maximum_vehicle_speed = max(abs(state.longitudinal_speed_mps) for state in driving)
        maximum_penetration = max(state.maximum_penetration_depth_m for state in cutting)
        maximum_force = max(
            math.hypot(state.bucket_wrench.force.x, state.bucket_wrench.force.z)
            for state in cutting
        )
        maximum_inflow = max(state.material_inflow_m3ps for state in cutting)
        final_interaction = cutting[-1]
        final_terrain = node.terrain_states[-1]
        maximum_reported_payload_mass = max(state.bucket_payload_mass_kg for state in driving)

        require(maximum_wheel_speed > 0.1, "wheel torque did not produce wheel motion")
        require(maximum_vehicle_speed > 0.05, "wheel torque did not produce vehicle motion")
        require(maximum_vehicle_speed < 4.0, "soil proxy caused an implausible speed transient")
        require(maximum_penetration > 0.01, "bucket never penetrated analytic terrain")
        require(maximum_force > 1000.0, "analytic terrain never applied meaningful bucket force")
        require(maximum_inflow > 0.0, "no material inflow was observed")
        require(
            final_interaction.bucket_material_volume_m3 > 1.0e-4,
            "no terrain material reached the bucket payload",
        )
        require(
            abs(
                final_interaction.bucket_material_mass_kg
                - 1600.0 * final_interaction.bucket_material_volume_m3
            )
            <= 1.0e-6,
            "payload mass and volume are inconsistent",
        )
        require(
            abs(final_terrain.relative_volume_conservation_error) <= 1.0e-9,
            "terrain/bucket volume ledger is not conservative",
        )
        require(
            max(
                abs(current - initial)
                for current, initial in zip(final_terrain.height_profile_m, initial_profile)
            )
            > 0.01,
            "terrain height profile did not change during excavation",
        )
        require(
            maximum_reported_payload_mass > 1.0,
            "VehicleState did not receive soil payload feedback",
        )
        require(
            abs(maximum_reported_payload_mass - final_interaction.bucket_material_mass_kg) <= 5.0,
            "VehicleState payload differs from BucketInteraction payload",
        )
        require(
            all(
                state.bucket_wrench.force.x <= 1.0e-6
                for state in cutting
                if math.hypot(state.bucket_wrench.force.x, state.bucket_wrench.force.z) > 1.0
            ),
            "soil force did not oppose forward travel",
        )

        cut_volume = final_interaction.bucket_material_volume_m3
        lift_state_start_index = len(node.vehicle_states)
        run_phase(
            node,
            2.2,
            gear=VehicleCommand.GEAR_NEUTRAL,
            torque=0.0,
            brake=1.0,
            lift_valve=1.0,
        )
        lifting_states = node.vehicle_states[lift_state_start_index:]
        require(bool(lifting_states), "no vehicle feedback during lift phase")
        lift_positions = [joint_position(state, "lift_joint") for state in lifting_states]
        require(
            max(lift_positions) - min(lift_positions) > 0.10,
            "lift command did not move the bucket clear of the ground",
        )
        loaded_volume = node.interactions[-1].bucket_material_volume_m3
        require(loaded_volume >= cut_volume, "payload unexpectedly decreased during lift")

        reverse_state_start_index = len(node.vehicle_states)
        run_phase(
            node,
            3.0,
            gear=VehicleCommand.GEAR_REVERSE,
            torque=-10000.0,
        )
        reversing_states = node.vehicle_states[reverse_state_start_index:]
        require(bool(reversing_states), "no vehicle feedback during reverse phase")
        require(
            min(state.longitudinal_speed_mps for state in reversing_states) < -0.20,
            "loader did not reverse away from the source pile",
        )
        run_phase(
            node,
            0.8,
            gear=VehicleCommand.GEAR_NEUTRAL,
            torque=0.0,
            brake=1.0,
        )

        dump_start_index = len(node.interactions)
        dump_state_start_index = len(node.vehicle_states)
        run_phase(
            node,
            3.0,
            gear=VehicleCommand.GEAR_NEUTRAL,
            torque=0.0,
            brake=1.0,
            tilt_valve=-1.0,
        )
        dumping = node.interactions[dump_start_index:]
        dump_vehicle_states = node.vehicle_states[dump_state_start_index:]
        require(bool(dumping), "no soil feedback during dump phase")
        require(bool(dump_vehicle_states), "no vehicle feedback during dump phase")
        maximum_outflow = max(state.material_outflow_m3ps for state in dumping)
        tilt_positions = [
            joint_position(state, "bucket_tilt_joint") for state in dump_vehicle_states
        ]
        post_dump_interaction = dumping[-1]
        post_dump_terrain = node.terrain_states[-1]
        require(
            maximum_outflow > 0.1,
            "tilting the bucket did not start material outflow "
            f"(tilt range {min(tilt_positions):.3f}..{max(tilt_positions):.3f} rad)",
        )
        require(
            post_dump_interaction.bucket_material_volume_m3 < 0.05 * loaded_volume,
            "bucket retained too much material after unloading "
            f"(loaded={loaded_volume:.3f}, remaining="
            f"{post_dump_interaction.bucket_material_volume_m3:.3f}, "
            f"dumped={post_dump_terrain.dumped_volume_m3:.3f}, "
            f"tilt={min(tilt_positions):.3f}..{max(tilt_positions):.3f})",
        )
        require(
            post_dump_terrain.dumped_volume_m3 > 0.90 * loaded_volume,
            "terrain did not record the unloaded volume",
        )
        require(
            abs(post_dump_terrain.relative_volume_conservation_error) <= 1.0e-9,
            "terrain/bucket volume ledger failed during unloading",
        )
        final_profile = list(post_dump_terrain.height_profile_m)
        require(len(final_profile) == len(initial_profile), "final height profile size changed")
        profile_volume = (
            sum(final_profile)
            * post_dump_terrain.cell_size_m
            * post_dump_terrain.slice_width_m
        )
        require(
            abs(profile_volume - post_dump_terrain.remaining_volume_m3) <= 1.0e-9,
            "height profile volume differs from TerrainState remaining volume",
        )
        changed_index = max(
            range(len(final_profile)),
            key=lambda index: abs(final_profile[index] - initial_profile[index]),
        )
        maximum_profile_change = abs(
            final_profile[changed_index] - initial_profile[changed_index]
        )
        require(maximum_profile_change > 0.01, "transport and unloading left no terrain change")
        if args.proxy_expectation is not None:
            args.proxy_expectation.parent.mkdir(parents=True, exist_ok=True)
            expected_center_z = final_profile[changed_index] - 0.9
            args.proxy_expectation.write_text(
                f"{changed_index} {expected_center_z:.9f}\n", encoding="utf-8"
            )

        run_phase(
            node,
            0.6,
            gear=VehicleCommand.GEAR_NEUTRAL,
            torque=0.0,
            emergency_stop=True,
        )
        require(node.vehicle_states[-1].emergency_stop_active, "final emergency stop not active")

        lidar_summary = ""
        if initial_observer_cloud is not None:
            require(bool(node.observer_clouds), "no final observer lidar point cloud")
            final_observer_cloud = node.observer_clouds[-1]
            require(
                (initial_observer_cloud.width, initial_observer_cloud.height)
                == (final_observer_cloud.width, final_observer_cloud.height),
                "observer lidar shape changed",
            )
            initial_points = point_xyz(initial_observer_cloud)
            final_points = point_xyz(final_observer_cloud)
            paired_points = [
                (initial, final)
                for initial, final in zip(initial_points, final_points)
                if all(math.isfinite(value) for value in (*initial, *final))
            ]
            differences = [
                abs(math.dist((0.0, 0.0, 0.0), final) - math.dist((0.0, 0.0, 0.0), initial))
                for initial, final in paired_points
            ]
            require(len(differences) > 100, "too few paired finite observer lidar rays")
            changed_rays = sum(difference > 0.05 for difference in differences)
            maximum_range_change = max(differences)
            require(changed_rays >= 5, "observer lidar did not detect terrain geometry changes")
            changed_cell_x = (
                post_dump_terrain.domain_min_m
                + (changed_index + 0.5) * post_dump_terrain.cell_size_m
            )
            terrain_changed_rays = sum(
                abs(math.dist((0.0, 0.0, 0.0), final) - math.dist((0.0, 0.0, 0.0), initial))
                > 0.05
                and (
                    abs((5.3 - initial[1]) - changed_cell_x) <= 0.20
                    or abs((5.3 - final[1]) - changed_cell_x) <= 0.20
                )
                for initial, final in paired_points
            )
            require(
                terrain_changed_rays >= 3,
                "observer lidar changes did not intersect the changed terrain columns",
            )
            lidar_summary = (
                f" lidar_changed_rays={changed_rays} "
                f"terrain_rays={terrain_changed_rays} "
                f"lidar_max_delta={maximum_range_change:.3f}m"
            )

        print(
            "PASS loader-soil coupled smoke: "
            f"wheel_speed={maximum_wheel_speed:.3f}rad/s "
            f"vehicle_speed={maximum_vehicle_speed:.3f}m/s "
            f"penetration={maximum_penetration:.3f}m "
            f"peak_force={maximum_force / 1000.0:.2f}kN "
            f"payload={final_interaction.bucket_material_volume_m3:.6f}m3/"
            f"{final_interaction.bucket_material_mass_kg:.2f}kg "
            f"unloaded={post_dump_terrain.dumped_volume_m3:.6f}m3 "
            f"terrain_delta={maximum_profile_change:.3f}m "
            f"balance={post_dump_terrain.relative_volume_conservation_error:.3e}"
            f"{lidar_summary}"
        )
        return 0
    except RuntimeError as error:
        print(f"FAIL loader-soil coupled smoke: {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
