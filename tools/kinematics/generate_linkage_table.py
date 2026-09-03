#!/usr/bin/env python3
"""Generate and validate the nominal two-coordinate loader linkage table."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def rotate(point: tuple[float, float], angle: float) -> tuple[float, float]:
    x, z = point
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return cosine * x - sine * z, sine * x + cosine * z


def rotate_derivative(point: tuple[float, float], angle: float) -> tuple[float, float]:
    x, z = point
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return -sine * x - cosine * z, cosine * x - sine * z


def add(first: tuple[float, float], second: tuple[float, float]) -> tuple[float, float]:
    return first[0] + second[0], first[1] + second[1]


def subtract(first: tuple[float, float], second: tuple[float, float]) -> tuple[float, float]:
    return first[0] - second[0], first[1] - second[1]


def length_and_derivative(vector: tuple[float, float], derivative: tuple[float, float]) -> tuple[float, float]:
    length = math.hypot(*vector)
    if length <= 0.0:
        raise ValueError("Cylinder length is zero")
    rate = (vector[0] * derivative[0] + vector[1] * derivative[1]) / length
    return length, rate


def linkage_state(config: dict, lift_angle: float, tilt_angle: float) -> dict[str, float]:
    geometry = config["geometry_m"]
    lift_pivot = tuple(geometry["lift_pivot_front_frame_xz"])
    bucket_pivot_offset = tuple(geometry["bucket_pivot_from_lift_pivot_xz"])
    bucket_tip_offset = tuple(geometry["bucket_tip_from_bucket_pivot_xz"])

    bucket_pivot = add(lift_pivot, rotate(bucket_pivot_offset, lift_angle))
    bucket_pivot_dlift = rotate_derivative(bucket_pivot_offset, lift_angle)

    bucket_pitch = lift_angle + tilt_angle
    bucket_tip = add(bucket_pivot, rotate(bucket_tip_offset, bucket_pitch))
    tip_rotation_rate = rotate_derivative(bucket_tip_offset, bucket_pitch)
    bucket_tip_dlift = add(bucket_pivot_dlift, tip_rotation_rate)
    bucket_tip_dtilt = tip_rotation_rate

    lift_cylinder = geometry["lift_cylinder"]
    lift_base = tuple(lift_cylinder["base_front_frame_xz"])
    lift_rod = add(lift_pivot, rotate(tuple(lift_cylinder["rod_mount_lift_arm_xz"]), lift_angle))
    lift_rod_dlift = rotate_derivative(tuple(lift_cylinder["rod_mount_lift_arm_xz"]), lift_angle)
    lift_length, lift_length_dlift = length_and_derivative(subtract(lift_rod, lift_base), lift_rod_dlift)

    tilt_cylinder = geometry["tilt_cylinder"]
    tilt_base_offset = tuple(tilt_cylinder["base_lift_arm_xz"])
    tilt_rod_offset = tuple(tilt_cylinder["rod_mount_bucket_xz"])
    tilt_base = add(lift_pivot, rotate(tilt_base_offset, lift_angle))
    tilt_base_dlift = rotate_derivative(tilt_base_offset, lift_angle)
    tilt_rod = add(bucket_pivot, rotate(tilt_rod_offset, bucket_pitch))
    tilt_rod_dlift = add(bucket_pivot_dlift, rotate_derivative(tilt_rod_offset, bucket_pitch))
    tilt_rod_dtilt = rotate_derivative(tilt_rod_offset, bucket_pitch)
    tilt_vector = subtract(tilt_rod, tilt_base)
    tilt_length, tilt_length_dlift = length_and_derivative(
        tilt_vector, subtract(tilt_rod_dlift, tilt_base_dlift)
    )
    _, tilt_length_dtilt = length_and_derivative(tilt_vector, tilt_rod_dtilt)

    return {
        "lift_angle_rad": lift_angle,
        "tilt_angle_rad": tilt_angle,
        "bucket_pitch_rad": bucket_pitch,
        "bucket_pivot_x_m": bucket_pivot[0],
        "bucket_pivot_z_m": bucket_pivot[1],
        "bucket_tip_x_m": bucket_tip[0],
        "bucket_tip_z_m": bucket_tip[1],
        "lift_cylinder_length_m": lift_length,
        "tilt_cylinder_length_m": tilt_length,
        "dlift_length_dlift_m_per_rad": lift_length_dlift,
        "dtilt_length_dlift_m_per_rad": tilt_length_dlift,
        "dtilt_length_dtilt_m_per_rad": tilt_length_dtilt,
        "dbucket_tip_x_dlift_m_per_rad": bucket_tip_dlift[0],
        "dbucket_tip_z_dlift_m_per_rad": bucket_tip_dlift[1],
        "dbucket_tip_x_dtilt_m_per_rad": bucket_tip_dtilt[0],
        "dbucket_tip_z_dtilt_m_per_rad": bucket_tip_dtilt[1],
    }


def samples(lower: float, upper: float, step: float) -> list[float]:
    count = round((upper - lower) / step)
    if not math.isclose(lower + count * step, upper, abs_tol=1.0e-10):
        raise ValueError("Joint range must be divisible by the requested table step")
    return [lower + index * step for index in range(count + 1)]


def finite_difference(config: dict, lift: float, tilt: float, coordinate: str, key: str, step: float) -> float:
    if coordinate == "lift":
        plus = linkage_state(config, lift + step, tilt)[key]
        minus = linkage_state(config, lift - step, tilt)[key]
    else:
        plus = linkage_state(config, lift, tilt + step)[key]
        minus = linkage_state(config, lift, tilt - step)[key]
    return (plus - minus) / (2.0 * step)


def validate_xacro(config: dict, xacro_path: Path) -> None:
    root = ET.parse(xacro_path).getroot()
    joints = {joint.attrib.get("name"): joint for joint in root.findall("joint")}
    expected = {
        "lift_joint": config["joint_limits_rad"]["lift"],
        "bucket_tilt_joint": config["joint_limits_rad"]["tilt"],
    }
    expected_axis = " ".join(str(int(value)) for value in config["coordinate_convention"]["joint_axis_xyz"])
    for name, limits in expected.items():
        joint = joints.get(name)
        if joint is None:
            raise AssertionError(f"Missing joint in Xacro: {name}")
        axis = joint.find("axis")
        limit = joint.find("limit")
        if axis is None or axis.attrib.get("xyz") != expected_axis:
            raise AssertionError(f"Unexpected axis for {name}")
        if limit is None:
            raise AssertionError(f"Missing limits for {name}")
        if not math.isclose(float(limit.attrib["lower"]), float(limits[0]), abs_tol=1.0e-9):
            raise AssertionError(f"Lower limit mismatch for {name}")
        if not math.isclose(float(limit.attrib["upper"]), float(limits[1]), abs_tol=1.0e-9):
            raise AssertionError(f"Upper limit mismatch for {name}")


def validate(config: dict, xacro_path: Path | None) -> tuple[float, float]:
    checks = config["self_consistency_checks"]
    difference_step = float(checks["finite_difference_step_rad"])
    tolerance = float(checks["jacobian_tolerance"])
    limits = config["joint_limits_rad"]
    lift_points = samples(float(limits["lift"][0]), float(limits["lift"][1]), 0.325)
    tilt_points = samples(float(limits["tilt"][0]), float(limits["tilt"][1]), 0.5)

    derivative_pairs = (
        ("lift", "lift_cylinder_length_m", "dlift_length_dlift_m_per_rad"),
        ("lift", "tilt_cylinder_length_m", "dtilt_length_dlift_m_per_rad"),
        ("tilt", "tilt_cylinder_length_m", "dtilt_length_dtilt_m_per_rad"),
        ("lift", "bucket_tip_x_m", "dbucket_tip_x_dlift_m_per_rad"),
        ("lift", "bucket_tip_z_m", "dbucket_tip_z_dlift_m_per_rad"),
        ("tilt", "bucket_tip_x_m", "dbucket_tip_x_dtilt_m_per_rad"),
        ("tilt", "bucket_tip_z_m", "dbucket_tip_z_dtilt_m_per_rad"),
    )
    maximum_error = 0.0
    minimum_moment_arm = math.inf
    for lift in lift_points:
        for tilt in tilt_points:
            state = linkage_state(config, lift, tilt)
            minimum_moment_arm = min(
                minimum_moment_arm,
                abs(state["dlift_length_dlift_m_per_rad"]),
                abs(state["dtilt_length_dtilt_m_per_rad"]),
            )
            for coordinate, value_key, derivative_key in derivative_pairs:
                numeric = finite_difference(config, lift, tilt, coordinate, value_key, difference_step)
                maximum_error = max(maximum_error, abs(numeric - state[derivative_key]))

    if maximum_error > tolerance:
        raise AssertionError(f"Analytic Jacobian error {maximum_error:.3e} exceeds {tolerance:.3e}")
    required_moment_arm = float(checks["minimum_absolute_moment_arm_m"])
    if minimum_moment_arm < required_moment_arm:
        raise AssertionError(
            f"Nominal linkage approaches a singularity: {minimum_moment_arm:.6f} m/rad"
        )

    zero = linkage_state(config, 0.0, 0.0)
    reference = checks["zero_pose_reference"]
    expected_values = {
        "bucket_pivot_x_m": reference["bucket_joint_xz_m"][0],
        "bucket_pivot_z_m": reference["bucket_joint_xz_m"][1],
        "bucket_tip_x_m": reference["bucket_tip_xz_m"][0],
        "bucket_tip_z_m": reference["bucket_tip_xz_m"][1],
        "lift_cylinder_length_m": reference["lift_cylinder_length_m"],
        "tilt_cylinder_length_m": reference["tilt_cylinder_length_m"],
    }
    for key, expected_value in expected_values.items():
        if not math.isclose(zero[key], float(expected_value), abs_tol=1.0e-9):
            raise AssertionError(f"Zero-pose reference mismatch for {key}")

    if xacro_path is not None:
        validate_xacro(config, xacro_path)
    return maximum_error, minimum_moment_arm


def write_table(config: dict, output_path: Path) -> int:
    limits = config["joint_limits_rad"]
    sampling = config["table_sampling_rad"]
    lift_values = samples(float(limits["lift"][0]), float(limits["lift"][1]), float(sampling["lift_step"]))
    tilt_values = samples(float(limits["tilt"][0]), float(limits["tilt"][1]), float(sampling["tilt_step"]))
    rows = [linkage_state(config, lift, tilt) for lift in lift_values for tilt in tilt_values]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return len(rows)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--xacro", type=Path)
    parser.add_argument("--check", action="store_true")
    arguments = parser.parse_args()

    with arguments.config.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    maximum_error = minimum_moment_arm = None
    if arguments.check:
        maximum_error, minimum_moment_arm = validate(config, arguments.xacro)
    row_count = write_table(config, arguments.output)
    print(f"PASS  linkage table rows={row_count} output={arguments.output}")
    if maximum_error is not None and minimum_moment_arm is not None:
        print(f"      max_jacobian_error={maximum_error:.3e}")
        print(f"      min_abs_moment_arm={minimum_moment_arm:.6f} m/rad")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

