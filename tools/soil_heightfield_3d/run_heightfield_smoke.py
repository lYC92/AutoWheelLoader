#!/usr/bin/env python3
"""Exercise conservation, 3D sweep, force symmetry, and unloading."""

from __future__ import annotations

import argparse
import csv
import hashlib
import math
from pathlib import Path
import struct
import sys

import yaml

from heightfield_model import Material, SoilHeightfield3D, all_finite


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def write_obj(path: Path, model: SoilHeightfield3D) -> None:
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write("# Nominal dry-sand 2.5D heightfield surface\n")
        for iy in range(model.ny):
            for ix in range(model.nx):
                stream.write(
                    f"v {model.center_x(ix):.6f} {model.center_y(iy):.6f} "
                    f"{model.heights_m[model.index(ix, iy)]:.6f}\n"
                )
        for iy in range(model.ny - 1):
            for ix in range(model.nx - 1):
                a = model.index(ix, iy) + 1
                b = model.index(ix + 1, iy) + 1
                c = model.index(ix + 1, iy + 1) + 1
                d = model.index(ix, iy + 1) + 1
                stream.write(f"f {a} {b} {c}\n")
                stream.write(f"f {a} {c} {d}\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    terrain = config["terrain"]
    bucket = config["bucket"]
    trajectory = config["smoke_trajectory"]
    material = Material(
        bulk_density_kg_m3=float(config["bulk_density_kg_m3"]),
        internal_friction_angle_rad=math.radians(config["internal_friction_angle_deg"]),
        cohesion_pa=float(config["cohesion_pa"]),
        soil_tool_friction_angle_rad=math.radians(config["soil_tool_friction_angle_deg"]),
        gravity_m_s2=float(config["gravity_m_s2"]),
        dynamic_coefficient=float(config["cutting_model"]["dynamic_coefficient"]),
        reference_speed_m_s=float(config["cutting_model"]["reference_speed_m_s"]),
        rake_angle_rad=math.radians(config["cutting_model"]["rake_angle_deg"]),
    )
    model = SoilHeightfield3D(
        domain_min_x_m=float(terrain["domain_min_x_m"]),
        domain_max_x_m=float(terrain["domain_max_x_m"]),
        domain_min_y_m=float(terrain["domain_min_y_m"]),
        domain_max_y_m=float(terrain["domain_max_y_m"]),
        cell_size_m=float(terrain["grid_resolution_m"]),
        subgrid_step_m=float(terrain["subgrid_step_m"]),
        bucket_width_m=float(bucket["width_m"]),
        bucket_capacity_m3=float(bucket["capacity_m3"]),
        angle_of_repose_rad=math.radians(terrain["angle_of_repose_deg"]),
        material=material,
    )
    model.initialize_conical_pile(
        center_x_m=float(terrain["initial_pile_center_x_m"]),
        center_y_m=float(terrain["initial_pile_center_y_m"]),
        height_m=float(terrain["initial_pile_height_m"]),
    )
    initial_volume = model.initial_volume_m3
    initial_heights = list(model.heights_m)
    interactions = model.excavate_segment(
        start_xyz_m=(
            float(trajectory["start_x_m"]),
            float(trajectory["start_y_m"]),
            float(trajectory["start_z_m"]),
        ),
        end_xyz_m=(
            float(trajectory["end_x_m"]),
            float(trajectory["end_y_m"]),
            float(trajectory["end_z_m"]),
        ),
        duration_s=float(trajectory["duration_s"]),
    )
    require(all_finite(interactions), "3D interaction contains non-finite values")
    require(model.maximum_substep_m <= 0.01 + 1.0e-12, "3D sweep substep exceeded 1 cm")
    require(model.payload_volume_m3 > 0.5, "3D sweep captured too little material")
    require(model.payload_volume_m3 <= model.bucket_capacity_m3 + 1.0e-12, "bucket overflowed")
    require(min(model.heights_m) >= -1.0e-12, "heightfield contains a negative cell")
    require(abs(model.volume_balance_error_m3) <= initial_volume * 1.0e-10, "excavation lost volume")
    changed_y_rows = set()
    for iy in range(model.ny):
        if any(
            abs(model.heights_m[model.index(ix, iy)] - initial_heights[model.index(ix, iy)])
            > 1.0e-6
            for ix in range(model.nx)
        ):
            changed_y_rows.add(iy)
    require(len(changed_y_rows) >= 20, "bucket sweep did not create transverse 3D deformation")
    reaction_residual = max(
        math.sqrt(
            (item.bucket_force_x_n + item.soil_reaction_x_n) ** 2
            + (item.bucket_force_y_n + item.soil_reaction_y_n) ** 2
            + (item.bucket_force_z_n + item.soil_reaction_z_n) ** 2
        )
        for item in interactions
    )
    require(reaction_residual <= 1.0e-9, "3D action/reaction ledger does not close")
    peak_force = max(
        math.sqrt(
            item.bucket_force_x_n**2
            + item.bucket_force_y_n**2
            + item.bucket_force_z_n**2
        )
        for item in interactions
    )
    require(peak_force > 1000.0, "3D cutting force was not exercised")

    captured = model.payload_volume_m3
    after_cut = list(model.heights_m)
    unloaded = model.unload_all(
        center_x_m=float(trajectory["dump_x_m"]),
        center_y_m=float(trajectory["dump_y_m"]),
    )
    require(abs(unloaded - captured) <= 1.0e-9, "3D unloading did not empty the payload")
    require(model.payload_volume_m3 <= 1.0e-12, "3D bucket retained material")
    require(abs(model.volume_balance_error_m3) <= initial_volume * 1.0e-10, "unloading lost volume")
    repose_slope = math.tan(model.angle_of_repose_rad)
    dump_slope = model.maximum_neighbor_slope(minimum_x_m=10.5)
    require(dump_slope <= repose_slope + 1.0e-8, "3D dump pile exceeds angle of repose")

    args.output.mkdir(parents=True, exist_ok=True)
    trace_path = args.output / "interaction_trace.csv"
    with trace_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(interactions[0].__dict__.keys()))
        writer.writeheader()
        for item in interactions:
            writer.writerow(item.__dict__)
    cross_section_path = args.output / "cross_sections.csv"
    with cross_section_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["x_m", "y_m", "initial_m", "after_cut_m", "after_dump_m"])
        for target_y in (0.0, float(trajectory["dump_y_m"])):
            iy = min(range(model.ny), key=lambda row: abs(model.center_y(row) - target_y))
            for ix in range(model.nx):
                offset = model.index(ix, iy)
                writer.writerow(
                    [
                        model.center_x(ix),
                        model.center_y(iy),
                        initial_heights[offset],
                        after_cut[offset],
                        model.heights_m[offset],
                    ]
                )
    mesh_path = args.output / "final_heightfield.obj"
    write_obj(mesh_path, model)
    digest = hashlib.sha256(
        b"".join(struct.pack("<d", height) for height in model.heights_m)
    ).hexdigest()
    summary_path = args.output / "heightfield_3d_smoke.txt"
    summary = (
        "PASS dry-sand 3D heightfield: "
        f"grid={model.nx}x{model.ny} cells={len(model.heights_m)} "
        f"initial={initial_volume:.6f}m3 captured={captured:.6f}m3 "
        f"unloaded={unloaded:.6f}m3 balance={model.volume_balance_error_m3:.3e}m3 "
        f"peak_force={peak_force / 1000.0:.2f}kN changed_y_rows={len(changed_y_rows)} "
        f"dump_slope_deg={math.degrees(math.atan(dump_slope)):.3f} "
        f"max_substep={model.maximum_substep_m:.4f}m sha256={digest}\n"
    )
    summary_path.write_text(summary, encoding="utf-8")
    print(summary, end="")
    print(f"Trace: {trace_path}")
    print(f"Cross-sections: {cross_section_path}")
    print(f"Continuous mesh: {mesh_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeError as error:
        print(f"FAIL dry-sand 3D heightfield: {error}", file=sys.stderr)
        raise SystemExit(1)
