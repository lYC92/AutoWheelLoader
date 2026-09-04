#!/usr/bin/env python3
"""Run and verify the nominal dry-sand 2D cutting/unloading prototype."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import sys

import yaml

from soil_slice_model import Material, SoilSlice, all_finite


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))

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
    terrain = config["terrain"]
    model = SoilSlice(
        domain_min_m=float(terrain["domain_min_m"]),
        domain_max_m=float(terrain["domain_max_m"]),
        cell_size_m=float(terrain["grid_resolution_m"]),
        width_m=float(terrain["slice_width_m"]),
        subgrid_step_m=float(terrain["subgrid_step_m"]),
        bucket_capacity_m3=float(config["bucket"]["capacity_m3"]),
        angle_of_repose_rad=math.radians(terrain["angle_of_repose_deg"]),
        material=material,
    )
    model.initialize_triangular_pile(
        center_m=float(terrain["initial_pile_center_m"]),
        height_m=float(terrain["initial_pile_height_m"]),
        slope_angle_rad=math.radians(terrain["angle_of_repose_deg"]),
    )
    initial_volume = model.terrain_volume_m3
    interactions = model.excavate_segment(
        time_s=0.0,
        start_x_m=1.0,
        start_z_m=0.35,
        end_x_m=7.0,
        end_z_m=0.35,
        duration_s=6.0,
    )

    require(all_finite(interactions), "interaction output contains non-finite values")
    require(model.max_observed_substep_m <= 0.01 + 1.0e-12, "subgrid step exceeded 1 cm")
    require(model.payload_volume_m3 > 0.1, "cutting pass captured too little material")
    require(model.payload_volume_m3 < model.bucket_capacity_m3 + 1.0e-12, "bucket overfilled")
    require(max(item.penetration_m for item in interactions) > 0.25, "no meaningful penetration")
    require(max(abs(item.bucket_force_x_n) for item in interactions) > 1000.0, "cutting force too small")
    require(
        all(item.bucket_force_x_n <= 1.0e-9 for item in interactions),
        "bucket force did not oppose forward cutting velocity",
    )
    action_reaction_residual = max(
        math.hypot(
            item.bucket_force_x_n + item.soil_reaction_x_n,
            item.bucket_force_z_n + item.soil_reaction_z_n,
        )
        for item in interactions
    )
    require(action_reaction_residual <= 1.0e-9, "action/reaction ledger does not close")
    excavation_balance = abs(model.mass_balance_error_m3)
    require(excavation_balance <= max(initial_volume * 1.0e-9, 1.0e-12), "excavation lost material")

    payload_before_unload = model.payload_volume_m3
    model.unload_all(center_m=11.0)
    final_balance = abs(model.mass_balance_error_m3)
    require(model.payload_volume_m3 <= 1.0e-12, "payload was not emptied")
    require(
        abs(model.unloaded_volume_m3 - payload_before_unload) <= 1.0e-9,
        "unloaded volume does not equal captured payload",
    )
    require(final_balance <= max(initial_volume * 1.0e-9, 1.0e-12), "unloading lost material")
    maximum_deposit_slope = model.maximum_slope(minimum_x_m=9.0)
    repose_slope = math.tan(model.angle_of_repose_rad)
    require(
        maximum_deposit_slope <= repose_slope + 1.0e-8,
        "unloaded pile exceeds configured angle of repose",
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(interactions[0].__dict__.keys()))
        writer.writeheader()
        for interaction in interactions:
            writer.writerow(interaction.__dict__)

    peak_force = max(
        math.hypot(item.bucket_force_x_n, item.bucket_force_z_n) for item in interactions
    )
    print(
        "PASS dry-sand 2D slice: "
        f"initial={initial_volume:.6f}m3 "
        f"excavated={model.excavated_volume_m3:.6f}m3 "
        f"unloaded={model.unloaded_volume_m3:.6f}m3 "
        f"balance_error={final_balance:.3e}m3 "
        f"peak_force={peak_force / 1000.0:.2f}kN "
        f"reaction_residual={action_reaction_residual:.3e}N "
        f"max_substep={model.max_observed_substep_m:.4f}m"
    )
    print(f"Trace: {args.output}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeError as error:
        print(f"FAIL dry-sand 2D slice: {error}", file=sys.stderr)
        raise SystemExit(1)
