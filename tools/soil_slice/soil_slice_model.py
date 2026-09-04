"""Deterministic 2D terrain-transfer and nominal cutting-force prototype."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


@dataclass(frozen=True)
class Material:
    bulk_density_kg_m3: float
    internal_friction_angle_rad: float
    cohesion_pa: float
    soil_tool_friction_angle_rad: float
    gravity_m_s2: float
    dynamic_coefficient: float
    reference_speed_m_s: float
    rake_angle_rad: float


@dataclass(frozen=True)
class Interaction:
    time_s: float
    edge_x_m: float
    edge_z_m: float
    penetration_m: float
    swept_volume_m3: float
    bucket_force_x_n: float
    bucket_force_z_n: float
    soil_reaction_x_n: float
    soil_reaction_z_n: float
    payload_volume_m3: float
    mass_balance_error_m3: float


class SoilSlice:
    """A height-column slice with conservative bucket/terrain material transfer.

    Force magnitude uses a transparent Rankine passive-wedge approximation.  It
    is deliberately isolated from the conservative geometry update so that a
    later Chrono/experiment-fitted force law cannot break volume accounting.
    """

    def __init__(
        self,
        *,
        domain_min_m: float,
        domain_max_m: float,
        cell_size_m: float,
        width_m: float,
        subgrid_step_m: float,
        bucket_capacity_m3: float,
        angle_of_repose_rad: float,
        material: Material,
    ) -> None:
        if not domain_max_m > domain_min_m:
            raise ValueError("terrain domain must have positive length")
        if cell_size_m <= 0.0 or subgrid_step_m <= 0.0 or width_m <= 0.0:
            raise ValueError("grid, subgrid, and width values must be positive")
        if subgrid_step_m > 0.01 + 1.0e-12:
            raise ValueError("swept-volume subgrid step must not exceed 0.01 m")
        self.domain_min_m = domain_min_m
        self.cell_size_m = cell_size_m
        self.width_m = width_m
        self.subgrid_step_m = subgrid_step_m
        self.bucket_capacity_m3 = bucket_capacity_m3
        self.angle_of_repose_rad = angle_of_repose_rad
        self.material = material
        cell_count = math.ceil((domain_max_m - domain_min_m) / cell_size_m)
        self.heights_m = [0.0] * cell_count
        self.payload_volume_m3 = 0.0
        self.excavated_volume_m3 = 0.0
        self.unloaded_volume_m3 = 0.0
        self._initial_total_volume_m3 = 0.0
        self._pass_start_heights = list(self.heights_m)
        self._pass_coverage = [0.0] * cell_count
        self._pass_target_z = [math.inf] * cell_count
        self.max_observed_substep_m = 0.0

    @property
    def domain_max_m(self) -> float:
        return self.domain_min_m + len(self.heights_m) * self.cell_size_m

    @property
    def terrain_volume_m3(self) -> float:
        return sum(self.heights_m) * self.cell_size_m * self.width_m

    @property
    def total_material_volume_m3(self) -> float:
        return self.terrain_volume_m3 + self.payload_volume_m3

    @property
    def mass_balance_error_m3(self) -> float:
        return self.total_material_volume_m3 - self._initial_total_volume_m3

    def cell_center_m(self, index: int) -> float:
        return self.domain_min_m + (index + 0.5) * self.cell_size_m

    def initialize_triangular_pile(
        self, *, center_m: float, height_m: float, slope_angle_rad: float
    ) -> None:
        slope = math.tan(slope_angle_rad)
        for index in range(len(self.heights_m)):
            distance = abs(self.cell_center_m(index) - center_m)
            self.heights_m[index] = max(0.0, height_m - slope * distance)
        self._initial_total_volume_m3 = self.total_material_volume_m3
        self.begin_cutting_pass()

    def begin_cutting_pass(self) -> None:
        self._pass_start_heights = list(self.heights_m)
        self._pass_coverage = [0.0] * len(self.heights_m)
        self._pass_target_z = [math.inf] * len(self.heights_m)

    def height_at(self, x_m: float) -> float:
        index = self._cell_index(x_m)
        return self.heights_m[index] if index is not None else 0.0

    def cutting_force(self, penetration_m: float, velocity_x_m_s: float) -> tuple[float, float]:
        if penetration_m <= 0.0 or abs(velocity_x_m_s) <= 1.0e-9:
            return 0.0, 0.0
        material = self.material
        sin_phi = math.sin(material.internal_friction_angle_rad)
        passive_coefficient = (1.0 + sin_phi) / (1.0 - sin_phi)
        unit_weight = material.bulk_density_kg_m3 * material.gravity_m_s2
        force_per_width = (
            0.5 * unit_weight * penetration_m**2 * passive_coefficient
            + 2.0
            * material.cohesion_pa
            * penetration_m
            * math.sqrt(passive_coefficient)
        )
        speed_ratio = abs(velocity_x_m_s) / max(material.reference_speed_m_s, 1.0e-9)
        dynamic_factor = 1.0 + material.dynamic_coefficient * speed_ratio**2
        magnitude = force_per_width * self.width_m * dynamic_factor
        uplift_angle = max(
            math.radians(-30.0),
            min(
                math.radians(30.0),
                material.rake_angle_rad - material.soil_tool_friction_angle_rad,
            ),
        )
        direction = -1.0 if velocity_x_m_s > 0.0 else 1.0
        return direction * magnitude * math.cos(uplift_angle), magnitude * math.sin(uplift_angle)

    def excavate_segment(
        self,
        *,
        time_s: float,
        start_x_m: float,
        start_z_m: float,
        end_x_m: float,
        end_z_m: float,
        duration_s: float,
    ) -> list[Interaction]:
        distance = math.hypot(end_x_m - start_x_m, end_z_m - start_z_m)
        step_count = max(1, math.ceil(distance / self.subgrid_step_m))
        velocity_x = (end_x_m - start_x_m) / max(duration_s, 1.0e-9)
        interactions: list[Interaction] = []
        for step in range(step_count):
            ratio0 = step / step_count
            ratio1 = (step + 1) / step_count
            x0 = start_x_m + (end_x_m - start_x_m) * ratio0
            z0 = start_z_m + (end_z_m - start_z_m) * ratio0
            x1 = start_x_m + (end_x_m - start_x_m) * ratio1
            z1 = start_z_m + (end_z_m - start_z_m) * ratio1
            xm = 0.5 * (x0 + x1)
            zm = 0.5 * (z0 + z1)
            substep = math.hypot(x1 - x0, z1 - z0)
            self.max_observed_substep_m = max(self.max_observed_substep_m, substep)
            surface_before = self.height_at(xm)
            penetration = max(0.0, surface_before - zm)
            force_x, force_z = self.cutting_force(penetration, velocity_x)
            removed_volume = self._remove_swept_material(x0, z0, x1, z1)
            interactions.append(
                Interaction(
                    time_s=time_s + duration_s * ratio1,
                    edge_x_m=x1,
                    edge_z_m=z1,
                    penetration_m=penetration,
                    swept_volume_m3=removed_volume,
                    bucket_force_x_n=force_x,
                    bucket_force_z_n=force_z,
                    soil_reaction_x_n=-force_x,
                    soil_reaction_z_n=-force_z,
                    payload_volume_m3=self.payload_volume_m3,
                    mass_balance_error_m3=self.mass_balance_error_m3,
                )
            )
        return interactions

    def unload_all(self, *, center_m: float) -> float:
        volume = self.payload_volume_m3
        if volume <= 0.0:
            return 0.0
        target_area = volume / self.width_m
        slope = math.tan(self.angle_of_repose_rad)
        original = list(self.heights_m)

        def added_area(apex_height: float) -> float:
            return sum(
                max(0.0, apex_height - slope * abs(self.cell_center_m(index) - center_m) - height)
                for index, height in enumerate(original)
            ) * self.cell_size_m

        lower = 0.0
        upper = max(original) + 1.0
        while added_area(upper) < target_area:
            upper *= 2.0
        for _ in range(80):
            middle = 0.5 * (lower + upper)
            if added_area(middle) < target_area:
                lower = middle
            else:
                upper = middle
        apex = 0.5 * (lower + upper)
        for index, height in enumerate(original):
            repose_surface = apex - slope * abs(self.cell_center_m(index) - center_m)
            self.heights_m[index] = max(height, repose_surface)

        # The bisection residual is near machine precision.  Move the exact
        # payload volume into terrain accounting and retain the geometric result.
        self.payload_volume_m3 = 0.0
        self.unloaded_volume_m3 += volume
        return volume

    def maximum_slope(self, *, minimum_x_m: float = -math.inf) -> float:
        maximum = 0.0
        for index in range(len(self.heights_m) - 1):
            if self.cell_center_m(index) < minimum_x_m:
                continue
            maximum = max(
                maximum,
                abs(self.heights_m[index + 1] - self.heights_m[index]) / self.cell_size_m,
            )
        return maximum

    def _cell_index(self, x_m: float) -> int | None:
        index = math.floor((x_m - self.domain_min_m) / self.cell_size_m)
        return index if 0 <= index < len(self.heights_m) else None

    def _remove_swept_material(self, x0: float, z0: float, x1: float, z1: float) -> float:
        if abs(x1 - x0) <= 1.0e-12:
            return 0.0
        segment_min = min(x0, x1)
        segment_max = max(x0, x1)
        first = max(0, math.floor((segment_min - self.domain_min_m) / self.cell_size_m))
        last = min(
            len(self.heights_m) - 1,
            math.floor((segment_max - self.domain_min_m) / self.cell_size_m),
        )
        removed = 0.0
        for index in range(first, last + 1):
            cell_min = self.domain_min_m + index * self.cell_size_m
            cell_max = cell_min + self.cell_size_m
            overlap_min = max(segment_min, cell_min)
            overlap_max = min(segment_max, cell_max)
            overlap = max(0.0, overlap_max - overlap_min)
            if overlap <= 0.0:
                continue
            midpoint = 0.5 * (overlap_min + overlap_max)
            segment_ratio = (midpoint - x0) / (x1 - x0)
            target_z = z0 + (z1 - z0) * segment_ratio
            self._pass_target_z[index] = min(self._pass_target_z[index], target_z)
            previous_coverage = self._pass_coverage[index]
            new_coverage = min(1.0, previous_coverage + overlap / self.cell_size_m)
            self._pass_coverage[index] = new_coverage
            start_height = self._pass_start_heights[index]
            desired_height = max(
                self._pass_target_z[index],
                start_height - new_coverage * (start_height - self._pass_target_z[index]),
            )
            desired_height = max(0.0, min(self.heights_m[index], desired_height))
            available_capacity = self.bucket_capacity_m3 - self.payload_volume_m3
            proposed_volume = (
                (self.heights_m[index] - desired_height) * self.cell_size_m * self.width_m
            )
            actual_volume = min(max(0.0, proposed_volume), max(0.0, available_capacity))
            if actual_volume > 0.0:
                self.heights_m[index] -= actual_volume / (self.cell_size_m * self.width_m)
                self.payload_volume_m3 += actual_volume
                self.excavated_volume_m3 += actual_volume
                removed += actual_volume
        return removed


def all_finite(interactions: Iterable[Interaction]) -> bool:
    return all(
        all(math.isfinite(value) for value in interaction.__dict__.values())
        for interaction in interactions
    )
