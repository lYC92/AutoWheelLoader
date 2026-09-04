"""Deterministic conservative 2.5D dry-sand heightfield prototype."""

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
    edge_y_m: float
    edge_z_m: float
    maximum_penetration_m: float
    active_width_m: float
    swept_volume_m3: float
    bucket_force_x_n: float
    bucket_force_y_n: float
    bucket_force_z_n: float
    soil_reaction_x_n: float
    soil_reaction_y_n: float
    soil_reaction_z_n: float
    payload_volume_m3: float
    volume_balance_error_m3: float


class SoilHeightfield3D:
    """A 2D height grid with conservative bucket sweep and conical unloading."""

    def __init__(
        self,
        *,
        domain_min_x_m: float,
        domain_max_x_m: float,
        domain_min_y_m: float,
        domain_max_y_m: float,
        cell_size_m: float,
        subgrid_step_m: float,
        bucket_width_m: float,
        bucket_capacity_m3: float,
        angle_of_repose_rad: float,
        material: Material,
    ) -> None:
        if domain_max_x_m <= domain_min_x_m or domain_max_y_m <= domain_min_y_m:
            raise ValueError("heightfield domain must have positive area")
        if cell_size_m <= 0.0 or subgrid_step_m <= 0.0:
            raise ValueError("grid and subgrid steps must be positive")
        if subgrid_step_m > 0.01 + 1.0e-12:
            raise ValueError("bucket sweep substep must not exceed 0.01 m")
        if bucket_width_m <= 0.0 or bucket_capacity_m3 <= 0.0:
            raise ValueError("bucket dimensions must be positive")

        self.domain_min_x_m = domain_min_x_m
        self.domain_min_y_m = domain_min_y_m
        self.cell_size_m = cell_size_m
        self.subgrid_step_m = subgrid_step_m
        self.bucket_width_m = bucket_width_m
        self.bucket_capacity_m3 = bucket_capacity_m3
        self.angle_of_repose_rad = angle_of_repose_rad
        self.material = material
        self.nx = math.ceil((domain_max_x_m - domain_min_x_m) / cell_size_m)
        self.ny = math.ceil((domain_max_y_m - domain_min_y_m) / cell_size_m)
        self.heights_m = [0.0] * (self.nx * self.ny)
        self.payload_volume_m3 = 0.0
        self.excavated_volume_m3 = 0.0
        self.dumped_volume_m3 = 0.0
        self.initial_volume_m3 = 0.0
        self.maximum_substep_m = 0.0
        self._pass_start_heights = list(self.heights_m)
        self._pass_coverage = [0.0] * len(self.heights_m)
        self._pass_target_z = [math.inf] * len(self.heights_m)

    def index(self, ix: int, iy: int) -> int:
        return iy * self.nx + ix

    def center_x(self, ix: int) -> float:
        return self.domain_min_x_m + (ix + 0.5) * self.cell_size_m

    def center_y(self, iy: int) -> float:
        return self.domain_min_y_m + (iy + 0.5) * self.cell_size_m

    @property
    def terrain_volume_m3(self) -> float:
        return sum(self.heights_m) * self.cell_size_m**2

    @property
    def volume_balance_error_m3(self) -> float:
        return self.terrain_volume_m3 + self.payload_volume_m3 - self.initial_volume_m3

    def initialize_conical_pile(
        self, *, center_x_m: float, center_y_m: float, height_m: float
    ) -> None:
        slope = math.tan(self.angle_of_repose_rad)
        for iy in range(self.ny):
            for ix in range(self.nx):
                radius = math.hypot(
                    self.center_x(ix) - center_x_m,
                    self.center_y(iy) - center_y_m,
                )
                self.heights_m[self.index(ix, iy)] = max(0.0, height_m - slope * radius)
        self.initial_volume_m3 = self.terrain_volume_m3
        self.begin_cutting_pass()

    def begin_cutting_pass(self) -> None:
        self._pass_start_heights = list(self.heights_m)
        self._pass_coverage = [0.0] * len(self.heights_m)
        self._pass_target_z = [math.inf] * len(self.heights_m)

    def excavate_segment(
        self,
        *,
        start_xyz_m: tuple[float, float, float],
        end_xyz_m: tuple[float, float, float],
        duration_s: float,
    ) -> list[Interaction]:
        delta = tuple(end - start for start, end in zip(start_xyz_m, end_xyz_m))
        distance = math.sqrt(sum(value * value for value in delta))
        steps = max(1, math.ceil(distance / self.subgrid_step_m))
        velocity_x = delta[0] / max(duration_s, 1.0e-9)
        velocity_y = delta[1] / max(duration_s, 1.0e-9)
        interactions: list[Interaction] = []
        for step in range(steps):
            ratio0 = step / steps
            ratio1 = (step + 1) / steps
            start = tuple(start_xyz_m[i] + delta[i] * ratio0 for i in range(3))
            end = tuple(start_xyz_m[i] + delta[i] * ratio1 for i in range(3))
            substep = math.dist(start, end)
            self.maximum_substep_m = max(self.maximum_substep_m, substep)
            midpoint = tuple(0.5 * (start[i] + end[i]) for i in range(3))
            penetration, active_width = self._penetration(midpoint)
            force = self._cutting_force(
                penetration_m=penetration,
                active_width_m=active_width,
                velocity_x_m_s=velocity_x,
                velocity_y_m_s=velocity_y,
            )
            removed = self._remove_sweep(start, end)
            interactions.append(
                Interaction(
                    time_s=duration_s * ratio1,
                    edge_x_m=end[0],
                    edge_y_m=end[1],
                    edge_z_m=end[2],
                    maximum_penetration_m=penetration,
                    active_width_m=active_width,
                    swept_volume_m3=removed,
                    bucket_force_x_n=force[0],
                    bucket_force_y_n=force[1],
                    bucket_force_z_n=force[2],
                    soil_reaction_x_n=-force[0],
                    soil_reaction_y_n=-force[1],
                    soil_reaction_z_n=-force[2],
                    payload_volume_m3=self.payload_volume_m3,
                    volume_balance_error_m3=self.volume_balance_error_m3,
                )
            )
        return interactions

    def unload_all(self, *, center_x_m: float, center_y_m: float) -> float:
        volume = self.payload_volume_m3
        if volume <= 0.0:
            return 0.0
        original = list(self.heights_m)
        target_area_height = volume
        slope = math.tan(self.angle_of_repose_rad)

        def added_volume(apex_m: float) -> float:
            total = 0.0
            for iy in range(self.ny):
                for ix in range(self.nx):
                    radial = math.hypot(
                        self.center_x(ix) - center_x_m,
                        self.center_y(iy) - center_y_m,
                    )
                    surface = apex_m - slope * radial
                    total += max(0.0, surface - original[self.index(ix, iy)])
            return total * self.cell_size_m**2

        lower = 0.0
        upper = max(original) + 1.0
        while added_volume(upper) < target_area_height:
            upper *= 2.0
        for _ in range(80):
            middle = 0.5 * (lower + upper)
            if added_volume(middle) < target_area_height:
                lower = middle
            else:
                upper = middle
        apex = 0.5 * (lower + upper)
        for iy in range(self.ny):
            for ix in range(self.nx):
                radial = math.hypot(
                    self.center_x(ix) - center_x_m,
                    self.center_y(iy) - center_y_m,
                )
                offset = self.index(ix, iy)
                self.heights_m[offset] = max(original[offset], apex - slope * radial)
        self.payload_volume_m3 = 0.0
        self.dumped_volume_m3 += volume
        return volume

    def maximum_neighbor_slope(self, *, minimum_x_m: float = -math.inf) -> float:
        maximum = 0.0
        for iy in range(self.ny):
            for ix in range(self.nx):
                if self.center_x(ix) < minimum_x_m:
                    continue
                here = self.heights_m[self.index(ix, iy)]
                if ix + 1 < self.nx:
                    maximum = max(
                        maximum,
                        abs(self.heights_m[self.index(ix + 1, iy)] - here) / self.cell_size_m,
                    )
                if iy + 1 < self.ny:
                    maximum = max(
                        maximum,
                        abs(self.heights_m[self.index(ix, iy + 1)] - here) / self.cell_size_m,
                    )
        return maximum

    def _cell_range(self, minimum: float, maximum: float, origin: float, count: int) -> range:
        first = max(0, math.floor((minimum - origin) / self.cell_size_m))
        last = min(count - 1, math.floor((maximum - origin) / self.cell_size_m))
        return range(first, last + 1)

    def _penetration(self, midpoint: tuple[float, float, float]) -> tuple[float, float]:
        half_width = 0.5 * self.bucket_width_m
        maximum = 0.0
        active_width = 0.0
        for iy in self._cell_range(
            midpoint[1] - half_width,
            midpoint[1] + half_width,
            self.domain_min_y_m,
            self.ny,
        ):
            ix = math.floor((midpoint[0] - self.domain_min_x_m) / self.cell_size_m)
            if not 0 <= ix < self.nx:
                continue
            depth = max(0.0, self.heights_m[self.index(ix, iy)] - midpoint[2])
            maximum = max(maximum, depth)
            if depth > 0.0:
                active_width += self.cell_size_m
        return maximum, min(active_width, self.bucket_width_m)

    def _cutting_force(
        self,
        *,
        penetration_m: float,
        active_width_m: float,
        velocity_x_m_s: float,
        velocity_y_m_s: float,
    ) -> tuple[float, float, float]:
        speed = math.hypot(velocity_x_m_s, velocity_y_m_s)
        if penetration_m <= 0.0 or active_width_m <= 0.0 or speed <= 1.0e-9:
            return (0.0, 0.0, 0.0)
        sin_phi = math.sin(self.material.internal_friction_angle_rad)
        passive = (1.0 + sin_phi) / (1.0 - sin_phi)
        unit_weight = self.material.bulk_density_kg_m3 * self.material.gravity_m_s2
        force_per_width = (
            0.5 * unit_weight * penetration_m**2 * passive
            + 2.0 * self.material.cohesion_pa * penetration_m * math.sqrt(passive)
        )
        speed_ratio = speed / max(self.material.reference_speed_m_s, 1.0e-9)
        magnitude = (
            force_per_width
            * active_width_m
            * (1.0 + self.material.dynamic_coefficient * speed_ratio**2)
        )
        uplift = max(
            math.radians(-30.0),
            min(
                math.radians(30.0),
                self.material.rake_angle_rad - self.material.soil_tool_friction_angle_rad,
            ),
        )
        horizontal = magnitude * math.cos(uplift)
        return (
            -horizontal * velocity_x_m_s / speed,
            -horizontal * velocity_y_m_s / speed,
            magnitude * math.sin(uplift),
        )

    def _remove_sweep(
        self,
        start: tuple[float, float, float],
        end: tuple[float, float, float],
    ) -> float:
        minimum_x, maximum_x = sorted((start[0], end[0]))
        midpoint_y = 0.5 * (start[1] + end[1])
        half_width = 0.5 * self.bucket_width_m
        removed = 0.0
        for ix in self._cell_range(minimum_x, maximum_x, self.domain_min_x_m, self.nx):
            cell_min_x = self.domain_min_x_m + ix * self.cell_size_m
            cell_max_x = cell_min_x + self.cell_size_m
            overlap_x = max(0.0, min(maximum_x, cell_max_x) - max(minimum_x, cell_min_x))
            if overlap_x <= 0.0:
                continue
            segment_x = 0.5 * (max(minimum_x, cell_min_x) + min(maximum_x, cell_max_x))
            ratio = (segment_x - start[0]) / max(end[0] - start[0], 1.0e-12)
            target_z = start[2] + (end[2] - start[2]) * ratio
            for iy in self._cell_range(
                midpoint_y - half_width,
                midpoint_y + half_width,
                self.domain_min_y_m,
                self.ny,
            ):
                cell_min_y = self.domain_min_y_m + iy * self.cell_size_m
                cell_max_y = cell_min_y + self.cell_size_m
                overlap_y = max(
                    0.0,
                    min(midpoint_y + half_width, cell_max_y)
                    - max(midpoint_y - half_width, cell_min_y),
                )
                if overlap_y <= 0.0:
                    continue
                offset = self.index(ix, iy)
                coverage_increment = overlap_x * overlap_y / self.cell_size_m**2
                self._pass_target_z[offset] = min(self._pass_target_z[offset], target_z)
                old_coverage = self._pass_coverage[offset]
                new_coverage = min(1.0, old_coverage + coverage_increment)
                self._pass_coverage[offset] = new_coverage
                start_height = self._pass_start_heights[offset]
                desired = max(
                    self._pass_target_z[offset],
                    start_height - new_coverage * (start_height - self._pass_target_z[offset]),
                )
                desired = max(0.0, min(self.heights_m[offset], desired))
                proposed = (self.heights_m[offset] - desired) * self.cell_size_m**2
                available = self.bucket_capacity_m3 - self.payload_volume_m3
                actual = min(max(0.0, proposed), max(0.0, available))
                if actual > 0.0:
                    self.heights_m[offset] -= actual / self.cell_size_m**2
                    self.payload_volume_m3 += actual
                    self.excavated_volume_m3 += actual
                    removed += actual
        return removed


def all_finite(interactions: Iterable[Interaction]) -> bool:
    return all(
        all(math.isfinite(value) for value in interaction.__dict__.values())
        for interaction in interactions
    )
