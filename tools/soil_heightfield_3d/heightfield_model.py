"""Deterministic conservative 2.5D dry-sand heightfield prototype."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


# Convex fragments retain subcell cut depth: crossing or retracing a sweep
# cannot remove the same material twice. Coordinates are world XY, metres.
def polygon_area(poly):
    if len(poly) < 3:
        return 0.0
    x0, y0 = poly[0]
    return abs(sum((a[0]-x0)*(b[1]-y0)-(b[0]-x0)*(a[1]-y0)
                   for a, b in zip(poly, poly[1:]+poly[:1]))) * 0.5


def split_polygon(poly, a, b):
    inside, outside = [], []
    def side(p):
        return (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])
    for p, q in zip(poly, poly[1:]+poly[:1]):
        dp, dq = side(p), side(q)
        if dp >= 0:
            inside.append(p)
        if dp <= 0:
            outside.append(p)
        if (dp > 0 and dq < 0) or (dp < 0 and dq > 0):
            t = dp / (dp-dq)
            cross = (p[0]+t*(q[0]-p[0]), p[1]+t*(q[1]-p[1]))
            inside.append(cross)
            outside.append(cross)
    return inside, outside


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
    bucket_torque_z_nm: float
    soil_reaction_torque_z_nm: float
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
        self._fragments = {}

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
        self._fragments.clear()
        self.initial_volume_m3 = self.terrain_volume_m3
        self.begin_cutting_pass()

    def begin_cutting_pass(self) -> None:
        # Preserve subcell geometry across passes. Only terrain replacement
        # (initialization or deposition) invalidates these fragments.
        pass

    def excavate_segment(
        self,
        *,
        start_xyz_m: tuple[float, float, float],
        end_xyz_m: tuple[float, float, float],
        duration_s: float,
        start_yaw_rad: float = 0.0,
        end_yaw_rad: float | None = None,
    ) -> list[Interaction]:
        if end_yaw_rad is None:
            end_yaw_rad = start_yaw_rad
        if duration_s <= 0 or not all(math.isfinite(v) for v in
                (*start_xyz_m, *end_xyz_m, duration_s, start_yaw_rad, end_yaw_rad)):
            raise ValueError("segment coordinates and yaw must be finite; duration must be positive")
        yaw_delta = math.remainder(end_yaw_rad-start_yaw_rad, 2*math.pi)
        delta = tuple(end - start for start, end in zip(start_xyz_m, end_xyz_m))
        distance = math.sqrt(sum(value * value for value in delta))
        steps = max(1, math.ceil((distance + abs(yaw_delta)*self.bucket_width_m/2) / self.subgrid_step_m))
        velocity_x = delta[0] / max(duration_s, 1.0e-9)
        velocity_y = delta[1] / max(duration_s, 1.0e-9)
        interactions: list[Interaction] = []
        for step in range(steps):
            ratio0 = step / steps
            ratio1 = (step + 1) / steps
            start = tuple(start_xyz_m[i] + delta[i] * ratio0 for i in range(3))
            end = tuple(start_xyz_m[i] + delta[i] * ratio1 for i in range(3))
            yaw0 = start_yaw_rad + yaw_delta*ratio0
            yaw1 = start_yaw_rad + yaw_delta*ratio1
            substep = math.dist(start, end) + abs(yaw1-yaw0)*self.bucket_width_m/2
            self.maximum_substep_m = max(self.maximum_substep_m, substep)
            midpoint = tuple(0.5 * (start[i] + end[i]) for i in range(3))
            penetration, active_width = self._penetration(midpoint, (yaw0+yaw1)/2)
            force, torque_z = self._blade_wrench(midpoint, (yaw0+yaw1)/2,
                velocity_x, velocity_y, yaw_delta/duration_s)
            removed = self._remove_sweep(start, end, yaw0, yaw1)
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
                    bucket_torque_z_nm=torque_z,
                    soil_reaction_torque_z_nm=-torque_z,
                    soil_reaction_x_n=-force[0],
                    soil_reaction_y_n=-force[1],
                    soil_reaction_z_n=-force[2],
                    payload_volume_m3=self.payload_volume_m3,
                    volume_balance_error_m3=self.volume_balance_error_m3,
                )
            )
        return interactions

    def unload_all(self, *, center_x_m: float, center_y_m: float) -> float:
        if not (math.isfinite(center_x_m) and math.isfinite(center_y_m)
                and self.domain_min_x_m <= center_x_m < self.domain_min_x_m+self.nx*self.cell_size_m
                and self.domain_min_y_m <= center_y_m < self.domain_min_y_m+self.ny*self.cell_size_m):
            raise ValueError("unloading centre must be inside the heightfield")
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
        self._fragments.clear()
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

    def _penetration(self, midpoint, yaw=0.0):
        count = max(1, math.ceil(self.bucket_width_m / self.subgrid_step_m))
        width = self.bucket_width_m / count
        maximum, active = 0.0, 0.0
        for i in range(count):
            t = (i+0.5)*width-self.bucket_width_m/2
            x, y = midpoint[0]-math.sin(yaw)*t, midpoint[1]+math.cos(yaw)*t
            ix = math.floor((x-self.domain_min_x_m)/self.cell_size_m)
            iy = math.floor((y-self.domain_min_y_m)/self.cell_size_m)
            if 0 <= ix < self.nx and 0 <= iy < self.ny:
                depth = max(0.0, self.heights_m[self.index(ix, iy)]-midpoint[2])
                maximum = max(maximum, depth)
                if depth > 0:
                    active += width
        return maximum, active

    def _blade_wrench(self, midpoint, yaw, vx, vy, omega):
        # Integrate local blade velocity, including yaw. The yaw moment is
        # about the edge centre; the host adds the moment arm to the body.
        count = max(1, math.ceil(self.bucket_width_m/self.subgrid_step_m))
        width = self.bucket_width_m/count
        force, torque = [0.0, 0.0, 0.0], 0.0
        for i in range(count):
            t = (i+0.5)*width-self.bucket_width_m/2
            rx, ry = -math.sin(yaw)*t, math.cos(yaw)*t
            ix = math.floor((midpoint[0]+rx-self.domain_min_x_m)/self.cell_size_m)
            iy = math.floor((midpoint[1]+ry-self.domain_min_y_m)/self.cell_size_m)
            if not (0 <= ix < self.nx and 0 <= iy < self.ny):
                continue
            depth = max(0.0, self.heights_m[self.index(ix, iy)]-midpoint[2])
            local = self._cutting_force(penetration_m=depth, active_width_m=width,
                velocity_x_m_s=vx-omega*ry, velocity_y_m_s=vy+omega*rx)
            for axis in range(3):
                force[axis] += local[axis]
            torque += rx*local[1]-ry*local[0]
        return tuple(force), torque

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

    def _remove_sweep(self, start, end, yaw0=0.0, yaw1=0.0):
        # Split at the blade centre so pure rotation cannot create a bow-tie
        # quadrilateral. Linear endpoint chords approximate each <= 1 cm step.
        def edge(center, yaw, t):
            return (center[0]-math.sin(yaw)*t, center[1]+math.cos(yaw)*t)
        removed = 0.0
        target = max(0.0, (start[2]+end[2])/2)
        for lo, hi in ((-self.bucket_width_m/2, 0), (0, self.bucket_width_m/2)):
            a, b = edge(start, yaw0, lo), edge(start, yaw0, hi)
            c, d = edge(end, yaw1, hi), edge(end, yaw1, lo)
            for triangle in ([a,b,c], [a,c,d]):
                if polygon_area(triangle) <= 1e-16:
                    continue
                cross = ((triangle[1][0]-triangle[0][0])*(triangle[2][1]-triangle[0][1])
                         -(triangle[1][1]-triangle[0][1])*(triangle[2][0]-triangle[0][0]))
                if cross < 0:
                    triangle.reverse()
                xs, ys = zip(*triangle)
                for iy in self._cell_range(min(ys), max(ys), self.domain_min_y_m, self.ny):
                    for ix in self._cell_range(min(xs), max(xs), self.domain_min_x_m, self.nx):
                        if self.payload_volume_m3 >= self.bucket_capacity_m3-1e-14:
                            return removed
                        offset = self.index(ix, iy)
                        if offset not in self._fragments:
                            x = self.domain_min_x_m+ix*self.cell_size_m
                            y = self.domain_min_y_m+iy*self.cell_size_m
                            r = self.cell_size_m
                            self._fragments[offset] = [([(x,y),(x+r,y),(x+r,y+r),(x,y+r)], self.heights_m[offset])]
                        updated = []
                        for poly, height in self._fragments[offset]:
                            if height <= target:
                                updated.append((poly, height))
                                continue
                            inside = poly
                            outside_parts = []
                            for a, b in zip(triangle, triangle[1:]+triangle[:1]):
                                inside, outside = split_polygon(inside, a, b)
                                if polygon_area(outside) > 1e-16:
                                    outside_parts.append((outside, height))
                                if not inside:
                                    break
                            area = polygon_area(inside)
                            if area <= 1e-16:
                                updated.append((poly, height))
                                continue
                            actual = min(area*(height-target),
                                         max(0.0, self.bucket_capacity_m3-self.payload_volume_m3))
                            updated.extend(outside_parts)
                            updated.append((inside, height-actual/area))
                            self.heights_m[offset] -= actual/self.cell_size_m**2
                            self.payload_volume_m3 += actual
                            self.excavated_volume_m3 += actual
                            removed += actual
                        self._fragments[offset] = updated
        return removed


def all_finite(interactions: Iterable[Interaction]) -> bool:
    return all(
        all(math.isfinite(value) for value in interaction.__dict__.values())
        for interaction in interactions
    )
