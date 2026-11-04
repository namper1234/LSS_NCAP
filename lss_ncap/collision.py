"""Continuous first-contact detection for rectangular simulated vehicles."""
from __future__ import annotations

import math
from collections.abc import Iterator
from dataclasses import dataclass, fields, replace

from .config import VehicleParams
from .simulation import SimulationSample, TargetVehicleSample

Polygon = tuple[tuple[float, float], ...]


def _separating_axes(
        first: Polygon, second: Polygon) -> Iterator[tuple[float, float]]:
    """Yield the two distinct edge normals from each rectangular footprint."""
    for polygon in (first, second):
        for index in range(2):
            x1, y1 = polygon[index]
            x2, y2 = polygon[index + 1]
            axis_x, axis_y = -(y2 - y1), x2 - x1
            magnitude = math.hypot(axis_x, axis_y)
            yield axis_x / magnitude, axis_y / magnitude


@dataclass(frozen=True, slots=True)
class CollisionResult:
    time_s: float
    ego: SimulationSample
    target: TargetVehicleSample
    contact_point_m: tuple[float, float]


def vehicle_rectangle(center_x: float, center_y: float, yaw_deg: float,
               length: float, width: float) -> tuple[tuple[float, float], ...]:
    yaw = math.radians(yaw_deg)
    longitudinal = (math.cos(yaw), math.sin(yaw))
    lateral = (-math.sin(yaw), math.cos(yaw))
    return tuple((center_x + sx * length / 2 * longitudinal[0]
                  + sy * width / 2 * lateral[0],
                  center_y + sx * length / 2 * longitudinal[1]
                  + sy * width / 2 * lateral[1])
                 for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1)))


def ego_polygon(sample: SimulationSample, vehicle: VehicleParams
                ) -> tuple[tuple[float, float], ...]:
    yaw = math.radians(sample.yaw_deg)
    return vehicle_rectangle(
        sample.front_x_m - vehicle.l_front_m * math.cos(yaw),
        sample.front_y_m - vehicle.l_front_m * math.sin(yaw),
        sample.yaw_deg, vehicle.body_length_m, vehicle.body_width_m)


def target_polygon(sample: TargetVehicleSample, vehicle: VehicleParams
                   ) -> tuple[tuple[float, float], ...]:
    return vehicle_rectangle(sample.x_m, sample.y_m, sample.yaw_deg,
                      vehicle.body_length_m, vehicle.body_width_m)


def lane_marker_clearance_m(ego: SimulationSample, vehicle: VehicleParams,
                            marker_inner_y_m: float, side: int) -> float:
    """Signed road-frame body clearance: positive inside, negative beyond the marker."""
    if side not in (-1, 1):
        raise ValueError("Departure side must be +1 or -1")
    return side * marker_inner_y_m - max(side * y for _, y in ego_polygon(ego, vehicle))


def vehicles_intersect(ego: SimulationSample, target: TargetVehicleSample,
                       vehicle: VehicleParams,
                       target_vehicle: VehicleParams | None = None) -> bool:
    return rectangles_intersect(ego_polygon(ego, vehicle),
                                target_polygon(target, target_vehicle or vehicle))


def lateral_separation_m(ego: SimulationSample, target: TargetVehicleSample,
                         vehicle: VehicleParams,
                         target_vehicle: VehicleParams | None = None) -> float:
    """Road-frame gap between vehicle polygons; zero means lateral overlap."""
    ego_y = [point[1] for point in ego_polygon(ego, vehicle)]
    target_y = [point[1] for point in target_polygon(target, target_vehicle or vehicle)]
    return max(0.0, min(target_y) - max(ego_y), min(ego_y) - max(target_y))


def time_to_collision_s(ego: SimulationSample, target: TargetVehicleSample,
                        vehicle: VehicleParams, ego_speed_mps: float,
                        target_vehicle: VehicleParams | None = None) -> float | None:
    """Constant-velocity TTC for the current oriented vehicle polygons."""
    first, second = ego_polygon(ego, vehicle), target_polygon(target, target_vehicle or vehicle)
    ego_yaw, target_yaw = math.radians(ego.yaw_deg), math.radians(target.yaw_deg)
    relative_velocity = (
        target.speed_mps * math.cos(target_yaw) - ego_speed_mps * math.cos(ego_yaw),
        target.speed_mps * math.sin(target_yaw) - ego_speed_mps * math.sin(ego_yaw),
    )
    entry, exit_time = -math.inf, math.inf
    for axis_x, axis_y in _separating_axes(first, second):
        first_projection = [x * axis_x + y * axis_y for x, y in first]
        second_projection = [x * axis_x + y * axis_y for x, y in second]
        first_min, first_max = min(first_projection), max(first_projection)
        second_min, second_max = min(second_projection), max(second_projection)
        relative_speed = (relative_velocity[0] * axis_x
                          + relative_velocity[1] * axis_y)
        if abs(relative_speed) < 1e-12:
            if second_max < first_min or first_max < second_min:
                return None
            continue
        first_time = (first_min - second_max) / relative_speed
        second_time = (first_max - second_min) / relative_speed
        axis_entry, axis_exit = sorted((first_time, second_time))
        entry, exit_time = max(entry, axis_entry), min(exit_time, axis_exit)
        if entry > exit_time:
            return None
    if exit_time < 0:
        return None
    return max(0.0, entry)


def rectangles_intersect(first: tuple[tuple[float, float], ...],
                         second: tuple[tuple[float, float], ...],
                         tolerance_m: float = 1e-9) -> bool:
    """Separating-axis test; touching edges count as first contact."""
    return polygon_separation_m(first, second) <= tolerance_m


def polygon_separation_m(first: tuple[tuple[float, float], ...],
                         second: tuple[tuple[float, float], ...]) -> float:
    """Largest SAT gap, a conservative lower bound on polygon distance."""
    separation = 0.0
    for axis_x, axis_y in _separating_axes(first, second):
        projection_a = [x * axis_x + y * axis_y for x, y in first]
        projection_b = [x * axis_x + y * axis_y for x, y in second]
        separation = max(separation, min(projection_b) - max(projection_a),
                         min(projection_a) - max(projection_b))
    return separation


def clip_longitudinal(polygon: tuple[tuple[float, float], ...],
                      minimum_x: float, maximum_x: float
                      ) -> tuple[tuple[float, float], ...]:
    """Clip a convex footprint to the target's road-frame longitudinal span."""
    for boundary, sign in ((minimum_x, 1), (maximum_x, -1)):
        result = []
        for start, end in zip(polygon, polygon[1:] + polygon[:1]):
            start_inside = sign * (start[0] - boundary) >= 0
            end_inside = sign * (end[0] - boundary) >= 0
            if start_inside:
                result.append(start)
            if start_inside != end_inside:
                fraction = (boundary - start[0]) / (end[0] - start[0])
                result.append((boundary, start[1] + fraction * (end[1] - start[1])))
        polygon = tuple(result)
    return polygon


def contact_point(first: tuple[tuple[float, float], ...],
                  second: tuple[tuple[float, float], ...]) -> tuple[float, float]:
    """Closest boundary pair midpoint at first contact (not vehicle centres)."""
    candidates = []
    for vertices, edges in ((first, second), (second, first)):
        for point in vertices:
            for start, end in zip(edges, edges[1:] + edges[:1]):
                dx, dy = end[0] - start[0], end[1] - start[1]
                fraction = max(0.0, min(1.0, (
                    (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
                ) / (dx * dx + dy * dy)))
                closest = (start[0] + fraction * dx, start[1] + fraction * dy)
                candidates.append((math.dist(point, closest),
                                   ((point[0] + closest[0]) / 2,
                                    (point[1] + closest[1]) / 2)))
    return min(candidates)[1]


def _interpolate_dataclass(left: object, right: object, fraction: float) -> object:
    values: dict[str, object] = {}
    for field in fields(left):  # type: ignore[arg-type]
        first, second = getattr(left, field.name), getattr(right, field.name)
        if isinstance(first, (int, float)) and isinstance(second, (int, float)):
            values[field.name] = first + (second - first) * fraction
        else:
            values[field.name] = second if fraction >= 0.5 else first
    return replace(left, **values)  # type: ignore[arg-type]


def interpolate_sample(left: SimulationSample, right: SimulationSample,
                       fraction: float) -> SimulationSample:
    return _interpolate_dataclass(left, right, fraction)  # type: ignore[return-value]


def interpolate_target(left: TargetVehicleSample, right: TargetVehicleSample,
                       fraction: float) -> TargetVehicleSample:
    return _interpolate_dataclass(left, right, fraction)  # type: ignore[return-value]


def first_collision(samples: tuple[SimulationSample, ...],
                    targets: tuple[TargetVehicleSample, ...],
                    vehicle: VehicleParams,
                    target_vehicle: VehicleParams | None = None) -> CollisionResult | None:
    """Find earliest polygon contact, including a pass between output samples."""
    if len(samples) != len(targets):
        raise ValueError("Ego and target trajectories must have equal lengths")
    target_vehicle = target_vehicle or vehicle
    for index, (ego, target) in enumerate(zip(samples, targets)):
        if index == 0:
            if not vehicles_intersect(ego, target, vehicle, target_vehicle):
                continue
            impact_ego, impact_target = ego, target
        else:
            previous_ego, previous_target = samples[index - 1], targets[index - 1]
            relative_translation = math.hypot(
                ego.front_x_m - previous_ego.front_x_m - target.x_m + previous_target.x_m,
                ego.front_y_m - previous_ego.front_y_m - target.y_m + previous_target.y_m)
            motion_bound = (
                relative_translation
                + (vehicle.l_front_m + math.hypot(vehicle.body_length_m, vehicle.body_width_m) / 2)
                * abs(math.radians(ego.yaw_deg - previous_ego.yaw_deg))
                + math.hypot(target_vehicle.body_length_m, target_vehicle.body_width_m) / 2
                * abs(math.radians(target.yaw_deg - previous_target.yaw_deg)))

            def search(low: float, high: float) -> float | None:
                middle = (low + high) / 2
                candidate_ego = interpolate_sample(previous_ego, ego, middle)
                candidate_target = interpolate_target(previous_target, target, middle)
                gap = polygon_separation_m(
                    ego_polygon(candidate_ego, vehicle),
                    target_polygon(candidate_target, target_vehicle))
                if gap > motion_bound * (high - low) / 2 + 1e-9:
                    return None
                if (high - low) * (ego.time_s - previous_ego.time_s) < 1e-9:
                    for fraction in (low, middle, high):
                        if vehicles_intersect(
                                interpolate_sample(previous_ego, ego, fraction),
                                interpolate_target(previous_target, target, fraction),
                                vehicle, target_vehicle):
                            return fraction
                    return None
                first = search(low, middle)
                return first if first is not None else search(middle, high)

            fraction = search(0.0, 1.0)
            if fraction is None:
                continue
            impact_ego = interpolate_sample(previous_ego, ego, fraction)
            impact_target = interpolate_target(previous_target, target, fraction)
        impact_ego = replace(
            impact_ego,
            phase="Collision", body_lateral_speed_mps=0.0,
            yaw_rate_deg_s=0.0, v_lat_mps=0.0)
        impact_target = replace(
            impact_target, speed_mps=0.0)
        return CollisionResult(
            impact_ego.time_s, impact_ego, impact_target,
            contact_point(ego_polygon(impact_ego, vehicle),
                          target_polygon(impact_target, target_vehicle)))
    return None
