"""Continuous first-contact detection for rectangular simulated vehicles."""
from __future__ import annotations

import math
from dataclasses import dataclass, fields, replace

from .config import VehicleParams
from .simulation import SimulationSample, TargetVehicleSample


@dataclass(frozen=True, slots=True)
class CollisionResult:
    time_s: float
    ego: SimulationSample
    target: TargetVehicleSample


def _rectangle(center_x: float, center_y: float, yaw_deg: float,
               length: float, width: float) -> tuple[tuple[float, float], ...]:
    yaw = math.radians(yaw_deg)
    longitudinal = (math.cos(yaw), math.sin(yaw))
    lateral = (-math.sin(yaw), math.cos(yaw))
    return tuple((center_x + sx * length / 2 * longitudinal[0]
                  + sy * width / 2 * lateral[0],
                  center_y + sx * length / 2 * longitudinal[1]
                  + sy * width / 2 * lateral[1])
                 for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1)))


def _ego_rectangle(sample: SimulationSample, vehicle: VehicleParams
                   ) -> tuple[tuple[float, float], ...]:
    yaw = math.radians(sample.yaw_deg)
    return _rectangle(
        sample.front_x_m - vehicle.l_front_m * math.cos(yaw),
        sample.front_y_m - vehicle.l_front_m * math.sin(yaw),
        sample.yaw_deg, vehicle.body_length_m, vehicle.body_width_m)


def _target_rectangle(sample: TargetVehicleSample, vehicle: VehicleParams
                      ) -> tuple[tuple[float, float], ...]:
    return _rectangle(sample.x_m, sample.y_m, sample.yaw_deg,
                      vehicle.body_length_m, vehicle.body_width_m)


def vehicles_intersect(ego: SimulationSample, target: TargetVehicleSample,
                       vehicle: VehicleParams) -> bool:
    return rectangles_intersect(_ego_rectangle(ego, vehicle),
                                _target_rectangle(target, vehicle))


def rectangles_intersect(first: tuple[tuple[float, float], ...],
                         second: tuple[tuple[float, float], ...],
                         tolerance_m: float = 1e-9) -> bool:
    """Separating-axis test; touching edges count as first contact."""
    for polygon in (first, second):
        for index in range(2):
            x1, y1 = polygon[index]
            x2, y2 = polygon[index + 1]
            edge_x, edge_y = x2 - x1, y2 - y1
            axis_x, axis_y = -edge_y, edge_x
            magnitude = math.hypot(axis_x, axis_y)
            axis_x, axis_y = axis_x / magnitude, axis_y / magnitude
            projection_a = [x * axis_x + y * axis_y for x, y in first]
            projection_b = [x * axis_x + y * axis_y for x, y in second]
            if (max(projection_a) < min(projection_b) - tolerance_m
                    or max(projection_b) < min(projection_a) - tolerance_m):
                return False
    return True


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
                    vehicle: VehicleParams) -> CollisionResult | None:
    """Find first contact, refining the containing sample interval to <1 ns."""
    if len(samples) != len(targets):
        raise ValueError("Ego and target trajectories must have equal lengths")
    for index, (ego, target) in enumerate(zip(samples, targets)):
        if not vehicles_intersect(ego, target, vehicle):
            continue
        if index == 0:
            return CollisionResult(ego.time_s, ego,
                                   replace(target, speed_mps=0.0))
        previous_ego, previous_target = samples[index - 1], targets[index - 1]
        low, high = 0.0, 1.0
        for _ in range(50):
            middle = (low + high) / 2
            candidate_ego = interpolate_sample(previous_ego, ego, middle)
            candidate_target = interpolate_target(previous_target, target, middle)
            if vehicles_intersect(candidate_ego, candidate_target, vehicle):
                high = middle
            else:
                low = middle
        impact_ego = replace(
            interpolate_sample(previous_ego, ego, high),
            phase="Collision", body_lateral_speed_mps=0.0,
            yaw_rate_deg_s=0.0, v_lat_mps=0.0)
        impact_target = replace(
            interpolate_target(previous_target, target, high), speed_mps=0.0)
        return CollisionResult(impact_ego.time_s, impact_ego, impact_target)
    return None
