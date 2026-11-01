"""Closed-loop Pure Pursuit steering for the front-axle NCAP path."""
from __future__ import annotations

import math
from dataclasses import dataclass

from .config import SIMULATION, SimulationConfig, VehicleParams
from .scenarios.base import ReferencePath


@dataclass(frozen=True, slots=True)
class SteeringCommand:
    road_wheel_rad: float
    curvature_per_m: float
    lookahead_x_m: float
    lookahead_y_m: float
    lookahead_distance_m: float


class PurePursuitController:
    """Pursue a rear-axle target derived from the front-axle reference path."""

    def __init__(self, vehicle: VehicleParams,
                 config: SimulationConfig = SIMULATION) -> None:
        self.vehicle = vehicle
        self.config = config
        self.lookahead_m = max(config.lookahead_min_m,
                               config.lookahead_time_s * config.speed_mps)

    def command(self, path: ReferencePath, *, front_x_m: float,
                front_y_m: float, yaw_rad: float) -> SteeringCommand:
        distance = path.distance_at_x(front_x_m)
        wheelbase = self.vehicle.wheelbase_m
        # The NCAP path is specified at the front axle. Approximate the rear
        # axle reference at an earlier path station, preserving the intended
        # look-ahead from the current rear axle. Shifting the target along the
        # front-path tangent creates a false opposite-side target at arc entry.
        tx, ty, _, _, _ = path.at(distance + self.lookahead_m - wheelbase)
        rear_x = front_x_m - wheelbase * math.cos(yaw_rad)
        rear_y = front_y_m - wheelbase * math.sin(yaw_rad)
        dx, dy = tx - rear_x, ty - rear_y
        chord_squared = dx * dx + dy * dy
        if chord_squared <= 1e-12:
            raise ValueError("Pure Pursuit target coincides with the rear axle")
        lateral_target = -dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
        curvature = 2.0 * lateral_target / chord_squared
        road_wheel = math.atan(wheelbase * curvature)
        road_wheel = max(-self.vehicle.max_road_wheel_rad,
                         min(self.vehicle.max_road_wheel_rad, road_wheel))
        applied_curvature = math.tan(road_wheel) / wheelbase
        return SteeringCommand(road_wheel, applied_curvature, tx, ty,
                               self.lookahead_m)

    def reference_steering_wheel_rad(self, curvature_per_m: float) -> float:
        road_wheel = math.atan(self.vehicle.wheelbase_m * curvature_per_m)
        return self.vehicle.steer_ratio * max(
            -self.vehicle.max_road_wheel_rad,
            min(self.vehicle.max_road_wheel_rad, road_wheel))

    @property
    def lateral_gain_mps_per_deg_s(self) -> float:
        speed = self.config.speed_mps
        return (speed * speed / (self.vehicle.steer_ratio * self.vehicle.wheelbase_m)
                * math.pi / 180.0)
