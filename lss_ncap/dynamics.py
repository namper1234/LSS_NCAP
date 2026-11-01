"""Kinematic replay model and optional dynamic lateral bicycle model."""
from __future__ import annotations

import math
from dataclasses import replace

from .config import SIMULATION, SimulationConfig, VehicleParams


class KinematicBicycleModel:
    """Rear-axle bicycle pose integrated at fixed steering substeps."""

    def __init__(self, params: VehicleParams, speed_mps: float,
                 config: SimulationConfig = SIMULATION) -> None:
        if not math.isfinite(speed_mps) or speed_mps <= 0:
            raise ValueError("Longitudinal speed must be finite and positive")
        self.params = params
        self.speed_mps = speed_mps
        self.h = config.substep_s
        self.rear_x_m = -params.wheelbase_m
        self.rear_y_m = 0.0
        self.yaw_rad = 0.0
        self.yaw_rate_rad_s = 0.0

    @property
    def front_pose(self) -> tuple[float, float, float]:
        yaw = self.yaw_rad
        return (self.rear_x_m + self.params.wheelbase_m * math.cos(yaw),
                self.rear_y_m + self.params.wheelbase_m * math.sin(yaw), yaw)

    @property
    def front_lateral_speed_mps(self) -> float:
        yaw = self.yaw_rad
        return (self.speed_mps * math.sin(yaw)
                + self.params.wheelbase_m * self.yaw_rate_rad_s * math.cos(yaw))

    def step_substep(self, road_wheel_rad: float) -> None:
        if not math.isfinite(road_wheel_rad):
            raise ValueError("Road-wheel angle must be finite")
        rate = (self.speed_mps / self.params.wheelbase_m
                * math.tan(road_wheel_rad))
        middle_yaw = self.yaw_rad + rate * self.h / 2
        self.rear_x_m += self.speed_mps * math.cos(middle_yaw) * self.h
        self.rear_y_m += self.speed_mps * math.sin(middle_yaw) * self.h
        self.yaw_rad += rate * self.h
        self.yaw_rate_rad_s = rate


class LateralBicycleModel:
    """Linear A/B model integrated with fixed forward-Euler substeps."""

    def __init__(self, params: VehicleParams, vx_mps: float,
                 substep_s: float | None = None, *,
                 config: SimulationConfig = SIMULATION) -> None:
        if vx_mps <= 0:
            raise ValueError("Longitudinal speed must be positive")
        if substep_s is not None:
            config = replace(config, substep_s=substep_s)
        self.params = params
        self.p = params  # Legacy public attribute.
        self.vx = vx_mps
        self.config = config
        self.h = config.substep_s  # Legacy public attribute.
        self.vy = 0.0
        self.yaw_rate = 0.0
        m, iz, lf, lr = (params.mass_kg, params.iz_kgm2,
                         params.l_front_m, params.l_rear_m)
        cf, cr, v = (params.cornering_stiffness_f,
                      params.cornering_stiffness_r, vx_mps)
        self.a11 = -(cf + cr) / (m * v)
        self.a12 = -(lf * cf - lr * cr) / (m * v) - v
        self.a21 = -(lf * cf - lr * cr) / (iz * v)
        self.a22 = -(lf * lf * cf + lr * lr * cr) / (iz * v)
        self.b1, self.b2 = cf / m, lf * cf / iz

    def step(self, road_wheel_rad: float) -> None:
        for _ in range(round(self.config.sample_period_s / self.h)):
            self.step_substep(road_wheel_rad)

    def step_substep(self, road_wheel_rad: float) -> None:
        """Advance one fixed substep using the commanded road-wheel angle."""
        dvy = self.a11 * self.vy + self.a12 * self.yaw_rate + self.b1 * road_wheel_rad
        dr = self.a21 * self.vy + self.a22 * self.yaw_rate + self.b2 * road_wheel_rad
        self.vy += self.h * dvy
        self.yaw_rate += self.h * dr

    def steady_state_for_unit_delta(self) -> tuple[float, float]:
        determinant = self.a11 * self.a22 - self.a12 * self.a21
        if abs(determinant) < 1e-12:
            raise ValueError("Bicycle model has a singular steady-state system")
        vy = (-self.b1 * self.a22 + self.a12 * self.b2) / determinant
        rate = (-self.a11 * self.b2 + self.a21 * self.b1) / determinant
        return vy, rate
