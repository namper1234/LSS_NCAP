"""Validated physical and display configuration for the NCAP viewer."""
from __future__ import annotations

import math
from dataclasses import dataclass


NCAP_STANDARD = "standard"
NCAP_ALTERNATIVE_DIM = "alternative_dim"

# Euro NCAP LSS v4.2 §7.2.3. Each row is exactly
# Vlat: (R [m], yaw [deg], d1 [m], d2 [m]).
NCAP_STANDARD_TABLE: dict[float, tuple[float, float, float, float]] = {
    0.2: (1200.0, 0.57, 0.06, 0.70),
    0.3: (1200.0, 0.86, 0.14, 0.90),
    0.4: (1200.0, 1.15, 0.24, 0.80),
    0.5: (1200.0, 1.43, 0.38, 0.75),
    0.6: (1200.0, 1.72, 0.54, 0.60),
    0.7: (1200.0, 2.01, 0.74, 0.53),
    0.8: (1200.0, 2.29, 0.96, 0.40),
    0.9: (1200.0, 2.58, 1.22, 0.23),
    1.0: (1200.0, 2.87, 1.50, 0.00),
}

NCAP_ALTERNATIVE_DIM_TABLE: dict[float, tuple[float, float, float, float]] = {
    0.2: (1200.0, 0.57, 0.06, 0.70),
    0.3: (1200.0, 0.86, 0.14, 0.90),
    0.4: (1200.0, 1.15, 0.24, 0.80),
    0.5: (800.0, 1.43, 0.25, 1.00),
    0.6: (800.0, 1.72, 0.36, 1.20),
    0.7: (800.0, 2.01, 0.49, 1.40),
    0.8: (800.0, 2.29, 0.64, 1.60),
    0.9: (800.0, 2.58, 0.81, 1.80),
    1.0: (800.0, 2.87, 1.00, 2.00),
}

NCAP_PATH_TABLES = {
    NCAP_STANDARD: NCAP_STANDARD_TABLE,
    NCAP_ALTERNATIVE_DIM: NCAP_ALTERNATIVE_DIM_TABLE,
}

# Compatibility view used by ELK definitions and existing integrations.
NCAP_PATH: dict[float, tuple[float, float, float]] = {
    vlat: (yaw_deg, d1_m, d2_m)
    for vlat, (_radius_m, yaw_deg, d1_m, d2_m) in NCAP_STANDARD_TABLE.items()
}


@dataclass(frozen=True, slots=True)
class SimulationConfig:
    sample_period_s: float = 0.025
    substep_s: float = 0.005
    speed_mps: float = 72.0 / 3.6
    path_radius_m: float = 1200.0
    # Scheduled Euro NCAP arc start; pursuit may steer before reaching it.
    approach_s: float = 5.0
    departure_tail_s: float = 1.5
    lookahead_min_m: float = 8.0
    lookahead_time_s: float = 0.5

    def __post_init__(self) -> None:
        positive_values = (
            self.sample_period_s, self.substep_s, self.speed_mps,
            self.path_radius_m, self.lookahead_min_m, self.lookahead_time_s,
        )
        if not all(math.isfinite(value) and value > 0 for value in positive_values):
            raise ValueError("Simulation time steps, speed, and radius must be positive")
        durations = (self.approach_s, self.departure_tail_s)
        if not all(math.isfinite(value) and value >= 0 for value in durations):
            raise ValueError("Approach and departure durations cannot be negative")
        ratio = self.sample_period_s / self.substep_s
        if not math.isclose(ratio, round(ratio), rel_tol=0.0, abs_tol=1e-9):
            raise ValueError("Substep must divide the sample period exactly")


@dataclass(frozen=True, slots=True)
class VehicleParams:
    mass_kg: float = 2485.0
    iz_kgm2: float = 4085.0
    l_front_m: float = 1.3295774647887325
    l_rear_m: float = 1.6204225352112676
    cornering_stiffness_f: float = 173426.0
    cornering_stiffness_r: float = 262814.0
    steer_ratio: float = 17.86
    max_road_wheel_rad: float = 0.785
    body_length_m: float = 4.957
    body_width_m: float = 2.013

    @property
    def wheelbase_m(self) -> float:
        return self.l_front_m + self.l_rear_m

    @property
    def front_overhang_m(self) -> float:
        """Body distance ahead of the front axle in the rectangle model."""
        return self.body_length_m / 2 - self.l_front_m

    @property
    def rear_overhang_m(self) -> float:
        """Body distance behind the rear axle in the rectangle model."""
        return self.body_length_m / 2 - self.l_rear_m

    @property
    def understeer_gradient_s2_per_m(self) -> float:
        """Steady-state gradient derived from this model's axle stiffnesses."""
        return self.mass_kg / self.wheelbase_m * (
            self.l_rear_m / self.cornering_stiffness_f
            - self.l_front_m / self.cornering_stiffness_r)

    def __post_init__(self) -> None:
        if not all(math.isfinite(value) and value > 0 for value in (
            self.mass_kg, self.iz_kgm2, self.l_front_m, self.l_rear_m,
            self.cornering_stiffness_f, self.cornering_stiffness_r,
            self.steer_ratio, self.max_road_wheel_rad,
            self.body_length_m, self.body_width_m,
        )):
            raise ValueError("Vehicle parameters must be positive")


@dataclass(frozen=True, slots=True)
class RoadConfig:
    lane_width_options_m: tuple[float, ...] = (3.5, 3.7)
    default_lane_width_m: float = 3.5
    marking_width_m: float = 0.2
    dashed_mark_m: float = 4.0
    dashed_period_m: float = 8.0
    lateral_view_margin_m: float = 3.0
    longitudinal_view_m: float = 38.0
    camera_x_fraction: float = 0.31
    divider_y_fraction: float = 0.42

    def __post_init__(self) -> None:
        if (not self.lane_width_options_m
                or not all(math.isfinite(v) and v > 0 for v in self.lane_width_options_m)):
            raise ValueError("Lane widths must be positive")
        if self.default_lane_width_m not in self.lane_width_options_m:
            raise ValueError("Default lane width must be selectable")
        scalar_values = (
            self.marking_width_m, self.dashed_mark_m, self.dashed_period_m,
            self.lateral_view_margin_m, self.longitudinal_view_m,
            self.camera_x_fraction, self.divider_y_fraction,
        )
        if not all(math.isfinite(value) for value in scalar_values):
            raise ValueError("Road configuration must be finite")
        if self.marking_width_m <= 0 or self.marking_width_m >= min(self.lane_width_options_m):
            raise ValueError("Marking width must fit inside a lane")
        if not 0 < self.dashed_mark_m < self.dashed_period_m:
            raise ValueError("Dashed mark must be shorter than its period")
        if self.lateral_view_margin_m < 0 or self.longitudinal_view_m <= 0:
            raise ValueError("Road viewport dimensions are invalid")
        if not 0 < self.camera_x_fraction < 1 or not 0 < self.divider_y_fraction < 1:
            raise ValueError("Camera fractions must be between zero and one")


SIMULATION = SimulationConfig()
VEHICLE = VehicleParams()
ROAD = RoadConfig()
