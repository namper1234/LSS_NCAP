"""Validated viewer inputs for vehicle and path configuration."""
from __future__ import annotations

import math
from dataclasses import dataclass, replace

from ..config import VehicleParams
from ..simulation import SimulationEngine


@dataclass(frozen=True, slots=True)
class VehicleConfiguration:
    wheelbase_m: float
    mass_kg: float
    steer_ratio: float
    body_length_m: float
    body_width_m: float
    lka_radius_m: float

    def build_engine(self, base: SimulationEngine) -> SimulationEngine:
        values = (self.wheelbase_m, self.mass_kg, self.steer_ratio,
                  self.body_length_m, self.body_width_m, self.lka_radius_m)
        if not all(math.isfinite(value) and value > 0 for value in values):
            raise ValueError("Vehicle dimensions, mass, steering ratio, and radius must be positive")
        if self.body_length_m < self.wheelbase_m:
            raise ValueError("Vehicle length must be at least the wheelbase")
        front_fraction = base.vehicle.l_front_m / base.vehicle.wheelbase_m
        vehicle = replace(base.vehicle,
                          l_front_m=self.wheelbase_m * front_fraction,
                          l_rear_m=self.wheelbase_m * (1 - front_fraction),
                          mass_kg=self.mass_kg, steer_ratio=self.steer_ratio,
                          body_length_m=self.body_length_m,
                          body_width_m=self.body_width_m)
        return SimulationEngine(base.config, vehicle)
