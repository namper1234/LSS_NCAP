"""Euro NCAP LKA test path geometry."""
from __future__ import annotations

import math

from ..config import NCAP_PATH, SIMULATION, SimulationConfig
from .base import PathState


class NcapPath:
    def __init__(self, lateral_speed: float, side: int,
                 straight_s: float | None = None,
                 config: SimulationConfig = SIMULATION, *,
                 path_parameters: dict[float, tuple[float, float, float]] | None = None,
                 radius_m: float | None = None) -> None:
        parameters = path_parameters or NCAP_PATH
        if lateral_speed not in parameters:
            allowed = ", ".join(f"{value:.1f}" for value in sorted(parameters))
            raise ValueError(f"lateral_speed must be one of {allowed} m/s")
        if side not in (-1, 1):
            raise ValueError("side must be +1 (left) or -1 (right)")
        if straight_s is not None and straight_s < 0:
            raise ValueError("straight_s cannot be negative")
        self.vlat, self.side = lateral_speed, side
        self.config = config
        self.psi_deg, self.d1_table, self.d2 = parameters[lateral_speed]
        self.psi = math.radians(self.psi_deg)
        self.radius_m = config.path_radius_m if radius_m is None else radius_m
        if not math.isfinite(self.radius_m) or self.radius_m <= 0:
            raise ValueError("path radius must be finite and positive")
        radius = self.radius_m
        self.s0 = config.speed_mps * (config.approach_s if straight_s is None else straight_s)
        self.arc_len = radius * self.psi
        self.s1 = self.s0 + self.arc_len
        self.tangent_len = self.d2 / math.sin(self.psi)
        self.s2 = self.s1 + self.tangent_len
        self.arc_x = self.s0 + radius * math.sin(self.psi)
        self.arc_y = radius * (1.0 - math.cos(self.psi))

    def at(self, s: float) -> PathState:
        """Front-axle x, y, heading, curvature, and phase at path distance s."""
        radius = self.radius_m
        if s < self.s0:
            return s, 0.0, 0.0, 0.0, "Straight approach"
        if s < self.s1:
            a = (s - self.s0) / radius
            return (self.s0 + radius * math.sin(a),
                    self.side * radius * (1 - math.cos(a)),
                    self.side * a, self.side / radius,
                    f"Radius {radius:g} m")
        u = s - self.s1
        return (self.arc_x + u * math.cos(self.psi),
                self.side * (self.arc_y + u * math.sin(self.psi)),
                self.side * self.psi, 0.0, "Straight departure")

    def at_x(self, x: float) -> PathState:
        return self.at(self.distance_at_x(x))

    def distance_at_x(self, x: float) -> float:
        """Path distance corresponding to a front-axle longitudinal position."""
        radius = self.radius_m
        if x < self.s0:
            return x
        if x < self.arc_x:
            return self.s0 + radius * math.asin((x - self.s0) / radius)
        return self.s1 + (x - self.arc_x) / math.cos(self.psi)
