"""Configurable straight-arc-tangent LKA reference geometry."""
from __future__ import annotations

import math

from ..config import (NCAP_PATH, NCAP_PATH_TABLES, NCAP_STANDARD, SIMULATION,
                      SimulationConfig)
from .base import PathState


class NcapPath:
    """Derive LKA geometry from inputs, or explicitly reproduce a legacy table."""

    def __init__(self, lateral_speed: float, side: int,
                 straight_s: float | None = None,
                 config: SimulationConfig = SIMULATION, *,
                 profile: str | None = None,
                 path_parameters: dict[float, tuple[float, float, float]] | None = None,
                 radius_m: float | None = None,
                 d2_m: float | None = None) -> None:
        if profile is not None and profile not in NCAP_PATH_TABLES:
            allowed_profiles = ", ".join(NCAP_PATH_TABLES)
            raise ValueError(f"profile must be one of {allowed_profiles}")
        if side not in (-1, 1):
            raise ValueError("side must be +1 (left) or -1 (right)")
        if straight_s is not None and (not math.isfinite(straight_s) or straight_s < 0):
            raise ValueError("straight_s must be finite and nonnegative")
        self.vlat, self.side = lateral_speed, side
        self.config = config
        self.profile = profile
        if profile is None and path_parameters is None:
            if not math.isfinite(lateral_speed) or not 0 < lateral_speed < config.speed_mps:
                raise ValueError("Lateral speed must be positive and below vehicle speed")
            self.radius_m = config.path_radius_m if radius_m is None else radius_m
            self.d2 = config.lka_d2_m if d2_m is None else d2_m
            if not math.isfinite(self.d2) or self.d2 < 0:
                raise ValueError("d2 must be finite and nonnegative")
            self.psi = math.asin(lateral_speed / config.speed_mps)
            self.psi_deg = math.degrees(self.psi)
            self.d1_table = self.radius_m * (1.0 - math.cos(self.psi))
        else:
            if d2_m is not None:
                raise ValueError("d2 cannot override a protocol table; omit profile")
            parameters = NCAP_PATH if path_parameters is None else path_parameters
            if lateral_speed not in parameters:
                allowed = ", ".join(f"{value:.1f}" for value in sorted(parameters))
                raise ValueError(f"lateral_speed must be one of {allowed} m/s")
            if path_parameters is None:
                table = NCAP_PATH_TABLES[profile or NCAP_STANDARD]
                table_radius_m, self.psi_deg, self.d1_table, self.d2 = table[lateral_speed]
                if radius_m is not None and not math.isclose(radius_m, table_radius_m):
                    raise ValueError("LKA radius is fixed by the selected Euro NCAP table row")
            else:
                table_radius_m = config.path_radius_m
                self.psi_deg, self.d1_table, self.d2 = parameters[lateral_speed]
            self.psi = math.radians(self.psi_deg)
            self.radius_m = table_radius_m if radius_m is None else radius_m
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

    @property
    def line_offset_m(self) -> float:
        """Lane-marker offset from the arc and tangent lateral distances."""
        return self.d1_table + self.d2

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
