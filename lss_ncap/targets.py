"""Verified target catalogue, ordered as the six supplied reference images."""
from __future__ import annotations

import math
from dataclasses import dataclass
from types import MappingProxyType
from typing import Literal

from .config import VEHICLE


@dataclass(frozen=True, slots=True)
class TargetType:
    key: str
    name: str
    description: str
    length_m: float | None
    width_m: float
    wheelbase_m: float
    appearance: Literal["scooter", "motorcycle", "small_car", "sedan", "suv", "custom"]
    color: str
    source_url: str
    length_wheel_diameter_m: float | None = None

    @property
    def estimated_length_m(self) -> float | None:
        if self.length_wheel_diameter_m is None:
            return None
        return self.wheelbase_m + self.length_wheel_diameter_m / 2

    @property
    def default_length_m(self) -> float | None:
        return self.length_m if self.length_m is not None else self.estimated_length_m

    @property
    def option_label(self) -> str:
        if self.is_custom:
            return f"{self.name} - editable dimensions"
        default_length = self.default_length_m
        length = f"{default_length:g}" if default_length is not None else "confirm length"
        if self.estimated_length_m is not None:
            length += " (estimate)"
        return f"{self.name} - {length} x {self.width_m:.3f} m"

    @property
    def is_custom(self) -> bool:
        return self.appearance == "custom"

    def dimensions(self, confirmed_length_m: float | None = None, *,
                   width_m: float | None = None) -> tuple[float, float]:
        if self.is_custom:
            if (confirmed_length_m is None or width_m is None
                    or not all(math.isfinite(value) and value > 0
                               for value in (confirmed_length_m, width_m))):
                raise ValueError("Custom GVT length and width must be finite and positive")
            return confirmed_length_m, width_m
        if width_m is not None and not math.isclose(width_m, self.width_m):
            raise ValueError(f"{self.name} width is fixed at {self.width_m:g} m")
        length = self.length_m
        if length is None:
            if confirmed_length_m is None:
                confirmed_length_m = self.estimated_length_m
            if confirmed_length_m is None:
                raise ValueError(f"{self.name}: confirm overall length before simulation")
            if not math.isfinite(confirmed_length_m) or confirmed_length_m <= self.wheelbase_m:
                raise ValueError(
                    f"{self.name}: confirmed overall length must be finite and greater "
                    f"than its {self.wheelbase_m:g} m wheelbase")
            length = confirmed_length_m
        elif confirmed_length_m is not None and not math.isclose(confirmed_length_m, length):
            raise ValueError(f"{self.name} overall length is fixed at {length:g} m")
        return length, self.width_m

    def length_source(self, length_m: float) -> str:
        if self.is_custom:
            return "custom"
        estimate = self.estimated_length_m
        if estimate is not None and math.isclose(length_m, estimate, abs_tol=1e-9):
            return "wheelbase_plus_half_wheel_diameter"
        return "manufacturer" if self.length_m is not None else "user_confirmed"


TARGET_TYPES = (
    TargetType("mc_sta", "4activeMC-STA", "Scooter target", None, 0.540, 1.210,
               "scooter", "#e7edf5", "https://www.4activesystems.at/4activemc-sta/",
               length_wheel_diameter_m=0.400),
    TargetType("mc_amt", "4activeMC-AMT", "ASEAN motorcyclist target",
               None, 0.675, 1.255, "scooter", "#74b9dd",
               "https://www.4activesystems.at/4activemc-amt/",
               length_wheel_diameter_m=0.533),
    TargetType("mc_emt", "4activeMC-EMT", "European motorcyclist target",
               None, 0.750, 1.420, "motorcycle", "#e3d3a7",
               "https://www.4activesystems.at/4activemc-emt/",
               length_wheel_diameter_m=0.600),
    TargetType("c2_svt", "4activeC2-SVT", "Small vehicle target",
               2.800, 1.550, 1.800, "small_car", "#d9e8ef",
               "https://www.4activesystems.at/4activec2-svt/"),
    TargetType("c2_gvt", "4activeC2-GVT", "Global vehicle target",
               4.060, 1.800, 2.560, "sedan", "#58ae8a",
               "https://www.4activesystems.at/4activec2-gvt/"),
    TargetType("c2_suv", "4activeC2-SUV", "SUV target (width excludes mirrors)",
               4.350, 1.805, 2.600, "suv", "#dce2ed",
               "https://www.4activesystems.at/4activec2-suv/"),
)
TARGET_TYPES_BY_KEY = MappingProxyType({target.key: target for target in TARGET_TYPES})
DEFAULT_TARGET_TYPE = TARGET_TYPES[0]
CUSTOM_TARGET_TYPE = TargetType(
    "custom", "Custom GVT", "User-configured vehicle target",
    VEHICLE.body_length_m, VEHICLE.body_width_m, VEHICLE.wheelbase_m,
    "custom", "#58ae8a", "")
GVT_TYPES = (*TARGET_TYPES, CUSTOM_TARGET_TYPE)
GVT_TYPES_BY_KEY = MappingProxyType({target.key: target for target in GVT_TYPES})


def target_type(key: str) -> TargetType:
    try:
        return GVT_TYPES_BY_KEY[key]
    except KeyError:
        raise ValueError(f"Unknown GVT type: {key}") from None
