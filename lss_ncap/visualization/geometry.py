"""Physical road and vehicle geometry shared by renderers and tests."""
from __future__ import annotations

import math
from dataclasses import dataclass

from ..config import ROAD, RoadConfig, VehicleParams
from ..simulation import SimulationSample


@dataclass(frozen=True, slots=True)
class RoadViewport:
    width_px: int
    height_px: int
    front_x_m: float
    divider_y_m: float
    lane_width_m: float
    road: RoadConfig = ROAD

    def __post_init__(self) -> None:
        if self.width_px <= 0 or self.height_px <= 0 or self.lane_width_m <= 0:
            raise ValueError("Viewport size and lane width must be positive")

    @property
    def scale_px_per_m(self) -> float:
        return min(self.width_px / self.road.longitudinal_view_m,
                   self.height_px / (2 * self.lane_width_m + self.road.lateral_view_margin_m))

    @property
    def marking_width_px(self) -> float:
        return self.road.marking_width_m * self.scale_px_per_m

    def screen(self, world_x_m: float, world_y_m: float) -> tuple[float, float]:
        scale = self.scale_px_per_m
        return (self.width_px * self.road.camera_x_fraction
                + (world_x_m - self.front_x_m) * scale,
                self.height_px * self.road.divider_y_fraction
                - (world_y_m - self.divider_y_m) * scale)

    def visible_x_range(self) -> tuple[float, float]:
        scale = self.scale_px_per_m
        camera_x = self.width_px * self.road.camera_x_fraction
        return (self.front_x_m - camera_x / scale,
                self.front_x_m + (self.width_px - camera_x) / scale)


def vehicle_corners(sample: SimulationSample, params: VehicleParams,
                    viewport: RoadViewport) -> list[tuple[float, float]]:
    yaw = math.radians(sample.yaw_deg)
    cx = sample.front_x_m - params.l_front_m * math.cos(yaw)
    cy = sample.front_y_m - params.l_front_m * math.sin(yaw)
    half_length, half_width = params.body_length_m / 2, params.body_width_m / 2
    corners = []
    for longitudinal, lateral in ((half_length, -half_width),
                                   (half_length, half_width),
                                   (-half_length, half_width),
                                   (-half_length, -half_width)):
        corners.append(viewport.screen(
            cx + longitudinal * math.cos(yaw) - lateral * math.sin(yaw),
            cy + longitudinal * math.sin(yaw) + lateral * math.cos(yaw),
        ))
    return corners


def oriented_rectangle(center_x_m: float, center_y_m: float, yaw_deg: float,
                       length_m: float, width_m: float,
                       viewport: RoadViewport) -> list[tuple[float, float]]:
    yaw = math.radians(yaw_deg)
    half_length, half_width = length_m / 2, width_m / 2
    return [viewport.screen(
        center_x_m + longitudinal * math.cos(yaw) - lateral * math.sin(yaw),
        center_y_m + longitudinal * math.sin(yaw) + lateral * math.cos(yaw),
    ) for longitudinal, lateral in (
        (half_length, -half_width), (half_length, half_width),
        (-half_length, half_width), (-half_length, -half_width),
    )]
