"""Two-lane road, trajectory, vehicle, and live telemetry painter."""
from __future__ import annotations

import math
import tkinter as tk
from dataclasses import replace

from ..config import ROAD, SIMULATION, VEHICLE, RoadConfig, SimulationConfig, VehicleParams
from ..simulation import SimulationRun
from ..ui.theme import ThemeMode, UiPalette, palette_for
from ..ui.typography import TYPE_SCALE, ui_font
from .geometry import RoadViewport
from .vehicle import (EGO_STYLE, GVT_SPRITE_PATH, TARGET_STYLE,
                      SpriteCarPainter)


class RoadRenderer:
    def __init__(self, road: RoadConfig = ROAD,
                 vehicle: VehicleParams = VEHICLE,
                 simulation: SimulationConfig = SIMULATION,
                 palette: UiPalette | None = None) -> None:
        self.road, self.vehicle, self.simulation = road, vehicle, simulation
        self.palette = palette or palette_for(ThemeMode.DARK)
        self.ego_vehicle_painter = SpriteCarPainter()
        self.target_vehicle_painter = SpriteCarPainter(GVT_SPRITE_PATH)

    def set_palette(self, palette: UiPalette) -> None:
        self.palette = palette

    def draw(self, canvas: tk.Canvas, run: SimulationRun, index: int,
             lane_width_m: float, line_style: str) -> None:
        self.vehicle = run.vehicle
        canvas.delete("all")
        sample, path = run.samples[index], run.path
        width, height = max(canvas.winfo_width(), 1), max(canvas.winfo_height(), 1)
        divider_y = path.side * (path.line_offset_m + self.vehicle.body_width_m / 2)
        # A road edge should read as a shoulder, not as half a canvas of grass.
        # Move the tested boundary lower while keeping the exact metre scale.
        viewport_road = (replace(self.road, divider_y_fraction=0.68)
                         if run.road_layout == "road_edge" else self.road)
        view = RoadViewport(width, height, sample.front_x_m, divider_y,
                            lane_width_m, viewport_road)
        screen = view.screen
        opposite_y = divider_y - path.side * lane_width_m
        far_y = divider_y + path.side * lane_width_m
        _, divider_px = screen(sample.front_x_m, divider_y)
        _, outer_a_px = screen(sample.front_x_m, opposite_y)
        _, outer_b_px = screen(sample.front_x_m, far_y)
        road_top, road_bottom = sorted((outer_a_px, outer_b_px))

        canvas.create_rectangle(0, 0, width, height,
                                fill=self.palette.background, outline="")
        if run.road_layout == "road_edge":
            lane_top, lane_bottom = sorted((outer_a_px, divider_px))
            canvas.create_rectangle(18, lane_top, width - 18, lane_bottom,
                                    fill="#323a44", outline="")
            if path.side == 1:
                canvas.create_rectangle(18, 0, width - 18, divider_px,
                                        fill="#6d9254", outline="")
            else:
                canvas.create_rectangle(18, divider_px, width - 18, height,
                                        fill="#6d9254", outline="")
            canvas.create_line(18, outer_a_px, width - 18, outer_a_px,
                               fill="#e9eef2", width=view.marking_width_px,
                               dash=(12, 8))
        else:
            canvas.create_rectangle(18, road_top, width - 18, road_bottom,
                                    fill="#323a44", outline="")
            canvas.create_rectangle(18, road_top + 3, width - 18, divider_px - 3,
                                    fill="#343d47", outline="")
            edge_color = "#e9eef2" if run.mode == "ELK" else "#ddb84b"
            for edge_px in (outer_a_px, outer_b_px):
                canvas.create_line(18, edge_px, width - 18, edge_px,
                                   fill=edge_color, width=view.marking_width_px)
        effective_line_style = ("Solid" if run.road_layout == "solid_line"
                                else "Dashed" if run.road_layout in ("oncoming", "overtaking")
                                else line_style)
        if run.road_layout != "road_edge" and effective_line_style == "Solid":
            canvas.create_line(18, divider_px, width - 18, divider_px,
                               fill="#e9eef2", width=view.marking_width_px)
        elif run.road_layout != "road_edge":
            x_min, x_max = view.visible_x_range()
            period = self.road.dashed_period_m
            world_x = math.floor(x_min / period) * period
            while world_x < x_max:
                start_px, _ = screen(world_x, divider_y)
                end_px, _ = screen(world_x + self.road.dashed_mark_m, divider_y)
                start_px, end_px = max(18, start_px), min(width - 18, end_px)
                if end_px > start_px:
                    canvas.create_line(start_px, divider_px, end_px, divider_px,
                                       fill="#e9eef2", width=view.marking_width_px,
                                       capstyle="butt")
                world_x += period

        x_min, x_max = view.visible_x_range()
        first = max(0, int(x_min * 2))
        last = max(first + 1, int(x_max * 2))
        reference = [screen(*path.at(j * 0.5)[:2]) for j in range(first, last)]
        if len(reference) > 1:
            canvas.create_line(*[v for point in reference for v in point],
                               fill="#e88a63", width=2, smooth=True)
        trail = [screen(s.front_x_m, s.front_y_m)
                 for s in run.samples[max(0, index - 300):index + 1:3]]
        if len(trail) > 1:
            canvas.create_line(*[v for point in trail for v in point],
                               fill=self.palette.cyan, width=2, smooth=True)

        if run.target_samples:
            target_trail = [screen(target.x_m, target.y_m)
                            for target in run.target_samples[:index + 1:3]]
            if len(target_trail) > 1:
                canvas.create_line(
                    *[coordinate for point in target_trail for coordinate in point],
                    fill=self.palette.green, width=2, dash=(5, 3), smooth=True)
            target_sample = run.target_samples[index]
            target_outline = self.target_vehicle_painter.draw(
                canvas, view, center_x_m=target_sample.x_m,
                center_y_m=target_sample.y_m, yaw_deg=target_sample.yaw_deg,
                length_m=self.vehicle.body_length_m,
                width_m=self.vehicle.body_width_m,
                style=TARGET_STYLE, label=target_sample.label)
            self._draw_target_locator(
                canvas, view, sample.front_x_m, target_sample.x_m,
                target_sample.y_m, target_outline, width, height)

        yaw = math.radians(sample.yaw_deg)
        ego_center_x = sample.front_x_m - self.vehicle.l_front_m * math.cos(yaw)
        ego_center_y = sample.front_y_m - self.vehicle.l_front_m * math.sin(yaw)
        self.ego_vehicle_painter.draw(
            canvas, view, center_x_m=ego_center_x, center_y_m=ego_center_y,
            yaw_deg=sample.yaw_deg, length_m=self.vehicle.body_length_m,
            width_m=self.vehicle.body_width_m, style=EGO_STYLE)
        axle_x, axle_y = screen(sample.front_x_m, sample.front_y_m)
        canvas.create_oval(axle_x - 3, axle_y - 3, axle_x + 3, axle_y + 3,
                           fill="#5ba8ef", outline="#e2e9ee")

        legend_y = height - 24
        legend_right = 470 if run.target_samples else 334
        canvas.create_rectangle(14, legend_y - 15, legend_right, legend_y + 15,
                                fill=self.palette.panel, outline=self.palette.border)
        canvas.create_line(22, legend_y, 45, legend_y, fill="#e88a63", width=3)
        canvas.create_text(52, legend_y, anchor="w", fill=self.palette.foreground,
                           font=ui_font(TYPE_SCALE.label), text="NCAP reference")
        canvas.create_line(176, legend_y, 199, legend_y,
                           fill=self.palette.cyan, width=3)
        canvas.create_text(206, legend_y, anchor="w", fill=self.palette.foreground,
                           font=ui_font(TYPE_SCALE.label), text="Front axle path")
        if run.target_samples:
            canvas.create_line(326, legend_y, 349, legend_y,
                               fill=self.palette.green, width=3, dash=(5, 3))
            canvas.create_text(356, legend_y, anchor="w",
                               fill=self.palette.foreground,
                               font=ui_font(TYPE_SCALE.label), text="GVT path")

    def _draw_target_locator(
            self, canvas: tk.Canvas, view: RoadViewport, ego_x_m: float,
            target_x_m: float, target_y_m: float,
            outline: list[tuple[float, float]], width: int, height: int) -> None:
        """Keep the GVT unambiguous even before it enters the road viewport."""
        accent = "#42e6a4" if self.palette.background.startswith("#0") else "#08784f"
        min_x = min(point[0] for point in outline)
        max_x = max(point[0] for point in outline)
        min_y = min(point[1] for point in outline)
        max_y = max(point[1] for point in outline)
        visible = max_x >= 18 and min_x <= width - 18 and max_y >= 4 and min_y <= height - 4
        if visible:
            canvas.create_polygon(
                *[coordinate for point in outline for coordinate in point],
                fill="", outline=accent, width=3)
            label_x = max(42, min(width - 42, (min_x + max_x) / 2))
            label_y = max(13, min_y - 9)
            canvas.create_text(label_x, label_y, text="GVT", fill=accent,
                               font=ui_font(TYPE_SCALE.label, "bold"))
            return

        screen_x, screen_y = view.screen(target_x_m, target_y_m)
        marker_x = max(30, min(width - 30, screen_x))
        marker_y = max(18, min(height - 42, screen_y))
        points = ((marker_x - 10, marker_y - 8),
                  (marker_x + 10, marker_y),
                  (marker_x - 10, marker_y + 8))
        if screen_x < 0:
            points = tuple((2 * marker_x - x, y) for x, y in points)
        canvas.create_polygon(
            *[coordinate for point in points for coordinate in point],
            fill=accent, outline=self.palette.foreground, width=1)
        distance_m = abs(target_x_m - ego_x_m)
        anchor = "e" if screen_x > width else "w"
        text_x = marker_x - 15 if anchor == "e" else marker_x + 15
        canvas.create_text(
            text_x, marker_y, anchor=anchor, fill=accent,
            font=ui_font(TYPE_SCALE.label, "bold"),
            text=f"GVT {distance_m:.0f} m")
