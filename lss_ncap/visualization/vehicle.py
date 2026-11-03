"""Lightweight vector top-down vehicle painter with physical dimensions."""
from __future__ import annotations

import math
import tkinter as tk
import base64
import io
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path

from PIL import Image

from ..runtime import resource_path
from ..ui.typography import TYPE_SCALE, ui_font
from .geometry import RoadViewport


ASSET_DIRECTORY = resource_path("assets")
EGO_SPRITE_PATH = ASSET_DIRECTORY / "top_down_blue_car.png"
GVT_SPRITE_PATH = ASSET_DIRECTORY / "top_down_green_gvt.png"


@dataclass(frozen=True, slots=True)
class VehicleVisualStyle:
    body: str
    body_edge: str
    roof: str
    glass: str
    accent: str
    label: str


EGO_STYLE = VehicleVisualStyle(
    body="#e5ebef", body_edge="#7195ad", roof="#c9d5dc",
    glass="#6f8999", accent="#3c8ed8", label="#162630")
TARGET_STYLE = VehicleVisualStyle(
    body="#aeb9c3", body_edge="#edf2f5", roof="#8f9da8",
    glass="#526875", accent="#e58a42", label="#15232c")


class TopDownCarPainter:
    """Draw a detailed car while retaining exact outer length and width."""

    @staticmethod
    def _world_point(center_x: float, center_y: float, yaw: float,
                     longitudinal: float, lateral: float) -> tuple[float, float]:
        return (center_x + longitudinal * math.cos(yaw) - lateral * math.sin(yaw),
                center_y + longitudinal * math.sin(yaw) + lateral * math.cos(yaw))

    def _points(self, viewport: RoadViewport, center_x: float, center_y: float,
                yaw: float, coordinates: tuple[tuple[float, float], ...]
                ) -> list[tuple[float, float]]:
        return [viewport.screen(*self._world_point(center_x, center_y, yaw, x, y))
                for x, y in coordinates]

    @staticmethod
    def _polygon(canvas: tk.Canvas, points: list[tuple[float, float]],
                 **style: object) -> None:
        canvas.create_polygon(*[value for point in points for value in point], **style)

    def draw(self, canvas: tk.Canvas, viewport: RoadViewport, *,
             center_x_m: float, center_y_m: float, yaw_deg: float,
             length_m: float, width_m: float, style: VehicleVisualStyle,
             label: str = "") -> list[tuple[float, float]]:
        """Paint the car and return its exact-dimension outer silhouette."""
        yaw = math.radians(yaw_deg)
        half_l, half_w = length_m / 2, width_m / 2
        # The nose/rear centre and widest side points reach the configured
        # physical dimensions; chamfers make the silhouette recognisably car-like.
        outline_local = (
            (half_l, 0.58 * half_w), (0.90 * half_l, half_w),
            (-0.78 * half_l, half_w), (-half_l, 0.68 * half_w),
            (-half_l, -0.68 * half_w), (-0.78 * half_l, -half_w),
            (0.90 * half_l, -half_w), (half_l, -0.58 * half_w),
        )
        outline = self._points(viewport, center_x_m, center_y_m, yaw, outline_local)
        shadow = [(x + 4, y + 4) for x, y in outline]
        self._polygon(canvas, shadow, fill="#17212a", outline="")

        # Wheels are drawn first so they appear tucked beneath the body.
        wheel_length, wheel_width = 0.72, 0.20
        for axle_x in (0.58 * half_l, -0.58 * half_l):
            for side in (-1, 1):
                wheel_center = side * (half_w - wheel_width / 3)
                wheel = self._points(viewport, center_x_m, center_y_m, yaw, (
                    (axle_x + wheel_length / 2, wheel_center + wheel_width / 2),
                    (axle_x + wheel_length / 2, wheel_center - wheel_width / 2),
                    (axle_x - wheel_length / 2, wheel_center - wheel_width / 2),
                    (axle_x - wheel_length / 2, wheel_center + wheel_width / 2),
                ))
                self._polygon(canvas, wheel, fill="#172027", outline="#080c10", width=1)

        self._polygon(canvas, outline, fill=style.body,
                      outline=style.body_edge, width=2)

        # Roof and glass are scaled from the same physical footprint.
        roof_local = (
            (0.38 * half_l, 0.68 * half_w), (-0.42 * half_l, 0.68 * half_w),
            (-0.62 * half_l, 0.48 * half_w), (-0.62 * half_l, -0.48 * half_w),
            (-0.42 * half_l, -0.68 * half_w), (0.38 * half_l, -0.68 * half_w),
            (0.60 * half_l, -0.46 * half_w), (0.60 * half_l, 0.46 * half_w),
        )
        roof = self._points(viewport, center_x_m, center_y_m, yaw, roof_local)
        self._polygon(canvas, roof, fill=style.roof, outline=style.body_edge, width=1)
        windshield = self._points(viewport, center_x_m, center_y_m, yaw, (
            (0.54 * half_l, 0.43 * half_w), (0.36 * half_l, 0.61 * half_w),
            (0.36 * half_l, -0.61 * half_w), (0.54 * half_l, -0.43 * half_w),
        ))
        rear_glass = self._points(viewport, center_x_m, center_y_m, yaw, (
            (-0.42 * half_l, 0.61 * half_w), (-0.56 * half_l, 0.44 * half_w),
            (-0.56 * half_l, -0.44 * half_w), (-0.42 * half_l, -0.61 * half_w),
        ))
        self._polygon(canvas, windshield, fill=style.glass, outline="")
        self._polygon(canvas, rear_glass, fill=style.glass, outline="")

        # Centre roof seam and front/rear light clusters reinforce heading.
        seam = self._points(viewport, center_x_m, center_y_m, yaw,
                            ((-0.38 * half_l, 0), (0.32 * half_l, 0)))
        canvas.create_line(*seam[0], *seam[1], fill=style.body_edge, width=1)
        for x, color in ((0.95 * half_l, "#f5e19b"), (-0.94 * half_l, "#d95855")):
            for side in (-1, 1):
                px, py = viewport.screen(*self._world_point(
                    center_x_m, center_y_m, yaw, x, side * 0.58 * half_w))
                radius = max(1.5, viewport.scale_px_per_m * 0.07)
                canvas.create_oval(px - radius, py - radius, px + radius, py + radius,
                                   fill=color, outline="")

        # Front bumper accent provides an immediate orientation cue.
        front = self._points(viewport, center_x_m, center_y_m, yaw, (
            (half_l, 0.50 * half_w), (half_l, -0.50 * half_w)))
        canvas.create_line(*front[0], *front[1], fill=style.accent, width=3)
        if label:
            center_px = viewport.screen(center_x_m, center_y_m)
            canvas.create_text(*center_px, fill=style.label,
                               font=ui_font(TYPE_SCALE.caption, "bold"), text=label)
        return outline


class SpriteCarPainter:
    """Dimensionally scaled, rotation-cached raster painter with vector fallback."""

    def __init__(self, asset_path: Path | None = None, cache_size: int = 128, *,
                 source_image: Image.Image | None = None) -> None:
        self.asset_path = asset_path or EGO_SPRITE_PATH
        source = (Image.open(self.asset_path).convert("RGBA")
                  if source_image is None else source_image.convert("RGBA"))
        alpha_bounds = source.getchannel("A").getbbox()
        self.source = source.crop(alpha_bounds) if alpha_bounds else source
        self.cache_size = cache_size
        self.cache: OrderedDict[tuple[int, int, int, int], tk.PhotoImage] = OrderedDict()
        self.fallback = TopDownCarPainter()

    @staticmethod
    def _cache_angle(yaw_deg: float) -> int:
        # 0.1° resolution is visually smooth for the small NCAP headings while
        # bounding the number of expensive raster rotations.
        return round((yaw_deg % 360.0) * 10)

    def _photo(self, canvas: tk.Canvas, length_px: int, width_px: int,
               yaw_deg: float) -> tk.PhotoImage:
        interpreter = id(canvas.tk)
        angle_key = self._cache_angle(yaw_deg)
        key = (interpreter, length_px, width_px, angle_key)
        if key in self.cache:
            self.cache.move_to_end(key)
            return self.cache[key]
        resampling = getattr(Image, "Resampling", Image)
        scaled = self.source.resize((length_px, width_px), resampling.LANCZOS)
        rotated = scaled.rotate(angle_key / 10, resample=resampling.BICUBIC,
                                expand=True)
        encoded = io.BytesIO()
        rotated.save(encoded, format="PNG", compress_level=1)
        photo = tk.PhotoImage(data=base64.b64encode(encoded.getvalue()), master=canvas)
        self.cache[key] = photo
        if len(self.cache) > self.cache_size:
            self.cache.popitem(last=False)
        return photo

    def draw(self, canvas: tk.Canvas, viewport: RoadViewport, *,
             center_x_m: float, center_y_m: float, yaw_deg: float,
             length_m: float, width_m: float, style: VehicleVisualStyle,
             label: str = "") -> list[tuple[float, float]]:
        # Test doubles and non-Tk canvas implementations use the deterministic
        # vector model; production Tk canvases use the reference-image sprite.
        if not hasattr(canvas, "tk") or not hasattr(canvas, "create_image"):
            return self.fallback.draw(
                canvas, viewport, center_x_m=center_x_m, center_y_m=center_y_m,
                yaw_deg=yaw_deg, length_m=length_m, width_m=width_m,
                style=style, label=label)
        length_px = max(2, round(length_m * viewport.scale_px_per_m))
        width_px = max(2, round(width_m * viewport.scale_px_per_m))
        photo = self._photo(canvas, length_px, width_px, yaw_deg)
        center = viewport.screen(center_x_m, center_y_m)
        canvas.create_image(*center, image=photo, anchor="center")
        if label:
            canvas.create_text(*center, fill="#ffffff",
                               font=ui_font(TYPE_SCALE.caption, "bold"), text=label)
        yaw = math.radians(yaw_deg)
        half_l, half_w = length_m / 2, width_m / 2
        return [viewport.screen(*TopDownCarPainter._world_point(
            center_x_m, center_y_m, yaw, x, y))
            for x, y in ((half_l, half_w), (half_l, -half_w),
                         (-half_l, -half_w), (-half_l, half_w))]
