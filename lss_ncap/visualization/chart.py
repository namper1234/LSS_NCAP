"""Independent, theme-aware steering and lateral-velocity chart renderers."""
from __future__ import annotations

import math
import tkinter as tk
from dataclasses import dataclass
from typing import Callable

from ..simulation import SimulationRun
from ..ui.chart_interaction import ChartView
from ..ui.theme import ThemeMode, UiPalette, palette_for
from ..ui.typography import TYPE_SCALE, ui_font


@dataclass(frozen=True, slots=True)
class ChartGeometry:
    left: float
    right: float
    top: float
    bottom: float


class BaseChartRenderer:
    """Shared timeline, grid, marker, and palette behavior."""

    def __init__(self, palette: UiPalette | None = None,
                 view: ChartView | None = None) -> None:
        self.palette = palette or palette_for(ThemeMode.DARK)
        self.view = view or ChartView()

    def set_palette(self, palette: UiPalette) -> None:
        self.palette = palette

    @staticmethod
    def _dimensions(canvas: tk.Canvas) -> tuple[int, int, ChartGeometry]:
        width, height = max(canvas.winfo_width(), 1), max(canvas.winfo_height(), 190)
        return width, height, ChartGeometry(54, max(55, width - 18), 82, height - 34)

    def _background(self, canvas: tk.Canvas, width: int, height: int) -> None:
        canvas.delete("all")
        canvas.create_rectangle(0, 0, width, height,
                                fill=self.palette.panel, outline="")

    def _x_positions(self, run: SimulationRun, geometry: ChartGeometry) -> list[float]:
        count = len(run.samples)
        span = self.view.x_max - self.view.x_min
        return [geometry.left + (geometry.right - geometry.left)
                * (i / max(1, count - 1) - self.view.x_min) / span
                for i in range(count)]

    @staticmethod
    def _draw_series(canvas: tk.Canvas, points: list[tuple[float, float]],
                     geometry: ChartGeometry, **style: object) -> None:
        visible = [(x, y) for x, y in points
                   if geometry.left <= x <= geometry.right
                   and geometry.top <= y <= geometry.bottom]
        if len(visible) > 1:
            canvas.create_line(*[coordinate for point in visible for coordinate in point],
                               **style)

    def _grid(self, canvas: tk.Canvas, geometry: ChartGeometry,
              values: tuple[float, ...], position: Callable[[float], float],
              decimals: int) -> None:
        for value in values:
            y = position(value)
            canvas.create_line(geometry.left, y, geometry.right, y,
                               fill=self.palette.grid, width=1, dash=(2, 5))
            canvas.create_text(geometry.left - 8, y, anchor="e",
                               fill=self.palette.muted, font=ui_font(TYPE_SCALE.caption),
                               text=f"{value:.{decimals}f}")

    def _time_grid(self, canvas: tk.Canvas, run: SimulationRun,
                   geometry: ChartGeometry, divisions: int = 5) -> None:
        start_s = run.duration_s * self.view.x_min
        end_s = run.duration_s * self.view.x_max
        for index in range(divisions + 1):
            fraction = index / divisions
            x = geometry.left + (geometry.right - geometry.left) * fraction
            canvas.create_line(x, geometry.top, x, geometry.bottom,
                               fill=self.palette.grid, width=1, dash=(2, 7))
            anchor = "w" if index == 0 else "e" if index == divisions else "center"
            canvas.create_text(x, geometry.bottom + 10, anchor=anchor,
                               fill=self.palette.muted, font=ui_font(TYPE_SCALE.caption),
                               text=f"{start_s + (end_s - start_s) * fraction:.2f} s")

    @staticmethod
    def _nice_step(raw_step: float) -> float:
        if raw_step <= 0:
            return 1.0
        magnitude = 10 ** math.floor(math.log10(raw_step))
        normalized = raw_step / magnitude
        factor = next(value for value in (1.0, 1.25, 1.5, 2.0, 2.5, 5.0, 10.0)
                      if normalized <= value)
        return factor * magnitude

    @classmethod
    def _linear_axis(cls, minimum: float, maximum: float,
                     target_intervals: int = 4) -> tuple[float, float, tuple[float, ...]]:
        span = max(maximum - minimum, 1e-9)
        padding = max(span * 0.08, 1e-6)
        padded_min, padded_max = minimum - padding, maximum + padding
        step = cls._nice_step((padded_max - padded_min) / target_intervals)
        lower = math.floor(padded_min / step) * step
        upper = math.ceil(padded_max / step) * step
        if minimum >= 0:
            lower = 0.0
        if maximum <= 0:
            upper = 0.0
        count = round((upper - lower) / step)
        ticks = tuple(lower + index * step for index in range(count + 1))
        return lower, upper, ticks

    @classmethod
    def _ticks_between(cls, minimum: float, maximum: float,
                       target_intervals: int = 4) -> tuple[float, ...]:
        step = cls._nice_step((maximum - minimum) / target_intervals)
        first = math.ceil(minimum / step) * step
        count = max(0, math.floor((maximum - first) / step))
        return tuple(first + index * step for index in range(count + 1))

    def _visible_y_axis(self, minimum: float, maximum: float
                        ) -> tuple[float, float, tuple[float, ...]]:
        span = maximum - minimum
        visible_min = minimum + span * self.view.y_min
        visible_max = minimum + span * self.view.y_max
        return visible_min, visible_max, self._ticks_between(visible_min, visible_max)

    def _events(self, canvas: tk.Canvas, run: SimulationRun,
                geometry: ChartGeometry) -> None:
        for time_s, label, color in (
            (run.reference_arc_start_s, "NCAP arc", self.palette.orange),
        ):
            fraction = time_s / run.duration_s
            if not self.view.x_min <= fraction <= self.view.x_max:
                continue
            x = geometry.left + ((geometry.right - geometry.left)
                                 * (fraction - self.view.x_min)
                                 / (self.view.x_max - self.view.x_min))
            canvas.create_line(x, geometry.top, x, geometry.bottom,
                               fill=color, width=1, dash=(3, 4))
            canvas.create_text(x + 4, geometry.top + 4, anchor="nw", fill=color,
                               font=ui_font(TYPE_SCALE.caption, "bold"), text=label)

    def _selection(self, canvas: tk.Canvas, x: float,
                   geometry: ChartGeometry, y: float, color: str) -> None:
        canvas.create_line(x, geometry.top, x, geometry.bottom,
                           fill=self.palette.muted, width=1, dash=(4, 3))
        canvas.create_oval(x - 4, y - 4, x + 4, y + 4,
                           fill=color, outline=self.palette.panel)


class SteeringAngleChartRenderer(BaseChartRenderer):
    def draw(self, canvas: tk.Canvas, run: SimulationRun, index: int) -> None:
        width, height, geometry = self._dimensions(canvas)
        self._background(canvas, width, height)
        sample = run.samples[index]
        canvas.create_text(16, 12, anchor="nw", fill=self.palette.foreground,
                           font=ui_font(TYPE_SCALE.body, "bold"),
                           text="STEERING WHEEL ANGLE (SWA) [°]")
        canvas.create_text(16, 34, anchor="nw", fill=self.palette.cyan,
                           font=ui_font(TYPE_SCALE.label),
                           text=f"SWA {sample.steering_command_deg:+.2f}°")
        for x, label, color, dash in (
            (16, "Steering Wheel Angle (SWA)", self.palette.cyan, ()),
            (235, "NCAP reference", self.palette.orange, (2, 4)),
        ):
            canvas.create_line(x, 64, x + 16, 64, fill=color, width=3, dash=dash)
            canvas.create_text(x + 21, 64, anchor="w", fill=self.palette.muted,
                               font=ui_font(TYPE_SCALE.caption), text=label)
        fields = ("steering_command_deg", "reference_steering_wheel_deg")
        peak = max(1.0, max(abs(getattr(s, field)) for s in run.samples for field in fields))
        tick_step = self._nice_step(peak / 2)
        axis_peak = 2 * tick_step
        minimum, maximum, ticks = self._visible_y_axis(-axis_peak, axis_peak)

        def y_position(value: float) -> float:
            return geometry.bottom - ((geometry.bottom - geometry.top)
                                      * (value - minimum) / (maximum - minimum))

        self._time_grid(canvas, run, geometry)
        self._grid(canvas, geometry, ticks, y_position, 1)
        xs = self._x_positions(run, geometry)
        for field, color, dash in (
            ("reference_steering_wheel_deg", self.palette.orange, (2, 4)),
            ("steering_command_deg", self.palette.cyan, ()),
        ):
            points = [(x, y_position(getattr(s, field))) for x, s in zip(xs, run.samples)]
            self._draw_series(
                canvas, points, geometry, fill=color,
                width=3 if field == "steering_command_deg" else 2, dash=dash)
        self._events(canvas, run, geometry)
        selected_y = y_position(sample.steering_command_deg)
        if (geometry.left <= xs[index] <= geometry.right
                and geometry.top <= selected_y <= geometry.bottom):
            self._selection(canvas, xs[index], geometry, selected_y, self.palette.cyan)


class LateralVelocityChartRenderer(BaseChartRenderer):
    def draw(self, canvas: tk.Canvas, run: SimulationRun, index: int) -> None:
        width, height, geometry = self._dimensions(canvas)
        self._background(canvas, width, height)
        sample = run.samples[index]
        canvas.create_text(16, 12, anchor="nw", fill=self.palette.foreground,
                           font=ui_font(TYPE_SCALE.body, "bold"),
                           text="LATERAL VELOCITY [m/s]")
        canvas.create_text(16, 34, anchor="nw", fill=self.palette.yellow,
                           font=ui_font(TYPE_SCALE.label),
                           text=f"Actual {sample.v_lat_mps:+.3f}  ·  "
                                f"Area estimate ≈ {sample.v_lat_from_steering_mps:+.3f}")
        for x, label, color, dash in (
            (16, "Vehicle state", self.palette.yellow, ()),
            (132, "SWA area estimate", self.palette.green, (3, 3)),
        ):
            canvas.create_line(x, 64, x + 20, 64, fill=color, width=3, dash=dash)
            canvas.create_text(x + 26, 64, anchor="w", fill=self.palette.muted,
                               font=ui_font(TYPE_SCALE.caption), text=label)
        velocities = [getattr(sample_value, field) for sample_value in run.samples
                      for field in ("v_lat_mps", "v_lat_from_steering_mps")]
        raw_minimum, raw_maximum = min(0.0, min(velocities)), max(0.0, max(velocities))
        if raw_maximum - raw_minimum < 0.1:
            raw_maximum = raw_minimum + 0.1
        full_minimum, full_maximum, _ = self._linear_axis(raw_minimum, raw_maximum)
        minimum, maximum, ticks = self._visible_y_axis(full_minimum, full_maximum)

        def y_position(value: float) -> float:
            return geometry.bottom - ((geometry.bottom - geometry.top)
                                      * (value - minimum) / (maximum - minimum))

        self._time_grid(canvas, run, geometry)
        self._grid(canvas, geometry, ticks, y_position, 2)
        xs = self._x_positions(run, geometry)
        for field, color, dash in (
            ("v_lat_mps", self.palette.yellow, ()),
            ("v_lat_from_steering_mps", self.palette.green, (3, 3)),
        ):
            points = [(x, y_position(getattr(value, field)))
                      for x, value in zip(xs, run.samples)]
            self._draw_series(canvas, points, geometry, fill=color,
                              width=3 if field == "v_lat_mps" else 2, dash=dash)
        self._events(canvas, run, geometry)
        selected_y = y_position(sample.v_lat_mps)
        if (geometry.left <= xs[index] <= geometry.right
                and geometry.top <= selected_y <= geometry.bottom):
            self._selection(canvas, xs[index], geometry, selected_y, self.palette.yellow)


class LateralTrajectoryChartRenderer(BaseChartRenderer):
    """Scheduled NCAP reference versus simulated road lateral displacement."""

    def draw(self, canvas: tk.Canvas, run: SimulationRun, index: int) -> None:
        width, height, geometry = self._dimensions(canvas)
        self._background(canvas, width, height)
        sample = run.samples[index]
        canvas.create_text(16, 12, anchor="nw", fill=self.palette.foreground,
                           font=ui_font(TYPE_SCALE.body, "bold"),
                           text="LATERAL TRAJECTORY [m]")
        canvas.create_text(16, 34, anchor="nw", fill=self.palette.muted,
                           font=ui_font(TYPE_SCALE.label),
                           text=f"Tracking error {sample.lateral_error_m:+.3f} m")
        for x, label, color, dash in (
            (16, "NCAP reference", self.palette.orange, (5, 3)),
            (150, "Simulated", self.palette.magenta, ()),
        ):
            canvas.create_line(x, 64, x + 20, 64, fill=color, width=3, dash=dash)
            canvas.create_text(x + 26, 64, anchor="w", fill=self.palette.muted,
                               font=ui_font(TYPE_SCALE.caption), text=label)

        lateral_values = [getattr(value, field) for value in run.samples
                          for field in ("reference_y_m", "front_y_m")]
        raw_minimum = min(0.0, min(lateral_values))
        raw_maximum = max(0.0, max(lateral_values))
        if raw_maximum - raw_minimum < 0.1:
            raw_maximum = raw_minimum + 0.1
        full_minimum, full_maximum, _ = self._linear_axis(raw_minimum, raw_maximum)
        minimum, maximum, ticks = self._visible_y_axis(full_minimum, full_maximum)

        def y_position(value: float) -> float:
            return geometry.bottom - ((geometry.bottom - geometry.top)
                                      * (value - minimum) / (maximum - minimum))

        self._time_grid(canvas, run, geometry)
        self._grid(canvas, geometry, ticks, y_position, 2)
        xs = self._x_positions(run, geometry)
        for field, color, dash in (
            ("reference_y_m", self.palette.orange, (5, 3)),
            ("front_y_m", self.palette.magenta, ()),
        ):
            points = [(x, y_position(getattr(value, field)))
                      for x, value in zip(xs, run.samples)]
            self._draw_series(canvas, points, geometry, fill=color,
                              width=3 if field == "front_y_m" else 2, dash=dash)
        self._events(canvas, run, geometry)
        selected_y = y_position(sample.front_y_m)
        if (geometry.left <= xs[index] <= geometry.right
                and geometry.top <= selected_y <= geometry.bottom):
            self._selection(canvas, xs[index], geometry, selected_y, self.palette.magenta)


class CurvatureChartRenderer(BaseChartRenderer):
    """Geometric NCAP curvature and closed-loop pursuit curvature."""

    def draw(self, canvas: tk.Canvas, run: SimulationRun, index: int) -> None:
        width, height, geometry = self._dimensions(canvas)
        self._background(canvas, width, height)
        sample = run.samples[index]
        canvas.create_text(16, 12, anchor="nw", fill=self.palette.foreground,
                           font=ui_font(TYPE_SCALE.body, "bold"),
                           text="CURVATURE [1/km]")
        canvas.create_text(16, 34, anchor="nw", fill=self.palette.muted,
                           font=ui_font(TYPE_SCALE.label),
                           text=(f"Reference {sample.reference_curvature_per_m * 1000:+.3f}"
                                 f" · Pursuit {sample.profile_curvature_per_m * 1000:+.3f}"))
        for x, label, color, dash in (
            (16, "NCAP reference", self.palette.orange, (2, 4)),
            (164, "Pursuit command", self.palette.cyan, ()),
        ):
            canvas.create_line(x, 64, x + 16, 64, fill=color, width=3, dash=dash)
            canvas.create_text(x + 21, 64, anchor="w", fill=self.palette.muted,
                               font=ui_font(TYPE_SCALE.caption), text=label)
        values = [getattr(value, field) * 1000 for value in run.samples
                  for field in ("reference_curvature_per_m", "profile_curvature_per_m")]
        raw_minimum, raw_maximum = min(0.0, min(values)), max(0.0, max(values))
        if raw_maximum - raw_minimum < 0.1:
            raw_maximum = raw_minimum + 0.1
        full_minimum, full_maximum, _ = self._linear_axis(raw_minimum, raw_maximum)
        minimum, maximum, ticks = self._visible_y_axis(full_minimum, full_maximum)

        def y_position(value: float) -> float:
            return geometry.bottom - ((geometry.bottom - geometry.top)
                                      * (value - minimum) / (maximum - minimum))

        self._time_grid(canvas, run, geometry)
        self._grid(canvas, geometry, ticks, y_position, 2)
        xs = self._x_positions(run, geometry)
        for field, color, dash in (
            ("reference_curvature_per_m", self.palette.orange, (2, 4)),
            ("profile_curvature_per_m", self.palette.cyan, ()),
        ):
            points = [(x, y_position(getattr(value, field) * 1000))
                      for x, value in zip(xs, run.samples)]
            self._draw_series(canvas, points, geometry, fill=color, width=2, dash=dash)
        self._events(canvas, run, geometry)
        selected_y = y_position(sample.profile_curvature_per_m * 1000)
        if (geometry.left <= xs[index] <= geometry.right
                and geometry.top <= selected_y <= geometry.bottom):
            self._selection(canvas, xs[index], geometry, selected_y, self.palette.cyan)


class SteeringChartRenderer:
    """Compatibility facade that retains the original combined chart API."""

    def __init__(self, palette: UiPalette | None = None) -> None:
        self.steering = SteeringAngleChartRenderer(palette)
        self.velocity = LateralVelocityChartRenderer(palette)

    def set_palette(self, palette: UiPalette) -> None:
        self.steering.set_palette(palette)
        self.velocity.set_palette(palette)

    def draw(self, canvas: tk.Canvas, run: SimulationRun, index: int) -> None:
        canvas.delete("all")
        height = max(canvas.winfo_height(), 380)
        split = height / 2
        self.steering.draw(_CanvasRegion(canvas, 0, split), run, index)
        self.velocity.draw(_CanvasRegion(canvas, split, height - split), run, index)


class _CanvasRegion:
    """Canvas-compatible vertical viewport used by the legacy combined chart."""

    def __init__(self, canvas: tk.Canvas, offset_y: float, height: float) -> None:
        self.canvas, self.offset_y, self.height = canvas, offset_y, height

    def winfo_width(self) -> int:
        return self.canvas.winfo_width()

    def winfo_height(self) -> int:
        return round(self.height)

    def delete(self, *_args: object) -> None:
        return None

    def _coordinates(self, args: tuple[object, ...]) -> list[object]:
        return [value + self.offset_y if index % 2 else value
                for index, value in enumerate(args)]

    def create_line(self, *args: object, **kwargs: object) -> object:
        return self.canvas.create_line(*self._coordinates(args), **kwargs)

    def create_rectangle(self, *args: object, **kwargs: object) -> object:
        return self.canvas.create_rectangle(*self._coordinates(args), **kwargs)

    def create_oval(self, *args: object, **kwargs: object) -> object:
        return self.canvas.create_oval(*self._coordinates(args), **kwargs)

    def create_text(self, x: float, y: float, **kwargs: object) -> object:
        return self.canvas.create_text(x, y + self.offset_y, **kwargs)
