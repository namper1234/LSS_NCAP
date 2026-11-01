"""Reusable mouse interaction state for canvas charts."""
from __future__ import annotations

import tkinter as tk
from dataclasses import dataclass, field
from typing import Callable, Protocol

from .theme import UiPalette


@dataclass(slots=True)
class ChartView:
    x_min: float = 0.0
    x_max: float = 1.0
    y_min: float = 0.0
    y_max: float = 1.0
    hidden_series: set[str] = field(default_factory=set)

    def reset(self) -> None:
        """Auto-fit the axes while retaining signal visibility."""
        self.x_min, self.x_max = 0.0, 1.0
        self.y_min, self.y_max = 0.0, 1.0

    def is_visible(self, field_name: str) -> bool:
        return field_name not in self.hidden_series

    def toggle_series(self, field_name: str) -> None:
        if field_name in self.hidden_series:
            self.hidden_series.remove(field_name)
        else:
            self.hidden_series.add(field_name)

    def ensure_x_visible(self, fraction: float) -> None:
        """Follow a selected time without changing the current zoom span."""
        fraction = max(0.0, min(1.0, fraction))
        span = self.x_max - self.x_min
        if fraction < self.x_min:
            self.x_min, self.x_max = self._bounded(fraction, fraction + span)
        elif fraction > self.x_max:
            self.x_min, self.x_max = self._bounded(fraction - span, fraction)

    def box_zoom(self, x0: float, y0: float, x1: float, y1: float) -> bool:
        """Zoom to plot-relative fractions; canvas Y increases downward."""
        left, right = sorted((max(0.0, min(1.0, x0)),
                              max(0.0, min(1.0, x1))))
        top, bottom = sorted((max(0.0, min(1.0, y0)),
                              max(0.0, min(1.0, y1))))
        if right - left < 0.02 or bottom - top < 0.02:
            return False
        x_min, x_span = self.x_min, self.x_max - self.x_min
        y_min, y_span = self.y_min, self.y_max - self.y_min
        self.x_min, self.x_max = x_min + left * x_span, x_min + right * x_span
        self.y_min, self.y_max = y_min + (1 - bottom) * y_span, y_min + (1 - top) * y_span
        return True

    @staticmethod
    def _bounded(start: float, end: float) -> tuple[float, float]:
        span = end - start
        if start < 0:
            return 0.0, span
        if end > 1:
            return 1.0 - span, 1.0
        return start, end

    def zoom(self, anchor_x: float, anchor_y: float, zoom_in: bool) -> None:
        factor = 0.8 if zoom_in else 1.25
        x_span = min(1.0, max(0.02, (self.x_max - self.x_min) * factor))
        y_span = min(1.0, max(0.02, (self.y_max - self.y_min) * factor))
        anchor_x = max(0.0, min(1.0, anchor_x))
        anchor_y = max(0.0, min(1.0, anchor_y))
        x_value = self.x_min + anchor_x * (self.x_max - self.x_min)
        # Canvas Y grows downward, while normalized plot Y grows upward.
        y_value = self.y_min + (1.0 - anchor_y) * (self.y_max - self.y_min)
        self.x_min, self.x_max = self._bounded(
            x_value - anchor_x * x_span, x_value + (1.0 - anchor_x) * x_span)
        self.y_min, self.y_max = self._bounded(
            y_value - (1.0 - anchor_y) * y_span, y_value + anchor_y * y_span)

    def pan(self, dx_fraction: float, dy_fraction: float) -> None:
        x_span, y_span = self.x_max - self.x_min, self.y_max - self.y_min
        self.x_min, self.x_max = self._bounded(
            self.x_min - dx_fraction * x_span,
            self.x_max - dx_fraction * x_span)
        self.y_min, self.y_max = self._bounded(
            self.y_min + dy_fraction * y_span,
            self.y_max + dy_fraction * y_span)


class ChartInteractionTarget(Protocol):
    palette: UiPalette

    def plot_bounds(self) -> tuple[float, float, float, float]: ...

    def legend_at(self, x: int, y: int) -> str | None: ...


class ChartInteractionController:
    """Wheel zoom, drag pan, box zoom, and independent legend toggles."""

    def __init__(self, canvas: tk.Canvas, view: ChartView,
                 redraw: Callable[[], None], target: ChartInteractionTarget) -> None:
        self.canvas, self.view, self.redraw, self.target = canvas, view, redraw, target
        self._drag_origin: tuple[int, int] | None = None
        self._mode: str | None = None
        canvas.bind("<MouseWheel>", self._wheel)
        canvas.bind("<Button-4>", lambda event: self._wheel_direction(event, True))
        canvas.bind("<Button-5>", lambda event: self._wheel_direction(event, False))
        canvas.bind("<ButtonPress-1>", self._drag_start)
        canvas.bind("<B1-Motion>", self._drag_move)
        canvas.bind("<ButtonRelease-1>", self._drag_end)
        canvas.bind("<ButtonPress-3>", self._box_start)
        canvas.bind("<B3-Motion>", self._drag_move)
        canvas.bind("<ButtonRelease-3>", self._drag_end)
        canvas.bind("<Motion>", self._hover)
        canvas.bind("<Leave>", lambda _event: self.canvas.configure(cursor="crosshair"))
        canvas.configure(cursor="crosshair")

    def _wheel(self, event: tk.Event[tk.Misc]) -> str:
        return self._wheel_direction(event, event.delta > 0)

    def _wheel_direction(self, event: tk.Event[tk.Misc], zoom_in: bool) -> str:
        left, top, right, bottom = self.target.plot_bounds()
        if not left <= event.x <= right or not top <= event.y <= bottom:
            return "break"
        self.view.zoom((event.x - left) / (right - left),
                       (event.y - top) / (bottom - top), zoom_in)
        self.redraw()
        return "break"

    def _drag_start(self, event: tk.Event[tk.Misc]) -> None:
        field_name = self.target.legend_at(event.x, event.y)
        if field_name is not None:
            self.view.toggle_series(field_name)
            self.redraw()
            return
        if not self._in_plot(event.x, event.y):
            return
        self._drag_origin = event.x, event.y
        self._mode = "box" if event.state & 0x0001 else "pan"
        self.canvas.configure(cursor="crosshair" if self._mode == "box" else "fleur")

    def _box_start(self, event: tk.Event[tk.Misc]) -> None:
        if self._in_plot(event.x, event.y):
            self._drag_origin = event.x, event.y
            self._mode = "box"

    def _in_plot(self, x: int, y: int) -> bool:
        left, top, right, bottom = self.target.plot_bounds()
        return left <= x <= right and top <= y <= bottom

    def _clamp_to_plot(self, x: int, y: int) -> tuple[float, float]:
        left, top, right, bottom = self.target.plot_bounds()
        return max(left, min(right, x)), max(top, min(bottom, y))

    def _drag_move(self, event: tk.Event[tk.Misc]) -> None:
        if self._drag_origin is None:
            return
        if self._mode == "box":
            x0, y0 = self._clamp_to_plot(*self._drag_origin)
            x1, y1 = self._clamp_to_plot(event.x, event.y)
            self.canvas.delete("chart-selection")
            self.canvas.create_rectangle(
                x0, y0, x1, y1, outline=self.target.palette.accent_text,
                width=2, dash=(4, 3), tags="chart-selection")
        else:
            old_x, old_y = self._drag_origin
            left, top, right, bottom = self.target.plot_bounds()
            self.view.pan((event.x - old_x) / (right - left),
                          (event.y - old_y) / (bottom - top))
            self._drag_origin = event.x, event.y
            self.redraw()

    def _drag_end(self, event: tk.Event[tk.Misc]) -> None:
        if self._drag_origin is not None and self._mode == "box":
            x0, y0 = self._clamp_to_plot(*self._drag_origin)
            x1, y1 = self._clamp_to_plot(event.x, event.y)
            left, top, right, bottom = self.target.plot_bounds()
            if self.view.box_zoom((x0 - left) / (right - left),
                                  (y0 - top) / (bottom - top),
                                  (x1 - left) / (right - left),
                                  (y1 - top) / (bottom - top)):
                self.redraw()
            self.canvas.delete("chart-selection")
        self._drag_origin = None
        self._mode = None
        self.canvas.configure(cursor="crosshair")

    def _hover(self, event: tk.Event[tk.Misc]) -> None:
        if self._mode is not None:
            return
        self.canvas.configure(cursor=("hand2" if self.target.legend_at(event.x, event.y)
                                      is not None else "crosshair"))
