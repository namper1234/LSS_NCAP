"""Reusable mouse interaction state for canvas charts."""
from __future__ import annotations

import tkinter as tk
from dataclasses import dataclass
from typing import Callable


@dataclass(slots=True)
class ChartView:
    x_min: float = 0.0
    x_max: float = 1.0
    y_min: float = 0.0
    y_max: float = 1.0

    def reset(self) -> None:
        self.x_min, self.x_max = 0.0, 1.0
        self.y_min, self.y_max = 0.0, 1.0

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


class ChartInteractionController:
    """Bind wheel zooming and click-drag panning to one chart canvas."""

    def __init__(self, canvas: tk.Canvas, view: ChartView,
                 redraw: Callable[[], None]) -> None:
        self.canvas, self.view, self.redraw = canvas, view, redraw
        self._drag_origin: tuple[int, int] | None = None
        canvas.bind("<MouseWheel>", self._wheel)
        canvas.bind("<Button-4>", lambda event: self._wheel_direction(event, True))
        canvas.bind("<Button-5>", lambda event: self._wheel_direction(event, False))
        canvas.bind("<ButtonPress-1>", self._drag_start)
        canvas.bind("<B1-Motion>", self._drag_move)
        canvas.bind("<ButtonRelease-1>", self._drag_end)
        canvas.configure(cursor="crosshair")

    def _wheel(self, event: tk.Event[tk.Misc]) -> str:
        return self._wheel_direction(event, event.delta > 0)

    def _wheel_direction(self, event: tk.Event[tk.Misc], zoom_in: bool) -> str:
        width, height = max(1, self.canvas.winfo_width()), max(1, self.canvas.winfo_height())
        self.view.zoom(event.x / width, event.y / height, zoom_in)
        self.redraw()
        return "break"

    def _drag_start(self, event: tk.Event[tk.Misc]) -> None:
        self._drag_origin = event.x, event.y
        self.canvas.configure(cursor="fleur")

    def _drag_move(self, event: tk.Event[tk.Misc]) -> None:
        if self._drag_origin is None:
            return
        old_x, old_y = self._drag_origin
        width, height = max(1, self.canvas.winfo_width()), max(1, self.canvas.winfo_height())
        self.view.pan((event.x - old_x) / width, (event.y - old_y) / height)
        self._drag_origin = event.x, event.y
        self.redraw()

    def _drag_end(self, _event: tk.Event[tk.Misc]) -> None:
        self._drag_origin = None
        self.canvas.configure(cursor="crosshair")
