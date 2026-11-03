"""Compact, theme-aware vehicle parameter dashboard."""
from __future__ import annotations

import tkinter as tk
from collections.abc import Callable, Mapping
from tkinter import ttk


class VehicleParametersPanel(ttk.Frame):
    """One aligned parameter area with consistent units and edit behavior."""

    def __init__(self, parent: tk.Misc, variables: Mapping[str, tk.StringVar],
                 on_change: Callable[[], None]) -> None:
        super().__init__(parent, padding=(14, 11), style="VehicleShell.TFrame")
        self._variables = variables
        self._on_change = on_change
        self._compact = False
        self._field_widgets: list[tuple[ttk.Label, ttk.Frame]] = []
        self._build()

    def _build(self) -> None:
        header = ttk.Frame(self, style="VehiclePlain.TFrame")
        self._header = header
        header.pack(fill="x", pady=(0, 9))
        ttk.Label(header, text="VEHICLE CONFIGURATION",
                  style="VehicleTitle.TLabel").pack(side="left")

        fields = ttk.Frame(self, style="VehiclePlain.TFrame")
        self._fields = fields
        fields.pack(fill="x")
        for column, (key, label, unit) in enumerate((
                ("wheelbase", "Wheelbase", "m"),
                ("length", "Vehicle length", "m"),
                ("width", "Vehicle width", "m"),
                ("steer_ratio", "Steering ratio", ":1"))):
            fields.columnconfigure(column, weight=1, uniform="vehicle")
            self._field(fields, column, key, label, unit)

    def _field(self, parent: ttk.Frame, column: int, key: str,
               label: str, unit: str) -> None:
        cell = ttk.Frame(parent, style="VehiclePlain.TFrame")
        cell.grid(row=0, column=column, sticky="ew",
                  padx=(0, 12) if column < 3 else (0, 0))
        field_label = ttk.Label(cell, text=label, style="VehicleField.TLabel")
        field_label.pack(anchor="w", pady=(0, 4))
        value = ttk.Frame(cell, style="VehicleInput.TFrame")
        value.pack(fill="x")
        self._field_widgets.append((field_label, value))
        entry = ttk.Entry(value, textvariable=self._variables[key],
                          style="Vehicle.TEntry", width=8)
        entry.pack(side="left", fill="x", expand=True)
        ttk.Label(value, text=unit, style="VehicleUnit.TLabel").pack(
            side="right", padx=(5, 8))
        entry.bind("<Return>", lambda _event: self._on_change())
        entry.bind("<FocusOut>", lambda _event: self._on_change())

    def set_compact(self, compact: bool) -> None:
        """Keep vehicle editing available while leaving room for the road."""
        if compact == self._compact:
            return
        self._compact = compact
        self.configure(padding=(10, 6) if compact else (14, 11))
        for label, value in self._field_widgets:
            label.pack_forget()
            value.pack_forget()
            if compact:
                label.pack(side="left", padx=(0, 5))
                value.pack(side="left", fill="x", expand=True)
            else:
                label.pack(anchor="w", pady=(0, 4))
                value.pack(fill="x")
        if compact:
            self._header.pack_forget()
        else:
            self._header.pack(fill="x", pady=(0, 9),
                              before=self._fields)
