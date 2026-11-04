"""Application-wide light/dark palettes and ttk style configuration."""
from __future__ import annotations

import math
import tkinter as tk
from dataclasses import dataclass
from enum import Enum
from tkinter import ttk

from .typography import TYPE_SCALE, configure_typography, ui_font


class ThemeMode(str, Enum):
    DARK = "dark"
    LIGHT = "light"


@dataclass(frozen=True, slots=True)
class UiPalette:
    background: str
    panel: str
    panel_alt: str
    foreground: str
    muted: str
    border: str
    grid: str
    accent: str = "#4e95e8"
    accent_hover: str = "#6aa9ef"
    accent_text: str = "#4e95e8"
    cyan: str = "#2aa9bc"
    green: str = "#35a879"
    orange: str = "#d97941"
    yellow: str = "#d19a00"
    magenta: str = "#c13ab0"


PALETTES = {
    ThemeMode.DARK: UiPalette(
        background="#0d0f1c", panel="#171a2a", panel_alt="#22263a",
        foreground="#f0f0f7", muted="#a8a9bd", border="#373b59", grid="#3a3d57",
        accent="#6d61d5", accent_hover="#7467dc", accent_text="#b3aaff",
        cyan="#22d3ee", green="#42d392", orange="#ff8a4c",
        yellow="#ffd166", magenta="#f05bd8"),
    ThemeMode.LIGHT: UiPalette(
        background="#e9eef3", panel="#ffffff", panel_alt="#e2eaf1",
        foreground="#0b1f33", muted="#40566b", border="#aebdca", grid="#c5d0da",
        accent="#006fbe", accent_hover="#005b9f", accent_text="#005a9e",
        cyan="#006f91", green="#08783f", orange="#c4480b",
        yellow="#855b00", magenta="#9c178c"),
}

BACKGROUND = PALETTES[ThemeMode.DARK].background
PANEL = PALETTES[ThemeMode.DARK].panel
FOREGROUND = PALETTES[ThemeMode.DARK].foreground
MUTED = PALETTES[ThemeMode.DARK].muted
ACCENT = PALETTES[ThemeMode.DARK].accent


def palette_for(mode: ThemeMode | str) -> UiPalette:
    return PALETTES[ThemeMode(mode)]


def _blend(first: str, second: str, amount: float) -> str:
    channels = zip((int(first[i:i + 2], 16) for i in (1, 3, 5)),
                   (int(second[i:i + 2], 16) for i in (1, 3, 5)))
    return "#" + "".join(f"{round(a + (b - a) * amount):02x}"
                         for a, b in channels)


def _entry_image(root: tk.Tk, palette: UiPalette, focus: float) -> tk.PhotoImage:
    """Draw a scalable rounded field with a soft border and focus halo."""
    size = 14
    image = tk.PhotoImage(master=root, width=size, height=size)
    border = _blend(palette.border, palette.accent, focus * 0.75)
    border = _blend(palette.panel_alt, border, 0.68)
    halo = _blend(palette.panel_alt, palette.accent, focus * 0.25)
    center = (size - 1) / 2
    for y in range(size):
        row = []
        for x in range(size):
            dx = abs(x - center) - 2
            dy = abs(y - center) - 2
            distance = (math.hypot(max(dx, 0), max(dy, 0))
                        + min(max(dx, dy), 0) - 4.5)
            if distance > 1.2:
                row.append("")
            elif distance > 0.1:
                row.append(halo if focus else "")
            elif distance > -1.1:
                row.append(border)
            else:
                row.append(palette.panel_alt)
        for x, color in enumerate(row):
            if color:
                image.put(color, to=(x, y))
    return image


def _configure_numeric_entries(root: tk.Tk, style: ttk.Style,
                               mode: ThemeMode, palette: UiPalette) -> None:
    images = getattr(root, "_numeric_entry_images", {})
    if mode not in images:
        images[mode] = tuple(_entry_image(root, palette, step / 4)
                             for step in range(5))
        root._numeric_entry_images = images
    for step, image in enumerate(images[mode]):
        element = f"VehicleRounded{mode.value}{step}.field"
        if element not in style.element_names():
            style.element_create(element, "image", image, border=4,
                                 sticky="nswe")
        name = "Vehicle.TEntry" if step == 0 else f"VehicleFocus{step}.TEntry"
        style.layout(name, [(element, {"sticky": "nswe", "children": [
            ("Entry.padding", {"sticky": "nswe", "children": [
                ("Entry.textarea", {"sticky": "nswe"})]})]})])
        style.configure(name, fieldbackground=palette.panel_alt,
                        foreground=palette.foreground,
                        insertcolor=palette.foreground, padding=(6, 4),
                        font=ui_font(TYPE_SCALE.body, "bold"))


def apply_theme(root: tk.Tk, mode: ThemeMode | str = ThemeMode.DARK) -> UiPalette:
    """Apply a complete palette and return it for canvas-based components."""
    selected_mode = ThemeMode(mode)
    palette = palette_for(selected_mode)
    configure_typography(root)
    root.configure(bg=palette.background)
    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure("Dashboard.TFrame", background=palette.background)
    style.configure("Panel.TFrame", background=palette.panel)
    style.configure("Card.TFrame", background=palette.panel,
                    bordercolor=palette.border, borderwidth=1, relief="solid")
    style.configure("Dashboard.TLabel", background=palette.background,
                    foreground=palette.foreground, font=ui_font(TYPE_SCALE.body))
    style.configure("Title.TLabel", background=palette.background,
                    foreground=palette.foreground,
                    font=ui_font(TYPE_SCALE.title, "bold"))
    style.configure("Section.TLabel", background=palette.panel,
                    foreground=palette.foreground,
                    font=ui_font(TYPE_SCALE.section, "bold"))
    style.configure("TelemetryTitle.TLabel", background=palette.panel,
                    foreground=palette.foreground,
                    font=ui_font(TYPE_SCALE.telemetry, "bold"))
    style.configure("TelemetryValue.TLabel", background=palette.panel,
                    foreground=palette.muted, font=ui_font(TYPE_SCALE.label))
    style.configure("TelemetrySteering.TLabel", background=palette.panel,
                    foreground=palette.orange, font=ui_font(TYPE_SCALE.label))
    style.configure("TelemetryMotion.TLabel", background=palette.panel,
                    foreground=palette.cyan,
                    font=ui_font(TYPE_SCALE.label, "bold"))
    style.configure("Collision.TLabel", background=palette.panel,
                    foreground="#e24f4f",
                    font=ui_font(TYPE_SCALE.telemetry, "bold"))
    style.configure("Muted.TLabel", background=palette.background,
                    foreground=palette.muted, font=ui_font(TYPE_SCALE.label))
    style.configure("Panel.TLabel", background=palette.panel,
                    foreground=(palette.foreground if selected_mode == ThemeMode.LIGHT
                                else palette.muted),
                    font=ui_font(TYPE_SCALE.label))
    style.configure("VehicleShell.TFrame", background=palette.panel,
                    bordercolor=palette.border, borderwidth=1, relief="solid")
    style.configure("VehiclePlain.TFrame", background=palette.panel)
    style.configure("VehicleInput.TFrame", background=palette.panel_alt)
    style.configure("VehicleTitle.TLabel", background=palette.panel,
                    foreground=palette.foreground,
                    font=ui_font(TYPE_SCALE.section, "bold"))
    style.configure("VehicleField.TLabel", background=palette.panel,
                    foreground=palette.muted, font=ui_font(TYPE_SCALE.caption))
    style.configure("VehicleUnit.TLabel", background=palette.panel_alt,
                    foreground=palette.accent_text,
                    font=ui_font(TYPE_SCALE.caption, "bold"))
    _configure_numeric_entries(root, style, selected_mode, palette)
    style.configure("Dashboard.TButton", background=palette.panel_alt,
                    foreground=palette.foreground, borderwidth=0, padding=(12, 7),
                    font=ui_font(TYPE_SCALE.label, "bold"))
    style.map("Dashboard.TButton", background=[("active", palette.border)])
    style.configure("Accent.TButton", background=palette.accent, foreground="#ffffff",
                    borderwidth=0, padding=(15, 7),
                    font=ui_font(TYPE_SCALE.label, "bold"))
    style.map("Accent.TButton", background=[("active", palette.accent_hover)])
    style.configure("Dashboard.TCombobox", fieldbackground=palette.panel_alt,
                    background=palette.panel_alt, foreground=palette.foreground,
                    arrowcolor=palette.foreground, bordercolor=palette.border,
                    borderwidth=1, padding=5, font=ui_font(TYPE_SCALE.label))
    style.map("Dashboard.TCombobox",
              fieldbackground=[("disabled", palette.panel),
                               ("readonly", palette.panel_alt)],
              foreground=[("disabled", palette.muted),
                          ("readonly", palette.foreground)])
    style.configure("Dashboard.Horizontal.TScale", background=palette.panel,
                    troughcolor=palette.border)
    style.configure("Dashboard.Horizontal.TProgressbar",
                    background=palette.accent, troughcolor=palette.panel_alt,
                    borderwidth=0)
    return palette
