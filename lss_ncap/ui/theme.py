"""Application-wide light/dark palettes and ttk style configuration."""
from __future__ import annotations

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
        background="#f3f6f9", panel="#ffffff", panel_alt="#edf2f6",
        foreground="#172531", muted="#5f7382", border="#d2dce4", grid="#dbe3e9",
        accent="#2678d4", accent_hover="#438de0", accent_text="#315fba",
        cyan="#007f92", green="#087f4f", orange="#c65318",
        yellow="#936600", magenta="#a91b98"),
}

BACKGROUND = PALETTES[ThemeMode.DARK].background
PANEL = PALETTES[ThemeMode.DARK].panel
FOREGROUND = PALETTES[ThemeMode.DARK].foreground
MUTED = PALETTES[ThemeMode.DARK].muted
ACCENT = PALETTES[ThemeMode.DARK].accent


def palette_for(mode: ThemeMode | str) -> UiPalette:
    return PALETTES[ThemeMode(mode)]


def apply_theme(root: tk.Tk, mode: ThemeMode | str = ThemeMode.DARK) -> UiPalette:
    """Apply a complete palette and return it for canvas-based components."""
    palette = palette_for(mode)
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
                    foreground=palette.muted, font=ui_font(TYPE_SCALE.label))
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
    style.configure("Vehicle.TEntry", fieldbackground=palette.panel_alt,
                    foreground=palette.foreground, bordercolor=palette.border,
                    insertcolor=palette.foreground, padding=(8, 6),
                    font=ui_font(TYPE_SCALE.body, "bold"))
    style.map("Vehicle.TEntry", bordercolor=[("focus", palette.accent)])
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
