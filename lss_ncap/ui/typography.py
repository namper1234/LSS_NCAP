"""Central typography scale with Inter-first platform font resolution."""
from __future__ import annotations

import tkinter as tk
import tkinter.font as tkfont
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class TypeScale:
    caption: int = 8
    label: int = 9
    body: int = 10
    section: int = 11
    telemetry: int = 12
    title: int = 20


TYPE_SCALE = TypeScale()
_font_family = "Inter"


def configure_typography(root: tk.Misc) -> str:
    """Select Inter when available and a metrically clean fallback otherwise."""
    global _font_family
    available = set(tkfont.families(root))
    _font_family = next((candidate for candidate in (
        "Inter", "Inter Display", "Segoe UI", "SF Pro Display",
        "Helvetica Neue", "Noto Sans", "DejaVu Sans",
    ) if candidate in available), "TkDefaultFont")
    root.option_add("*Font", ui_font(TYPE_SCALE.body))
    root.option_add("*TCombobox*Listbox.font", ui_font(TYPE_SCALE.label))
    return _font_family


def ui_font(size: int, weight: str = "normal") -> tuple[str, int, str]:
    return _font_family, size, weight


def active_font_family() -> str:
    return _font_family
