"""Reusable polished canvas widgets for the desktop dashboard."""
from __future__ import annotations

import tkinter as tk
import tkinter.font as tkfont
from collections.abc import Callable

from .theme import UiPalette
from .typography import TYPE_SCALE, ui_font


class ModernButton(tk.Canvas):
    """Rounded, theme-aware button with hover, press, and keyboard states."""

    def __init__(self, parent: tk.Misc, text: str, command: Callable[[], None],
                 palette: UiPalette, *, accent: bool = False,
                 dashboard_surface: bool = False) -> None:
        self._text, self._command = text, command
        self._palette, self._accent = palette, accent
        self._dashboard_surface = dashboard_surface
        self._hovered = self._pressed = False
        self._font = tkfont.Font(parent, font=ui_font(TYPE_SCALE.label, "bold"))
        width = self._font.measure(text) + 32
        super().__init__(parent, width=width, height=36, highlightthickness=0,
                         bd=0, takefocus=True, cursor="hand2")
        self.bind("<Enter>", self._enter)
        self.bind("<Leave>", self._leave)
        self.bind("<ButtonPress-1>", self._press)
        self.bind("<ButtonRelease-1>", self._release)
        self.bind("<Key-space>", self._keyboard_activate)
        self.bind("<Key-Return>", self._keyboard_activate)
        self.bind("<FocusIn>", lambda _event: self._draw())
        self.bind("<FocusOut>", lambda _event: self._draw())
        self._draw()

    @staticmethod
    def _rounded_points(x1: float, y1: float, x2: float, y2: float,
                        radius: float) -> tuple[float, ...]:
        return (x1 + radius, y1, x2 - radius, y1, x2, y1, x2, y1 + radius,
                x2, y2 - radius, x2, y2, x2 - radius, y2, x1 + radius, y2,
                x1, y2, x1, y2 - radius, x1, y1 + radius, x1, y1)

    def _draw(self) -> None:
        self.delete("all")
        surface = (self._palette.background if self._dashboard_surface
                   else self._palette.panel)
        self.configure(bg=surface)
        width, height = int(self["width"]), int(self["height"])
        if self._accent:
            fill = self._palette.accent
            if self._hovered:
                fill = self._palette.accent_hover
            foreground, border = "#ffffff", fill
        else:
            fill = self._palette.border if self._hovered else self._palette.panel_alt
            foreground, border = self._palette.foreground, self._palette.border
        if self._pressed:
            fill = self._palette.border
        # One-pixel shadow and restrained rounded geometry mirror the reference.
        self.create_polygon(self._rounded_points(3, 4, width - 2, height - 1, 10),
                            smooth=True, splinesteps=18,
                            fill=self._palette.background, outline="")
        focus_color = self._palette.accent_text if self.focus_get() is self else border
        self.create_polygon(self._rounded_points(2, 2, width - 2, height - 3, 10),
                            smooth=True, splinesteps=18, fill=fill,
                            outline=focus_color, width=1)
        self.create_text(width / 2, height / 2 - 1, text=self._text,
                         fill=foreground, font=self._font)

    def set_palette(self, palette: UiPalette) -> None:
        self._palette = palette
        self._draw()

    def set_text(self, text: str) -> None:
        self._text = text
        self.configure(width=self._font.measure(text) + 32)
        self._draw()

    def _enter(self, _event: tk.Event[tk.Misc]) -> None:
        self._hovered = True
        self._draw()

    def _leave(self, _event: tk.Event[tk.Misc]) -> None:
        self._hovered = self._pressed = False
        self._draw()

    def _press(self, _event: tk.Event[tk.Misc]) -> None:
        self.focus_set()
        self._pressed = True
        self._draw()

    def _release(self, event: tk.Event[tk.Misc]) -> None:
        was_pressed = self._pressed
        self._pressed = False
        self._hovered = 0 <= event.x <= int(self["width"]) and 0 <= event.y <= int(self["height"])
        self._draw()
        if was_pressed and self._hovered:
            self._command()

    def _keyboard_activate(self, _event: tk.Event[tk.Misc]) -> str:
        self._command()
        return "break"


class ExpandButton(tk.Canvas):
    """Compact vector expand/restore control with a soft hover transition."""

    def __init__(self, parent: tk.Misc, command: Callable[[], None],
                 palette: UiPalette) -> None:
        self._command = command
        self._palette = palette
        self._expanded = False
        self._pressed = False
        self._hover = self._hover_target = 0.0
        self._animation: str | None = None
        super().__init__(parent, width=88, height=32, highlightthickness=0,
                         bd=0, takefocus=True, cursor="hand2")
        self.bind("<Enter>", lambda _event: self._set_hover(True))
        self.bind("<Leave>", lambda _event: self._set_hover(False))
        self.bind("<ButtonPress-1>", self._press)
        self.bind("<ButtonRelease-1>", self._release)
        self.bind("<Key-space>", self._keyboard_activate)
        self.bind("<Key-Return>", self._keyboard_activate)
        self.bind("<FocusIn>", lambda _event: self._draw())
        self.bind("<FocusOut>", lambda _event: self._draw())
        self._draw()

    @property
    def label(self) -> str:
        return "Restore" if self._expanded else "Expand"

    def set_expanded(self, expanded: bool) -> None:
        self._expanded = expanded
        self._draw()

    def set_palette(self, palette: UiPalette) -> None:
        self._palette = palette
        self._draw()

    def invoke(self) -> None:
        self._command()

    @staticmethod
    def _blend(start: str, end: str, amount: float) -> str:
        first = tuple(int(start[index:index + 2], 16) for index in (1, 3, 5))
        last = tuple(int(end[index:index + 2], 16) for index in (1, 3, 5))
        return "#" + "".join(f"{round(a + (b - a) * amount):02x}"
                             for a, b in zip(first, last))

    def _draw(self) -> None:
        self.delete("all")
        self.configure(bg=self._palette.panel)
        fill = self._blend(self._palette.panel_alt, self._palette.border,
                           max(self._hover, 0.45 if self._pressed else 0.0))
        stroke = self._blend(self._palette.muted, self._palette.accent_text,
                             self._hover)
        border = (self._palette.accent_text if self.focus_get() is self
                  else self._palette.border)
        self.create_polygon(ModernButton._rounded_points(1, 1, 87, 31, 8),
                            smooth=True, splinesteps=16, fill=fill,
                            outline=border, width=1)
        if self._expanded:
            paths = ((8, 8, 15, 15, 15, 10), (15, 15, 10, 15),
                     (27, 25, 20, 18, 20, 23), (20, 18, 25, 18))
        else:
            paths = ((15, 15, 8, 8, 8, 13), (8, 8, 13, 8),
                     (20, 18, 27, 25, 27, 20), (27, 25, 22, 25))
        for path in paths:
            self.create_line(*path, fill=stroke, width=1.8,
                             capstyle="round", joinstyle="round")
        self.create_text(38, 16, anchor="w", text=self.label,
                         fill=self._palette.foreground,
                         font=ui_font(TYPE_SCALE.caption, "bold"))

    def _set_hover(self, hovered: bool) -> None:
        self._hover_target = float(hovered)
        if self._animation is None:
            self._animate()

    def _animate(self) -> None:
        self._animation = None
        self._hover += (self._hover_target - self._hover) * 0.35
        if abs(self._hover_target - self._hover) < 0.02:
            self._hover = self._hover_target
        self._draw()
        if self._hover != self._hover_target:
            self._animation = self.after(16, self._animate)

    def _press(self, _event: tk.Event[tk.Misc]) -> None:
        self.focus_set()
        self._pressed = True
        self._draw()

    def _release(self, event: tk.Event[tk.Misc]) -> None:
        was_pressed = self._pressed
        self._pressed = False
        self._draw()
        if was_pressed and 0 <= event.x <= 88 and 0 <= event.y <= 32:
            self.invoke()

    def _keyboard_activate(self, _event: tk.Event[tk.Misc]) -> str:
        self.invoke()
        return "break"


class DropdownSelect(tk.Canvas):
    """Rounded selector with a floating, theme-aware option menu."""

    def __init__(self, parent: tk.Misc, variable: tk.StringVar,
                 values: tuple[str, ...], palette: UiPalette, width_chars: int) -> None:
        self.variable, self.values, self._palette = variable, tuple(values), palette
        self._state, self._hovered = "readonly", False
        self._popup: tk.Toplevel | None = None
        self._font = tkfont.Font(parent, font=ui_font(TYPE_SCALE.label))
        width = max(76, self._font.measure("0" * width_chars) + 38)
        super().__init__(parent, width=width, height=34, highlightthickness=0,
                         bd=0, takefocus=True, cursor="hand2")
        self.bind("<Enter>", lambda _event: self._set_hover(True))
        self.bind("<Leave>", lambda _event: self._set_hover(False))
        self.bind("<Button-1>", self._toggle_popup)
        self.bind("<Key-space>", self._toggle_popup)
        self.bind("<Key-Return>", self._toggle_popup)
        self.bind("<Escape>", lambda _event: self._close_popup())
        self.bind("<FocusIn>", lambda _event: self._draw())
        self.bind("<FocusOut>", lambda _event: self._draw())
        variable.trace_add("write", lambda *_args: self._draw())
        self._draw()

    def _set_hover(self, hovered: bool) -> None:
        self._hovered = hovered
        self._draw()

    def _draw(self) -> None:
        self.delete("all")
        super().configure(bg=self._palette.panel)
        width, height = int(self["width"]), int(self["height"])
        enabled = self._state != "disabled"
        fill = self._palette.border if self._hovered and enabled else self._palette.panel_alt
        border = (self._palette.accent_text if self.focus_get() is self
                  else self._palette.border)
        foreground = self._palette.foreground if enabled else self._palette.muted
        self.create_polygon(ModernButton._rounded_points(1, 1, width - 1, height - 2, 10),
                            smooth=True, splinesteps=18, fill=fill,
                            outline=border, width=1)
        self.create_text(13, height / 2, anchor="w", text=self.variable.get(),
                         fill=foreground, font=self._font)
        # Compact chevron avoids relying on platform-specific glyph rendering.
        arrow_x = width - 15
        self.create_line(arrow_x - 4, height / 2 - 2, arrow_x, height / 2 + 2,
                         arrow_x + 4, height / 2 - 2, fill=foreground, width=2)

    def set_palette(self, palette: UiPalette) -> None:
        self._palette = palette
        self._close_popup()
        self._draw()

    def configure(self, cnf: object | None = None, **kwargs: object) -> object:
        if "values" in kwargs:
            self.values = tuple(str(value) for value in kwargs.pop("values"))  # type: ignore[arg-type]
        if "state" in kwargs:
            self._state = str(kwargs.pop("state"))
            super().configure(cursor="hand2" if self._state != "disabled" else "arrow")
        result = super().configure(cnf, **kwargs)
        if hasattr(self, "_palette"):
            self._draw()
        return result

    config = configure

    def __getitem__(self, key: str) -> object:
        if key == "values":
            return self.values
        if key == "state":
            return self._state
        return super().__getitem__(key)

    def _toggle_popup(self, _event: tk.Event[tk.Misc] | None = None) -> str:
        if self._state == "disabled":
            return "break"
        if self._popup is not None:
            self._close_popup()
            return "break"
        self.focus_set()
        popup = tk.Toplevel(self)
        self._popup = popup
        popup.withdraw()
        popup.overrideredirect(True)
        popup.configure(bg=self._palette.border)
        popup.attributes("-topmost", True)
        container = tk.Frame(popup, bg=self._palette.panel, padx=5, pady=5,
                             highlightbackground=self._palette.border,
                             highlightthickness=1)
        container.pack(fill="both", expand=True)
        menu_width = max(int(self["width"]),
                         max((self._font.measure(value) for value in self.values), default=60) + 42)
        for value in self.values:
            selected = value == self.variable.get()
            label = tk.Label(
                container, text=("✓  " if selected else "    ") + value,
                anchor="w", padx=10, pady=7, bg=self._palette.panel,
                fg=self._palette.accent_text if selected else self._palette.foreground,
                font=ui_font(TYPE_SCALE.label, "bold" if selected else "normal"),
                cursor="hand2")
            label.pack(fill="x")
            label.bind("<Enter>", lambda _event, row=label:
                       row.configure(bg=self._palette.panel_alt))
            label.bind("<Leave>", lambda _event, row=label:
                       row.configure(bg=self._palette.panel))
            label.bind("<Button-1>", lambda _event, choice=value:
                       self._select(choice))
        popup.update_idletasks()
        x, y = self.winfo_rootx(), self.winfo_rooty() + self.winfo_height() + 5
        popup.geometry(f"{menu_width}x{popup.winfo_reqheight()}+{x}+{y}")
        popup.deiconify()
        popup.lift()
        popup.bind("<Escape>", lambda _event: self._close_popup())
        popup.bind("<FocusOut>", lambda _event: self.after(80, self._close_if_unfocused))
        popup.focus_force()
        return "break"

    def _select(self, value: str) -> None:
        self.variable.set(value)
        self._close_popup()
        self.event_generate("<<ComboboxSelected>>")

    def _close_if_unfocused(self) -> None:
        if self._popup is not None and self._popup.focus_displayof() is None:
            self._close_popup()

    def _close_popup(self) -> None:
        if self._popup is not None:
            self._popup.destroy()
            self._popup = None
