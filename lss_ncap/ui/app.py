"""Interactive Tk application. No vehicle equations or drawing logic live here."""
from __future__ import annotations

import logging
import tkinter as tk
import time
from collections.abc import Callable
from dataclasses import replace
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from ..config import (ROAD, SIMULATION, RoadConfig,
                      SimulationConfig)
from ..export import export_run, export_swa_profile
from ..lookahead_tuning import TuningRequest
from ..runtime import resource_path, user_export_dir
from ..scenarios.elk import (ELK_SCENARIOS, ElkParameters, ElkScenario, LaneChangeIntent,
                             elk_definition)
from ..simulation import SimulationEngine, SimulationRun
from ..targets import (CUSTOM_TARGET_TYPE, DEFAULT_TARGET_TYPE, GVT_TYPES,
                       TargetType, target_type)
from ..vehicle_configuration import VehicleConfiguration
from ..visualization.chart import (LateralVelocityChartRenderer,
                                   LateralTrajectoryChartRenderer,
                                   SteeringAngleChartRenderer)
from ..visualization.road import RoadRenderer
from ..visualization.targets import target_thumbnail
from .playback import PlaybackClock, PlaybackState
from .chart_interaction import ChartInteractionController, ChartView
from .telemetry import telemetry_view
from .theme import ThemeMode, apply_theme
from .typography import TYPE_SCALE, ui_font
from .vehicle_panel import VehicleParametersPanel
from .widgets import DropdownSelect, ExpandButton, ModernButton, NumericEntry

LOGGER = logging.getLogger(__name__)


class SimulationApp:
    def __init__(self, initial_vlat: float = 0.4, initial_side: int = 1,
                 *, engine: SimulationEngine | None = None,
                 radius_m: float | None = None, d2_m: float | None = None,
                 road_config: RoadConfig = ROAD,
                 simulation_config: SimulationConfig = SIMULATION,
                 root: tk.Tk | None = None) -> None:
        self.engine = engine or SimulationEngine(simulation_config)
        self.road_config = road_config
        self.simulation_config = self.engine.config
        self.run_data: SimulationRun = self.engine.run_lka(
            initial_vlat, initial_side, radius_m=radius_m, d2_m=d2_m)
        self.playback = PlaybackState(len(self.run_data.samples))
        self.root = root or tk.Tk()
        self.root.title("LSS Euro NCAP")
        self._window_icon = tk.PhotoImage(
            file=str(resource_path("assets", "vinfast_app_icon.png")))
        self.root.iconphoto(True, self._window_icon)
        self.root.geometry("1280x990")
        self.root.minsize(980, 700)
        self.root.resizable(True, True)
        self.theme_mode = ThemeMode.DARK
        self.palette = apply_theme(self.root, self.theme_mode)
        self.chart_views = [ChartView() for _ in range(3)]
        self.road_renderer = RoadRenderer(road_config, self.engine.vehicle,
                                          self.engine.config, self.palette)
        self.steering_chart_renderer = SteeringAngleChartRenderer(
            self.palette, self.chart_views[0])
        self.trajectory_chart_renderer = LateralTrajectoryChartRenderer(
            self.palette, self.chart_views[1])
        self.velocity_chart_renderer = LateralVelocityChartRenderer(
            self.palette, self.chart_views[2])
        self._timer: str | None = None
        self._regenerate_timer: str | None = None
        self._layout_timer: str | None = None
        self._generation_error: str | None = None
        self.playback_clock = PlaybackClock()
        self._syncing_timeline = False
        self.vlat_var = tk.StringVar(self.root, value=f"{initial_vlat:g}")
        self.mode_var = tk.StringVar(self.root, value="LKA")
        self.elk_scenario_var = tk.StringVar(self.root, value="Road Edge")
        self.intent_var = tk.StringVar(self.root, value="Unintentional")
        self.elk_speed_kph_var = tk.StringVar(
            self.root, value=f"{self.engine.config.speed_mps * 3.6:g}")
        self.gvt_speed_kph_var = tk.StringVar(self.root, value="72")
        self.gvt_marker_distance_var = tk.StringVar(self.root, value="1.5")
        self.overlap_var = tk.StringVar(self.root, value="10")
        self.gvt_type_var = tk.StringVar(self.root, value=DEFAULT_TARGET_TYPE.name)
        self._gvt_type_key = DEFAULT_TARGET_TYPE.key
        self._confirmed_gvt_lengths: dict[str, str] = {}
        self._custom_gvt_dimensions = (
            f"{CUSTOM_TARGET_TYPE.length_m:g}", f"{CUSTOM_TARGET_TYPE.width_m:g}",
            f"{CUSTOM_TARGET_TYPE.wheelbase_m:g}")
        self.gvt_length_var = tk.StringVar(
            self.root, value=f"{DEFAULT_TARGET_TYPE.default_length_m:g}")
        self.gvt_width_var = tk.StringVar(self.root, value=f"{DEFAULT_TARGET_TYPE.width_m:g}")
        self.gvt_wheelbase_var = tk.StringVar(
            self.root, value=f"{DEFAULT_TARGET_TYPE.wheelbase_m:g}")
        self.side_var = tk.StringVar(self.root, value="Left" if initial_side == 1 else "Right")
        self.line_var = tk.StringVar(self.root, value="Dashed")
        self.lane_width_var = tk.StringVar(self.root,
                                           value=f"{road_config.default_lane_width_m:.1f}")
        self.approach_s_var = tk.StringVar(
            self.root, value=f"{self.engine.config.approach_s:g}")
        self.speed_mps_var = tk.StringVar(
            self.root, value=f"{self.engine.config.speed_mps:g}")
        self.radius_var = tk.StringVar(self.root, value=f"{self.run_data.path.radius_m:g}")
        self.d2_var = tk.StringVar(self.root, value=f"{self.run_data.path.d2:g}")
        self.wheelbase_var = tk.StringVar(self.root,
                                          value=f"{self.engine.vehicle.wheelbase_m:g}")
        self.steer_ratio_var = tk.StringVar(self.root,
                                            value=f"{self.engine.vehicle.steer_ratio:g}")
        self.length_var = tk.StringVar(self.root,
                                       value=f"{self.engine.vehicle.body_length_m:g}")
        self.width_var = tk.StringVar(self.root,
                                      value=f"{self.engine.vehicle.body_width_m:g}")
        self.timeline_var = tk.DoubleVar(self.root, value=0)
        self.timeline_inspection_var = tk.StringVar(self.root, value="")
        self.status_var = tk.StringVar(self.root)
        self.scenario_help_var = tk.StringVar(self.root)
        self.telemetry_phase_var = tk.StringVar(self.root)
        self.collision_status_var = tk.StringVar(self.root)
        self.telemetry_state_var = tk.StringVar(self.root)
        self.telemetry_steering_var = tk.StringVar(self.root)
        self.telemetry_motion_var = tk.StringVar(self.root)
        self.telemetry_target_var = tk.StringVar(self.root)
        self.modern_buttons: list[ModernButton] = []
        self.modern_selectors: list[DropdownSelect] = []
        self._expanded_panel: str | None = None
        self._expand_buttons: dict[str, ExpandButton] = {}
        self._build_ui()
        self.gvt_type_var.trace_add("write", self._gvt_type_changed)
        for variable in (self.radius_var, self.d2_var, self.vlat_var,
                         self.approach_s_var, self.speed_mps_var,
                         self.elk_speed_kph_var, self.gvt_speed_kph_var,
                         self.gvt_marker_distance_var, self.overlap_var,
                         self.gvt_length_var, self.gvt_width_var, self.gvt_wheelbase_var):
            variable.trace_add("write", self._schedule_regeneration)
        self.root.bind("<Configure>", self._fit_chart_height)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.bind("<space>", self._keyboard_playback)
        self.root.bind("<Left>", lambda _event: self._step_frame(-1))
        self.root.bind("<Right>", lambda _event: self._step_frame(1))
        self.root.bind("<Home>", lambda _event: self._jump_to_frame(0))
        self.root.bind("<End>", lambda _event: self._jump_to_frame(
            self.playback.frame_count - 1))
        self.root.bind("<Escape>", self._restore_on_escape)
        self.redraw()

    def _fit_chart_height(self, event: tk.Event | None = None) -> None:
        """Keep the road visible when desktop chrome leaves less vertical space."""
        if event is not None and event.widget is not self.root:
            return
        if event is None:
            self._layout_timer = None
        window_height = self.root.winfo_height()
        self.vehicle_panel.set_compact(window_height < 900)
        height = max(120, min(210, round(window_height * 0.17)))
        if window_height < 900 and self.traffic_settings.winfo_manager():
            height = 100
        for canvas in self.chart_canvases.values():
            if int(canvas.cget("height")) != height:
                canvas.configure(height=height)

    def _build_ui(self) -> None:
        self._build_controls()
        self._build_road_panel()
        self._build_charts_and_timeline()

    def _build_controls(self) -> None:
        controls = ttk.Frame(self.root, padding=(18, 10), style="Card.TFrame")
        self.controls_panel = controls
        controls.pack(fill="x", padx=12, pady=(10, 8))
        settings = ttk.Frame(controls, style="Panel.TFrame")
        settings.pack(fill="x")
        settings_top = ttk.Frame(settings, style="Panel.TFrame")
        settings_top.pack(fill="x")
        settings_bottom = ttk.Frame(settings, style="Panel.TFrame")
        settings_bottom.pack(fill="x", pady=(8, 0))
        settings_third = ttk.Frame(settings, style="Panel.TFrame")
        settings_third.pack(fill="x", pady=(8, 0))
        self.traffic_settings = ttk.Frame(settings, style="Panel.TFrame")
        actions = ttk.Frame(controls, style="Panel.TFrame")
        self.mode_combo = self._combo(settings_top, "Function", self.mode_var,
                                      ("LKA", "ELK"), 6)
        self.scenario_combo = self._combo(
            settings_top, "Scenario", self.elk_scenario_var,
            tuple(definition.label for definition in ELK_SCENARIOS.values()), 20)
        self.elk_speed_entry = self._numeric_entry(
            settings_top, "VUT (km/h)", self.elk_speed_kph_var)
        self.gvt_speed_entry = self._numeric_entry(
            settings_top, "GVT (km/h)", self.gvt_speed_kph_var)
        self.vlat_entry = self._numeric_entry(
            settings_bottom, "Lateral speed (m/s)", self.vlat_var)
        self.speed_entry = self._numeric_entry(
            settings_bottom, "Vehicle speed (m/s)", self.speed_mps_var)
        self.side_combo = self._combo(settings_bottom, "Departure", self.side_var,
                                      ("Left", "Right"), 7)
        self.intent_combo = self._combo(settings_bottom, "Lane change", self.intent_var,
                                        ("Unintentional", "Intentional"), 13)
        target_field = ttk.Frame(settings_bottom, style="Panel.TFrame")
        target_field.pack(side="left", padx=(0, 16))
        ttk.Label(target_field, text="GVT type", style="Panel.TLabel").pack(
            side="left", padx=(0, 6))
        self.gvt_type_combo = DropdownSelect(
            target_field, self.gvt_type_var, tuple(target.name for target in GVT_TYPES),
            self.palette, 15,
            images={target.name: target_thumbnail(target, self.root) for target in GVT_TYPES},
            option_labels={target.name: target.option_label for target in GVT_TYPES})
        self.gvt_type_combo.pack(side="left")
        self.modern_selectors.append(self.gvt_type_combo)
        self.gvt_offset_entry = self._numeric_entry(
            self.traffic_settings, "GVT centre offset (m)",
            self.gvt_marker_distance_var, stacked=True)
        self.overlap_entry = self._numeric_entry(
            self.traffic_settings, "Overlap (% VUT)", self.overlap_var, stacked=True)
        self.gvt_length_entry = self._numeric_entry(
            self.traffic_settings, "GVT Length (m)", self.gvt_length_var, stacked=True)
        self.gvt_width_entry = self._numeric_entry(
            self.traffic_settings, "GVT Width (m)", self.gvt_width_var, stacked=True)
        self.gvt_wheelbase_entry = self._numeric_entry(
            self.traffic_settings, "GVT Wheelbase (m)", self.gvt_wheelbase_var, stacked=True)
        self.line_combo = self._combo(settings_third, "Lane line", self.line_var,
                                      ("Dashed", "Solid"), 7)
        self.d2_entry = self._numeric_entry(
            settings_third, "d2 (m)", self.d2_var)
        self.radius_entry = self._numeric_entry(
            settings_third, "R (m)", self.radius_var)
        width_combo = self._combo(settings_third, "Lane width (m)", self.lane_width_var,
                                  tuple(f"{value:.1f}" for value in
                                        self.road_config.lane_width_options_m), 5)
        self.approach_entry = self._numeric_entry(
            settings_third, "Approach time (s)", self.approach_s_var)
        self.mode_combo.bind("<<ComboboxSelected>>", self._scenario_selection_changed)
        self.scenario_combo.bind("<<ComboboxSelected>>", self._scenario_selection_changed)
        self.intent_combo.bind("<<ComboboxSelected>>", self._scenario_selection_changed)
        self.side_combo.bind("<<ComboboxSelected>>", lambda _event: self.regenerate())
        self.line_combo.bind("<<ComboboxSelected>>", lambda _event: self.redraw())
        width_combo.bind("<<ComboboxSelected>>", lambda _event: self.redraw())
        self._build_vehicle_parameters(controls)
        actions.pack(fill="x", pady=(9, 0))
        self.play_button = self._modern_button(
            actions, "▶  Play", self.toggle_playback, accent=True)
        self.play_button.pack(side="left", padx=(0, 7))
        self._modern_button(actions, "↻  Reset / Generate", self.regenerate).pack(
            side="left", padx=4)
        self._modern_button(actions, "⇩  Export CSV", self.export_csv).pack(
            side="left", padx=4)
        self._modern_button(actions, "⇩  Export SWA", self.export_swa).pack(
            side="left", padx=4)
        self._modern_button(actions, "⌖  Reset View", self.reset_chart_views).pack(
            side="left", padx=4)
        ttk.Label(actions, textvariable=self.scenario_help_var,
                  style="Panel.TLabel").pack(side="left", padx=(18, 0))
        self.theme_button = self._modern_button(
            actions, "☀  Light mode", self.toggle_theme)
        self.theme_button.pack(side="right", padx=(12, 0))

    def _build_road_panel(self) -> None:
        road_panel = ttk.Frame(self.root, padding=1, style="Card.TFrame")
        self.road_panel = road_panel
        road_panel.pack(fill="both", expand=True, padx=12)
        telemetry = ttk.Frame(road_panel, padding=(14, 9), style="Panel.TFrame")
        telemetry.pack(fill="x")
        telemetry.columnconfigure(0, weight=2)
        telemetry.columnconfigure(1, weight=2)
        telemetry.columnconfigure(2, weight=1)
        phase_header = ttk.Frame(telemetry, style="Panel.TFrame")
        phase_header.grid(row=0, column=0, sticky="w")
        ttk.Label(phase_header, textvariable=self.collision_status_var,
                  style="Collision.TLabel").pack(side="left", padx=(0, 12))
        ttk.Label(phase_header, textvariable=self.telemetry_phase_var,
                  style="TelemetryTitle.TLabel").pack(side="left")
        ttk.Label(telemetry, textvariable=self.telemetry_state_var,
                  style="TelemetryValue.TLabel").grid(row=1, column=0, sticky="w", pady=(4, 0))
        ttk.Label(telemetry, textvariable=self.telemetry_steering_var,
                  style="TelemetrySteering.TLabel").grid(row=0, column=1, sticky="w")
        ttk.Label(telemetry, textvariable=self.telemetry_motion_var,
                  style="TelemetryMotion.TLabel").grid(row=1, column=1, sticky="w", pady=(4, 0))
        ttk.Label(telemetry, textvariable=self.telemetry_target_var,
                  style="TelemetryValue.TLabel", justify="right").grid(
                      row=0, column=2, rowspan=2, sticky="e")
        self.road_canvas = tk.Canvas(road_panel, bg=self.palette.background,
                                     highlightthickness=0)
        self.road_canvas.pack(fill="both", expand=True)
        self._expand_button(self.road_canvas, "road")

    def _build_charts_and_timeline(self) -> None:
        charts = ttk.Frame(self.root, style="Dashboard.TFrame")
        self.charts_panel = charts
        charts.rowconfigure(0, weight=1)
        charts.columnconfigure(0, weight=1, uniform="chart")
        charts.columnconfigure(1, weight=1, uniform="chart")
        charts.columnconfigure(2, weight=1, uniform="chart")
        steering_panel = ttk.Frame(charts, padding=1, style="Card.TFrame")
        trajectory_panel = ttk.Frame(charts, padding=1, style="Card.TFrame")
        velocity_panel = ttk.Frame(charts, padding=1, style="Card.TFrame")
        self.chart_panels = {
            "steering": steering_panel,
            "trajectory": trajectory_panel,
            "velocity": velocity_panel,
        }
        steering_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 4))
        trajectory_panel.grid(row=0, column=1, sticky="nsew", padx=4)
        velocity_panel.grid(row=0, column=2, sticky="nsew", padx=(4, 0))
        self.steering_chart_canvas = tk.Canvas(
            steering_panel, height=210, bg=self.palette.panel, highlightthickness=0)
        self.trajectory_chart_canvas = tk.Canvas(
            trajectory_panel, height=210, bg=self.palette.panel, highlightthickness=0)
        self.velocity_chart_canvas = tk.Canvas(
            velocity_panel, height=210, bg=self.palette.panel, highlightthickness=0)
        self.chart_canvases = {
            "steering": self.steering_chart_canvas,
            "trajectory": self.trajectory_chart_canvas,
            "velocity": self.velocity_chart_canvas,
        }
        chart_renderers = {
            "steering": self.steering_chart_renderer,
            "trajectory": self.trajectory_chart_renderer,
            "velocity": self.velocity_chart_renderer,
        }
        self.chart_title_labels: dict[str, ttk.Label] = {}
        self.chart_reset_buttons: dict[str, ModernButton] = {}
        for key, canvas in self.chart_canvases.items():
            toolbar = ttk.Frame(self.chart_panels[key], style="Panel.TFrame")
            toolbar.pack(fill="x")
            title_label = ttk.Label(
                toolbar, text=chart_renderers[key].title, style="Panel.TLabel",
                font=ui_font(TYPE_SCALE.body, "bold"), anchor="w")
            self.chart_title_labels[key] = title_label
            expand_button = ExpandButton(
                toolbar, lambda chart=key: self.toggle_panel_expansion(chart),
                self.palette)
            expand_button.pack(side="right", padx=(0, 4))
            self._expand_buttons[key] = expand_button
            reset_button = self._modern_button(
                toolbar, "Reset View", lambda chart=key: self.reset_chart_view(chart))
            reset_button.pack(side="right", padx=(0, 4))
            self.chart_reset_buttons[key] = reset_button
            # Pack the flexible title last so controls remain reachable when
            # three chart panels share a narrow window.
            title_label.pack(side="left", fill="x", expand=True, padx=(8, 4))
            canvas.pack(fill="both", expand=True)
        self._normal_chart_grid = {
            key: panel.grid_info() for key, panel in self.chart_panels.items()
        }
        self.status_label = ttk.Label(self.root, textvariable=self.status_var,
                                      padding=(15, 7), style="Muted.TLabel")
        timeline_panel = ttk.Frame(self.root, padding=(12, 5), style="Card.TFrame")
        self.timeline_panel = timeline_panel
        timeline_panel.pack(side="bottom", fill="x", padx=12, pady=(0, 10),
                            before=self.road_panel)
        self.timeline = ttk.Scale(timeline_panel, from_=0,
                                  to=len(self.run_data.samples) - 1,
                                  variable=self.timeline_var, orient="horizontal",
                                  style="Dashboard.Horizontal.TScale",
                                  command=self.scrub)
        ttk.Label(timeline_panel, textvariable=self.timeline_inspection_var,
                  style="Panel.TLabel", font=ui_font(TYPE_SCALE.label)).pack(
                      fill="x", pady=(0, 3))
        self.timeline.pack(fill="x")
        self.status_label.pack(side="bottom", fill="x", before=self.road_panel)
        charts.pack(side="bottom", fill="x", padx=12, pady=(8, 0),
                    before=self.road_panel)
        self.road_canvas.bind("<Configure>", lambda _event: self.redraw())
        self.steering_chart_canvas.bind("<Configure>", lambda _event: self.redraw())
        self.velocity_chart_canvas.bind("<Configure>", lambda _event: self.redraw())
        self.trajectory_chart_canvas.bind("<Configure>", lambda _event: self.redraw())
        self.chart_interactions = [
            ChartInteractionController(canvas, view, self.redraw_charts, renderer)
            for canvas, view, renderer in zip(
                self.chart_canvases.values(), self.chart_views,
                (self.steering_chart_renderer, self.trajectory_chart_renderer,
                 self.velocity_chart_renderer))
        ]
        self._refresh_scenario_controls(regenerate=False)
        self._normal_root_pack = tuple(
            (widget, widget.pack_info()) for widget in self.root.pack_slaves()
        )

    def _expand_button(self, canvas: tk.Canvas, key: str) -> None:
        button = ExpandButton(canvas, lambda: self.toggle_panel_expansion(key),
                              self.palette)
        button.place(relx=1.0, x=-9, y=8, anchor="ne")
        self._expand_buttons[key] = button

    def toggle_panel_expansion(self, key: str) -> None:
        """Expand one existing view in place, or restore the dashboard."""
        if key != "road" and key not in self.chart_panels:
            raise ValueError(f"Unknown panel: {key}")
        self._expanded_panel = None if self._expanded_panel == key else key
        for panel in self.chart_panels.values():
            panel.grid_forget()
        for chart_key, panel in self.chart_panels.items():
            panel.grid(**self._normal_chart_grid[chart_key])
        if self._expanded_panel in self.chart_panels:
            for chart_key, panel in self.chart_panels.items():
                if chart_key != self._expanded_panel:
                    panel.grid_remove()
            selected = self.chart_panels[self._expanded_panel]
            selected.grid_configure(column=0, columnspan=len(self.chart_panels),
                                    padx=0, sticky="nsew")

        for widget, _options in self._normal_root_pack:
            widget.pack_forget()
        if self._expanded_panel is None:
            for widget, options in self._normal_root_pack:
                widget.pack(**options)
        else:
            selected = (self.road_panel if self._expanded_panel == "road"
                        else self.charts_panel)
            selected.pack(fill="both", expand=True, padx=12, pady=(10, 0))
            timeline_options = next(options for widget, options in self._normal_root_pack
                                    if widget is self.timeline_panel)
            self.timeline_panel.pack(**timeline_options)
        for button_key, button in self._expand_buttons.items():
            button.set_expanded(button_key == self._expanded_panel)
        self.redraw()

    def _restore_on_escape(self, _event: tk.Event[tk.Misc]) -> str | None:
        if self._expanded_panel is None:
            return None
        self.toggle_panel_expansion(self._expanded_panel)
        return "break"

    def reset_chart_views(self) -> None:
        for view in self.chart_views:
            view.reset()
        self.redraw_charts()

    def reset_chart_view(self, key: str) -> None:
        self.chart_views[tuple(self.chart_canvases).index(key)].reset()
        self.redraw_charts()

    def redraw_charts(self) -> None:
        """Refresh only interactive plots for responsive zoom and panning."""
        index = self.playback.index
        self.steering_chart_renderer.draw(
            self.steering_chart_canvas, self.run_data, index)
        self.trajectory_chart_renderer.draw(
            self.trajectory_chart_canvas, self.run_data, index)
        self.velocity_chart_renderer.draw(
            self.velocity_chart_canvas, self.run_data, index)

    def toggle_theme(self) -> None:
        """Switch palette without recreating widgets or simulation state."""
        self.theme_mode = (ThemeMode.LIGHT if self.theme_mode == ThemeMode.DARK
                           else ThemeMode.DARK)
        self.palette = apply_theme(self.root, self.theme_mode)
        self.road_renderer.set_palette(self.palette)
        self.steering_chart_renderer.set_palette(self.palette)
        self.velocity_chart_renderer.set_palette(self.palette)
        self.trajectory_chart_renderer.set_palette(self.palette)
        self.road_canvas.configure(bg=self.palette.background)
        self.steering_chart_canvas.configure(bg=self.palette.panel)
        self.velocity_chart_canvas.configure(bg=self.palette.panel)
        self.trajectory_chart_canvas.configure(bg=self.palette.panel)
        for button in self._expand_buttons.values():
            button.set_palette(self.palette)
        for button in self.modern_buttons:
            button.set_palette(self.palette)
        for selector in self.modern_selectors:
            selector.set_palette(self.palette)
        self.theme_button.set_text(
            "☾  Dark mode" if self.theme_mode == ThemeMode.LIGHT else "☀  Light mode")
        self.redraw()

    def _modern_button(self, parent: tk.Misc, text: str,
                       command: Callable[[], None], *, accent: bool = False,
                       dashboard_surface: bool = False) -> ModernButton:
        button = ModernButton(parent, text, command, self.palette,
                              accent=accent,
                              dashboard_surface=dashboard_surface)
        self.modern_buttons.append(button)
        return button

    def _combo(self, parent: ttk.Frame, label: str, variable: tk.StringVar,
               options: tuple[str, ...], width: int) -> DropdownSelect:
        field = ttk.Frame(parent, style="Panel.TFrame")
        field.pack(side="left", padx=(0, 16))
        ttk.Label(field, text=label, style="Panel.TLabel").pack(side="left", padx=(0, 6))
        combo = DropdownSelect(field, variable, options, self.palette, width)
        combo.pack(side="left")
        self.modern_selectors.append(combo)
        return combo

    def _numeric_entry(self, parent: ttk.Frame, label: str,
                       variable: tk.StringVar, *, stacked: bool = False) -> NumericEntry:
        field = ttk.Frame(parent, style="Panel.TFrame")
        field.pack(side="left", padx=(0, 16))
        caption = ttk.Label(field, text=label, style="Panel.TLabel")
        if stacked:
            caption.pack(anchor="w", pady=(0, 3))
        else:
            caption.pack(side="left", padx=(0, 6))
        entry = NumericEntry(field, textvariable=variable, width=7)
        entry.pack(side="left")
        entry.bind("<Return>", lambda _event: self.regenerate())
        entry.bind("<FocusOut>", lambda _event: self.regenerate())
        return entry

    def _build_vehicle_parameters(self, parent: ttk.Frame) -> None:
        self.vehicle_panel = VehicleParametersPanel(parent, {
            "wheelbase": self.wheelbase_var,
            "length": self.length_var,
            "width": self.width_var,
            "steer_ratio": self.steer_ratio_var,
        }, self.regenerate)
        self.vehicle_panel.pack(fill="x", pady=(10, 0))

    def _current_run_request(self) -> TuningRequest:
        configuration = self._vehicle_configuration()
        simulation_config = replace(
            self.engine.config, approach_s=float(self.approach_s_var.get()),
            speed_mps=(float(self.speed_mps_var.get())
                       if self.mode_var.get() == "LKA"
                       else float(self.elk_speed_kph_var.get()) / 3.6))
        vehicle_engine = configuration.build_engine(
            SimulationEngine(simulation_config, self.engine.vehicle,
                             self.road_config))
        side = 1 if self.side_var.get() == "Left" else -1
        if self.mode_var.get() == "LKA":
            return TuningRequest(simulation_config, vehicle_engine.vehicle,
                                 "LKA", float(self.vlat_var.get()), side,
                                 radius_m=float(self.radius_var.get()),
                                 d2_m=float(self.d2_var.get()),
                                 road=self.road_config)
        scenario = self._elk_scenario_from_label(self.elk_scenario_var.get())
        intent = (LaneChangeIntent.INTENTIONAL
                  if self.intent_var.get() == "Intentional"
                  else LaneChangeIntent.UNINTENTIONAL)
        return TuningRequest(simulation_config, vehicle_engine.vehicle,
                             "ELK", float(self.vlat_var.get()), side,
                             elk_scenario=scenario, intent=intent,
                             elk_parameters=self._elk_parameters(scenario),
                             road=self.road_config, safe_termination=False)

    def _elk_parameters(self, scenario: ElkScenario) -> ElkParameters:
        """Read the visible ELK controls into validated scenario parameters."""
        traffic = scenario in (ElkScenario.ONCOMING, ElkScenario.OVERTAKING)
        parameters = ElkParameters(
            radius_m=float(self.radius_var.get()), d2_m=float(self.d2_var.get()))
        if not traffic:
            return parameters
        target = self._selected_target_type()
        length_value = self.gvt_length_var.get().strip()
        if not length_value:
            raise ValueError("GVT Length (m) is required")
        width_value = float(self.gvt_width_var.get())
        length, catalogue_width = target.dimensions(
            float(length_value), width_m=width_value if target.is_custom else None)
        width = width_value if not target.is_custom else catalogue_width
        wheelbase_value = self.gvt_wheelbase_var.get().strip()
        return replace(
            parameters, gvt_speed_kph=float(self.gvt_speed_kph_var.get()),
            gvt_marker_distance_m=float(self.gvt_marker_distance_var.get()),
            overlap_percent=(float(self.overlap_var.get())
                             if scenario == ElkScenario.ONCOMING else parameters.overlap_percent),
            gvt_length_m=length, gvt_width_m=width, gvt_type_key=target.key,
            gvt_wheelbase_m=float(wheelbase_value) if wheelbase_value else None)

    def _selected_target_type(self) -> TargetType:
        target = next((target for target in GVT_TYPES
                       if target.name == self.gvt_type_var.get()), None)
        if target is None:
            raise ValueError(f"Unknown GVT type: {self.gvt_type_var.get()}")
        return target

    def _gvt_type_changed(self, *_args: str) -> None:
        previous = target_type(self._gvt_type_key)
        if previous.is_custom:
            self._custom_gvt_dimensions = (
                self.gvt_length_var.get(), self.gvt_width_var.get(),
                self.gvt_wheelbase_var.get())
        elif previous.length_m is None:
            self._confirmed_gvt_lengths[previous.key] = self.gvt_length_var.get()
        target = self._selected_target_type()
        self._gvt_type_key = target.key
        if target.is_custom:
            length, width, wheelbase = self._custom_gvt_dimensions
        else:
            default_length = (f"{target.default_length_m:g}"
                              if target.default_length_m is not None else "")
            length = (default_length if target.length_m is not None
                      else self._confirmed_gvt_lengths.get(target.key, default_length))
            width, wheelbase = f"{target.width_m:g}", f"{target.wheelbase_m:g}"
        self.gvt_length_var.set(length)
        self.gvt_width_var.set(width)
        self.gvt_wheelbase_var.set(wheelbase)
        self.gvt_length_entry.configure(
            state="normal" if target.is_custom or target.length_m is None else "readonly")
        self.gvt_width_entry.configure(state="normal")
        self.gvt_wheelbase_entry.configure(state="normal")
        self._schedule_regeneration()

    def _vehicle_configuration(self) -> VehicleConfiguration:
        return VehicleConfiguration(
            wheelbase_m=float(self.wheelbase_var.get()),
            mass_kg=self.engine.vehicle.mass_kg,
            steer_ratio=float(self.steer_ratio_var.get()),
            body_length_m=float(self.length_var.get()),
            body_width_m=float(self.width_var.get()),
        )

    def _stop_playback(self) -> None:
        self.playback.playing = False
        if self._timer is not None:
            self.root.after_cancel(self._timer)
            self._timer = None
        self.play_button.set_text("▶  Play")

    def _keyboard_playback(self, event: tk.Event[tk.Misc]) -> str | None:
        if event.widget.winfo_class() in ("TCombobox", "Entry"):
            return None
        self.toggle_playback()
        return "break"

    def _jump_to_frame(self, index: int) -> str:
        self._stop_playback()
        self.playback.index = max(0, min(index, self.playback.frame_count - 1))
        self._follow_selected_time()
        self.redraw()
        return "break"

    def _step_frame(self, direction: int) -> str:
        return self._jump_to_frame(self.playback.index + direction)

    @staticmethod
    def _elk_scenario_from_label(label: str) -> ElkScenario:
        return next(key for key, definition in ELK_SCENARIOS.items()
                    if definition.label == label)

    def _scenario_selection_changed(self, _event: object | None = None) -> None:
        self._refresh_scenario_controls(regenerate=True)

    def _refresh_scenario_controls(self, *, regenerate: bool) -> None:
        if self.mode_var.get() == "LKA":
            self._set_field_visible(self.gvt_type_combo, False)
            self._set_field_visible(self.scenario_combo, False)
            self._set_field_visible(self.intent_combo, False)
            self._set_field_visible(self.gvt_speed_entry, False)
            self._set_field_visible(self.elk_speed_entry, False)
            self.traffic_settings.pack_forget()
            self._set_field_visible(self.line_combo, True)
            self._set_field_visible(self.d2_entry, True)
            self._set_field_visible(self.radius_entry, True)
            self._set_field_visible(self.vlat_entry, True)
            self._set_field_visible(self.speed_entry, True)
            self.side_combo.configure(values=("Left", "Right"), state="readonly")
            self.line_combo.configure(state="readonly")
            self.scenario_help_var.set("Lane keeping · both departure sides")
        else:
            scenario = self._elk_scenario_from_label(self.elk_scenario_var.get())
            overtaking = scenario == ElkScenario.OVERTAKING
            self._set_field_visible(self.scenario_combo, True)
            self._set_field_visible(self.intent_combo, overtaking)
            traffic = scenario in (ElkScenario.ONCOMING, ElkScenario.OVERTAKING)
            self._set_field_visible(self.overlap_entry, scenario == ElkScenario.ONCOMING)
            self._set_field_visible(self.gvt_type_combo, traffic)
            self._set_field_visible(self.gvt_speed_entry, traffic)
            self._set_field_visible(self.elk_speed_entry, True)
            if traffic:
                self.traffic_settings.pack(fill="x", pady=(8, 0))
            else:
                self.traffic_settings.pack_forget()
            self._set_field_visible(self.line_combo, False)
            self._set_field_visible(self.d2_entry, True)
            self._set_field_visible(self.radius_entry, True)
            self._set_field_visible(self.vlat_entry, True)
            self._set_field_visible(self.speed_entry, False)
            self.scenario_combo.configure(state="readonly")
            self.intent_combo.configure(state="readonly")
            if not overtaking:
                self.intent_var.set("Unintentional")
            intent = (LaneChangeIntent.INTENTIONAL if self.intent_var.get() == "Intentional"
                      else LaneChangeIntent.UNINTENTIONAL)
            definition = elk_definition(scenario, intent=intent)
            side_values = tuple("Left" if side == 1 else "Right"
                                for side in definition.allowed_sides)
            self.side_combo.configure(values=side_values, state="readonly")
            if self.side_var.get() not in side_values:
                self.side_var.set(side_values[0])
            if scenario == ElkScenario.ROAD_EDGE:
                self.line_var.set("Solid")
            elif scenario in (ElkScenario.ONCOMING, ElkScenario.OVERTAKING):
                self.line_var.set("Dashed")
            else:
                self.line_var.set("Solid")
            side_text = " / ".join(side_values)
            self.scenario_help_var.set(
                f"{definition.tested_boundary} · {side_text} · No ELK intervention")
        if regenerate:
            self.regenerate(show_errors=False)
            if self._layout_timer is not None:
                self.root.after_cancel(self._layout_timer)
            self._layout_timer = self.root.after_idle(self._fit_chart_height)

    @staticmethod
    def _set_field_visible(combo: DropdownSelect | NumericEntry, visible: bool) -> None:
        """Show or hide a labeled field without leaving an empty gap."""
        field = combo.master
        if visible:
            if not field.winfo_manager():
                field.pack(side="left", padx=(0, 16))
        elif field.winfo_manager():
            field.pack_forget()

    def _schedule_regeneration(self, *_args: str) -> None:
        if self._regenerate_timer is not None:
            self.root.after_cancel(self._regenerate_timer)
        self._regenerate_timer = self.root.after(
            350, lambda: self.regenerate(show_errors=False))

    def regenerate(self, *, show_errors: bool = True) -> None:
        if self._regenerate_timer is not None:
            self.root.after_cancel(self._regenerate_timer)
            self._regenerate_timer = None
        self._stop_playback()
        try:
            request = self._current_run_request()
            candidate = request.run(request.config.lookahead_time_s)
            candidate_engine = SimulationEngine(request.config, request.vehicle,
                                                request.road)
        except (ValueError, ArithmeticError) as error:
            self._generation_error = str(error)
            LOGGER.warning("Could not generate LSS simulation: %s", error)
            self.redraw()
            if show_errors:
                messagebox.showerror("Simulation error", str(error), parent=self.root)
            return
        self._generation_error = None
        self.run_data = candidate
        self.engine = candidate_engine
        self.simulation_config = candidate_engine.config
        self.road_renderer.vehicle = candidate_engine.vehicle
        self.road_renderer.simulation = candidate_engine.config
        for view in self.chart_views:
            view.reset()
        self.playback.reset(len(candidate.samples))
        self._syncing_timeline = True
        self.timeline.configure(to=len(candidate.samples) - 1)
        self.timeline_var.set(0)
        self._syncing_timeline = False
        self.redraw()

    def export_csv(self) -> None:
        self._export_csv(profile_only=False)

    def export_swa(self) -> None:
        self._export_csv(profile_only=True)

    def _export_csv(self, *, profile_only: bool) -> None:
        if not self._ensure_current_run():
            return
        filename = filedialog.asksaveasfilename(
            parent=self.root, defaultextension=".csv",
            initialdir=str(user_export_dir()),
            initialfile=(f"{self.run_data.mode.lower()}_{self.run_data.scenario_key}_"
                         f"{self.vlat_var.get()}_{self.side_var.get().lower()}"
                         f"{'_swa' if profile_only else ''}.csv"),
            filetypes=[("CSV files", "*.csv")],
        )
        if not filename:
            return
        try:
            exporter = export_swa_profile if profile_only else export_run
            exporter(Path(filename), self.run_data)
        except (OSError, ValueError) as error:
            LOGGER.exception("Could not export simulation to %s", filename)
            messagebox.showerror("Export error", str(error), parent=self.root)
            return
        LOGGER.info("Exported %d samples to %s", len(self.run_data.samples), filename)
        self.status_var.set(f"Saved {len(self.run_data.samples)} samples to {filename}")

    def toggle_playback(self) -> None:
        if self.playback.playing:
            self._stop_playback()
            return
        if not self._ensure_current_run():
            return
        if self.playback.index == self.playback.frame_count - 1:
            self.playback.index = 0
            self._follow_selected_time()
            self.redraw()
        self.playback.playing = True
        self.playback_clock.start(self.playback.index, time.monotonic())
        self.play_button.set_text("Ⅱ  Pause")
        self._tick()

    def _ensure_current_run(self) -> bool:
        if self._regenerate_timer is not None:
            self.regenerate(show_errors=False)
        if self._generation_error is not None:
            messagebox.showerror("Simulation parameters", self._generation_error, parent=self.root)
            return False
        return True

    def _tick(self) -> None:
        self._timer = None
        if not self.playback.playing:
            return
        previous_index = self.playback.index
        self.playback.index = self.playback_clock.index_at(
            time.monotonic(), self.simulation_config.sample_period_s,
            self.playback.frame_count)
        if self.playback.index != previous_index:
            self._follow_selected_time()
        more = self.playback.index < self.playback.frame_count - 1
        if not more:
            self.playback.playing = False
        if self.playback.index != previous_index or not more:
            self.redraw()
        if more:
            self._timer = self.root.after(16, self._tick)
        else:
            self._stop_playback()

    def scrub(self, value: str) -> None:
        if self._syncing_timeline:
            return
        self._stop_playback()
        previous_index = self.playback.index
        self.playback.scrub(float(value))
        if self.playback.index != previous_index:
            self._follow_selected_time()
            self.redraw(sync_timeline=False)
        else:
            self._update_timeline_inspection()

    def _follow_selected_time(self) -> None:
        sample = self.run_data.samples[self.playback.index]
        fraction = sample.time_s / self.run_data.duration_s
        for view in self.chart_views:
            view.ensure_x_visible(fraction)

    def _update_timeline_inspection(self) -> None:
        sample = self.run_data.samples[self.playback.index]
        self.timeline_inspection_var.set(
            f"Time {sample.time_s:.3f} s  ·  SWA {sample.steering_command_deg:+.2f}°"
            f" / NCAP {sample.reference_steering_wheel_deg:+.2f}°"
            f"  ·  Vlat {sample.v_lat_mps:+.3f} / NCAP {sample.reference_v_lat_mps:+.3f} m/s"
            f"  ·  y ref {sample.reference_y_m:+.3f} / sim {sample.front_y_m:+.3f}"
            f" / error {sample.lateral_error_m:+.3f} m")

    def redraw(self, sync_timeline: bool = True) -> None:
        sample = self.run_data.samples[self.playback.index]
        if sync_timeline:
            self._syncing_timeline = True
            self.timeline_var.set(self.playback.index)
            self._syncing_timeline = False
        run = self.run_data
        telemetry = telemetry_view(run, self.playback.index)
        self.collision_status_var.set(telemetry.collision_status)
        self.telemetry_phase_var.set(telemetry.phase)
        self.telemetry_state_var.set(telemetry.state)
        self.telemetry_steering_var.set(telemetry.steering)
        self.telemetry_motion_var.set(telemetry.motion)
        self.telemetry_target_var.set(telemetry.target)
        self._update_timeline_inspection()
        self.road_renderer.draw(self.road_canvas, self.run_data, self.playback.index,
                                float(self.lane_width_var.get()), self.line_var.get())
        self.redraw_charts()
        path = self.run_data.path
        self.status_var.set(
            f"{run.mode} · {run.scenario_name} · {run.tested_boundary}"
            f"   |   Lane {self.lane_width_var.get()} m · marking {self.road_config.marking_width_m:.2f} m"
            f"   |   Approach {run.config.approach_s:g} s"
            f"   |   NCAP path R {path.radius_m:g} m · ψ {path.psi_deg:.2f}°"
            f" · d1 {path.d1_table:.2f} m · d2 {path.d2:.2f} m"
            f" · tyre-line offset {path.line_offset_m:.3f} m"
        )
        if run.target_samples:
            initial_ego = run.samples[0]
            initial_target = run.target_samples[0]
            self.status_var.set(
                self.status_var.get()
                + f"   |   Initial VUT ({initial_ego.front_x_m:+.2f}, {initial_ego.front_y_m:+.2f}) m"
                + f" · GVT ({initial_target.x_m:+.2f}, {initial_target.y_m:+.2f}) m"
            )
        if run.collision_detected and self.playback.index == len(run.samples) - 1:
            self.status_var.set(
                self.status_var.get()
                + f"   |   Collision at {run.collision_time_s:.6f} s · scenario terminated")
        if self._generation_error is not None:
            self.status_var.set(
                f"Invalid parameters: {self._generation_error}. "
                "Showing last valid run.   |   " + self.status_var.get())

    def close(self) -> None:
        if self._layout_timer is not None:
            self.root.after_cancel(self._layout_timer)
            self._layout_timer = None
        if self._regenerate_timer is not None:
            self.root.after_cancel(self._regenerate_timer)
            self._regenerate_timer = None
        self._stop_playback()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()
