"""Display-independent checks for physical scaling and timeline behavior."""
from __future__ import annotations

import unittest
from dataclasses import replace

from lss_ncap.config import ROAD
from lss_ncap.collision import ego_polygon
from lss_ncap.simulation import SimulationEngine
from lss_ncap.scenarios.elk import ElkParameters, ElkScenario
from lss_ncap.ui.playback import PlaybackClock, PlaybackState
from lss_ncap.ui.chart_interaction import ChartView
from lss_ncap.ui.telemetry import telemetry_view
from lss_ncap.visualization.geometry import RoadViewport, vehicle_corners
from lss_ncap.visualization.road import RoadRenderer
from lss_ncap.visualization.vehicle import EGO_STYLE, TopDownCarPainter
from lss_ncap.ui.theme import ThemeMode, palette_for
from lss_ncap.visualization.chart import (CurvatureChartRenderer,
                                          LateralVelocityChartRenderer,
                                          LateralTrajectoryChartRenderer,
                                          SteeringAngleChartRenderer,
                                          SteeringChartRenderer)


class RecordingCanvas:
    def __init__(self, width: int = 1120, height: int = 500) -> None:
        self.width, self.height = width, height
        self.lines: list[tuple[tuple[object, ...], dict[str, object]]] = []
        self.polygons: list[tuple[tuple[object, ...], dict[str, object]]] = []
        self.texts: list[str] = []
        self.rectangles: list[dict[str, object]] = []

    def winfo_width(self) -> int:
        return self.width

    def winfo_height(self) -> int:
        return self.height

    def delete(self, *_: object) -> None:
        self.lines.clear()
        self.polygons.clear()
        self.texts.clear()
        self.rectangles.clear()

    def create_line(self, *args: object, **kwargs: object) -> None:
        self.lines.append((args, kwargs))

    def create_polygon(self, *args: object, **kwargs: object) -> None:
        self.polygons.append((args, kwargs))

    def create_rectangle(self, *args: object, **kwargs: object) -> None:
        self.rectangles.append(kwargs)

    def create_oval(self, *args: object, **kwargs: object) -> None:
        pass

    def create_text(self, *args: object, **kwargs: object) -> None:
        self.texts.append(str(kwargs.get("text", "")))


class ViewModelTests(unittest.TestCase):
    def test_initial_longitudinal_readout_uses_both_body_centres(self) -> None:
        for scenario in (ElkScenario.ONCOMING, ElkScenario.OVERTAKING):
            run = SimulationEngine().run_elk(scenario)
            for yaw_deg in (0.0, 30.0, -45.0):
                with self.subTest(scenario=scenario, yaw_deg=yaw_deg):
                    initial_vut = replace(run.samples[0], front_x_m=7.0, yaw_deg=yaw_deg)
                    changed = replace(run, samples=(initial_vut,) + run.samples[1:])
                    vut_center_x = sum(x for x, _ in ego_polygon(initial_vut, run.vehicle)) / 4
                    expected_dx = run.target_samples[0].x_m - vut_center_x
                    label = f"Initial centre dx {expected_dx:+.2f} m"
                    for index in (0, len(changed.samples) - 1):
                        text = telemetry_view(changed, index).target
                        self.assertIn(label, text)
                        self.assertNotIn("Initial x ", text)

    @staticmethod
    def _contrast_ratio(first: str, second: str) -> float:
        def luminance(color: str) -> float:
            channels = [int(color[index:index + 2], 16) / 255
                        for index in (1, 3, 5)]
            linear = [value / 12.92 if value <= 0.04045
                      else ((value + 0.055) / 1.055) ** 2.4
                      for value in channels]
            return 0.2126 * linear[0] + 0.7152 * linear[1] + 0.0722 * linear[2]

        lighter, darker = sorted((luminance(first), luminance(second)),
                                  reverse=True)
        return (lighter + 0.05) / (darker + 0.05)

    def test_light_palette_is_high_contrast_and_dark_palette_is_unchanged(self) -> None:
        light = palette_for(ThemeMode.LIGHT)
        dark = palette_for(ThemeMode.DARK)
        self.assertEqual(
            (dark.background, dark.panel, dark.panel_alt, dark.foreground,
             dark.muted, dark.border, dark.grid, dark.accent, dark.accent_hover,
             dark.accent_text, dark.cyan, dark.green, dark.orange,
             dark.yellow, dark.magenta),
            ("#0d0f1c", "#171a2a", "#22263a", "#f0f0f7", "#a8a9bd",
             "#373b59", "#3a3d57", "#6d61d5", "#7467dc", "#b3aaff",
             "#22d3ee", "#42d392", "#ff8a4c", "#ffd166", "#f05bd8"))
        self.assertGreaterEqual(self._contrast_ratio(light.foreground, light.panel), 12)
        self.assertGreaterEqual(self._contrast_ratio(light.muted, light.panel), 7)
        for color in (light.cyan, light.green, light.orange,
                      light.yellow, light.magenta):
            self.assertGreaterEqual(self._contrast_ratio(color, light.panel), 4.5)
        self.assertGreater(self._contrast_ratio(light.border, light.panel),
                           self._contrast_ratio("#d2dce4", "#ffffff"))
        self.assertGreater(self._contrast_ratio(light.grid, light.panel),
                           self._contrast_ratio("#dbe3e9", "#ffffff"))

    def test_chart_view_zoom_pan_and_reset_are_bounded(self) -> None:
        view = ChartView()
        view.zoom(0.5, 0.5, zoom_in=True)
        self.assertAlmostEqual(view.x_min, 0.1)
        self.assertAlmostEqual(view.x_max, 0.9)
        self.assertAlmostEqual(view.y_min, 0.1)
        self.assertAlmostEqual(view.y_max, 0.9)
        view.pan(0.25, -0.25)
        self.assertGreaterEqual(view.x_min, 0.0)
        self.assertLessEqual(view.x_max, 1.0)
        self.assertGreaterEqual(view.y_min, 0.0)
        self.assertLessEqual(view.y_max, 1.0)
        view.zoom(0.0, 0.0, zoom_in=False)
        self.assertGreaterEqual(view.x_min, 0.0)
        self.assertLessEqual(view.x_max, 1.0)
        view.reset()
        self.assertEqual((view.x_min, view.x_max, view.y_min, view.y_max),
                         (0.0, 1.0, 0.0, 1.0))

    def test_charts_are_independent_and_support_both_palettes(self) -> None:
        run = SimulationEngine().run_lka()
        for mode in ThemeMode:
            palette = palette_for(mode)
            steering = RecordingCanvas(width=620, height=220)
            velocity = RecordingCanvas(width=620, height=220)
            trajectory = RecordingCanvas(width=620, height=220)
            curvature = RecordingCanvas(width=620, height=220)
            SteeringAngleChartRenderer(palette).draw(steering, run, 120)
            LateralVelocityChartRenderer(palette).draw(velocity, run, 120)
            LateralTrajectoryChartRenderer(palette).draw(trajectory, run, 120)
            CurvatureChartRenderer(palette).draw(curvature, run, 120)
            self.assertEqual(SteeringAngleChartRenderer.title,
                             "STEERING WHEEL ANGLE (SWA) [°]")
            self.assertNotIn(SteeringAngleChartRenderer.title, steering.texts)
            self.assertIn("Steering Wheel Angle (SWA)", steering.texts)
            self.assertIn("NCAP reference", steering.texts)
            self.assertNotIn("Applied", steering.texts)
            self.assertNotIn("Pursuit", steering.texts)
            self.assertEqual(LateralVelocityChartRenderer.title,
                             "LATERAL VELOCITY [m/s]")
            self.assertNotIn(LateralVelocityChartRenderer.title, velocity.texts)
            self.assertEqual(LateralTrajectoryChartRenderer.title,
                             "LATERAL TRAJECTORY [m]")
            self.assertNotIn(LateralTrajectoryChartRenderer.title, trajectory.texts)
            self.assertEqual(CurvatureChartRenderer.title, "CURVATURE [1/km]")
            self.assertNotIn(CurvatureChartRenderer.title, curvature.texts)
            self.assertFalse(any("Tracking error" in text for text in trajectory.texts))
            self.assertTrue(any(rectangle.get("fill") == palette.panel
                                for rectangle in steering.rectangles))
            signal_colors = {palette.magenta, palette.cyan, palette.orange,
                             palette.yellow, palette.green}
            self.assertEqual(len(signal_colors), 5)
            steering_colors = {line.get("fill") for _, line in steering.lines}
            velocity_colors = {line.get("fill") for _, line in velocity.lines}
            trajectory_colors = {line.get("fill") for _, line in trajectory.lines}
            self.assertTrue({palette.cyan, palette.orange} <= steering_colors)
            self.assertNotIn(palette.magenta, steering_colors)
            self.assertTrue({palette.yellow, palette.green} <= velocity_colors)
            self.assertTrue({palette.orange, palette.magenta} <= trajectory_colors)

    def test_elk_road_edge_and_target_vehicle_layouts(self) -> None:
        renderer = RoadRenderer()
        self.assertEqual(renderer.target_vehicle_painter.asset_path.name,
                         "top_down_green_gvt.png")
        self.assertNotEqual(renderer.target_vehicle_painter.asset_path,
                            renderer.ego_vehicle_painter.asset_path)
        road_edge = SimulationEngine().run_elk(ElkScenario.ROAD_EDGE, 0.4)
        edge_canvas = RecordingCanvas()
        renderer.draw(edge_canvas, road_edge, 100, 3.5, "Solid")
        self.assertTrue(any(rectangle.get("fill") == "#6d9254"
                            for rectangle in edge_canvas.rectangles))

        oncoming = SimulationEngine().run_elk(ElkScenario.ONCOMING, 0.4)
        target_canvas = RecordingCanvas()
        renderer.draw(target_canvas, oncoming, len(oncoming.samples) - 40, 3.5, "Dashed")
        self.assertGreaterEqual(len(target_canvas.polygons), 4)
        self.assertIn("GVT", target_canvas.texts)
        self.assertIn("Initial", telemetry_view(oncoming, 0).target)

        overtaking = SimulationEngine().run_elk(
            ElkScenario.OVERTAKING, 0.4, safe_termination=False)
        synchronization = telemetry_view(overtaking, 0).target
        self.assertIn("Nominal Encounter Time", synchronization)
        self.assertIn("Longitudinal Synchronization Error", synchronization)
        self.assertIn("Longitudinal Synchronization Error: +0.000000 m", synchronization)

        initial_canvas = RecordingCanvas(width=1120, height=260)
        renderer.draw(initial_canvas, oncoming, 0, 3.5, "Dashed")
        self.assertTrue(any(text.startswith("GVT ")
                            for text in initial_canvas.texts))

    def test_chart_values_stay_hidden_without_timeline_drag(self) -> None:
        run = SimulationEngine().run_lka()
        chart = RecordingCanvas(height=260)
        road = RecordingCanvas()
        for index in (0, 120, len(run.samples) - 1):
            sample = run.samples[index]
            SteeringChartRenderer().draw(chart, run, index)
            RoadRenderer().draw(road, run, index, 3.5, "Dashed")
            self.assertFalse(any(f"SWA {sample.steering_command_deg:+.2f}°" in text
                                 for text in chart.texts))
            self.assertFalse(any("Area estimate ≈" in text for text in chart.texts))
            self.assertEqual(telemetry_view(run, index).steering,
                             (f"Robot {sample.robot_steering_command_deg:+.2f}° · "
                              f"EPS {sample.eps_steering_wheel_deg:+.2f}° · "
                              f"ELK {sample.elk_steering_command_deg:+.2f}°"))
            self.assertFalse(any("look-ahead" in text.lower() for text in road.texts))
            if index < len(run.samples) - 1:
                self.assertAlmostEqual(run.samples[index + 1].steering_wheel_deg,
                                       sample.steering_command_deg)

    def test_handover_reports_profile_override(self) -> None:
        run = SimulationEngine().run_lka(lane_support=lambda _state: 0.0)
        canvas = RecordingCanvas()
        RoadRenderer().draw(canvas, run, 0, 3.5, "Solid")
        self.assertIn("ELK intervention", telemetry_view(run, 0).phase)

    def test_lane_and_marking_widths_share_physical_scale(self) -> None:
        for lane_width in ROAD.lane_width_options_m:
            view = RoadViewport(1120, 500, 40, 1.5, lane_width)
            divider = view.screen(40, 1.5)[1]
            edge = view.screen(40, 1.5 + lane_width)[1]
            self.assertAlmostEqual(abs(edge - divider),
                                   lane_width * view.scale_px_per_m)
            self.assertAlmostEqual(view.marking_width_px,
                                   0.2 * view.scale_px_per_m)

    def test_road_renderer_uses_two_edges_and_scaled_markings(self) -> None:
        run = SimulationEngine().run_lka()
        canvas = RecordingCanvas()
        RoadRenderer().draw(canvas, run, 20, 3.7, "Dashed")
        yellow = [kw for _, kw in canvas.lines if kw.get("fill") == "#ddb84b"]
        white = [kw for _, kw in canvas.lines if kw.get("fill") == "#e9eef2"]
        expected_width = RoadViewport(1120, 500, 40, 1.5, 3.7).marking_width_px
        self.assertEqual(len(yellow), 2)
        self.assertGreater(len(white), 1)
        self.assertTrue(all(kw["width"] == expected_width for kw in yellow + white))
        self.assertGreaterEqual(len(canvas.polygons), 2)  # shadow and car body

    def test_overtaking_outlines_show_nominal_time_without_moving_impact_vehicles(self) -> None:
        run = SimulationEngine().run_elk("overtaking", parameters=ElkParameters())
        nominal = run.nominal_encounter
        assert nominal is not None and run.collision_time_s is not None
        gap = nominal.time_s - run.collision_time_s
        self.assertLess(abs(gap), 0.001)
        self.assertLess(abs(nominal.vut_front_x_m - run.samples[-1].front_x_m), 0.02)
        self.assertLess(abs(nominal.target_x_m - run.target_samples[-1].x_m), 0.02)
        samples, targets = run.samples, run.target_samples
        canvas = RecordingCanvas()
        RoadRenderer().draw(canvas, run, len(samples) - 1, 3.5, "Dashed")
        text = " ".join(canvas.texts)
        self.assertNotIn("No-ELK encounter", text)
        self.assertNotIn("Orange outlines:", text)
        self.assertTrue(any(style.get("outline") == RoadRenderer().palette.orange
                            and style.get("dash") == (4, 4)
                            for _, style in canvas.polygons))
        self.assertIn("First contact", text)
        telemetry = telemetry_view(run, len(samples) - 1)
        self.assertIn("Actual Collision Time", telemetry.target)
        self.assertEqual(telemetry.collision_status, "COLLISION DETECTED")
        self.assertIs(run.samples, samples)
        self.assertIs(run.target_samples, targets)
        RoadRenderer().draw(canvas, run, 0, 3.5, "Dashed")
        self.assertNotIn("earlier", " ".join(canvas.texts))
        self.assertNotIn("after impact", telemetry_view(run, 0).target)
        self.assertIsNone(replace(run, collision_time_s=None).collision_before_nominal_s)
        self.assertIsNone(replace(run, collision_time_s=nominal.time_s + 1).collision_before_nominal_s)

    def test_vehicle_footprint_uses_configured_dimensions(self) -> None:
        run = SimulationEngine().run_lka()
        sample = run.samples[0]
        view = RoadViewport(1120, 500, 0, 1.5, 3.5)
        corners = vehicle_corners(sample, run.vehicle, view)
        self.assertAlmostEqual(corners[0][0] - corners[3][0],
                               run.vehicle.body_length_m * view.scale_px_per_m)
        self.assertAlmostEqual(corners[1][1] - corners[0][1],
                               -run.vehicle.body_width_m * view.scale_px_per_m)

    def test_top_down_car_uses_exact_configured_outer_dimensions(self) -> None:
        canvas = RecordingCanvas()
        view = RoadViewport(1120, 500, 0, 0, 3.5)
        outline = TopDownCarPainter().draw(
            canvas, view, center_x_m=0, center_y_m=0, yaw_deg=0,
            length_m=4.6, width_m=1.9, style=EGO_STYLE)
        x_values = [point[0] for point in outline]
        y_values = [point[1] for point in outline]
        self.assertAlmostEqual(max(x_values) - min(x_values),
                               4.6 * view.scale_px_per_m)
        self.assertAlmostEqual(max(y_values) - min(y_values),
                               1.9 * view.scale_px_per_m)
        self.assertGreaterEqual(len(canvas.polygons), 9)

    def test_scrubbing_clamps_and_pauses_playback(self) -> None:
        playback = PlaybackState(10, playing=True)
        playback.scrub(4.6)
        self.assertEqual(playback.index, 5)
        self.assertFalse(playback.playing)
        playback.scrub(99)
        self.assertEqual(playback.index, 9)
        playback.scrub(-3)
        self.assertEqual(playback.index, 0)
        playback.playing = True
        for _ in range(9):
            playback.advance()
        self.assertEqual(playback.index, 9)
        self.assertFalse(playback.playing)

    def test_playback_clock_is_drift_free_and_clamped(self) -> None:
        clock = PlaybackClock()
        clock.start(index=4, now_s=10.0)
        self.assertEqual(clock.index_at(10.024, 0.025, 20), 4)
        self.assertEqual(clock.index_at(10.025, 0.025, 20), 5)
        self.assertEqual(clock.index_at(99.0, 0.025, 20), 19)


if __name__ == "__main__":
    unittest.main()
