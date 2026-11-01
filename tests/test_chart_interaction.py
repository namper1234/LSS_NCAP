"""Plot interaction behavior in dashboard and expanded viewer layouts."""
from __future__ import annotations

import tkinter as tk
import unittest
from types import SimpleNamespace

from lss_ncap.simulation import SimulationEngine
from lss_ncap.ui.app import SimulationApp
from lss_ncap.ui.chart_interaction import ChartView
from lss_ncap.visualization.chart import BaseChartRenderer, ChartGeometry


class ChartViewTests(unittest.TestCase):
    def test_timeline_readout_tracks_selected_sample_without_mouse_events(self) -> None:
        run = SimulationEngine().run_lka()

        class Readout:
            value = ""

            def set(self, value: str) -> None:
                self.value = value

        readout = Readout()
        app = SimpleNamespace(run_data=run, playback=SimpleNamespace(index=0),
                              timeline_inspection_var=readout)
        for index in (0, 40, len(run.samples) - 1):
            app.playback.index = index
            SimulationApp._update_timeline_inspection(app)
            sample = run.samples[index]
            self.assertIn(f"Time {sample.time_s:.3f} s", readout.value)
            self.assertIn(f"SWA {sample.steering_command_deg:+.2f}°", readout.value)
            self.assertIn(f"Vlat {sample.v_lat_mps:+.3f}", readout.value)

    def test_series_segments_crossing_view_edges_are_clipped(self) -> None:
        geometry = ChartGeometry(10, 100, 20, 80)
        self.assertEqual(
            BaseChartRenderer._clip_segment(-10, 50, 50, 50, geometry),
            ((10.0, 50.0), (50.0, 50.0)))
        self.assertIsNone(BaseChartRenderer._clip_segment(
            -10, 10, -5, 50, geometry))

    def test_compact_header_increases_vertical_plot_area(self) -> None:
        canvas = SimpleNamespace(winfo_width=lambda: 620,
                                 winfo_height=lambda: 210)
        _width, height, geometry = BaseChartRenderer._dimensions(canvas)
        self.assertEqual(geometry.top, 29)
        self.assertEqual(geometry.bottom, height - 27)
        self.assertGreater(geometry.bottom - geometry.top, 150)
        self.assertLess(BaseChartRenderer.LEGEND_Y, geometry.top)

    def test_box_zoom_and_reset_keep_legend_visibility_independent(self) -> None:
        view = ChartView()
        view.toggle_series("front_y_m")
        self.assertFalse(view.is_visible("front_y_m"))
        self.assertTrue(view.is_visible("reference_y_m"))
        self.assertFalse(view.box_zoom(0.4, 0.4, 0.41, 0.41))
        for _ in range(3):
            self.assertTrue(view.box_zoom(0.2, 0.1, 0.8, 0.9))
        self.assertLess(view.x_max - view.x_min, 0.25)
        self.assertLess(view.y_max - view.y_min, 0.6)
        view.reset()
        self.assertEqual((view.x_min, view.x_max, view.y_min, view.y_max),
                         (0.0, 1.0, 0.0, 1.0))
        self.assertFalse(view.is_visible("front_y_m"))
        view.toggle_series("front_y_m")
        self.assertTrue(view.is_visible("front_y_m"))
        view.zoom(0.5, 0.5, zoom_in=True)
        original_span = view.x_max - view.x_min
        view.ensure_x_visible(1.0)
        self.assertAlmostEqual(view.x_max, 1.0)
        self.assertAlmostEqual(view.x_max - view.x_min, original_span)

    def test_canvas_interactions_survive_expansion(self) -> None:
        try:
            root = tk.Tk()
        except tk.TclError as error:
            self.skipTest(f"Tk display unavailable: {error}")
        app = None
        try:
            app = SimulationApp(root=root)
            root.update()
            canvas = app.trajectory_chart_canvas
            renderer = app.trajectory_chart_renderer
            view = app.chart_views[1]
            legend_y = renderer.LEGEND_Y
            self.assertEqual(renderer.legend_at(170, legend_y), "front_y_m")
            self.assertTrue(canvas.find_withtag("series:front_y_m"))
            canvas.event_generate("<Motion>", x=170, y=legend_y)
            root.update()
            self.assertEqual(canvas.cget("cursor"), "hand2")
            self.assertFalse(canvas.find_withtag("chart-tooltip"))
            canvas.event_generate("<ButtonPress-1>", x=170, y=legend_y)
            root.update()
            self.assertFalse(view.is_visible("front_y_m"))
            self.assertFalse(canvas.find_withtag("series:front_y_m"))
            self.assertTrue(canvas.find_withtag("series:reference_y_m"))

            app._expand_buttons["trajectory"].invoke()
            root.update()
            self.assertFalse(view.is_visible("front_y_m"))
            left, top, right, bottom = renderer.plot_bounds()
            x0, y0 = round(left + 0.2 * (right - left)), round(top + 0.2 * (bottom - top))
            x1, y1 = round(left + 0.8 * (right - left)), round(top + 0.8 * (bottom - top))
            canvas.event_generate("<ButtonPress-3>", x=x0, y=y0)
            canvas.event_generate("<B3-Motion>", x=x1, y=y1, state=0x0400)
            root.update()
            self.assertTrue(canvas.find_withtag("chart-selection"))
            canvas.event_generate("<ButtonRelease-3>", x=x1, y=y1)
            root.update()
            self.assertLess(view.x_max - view.x_min, 0.7)
            self.assertLess(view.y_max - view.y_min, 0.7)

            center_x, center_y = round((left + right) / 2), round((top + bottom) / 2)
            before_wheel = view.x_max - view.x_min
            canvas.event_generate("<MouseWheel>", x=center_x, y=center_y, delta=120)
            root.update()
            self.assertLess(view.x_max - view.x_min, before_wheel)
            before_pan = view.x_min
            canvas.event_generate("<ButtonPress-1>", x=center_x, y=center_y)
            canvas.event_generate("<B1-Motion>", x=center_x - 25, y=center_y,
                                  state=0x0100)
            canvas.event_generate("<ButtonRelease-1>", x=center_x - 25,
                                  y=center_y)
            root.update()
            self.assertGreater(view.x_min, before_pan)
            canvas.event_generate("<Motion>", x=center_x, y=center_y)
            root.update()
            self.assertFalse(canvas.find_withtag("chart-tooltip"))

            zoomed_limits = (view.x_min, view.x_max, view.y_min, view.y_max)
            app._expand_buttons["trajectory"].invoke()
            root.update()
            self.assertEqual((view.x_min, view.x_max, view.y_min, view.y_max),
                             zoomed_limits)
            app._expand_buttons["trajectory"].invoke()
            root.update()
            self.assertEqual((view.x_min, view.x_max, view.y_min, view.y_max),
                             zoomed_limits)
            app.toggle_theme()
            root.update()
            self.assertEqual((view.x_min, view.x_max, view.y_min, view.y_max),
                             zoomed_limits)
            self.assertFalse(view.is_visible("front_y_m"))

            reset = app.chart_reset_buttons["trajectory"]
            button_x, button_y = reset.winfo_width() // 2, reset.winfo_height() // 2
            reset.event_generate("<ButtonPress-1>", x=button_x, y=button_y)
            reset.event_generate("<ButtonRelease-1>", x=button_x, y=button_y)
            root.update()
            self.assertEqual((view.x_min, view.x_max, view.y_min, view.y_max),
                             (0.0, 1.0, 0.0, 1.0))
            self.assertFalse(view.is_visible("front_y_m"))
            app._expand_buttons["trajectory"].invoke()
            root.update()
            self.assertFalse(view.is_visible("front_y_m"))
            canvas.event_generate("<ButtonPress-1>", x=170, y=legend_y)
            root.update()
            self.assertTrue(view.is_visible("front_y_m"))
            self.assertTrue(canvas.find_withtag("series:front_y_m"))
            for chart_canvas, chart_view, fields, positions in (
                (app.steering_chart_canvas, app.chart_views[0],
                 ("steering_command_deg", "reference_steering_wheel_deg"),
                 (20, 245)),
                (app.velocity_chart_canvas, app.chart_views[2],
                 ("v_lat_mps", "v_lat_from_steering_mps"), (20, 145)),
            ):
                chart_canvas.event_generate("<ButtonPress-1>",
                                            x=positions[0],
                                            y=BaseChartRenderer.LEGEND_Y)
                root.update()
                self.assertFalse(chart_view.is_visible(fields[0]))
                self.assertTrue(chart_canvas.find_withtag(f"series:{fields[1]}"))
                chart_canvas.event_generate("<ButtonPress-1>",
                                            x=positions[1],
                                            y=BaseChartRenderer.LEGEND_Y)
                root.update()
                self.assertFalse(chart_view.is_visible(fields[1]))
                self.assertFalse(chart_canvas.find_withtag(f"series:{fields[0]}"))
                self.assertFalse(chart_canvas.find_withtag(f"series:{fields[1]}"))
                for position in positions:
                    chart_canvas.event_generate("<ButtonPress-1>",
                                                x=position,
                                                y=BaseChartRenderer.LEGEND_Y)
                root.update()
                self.assertTrue(all(chart_view.is_visible(field) for field in fields))
        finally:
            if app is not None:
                app.close()
            else:
                root.destroy()

    def test_selected_signal_values_remain_visible_without_hover(self) -> None:
        try:
            root = tk.Tk()
        except tk.TclError as error:
            self.skipTest(f"Tk display unavailable: {error}")
        app = None
        try:
            app = SimulationApp(root=root)
            root.update()
            initial_sample = app.run_data.samples[0]
            self.assertIn(f"Time {initial_sample.time_s:.3f} s",
                          app.timeline_inspection_var.get())
            self.assertIn(f"SWA {initial_sample.steering_command_deg:+.2f}°",
                          app.telemetry_steering_var.get())
            for canvas in app.chart_canvases.values():
                self.assertEqual(len(canvas.find_withtag("time-cursor")), 1)
                canvas.event_generate("<Motion>", x=180, y=110)
                root.update()
                self.assertFalse(canvas.find_withtag("chart-tooltip"))

            slider_y = app.timeline.winfo_height() // 2
            self.assertGreater(app.timeline.winfo_width(), 500)
            app.timeline.event_generate("<ButtonPress-1>", x=10, y=slider_y)
            root.update()
            app.timeline.event_generate("<B1-Motion>",
                                        x=app.timeline.winfo_width() // 2,
                                        y=slider_y, state=0x0100)
            root.update()
            self.assertGreater(app.playback.index, 0)
            self.assertIn("Time ", app.timeline_inspection_var.get())
            for index in (40, 120, 200):
                app.scrub(str(index))
                root.update()
                sample = app.run_data.samples[index]
                readout = app.timeline_inspection_var.get()
                for expected in (
                    f"Time {sample.time_s:.3f} s",
                    f"SWA {sample.steering_command_deg:+.2f}°",
                    f"NCAP {sample.reference_steering_wheel_deg:+.2f}°",
                    f"Vlat {sample.v_lat_mps:+.3f}",
                    f"area {sample.v_lat_from_steering_mps:+.3f}",
                    f"y ref {sample.reference_y_m:+.3f}",
                    f"sim {sample.front_y_m:+.3f}",
                ):
                    self.assertIn(expected, readout)
                for key, canvas in app.chart_canvases.items():
                    cursor = canvas.find_withtag("time-cursor")
                    self.assertEqual(len(cursor), 1, key)
                    left, _top, right, _bottom = {
                        "steering": app.steering_chart_renderer,
                        "trajectory": app.trajectory_chart_renderer,
                        "velocity": app.velocity_chart_renderer,
                    }[key].plot_bounds()
                    expected_x = left + (right - left) * sample.time_s / app.run_data.duration_s
                    self.assertAlmostEqual(canvas.coords(cursor[0])[0], expected_x)

            app.toggle_panel_expansion("velocity")
            root.update()
            app.scrub("240")
            root.update()
            self.assertIn(f"Time {app.run_data.samples[240].time_s:.3f} s",
                          app.timeline_inspection_var.get())
            self.assertEqual(len(app.velocity_chart_canvas.find_withtag("time-cursor")), 1)
            app.timeline.event_generate("<ButtonRelease-1>", x=10, y=slider_y)
            root.update()
            selected_sample = app.run_data.samples[app.playback.index]
            self.assertIn(f"Time {selected_sample.time_s:.3f} s",
                          app.timeline_inspection_var.get())
            self.assertEqual(app.telemetry_steering_var.get(),
                             f"SWA {selected_sample.steering_command_deg:+.2f}°")
            for canvas in app.chart_canvases.values():
                canvas.event_generate("<Motion>", x=180, y=110)
                root.update()
                self.assertFalse(canvas.find_withtag("chart-tooltip"))

            app.mode_var.set("ELK")
            app.elk_scenario_var.set("Oncoming Vehicle")
            app._scenario_selection_changed()
            root.update()
            self.assertEqual(app.run_data.mode, "ELK")
            app.timeline.event_generate("<ButtonPress-1>", x=10, y=slider_y)
            last_index = len(app.run_data.samples) - 1
            app.scrub(str(last_index))
            root.update()
            final_sample = app.run_data.samples[last_index]
            self.assertIn(f"Time {final_sample.time_s:.3f} s",
                          app.timeline_inspection_var.get())
            for canvas in app.chart_canvases.values():
                self.assertEqual(len(canvas.find_withtag("time-cursor")), 1)
            app.timeline.event_generate("<ButtonRelease-1>", x=10, y=slider_y)
            root.update()
            self.assertIn(f"Time {final_sample.time_s:.3f} s",
                          app.timeline_inspection_var.get())
            app._jump_to_frame(10)
            root.update()
            self.assertIn(f"Time {app.run_data.samples[10].time_s:.3f} s",
                          app.timeline_inspection_var.get())
        finally:
            if app is not None:
                app.close()
            else:
                root.destroy()


if __name__ == "__main__":
    unittest.main()
