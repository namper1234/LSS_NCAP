"""Live Tk layout regression for panel expansion and restoration."""
from __future__ import annotations

import tkinter as tk
import unittest
from tkinter import ttk

from lss_ncap.ui.app import SimulationApp


class PanelExpansionTests(unittest.TestCase):
    def test_charts_yield_space_to_road_in_short_windows(self) -> None:
        try:
            root = tk.Tk()
        except tk.TclError as error:
            self.skipTest(f"Tk display unavailable: {error}")
        app = None
        try:
            app = SimulationApp(root=root)
            root.geometry("1280x780")
            root.update()
            # Window managers may clamp a taller requested window to the
            # screen, so check the actual compact layout instead.
            self.assertLessEqual(app.steering_chart_canvas.winfo_height(), 135)
            self.assertGreaterEqual(app.road_canvas.winfo_height(), 90)
            self.assertEqual(len({canvas.winfo_height() for canvas in
                                  app.chart_canvases.values()}), 1)
        finally:
            if app is not None:
                app.close()
            else:
                root.destroy()

    def test_lookahead_defaults_are_internal_to_viewer(self) -> None:
        try:
            root = tk.Tk()
        except tk.TclError as error:
            self.skipTest(f"Tk display unavailable: {error}")
        app = None
        try:
            app = SimulationApp(root=root)
            root.update()
            self.assertFalse(hasattr(app, "mass_var"))
            self.assertEqual(app.run_data.vehicle.mass_kg, 2485.0)
            self.assertFalse(hasattr(app, "lookahead_min_var"))
            self.assertFalse(hasattr(app, "lookahead_time_var"))
            self.assertFalse(hasattr(app, "tune_button"))
            self.assertFalse(hasattr(app, "apply_tuning_button"))
            self.assertEqual(app.run_data.config.lookahead_min_m, 8.0)
            self.assertEqual(app.run_data.config.lookahead_time_s, 0.5)
            self.assertEqual(app.approach_s_var.get(), "5")
            self.assertEqual(app.run_data.config.approach_s, 5.0)
            self.assertFalse(hasattr(app, "profile_combo"))
            self.assertAlmostEqual(float(app.d2_var.get()), app.run_data.path.d2,
                                   places=6)
            self.assertEqual(app.run_data.path.radius_m, 1200.0)
            self.assertEqual(app.run_data.pursuit_controller.lookahead_m, 10.0)
            self.assertFalse(hasattr(app, "steering_info_label"))
            self.assertFalse(hasattr(app, "steering_info_var"))
            expected_titles = {
                "steering": "STEERING WHEEL ANGLE (SWA) [°]",
                "trajectory": "LATERAL TRAJECTORY [m]",
                "velocity": "LATERAL VELOCITY [m/s]",
            }
            for key, panel in app.chart_panels.items():
                toolbar = next(child for child in panel.winfo_children()
                               if isinstance(child, ttk.Frame))
                labels = [child for child in toolbar.winfo_children()
                          if isinstance(child, ttk.Label)]
                self.assertEqual(len(labels), 1)
                self.assertEqual(labels[0].cget("text"), expected_titles[key])
            app.regenerate()
            self.assertEqual(app.run_data.config.lookahead_min_m, 8.0)
            self.assertEqual(app.run_data.config.lookahead_time_s, 0.5)
            app.approach_s_var.set("3.25")
            app.regenerate()
            self.assertEqual(app.run_data.config.approach_s, 3.25)
            self.assertEqual(app.engine.config.approach_s, 3.25)
            self.assertEqual(app.run_data.reference_arc_start_s, 3.25)
            self.assertAlmostEqual(
                app.run_data.path.s0,
                app.run_data.config.speed_mps * app.run_data.config.approach_s)
            app.radius_var.set("800")
            app.d2_var.set("1.2")
            app.vlat_var.set("0.6")
            app.regenerate()
            self.assertEqual(app.run_data.path.radius_m, 800.0)
            self.assertEqual(app.run_data.path.d2, 1.2)
            self.assertAlmostEqual(app.run_data.path.d1_table,
                                   app.run_data.path.arc_y)
        finally:
            if app is not None:
                app.close()
            else:
                root.destroy()

    def test_expand_restore_buttons_preserve_dashboard_and_theme(self) -> None:
        try:
            root = tk.Tk()
        except tk.TclError as error:
            self.skipTest(f"Tk display unavailable: {error}")
        app = None
        try:
            app = SimulationApp(root=root)
            root.update()
            app.regenerate()
            root.update()
            self.assertEqual(app.engine.config.lookahead_time_s, 0.5)
            self.assertEqual(app.run_data.pursuit_controller.lookahead_m, 10.0)
            self.assertTrue(any(abs(sample.steering_command_deg) > 0.01
                                for sample in app.run_data.samples))
            normal_width = app.trajectory_chart_canvas.winfo_width()
            normal_count = len(app._normal_root_pack)
            self.assertEqual(tuple(app.chart_panels),
                             ("steering", "trajectory", "velocity"))
            self.assertEqual(len(app.chart_views), 3)
            self.assertFalse(hasattr(app, "curvature_chart_canvas"))
            widths = [panel.winfo_width() for panel in app.chart_panels.values()]
            self.assertLessEqual(max(widths) - min(widths), 8)
            for key in ("road", "steering", "trajectory", "velocity"):
                app._expand_buttons[key].invoke()
                root.update()
                self.assertEqual(app._expanded_panel, key)
                self.assertEqual(app._expand_buttons[key].label, "Restore")
                if key == "trajectory":
                    self.assertGreater(app.trajectory_chart_canvas.winfo_width(),
                                       normal_width * 2)
                    app.toggle_theme()
                    root.update()
                    self.assertEqual(app._expanded_panel, key)
                app._expand_buttons[key].invoke()
                root.update()
                self.assertIsNone(app._expanded_panel)
                self.assertEqual(app._expand_buttons[key].label, "Expand")
                self.assertEqual(len(root.pack_slaves()), normal_count)
                self.assertTrue(all(panel.winfo_manager() == "grid"
                                    for panel in app.chart_panels.values()))
        finally:
            if app is not None:
                app.close()
            else:
                root.destroy()


if __name__ == "__main__":
    unittest.main()
