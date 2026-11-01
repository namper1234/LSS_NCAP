"""Live Tk layout regression for panel expansion and restoration."""
from __future__ import annotations

import tkinter as tk
import unittest

from lss_ncap.ui.app import SimulationApp


class PanelExpansionTests(unittest.TestCase):
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
            self.assertEqual(app.run_data.config.lookahead_time_s, 1.1)
            self.assertEqual(app.run_data.pursuit_controller.lookahead_m, 22.0)
            self.assertNotIn("look-ahead", app.steering_info_var.get().lower())
            app.regenerate()
            self.assertEqual(app.run_data.config.lookahead_min_m, 8.0)
            self.assertEqual(app.run_data.config.lookahead_time_s, 1.1)
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
            self.assertEqual(app.engine.config.lookahead_time_s, 1.1)
            self.assertEqual(app.run_data.pursuit_controller.lookahead_m, 22.0)
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
