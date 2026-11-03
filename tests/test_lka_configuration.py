"""Configurable LKA geometry, control, and automatic viewer updates."""
from __future__ import annotations

import csv
import math
import tempfile
import tkinter as tk
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

from lka_ncap_profile import generate
from lss_ncap.cli import main
from lss_ncap.config import NCAP_STANDARD, SimulationConfig, VehicleParams
from lss_ncap.lookahead_tuning import TuningRequest
from lss_ncap.scenarios.lka import NcapPath
from lss_ncap.simulation import SimulationEngine
from lss_ncap.ui.app import SimulationApp


class ConfigurableLkaTests(unittest.TestCase):
    def test_geometry_is_derived_for_arbitrary_combinations_and_both_sides(self) -> None:
        for speed in (15.0, 25.0):
            config = SimulationConfig(speed_mps=speed, approach_s=2.3)
            for radius, d2, vlat in ((150.0, 0.0, 0.37),
                                     (675.0, 0.35, 0.63),
                                     (2100.0, 1.8, 1.13)):
                for side in (-1, 1):
                    with self.subTest(speed=speed, radius=radius, d2=d2, side=side):
                        path = NcapPath(vlat, side, config=config,
                                        radius_m=radius, d2_m=d2)
                        psi = math.asin(vlat / speed)
                        d1 = radius * (1 - math.cos(psi))
                        self.assertAlmostEqual(path.psi, psi)
                        self.assertAlmostEqual(path.s1 - path.s0, radius * psi)
                        self.assertAlmostEqual(path.s2 - path.s1, d2 / math.sin(psi))
                        self.assertAlmostEqual(path.line_offset_m, d1 + d2)
                        self.assertAlmostEqual(path.at(path.s2)[1], side * (d1 + d2))
                        for boundary in (path.s0, path.s1):
                            before, after = path.at(boundary - 1e-7), path.at(boundary + 1e-7)
                            self.assertLess(math.dist(before[:2], after[:2]), 3e-7)
                            self.assertAlmostEqual(before[2], after[2], places=7)
                        for station in (0.0, path.s0, (path.s0 + path.s1) / 2,
                                        path.s1, path.s2 + 10):
                            self.assertAlmostEqual(
                                path.distance_at_x(path.at(station)[0]), station)

    def test_config_defaults_and_request_preserve_geometry(self) -> None:
        config = SimulationConfig(path_radius_m=930.0, lka_d2_m=0.45)
        default = SimulationEngine(config).run_lka(0.47)
        self.assertEqual((default.path.radius_m, default.path.d2), (930.0, 0.45))
        request = TuningRequest(config, VehicleParams(), "LKA", 0.47, -1,
                                radius_m=555.0, d2_m=1.35)
        run = request.run(0.7)
        self.assertEqual((run.path.radius_m, run.path.d2), (555.0, 1.35))
        self.assertEqual(run.config.lookahead_time_s, 0.7)
        self.assertEqual(run.scenario_key, "lka_custom")

    def test_pursuit_recalculates_steering_trajectory_and_duration(self) -> None:
        engine = SimulationEngine()
        wide = engine.run_lka(0.63, radius_m=1800, d2_m=0.3)
        tight = engine.run_lka(0.63, radius_m=650, d2_m=0.3)
        longer = engine.run_lka(0.63, radius_m=650, d2_m=1.4)
        self.assertGreater(tight.reference_arc_steering_deg, wide.reference_arc_steering_deg)
        self.assertNotEqual(tight.samples[210].steering_command_deg,
                            wide.samples[210].steering_command_deg)
        self.assertNotEqual(tight.samples[230].front_y_m, wide.samples[230].front_y_m)
        self.assertGreater(longer.duration_s, tight.duration_s)
        for run in (wide, tight, longer):
            self.assertLess(max(abs(s.lateral_error_m) for s in run.samples), 0.05)
            self.assertAlmostEqual(run.samples[-1].v_lat_mps, 0.63, delta=0.02)
            self.assertTrue(all(s.steering_source == "pursuit" for s in run.samples))
            arc = next(s for s in run.samples if s.time_s == run.config.approach_s)
            self.assertEqual(arc.reference_curvature_per_m, 1 / run.path.radius_m)

    def test_reference_velocity_matches_scheduled_geometry_and_position_derivative(self) -> None:
        for side in (-1, 1):
            for radius, d2, speed in ((350.0, 0.0, 15.0),
                                       (725.0, 1.3, 20.0),
                                       (1800.0, 0.5, 25.0)):
                with self.subTest(side=side, radius=radius, d2=d2, speed=speed):
                    config = SimulationConfig(speed_mps=speed, approach_s=2.3)
                    run = SimulationEngine(config).run_lka(
                        0.47, side, radius_m=radius, d2_m=d2)
                    arc_end_s = run.path.s1 / speed
                    for sample in run.samples:
                        arc_time = max(0.0, min(sample.time_s - config.approach_s,
                                                arc_end_s - config.approach_s))
                        expected = side * speed * math.sin(speed * arc_time / radius)
                        self.assertAlmostEqual(sample.reference_v_lat_mps, expected)
                        if sample.time_s <= config.approach_s:
                            self.assertEqual(sample.reference_v_lat_mps, 0.0)
                    self.assertAlmostEqual(run.samples[-1].reference_v_lat_mps, side * 0.47)
                    for before, sample, after in zip(
                            run.samples, run.samples[1:], run.samples[2:]):
                        if (before.time_s < config.approach_s < after.time_s
                                or before.time_s < arc_end_s < after.time_s):
                            continue
                        derivative = ((after.reference_y_m - before.reference_y_m)
                                      / (after.time_s - before.time_s))
                        self.assertAlmostEqual(sample.reference_v_lat_mps,
                                               derivative, delta=1e-5)
                    self.assertEqual(run.rows()[-1]["reference_v_lat_mps"],
                                     run.samples[-1].reference_v_lat_mps)

    def test_reference_velocity_is_independent_of_vehicle_and_controller(self) -> None:
        config = SimulationConfig()
        baseline = SimulationEngine(config).run_lka(
            0.47, radius_m=725, d2_m=1.3)
        altered_controller = SimulationEngine(
            replace(config, lookahead_time_s=1.2),
            replace(VehicleParams(), steer_ratio=12.0)).run_lka(
                0.47, radius_m=725, d2_m=1.3)
        no_steering = SimulationEngine(config).run_lka(
            0.47, radius_m=725, d2_m=1.3, lane_support=lambda _state: 0.0)
        expected = [s.reference_v_lat_mps for s in baseline.samples]
        self.assertEqual([s.reference_v_lat_mps for s in altered_controller.samples], expected)
        self.assertEqual([s.reference_v_lat_mps for s in no_steering.samples], expected)
        self.assertTrue(all(s.v_lat_mps == 0.0 for s in no_steering.samples))
        self.assertGreater(no_steering.samples[-1].reference_v_lat_mps, 0.0)
        self.assertNotEqual([s.v_lat_mps for s in baseline.samples],
                            [s.v_lat_mps for s in altered_controller.samples])

    def test_invalid_inputs_are_rejected_without_table_fallback(self) -> None:
        for value in (0, -1, math.nan, math.inf):
            with self.subTest(radius=value), self.assertRaises(ValueError):
                NcapPath(0.43, 1, radius_m=value)
        for value in (-1, math.nan, math.inf):
            with self.subTest(d2=value), self.assertRaises(ValueError):
                NcapPath(0.43, 1, d2_m=value)
        for vlat in (0, -0.1, 20, 21, math.nan, math.inf):
            with self.subTest(vlat=vlat), self.assertRaises(ValueError):
                NcapPath(vlat, 1)
        with self.assertRaises(ValueError):
            NcapPath(0.4, 1, profile=NCAP_STANDARD, d2_m=1.2)

    def test_compatibility_entry_point_accepts_custom_geometry(self) -> None:
        path, rows = generate(0.47, -1, radius_m=725, d2_m=1.3)
        self.assertEqual((path.radius_m, path.d2), (725, 1.3))
        self.assertLess(float(rows[-1]["reference_y_m"]), 0)

    def test_cli_exports_custom_geometry_and_validates_mode(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "custom.csv"
            with patch("builtins.print"):
                self.assertEqual(main(["--headless", "--vlat", "0.47",
                                       "--radius", "725", "--d2", "1.3",
                                       "--output", str(output)]), 0)
            with output.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            run = SimulationEngine().run_lka(0.47, radius_m=725, d2_m=1.3)
            self.assertEqual(len(rows), len(run.samples))
            self.assertEqual(float(rows[-1]["reference_y_m"]),
                             run.samples[-1].reference_y_m)
        for args in (["--headless", "--mode", "elk", "--ncap-profile", "standard"],
                     ["--headless", "--ncap-profile", "standard", "--d2", "1.3"]):
            with patch("sys.stderr"), self.assertRaises(SystemExit) as raised:
                main(args)
            self.assertEqual(raised.exception.code, 2)


class ConfigurableLkaViewerTests(unittest.TestCase):
    def setUp(self) -> None:
        try:
            self.root = tk.Tk()
        except tk.TclError as error:
            self.skipTest(f"Tk display unavailable: {error}")
        self.app = SimulationApp(root=self.root)
        self.root.update()

    def tearDown(self) -> None:
        if hasattr(self, "app"):
            self.app.close()

    def wait_for_regeneration(self) -> None:
        self.root.after(500, self.root.quit)
        self.root.mainloop()

    def test_edits_automatically_refresh_road_and_all_charts(self) -> None:
        app = self.app
        previous = app.run_data
        with (patch.object(app.road_renderer, "draw", wraps=app.road_renderer.draw) as road,
              patch.object(app.steering_chart_renderer, "draw",
                           wraps=app.steering_chart_renderer.draw) as steering,
              patch.object(app.trajectory_chart_renderer, "draw",
                           wraps=app.trajectory_chart_renderer.draw) as trajectory,
              patch.object(app.velocity_chart_renderer, "draw",
                           wraps=app.velocity_chart_renderer.draw) as velocity):
            app.d2_var.set("1.6")
            self.wait_for_regeneration()
            self.assertEqual(app.run_data.path.d2, 1.6)
            self.assertEqual(app.run_data.path.radius_m, previous.path.radius_m)
            self.assertGreater(app.run_data.duration_s, previous.duration_s)
            initial_reference = app.run_data.samples[210].reference_v_lat_mps
            app.radius_var.set("730")
            self.wait_for_regeneration()
            self.assertEqual(app.run_data.path.radius_m, 730)
            self.assertNotEqual(app.run_data.samples[210].reference_v_lat_mps,
                                initial_reference)
            before_speed = app.run_data
            app.speed_mps_var.set("15")
            self.wait_for_regeneration()
            self.assertEqual(app.run_data.config.speed_mps, 15)
            self.assertNotEqual(app.run_data.samples[210].reference_v_lat_mps,
                                before_speed.samples[210].reference_v_lat_mps)
            self.assertNotEqual(app.run_data.samples[210].v_lat_mps,
                                before_speed.samples[210].v_lat_mps)
            app.vlat_var.set("0.47")
            self.wait_for_regeneration()
            self.assertIsNot(app.run_data, previous)
            self.assertEqual((app.run_data.path.radius_m, app.run_data.path.d2), (730, 1.6))
            self.assertAlmostEqual(
                app.run_data.config.speed_mps * math.sin(
                    math.radians(app.run_data.path.psi_deg)), 0.47)
            for renderer in (road, steering, trajectory, velocity):
                self.assertIs(renderer.call_args.args[1], app.run_data)
            self.assertEqual(app.playback.frame_count, len(app.run_data.samples))
            self.assertEqual(app.playback.index, 0)
            self.assertIn("R 730 m", app.status_var.get())

    def test_invalid_edit_is_visible_and_recovers_without_replacing_valid_run(self) -> None:
        app = self.app
        previous = app.run_data
        with self.assertLogs("lss_ncap.ui.app", level="WARNING"):
            app.radius_var.set("")
            self.wait_for_regeneration()
        self.assertIs(app.run_data, previous)
        self.assertIn("Invalid parameters", app.status_var.get())
        app.radius_var.set("915")
        self.wait_for_regeneration()
        self.assertEqual(app.run_data.path.radius_m, 915)
        self.assertNotIn("Invalid parameters", app.status_var.get())
        app.speed_mps_var.set("15")
        self.wait_for_regeneration()
        app.mode_var.set("ELK")
        app._scenario_selection_changed()
        self.assertTrue(app.radius_entry.master.winfo_manager())
        self.assertTrue(app.d2_entry.master.winfo_manager())
        self.assertFalse(app.speed_entry.master.winfo_manager())
        self.assertEqual(app.run_data.mode, "ELK")
        self.assertEqual(app.run_data.config.speed_mps, 20)
        app.mode_var.set("LKA")
        app._scenario_selection_changed()
        self.assertEqual(app.run_data.path.radius_m, 915)
        self.assertEqual(app.run_data.path.d2, 1)
        self.assertEqual(app.run_data.config.speed_mps, 15)
