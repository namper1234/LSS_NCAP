"""Verified target dimensions, synchronization, rendering, and selector behavior."""
from __future__ import annotations

import csv
import math
import tempfile
import tkinter as tk
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

from PIL import Image

from lss_ncap.cli import main
from lss_ncap.collision import ego_polygon, target_polygon, vehicles_intersect
from lss_ncap.scenarios.elk import ElkParameters
from lss_ncap.simulation import SimulationEngine
from lss_ncap.targets import (CUSTOM_TARGET_TYPE, DEFAULT_TARGET_TYPE, GVT_TYPES,
                              TARGET_TYPES, target_type)
from lss_ncap.ui.app import SimulationApp
from lss_ncap.visualization.geometry import RoadViewport
from lss_ncap.visualization.targets import target_image
from lss_ncap.visualization.vehicle import GVT_SPRITE_PATH, TARGET_STYLE


class TargetTypeTests(unittest.TestCase):
    def test_custom_dimensions_wheelbase_and_asset(self) -> None:
        self.assertEqual(GVT_TYPES[-1].name, "Custom GVT")
        self.assertIs(target_type("custom"), CUSTOM_TARGET_TYPE)
        with Image.open(GVT_SPRITE_PATH) as image:
            source = image.convert("RGBA")
        self.assertEqual(target_image(CUSTOM_TARGET_TYPE).tobytes(),
                         source.crop(source.getchannel("A").getbbox()).tobytes())
        for invalid in (0, -1, math.nan, math.inf, 3.8, 5):
            with self.subTest(wheelbase=invalid), self.assertRaisesRegex(ValueError, "wheelbase"):
                ElkParameters(gvt_type_key="custom", gvt_length_m=3.8,
                              gvt_width_m=1.7, gvt_wheelbase_m=invalid)
        for invalid in (0, -1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                CUSTOM_TARGET_TYPE.dimensions(3.8, width_m=invalid)
            with self.assertRaises(ValueError):
                CUSTOM_TARGET_TYPE.dimensions(invalid, width_m=1.7)
        for target in TARGET_TYPES:
            length, width = target.dimensions(1.95 if target.length_m is None else None)
            with self.assertRaisesRegex(ValueError, "wheelbase is fixed"):
                ElkParameters(gvt_type_key=target.key, gvt_length_m=length,
                              gvt_width_m=width, gvt_wheelbase_m=target.wheelbase_m + 0.01)
        for scenario in ("oncoming", "overtaking"):
            parameters = ElkParameters(
                gvt_type_key="custom", gvt_length_m=3.8, gvt_width_m=1.7,
                gvt_wheelbase_m=2.4, gvt_speed_kph=86, overlap_percent=20)
            run = SimulationEngine().run_elk(scenario, 0.47, parameters=parameters)
            numeric = SimulationEngine().run_elk(
                scenario, 0.47, parameters=replace(parameters, gvt_type_key=None))
            self.assertEqual(run.samples, numeric.samples)
            self.assertEqual(run.nominal_encounter, numeric.nominal_encounter)
            self.assertEqual(run.target_vehicle.body_length_m, 3.8)
            self.assertEqual(run.target_vehicle.body_width_m, 1.7)
            self.assertEqual(run.target_vehicle.wheelbase_m, 2.4)
            if scenario == "oncoming":
                self.assertAlmostEqual(run.nominal_encounter.overlap_m, run.vehicle.body_width_m * 0.2)
            self.assertTrue(run.collision_detected)
            self.assertTrue(vehicles_intersect(
                run.samples[-1], run.target_samples[-1], run.vehicle, run.target_vehicle))
            for changes in ({"gvt_length_m": 4.5}, {"gvt_width_m": 1.9}):
                changed = SimulationEngine().run_elk(
                    scenario, 0.47, parameters=replace(parameters, **changes))
                self.assertNotEqual(changed.nominal_encounter.initial_x_m,
                                    run.nominal_encounter.initial_x_m)
            optional = SimulationEngine().run_elk(
                scenario, 0.47, parameters=replace(parameters, gvt_wheelbase_m=None))
            self.assertEqual(optional.nominal_encounter, run.nominal_encounter)
            self.assertEqual(optional.rows()[0]["gvt_wheelbase_m"], "")

    def test_custom_cli_dimensions_and_wheelbase_export(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "custom.csv"
            command = ["--headless", "--mode", "elk", "--elk-scenario", "oncoming",
                       "--gvt-type", "custom", "--gvt-length", "3.8", "--gvt-width", "1.7",
                       "--gvt-wheelbase", "2.4", "--output", str(output)]
            with patch("builtins.print"):
                self.assertEqual(main(command), 0)
            with output.open(newline="") as stream:
                row = next(csv.DictReader(stream))
            self.assertEqual(row["gvt_type"], "custom")
            self.assertEqual(row["gvt_length_source"], "custom")
            self.assertEqual(float(row["gvt_length_m"]), 3.8)
            self.assertEqual(float(row["gvt_width_m"]), 1.7)
            self.assertEqual(float(row["gvt_wheelbase_m"]), 2.4)
            with patch("sys.stderr"), self.assertLogs("lss_ncap.cli", level="ERROR"):
                with self.assertRaises(SystemExit) as failure:
                    main(command + ["--gvt-wheelbase", "4"])
            self.assertEqual(failure.exception.code, 2)

    def test_exact_verified_dimensions_and_order(self) -> None:
        self.assertIs(DEFAULT_TARGET_TYPE, TARGET_TYPES[0])
        self.assertEqual(
            [(target.name, target.length_m, target.width_m, target.wheelbase_m)
             for target in TARGET_TYPES],
            [("4activeMC-STA", None, 0.540, 1.210),
             ("4activeMC-AMT", None, 0.675, 1.255),
             ("4activeMC-EMT", None, 0.750, 1.420),
             ("4activeC2-SVT", 2.800, 1.550, 1.800),
             ("4activeC2-GVT", 4.060, 1.800, 2.560),
             ("4activeC2-SUV", 4.350, 1.805, 2.600)])
        for target, diameter, estimate in zip(
                TARGET_TYPES[:3], (0.400, 0.533, 0.600), (1.410, 1.5215, 1.720)):
            self.assertEqual(target.length_wheel_diameter_m, diameter)
            self.assertAlmostEqual(target.default_length_m, estimate, places=12)
            self.assertAlmostEqual(target.dimensions()[0], estimate, places=12)
            self.assertEqual(target.length_source(estimate), "wheelbase_plus_half_wheel_diameter")
            self.assertIn("(estimate)", target.option_label)
        self.assertEqual(len({target_image(target).tobytes() for target in TARGET_TYPES}), 6)
        for target in TARGET_TYPES:
            self.assertIs(target_type(target.key), target)
            self.assertIn("4activesystems.at", target.source_url)
        with self.assertRaisesRegex(ValueError, "Unknown GVT"):
            target_type("unknown")

    def test_invalid_lengths_and_mismatched_presets_are_rejected(self) -> None:
        for target in TARGET_TYPES[:3]:
            with self.subTest(target=target.name):
                for length in (0, -1, math.nan, math.inf, target.wheelbase_m):
                    with self.assertRaises(ValueError):
                        target.dimensions(length)
                self.assertEqual(target.dimensions(1.95), (1.95, target.width_m))
        for target in TARGET_TYPES[3:]:
            with self.assertRaisesRegex(ValueError, "fixed"):
                target.dimensions(9)
            with self.assertRaises(ValueError):
                ElkParameters(gvt_type_key=target.key, gvt_length_m=target.length_m,
                              gvt_width_m=2)

    def test_every_target_drives_encounter_polygons_and_first_collision(self) -> None:
        initial_positions: dict[str, set[tuple[float, float]]] = {
            "oncoming": set(), "overtaking": set()}
        for scenario in initial_positions:
            for target in TARGET_TYPES:
                with self.subTest(scenario=scenario, target=target.name):
                    length, width = target.dimensions()
                    parameters = ElkParameters(
                        gvt_type_key=target.key, gvt_length_m=length,
                        gvt_width_m=width, gvt_speed_kph=86)
                    run = SimulationEngine().run_elk(scenario, 0.47, parameters=parameters)
                    self.assertEqual(run.target_vehicle.body_length_m, length)
                    self.assertEqual(run.target_vehicle.body_width_m, width)
                    self.assertEqual(run.target_vehicle.wheelbase_m, target.wheelbase_m)
                    polygon = target_polygon(run.target_samples[0], run.target_vehicle)
                    self.assertAlmostEqual(max(x for x, _ in polygon)
                                           - min(x for x, _ in polygon), length)
                    self.assertAlmostEqual(max(y for _, y in polygon)
                                           - min(y for _, y in polygon), width)
                    nominal = run.nominal_encounter
                    self.assertIsNotNone(nominal)
                    if scenario == "oncoming":
                        self.assertAlmostEqual(nominal.overlap_m, run.vehicle.body_width_m * 0.1)
                    reference_x, reference_y, reference_yaw, _, _ = run.path.at(
                        run.config.speed_mps * nominal.time_s)
                    nominal_vut = ego_polygon(replace(
                        run.samples[0], front_x_m=reference_x, front_y_m=reference_y,
                        yaw_deg=math.degrees(reference_yaw)), run.vehicle)
                    nominal_gvt = target_polygon(replace(
                        run.target_samples[0], x_m=nominal.target_x_m,
                        y_m=nominal.path_y_m), run.target_vehicle)
                    measured_overlap = (
                        min(max(y for _, y in nominal_vut), max(y for _, y in nominal_gvt))
                        - max(min(y for _, y in nominal_vut), min(y for _, y in nominal_gvt)))
                    self.assertAlmostEqual(measured_overlap, nominal.overlap_m)
                    if scenario == "oncoming":
                        self.assertAlmostEqual(max(x for x, _ in nominal_vut),
                                               min(x for x, _ in nominal_gvt))
                    else:
                        self.assertAlmostEqual(
                            reference_x - run.vehicle.wheelbase_m * math.cos(reference_yaw),
                            max(x for x, _ in nominal_gvt))
                    signed_speed = (-1 if scenario == "oncoming" else 1) * 86 / 3.6
                    self.assertAlmostEqual(
                        nominal.initial_x_m + signed_speed * nominal.time_s,
                        nominal.target_x_m)
                    initial_positions[scenario].add((nominal.initial_x_m, nominal.path_y_m))
                    self.assertTrue(run.collision_detected)
                    self.assertEqual(run.duration_s, run.collision_time_s)
                    self.assertTrue(vehicles_intersect(
                        run.samples[-1], run.target_samples[-1], run.vehicle, run.target_vehicle))
                    self.assertFalse(vehicles_intersect(
                        run.samples[-2], run.target_samples[-2], run.vehicle, run.target_vehicle))
                    for sample in run.samples:
                        self.assertAlmostEqual(
                            sample.reference_v_lat_mps,
                            run.config.speed_mps * math.sin(
                                run.path.at(run.config.speed_mps * sample.time_s)[2]))
        for positions in initial_positions.values():
            self.assertEqual(len(positions), 6)

    def test_narrow_target_rejects_impossible_overlap(self) -> None:
        target = DEFAULT_TARGET_TYPE
        with self.assertRaisesRegex(ValueError, "overlap"):
            SimulationEngine().run_elk(
                "oncoming", parameters=ElkParameters(
                    gvt_type_key=target.key, gvt_length_m=1.95,
                    gvt_width_m=target.width_m, overlap_percent=100))

    def test_named_target_cli_exports_estimated_and_overridden_lengths(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "target.csv"
            command = ["--headless", "--mode", "elk", "--elk-scenario", "oncoming",
                       "--gvt-type", "mc_sta", "--output", str(output)]
            for target, estimate in zip(TARGET_TYPES[:3], (1.410, 1.5215, 1.720)):
                with patch("builtins.print"):
                    self.assertEqual(main(command[:-4] + [
                        "--gvt-type", target.key, "--output", str(output)]), 0)
                with output.open(newline="") as stream:
                    row = next(csv.DictReader(stream))
                self.assertAlmostEqual(float(row["gvt_length_m"]), estimate, places=12)
                self.assertEqual(row["gvt_length_source"], "wheelbase_plus_half_wheel_diameter")
                self.assertEqual(float(row["gvt_length_wheel_diameter_m"]),
                                 target.length_wheel_diameter_m)
                self.assertEqual(float(row["gvt_wheelbase_m"]), target.wheelbase_m)
            with patch("builtins.print"):
                self.assertEqual(main(command + ["--gvt-length", "1.95"]), 0)
            with output.open(newline="") as stream:
                row = next(csv.DictReader(stream))
            self.assertEqual(row["gvt_type"], "mc_sta")
            self.assertEqual(row["gvt_length_source"], "user_confirmed")
            self.assertEqual(float(row["gvt_length_m"]), 1.95)
            self.assertEqual(float(row["gvt_width_m"]), 0.54)
            with patch("builtins.print"):
                self.assertEqual(main(command[:-4] + [
                    "--gvt-type", "c2_suv", "--output", str(output)]), 0)
            with output.open(newline="") as stream:
                row = next(csv.DictReader(stream))
            self.assertEqual(float(row["gvt_length_m"]), 4.35)
            self.assertEqual(float(row["gvt_width_m"]), 1.805)
            self.assertEqual(row["gvt_length_source"], "manufacturer")
            self.assertEqual(row["gvt_length_wheel_diameter_m"], "")


class TargetSelectorTests(unittest.TestCase):
    def test_custom_selector_edits_update_geometry_and_preserve_dimensions(self) -> None:
        try:
            root = tk.Tk()
        except tk.TclError as error:
            self.skipTest(f"Tk display unavailable: {error}")
        app = None
        try:
            app = SimulationApp(root=root)
            app.gvt_type_var.set("Custom GVT")
            app.mode_var.set("ELK")
            app.elk_scenario_var.set("Oncoming Vehicle")
            app._scenario_selection_changed()
            for widget in (app.gvt_length_entry, app.gvt_width_entry, app.gvt_wheelbase_entry):
                self.assertEqual(str(widget["state"]), "normal")
            for widget, name in ((app.gvt_length_entry, "GVT Length (m)"),
                                 (app.gvt_width_entry, "GVT Width (m)"),
                                 (app.gvt_wheelbase_entry, "GVT Wheelbase (m)")):
                self.assertEqual(widget.master.winfo_children()[0].cget("text"), name)
            for variable, value in ((app.gvt_length_var, "3.8"),
                                    (app.gvt_width_var, "1.7"),
                                    (app.gvt_wheelbase_var, "2.4")):
                previous = app.run_data
                variable.set(value)
                root.after(450, root.quit)
                root.mainloop()
                self.assertIsNot(app.run_data, previous)
                self.assertIsNone(app._generation_error)
            run = app.run_data
            self.assertEqual(run.target_vehicle.body_length_m, 3.8)
            self.assertEqual(run.target_vehicle.body_width_m, 1.7)
            self.assertEqual(run.target_vehicle.wheelbase_m, 2.4)
            painter = app.road_renderer.target_vehicle_painter
            self.assertIs(app.road_renderer.target_type_painters["custom"], painter)
            with patch.object(painter, "draw", wraps=painter.draw) as draw:
                app.redraw()
                self.assertEqual(draw.call_args.kwargs["length_m"], 3.8)
                self.assertEqual(draw.call_args.kwargs["width_m"], 1.7)
            app.gvt_type_var.set("4activeMC-AMT")
            self.assertEqual(float(app.gvt_wheelbase_var.get()), 1.255)
            self.assertEqual(str(app.gvt_wheelbase_entry["state"]), "readonly")
            self.assertEqual(float(app.gvt_length_var.get()), 1.5215)
            app.gvt_type_var.set("4activeC2-SUV")
            self.assertEqual(float(app.gvt_length_var.get()), 4.35)
            self.assertEqual(float(app.gvt_width_var.get()), 1.805)
            self.assertEqual(float(app.gvt_wheelbase_var.get()), 2.6)
            app.gvt_type_var.set("Custom GVT")
            self.assertEqual((app.gvt_length_var.get(), app.gvt_width_var.get(),
                              app.gvt_wheelbase_var.get()), ("3.8", "1.7", "2.4"))
            app.regenerate(show_errors=False)
            root.geometry("980x780")
            app.elk_scenario_var.set("Overtaking Vehicle")
            app._scenario_selection_changed()
            root.update()
            self.assertEqual(app.run_data.elk_parameters.gvt_type_key, "custom")
            self.assertGreaterEqual(app.road_canvas.winfo_height(), 90)
            for widget in (app.gvt_length_entry, app.gvt_width_entry, app.gvt_wheelbase_entry):
                self.assertTrue(widget.winfo_manager())
                self.assertLessEqual(
                    widget.winfo_rootx() - root.winfo_rootx() + widget.winfo_width(),
                    root.winfo_width())
            app.gvt_wheelbase_var.set("")
            app.regenerate(show_errors=False)
            self.assertIsNone(app._generation_error)
            self.assertIsNone(app.run_data.elk_parameters.resolved_gvt_wheelbase_m)
            previous = app.run_data
            app.gvt_wheelbase_var.set("5")
            with self.assertLogs("lss_ncap.ui.app", level="WARNING"):
                app.regenerate(show_errors=False)
            self.assertIs(app.run_data, previous)
            with patch("lss_ncap.ui.app.messagebox.showerror"):
                app.toggle_playback()
                self.assertFalse(app.playback.playing)
            app.gvt_wheelbase_var.set("2.4")
            app.regenerate(show_errors=False)
            app.toggle_theme()
            root.update()
            self.assertIn("Custom GVT", app.gvt_type_combo.images)
            app.mode_var.set("LKA")
            app._scenario_selection_changed()
            self.assertIsNone(app._generation_error)
            self.assertFalse(app.traffic_settings.winfo_manager())
        finally:
            if app is None:
                root.destroy()
            else:
                app.close()

    def test_image_selection_confirmations_rendering_and_stale_run_gates(self) -> None:
        try:
            root = tk.Tk()
        except tk.TclError as error:
            self.skipTest(f"Tk display unavailable: {error}")
        app = None
        try:
            app = SimulationApp(root=root)
            self.assertEqual(app.gvt_type_var.get(), DEFAULT_TARGET_TYPE.name)
            self.assertEqual(float(app.gvt_length_var.get()), 1.410)
            self.assertEqual(str(app.gvt_width_entry["state"]), "readonly")
            self.assertEqual(len(app.gvt_type_combo.images), 7)
            self.assertEqual(app.gvt_type_combo.values,
                             tuple(target.name for target in GVT_TYPES))
            self.assertEqual(float(app.gvt_wheelbase_var.get()),
                             DEFAULT_TARGET_TYPE.wheelbase_m)
            app.gvt_type_combo._toggle_popup()
            root.update()
            self.assertIsNotNone(app.gvt_type_combo._popup)
            app.gvt_type_combo._close_popup()
            app.mode_var.set("ELK")
            app.elk_scenario_var.set("Oncoming Vehicle")
            app._scenario_selection_changed()
            self.assertIsNone(app._generation_error)
            self.assertEqual(app.run_data.elk_parameters.gvt_type_key, "mc_sta")
            self.assertIn("length estimate", app.telemetry_target_var.get())
            previous = app.run_data
            app.gvt_length_var.set("")
            with self.assertLogs("lss_ncap.ui.app", level="WARNING"):
                app.regenerate(show_errors=False)
            self.assertIs(app.run_data, previous)
            self.assertIn("GVT Length (m) is required", app.status_var.get())
            with patch("lss_ncap.ui.app.messagebox.showerror") as error, patch(
                    "lss_ncap.ui.app.filedialog.asksaveasfilename") as save:
                app.toggle_playback()
                app.export_csv()
                app.export_swa()
                self.assertFalse(app.playback.playing)
                self.assertEqual(error.call_count, 3)
                save.assert_not_called()
            app.gvt_length_var.set("1.95")
            app.regenerate(show_errors=False)
            self.assertIsNone(app._generation_error)
            self.assertEqual(app.run_data.elk_parameters.gvt_type_key, "mc_sta")
            for target in TARGET_TYPES:
                app.gvt_type_var.set(target.name)
                # Exercise the actual debounce callback, not just manual regeneration.
                root.after(450, root.quit)
                root.mainloop()
                self.assertIsNone(app._generation_error)
                self.assertEqual(app.run_data.elk_parameters.gvt_type_key, target.key)
                self.assertEqual(app.run_data.target_vehicle.body_width_m, target.width_m)
                expected_length = 1.95 if target is DEFAULT_TARGET_TYPE else target.default_length_m
                self.assertAlmostEqual(app.run_data.target_vehicle.body_length_m, expected_length)
                self.assertEqual(float(app.gvt_wheelbase_var.get()), target.wheelbase_m)
                self.assertEqual(app.run_data.target_vehicle.wheelbase_m, target.wheelbase_m)
                self.assertEqual(str(app.gvt_wheelbase_entry["state"]), "readonly")
                self.assertEqual(str(app.gvt_length_entry["state"]),
                                 "normal" if target.length_m is None else "readonly")
                self.assertEqual(app.run_data.elk_parameters.overlap_percent, 10)
                self.assertEqual(app.run_data.elk_parameters.gvt_marker_distance_m, 1.5)
                painter = app.road_renderer.target_type_painters[target.key]
                self.assertEqual(painter.source.tobytes(), target_image(target).tobytes())
                with patch.object(painter, "draw", wraps=painter.draw) as draw:
                    app.redraw()
                    self.assertEqual(draw.call_args.kwargs["width_m"], target.width_m)
                app.road_canvas.delete("all")
                viewport = RoadViewport(900, 400, 0, 0, 3.5)
                outline = painter.draw(
                    app.road_canvas, viewport, center_x_m=0, center_y_m=0, yaw_deg=0,
                    length_m=app.run_data.target_vehicle.body_length_m,
                    width_m=target.width_m, style=TARGET_STYLE)
                self.assertAlmostEqual(max(y for _, y in outline) - min(y for _, y in outline),
                                       target.width_m * viewport.scale_px_per_m)
                self.assertTrue(any(app.road_canvas.type(item) == "image"
                                    for item in app.road_canvas.find_all()))
            app.gvt_type_var.set(DEFAULT_TARGET_TYPE.name)
            self.assertEqual(app.gvt_length_var.get(), "1.95")
            app.gvt_type_var.set(TARGET_TYPES[3].name)
            app.regenerate(show_errors=False)
            root.geometry("980x780")
            app.elk_scenario_var.set("Overtaking Vehicle")
            app._scenario_selection_changed()
            root.update()
            self.assertGreaterEqual(app.road_canvas.winfo_height(), 90)
            for widget in (app.gvt_type_combo, app.intent_combo, app.gvt_length_entry,
                           app.gvt_width_entry, app.gvt_wheelbase_entry,
                           app.gvt_offset_entry, app.overlap_entry):
                self.assertTrue(widget.winfo_manager())
                self.assertGreater(widget.winfo_width(), 50)
                self.assertLessEqual(
                    widget.winfo_rootx() - root.winfo_rootx() + widget.winfo_width(),
                    root.winfo_width())
            previous = app.run_data
            app.gvt_type_var.set(TARGET_TYPES[1].name)
            app.gvt_length_var.set("")
            with self.assertLogs("lss_ncap.ui.app", level="WARNING"):
                root.after(450, root.quit)
                root.mainloop()
            self.assertIs(app.run_data, previous)
            self.assertIn("GVT Length (m) is required", app._generation_error)
            self.assertEqual(app.gvt_width_var.get(), "0.675")
            app.mode_var.set("LKA")
            app._scenario_selection_changed()
            self.assertIsNone(app._generation_error)
            self.assertFalse(app.gvt_type_combo.master.winfo_manager())
            app.toggle_theme()
            root.update()
            self.assertEqual(len(app.gvt_type_combo.images), 7)
        finally:
            if app is None:
                root.destroy()
            else:
                app.close()


if __name__ == "__main__":
    unittest.main()
