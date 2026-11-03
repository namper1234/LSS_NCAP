"""Custom ELK geometry, nominal synchronization, and viewer regression tests."""
from __future__ import annotations

import csv
import math
import tempfile
import tkinter as tk
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

from lss_ncap.cli import main
from lss_ncap.collision import ego_polygon, target_polygon, vehicles_intersect
from lss_ncap.config import SimulationConfig, VehicleParams
from lss_ncap.lookahead_tuning import TuningRequest
from lss_ncap.scenarios.elk import ElkParameters, ElkScenario, LaneChangeIntent
from lss_ncap.simulation import SimulationEngine, TargetVehicleSample
from lss_ncap.ui.app import SimulationApp
from lss_ncap.ui.telemetry import telemetry_view


class ElkConfigurationTests(unittest.TestCase):
    def test_nominal_geometry_uses_configured_dimensions_speeds_and_overlap(self) -> None:
        for scenario in (ElkScenario.ONCOMING, ElkScenario.OVERTAKING):
            for overlap in (0.0, 10.0, 35.0, 100.0):
                with self.subTest(scenario=scenario, overlap=overlap):
                    config = SimulationConfig(speed_mps=65 / 3.6, approach_s=3.2)
                    vehicle = replace(VehicleParams(), body_length_m=5.3, body_width_m=1.85)
                    parameters = ElkParameters(
                        radius_m=675, d2_m=0.37, gvt_speed_kph=89,
                        gvt_marker_distance_m=1.7, overlap_percent=overlap,
                        gvt_length_m=4.2, gvt_width_m=2.1)
                    run = SimulationEngine(config, vehicle).run_elk(
                        scenario, 0.53, parameters=parameters)
                    nominal = run.nominal_encounter
                    self.assertIsNotNone(nominal)
                    assert nominal is not None
                    if scenario == ElkScenario.ONCOMING:
                        self.assertAlmostEqual(nominal.overlap_m, overlap / 100 * vehicle.body_width_m)
                    x, y, yaw, _, _ = run.path.at(config.speed_mps * nominal.time_s)
                    self.assertAlmostEqual(nominal.vut_front_x_m, x)
                    self.assertAlmostEqual(nominal.vut_front_y_m, y)
                    pose = replace(run.samples[0], front_x_m=x, front_y_m=y,
                                   yaw_deg=math.degrees(yaw))
                    target = TargetVehicleSample(
                        nominal.time_s, nominal.target_x_m, nominal.path_y_m,
                        180 if scenario == ElkScenario.ONCOMING else 0,
                        parameters.gvt_speed_kph / 3.6)
                    assert run.target_vehicle is not None
                    vut_points = ego_polygon(pose, vehicle)
                    gvt_points = target_polygon(target, run.target_vehicle)
                    actual_overlap = (min(max(y for _, y in vut_points),
                                          max(y for _, y in gvt_points))
                                      - max(min(y for _, y in vut_points),
                                            min(y for _, y in gvt_points)))
                    self.assertAlmostEqual(actual_overlap, nominal.overlap_m, places=10)
                    if scenario == ElkScenario.ONCOMING:
                        self.assertAlmostEqual(max(x for x, _ in vut_points),
                                               min(x for x, _ in gvt_points), places=10)
                        signed_speed = -parameters.gvt_speed_kph / 3.6
                    else:
                        rear_axle_x = x - vehicle.wheelbase_m * math.cos(yaw)
                        self.assertAlmostEqual(rear_axle_x, max(x for x, _ in gvt_points),
                                               places=10)
                        signed_speed = parameters.gvt_speed_kph / 3.6
                    self.assertAlmostEqual(
                        nominal.initial_x_m + signed_speed * nominal.time_s,
                        nominal.target_x_m, places=10)
                    inner_edge = (run.path.line_offset_m + vehicle.body_width_m / 2
                                  - SimulationEngine().road.marking_width_m / 2)
                    self.assertAlmostEqual(nominal.path_y_m - inner_edge,
                                           parameters.gvt_marker_distance_m)
                    self.assertEqual(run.target_samples[0].speed_mps,
                                     parameters.gvt_speed_kph / 3.6)
                    self.assertEqual(run.path.radius_m, 675)
                    self.assertEqual(run.path.d2, 0.37)
                    self.assertTrue(run.no_intervention_samples)

    def test_nominal_encounter_is_independent_of_pursuit_and_intervention(self) -> None:
        parameters = ElkParameters(gvt_speed_kph=90, overlap_percent=23)
        config = SimulationConfig(speed_mps=18)
        baseline = SimulationEngine(config).run_elk(
            "oncoming", 0.43, parameters=parameters)
        controlled = SimulationEngine(replace(config, lookahead_time_s=1.2)).run_elk(
            "oncoming", 0.43, parameters=parameters,
            lane_support=lambda state: math.radians(-3) if state.time_s >= 6.5 else None)
        self.assertEqual(baseline.nominal_encounter, controlled.nominal_encounter)
        self.assertEqual(baseline.target_samples[0], controlled.target_samples[0])
        self.assertEqual(controlled.elk_intervention_time_s, 6.5)
        self.assertAlmostEqual(controlled.robot_release_time_s, 6.5 - 5 / 18)
        self.assertEqual(controlled.target_vehicle, baseline.target_vehicle)
        self.assertNotEqual(controlled.samples[300].front_y_m,
                            baseline.samples[300].front_y_m)
        self.assertIn("ELK-controlled", telemetry_view(controlled, 0).phase)
        for sample in controlled.samples:
            expected = config.speed_mps * math.sin(
                controlled.path.at(config.speed_mps * sample.time_s)[2])
            self.assertAlmostEqual(sample.reference_v_lat_mps, expected, places=12)
        self.assertEqual(controlled.nominal_encounter, baseline.nominal_encounter)

    def test_no_intervention_run_stops_at_first_contact_with_custom_gvt(self) -> None:
        for scenario in ("oncoming", "overtaking"):
            parameters = ElkParameters(gvt_speed_kph=86, gvt_width_m=1.7, gvt_length_m=3.8)
            run = SimulationEngine().run_elk(scenario, 0.47, parameters=parameters)
            self.assertTrue(run.collision_detected)
            self.assertEqual(run.duration_s, run.collision_time_s)
            self.assertEqual(run.termination_reason, "collision")
            self.assertFalse(vehicles_intersect(
                run.samples[-2], run.target_samples[-2], run.vehicle, run.target_vehicle))
            self.assertTrue(vehicles_intersect(
                run.samples[-1], run.target_samples[-1], run.vehicle, run.target_vehicle))
            self.assertEqual(run.samples[-1].v_lat_mps, 0)
            self.assertEqual(run.target_samples[-1].speed_mps, 0)
            self.assertIn("Nominal Encounter Time (no ELK)", telemetry_view(run, 0).target)
            self.assertIn("No ELK intervention", telemetry_view(run, 0).phase)

    def test_low_lateral_speed_is_not_limited_to_a_fixed_traffic_tail(self) -> None:
        run = SimulationEngine().run_elk(
            "oncoming", 0.08,
            parameters=ElkParameters(d2_m=0.2, gvt_marker_distance_m=2.4))
        assert run.nominal_encounter is not None
        self.assertGreater(run.nominal_encounter.time_s,
                           run.path.s2 / run.config.speed_mps + 5)
        self.assertTrue(run.collision_detected)

    def test_nontraffic_and_intentional_paths_are_configurable(self) -> None:
        for scenario, side in (("solid_line", 1), ("solid_line", -1), ("road_edge", -1)):
            run = SimulationEngine().run_elk(
                scenario, 0.77, side, parameters=ElkParameters(radius_m=930, d2_m=0.42))
            self.assertEqual(run.path.radius_m, 930)
            self.assertEqual(run.path.d2, 0.42)
            self.assertAlmostEqual(abs(run.samples[-1].reference_v_lat_mps), 0.77)
            self.assertFalse(run.target_samples)
        run = SimulationEngine().run_elk(
            "overtaking", 0.83, intent=LaneChangeIntent.INTENTIONAL,
            parameters=ElkParameters(radius_m=555, gvt_speed_kph=95))
        self.assertEqual(run.path.radius_m, 555)
        self.assertEqual(run.intent, "intentional")

    def test_invalid_or_infeasible_geometry_is_explicitly_rejected(self) -> None:
        for changes in ({"overlap_percent": -1}, {"overlap_percent": 101},
                        {"overlap_percent": math.nan}, {"gvt_speed_kph": 0},
                        {"gvt_marker_distance_m": -1}, {"gvt_width_m": math.inf},
                        {"d2_m": -1}, {"radius_m": 0}, {"gvt_length_m": 0}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                ElkParameters(**changes)
        with self.assertRaisesRegex(ValueError, "exceeds the GVT width"):
            SimulationEngine().run_elk(
                "oncoming", parameters=ElkParameters(overlap_percent=100, gvt_width_m=1))
        with self.assertRaisesRegex(ValueError, "Overtaking requires"):
            SimulationEngine().run_elk(
                "overtaking", parameters=ElkParameters(gvt_speed_kph=50))
        with self.assertRaises(ValueError):
            SimulationEngine().run_elk("oncoming", 25)

    def test_request_and_cli_export_nominal_geometry_and_reference_velocity(self) -> None:
        parameters = ElkParameters(radius_m=675, d2_m=0.37, gvt_speed_kph=89,
                                   overlap_percent=25, gvt_marker_distance_m=1.7)
        request = TuningRequest(
            SimulationConfig(speed_mps=65 / 3.6), VehicleParams(), "ELK", 0.53, 1,
            elk_scenario=ElkScenario.ONCOMING, elk_parameters=parameters,
            safe_termination=False)
        expected = request.run(0.5)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "elk.csv"
            with patch("builtins.print"):
                self.assertEqual(main([
                    "--headless", "--mode", "elk", "--elk-scenario", "oncoming",
                    "--vlat", "0.53", "--vut-speed", "65", "--target-speed", "89",
                    "--radius", "675", "--d2", "0.37", "--overlap", "25",
                    "--gvt-marker-distance", "1.7", "--output", str(output)]), 0)
            with output.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), len(expected.samples))
            self.assertEqual(rows[0]["nominal_reference"], "scheduled_path_no_intervention")
            self.assertAlmostEqual(float(rows[0]["nominal_overlap_m"]),
                                   0.25 * expected.vehicle.body_width_m)
            self.assertAlmostEqual(float(rows[0]["nominal_sync_time_s"]),
                                   expected.nominal_sync_time_s)
            self.assertEqual(float(rows[-1]["reference_v_lat_mps"]),
                             expected.samples[-1].reference_v_lat_mps)


class ElkConfigurationViewerTests(unittest.TestCase):
    def test_custom_traffic_inputs_regenerate_and_keep_plot_interactions(self) -> None:
        try:
            root = tk.Tk()
        except tk.TclError as error:
            self.skipTest(f"Tk display unavailable: {error}")
        app = None
        try:
            app = SimulationApp(root=root)
            app.gvt_type_var.set("4activeC2-GVT")
            app.mode_var.set("ELK")
            app.elk_scenario_var.set("Oncoming Vehicle")
            app._scenario_selection_changed()
            root.update()
            self.assertEqual(app.run_data.elk_parameters.overlap_percent, 10)
            self.assertTrue(app.traffic_settings.winfo_manager())
            root.geometry("980x780")
            root.update()
            self.assertGreaterEqual(app.road_canvas.winfo_height(), 90)
            self.assertTrue(app.vehicle_panel._compact)
            for entry in (app.gvt_offset_entry, app.overlap_entry,
                          app.gvt_length_entry, app.gvt_width_entry):
                self.assertGreater(entry.winfo_width(), 50)
                self.assertLessEqual(
                    entry.winfo_rootx() - root.winfo_rootx() + entry.winfo_width(),
                    root.winfo_width())
            root.geometry("1280x990")
            root.update()
            self.assertFalse(app.vehicle_panel._compact)
            for variable, value in (
                (app.vlat_var, "0.53"), (app.d2_var, "0.37"),
                (app.radius_var, "675"), (app.elk_speed_kph_var, "65"),
                (app.gvt_speed_kph_var, "89"), (app.gvt_marker_distance_var, "1.7"),
                (app.overlap_var, "25"), (app.gvt_type_var, "4activeC2-SUV"),
                (app.gvt_type_var, "4activeC2-SVT"),
            ):
                previous = app.run_data
                variable.set(value)
                root.after(450, root.quit)
                root.mainloop()
                self.assertIsNot(previous, app.run_data)
                self.assertIsNone(app._generation_error)
            run = app.run_data
            self.assertEqual(run.config.speed_mps, 65 / 3.6)
            self.assertEqual(run.elk_parameters.overlap_percent, 25)
            self.assertEqual(run.target_vehicle.body_width_m, 1.55)
            self.assertTrue(run.collision_detected)
            app._jump_to_frame(len(run.samples) - 1)
            root.update()
            self.assertNotIn("No-ELK encounter",
                          " ".join(app.road_canvas.itemcget(item, "text")
                                   for item in app.road_canvas.find_all()
                                   if app.road_canvas.type(item) == "text"))
            self.assertIn("Nominal Encounter Time (no ELK)", app.telemetry_target_var.get())
            self.assertTrue(app.velocity_chart_canvas.find_withtag("series:reference_v_lat_mps"))
            view = app.chart_views[2]
            view.toggle_series("reference_v_lat_mps")
            app.toggle_panel_expansion("velocity")
            app.toggle_theme()
            app.reset_chart_view("velocity")
            root.update()
            self.assertFalse(view.is_visible("reference_v_lat_mps"))
            self.assertFalse(app.velocity_chart_canvas.find_withtag("series:reference_v_lat_mps"))
            app.toggle_panel_expansion("velocity")
            app.elk_scenario_var.set("Overtaking Vehicle")
            app._scenario_selection_changed()
            root.update()
            self.assertEqual(app.run_data.scenario_key, "overtaking")
            self.assertFalse(app.overlap_entry.master.winfo_manager())
            self.assertEqual(app.overlap_var.get(), "25")
            self.assertAlmostEqual(app.run_data.nominal_sync_error_m, 0)
            app.elk_scenario_var.set("Road Edge")
            app._scenario_selection_changed()
            self.assertFalse(app.traffic_settings.winfo_manager())
            self.assertFalse(app.run_data.target_samples)
        finally:
            if app is not None:
                app.close()
            else:
                root.destroy()
