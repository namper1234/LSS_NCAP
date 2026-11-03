"""Protocol v4.2 overtaking impact alignment, independent timing, and swept contact."""
from __future__ import annotations

import math
import tkinter as tk
import unittest
from dataclasses import replace
from unittest.mock import patch

from lss_ncap.cli import main
from lss_ncap.collision import (first_collision, rectangles_intersect,
                                vehicle_rectangle, vehicles_intersect)
from lss_ncap.config import SimulationConfig, VehicleParams
from lss_ncap.scenarios.elk import ElkParameters
from lss_ncap.simulation import SimulationEngine, TargetVehicleSample
from lss_ncap.ui.app import SimulationApp
from lss_ncap.ui.telemetry import telemetry_view


class OvertakingGeometryTests(unittest.TestCase):
    def test_required_72_72_and_72_80_nominal_impact_alignment(self) -> None:
        engine = SimulationEngine(SimulationConfig(speed_mps=20))
        for target_speed in (72, 80):
            parameters = ElkParameters(radius_m=1200, gvt_speed_kph=target_speed,
                                       gvt_marker_distance_m=1.5,
                                       gvt_length_m=4.957, gvt_width_m=2.013,
                                       gvt_wheelbase_m=2.95, gvt_type_key="custom")
            with self.subTest(speed=target_speed):
                run = engine.run_elk("overtaking", 0.4, 1, intent="unintentional",
                                     parameters=parameters)
                nominal = run.nominal_encounter
                assert nominal is not None and run.collision_time_s is not None
                rear_x = nominal.vut_front_x_m - 2.95 * math.cos(nominal.vut_yaw_rad)
                self.assertAlmostEqual(nominal.target_x_m + 4.957 / 2, rear_x, places=10)
                self.assertAlmostEqual(run.nominal_sync_error_m, 0, places=10)
                self.assertEqual(run.target_samples[0].yaw_deg, 0)
                self.assertEqual(run.target_samples[0].speed_mps, target_speed / 3.6)
                self.assertLess(run.target_samples[0].x_m, -run.vehicle.l_front_m)
                inner_edge = run.path.line_offset_m + 2.013 / 2 - engine.road.marking_width_m / 2
                self.assertAlmostEqual(nominal.path_y_m - inner_edge, 1.5)
                self.assertAlmostEqual(
                    nominal.initial_x_m + target_speed / 3.6 * nominal.time_s,
                    nominal.target_x_m, places=10)
                for delta in (-0.001, 0.0):
                    time = nominal.time_s + delta
                    x, y, yaw, _, _ = run.path.at(time * 20)
                    reference = replace(run.samples[0], time_s=time, front_x_m=x,
                                        front_y_m=y, yaw_deg=math.degrees(yaw))
                    target = replace(run.target_samples[0], time_s=time,
                                     x_m=nominal.initial_x_m + target_speed / 3.6 * time)
                    self.assertEqual(vehicles_intersect(reference, target,
                                                       run.vehicle, run.target_vehicle), delta == 0)
                self.assertTrue(run.collision_detected)
                self.assertEqual(run.duration_s, run.collision_time_s)
                self.assertNotEqual(run.collision_time_s, nominal.time_s)
                self.assertLess(abs(run.collision_time_s - nominal.time_s), 0.001)
                self.assertFalse(vehicles_intersect(
                    run.samples[-2], run.target_samples[-2], run.vehicle, run.target_vehicle))
                self.assertTrue(vehicles_intersect(
                    run.samples[-1], run.target_samples[-1], run.vehicle, run.target_vehicle))
                self.assertIsNotNone(run.collision_point_m)
                contact_x, contact_y = run.collision_point_m
                self.assertAlmostEqual(contact_y, nominal.path_y_m - 2.013 / 2, places=7)
                self.assertAlmostEqual(contact_x, run.target_samples[-1].x_m + 4.957 / 2, places=7)
                row = run.rows()[-1]
                self.assertEqual(row["target_overlap_percent"], "")
                self.assertEqual(row["first_contact_x_m"], contact_x)
                self.assertEqual(row["first_contact_y_m"], contact_y)
                self.assertEqual(row["actual_collision_time_s"], run.collision_time_s)
                self.assertIn("Nominal Encounter Time", telemetry_view(run, 0).target)
                self.assertIn("Actual Collision Time", telemetry_view(run, 0).target)
                self.assertNotIn("overlap", telemetry_view(run, 0).target)
                # The final zero lateral speed represents stopping at impact, not controller jerk.
                samples = run.samples[:-1]
                self.assertLess(max(abs(b.steering_command_deg - a.steering_command_deg)
                                    for a, b in zip(samples, samples[1:])), 0.2)
                self.assertLess(max(abs(b.v_lat_mps - a.v_lat_mps)
                                    for a, b in zip(samples, samples[1:])), 0.02)
                for sample in run.samples:
                    self.assertAlmostEqual(
                        sample.reference_v_lat_mps,
                        20 * math.sin(run.path.at(20 * sample.time_s)[2]), places=12)

    def test_overtaking_ignores_oncoming_overlap_and_controller_response(self) -> None:
        engine = SimulationEngine()
        parameters = ElkParameters(gvt_speed_kph=80)
        baseline = engine.run_elk("overtaking", parameters=parameters)
        for overlap in (0, 35, 100):
            changed = engine.run_elk(
                "overtaking", parameters=replace(parameters, overlap_percent=overlap))
            self.assertEqual(baseline.nominal_encounter, changed.nominal_encounter)
            self.assertEqual(baseline.samples, changed.samples)
        controlled = engine.run_elk(
            "overtaking", parameters=parameters,
            lane_support=lambda state: math.radians(-3) if state.time_s >= 6.5 else None)
        self.assertEqual(baseline.nominal_encounter, controlled.nominal_encounter)
        self.assertEqual(baseline.target_samples[0], controlled.target_samples[0])
        self.assertNotEqual(baseline.collision_time_s, controlled.collision_time_s)

    def test_cli_rejects_overtaking_overlap(self) -> None:
        with patch("sys.stderr"), self.assertRaises(SystemExit) as error:
            main(["--headless", "--mode", "elk", "--elk-scenario", "overtaking", "--overlap", "10"])
        self.assertEqual(error.exception.code, 2)

    def test_sat_rejects_overlapping_bounding_boxes_for_rotated_near_miss(self) -> None:
        first = vehicle_rectangle(0, 0, 45, 4, 0.5)
        second = vehicle_rectangle(-0.5, 0.5, 45, 4, 0.5)
        self.assertLess(max(min(x for x, _ in first), min(x for x, _ in second)),
                        min(max(x for x, _ in first), max(x for x, _ in second)))
        self.assertFalse(rectangles_intersect(first, second))

    def test_first_contact_detects_complete_pass_between_output_samples(self) -> None:
        engine = SimulationEngine()
        seed = engine.run_lka().samples[0]
        vehicle = replace(VehicleParams(), body_length_m=1, body_width_m=1)
        ego = replace(seed, front_x_m=vehicle.l_front_m, front_y_m=0, yaw_deg=0)
        samples = (ego, replace(ego, time_s=0.025))
        targets = (TargetVehicleSample(0, -3, 0, 0, 240),
                   TargetVehicleSample(0.025, 3, 0, 0, 240))
        self.assertFalse(vehicles_intersect(samples[0], targets[0], vehicle))
        self.assertFalse(vehicles_intersect(samples[1], targets[1], vehicle))
        collision = first_collision(samples, targets, vehicle)
        assert collision is not None
        self.assertAlmostEqual(collision.time_s, 2 / 240, places=8)
        self.assertTrue(vehicles_intersect(collision.ego, collision.target, vehicle))
        self.assertAlmostEqual(collision.contact_point_m[0], -0.5, places=7)
        near_miss = tuple(replace(target, y_m=1.01) for target in targets)
        self.assertIsNone(first_collision(samples, near_miss, vehicle))


class OvertakingViewerTests(unittest.TestCase):
    def test_overlap_is_hidden_and_invalid_hidden_text_does_not_affect_overtaking(self) -> None:
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
            self.assertTrue(app.overlap_entry.master.winfo_manager())
            app.overlap_var.set("25")
            app.regenerate(show_errors=False)
            self.assertEqual(app.run_data.elk_parameters.overlap_percent, 25)
            app.elk_scenario_var.set("Overtaking Vehicle")
            app._scenario_selection_changed()
            self.assertFalse(app.overlap_entry.master.winfo_manager())
            app.overlap_var.set("not used")
            app.regenerate(show_errors=False)
            self.assertIsNone(app._generation_error)
            app._jump_to_frame(len(app.run_data.samples) - 1)
            root.update()
            texts = " ".join(app.road_canvas.itemcget(item, "text")
                             for item in app.road_canvas.find_all()
                             if app.road_canvas.type(item) == "text")
            self.assertNotIn("No-ELK encounter", texts)
            self.assertNotIn("Orange outlines:", texts)
            self.assertIn("First contact", texts)
            self.assertNotIn("overlap", texts)
            for label in ("Nominal Encounter Time", "Actual Collision Time",
                          "Longitudinal Synchronization Error"):
                self.assertIn(label, app.telemetry_target_var.get())
            app.toggle_theme()
            root.update()
            app.overlap_var.set("25")
            app.elk_scenario_var.set("Oncoming Vehicle")
            app._scenario_selection_changed()
            self.assertTrue(app.overlap_entry.master.winfo_manager())
            self.assertEqual(app.run_data.elk_parameters.overlap_percent, 25)
        finally:
            if app is None:
                root.destroy()
            else:
                app.close()


if __name__ == "__main__":
    unittest.main()
