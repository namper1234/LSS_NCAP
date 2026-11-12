"""Actual VUT body-to-marking clearance, coordinate signs, and viewer updates."""
from __future__ import annotations

import math
import tkinter as tk
import unittest
from dataclasses import replace

from lss_ncap.collision import ego_polygon, lane_marker_clearance_m
from lss_ncap.config import RoadConfig, VehicleParams
from lss_ncap.simulation import SimulationEngine
from lss_ncap.ui.app import SimulationApp
from lss_ncap.ui.telemetry import telemetry_view


class LaneMarkerClearanceTests(unittest.TestCase):
    def test_initial_clearance_uses_d2_radius_speed_and_actual_marker_width(self) -> None:
        road = replace(RoadConfig(), marking_width_m=0.3)
        for side in (-1, 1):
            for radius, d2 in ((1200, 1), (650, 0.4)):
                run = SimulationEngine(road=road).run_lka(0.4, side, radius_m=radius, d2_m=d2)
                expected = radius * (1 - math.cos(math.asin(0.4 / run.config.speed_mps))) + d2 - 0.15
                self.assertAlmostEqual(run.lane_marker_clearance_m(0), expected)
                self.assertAlmostEqual(
                    run.initial_lane_marker_distance_m, expected + 0.15 + run.vehicle.body_width_m / 2)
                self.assertEqual(run.road, road)
                self.assertAlmostEqual(run.rows()[0]["vut_lane_marker_clearance_m"], expected)
                self.assertNotIn("VUT to lane marker", telemetry_view(run, 0).state)

    def test_rotated_body_clearance_and_crossing_signs(self) -> None:
        seed = SimulationEngine().run_lka().samples[0]
        vehicle = replace(VehicleParams(), body_length_m=5.3, body_width_m=1.9)
        for side in (-1, 1):
            yaw = side * math.radians(15)
            inner_y = side * 2.4
            sample = replace(seed, yaw_deg=math.degrees(yaw), front_y_m=side * 0.6)
            expected = 2.4 - (
                side * (sample.front_y_m - vehicle.l_front_m * math.sin(yaw))
                + vehicle.body_length_m / 2 * abs(math.sin(yaw))
                + vehicle.body_width_m / 2 * abs(math.cos(yaw)))
            self.assertAlmostEqual(lane_marker_clearance_m(sample, vehicle, inner_y, side), expected)
            touching = replace(sample, front_y_m=sample.front_y_m + side * expected)
            self.assertAlmostEqual(lane_marker_clearance_m(touching, vehicle, inner_y, side), 0)
            crossed = replace(touching, front_y_m=touching.front_y_m + side * 0.2)
            self.assertAlmostEqual(lane_marker_clearance_m(crossed, vehicle, inner_y, side), -0.2)
        with self.assertRaises(ValueError):
            lane_marker_clearance_m(seed, vehicle, 2.4, 0)

    def test_all_scenarios_measure_actual_not_reference_body(self) -> None:
        engine = SimulationEngine()
        runs = [engine.run_lka(0.4, side) for side in (-1, 1)]
        runs += [engine.run_elk(scenario, 0.4)
                 for scenario in ("oncoming", "overtaking", "solid_line", "road_edge")]
        for run in runs:
            for index in (0, len(run.samples) // 2, len(run.samples) - 1):
                expected = run.path.side * run.lane_marker_inner_y_m - max(
                    run.path.side * y for _, y in ego_polygon(run.samples[index], run.vehicle))
                self.assertAlmostEqual(run.lane_marker_clearance_m(index), expected)
                self.assertEqual(run.rows()[index]["vut_lane_marker_clearance_m"], expected)
            if run.road_layout == "road_edge":
                self.assertNotIn("VUT to road edge", telemetry_view(run, 0).state)
            changed = replace(run, samples=(replace(run.samples[0], front_y_m=run.path.side * 0.3),))
            self.assertEqual(changed.initial_lane_marker_distance_m, run.initial_lane_marker_distance_m)
            wider = replace(run, vehicle=replace(run.vehicle, body_width_m=run.vehicle.body_width_m + 0.4))
            self.assertAlmostEqual(wider.initial_lane_marker_distance_m,
                                   run.initial_lane_marker_distance_m + 0.2)
            self.assertAlmostEqual(changed.lane_marker_clearance_m(0),
                                   run.lane_marker_clearance_m(0) - 0.3)


class LaneMarkerViewerTests(unittest.TestCase):
    def test_clearance_updates_on_scrub_geometry_and_theme(self) -> None:
        try:
            root = tk.Tk()
        except tk.TclError as error:
            self.skipTest(f"Tk display unavailable: {error}")
        app = None
        try:
            app = SimulationApp(root=root)
            root.update()
            for index in (0, 250):
                app._jump_to_frame(index)
                root.update()
                label = f"d = d1 + d2 + VUT width/2: {app.run_data.initial_lane_marker_distance_m:.3f} m"
                self.assertNotIn("VUT to lane marker", app.telemetry_state_var.get())
                self.assertIn(label, app.telemetry_state_var.get())
                items = app.road_canvas.find_withtag("lane-marker-clearance")
                self.assertEqual(len(items), 0)
            previous = app.run_data.initial_lane_marker_distance_m
            previous_d2 = app.run_data.path.d2
            app.d2_var.set("1.7")
            app.regenerate(show_errors=False)
            self.assertAlmostEqual(app.run_data.initial_lane_marker_distance_m,
                                   previous + 1.7 - previous_d2)
            root.update()
            label = f"d = d1 + d2 + VUT width/2: {app.run_data.initial_lane_marker_distance_m:.3f} m"
            self.assertIn(label, app.telemetry_state_var.get())
            app.gvt_type_var.set("Custom GVT")
            app.mode_var.set("ELK")
            app.elk_scenario_var.set("Overtaking Vehicle")
            app._scenario_selection_changed()
            root.geometry("980x780")
            root.update()
            self.assertGreaterEqual(app.road_canvas.winfo_height(), 90)
            app.toggle_theme()
            app.toggle_panel_expansion("road")
            root.update()
            self.assertEqual(len(app.road_canvas.find_withtag("lane-marker-clearance")), 0)
        finally:
            if app is None:
                root.destroy()
            else:
                app.close()
