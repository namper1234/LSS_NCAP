"""Closed-loop pursuit and kinematic vehicle regression tests."""
from __future__ import annotations

import math
import unittest
from dataclasses import replace

from lss_ncap.config import SIMULATION, VEHICLE
from lss_ncap.dynamics import KinematicBicycleModel
from lss_ncap.scenarios.lka import NcapPath
from lss_ncap.simulation import SimulationEngine, VehicleState
from lss_ncap.steering import PurePursuitController


class PurePursuitTests(unittest.TestCase):
    def test_lookahead_combines_minimum_distance_and_time(self) -> None:
        default = PurePursuitController(VEHICLE)
        self.assertEqual(default.lookahead_m, 22.0)
        self.assertEqual(PurePursuitController(
            VEHICLE, replace(SIMULATION, speed_mps=5)).lookahead_m, 8.0)
        self.assertEqual(PurePursuitController(
            VEHICLE, replace(SIMULATION, lookahead_time_s=1.5)).lookahead_m, 30.0)

    def test_command_responds_to_position_error_and_respects_limit(self) -> None:
        pursuit = PurePursuitController(VEHICLE)
        path = NcapPath(0.4, 1)
        straight = pursuit.command(path, front_x_m=0, front_y_m=0, yaw_rad=0)
        self.assertEqual(straight.road_wheel_rad, 0.0)
        self.assertEqual(straight.lookahead_distance_m, 22.0)
        for offset in (-1.0, 1.0):
            command = pursuit.command(path, front_x_m=0,
                                      front_y_m=offset, yaw_rad=0)
            self.assertLess(command.road_wheel_rad * offset, 0)
            self.assertLessEqual(abs(command.road_wheel_rad),
                                 VEHICLE.max_road_wheel_rad)

    def test_kinematic_bicycle_yaw_and_front_speed(self) -> None:
        plant = KinematicBicycleModel(VEHICLE, SIMULATION.speed_mps)
        delta = math.radians(2)
        expected_rate = SIMULATION.speed_mps / VEHICLE.wheelbase_m * math.tan(delta)
        plant.step_substep(delta)
        self.assertAlmostEqual(plant.yaw_rate_rad_s, expected_rate)
        self.assertAlmostEqual(plant.yaw_rad, expected_rate * SIMULATION.substep_s)
        expected_front_vlat = (SIMULATION.speed_mps * math.sin(plant.yaw_rad)
                               + VEHICLE.wheelbase_m * expected_rate
                               * math.cos(plant.yaw_rad))
        self.assertAlmostEqual(plant.front_lateral_speed_mps, expected_front_vlat)

    def test_closed_loop_tracks_path_and_reaches_target_speed(self) -> None:
        run = SimulationEngine().run_lka()
        self.assertLess(run.pursuit_command_start_s, run.reference_arc_start_s)
        self.assertLess(max(abs(s.lateral_error_m) for s in run.samples), 0.05)
        self.assertAlmostEqual(run.samples[-1].v_lat_mps, 0.4, delta=0.02)
        before_arc = [s for s in run.samples if s.time_s < 5.0][-1]
        at_arc = next(s for s in run.samples if s.time_s == 5.0)
        self.assertEqual(before_arc.reference_curvature_per_m, 0.0)
        self.assertGreater(at_arc.reference_curvature_per_m, 0.0)

    def test_arc_approach_has_no_opposite_direction_steering_dip(self) -> None:
        for lookahead_time_s in (0.5, 1.1, 1.5):
            engine = SimulationEngine(replace(
                SIMULATION, lookahead_time_s=lookahead_time_s))
            for radius_m in (800.0, 1200.0):
                for side in (-1, 1):
                    with self.subTest(lookahead_time_s=lookahead_time_s,
                                      radius_m=radius_m, side=side):
                        run = engine.run_lka(side=side, radius_m=radius_m)
                        approach = [sample for sample in run.samples
                                    if sample.time_s < run.reference_arc_start_s]
                        self.assertTrue(any(sample.steering_command_deg * side > 0
                                            for sample in approach))
                        self.assertGreaterEqual(min(sample.steering_command_deg * side
                                                    for sample in approach), -1e-9)

    def test_front_lateral_speed_matches_position_derivative(self) -> None:
        run = SimulationEngine().run_lka()
        for index in (170, 200, 230, 260):
            left, right = run.samples[index - 1], run.samples[index + 1]
            derivative = ((right.front_y_m - left.front_y_m)
                          / (right.time_s - left.time_s))
            self.assertAlmostEqual(run.samples[index].v_lat_mps,
                                   derivative, delta=0.02)

    def test_direct_command_is_causal_and_handover_is_retained(self) -> None:
        def support(state: VehicleState) -> float | None:
            return 0.0 if state.time_s >= 5.5 else None

        run = SimulationEngine().run_lka(lane_support=support)
        handover = next(s for s in run.samples if s.steering_source == "lane_support")
        self.assertGreater(handover.steering_wheel_deg, 0)
        self.assertEqual(handover.steering_command_deg, 0)
        next_index = round(handover.time_s / run.config.sample_period_s) + 1
        self.assertEqual(run.samples[next_index].steering_wheel_deg, 0)
        with self.assertRaisesRegex(ValueError, "after taking control"):
            SimulationEngine().run_lka(lane_support=lambda state:
                                      0.0 if state.time_s == 0 else None)

    def test_right_departure_is_mirror_of_left(self) -> None:
        engine = SimulationEngine()
        left, right = engine.run_lka(side=1), engine.run_lka(side=-1)
        for a, b in zip(left.samples, right.samples):
            for field in ("reference_y_m", "reference_curvature_per_m",
                          "profile_curvature_per_m", "steering_command_deg",
                          "front_y_m", "yaw_deg", "v_lat_mps"):
                self.assertAlmostEqual(getattr(a, field), -getattr(b, field), places=11)


if __name__ == "__main__":
    unittest.main()
