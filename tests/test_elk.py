"""Euro NCAP LSS v4.2 ELK scenario and target synchronization tests."""
from __future__ import annotations

import math
import unittest

from lss_ncap.collision import (ego_polygon, interpolate_sample,
                                target_polygon)
from lss_ncap.scenarios.elk import (ElkPath, ElkScenario, LaneChangeIntent,
                                    TargetMotion, ELK_EXECUTION_TOLERANCES,
                                    elk_definition)
from lss_ncap.simulation import SimulationEngine


class ElkScenarioTests(unittest.TestCase):
    def setUp(self) -> None:
        self.engine = SimulationEngine()

    def test_protocol_sides_and_speed_ranges(self) -> None:
        self.assertEqual(elk_definition(ElkScenario.ROAD_EDGE).allowed_sides, (-1,))
        self.assertEqual(elk_definition(ElkScenario.SOLID_LINE).allowed_sides, (-1, 1))
        self.assertEqual(elk_definition(ElkScenario.ONCOMING).allowed_sides, (1,))
        self.assertEqual(elk_definition(ElkScenario.OVERTAKING).speed_options_mps,
                         (0.2, 0.3, 0.4, 0.5, 0.6))
        intentional = elk_definition(ElkScenario.OVERTAKING,
                                     intent=LaneChangeIntent.INTENTIONAL)
        self.assertEqual(intentional.speed_options_mps, (0.5, 0.6, 0.7))
        with self.assertRaises(ValueError):
            ElkPath(0.4, 1, ElkScenario.ROAD_EDGE)
        with self.assertRaises(ValueError):
            ElkPath(0.4, -1, ElkScenario.ONCOMING)

    def test_intentional_overtaking_uses_protocol_800m_path(self) -> None:
        path = ElkPath(0.6, 1, ElkScenario.OVERTAKING,
                       LaneChangeIntent.INTENTIONAL)
        self.assertEqual(path.radius_m, 800.0)
        self.assertEqual((path.psi_deg, path.d1_table, path.d2),
                         (1.72, 0.36, 0.60))

    def test_nontraffic_scenarios_have_no_target(self) -> None:
        for scenario in (ElkScenario.ROAD_EDGE, ElkScenario.SOLID_LINE):
            run = self.engine.run_elk(scenario, 0.4)
            self.assertEqual(run.mode, "ELK")
            self.assertFalse(run.target_samples)

    def test_every_protocol_traffic_speed_reaches_a_synchronized_encounter(self) -> None:
        cases = (
            (ElkScenario.ONCOMING, LaneChangeIntent.UNINTENTIONAL,
             (TargetMotion.ONCOMING_72,)),
            (ElkScenario.OVERTAKING, LaneChangeIntent.UNINTENTIONAL,
             (TargetMotion.OVERTAKING_72, TargetMotion.OVERTAKING_80)),
            (ElkScenario.OVERTAKING, LaneChangeIntent.INTENTIONAL,
             (TargetMotion.OVERTAKING_72, TargetMotion.OVERTAKING_80)),
        )
        for scenario, intent, motions in cases:
            definition = elk_definition(scenario, intent=intent)
            for speed in definition.speed_options_mps:
                for motion in motions:
                    with self.subTest(scenario=scenario, intent=intent,
                                      speed=speed, motion=motion):
                        run = self.engine.run_elk(
                            scenario, speed, intent=intent,
                            target_motion=motion)
                        self.assertTrue(run.target_samples)
                        self.assertIsNotNone(run.predicted_collision_time_s)
                        if scenario == ElkScenario.OVERTAKING:
                            self.assertTrue(run.overtaking_sync_compliant)
                            self.assertAlmostEqual(run.nominal_sync_error_m,
                                                   0.0, places=12)
                            self.assertGreater(run.predicted_collision_time_s,
                                               run.nominal_sync_time_s)

    def test_oncoming_initial_location_and_nominal_10_percent_overlap(self) -> None:
        run = self.engine.run_elk(ElkScenario.ONCOMING, 0.4)
        self.assertEqual(len(run.target_samples), len(run.samples))
        initial, last = run.target_samples[0], run.target_samples[-1]
        self.assertEqual(initial.yaw_deg, 180.0)
        self.assertAlmostEqual(initial.speed_mps, 72 / 3.6)
        self.assertLess(last.x_m, initial.x_m)
        baseline = self.engine.run(run.path)
        encounter = self.engine._target_encounter(
            run.path, TargetMotion.ONCOMING_72, baseline)
        assert encounter is not None
        ego = self._sample_at(baseline, encounter.time_s)
        target = self.engine._target_samples(
            baseline, TargetMotion.ONCOMING_72, encounter)[0]
        target = type(target)(encounter.time_s,
                              encounter.initial_x_m - run.config.speed_mps
                              * encounter.time_s,
                              encounter.path_y_m, 180.0,
                              run.config.speed_mps)
        ego_points = ego_polygon(ego, run.vehicle)
        target_points = target_polygon(target, run.vehicle)
        self.assertAlmostEqual(max(x for x, _ in ego_points),
                               min(x for x, _ in target_points), places=10)
        ego_min, ego_max = self._y_interval(ego_points)
        target_min, target_max = self._y_interval(target_points)
        overlap = min(ego_max, target_max) - max(ego_min, target_min)
        self.assertAlmostEqual(overlap, 0.1 * run.vehicle.body_width_m,
                               places=10)
        marker_center = (run.path.line_offset_m
                         + run.vehicle.body_width_m / 2)
        marker_inner_edge = marker_center - self.engine.road.marking_width_m / 2
        self.assertAlmostEqual(encounter.path_y_m - marker_inner_edge, 1.5)

    def test_overtaking_initial_location_and_longitudinal_sync(self) -> None:
        for motion, speed in ((TargetMotion.OVERTAKING_72, 72 / 3.6),
                              (TargetMotion.OVERTAKING_80, 80 / 3.6)):
            with self.subTest(motion=motion):
                run = self.engine.run_elk(ElkScenario.OVERTAKING, 0.4,
                                          target_motion=motion)
                initial, last = run.target_samples[0], run.target_samples[-1]
                self.assertAlmostEqual(initial.speed_mps, speed)
                self.assertGreater(last.x_m, initial.x_m)
                baseline = self.engine.run(run.path)
                encounter = self.engine._target_encounter(run.path, motion,
                                                           baseline)
                assert encounter is not None
                ego = self._sample_at(baseline, encounter.time_s)
                target_x = encounter.initial_x_m + speed * encounter.time_s
                target_leading_edge = target_x + run.vehicle.body_length_m / 2
                yaw_rad = math.radians(ego.yaw_deg)
                vut_rear_axle = (ego.front_x_m - run.vehicle.wheelbase_m
                                 * math.cos(yaw_rad))
                self.assertAlmostEqual(target_leading_edge, vut_rear_axle, places=12)
                self.assertTrue(run.overtaking_sync_compliant)
                self.assertAlmostEqual(run.nominal_sync_time_s,
                                       encounter.time_s, places=12)
                self.assertAlmostEqual(run.nominal_gvt_front_edge_x_m,
                                       target_leading_edge, places=12)
                self.assertAlmostEqual(run.nominal_vut_rear_axle_x_m,
                                       vut_rear_axle, places=12)
                self.assertAlmostEqual(run.nominal_sync_error_m, 0.0, places=12)
                self.assertIsNotNone(run.predicted_collision_time_s)
                self.assertGreater(run.predicted_collision_time_s,
                                   run.nominal_sync_time_s)
                self.assertAlmostEqual(
                    run.nominal_sync_to_collision_s,
                    run.predicted_collision_time_s - run.nominal_sync_time_s,
                    places=12)
                marker_center = (run.path.line_offset_m
                                 + run.vehicle.body_width_m / 2)
                marker_inner_edge = (marker_center
                                     - self.engine.road.marking_width_m / 2)
                self.assertAlmostEqual(encounter.path_y_m - marker_inner_edge,
                                       1.5)
                expected_initial_x = (
                    vut_rear_axle - run.vehicle.body_length_m / 2
                    - speed * encounter.time_s)
                self.assertAlmostEqual(initial.x_m, expected_initial_x,
                                       places=12)

    def test_vehicle_overhangs_match_collision_rectangle_geometry(self) -> None:
        vehicle = self.engine.vehicle
        self.assertAlmostEqual(
            vehicle.l_front_m + vehicle.front_overhang_m,
            vehicle.body_length_m / 2)
        self.assertAlmostEqual(
            vehicle.l_rear_m + vehicle.rear_overhang_m,
            vehicle.body_length_m / 2)

    @staticmethod
    def _sample_at(run, time_s):
        for left, right in zip(run.samples, run.samples[1:]):
            if left.time_s <= time_s <= right.time_s:
                fraction = ((time_s - left.time_s)
                            / (right.time_s - left.time_s))
                return interpolate_sample(left, right, fraction)
        raise AssertionError("encounter is outside the baseline run")

    @staticmethod
    def _y_interval(points):
        values = [point[1] for point in points]
        return min(values), max(values)

    def test_target_positions_and_steering_profile_are_exported(self) -> None:
        run = self.engine.run_elk(ElkScenario.ONCOMING, 0.4)
        first = run.rows()[0]
        for column in ("target_x_m", "target_y_m", "target_yaw_deg",
                       "target_speed_mps", "steering_command_deg",
                       "steering_wheel_deg"):
            self.assertIn(column, first)
        self.assertEqual(first["target_x_m"], run.target_samples[0].x_m)
        overtaking = self.engine.run_elk(
            ElkScenario.OVERTAKING, 0.4, safe_termination=False)
        exported = overtaking.rows()[0]
        self.assertEqual(exported["nominal_sync_time_s"],
                         overtaking.nominal_sync_time_s)
        self.assertEqual(exported["nominal_sync_error_m"], 0.0)
        self.assertEqual(exported["first_polygon_collision_time_s"],
                         overtaking.predicted_collision_time_s)

    def test_protocol_tolerances_and_intentional_indicator_timing(self) -> None:
        tolerances = ELK_EXECUTION_TOLERANCES
        self.assertEqual(tolerances.path_deviation_m, 0.05)
        self.assertEqual(tolerances.lateral_speed_mps, 0.05)
        self.assertEqual(tolerances.relative_speed_kph, 1.0)
        run = self.engine.run_elk(
            ElkScenario.OVERTAKING, 0.5,
            intent=LaneChangeIntent.INTENTIONAL,
            target_motion=TargetMotion.OVERTAKING_80)
        self.assertAlmostEqual(run.turn_signal_time_s,
                               run.reference_arc_start_s - 1.0)

    def test_robot_releases_five_metres_before_elk_intervention(self) -> None:
        intervention_s = 6.5

        def support(state):
            return math.radians(-4.0) if state.time_s >= intervention_s else None

        run = self.engine.run_elk(ElkScenario.ONCOMING, 0.4,
                                  lane_support=support)
        self.assertAlmostEqual(run.robot_release_time_s,
                               intervention_s - 5.0 / run.config.speed_mps)
        released = [sample for sample in run.samples
                    if run.robot_release_time_s <= sample.time_s < intervention_s]
        self.assertTrue(released)
        self.assertTrue(all(sample.steering_source == "passive"
                            and sample.robot_steering_command_deg == 0.0
                            for sample in released))
        intervention = next(sample for sample in run.samples
                            if sample.time_s >= intervention_s)
        self.assertEqual(intervention.steering_source, "lane_support")
        self.assertEqual(intervention.robot_steering_command_deg, 0.0)
        self.assertAlmostEqual(intervention.elk_steering_command_deg, -4.0)
        self.assertIsNotNone(run.predicted_collision_time_s)
        self.assertEqual(run.termination_reason, "elk_collision_avoided")
        self.assertLess(run.duration_s, run.predicted_collision_time_s)


if __name__ == "__main__":
    unittest.main()
