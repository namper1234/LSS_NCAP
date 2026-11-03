"""Regression checks for simulation-only Pure Pursuit look-ahead tuning."""
from __future__ import annotations

import csv
import math
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from lss_ncap.config import SIMULATION, VEHICLE
from lss_ncap.export import export_swa_profile
from lss_ncap.lookahead_tuning import (LookaheadTuner, TuningConstraints,
                                       TuningRequest, TuningWeights)
from lss_ncap.scenarios.elk import ElkScenario, LaneChangeIntent, TargetMotion


class LookaheadTuningTests(unittest.TestCase):
    def setUp(self) -> None:
        self.request = TuningRequest(SIMULATION, VEHICLE, "LKA", 0.4, 1)

    def test_range_distance_and_progress_are_bounded(self) -> None:
        progress: list[tuple[int, int, float]] = []
        result = LookaheadTuner().tune(
            self.request, lambda done, total, candidate:
            progress.append((done, total, candidate.lookahead_time_s)))
        self.assertEqual(progress[-1][0], progress[-1][1])
        self.assertEqual(len(progress), len(result.candidates))
        self.assertEqual(len({item.lookahead_time_s for item in result.candidates}),
                         len(result.candidates))
        self.assertTrue(all(0.5 <= item.lookahead_time_s <= 1.5
                            for item in result.candidates))
        self.assertIsNotNone(result.optimal)
        for candidate in result.candidates:
            self.assertAlmostEqual(candidate.lookahead_distance_m,
                                   max(SIMULATION.lookahead_min_m,
                                       SIMULATION.speed_mps * candidate.lookahead_time_s))
        minimum_limited = replace(self.request,
                                  config=replace(SIMULATION, lookahead_min_m=40.0))
        limited_result = LookaheadTuner().tune(minimum_limited)
        self.assertTrue(all(candidate.lookahead_distance_m == 40.0
                            for candidate in limited_result.candidates))

    def test_weights_change_score_and_constraints_cannot_be_traded_off(self) -> None:
        trajectory = LookaheadTuner(TuningWeights(1, 0, 0)).tune(self.request)
        smoothness = LookaheadTuner(TuningWeights(0, 0, 1)).tune(self.request)
        self.assertEqual(trajectory.optimal.lookahead_time_s, 0.5)
        self.assertEqual(smoothness.optimal.lookahead_time_s, 1.5)
        constrained = LookaheadTuner(
            TuningWeights(1, 0, 0),
            replace(TuningConstraints(), max_steering_rate_deg_s=2.0)
        ).tune(self.request)
        self.assertGreater(constrained.optimal.lookahead_time_s, 0.5)
        self.assertTrue(constrained.optimal.metrics.feasible)
        self.assertFalse(constrained.candidates[0].metrics.feasible)

    def test_reproducibility_scenario_identity_and_export_consistency(self) -> None:
        tuner = LookaheadTuner()
        first = tuner.tune(self.request)
        second = tuner.tune(self.request)
        self.assertEqual(first, second)
        selected = first.optimal
        final_run = self.request.run(selected.lookahead_time_s)
        self.assertEqual(final_run.config.lookahead_time_s,
                         selected.lookahead_time_s)
        self.assertAlmostEqual(tuner.evaluate(final_run, 0.4).metrics.score,
                               selected.metrics.score)
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / "optimized_swa.csv"
            export_swa_profile(filename, final_run)
            with filename.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), len(final_run.samples))
        for row, sample in zip(rows, final_run.samples):
            self.assertEqual(float(row["time_s"]), sample.time_s)
            self.assertEqual(float(row["steering_wheel_angle_deg"]),
                             sample.steering_command_deg)
            self.assertAlmostEqual(float(row["steering_wheel_angle_rad"]),
                                   math.radians(sample.steering_command_deg))
        elk = TuningRequest(SIMULATION, VEHICLE, "ELK", 0.4, -1,
                            elk_scenario=ElkScenario.ROAD_EDGE)
        elk_result = tuner.tune(elk)
        self.assertEqual(elk.run(elk_result.optimal.lookahead_time_s).scenario_key,
                         ElkScenario.ROAD_EDGE.value)
        for time_s in (0.5, 1.5):
            candidate_run = elk.run(time_s)
            self.assertEqual(candidate_run.vehicle, elk.vehicle)
            self.assertEqual(candidate_run.scenario_key, ElkScenario.ROAD_EDGE.value)
            self.assertEqual(candidate_run.path.side, elk.side)
            self.assertEqual(candidate_run.config.sample_period_s,
                             elk.config.sample_period_s)

    def test_invalid_inputs_and_no_feasible_candidate(self) -> None:
        with self.assertRaises(ValueError):
            TuningWeights(0, 0, 0)
        with self.assertRaises(ValueError):
            TuningConstraints(max_velocity_error_mps=float("nan"))
        with self.assertRaises(ValueError):
            TuningRequest(SIMULATION, VEHICLE, "ELK", 0.4, 1)
        with self.assertRaises(ValueError):
            self.request.run(float("inf"))
        strict = LookaheadTuner(constraints=replace(
            TuningConstraints(), max_steering_rate_deg_s=0.01))
        result = strict.tune(self.request)
        self.assertIsNone(result.optimal)
        self.assertTrue(all(not item.metrics.feasible for item in result.candidates))

    def test_all_elk_scenarios_keep_their_selected_conditions(self) -> None:
        for scenario in ElkScenario:
            with self.subTest(scenario=scenario):
                side = 1 if scenario in (ElkScenario.ONCOMING,
                                         ElkScenario.OVERTAKING) else -1
                request = TuningRequest(SIMULATION, VEHICLE, "ELK", 0.4, side,
                                        elk_scenario=scenario)
                result = LookaheadTuner().tune(request)
                self.assertIsNotNone(result.optimal)
                final_run = request.run(result.optimal.lookahead_time_s)
                self.assertEqual(final_run.scenario_key, scenario.value)
                self.assertEqual(final_run.path.side, side)
                self.assertEqual(final_run.vehicle, VEHICLE)
        intentional = TuningRequest(
            SIMULATION, VEHICLE, "ELK", 0.6, 1,
            elk_scenario=ElkScenario.OVERTAKING,
            intent=LaneChangeIntent.INTENTIONAL,
            target_motion=TargetMotion.OVERTAKING_80, elk_parameters=None)
        selected = LookaheadTuner().tune(intentional).optimal
        self.assertIsNotNone(selected)
        final_run = intentional.run(selected.lookahead_time_s)
        self.assertEqual(final_run.intent, LaneChangeIntent.INTENTIONAL.value)
        self.assertEqual(final_run.target_motion, TargetMotion.OVERTAKING_80.value)
        self.assertEqual(final_run.path.radius_m, 800.0)


if __name__ == "__main__":
    unittest.main()
