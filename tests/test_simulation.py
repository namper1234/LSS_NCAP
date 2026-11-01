"""Regression tests for the physical model and public export contract."""
from __future__ import annotations

import csv
import math
import tempfile
import unittest
from pathlib import Path

from lss_ncap.config import NCAP_PATH, RoadConfig, SimulationConfig, VehicleParams
from lss_ncap.export import export_run, export_swa_profile, save_csv
from lss_ncap.scenarios.elka import ElkaInterventionPolicy
from lss_ncap.scenarios.lka import NcapPath
from lss_ncap.simulation import SimulationEngine, road_lateral_speed_mps


class SimulationTests(unittest.TestCase):
    def test_default_run_matches_existing_csv(self) -> None:
        baseline = Path(__file__).resolve().parents[1] / "lka_profile.csv"
        with baseline.open(newline="", encoding="utf-8") as file:
            saved = list(csv.DictReader(file))
        run = SimulationEngine().run_lka()
        self.assertEqual(len(saved), len(run.samples))
        self.assertEqual(saved, [
            {key: str(value) for key, value in sample.as_row().items()}
            for sample in run.samples
        ])

    def test_reference_path_reaches_selected_lateral_speed_on_both_sides(self) -> None:
        engine = SimulationEngine()
        for target in NCAP_PATH:
            for side in (-1, 1):
                with self.subTest(target=target, side=side):
                    run = engine.run_lka(target, side)
                    self.assertAlmostEqual(run.samples[0].v_lat_mps, 0.0)
                    expected = run.config.speed_mps * math.sin(
                        math.radians(run.path.psi_deg))
                    self.assertAlmostEqual(expected, target, delta=0.01)
                    self.assertAlmostEqual(run.samples[-1].reference_curvature_per_m,
                                           0.0)
                    self.assertAlmostEqual(run.samples[-1].v_lat_mps,
                                           side * target, delta=0.02)
                    self.assertAlmostEqual(
                        run.samples[-1].v_lat_mps,
                        road_lateral_speed_mps(run.samples[-1].yaw_deg,
                                               run.samples[-1].body_lateral_speed_mps,
                                               yaw_rate_rad_s=math.radians(
                                                   run.samples[-1].yaw_rate_deg_s),
                                               front_offset_m=run.vehicle.wheelbase_m),
                    )

    def test_path_is_continuous_at_phase_boundaries(self) -> None:
        path = NcapPath(0.4, 1)
        for boundary in (path.s0, path.s1):
            before = path.at(boundary - 1e-6)
            after = path.at(boundary + 1e-6)
            self.assertLess(math.dist(before[:2], after[:2]), 3e-6)

    def test_invalid_configuration_and_scenario_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            SimulationConfig(substep_s=0.007)
        with self.assertRaises(ValueError):
            VehicleParams(mass_kg=0)
        with self.assertRaises(ValueError):
            VehicleParams(mass_kg=float("nan"))
        with self.assertRaises(ValueError):
            SimulationConfig(speed_mps=float("nan"))
        with self.assertRaises(ValueError):
            SimulationConfig(lookahead_time_s=0)
        with self.assertRaises(ValueError):
            RoadConfig(marking_width_m=float("nan"))
        with self.assertRaises(ValueError):
            NcapPath(0.7, 1)
        with self.assertRaises(ValueError):
            NcapPath(0.4, 0)

    def test_export_has_v_lat_and_rejects_empty_data(self) -> None:
        run = SimulationEngine().run_lka()
        with tempfile.TemporaryDirectory() as directory:
            file = Path(directory) / "profile.csv"
            export_run(file, run)
            with file.open(newline="", encoding="utf-8") as stream:
                reader = csv.DictReader(stream)
                rows = list(reader)
            self.assertIn("v_lat_mps", reader.fieldnames)
            self.assertEqual(len(rows), len(run.samples))
            self.assertEqual(float(rows[-1]["v_lat_mps"]), run.samples[-1].v_lat_mps)
            with self.assertRaises(ValueError):
                save_csv(file, [])

    def test_swa_export_is_time_indexed_and_matches_pursuit_commands(self) -> None:
        run = SimulationEngine().run_lka()
        with tempfile.TemporaryDirectory() as directory:
            file = Path(directory) / "swa.csv"
            export_swa_profile(file, run)
            with file.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), len(run.samples))
        self.assertEqual(list(rows[0]), ["time_s", "steering_wheel_angle_deg",
                                         "steering_wheel_angle_rad"])
        for index in (0, 200, 220, len(rows) - 1):
            self.assertAlmostEqual(float(rows[index]["time_s"]),
                                   run.samples[index].time_s)
            self.assertAlmostEqual(float(rows[index]["steering_wheel_angle_rad"]),
                                   math.radians(run.samples[index].steering_command_deg))

    def test_elka_trigger_policy_is_explicit(self) -> None:
        policy = ElkaInterventionPolicy(0.3)
        self.assertFalse(policy.should_intervene(0.29))
        self.assertTrue(policy.should_intervene(-0.3))


if __name__ == "__main__":
    unittest.main()
