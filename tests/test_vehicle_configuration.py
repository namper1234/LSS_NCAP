"""Configuration integration for the viewer and steering actuator."""
from __future__ import annotations

import math
import unittest
from dataclasses import replace

from lss_ncap.collision import vehicles_intersect
from lss_ncap.config import VEHICLE
from lss_ncap.simulation import SimulationEngine, TargetVehicleSample
from lss_ncap.ui.vehicle_configuration import VehicleConfiguration


class VehicleConfigurationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.base = SimulationEngine()

    def configuration(self, **changes: float | bool) -> VehicleConfiguration:
        values: dict[str, float | bool] = dict(
            wheelbase_m=self.base.vehicle.wheelbase_m,
            mass_kg=self.base.vehicle.mass_kg,
            steer_ratio=self.base.vehicle.steer_ratio,
            body_length_m=self.base.vehicle.body_length_m,
            body_width_m=self.base.vehicle.body_width_m,
            lka_radius_m=1200.0,
        )
        values.update(changes)
        return VehicleConfiguration(**values)

    def test_reference_vehicle_defaults_use_metres_and_kerb_mass(self) -> None:
        self.assertAlmostEqual(VEHICLE.wheelbase_m, 2.950)
        self.assertEqual(VEHICLE.mass_kg, 2485)
        self.assertEqual(VEHICLE.steer_ratio, 17.86)
        self.assertEqual(VEHICLE.body_length_m, 4.957)
        self.assertEqual(VEHICLE.body_width_m, 2.013)

    def test_modified_vehicle_changes_steering_and_response(self) -> None:
        baseline = self.base.run_lka()
        config = self.configuration(wheelbase_m=3.1, mass_kg=2300,
                                    steer_ratio=17, body_length_m=5.1,
                                    body_width_m=2.1)
        engine = config.build_engine(self.base)
        run = engine.run_lka(radius_m=config.lka_radius_m)
        self.assertAlmostEqual(run.vehicle.wheelbase_m, 3.1)
        self.assertAlmostEqual(run.vehicle.mass_kg, 2300)
        self.assertAlmostEqual(run.vehicle.steer_ratio, 17)
        self.assertAlmostEqual(run.vehicle.body_length_m, 5.1)
        self.assertAlmostEqual(run.vehicle.body_width_m, 2.1)
        self.assertNotEqual(run.samples[205].steering_command_deg,
                            baseline.samples[205].steering_command_deg)
        self.assertNotEqual(run.samples[210].front_y_m,
                            baseline.samples[210].front_y_m)

    def test_direct_steering_applies_command_in_first_substep(self) -> None:
        run = self.base.run_lka(
            lane_support=lambda _state: math.radians(3))
        self.assertEqual(run.samples[0].steering_wheel_deg, 0)
        self.assertAlmostEqual(run.samples[1].steering_wheel_deg, 3)
        self.assertAlmostEqual(run.samples[1].road_wheel_deg,
                               3 / run.vehicle.steer_ratio)
        self.assertAlmostEqual(run.samples[1].steering_area_deg_s,
                               3 * run.config.sample_period_s)
        self.assertGreater(run.samples[1].front_y_m, 0)

    def test_800_m_scenario_preserves_standard_path(self) -> None:
        standard = self.base.run_lka()
        tight = self.base.run_lka(radius_m=800)
        self.assertEqual(standard.path.radius_m, 1200)
        self.assertEqual(tight.path.radius_m, 800)
        self.assertEqual(tight.scenario_key, "lka_r800")
        self.assertNotEqual(standard.reference_arc_steering_deg,
                            tight.reference_arc_steering_deg)
        self.assertNotEqual(standard.samples[210].front_y_m,
                            tight.samples[210].front_y_m)

    def test_length_and_width_change_contact_geometry(self) -> None:
        ego = self.base.run_lka().samples[0]
        center_x = ego.front_x_m - VEHICLE.l_front_m
        longitudinal = TargetVehicleSample(0, center_x + 5.2, 0, 0, 0)
        lateral = TargetVehicleSample(0, center_x, 2.1, 0, 0)
        self.assertFalse(vehicles_intersect(ego, longitudinal, VEHICLE))
        self.assertTrue(vehicles_intersect(
            ego, longitudinal, replace(VEHICLE, body_length_m=5.4)))
        self.assertFalse(vehicles_intersect(ego, lateral, VEHICLE))
        self.assertTrue(vehicles_intersect(
            ego, lateral, replace(VEHICLE, body_width_m=2.2)))

    def test_invalid_vehicle_inputs_are_rejected(self) -> None:
        for changes in ({"wheelbase_m": 0}, {"mass_kg": -1},
                        {"steer_ratio": float("nan")},
                        {"body_length_m": 1}, {"body_width_m": 0}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.configuration(**changes).build_engine(self.base)


if __name__ == "__main__":
    unittest.main()
