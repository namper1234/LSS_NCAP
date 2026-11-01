"""Euro NCAP LSS v4.2 ELK scenario and target synchronization tests."""
from __future__ import annotations

import math
import unittest

from lss_ncap.scenarios.elk import (ElkPath, ElkScenario, LaneChangeIntent,
                                    TargetMotion, elk_definition)
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

    def test_oncoming_initial_location_and_nominal_10_percent_overlap(self) -> None:
        run = self.engine.run_elk(ElkScenario.ONCOMING, 0.4)
        self.assertEqual(len(run.target_samples), len(run.samples))
        initial, last = run.target_samples[0], run.target_samples[-1]
        self.assertEqual(initial.yaw_deg, 180.0)
        self.assertAlmostEqual(initial.speed_mps, 72 / 3.6)
        self.assertLess(last.x_m, initial.x_m)
        encounter = self.engine._target_encounter(run.path, TargetMotion.ONCOMING_72)
        assert encounter is not None
        time_s, initial_x, target_y = encounter
        distance = time_s * run.config.speed_mps
        front_x, front_y, heading, _, _ = run.path.at(distance)
        target_x = initial_x - run.config.speed_mps * time_s
        vut_front_edge = front_x + (run.vehicle.body_length_m / 2
                                    - run.vehicle.l_front_m) * math.cos(heading)
        target_front_edge = target_x - run.vehicle.body_length_m / 2
        self.assertAlmostEqual(vut_front_edge, target_front_edge, places=12)
        self.assertAlmostEqual(abs(target_y - front_y),
                               0.9 * run.vehicle.body_width_m, places=12)

    def test_overtaking_initial_location_and_longitudinal_sync(self) -> None:
        for motion, speed in ((TargetMotion.OVERTAKING_72, 72 / 3.6),
                              (TargetMotion.OVERTAKING_80, 80 / 3.6)):
            with self.subTest(motion=motion):
                run = self.engine.run_elk(ElkScenario.OVERTAKING, 0.4,
                                          target_motion=motion)
                initial, last = run.target_samples[0], run.target_samples[-1]
                self.assertAlmostEqual(initial.speed_mps, speed)
                self.assertGreater(last.x_m, initial.x_m)
                encounter = self.engine._target_encounter(run.path, motion)
                assert encounter is not None
                time_s, initial_x, target_y = encounter
                distance = time_s * run.config.speed_mps
                front_x, front_y, heading, _, _ = run.path.at(distance)
                target_x = initial_x + speed * time_s
                target_leading_edge = target_x + run.vehicle.body_length_m / 2
                vut_rear_axle = front_x - run.vehicle.wheelbase_m * math.cos(heading)
                self.assertAlmostEqual(target_leading_edge, vut_rear_axle, places=12)
                self.assertAlmostEqual(target_y, front_y, places=12)

    def test_target_positions_and_steering_profile_are_exported(self) -> None:
        run = self.engine.run_elk(ElkScenario.ONCOMING, 0.4)
        first = run.rows()[0]
        for column in ("target_x_m", "target_y_m", "target_yaw_deg",
                       "target_speed_mps", "steering_command_deg",
                       "steering_wheel_deg"):
            self.assertIn(column, first)
        self.assertEqual(first["target_x_m"], run.target_samples[0].x_m)


if __name__ == "__main__":
    unittest.main()
