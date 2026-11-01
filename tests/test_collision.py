"""First-contact termination tests for ELK traffic scenarios."""
from __future__ import annotations

import unittest

from lss_ncap.collision import vehicles_intersect
from lss_ncap.scenarios.elk import (ElkScenario, LaneChangeIntent,
                                    TargetMotion)
from lss_ncap.simulation import SimulationEngine
from lss_ncap.ui.telemetry import telemetry_view


class CollisionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.engine = SimulationEngine()

    def test_traffic_scenarios_end_at_first_contact(self) -> None:
        cases = (
            (ElkScenario.ONCOMING, 0.4, {}),
            (ElkScenario.OVERTAKING, 0.4, {}),
            (ElkScenario.OVERTAKING, 0.5, {
                "intent": LaneChangeIntent.INTENTIONAL,
                "target_motion": TargetMotion.OVERTAKING_80,
            }),
        )
        for scenario, lateral_speed, options in cases:
            with self.subTest(scenario=scenario, options=options):
                run = self.engine.run_elk(scenario, lateral_speed, **options)
                self.assertTrue(run.collision_detected)
                self.assertEqual(run.duration_s, run.collision_time_s)
                self.assertNotAlmostEqual(
                    run.duration_s / run.config.sample_period_s,
                    round(run.duration_s / run.config.sample_period_s), places=7)
                self.assertFalse(vehicles_intersect(
                    run.samples[-2], run.target_samples[-2], run.vehicle))
                self.assertTrue(vehicles_intersect(
                    run.samples[-1], run.target_samples[-1], run.vehicle))
                self.assertEqual(run.target_samples[-1].speed_mps, 0.0)
                self.assertEqual(run.samples[-1].v_lat_mps, 0.0)
                telemetry = telemetry_view(run, len(run.samples) - 1)
                self.assertEqual(telemetry.collision_status, "COLLISION DETECTED")
                self.assertTrue(telemetry.state.startswith("0 km/h"))

    def test_nontraffic_elk_has_no_collision_state(self) -> None:
        run = self.engine.run_elk(ElkScenario.ROAD_EDGE, 0.4)
        self.assertFalse(run.collision_detected)
        self.assertIsNone(run.collision_time_s)


if __name__ == "__main__":
    unittest.main()
