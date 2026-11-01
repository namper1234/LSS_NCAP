"""Euro NCAP LSS v4.2 Emergency Lane Keeping scenario definitions."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ..config import NCAP_PATH, SIMULATION, SimulationConfig
from .lka import NcapPath


class ElkScenario(str, Enum):
    ROAD_EDGE = "road_edge"
    SOLID_LINE = "solid_line"
    ONCOMING = "oncoming"
    OVERTAKING = "overtaking"


class LaneChangeIntent(str, Enum):
    UNINTENTIONAL = "unintentional"
    INTENTIONAL = "intentional"


class TargetMotion(str, Enum):
    NONE = "none"
    ONCOMING_72 = "oncoming_72"
    OVERTAKING_72 = "overtaking_72"
    OVERTAKING_80 = "overtaking_80"


ELK_INTENTIONAL_PATH: dict[float, tuple[float, float, float]] = {
    0.5: (1.43, 0.25, 0.75),
    0.6: (1.72, 0.36, 0.60),
    0.7: (2.01, 0.49, 0.53),
}


@dataclass(frozen=True, slots=True)
class ElkScenarioDefinition:
    scenario: ElkScenario
    label: str
    allowed_sides: tuple[int, ...]
    speed_options_mps: tuple[float, ...]
    road_layout: str
    tested_boundary: str
    target_motion: TargetMotion = TargetMotion.NONE


ELK_SCENARIOS: dict[ElkScenario, ElkScenarioDefinition] = {
    ElkScenario.ROAD_EDGE: ElkScenarioDefinition(
        ElkScenario.ROAD_EDGE, "Road Edge", (-1,), tuple(NCAP_PATH),
        "road_edge", "Road edge"),
    ElkScenario.SOLID_LINE: ElkScenarioDefinition(
        ElkScenario.SOLID_LINE, "Solid Line", (-1, 1), tuple(NCAP_PATH),
        "solid_line", "Solid line"),
    ElkScenario.ONCOMING: ElkScenarioDefinition(
        ElkScenario.ONCOMING, "Oncoming Vehicle", (1,), tuple(NCAP_PATH),
        "oncoming", "Centre dashed line", TargetMotion.ONCOMING_72),
    ElkScenario.OVERTAKING: ElkScenarioDefinition(
        ElkScenario.OVERTAKING, "Overtaking Vehicle", (1,), tuple(NCAP_PATH),
        "overtaking", "Centre dashed line", TargetMotion.OVERTAKING_72),
}


def elk_definition(scenario: ElkScenario | str, *,
                   target_motion: TargetMotion | str | None = None,
                   intent: LaneChangeIntent | str = LaneChangeIntent.UNINTENTIONAL,
                   ) -> ElkScenarioDefinition:
    scenario = ElkScenario(scenario)
    intent = LaneChangeIntent(intent)
    base = ELK_SCENARIOS[scenario]
    motion = base.target_motion if target_motion is None else TargetMotion(target_motion)
    if scenario != ElkScenario.OVERTAKING and motion != base.target_motion:
        raise ValueError("Target speed selection only applies to ELK Overtaking")
    if scenario == ElkScenario.OVERTAKING:
        if motion not in (TargetMotion.OVERTAKING_72, TargetMotion.OVERTAKING_80):
            raise ValueError("Overtaking target must travel at 72 or 80 km/h")
        speeds = (0.5, 0.6, 0.7) if intent == LaneChangeIntent.INTENTIONAL else tuple(NCAP_PATH)
        return ElkScenarioDefinition(base.scenario, base.label, base.allowed_sides,
                                     speeds, base.road_layout, base.tested_boundary, motion)
    if intent != LaneChangeIntent.UNINTENTIONAL:
        raise ValueError("Intentional lane change only applies to ELK Overtaking")
    return base


class ElkPath(NcapPath):
    def __init__(self, lateral_speed: float, side: int,
                 scenario: ElkScenario | str,
                 intent: LaneChangeIntent | str = LaneChangeIntent.UNINTENTIONAL,
                 config: SimulationConfig = SIMULATION) -> None:
        self.elk_scenario = ElkScenario(scenario)
        self.intent = LaneChangeIntent(intent)
        definition = elk_definition(self.elk_scenario, intent=self.intent)
        if side not in definition.allowed_sides:
            sides = " and ".join("left" if value == 1 else "right"
                                 for value in definition.allowed_sides)
            raise ValueError(f"{definition.label} permits {sides} departure only")
        if lateral_speed not in definition.speed_options_mps:
            allowed = ", ".join(f"{value:.1f}" for value in definition.speed_options_mps)
            raise ValueError(f"{definition.label} V_lat must be one of {allowed} m/s")
        intentional = (self.elk_scenario == ElkScenario.OVERTAKING
                       and self.intent == LaneChangeIntent.INTENTIONAL)
        super().__init__(lateral_speed, side, config=config,
                         path_parameters=ELK_INTENTIONAL_PATH if intentional else NCAP_PATH,
                         radius_m=800.0 if intentional else 1200.0)
