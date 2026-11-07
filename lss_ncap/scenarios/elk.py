"""Euro NCAP LSS v4.2 Emergency Lane Keeping scenario definitions."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from ..config import NCAP_PATH, SIMULATION, VEHICLE, SimulationConfig
from ..targets import target_type
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

ELK_UNINTENTIONAL_PATH = {
    speed: NCAP_PATH[speed] for speed in (0.2, 0.3, 0.4, 0.5, 0.6)
}


@dataclass(frozen=True, slots=True)
class ElkExecutionTolerances:
    """Euro NCAP LSS v4.2 §7.4 execution tolerances (not pass criteria)."""

    vut_speed_kph: float = 1.0
    target_speed_kph: float = 1.0
    relative_speed_kph: float = 1.0
    path_deviation_m: float = 0.05
    lateral_speed_mps: float = 0.05
    yaw_rate_deg_s: float = 1.0
    steering_wheel_velocity_deg_s: float = 15.0
    target_lateral_deviation_m: float = 0.05
    intentional_indicator_timing_s: float = 0.5


ELK_EXECUTION_TOLERANCES = ElkExecutionTolerances()


@dataclass(frozen=True, slots=True)
class ElkParameters:
    """Exploratory ELK inputs; GVT offset is its centre to the marker inner edge."""

    radius_m: float = SIMULATION.path_radius_m
    d2_m: float = SIMULATION.lka_d2_m
    gvt_speed_kph: float = 72.0
    gvt_marker_distance_m: float = 1.5
    overlap_percent: float = 10.0
    gvt_length_m: float = VEHICLE.body_length_m
    gvt_width_m: float = VEHICLE.body_width_m
    gvt_type_key: str | None = None
    gvt_wheelbase_m: float | None = None

    def __post_init__(self) -> None:
        positive = (self.radius_m, self.gvt_speed_kph,
                    self.gvt_length_m, self.gvt_width_m)
        if not all(math.isfinite(value) and value > 0 for value in positive):
            raise ValueError("ELK radius, GVT speed, length, and width must be finite and positive")
        if not all(math.isfinite(value) and value >= 0
                   for value in (self.d2_m, self.gvt_marker_distance_m)):
            raise ValueError("ELK d2 and GVT marker distance must be finite and nonnegative")
        if not math.isfinite(self.overlap_percent) or not 0 <= self.overlap_percent <= 100:
            raise ValueError("Target overlap must be between 0 and 100 percent of VUT width")
        if self.gvt_type_key is not None:
            target = target_type(self.gvt_type_key)
            if target.is_custom:
                length, _ = target.dimensions(
                    self.gvt_length_m, width_m=self.gvt_width_m)
            else:
                # Width and wheelbase are adjustable simulation inputs.  A
                # named target still supplies their defaults and retains its
                # catalogue/confirmed overall-length rules.
                length, _ = target.dimensions(self.gvt_length_m)
            if not math.isclose(self.gvt_length_m, length):
                raise ValueError(f"{target.name} requires its catalogue length")
        wheelbase = self.resolved_gvt_wheelbase_m
        if wheelbase is not None and (
                not math.isfinite(wheelbase) or not 0 < wheelbase < self.gvt_length_m):
            raise ValueError("GVT wheelbase must be finite, positive, and less than GVT length")

    @property
    def resolved_gvt_wheelbase_m(self) -> float | None:
        if self.gvt_wheelbase_m is not None:
            return self.gvt_wheelbase_m
        if self.gvt_type_key is not None:
            target = target_type(self.gvt_type_key)
            if not target.is_custom:
                return target.wheelbase_m
        return None


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
        ElkScenario.ROAD_EDGE, "Road Edge", (-1,), tuple(ELK_UNINTENTIONAL_PATH),
        "road_edge", "Road edge"),
    ElkScenario.SOLID_LINE: ElkScenarioDefinition(
        ElkScenario.SOLID_LINE, "Solid Line", (-1, 1), tuple(ELK_UNINTENTIONAL_PATH),
        "solid_line", "Solid line"),
    ElkScenario.ONCOMING: ElkScenarioDefinition(
        ElkScenario.ONCOMING, "Oncoming Vehicle", (1,), tuple(ELK_UNINTENTIONAL_PATH),
        "oncoming", "Centre dashed line", TargetMotion.ONCOMING_72),
    ElkScenario.OVERTAKING: ElkScenarioDefinition(
        ElkScenario.OVERTAKING, "Overtaking Vehicle", (1,), tuple(ELK_UNINTENTIONAL_PATH),
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
        speeds = ((0.5, 0.6, 0.7) if intent == LaneChangeIntent.INTENTIONAL
                  else tuple(ELK_UNINTENTIONAL_PATH))
        return ElkScenarioDefinition(base.scenario, base.label, base.allowed_sides,
                                     speeds, base.road_layout, base.tested_boundary, motion)
    if intent != LaneChangeIntent.UNINTENTIONAL:
        raise ValueError("Intentional lane change only applies to ELK Overtaking")
    return base


class ElkPath(NcapPath):
    def __init__(self, lateral_speed: float, side: int,
                 scenario: ElkScenario | str,
                 intent: LaneChangeIntent | str = LaneChangeIntent.UNINTENTIONAL,
                 config: SimulationConfig = SIMULATION, *,
                 parameters: ElkParameters | None = ElkParameters()) -> None:
        self.elk_scenario = ElkScenario(scenario)
        self.intent = LaneChangeIntent(intent)
        definition = elk_definition(self.elk_scenario, intent=self.intent)
        if side not in definition.allowed_sides:
            sides = " and ".join("left" if value == 1 else "right"
                                 for value in definition.allowed_sides)
            raise ValueError(f"{definition.label} permits {sides} departure only")
        if parameters is not None:
            super().__init__(lateral_speed, side, config=config,
                             radius_m=parameters.radius_m, d2_m=parameters.d2_m)
            return
        if lateral_speed not in definition.speed_options_mps:
            allowed = ", ".join(f"{value:.1f}" for value in definition.speed_options_mps)
            raise ValueError(f"{definition.label} V_lat must be one of {allowed} m/s")
        intentional = (self.elk_scenario == ElkScenario.OVERTAKING
                       and self.intent == LaneChangeIntent.INTENTIONAL)
        super().__init__(lateral_speed, side, config=config,
                         path_parameters=(ELK_INTENTIONAL_PATH if intentional
                                          else ELK_UNINTENTIONAL_PATH),
                         radius_m=800.0 if intentional else 1200.0)
