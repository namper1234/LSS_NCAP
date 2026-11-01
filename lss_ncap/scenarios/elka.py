"""Compatibility exports for the earlier ELKA spelling.

The implemented Euro NCAP terminology is ELK. Existing imports from this module
remain supported, including the standalone intervention-threshold helper.
"""
from __future__ import annotations

from dataclasses import dataclass

from .elk import (ELK_INTENTIONAL_PATH, ELK_SCENARIOS, ElkPath, ElkScenario,
                  ElkScenarioDefinition, LaneChangeIntent, TargetMotion,
                  elk_definition)


@dataclass(frozen=True, slots=True)
class ElkaInterventionPolicy:
    lateral_error_trigger_m: float

    def __post_init__(self) -> None:
        if self.lateral_error_trigger_m <= 0:
            raise ValueError("ELKA trigger must be positive")

    def should_intervene(self, lateral_error_m: float) -> bool:
        return abs(lateral_error_m) >= self.lateral_error_trigger_m
