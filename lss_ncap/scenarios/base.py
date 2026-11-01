"""Contract shared by scenario reference paths."""
from __future__ import annotations

from typing import Protocol

PathState = tuple[float, float, float, float, str]


class ReferencePath(Protocol):
    side: int
    s2: float
    arc_y: float
    d2: float
    psi_deg: float
    d1_table: float
    radius_m: float
    s0: float
    s1: float

    @property
    def line_offset_m(self) -> float: ...

    def at(self, s: float) -> PathState: ...
    def at_x(self, x: float) -> PathState: ...
    def distance_at_x(self, x: float) -> float: ...
