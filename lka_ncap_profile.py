#!/usr/bin/env python3
"""Euro NCAP LKA steering profile and interactive two-lane viewer.

Run ``python3 lka_ncap_profile.py`` for the GUI or add ``--headless`` for CSV.
The implementation lives in the :mod:`lss_ncap` package. This entry point
keeps the original command line and commonly imported names available.
"""
from __future__ import annotations

import math
from typing import cast

from lss_ncap.cli import main
from lss_ncap.config import NCAP_PATH, SIMULATION, VehicleParams
from lss_ncap.dynamics import LateralBicycleModel
from lss_ncap.export import save_csv
from lss_ncap.scenarios.lka import NcapPath
from lss_ncap.simulation import SimulationEngine
from lss_ncap.simulation import road_lateral_speed_mps as _road_lateral_speed_mps

DT = SIMULATION.sample_period_s
SPEED_MPS = SIMULATION.speed_mps
RADIUS_M = SIMULATION.path_radius_m


def generate(lateral_speed: float = 0.4, side: int = 1,
             params: VehicleParams | None = None, *,
             radius_m: float | None = None, d2_m: float | None = None
             ) -> tuple[NcapPath, list[dict[str, float | str]]]:
    """Compatibility wrapper returning the original path-and-row pair."""
    run = SimulationEngine(vehicle=params or VehicleParams()).run_lka(
        lateral_speed, side, radius_m=radius_m, d2_m=d2_m)
    return cast(NcapPath, run.path), run.rows()


def road_lateral_speed_mps(row: dict[str, float | str],
                           params: VehicleParams | None = None) -> float:
    """Front-axle road-frame lateral speed from a legacy row mapping."""
    vehicle = params or VehicleParams()
    return _road_lateral_speed_mps(float(row["yaw_deg"]),
                                   float(row["body_lateral_speed_mps"]),
                                   yaw_rate_rad_s=math.radians(
                                       float(row["yaw_rate_deg_s"])),
                                   front_offset_m=vehicle.wheelbase_m)


def launch_viewer(initial_vlat: float, initial_side: int) -> None:
    from lss_ncap.ui.app import SimulationApp
    SimulationApp(initial_vlat, initial_side).run()


if __name__ == "__main__":
    raise SystemExit(main())
