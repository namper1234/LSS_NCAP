"""Stable CSV serialization for simulation results."""
from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Mapping, Sequence

from .simulation import SimulationRun


def save_csv(filename: str | Path, rows: Sequence[Mapping[str, float | str]]) -> None:
    if not rows:
        raise ValueError("Cannot export an empty simulation")
    path = Path(filename)
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def export_run(filename: str | Path, run: SimulationRun) -> None:
    save_csv(filename, run.rows())


def export_swa_profile(filename: str | Path, run: SimulationRun) -> None:
    """Write a compact time/SWA table for external replay tools."""
    save_csv(filename, [
        {
            "time_s": sample.time_s,
            "steering_wheel_angle_deg": sample.steering_command_deg,
            "steering_wheel_angle_rad": math.radians(sample.steering_command_deg),
        }
        for sample in run.samples
    ])
