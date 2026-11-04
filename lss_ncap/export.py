"""Stable CSV serialization for simulation results."""
from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Mapping, Sequence

from .simulation import SimulationRun
from .targets import target_type


def _run_rows(run: SimulationRun) -> list[dict[str, float | str]]:
    """Build CSV-compatible rows in the established column order."""
    rows = [sample.as_row() for sample in run.samples]
    for index, row in enumerate(rows):
        row["vut_lane_marker_clearance_m"] = run.lane_marker_clearance_m(index)
    profile = (target_type(run.elk_parameters.gvt_type_key)
               if run.elk_parameters is not None
               and run.elk_parameters.gvt_type_key is not None else None)
    if run.target_samples:
        for row, target in zip(rows, run.target_samples):
            row.update({
                "target_x_m": target.x_m,
                "target_y_m": target.y_m,
                "target_yaw_deg": target.yaw_deg,
                "target_speed_mps": target.speed_mps,
            })
            if run.elk_parameters is not None:
                row.update({
                    "target_overlap_percent": (run.elk_parameters.overlap_percent
                                               if run.scenario_key == "oncoming" else ""),
                    "gvt_marker_distance_m": run.elk_parameters.gvt_marker_distance_m,
                    "gvt_width_m": run.elk_parameters.gvt_width_m,
                    "gvt_length_m": run.elk_parameters.gvt_length_m,
                    "gvt_type": run.elk_parameters.gvt_type_key or "custom",
                    "gvt_wheelbase_m": run.elk_parameters.resolved_gvt_wheelbase_m
                    if run.elk_parameters.resolved_gvt_wheelbase_m is not None else "",
                    "gvt_length_source": (
                        "custom" if profile is None else
                        profile.length_source(run.elk_parameters.gvt_length_m)),
                    "gvt_length_wheel_diameter_m": (
                        profile.length_wheel_diameter_m
                        if profile is not None and profile.length_wheel_diameter_m is not None
                        else ""),
                    "gvt_dimension_source": profile.source_url if profile is not None else "",
                    "intervention_active": row["steering_source"] == "lane_support",
                    "trajectory_kind": ("elk_controlled" if run.elk_intervention_time_s is not None
                                        else "no_intervention"),
                    "no_intervention_collision_time_s": run.predicted_collision_time_s,
                    "actual_collision_time_s": run.collision_time_s,
                    "first_contact_x_m": (run.collision_point_m[0]
                                         if run.collision_point_m is not None else ""),
                    "first_contact_y_m": (run.collision_point_m[1]
                                         if run.collision_point_m is not None else ""),
                })
                if run.nominal_encounter is not None:
                    row.update({
                        "nominal_overlap_m": run.nominal_encounter.overlap_m,
                        "nominal_vut_front_x_m": run.nominal_encounter.vut_front_x_m,
                        "nominal_vut_front_y_m": run.nominal_encounter.vut_front_y_m,
                        "nominal_vut_yaw_deg": math.degrees(run.nominal_encounter.vut_yaw_rad),
                        "nominal_gvt_x_m": run.nominal_encounter.target_x_m,
                        "nominal_vut_reference_x_m": run.nominal_encounter.vut_reference_x_m,
                        "nominal_reference": "scheduled_path_no_intervention",
                    })
            if run.nominal_sync_time_s is not None:
                row.update({
                    "nominal_sync_time_s": run.nominal_sync_time_s,
                    "nominal_gvt_front_edge_x_m":
                        run.nominal_gvt_front_edge_x_m,
                    "nominal_vut_rear_axle_x_m":
                        run.nominal_vut_rear_axle_x_m,
                    "nominal_sync_error_m": run.nominal_sync_error_m,
                    "first_polygon_collision_time_s":
                        run.predicted_collision_time_s,
                })
    return rows


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
