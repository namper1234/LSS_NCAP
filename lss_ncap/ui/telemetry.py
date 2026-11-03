"""Presentation model for the live vehicle-state telemetry strip."""
from __future__ import annotations

import math
from dataclasses import dataclass

from ..simulation import SimulationRun
from ..targets import target_type


@dataclass(frozen=True, slots=True)
class TelemetryView:
    collision_status: str
    phase: str
    state: str
    steering: str
    motion: str
    target: str


def telemetry_view(run: SimulationRun, index: int) -> TelemetryView:
    sample = run.samples[index]
    at_impact = run.collision_detected and index == len(run.samples) - 1
    at_end = index == len(run.samples) - 1
    source_label = {
        "pursuit": "Robot steering",
        "passive": "Robot released",
        "lane_support": "ELK intervention",
    }.get(sample.steering_source, sample.steering_source)
    if run.elk_parameters is not None:
        source_label = ("ELK-controlled" if run.elk_intervention_time_s is not None
                        else "No ELK intervention")
    phase = f"{sample.phase} · {source_label}"
    target = ""
    if run.target_samples:
        current = run.target_samples[index]
        initial = run.target_samples[0]
        initial_vut = run.samples[0]
        initial_vut_center_x = (
            initial_vut.front_x_m
            - run.vehicle.l_front_m * math.cos(math.radians(initial_vut.yaw_deg)))
        initial_center_dx = initial.x_m - initial_vut_center_x
        ttc = "—" if sample.ttc_s is None else f"{sample.ttc_s:.2f} s"
        separation = ("—" if sample.lateral_separation_m is None
                      else f"{sample.lateral_separation_m:.2f} m")
        target = (f"GVT x {current.x_m:+.2f} · y {current.y_m:+.2f} m"
                  f" · TTC {ttc}\nInitial centre dx {initial_center_dx:+.2f} m"
                  f" · lateral gap {separation}")
        if run.elk_parameters is not None and run.elk_parameters.gvt_type_key is not None:
            profile = target_type(run.elk_parameters.gvt_type_key)
            target_name = profile.name
            target = target.replace("GVT x", f"{target_name} x", 1)
            target += (f" · {run.elk_parameters.gvt_length_m:g}"
                       f" x {run.elk_parameters.gvt_width_m:g} m")
            if profile.length_source(run.elk_parameters.gvt_length_m) == "wheelbase_plus_half_wheel_diameter":
                target += " (length estimate)"
        if run.nominal_encounter is not None and run.elk_parameters is not None:
            target += f"\nNominal Encounter Time (no ELK): {run.nominal_encounter.time_s:.6f} s"
            if at_impact and run.collision_before_nominal_s is not None:
                target += f" · {run.collision_before_nominal_s:.3f} s after impact"
            if run.scenario_key == "oncoming":
                target += (f" · overlap {run.elk_parameters.overlap_percent:g}% VUT"
                           f" ({run.nominal_encounter.overlap_m:.3f} m)")
            collision_time = ("none" if run.collision_time_s is None
                              else f"{run.collision_time_s:.6f} s")
            target += (f"\nActual Collision Time: {collision_time}"
                       f" · Longitudinal Synchronization Error:"
                       f" {run.nominal_encounter.longitudinal_error_m:+.6f} m")
        elif run.nominal_sync_time_s is not None:
            target += (
                f"\nSync {run.nominal_sync_time_s:.3f} s · "
                f"GVT front {run.nominal_gvt_front_edge_x_m:+.3f} m"
                f" / VUT rear axle {run.nominal_vut_rear_axle_x_m:+.3f} m"
                f" · Δx {run.nominal_sync_error_m:+.4f} m")
    status = ""
    if at_impact:
        status = "COLLISION DETECTED"
    elif (run.protocol_termination_time_s is not None
          and run.protocol_termination_reason is not None
          and run.protocol_termination_reason.startswith("protocol_failure")
          and sample.time_s >= run.protocol_termination_time_s):
        status = "PROTOCOL FAILURE THRESHOLD"
    elif at_end and run.termination_reason.startswith("protocol_failure"):
        status = "PROTOCOL FAILURE THRESHOLD"
    elif at_end and run.termination_reason == "elk_collision_avoided":
        status = "COLLISION AVOIDED"
    return TelemetryView(
        collision_status=status,
        phase="Impact position" if at_impact else phase,
        state=(f"{0 if at_impact else run.config.speed_mps * 3.6:g} km/h  ·  "
               f"{sample.time_s:.3f} s  ·  "
               f"Path error {sample.lateral_error_m:+.3f} m\n"
               f"d = d1 + d2 + VUT width/2: {run.initial_lane_marker_distance_m:.3f} m"),
        steering=(f"Robot {sample.robot_steering_command_deg:+.2f}° · "
                  f"EPS {sample.eps_steering_wheel_deg:+.2f}° · "
                  f"ELK {sample.elk_steering_command_deg:+.2f}°"),
        motion=(f"V_lat {sample.v_lat_mps:+.3f} m/s  ·  "
                f"Heading ψ {sample.yaw_deg:+.3f}°"),
        target=target,
    )
