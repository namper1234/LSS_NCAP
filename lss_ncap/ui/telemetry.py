"""Presentation model for the live vehicle-state telemetry strip."""
from __future__ import annotations

from dataclasses import dataclass

from ..simulation import SimulationRun


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
    phase = f"{sample.phase} · {source_label}"
    target = ""
    if run.target_samples:
        current = run.target_samples[index]
        initial = run.target_samples[0]
        ttc = "—" if sample.ttc_s is None else f"{sample.ttc_s:.2f} s"
        separation = ("—" if sample.lateral_separation_m is None
                      else f"{sample.lateral_separation_m:.2f} m")
        target = (f"GVT x {current.x_m:+.2f} · y {current.y_m:+.2f} m"
                  f" · TTC {ttc}\nInitial x {initial.x_m:+.2f} m"
                  f" · lateral gap {separation}")
        if run.nominal_sync_time_s is not None:
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
               f"Path error {sample.lateral_error_m:+.3f} m"),
        steering=(f"Robot {sample.robot_steering_command_deg:+.2f}° · "
                  f"EPS {sample.eps_steering_wheel_deg:+.2f}° · "
                  f"ELK {sample.elk_steering_command_deg:+.2f}°"),
        motion=(f"V_lat {sample.v_lat_mps:+.3f} m/s  ·  "
                f"Heading ψ {sample.yaw_deg:+.3f}°"),
        target=target,
    )
