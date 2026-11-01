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
    phase = sample.phase + (" · Pursuit handover"
                            if sample.steering_source != "pursuit" else "")
    target = ""
    if run.target_samples:
        current = run.target_samples[index]
        initial = run.target_samples[0]
        target = (f"GVT now  x {current.x_m:+.2f} m · y {current.y_m:+.2f} m\n"
                  f"Initial  x {initial.x_m:+.2f} m · y {initial.y_m:+.2f} m")
    return TelemetryView(
        collision_status="COLLISION DETECTED" if at_impact else "",
        phase="Impact position" if at_impact else phase,
        state=(f"{0 if at_impact else run.config.speed_mps * 3.6:g} km/h  ·  "
               f"{sample.time_s:.3f} s  ·  "
               f"Path error {sample.lateral_error_m:+.3f} m"),
        steering=f"SWA {sample.steering_command_deg:+.2f}°",
        motion=(f"V_lat {sample.v_lat_mps:+.3f} m/s  ·  "
                f"Heading ψ {sample.yaw_deg:+.3f}°"),
        target=target,
    )
