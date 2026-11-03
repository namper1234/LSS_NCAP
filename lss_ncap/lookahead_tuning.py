"""Deterministic coarse-to-fine tuning for the existing bicycle simulation."""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Callable

from .config import (NCAP_PATH_TABLES, ROAD, RoadConfig,
                     SimulationConfig, VehicleParams)
from .scenarios.elk import ElkParameters, ElkScenario, LaneChangeIntent, TargetMotion
from .simulation import SimulationEngine, SimulationRun


@dataclass(frozen=True, slots=True)
class TuningRequest:
    config: SimulationConfig
    vehicle: VehicleParams
    mode: str
    lateral_speed_mps: float
    side: int
    path_profile: str | None = None
    radius_m: float | None = None
    elk_scenario: ElkScenario | None = None
    intent: LaneChangeIntent = LaneChangeIntent.UNINTENTIONAL
    target_motion: TargetMotion | None = None
    road: RoadConfig = ROAD
    safe_termination: bool = True
    d2_m: float | None = None
    elk_parameters: ElkParameters | None = ElkParameters()

    def __post_init__(self) -> None:
        if self.mode not in ("LKA", "ELK") or self.side not in (-1, 1):
            raise ValueError("A valid LKA or ELK mode and departure side are required")
        if not math.isfinite(self.lateral_speed_mps) or self.lateral_speed_mps <= 0:
            raise ValueError("Target lateral speed must be finite and positive")
        if self.mode == "ELK" and self.elk_scenario is None:
            raise ValueError("An ELK scenario is required")
        if (self.mode == "LKA" and self.path_profile is not None
                and self.path_profile not in NCAP_PATH_TABLES):
            raise ValueError("A valid Euro NCAP LKA path profile is required")
        if self.radius_m is not None and (not math.isfinite(self.radius_m)
                                          or self.radius_m <= 0):
            raise ValueError("Path radius must be finite and positive")
        if self.d2_m is not None and (not math.isfinite(self.d2_m) or self.d2_m < 0):
            raise ValueError("d2 must be finite and nonnegative")

    def run(self, lookahead_time_s: float) -> SimulationRun:
        if not math.isfinite(lookahead_time_s) or lookahead_time_s <= 0:
            raise ValueError("Look-ahead time must be finite and positive")
        engine = SimulationEngine(replace(self.config,
                                          lookahead_time_s=lookahead_time_s),
                                  self.vehicle, self.road)
        if self.mode == "LKA":
            return engine.run_lka(self.lateral_speed_mps, self.side,
                                  profile=self.path_profile, radius_m=self.radius_m,
                                  d2_m=self.d2_m)
        return engine.run_elk(self.elk_scenario, self.lateral_speed_mps,
                              self.side, intent=self.intent,
                              target_motion=self.target_motion,
                              safe_termination=self.safe_termination,
                              parameters=self.elk_parameters)


@dataclass(frozen=True, slots=True)
class TuningWeights:
    trajectory: float = 0.5
    velocity: float = 0.3
    smoothness: float = 0.2

    def __post_init__(self) -> None:
        values = (self.trajectory, self.velocity, self.smoothness)
        if not all(math.isfinite(value) and value >= 0 for value in values) or sum(values) <= 0:
            raise ValueError("Tuning weights must be finite, nonnegative, and nonzero")


@dataclass(frozen=True, slots=True)
class TuningConstraints:
    """Simulation screening limits, not protocol or DYNA4 pass criteria."""
    trajectory_scale_m: float = 0.05
    max_velocity_error_mps: float = 0.15
    max_timing_error_s: float = 1.5
    max_steering_rate_deg_s: float = 8.0
    max_steering_jerk_deg_s2: float = 150.0

    def __post_init__(self) -> None:
        if not all(math.isfinite(value) and value > 0 for value in (
            self.trajectory_scale_m, self.max_velocity_error_mps,
            self.max_timing_error_s,
            self.max_steering_rate_deg_s, self.max_steering_jerk_deg_s2,
        )):
            raise ValueError("Tuning constraints must be finite and positive")


@dataclass(frozen=True, slots=True)
class CandidateMetrics:
    trajectory_rmse_m: float
    velocity_error_mps: float
    timing_error_s: float
    steering_rate_rms_deg_s: float
    steering_rate_peak_deg_s: float
    steering_jerk_rms_deg_s2: float
    steering_jerk_peak_deg_s2: float
    oscillation_count: int
    evaluation_time_s: float
    violations: tuple[str, ...]
    score: float

    @property
    def feasible(self) -> bool:
        return not self.violations


@dataclass(frozen=True, slots=True)
class CandidateResult:
    lookahead_time_s: float
    lookahead_distance_m: float
    metrics: CandidateMetrics


@dataclass(frozen=True, slots=True)
class TuningResult:
    original: CandidateResult
    optimal: CandidateResult | None
    candidates: tuple[CandidateResult, ...]


ProgressCallback = Callable[[int, int, CandidateResult], None]


class LookaheadTuner:
    MIN_TIME_S = 0.5
    MAX_TIME_S = 1.5

    def __init__(self, weights: TuningWeights = TuningWeights(),
                 constraints: TuningConstraints = TuningConstraints()) -> None:
        self.weights = weights
        self.constraints = constraints

    def evaluate(self, run: SimulationRun, target_lateral_speed_mps: float
                 ) -> CandidateResult:
        """Score the arc-end event, where the reference first reaches target V_lat."""
        if not math.isfinite(target_lateral_speed_mps) or target_lateral_speed_mps <= 0:
            raise ValueError("Target lateral speed must be finite and positive")
        samples = run.samples
        event_time = run.path.s1 / run.config.speed_mps
        trajectory_end_time = run.path.s2 / run.config.speed_mps
        event_reached = event_time <= run.duration_s
        trajectory_end_reached = trajectory_end_time <= run.duration_s
        signed_target = run.path.side * target_lateral_speed_mps
        signed_speeds = [sample.v_lat_mps * run.path.side for sample in samples]
        if event_reached:
            interval = min(int(event_time / run.config.sample_period_s),
                           len(samples) - 2)
            before, after = samples[interval:interval + 2]
            fraction = ((event_time - before.time_s)
                        / (after.time_s - before.time_s))
            event_speed = before.v_lat_mps + fraction * (after.v_lat_mps - before.v_lat_mps)
            velocity_error = abs(event_speed - signed_target)
        else:
            velocity_error = math.inf
        crossing_time = next((samples[index - 1].time_s
                              + (samples[index].time_s - samples[index - 1].time_s)
                              * (target_lateral_speed_mps - signed_speeds[index - 1])
                              / (signed_speeds[index] - signed_speeds[index - 1])
                              for index in range(1, len(samples))
                              if signed_speeds[index - 1] < target_lateral_speed_mps
                              <= signed_speeds[index]), None)
        timing_error = abs(crossing_time - event_time) if crossing_time is not None else math.inf
        trajectory_samples = [sample for sample in samples
                              if sample.time_s <= trajectory_end_time]
        trajectory_rmse = math.sqrt(sum(
            (sample.front_y_m - sample.reference_y_m) ** 2
            for sample in trajectory_samples) / len(trajectory_samples))
        commands = [sample.steering_command_deg for sample in samples[:-1]]
        dt = run.config.sample_period_s
        rates = [(current - previous) / dt
                 for previous, current in zip(commands, commands[1:])]
        jerks = [(current - previous) / dt
                 for previous, current in zip(rates, rates[1:])]
        rms = lambda values: math.sqrt(sum(value * value for value in values)
                                       / len(values)) if values else 0.0
        rate_peak = max((abs(value) for value in rates), default=0.0)
        jerk_peak = max((abs(value) for value in jerks), default=0.0)
        directions = [1 if rate > 0.1 else -1 if rate < -0.1 else 0 for rate in rates]
        nonzero_directions = [direction for direction in directions if direction]
        oscillations = sum(a != b for a, b in zip(nonzero_directions,
                                                  nonzero_directions[1:]))
        limit = math.degrees(run.vehicle.max_road_wheel_rad * run.vehicle.steer_ratio)
        violations: list[str] = []
        if not event_reached:
            violations.append("NCAP arc-end event not reached")
        if not trajectory_end_reached:
            violations.append("NCAP trajectory end not reached")
        if velocity_error > self.constraints.max_velocity_error_mps:
            violations.append("target lateral-velocity error")
        if timing_error > self.constraints.max_timing_error_s:
            violations.append("target lateral-velocity timing")
        if rate_peak > self.constraints.max_steering_rate_deg_s:
            violations.append("steering rate limit")
        if jerk_peak > self.constraints.max_steering_jerk_deg_s2:
            violations.append("steering jerk limit")
        if any(abs(command) > limit + 1e-9 for command in commands):
            violations.append("steering angle limit")
        velocity_cost = (velocity_error / self.constraints.max_velocity_error_mps
                         + timing_error / self.constraints.max_timing_error_s) / 2
        smoothness_cost = (rms(rates) / self.constraints.max_steering_rate_deg_s
                           + rms(jerks) / self.constraints.max_steering_jerk_deg_s2
                           + oscillations / max(1, len(rates))) / 3
        weight_sum = (self.weights.trajectory + self.weights.velocity
                      + self.weights.smoothness)
        score = (math.inf if not math.isfinite(velocity_cost) else
                 (self.weights.trajectory * trajectory_rmse
                  / self.constraints.trajectory_scale_m
                  + self.weights.velocity * velocity_cost
                  + self.weights.smoothness * smoothness_cost) / weight_sum)
        metrics = CandidateMetrics(trajectory_rmse, velocity_error, timing_error,
                                   rms(rates), rate_peak, rms(jerks), jerk_peak,
                                   oscillations, event_time, tuple(violations), score)
        return CandidateResult(run.config.lookahead_time_s,
                               max(run.config.lookahead_min_m,
                                   run.config.speed_mps * run.config.lookahead_time_s),
                               metrics)

    def tune(self, request: TuningRequest,
             progress: ProgressCallback | None = None) -> TuningResult:
        """Search the full range, then refine around the best coarse candidate."""
        original = self.evaluate(request.run(request.config.lookahead_time_s),
                                 request.lateral_speed_mps)
        candidates: list[CandidateResult] = []
        coarse_times = [round(self.MIN_TIME_S + index * 0.1, 3)
                        for index in range(11)]
        for time_s in coarse_times:
            candidate = self.evaluate(request.run(time_s), request.lateral_speed_mps)
            candidates.append(candidate)
            if progress is not None:
                progress(len(candidates), len(coarse_times), candidate)
        coarse_best = min(candidates, key=lambda result: (
            not result.metrics.feasible, result.metrics.score, result.lookahead_time_s))
        fine_times = [round(coarse_best.lookahead_time_s + offset * 0.025, 3)
                      for offset in range(-4, 5)]
        fine_times = [time_s for time_s in fine_times
                      if self.MIN_TIME_S <= time_s <= self.MAX_TIME_S
                      and time_s not in coarse_times]
        total = len(coarse_times) + len(fine_times)
        for time_s in fine_times:
            candidate = self.evaluate(request.run(time_s), request.lateral_speed_mps)
            candidates.append(candidate)
            if progress is not None:
                progress(len(candidates), total, candidate)
        eligible = [candidate for candidate in candidates if candidate.metrics.feasible]
        if (self.MIN_TIME_S <= original.lookahead_time_s <= self.MAX_TIME_S
                and original.metrics.feasible):
            eligible.append(original)
        best = min(eligible, key=lambda result: (result.metrics.score,
                                                 result.lookahead_time_s),
                   default=None)
        return TuningResult(original, best, tuple(candidates))
