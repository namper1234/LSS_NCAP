"""Closed-loop Pure Pursuit with kinematic bicycle state feedback."""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, replace
from typing import Callable

from .config import SIMULATION, VEHICLE, SimulationConfig, VehicleParams
from .dynamics import KinematicBicycleModel
from .scenarios.base import ReferencePath
from .scenarios.lka import NcapPath
from .scenarios.elk import (ElkPath, ElkScenario, LaneChangeIntent,
                            TargetMotion, elk_definition)
from .steering import PurePursuitController


@dataclass(frozen=True, slots=True)
class VehicleState:
    time_s: float
    front_x_m: float
    front_y_m: float
    yaw_rad: float
    body_lateral_speed_mps: float
    yaw_rate_rad_s: float
    steering_wheel_rad: float


# Return None while inactive, then SW radians on every tick after intervention.
# Zero is an explicit angle command; it is never used as a synonym for hands off.
LaneSupportCommand = Callable[[VehicleState], float | None]


@dataclass(frozen=True, slots=True)
class TargetVehicleSample:
    time_s: float
    x_m: float
    y_m: float
    yaw_deg: float
    speed_mps: float
    label: str = "GVT"


@dataclass(frozen=True, slots=True)
class SimulationSample:
    time_s: float
    phase: str
    front_x_m: float
    front_y_m: float
    reference_x_m: float
    reference_y_m: float
    reference_curvature_per_m: float
    profile_curvature_per_m: float
    lookahead_x_m: float
    lookahead_y_m: float
    lookahead_distance_m: float
    lateral_error_m: float
    yaw_deg: float
    yaw_rate_deg_s: float
    body_lateral_speed_mps: float
    road_wheel_deg: float
    steering_wheel_deg: float
    v_lat_mps: float
    steering_command_deg: float
    reference_steering_wheel_deg: float
    steering_area_deg_s: float
    v_lat_from_steering_mps: float
    steering_source: str

    def as_row(self) -> dict[str, float | str]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class SimulationRun:
    path: ReferencePath
    samples: tuple[SimulationSample, ...]
    config: SimulationConfig = SIMULATION
    vehicle: VehicleParams = VEHICLE
    mode: str = "LKA"
    scenario_key: str = "lka"
    scenario_name: str = "Lane Keep Assist"
    road_layout: str = "lka"
    tested_boundary: str = "Lane marking"
    intent: str = "unintentional"
    target_motion: str = "none"
    target_samples: tuple[TargetVehicleSample, ...] = ()
    collision_detected: bool = False
    collision_time_s: float | None = None

    def __post_init__(self) -> None:
        if not self.samples:
            raise ValueError("A simulation run must contain at least one sample")

    @property
    def duration_s(self) -> float:
        return self.samples[-1].time_s

    @property
    def pursuit_controller(self) -> PurePursuitController:
        return PurePursuitController(self.vehicle, self.config)

    @property
    def reference_arc_steering_deg(self) -> float:
        return math.degrees(self.pursuit_controller.reference_steering_wheel_rad(
            self.path.side / self.path.radius_m))

    @property
    def reference_arc_duration_s(self) -> float:
        return (self.path.radius_m * math.radians(self.path.psi_deg)
                / self.config.speed_mps)

    @property
    def reference_arc_start_s(self) -> float:
        """Scheduled start of the NCAP path curvature, independent of vehicle lag."""
        return self.config.approach_s

    @property
    def pursuit_command_start_s(self) -> float:
        """First sampled nonzero closed-loop steering command."""
        return next((sample.time_s for sample in self.samples
                     if sample.steering_source == "pursuit"
                     and abs(sample.steering_command_deg) > 1e-6),
                    self.duration_s)

    def rows(self) -> list[dict[str, float | str]]:
        rows = [sample.as_row() for sample in self.samples]
        if self.target_samples:
            for row, target in zip(rows, self.target_samples):
                row.update({
                    "target_x_m": target.x_m,
                    "target_y_m": target.y_m,
                    "target_yaw_deg": target.yaw_deg,
                    "target_speed_mps": target.speed_mps,
                })
        return rows


def road_lateral_speed_mps(yaw_deg: float, body_lateral_speed_mps: float,
                           speed_mps: float = SIMULATION.speed_mps, *,
                           yaw_rate_rad_s: float = 0.0,
                           front_offset_m: float = 0.0) -> float:
    """Road-frame lateral velocity, optionally at a forward offset."""
    yaw = math.radians(yaw_deg)
    return (speed_mps * math.sin(yaw)
            + body_lateral_speed_mps * math.cos(yaw)
            + front_offset_m * yaw_rate_rad_s * math.cos(yaw))


class SimulationEngine:
    def __init__(self, config: SimulationConfig = SIMULATION,
                 vehicle: VehicleParams = VEHICLE) -> None:
        self.config = config
        self.vehicle = vehicle

    def run_lka(self, lateral_speed_mps: float = 0.4, side: int = 1, *,
                lane_support: LaneSupportCommand | None = None,
                radius_m: float | None = None) -> SimulationRun:
        path = NcapPath(lateral_speed_mps, side, config=self.config,
                        radius_m=radius_m)
        run = self.run(path, lane_support=lane_support)
        if path.radius_m == 800.0:
            return replace(run, scenario_key="lka_r800",
                           scenario_name="Tight Curve (R=800 m)")
        return run

    def run_elk(self, scenario: ElkScenario | str,
                lateral_speed_mps: float = 0.4, side: int | None = None, *,
                intent: LaneChangeIntent | str = LaneChangeIntent.UNINTENTIONAL,
                target_motion: TargetMotion | str | None = None,
                lane_support: LaneSupportCommand | None = None) -> SimulationRun:
        intent = LaneChangeIntent(intent)
        definition = elk_definition(scenario, target_motion=target_motion, intent=intent)
        selected_side = definition.allowed_sides[0] if side is None else side
        path = ElkPath(lateral_speed_mps, selected_side, definition.scenario,
                       intent=intent, config=self.config)
        encounter = self._target_encounter(path, definition.target_motion)
        end_time = None if encounter is None else encounter[0] + 1.0
        run = self.run(path, lane_support=lane_support, end_time_s=end_time)
        target_samples = self._target_samples(run, definition.target_motion, encounter)
        collision_detected = False
        collision_time_s = None
        if target_samples:
            # Local import avoids coupling the core sample definitions to the
            # optional traffic-interaction geometry module.
            from .collision import first_collision
            collision = first_collision(run.samples, target_samples, self.vehicle)
            if collision is not None:
                ego_prefix = tuple(sample for sample in run.samples
                                   if sample.time_s < collision.time_s - 1e-12)
                target_prefix = tuple(sample for sample in target_samples
                                      if sample.time_s < collision.time_s - 1e-12)
                run = replace(run, samples=ego_prefix + (collision.ego,))
                target_samples = target_prefix + (collision.target,)
                collision_detected = True
                collision_time_s = collision.time_s
        return replace(
            run, mode="ELK", scenario_key=definition.scenario.value,
            scenario_name=definition.label, road_layout=definition.road_layout,
            tested_boundary=definition.tested_boundary, intent=intent.value,
            target_motion=definition.target_motion.value,
            target_samples=target_samples,
            collision_detected=collision_detected,
            collision_time_s=collision_time_s,
        )

    def run(self, path: ReferencePath, *,
            lane_support: LaneSupportCommand | None = None,
            end_time_s: float | None = None) -> SimulationRun:
        p, config = self.vehicle, self.config
        speed, dt, h = config.speed_mps, config.sample_period_s, config.substep_s
        plant = KinematicBicycleModel(p, speed, config)
        pursuit = PurePursuitController(p, config)
        steering_area_deg_s = 0.0
        applied_sw_rad = 0.0
        pursuit_active = True
        samples: list[SimulationSample] = []
        nominal_end = path.s2 / speed + config.departure_tail_s
        duration_s = nominal_end if end_time_s is None else max(nominal_end, end_time_s)
        steps = math.ceil(duration_s / dt)
        for i in range(steps + 1):
            t = i * dt
            fx, fy, yaw = plant.front_pose
            ref_x, ref_y, _, reference_curvature, phase = path.at(speed * t)
            _, tracking_y, tracking_heading, _, _ = path.at_x(fx)
            state = VehicleState(t, fx, fy, yaw, 0.0, plant.yaw_rate_rad_s,
                                 applied_sw_rad)
            support_angle = lane_support(state) if lane_support is not None else None
            if support_angle is not None:
                if not math.isfinite(support_angle):
                    raise ValueError("Lane-support steering command must be finite")
                pursuit_active = False
            elif not pursuit_active:
                raise ValueError("Lane support must supply an angle after taking control")

            if pursuit_active:
                pursuit_command = pursuit.command(
                    path, front_x_m=fx, front_y_m=fy, yaw_rad=yaw)
                command_sw = pursuit_command.road_wheel_rad * p.steer_ratio
                commanded_curvature = pursuit_command.curvature_per_m
                lookahead_x = pursuit_command.lookahead_x_m
                lookahead_y = pursuit_command.lookahead_y_m
                lookahead = pursuit_command.lookahead_distance_m
            else:
                if support_angle is None:  # Defensive guard for optimized Python builds.
                    raise ValueError("Lane support must supply an angle after taking control")
                limit = p.max_road_wheel_rad * p.steer_ratio
                command_sw = max(-limit, min(limit, support_angle))
                commanded_curvature = math.tan(command_sw / p.steer_ratio) / p.wheelbase_m
                lookahead_x, lookahead_y, lookahead = fx, fy, 0.0

            yaw_deg = math.degrees(yaw)
            # Values at t describe the current state; command_sw is held over
            # [t, t+dt). The applied angle at t is the prior interval's command.
            actual_sw_deg = math.degrees(applied_sw_rad)
            samples.append(SimulationSample(
                time_s=t, phase=phase, front_x_m=fx, front_y_m=fy,
                reference_x_m=ref_x, reference_y_m=ref_y,
                reference_curvature_per_m=reference_curvature,
                profile_curvature_per_m=commanded_curvature,
                lookahead_x_m=lookahead_x, lookahead_y_m=lookahead_y,
                lookahead_distance_m=lookahead,
                lateral_error_m=(tracking_y - fy) * math.cos(tracking_heading),
                yaw_deg=yaw_deg, yaw_rate_deg_s=math.degrees(plant.yaw_rate_rad_s),
                body_lateral_speed_mps=0.0,
                road_wheel_deg=actual_sw_deg / p.steer_ratio,
                steering_wheel_deg=actual_sw_deg,
                v_lat_mps=road_lateral_speed_mps(
                    yaw_deg, 0.0, speed, yaw_rate_rad_s=plant.yaw_rate_rad_s,
                    front_offset_m=p.wheelbase_m),
                steering_command_deg=math.degrees(command_sw),
                reference_steering_wheel_deg=math.degrees(
                    pursuit.reference_steering_wheel_rad(reference_curvature)),
                steering_area_deg_s=steering_area_deg_s,
                v_lat_from_steering_mps=(steering_area_deg_s
                                       * pursuit.lateral_gain_mps_per_deg_s),
                steering_source="pursuit" if pursuit_active else "lane_support",
            ))
            if i == steps:
                break
            for _ in range(round(dt / h)):
                steering_area_deg_s += math.degrees(command_sw) * h
                plant.step_substep(command_sw / p.steer_ratio)
            applied_sw_rad = command_sw
        return SimulationRun(path, tuple(samples), config, p)

    def _distance_at_lateral_position(self, path: ReferencePath,
                                      lateral_y_m: float) -> float:
        """Nominal path distance at a requested signed lateral position."""
        lateral = path.side * lateral_y_m
        if lateral < 0:
            return 0.0
        if lateral <= path.arc_y:
            ratio = max(-1.0, min(1.0, 1.0 - lateral / path.radius_m))
            return path.s0 + path.radius_m * math.acos(ratio)
        return path.s1 + (lateral - path.arc_y) / math.sin(math.radians(path.psi_deg))

    def _target_encounter(self, path: ReferencePath,
                          motion: TargetMotion) -> tuple[float, float, float] | None:
        """Return nominal encounter time, target initial x, and target lane y."""
        if motion == TargetMotion.NONE:
            return None
        p, speed = self.vehicle, self.config.speed_mps
        boundary_y = path.side * (path.arc_y + path.d2 + p.body_width_m / 2)
        target_y = boundary_y + path.side * 1.5
        if motion == TargetMotion.ONCOMING_72:
            # Protocol: front edges meet at 10% overlap. The simulator uses the
            # VUT footprint for the GVT because ISO target dimensions are not
            # specified numerically in LSS v4.2.
            encounter_y = target_y - path.side * (0.9 * p.body_width_m)
            target_speed = -speed
        else:
            encounter_y = target_y
            target_speed = speed if motion == TargetMotion.OVERTAKING_72 else 80.0 / 3.6
        encounter_s = self._distance_at_lateral_position(path, encounter_y)
        encounter_t = encounter_s / speed
        front_x, _, heading, _, _ = path.at(encounter_s)
        if motion == TargetMotion.ONCOMING_72:
            meeting_x = front_x + (p.body_length_m / 2 - p.l_front_m) * math.cos(heading)
            target_x_at_encounter = meeting_x + p.body_length_m / 2
        else:
            rear_axle_x = front_x - p.wheelbase_m * math.cos(heading)
            target_x_at_encounter = rear_axle_x - p.body_length_m / 2
        initial_x = target_x_at_encounter - target_speed * encounter_t
        return encounter_t, initial_x, target_y

    def _target_samples(self, run: SimulationRun, motion: TargetMotion,
                        encounter: tuple[float, float, float] | None,
                        ) -> tuple[TargetVehicleSample, ...]:
        if encounter is None:
            return ()
        _, initial_x, target_y = encounter
        speed = (-self.config.speed_mps if motion == TargetMotion.ONCOMING_72
                 else self.config.speed_mps if motion == TargetMotion.OVERTAKING_72
                 else 80.0 / 3.6)
        yaw_deg = 180.0 if speed < 0 else 0.0
        return tuple(TargetVehicleSample(sample.time_s,
                                        initial_x + speed * sample.time_s,
                                        target_y, yaw_deg, abs(speed))
                     for sample in run.samples)
