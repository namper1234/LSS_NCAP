"""Closed-loop Pure Pursuit with kinematic bicycle state feedback."""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, replace
from typing import Callable

from .config import (NCAP_ALTERNATIVE_DIM, SIMULATION, VEHICLE,
                     ROAD, RoadConfig, SimulationConfig, VehicleParams)
from .dynamics import KinematicBicycleModel
from .scenarios.base import ReferencePath
from .scenarios.lka import NcapPath
from .scenarios.elk import (ElkParameters, ElkPath, ElkScenario, LaneChangeIntent,
                            TargetMotion, elk_definition)
from .scenarios.encounter import NominalEncounter, synchronize_encounter
from .steering import PurePursuitController
from .targets import target_type


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
class TargetEncounter:
    time_s: float
    initial_x_m: float
    path_y_m: float
    impact_x_m: float
    gvt_reference_edge_x_m: float
    vut_reference_x_m: float
    longitudinal_error_m: float

    def __iter__(self):  # Compatibility with the former private tuple result.
        return iter((self.time_s, self.initial_x_m, self.path_y_m))


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
    reference_v_lat_mps: float
    steering_command_deg: float
    reference_steering_wheel_deg: float
    steering_area_deg_s: float
    v_lat_from_steering_mps: float
    steering_source: str
    robot_steering_command_deg: float = 0.0
    elk_steering_command_deg: float = 0.0
    eps_steering_wheel_deg: float = 0.0
    ttc_s: float | None = None
    lateral_separation_m: float | None = None

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
    predicted_collision_time_s: float | None = None
    robot_release_time_s: float | None = None
    elk_intervention_time_s: float | None = None
    turn_signal_time_s: float | None = None
    protocol_termination_time_s: float | None = None
    protocol_termination_reason: str | None = None
    nominal_sync_time_s: float | None = None
    nominal_gvt_front_edge_x_m: float | None = None
    nominal_vut_rear_axle_x_m: float | None = None
    nominal_sync_error_m: float | None = None
    nominal_sync_to_collision_s: float | None = None
    overtaking_sync_compliant: bool | None = None
    termination_reason: str = "path_complete"
    protocol_assumptions: tuple[str, ...] = ()
    elk_parameters: ElkParameters | None = None
    nominal_encounter: NominalEncounter | None = None
    target_vehicle: VehicleParams | None = None
    no_intervention_samples: tuple[SimulationSample, ...] = ()
    collision_point_m: tuple[float, float] | None = None
    road: RoadConfig = ROAD

    def __post_init__(self) -> None:
        if not self.samples:
            raise ValueError("A simulation run must contain at least one sample")

    @property
    def duration_s(self) -> float:
        return self.samples[-1].time_s

    @property
    def collision_before_nominal_s(self) -> float | None:
        """Positive gap when first contact prevents reaching the nominal encounter."""
        if self.collision_time_s is None or self.nominal_encounter is None:
            return None
        gap = self.nominal_encounter.time_s - self.collision_time_s
        return gap if gap > 1e-9 else None

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

    @property
    def initial_lane_marker_distance_m(self) -> float:
        """Initial VUT centreline to marking centre: d1 + d2 + half VUT width."""
        return self.path.line_offset_m + self.vehicle.body_width_m / 2

    @property
    def lane_marker_inner_y_m(self) -> float:
        """VUT-side marking edge, or the unpainted road boundary for road-edge runs."""
        half_marking = 0.0 if self.road_layout == "road_edge" else self.road.marking_width_m / 2
        return self.path.side * (
            self.initial_lane_marker_distance_m - half_marking)

    def lane_marker_clearance_m(self, index: int) -> float:
        from .collision import lane_marker_clearance_m

        return lane_marker_clearance_m(
            self.samples[index], self.vehicle, self.lane_marker_inner_y_m, self.path.side)

    def rows(self) -> list[dict[str, float | str]]:
        rows = [sample.as_row() for sample in self.samples]
        for index, row in enumerate(rows):
            row["vut_lane_marker_clearance_m"] = self.lane_marker_clearance_m(index)
        profile = (target_type(self.elk_parameters.gvt_type_key)
                   if self.elk_parameters is not None
                   and self.elk_parameters.gvt_type_key is not None else None)
        if self.target_samples:
            for row, target in zip(rows, self.target_samples):
                row.update({
                    "target_x_m": target.x_m,
                    "target_y_m": target.y_m,
                    "target_yaw_deg": target.yaw_deg,
                    "target_speed_mps": target.speed_mps,
                })
                if self.elk_parameters is not None:
                    row.update({
                        "target_overlap_percent": (self.elk_parameters.overlap_percent
                                                   if self.scenario_key == "oncoming" else ""),
                        "gvt_marker_distance_m": self.elk_parameters.gvt_marker_distance_m,
                        "gvt_width_m": self.elk_parameters.gvt_width_m,
                        "gvt_length_m": self.elk_parameters.gvt_length_m,
                        "gvt_type": self.elk_parameters.gvt_type_key or "custom",
                        "gvt_wheelbase_m": self.elk_parameters.resolved_gvt_wheelbase_m
                        if self.elk_parameters.resolved_gvt_wheelbase_m is not None else "",
                        "gvt_length_source": (
                            "custom" if profile is None else
                            profile.length_source(self.elk_parameters.gvt_length_m)),
                        "gvt_length_wheel_diameter_m": (
                            profile.length_wheel_diameter_m
                            if profile is not None and profile.length_wheel_diameter_m is not None
                            else ""),
                        "gvt_dimension_source": profile.source_url if profile is not None else "",
                        "intervention_active": row["steering_source"] == "lane_support",
                        "trajectory_kind": ("elk_controlled" if self.elk_intervention_time_s is not None
                                            else "no_intervention"),
                        "no_intervention_collision_time_s": self.predicted_collision_time_s,
                        "actual_collision_time_s": self.collision_time_s,
                        "first_contact_x_m": (self.collision_point_m[0]
                                             if self.collision_point_m is not None else ""),
                        "first_contact_y_m": (self.collision_point_m[1]
                                             if self.collision_point_m is not None else ""),
                    })
                    if self.nominal_encounter is not None:
                        row.update({
                            "nominal_overlap_m": self.nominal_encounter.overlap_m,
                            "nominal_vut_front_x_m": self.nominal_encounter.vut_front_x_m,
                            "nominal_vut_front_y_m": self.nominal_encounter.vut_front_y_m,
                            "nominal_vut_yaw_deg": math.degrees(self.nominal_encounter.vut_yaw_rad),
                            "nominal_gvt_x_m": self.nominal_encounter.target_x_m,
                            "nominal_vut_reference_x_m": self.nominal_encounter.vut_reference_x_m,
                            "nominal_reference": "scheduled_path_no_intervention",
                        })
                if self.nominal_sync_time_s is not None:
                    row.update({
                        "nominal_sync_time_s": self.nominal_sync_time_s,
                        "nominal_gvt_front_edge_x_m":
                            self.nominal_gvt_front_edge_x_m,
                        "nominal_vut_rear_axle_x_m":
                            self.nominal_vut_rear_axle_x_m,
                        "nominal_sync_error_m": self.nominal_sync_error_m,
                        "first_polygon_collision_time_s":
                            self.predicted_collision_time_s,
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
                 vehicle: VehicleParams = VEHICLE,
                 road: RoadConfig = ROAD) -> None:
        self.config = config
        self.vehicle = vehicle
        self.road = road

    def run_lka(self, lateral_speed_mps: float = 0.4, side: int = 1, *,
                lane_support: LaneSupportCommand | None = None,
                profile: str | None = None,
                radius_m: float | None = None,
                d2_m: float | None = None) -> SimulationRun:
        """Track configurable LKA geometry; explicit profiles select legacy tables."""
        path = NcapPath(lateral_speed_mps, side, config=self.config,
                        profile=profile, radius_m=radius_m, d2_m=d2_m)
        run = self.run(path, lane_support=lane_support)
        if profile is None:
            return replace(run, scenario_key="lka_custom",
                           scenario_name="Configurable LKA")
        if profile == NCAP_ALTERNATIVE_DIM:
            return replace(run, scenario_key="lka_alternative_dim",
                           scenario_name="Alternative NCAP (DIM)")
        return run

    def run_elk(self, scenario: ElkScenario | str,
                lateral_speed_mps: float = 0.4, side: int | None = None, *,
                intent: LaneChangeIntent | str = LaneChangeIntent.UNINTENTIONAL,
                target_motion: TargetMotion | str | None = None,
                lane_support: LaneSupportCommand | None = None,
                safe_termination: bool | None = None,
                parameters: ElkParameters | None = ElkParameters()) -> SimulationRun:
        """Generate custom ELK; parameters=None explicitly reproduces legacy tables."""
        if parameters is not None and target_motion is not None:
            raise ValueError("Set parameters.gvt_speed_kph for custom ELK; target_motion is legacy-only")
        intent = LaneChangeIntent(intent)
        definition = elk_definition(scenario, target_motion=target_motion, intent=intent)
        selected_side = definition.allowed_sides[0] if side is None else side
        path = ElkPath(lateral_speed_mps, selected_side, definition.scenario,
                       intent=intent, config=self.config, parameters=parameters)
        if safe_termination is None:
            safe_termination = parameters is None
        target_vehicle = (self.vehicle if parameters is None else replace(
            self.vehicle, body_length_m=parameters.gvt_length_m,
            body_width_m=parameters.gvt_width_m))
        if parameters is not None and parameters.resolved_gvt_wheelbase_m is not None:
            # GVT motion is prescribed at its body centre; axle split is not measured.
            wheelbase = parameters.resolved_gvt_wheelbase_m
            target_vehicle = replace(target_vehicle, l_front_m=wheelbase / 2,
                                     l_rear_m=wheelbase / 2)
        nominal = (synchronize_encounter(
            path, definition.scenario, self.config.speed_mps,
            self.vehicle, self.road, parameters)
            if parameters is not None and definition.target_motion != TargetMotion.NONE
            else None)
        # Traffic encounters can occur well into the final constant-heading
        # straight at the lowest V_lat values.  Five seconds covers the
        # maximum polygon offset without altering any NCAP curve parameter.
        traffic_end = (path.s2 / self.config.speed_mps + 5.0
                       if definition.target_motion != TargetMotion.NONE else None)
        if nominal is not None:
            traffic_end = nominal.time_s + (
                self.vehicle.body_width_m + target_vehicle.body_width_m
            ) / lateral_speed_mps + self.config.departure_tail_s
        baseline = self.run(path, end_time_s=traffic_end)
        encounter = (nominal if nominal is not None else
                     self._target_encounter(path, definition.target_motion, baseline))
        end_time = (path.s2 / self.config.speed_mps + 2.0
                    if definition.scenario == ElkScenario.ROAD_EDGE
                    else None if encounter is None else encounter.time_s + 1.0)
        if end_time is not None and end_time > baseline.duration_s:
            baseline = self.run(path, end_time_s=end_time)
        predicted_collision_time_s = None
        if encounter is not None:
            from .collision import first_collision
            baseline_targets = self._target_samples(
                baseline, definition.target_motion, encounter, parameters=parameters)
            predicted = first_collision(
                baseline.samples, baseline_targets, self.vehicle, target_vehicle)
            if predicted is not None:
                predicted_collision_time_s = predicted.time_s
        is_overtaking = definition.target_motion in (
            TargetMotion.OVERTAKING_72, TargetMotion.OVERTAKING_80)
        sync_error_m = (encounter.longitudinal_error_m if encounter is not None else None)
        sync_compliant = (None if sync_error_m is None or not is_overtaking
                          else abs(sync_error_m) <= 1e-6)
        if sync_compliant is False:
            raise RuntimeError(
                "ELK Overtaking synchronization exceeds 1 µm tolerance")
        intervention_time = self._intervention_time(baseline, lane_support)
        release_time = (None if intervention_time is None else
                        max(0.0, intervention_time - 5.0 / self.config.speed_mps))
        run = (baseline if lane_support is None else
               self.run(path, lane_support=lane_support, end_time_s=end_time,
                        robot_release_time_s=release_time))
        target_samples = self._target_samples(
            run, definition.target_motion, encounter, parameters=parameters)
        if target_samples:
            run = replace(run, samples=self._traffic_telemetry(
                run.samples, target_samples, target_vehicle))
        collision_detected = False
        collision_time_s = None
        collision_point_m = None
        protocol_termination_time_s = None
        protocol_termination_reason = None
        termination_reason = "path_complete"
        if target_samples:
            # Local import avoids coupling the core sample definitions to the
            # optional traffic-interaction geometry module.
            from .collision import first_collision
            collision = first_collision(
                run.samples, target_samples, self.vehicle, target_vehicle)
            if collision is not None:
                ego_prefix = tuple(sample for sample in run.samples
                                   if sample.time_s < collision.time_s - 1e-12)
                target_prefix = tuple(sample for sample in target_samples
                                      if sample.time_s < collision.time_s - 1e-12)
                reference_heading = path.at(
                    self.config.speed_mps * collision.time_s)[2]
                impact_sample = replace(
                    collision.ego,
                    reference_v_lat_mps=self.config.speed_mps * math.sin(reference_heading))
                run = replace(run, samples=ego_prefix + (impact_sample,))
                target_samples = target_prefix + (collision.target,)
                collision_detected = True
                collision_time_s = collision.time_s
                collision_point_m = collision.contact_point_m
                termination_reason = "collision"
            safe_end = self._protocol_traffic_end(run.samples, intervention_time)
            if safe_end is not None:
                end_index, protocol_termination_reason = safe_end
                protocol_termination_time_s = run.samples[end_index].time_s
                if safe_termination:
                    termination_reason = protocol_termination_reason
                    run = replace(run, samples=run.samples[:end_index + 1])
                    target_samples = target_samples[:end_index + 1]
                    collision_detected = False
                    collision_time_s = None
                    collision_point_m = None
        return replace(
            run, mode="ELK", scenario_key=definition.scenario.value,
            scenario_name=definition.label, road_layout=definition.road_layout,
            tested_boundary=definition.tested_boundary, intent=intent.value,
            target_motion=definition.target_motion.value,
            target_samples=target_samples,
            collision_detected=collision_detected,
            collision_time_s=collision_time_s,
            collision_point_m=collision_point_m,
            predicted_collision_time_s=predicted_collision_time_s,
            robot_release_time_s=release_time,
            elk_intervention_time_s=intervention_time,
            turn_signal_time_s=(max(0.0, self.config.approach_s - 1.0)
                                if intent == LaneChangeIntent.INTENTIONAL else None),
            protocol_termination_time_s=protocol_termination_time_s,
            protocol_termination_reason=protocol_termination_reason,
            nominal_sync_time_s=(encounter.time_s
                                 if encounter is not None and (is_overtaking or nominal is not None)
                                 else None),
            nominal_gvt_front_edge_x_m=(
                encounter.gvt_reference_edge_x_m
                if encounter is not None else None),
            nominal_vut_rear_axle_x_m=(
                encounter.vut_reference_x_m
                if is_overtaking and encounter is not None else None),
            nominal_sync_error_m=sync_error_m,
            nominal_sync_to_collision_s=(
                predicted_collision_time_s - encounter.time_s
                if (is_overtaking and encounter is not None
                    and predicted_collision_time_s is not None) else None),
            overtaking_sync_compliant=sync_compliant,
            termination_reason=termination_reason,
            elk_parameters=parameters, nominal_encounter=nominal,
            target_vehicle=target_vehicle,
            no_intervention_samples=(baseline.samples if nominal is not None else ()),
            protocol_assumptions=(
                ("GVT uses independently configured length and width."
                 if parameters is not None else
                 "GVT footprint uses configured VUT dimensions because LSS v4.2 references ISO 19206-3 without numeric GVT dimensions."),
                "Custom geometry is exploratory, not a protocol certification; nominal synchronization uses the scheduled path without ELK reaction.",
                "When OEM intervention timing is unavailable, safe traffic termination uses the first §7.4.7 threshold reached.",
                "EPS response is the commanded road-wheel angle applied through the existing one-control-tick vehicle model.",
            ),
        )

    def run(self, path: ReferencePath, *,
            lane_support: LaneSupportCommand | None = None,
            end_time_s: float | None = None,
            robot_release_time_s: float | None = None) -> SimulationRun:
        p, config = self.vehicle, self.config
        speed, dt, h = config.speed_mps, config.sample_period_s, config.substep_s
        plant = KinematicBicycleModel(p, speed, config)
        pursuit = PurePursuitController(p, config)
        steering_area_deg_s = 0.0
        applied_sw_rad = 0.0
        support_active = False
        samples: list[SimulationSample] = []
        nominal_end = path.s2 / speed + config.departure_tail_s
        duration_s = nominal_end if end_time_s is None else max(nominal_end, end_time_s)
        steps = math.ceil(duration_s / dt)
        for i in range(steps + 1):
            t = i * dt
            fx, fy, yaw = plant.front_pose
            ref_x, ref_y, reference_heading, reference_curvature, phase = path.at(speed * t)
            _, tracking_y, tracking_heading, _, _ = path.at_x(fx)
            state = VehicleState(t, fx, fy, yaw, 0.0, plant.yaw_rate_rad_s,
                                 applied_sw_rad)
            support_angle = lane_support(state) if lane_support is not None else None
            if support_angle is not None:
                if not math.isfinite(support_angle):
                    raise ValueError("Lane-support steering command must be finite")
                support_active = True
            elif support_active:
                raise ValueError("Lane support must supply an angle after taking control")
            robot_available = (robot_release_time_s is None
                               or t < robot_release_time_s - 1e-12)
            robot_command_sw = 0.0
            elk_command_sw = 0.0
            if support_active:
                if support_angle is None:  # Defensive guard for optimized Python builds.
                    raise ValueError("Lane support must supply an angle after taking control")
                limit = p.max_road_wheel_rad * p.steer_ratio
                elk_command_sw = max(-limit, min(limit, support_angle))
                command_sw = elk_command_sw
                commanded_curvature = math.tan(command_sw / p.steer_ratio) / p.wheelbase_m
                lookahead_x, lookahead_y, lookahead = fx, fy, 0.0
                steering_source = "lane_support"
            elif robot_available:
                pursuit_command = pursuit.command(
                    path, front_x_m=fx, front_y_m=fy, yaw_rad=yaw)
                robot_command_sw = pursuit_command.road_wheel_rad * p.steer_ratio
                command_sw = robot_command_sw
                commanded_curvature = pursuit_command.curvature_per_m
                lookahead_x = pursuit_command.lookahead_x_m
                lookahead_y = pursuit_command.lookahead_y_m
                lookahead = pursuit_command.lookahead_distance_m
                steering_source = "pursuit"
            else:
                command_sw = 0.0
                commanded_curvature = 0.0
                lookahead_x, lookahead_y, lookahead = fx, fy, 0.0
                steering_source = "passive"

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
                reference_v_lat_mps=speed * math.sin(reference_heading),
                steering_command_deg=math.degrees(command_sw),
                reference_steering_wheel_deg=math.degrees(
                    pursuit.reference_steering_wheel_rad(reference_curvature)),
                steering_area_deg_s=steering_area_deg_s,
                v_lat_from_steering_mps=(steering_area_deg_s
                                       * pursuit.lateral_gain_mps_per_deg_s),
                steering_source=steering_source,
                robot_steering_command_deg=math.degrees(robot_command_sw),
                elk_steering_command_deg=math.degrees(elk_command_sw),
                eps_steering_wheel_deg=actual_sw_deg,
            ))
            if i == steps:
                break
            for _ in range(round(dt / h)):
                steering_area_deg_s += math.degrees(command_sw) * h
                plant.step_substep(command_sw / p.steer_ratio)
            applied_sw_rad = command_sw
        return SimulationRun(path, tuple(samples), config, p, road=self.road)

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

    def _target_encounter(self, path: ReferencePath, motion: TargetMotion,
                          baseline: SimulationRun | None = None
                          ) -> TargetEncounter | None:
        """Synchronize the GVT against the no-ELK simulated VUT polygon."""
        if motion == TargetMotion.NONE:
            return None
        from .collision import ego_polygon, interpolate_sample
        p = self.vehicle
        if baseline is None:
            baseline = self.run(path)
        marker_center_y = path.side * (path.line_offset_m + p.body_width_m / 2)
        marker_inner_edge_y = marker_center_y - path.side * self.road.marking_width_m / 2
        target_y = marker_inner_edge_y + path.side * 1.5
        target_near_edge_y = target_y - path.side * p.body_width_m / 2
        overlap_m = 0.1 * p.body_width_m if motion == TargetMotion.ONCOMING_72 else 0.0
        required_edge_y = target_near_edge_y + path.side * overlap_m

        def lateral_metric(sample: SimulationSample) -> float:
            ys = [point[1] for point in ego_polygon(sample, p)]
            departure_edge_y = max(ys) if path.side > 0 else min(ys)
            return path.side * (departure_edge_y - required_edge_y)

        encounter_sample: SimulationSample | None = None
        for left, right in zip(baseline.samples, baseline.samples[1:]):
            if lateral_metric(left) >= 0:
                encounter_sample = left
                break
            if lateral_metric(right) < 0:
                continue
            low, high = 0.0, 1.0
            for _ in range(50):
                middle = (low + high) / 2
                candidate = interpolate_sample(left, right, middle)
                if lateral_metric(candidate) >= 0:
                    high = middle
                else:
                    low = middle
            encounter_sample = interpolate_sample(left, right, high)
            break
        if encounter_sample is None:
            raise ValueError("Baseline VUT path does not reach the protocol GVT encounter")

        encounter_t = encounter_sample.time_s
        target_speed = self._signed_target_speed(motion)
        ego_points = ego_polygon(encounter_sample, p)
        if motion == TargetMotion.ONCOMING_72:
            impact_x = max(point[0] for point in ego_points)
            target_x_at_encounter = impact_x + p.body_length_m / 2
            gvt_reference_edge_x = target_x_at_encounter - p.body_length_m / 2
            vut_reference_x = impact_x
        else:
            yaw = math.radians(encounter_sample.yaw_deg)
            rear_axle_x = encounter_sample.front_x_m - p.wheelbase_m * math.cos(yaw)
            impact_x = rear_axle_x
            target_yaw = 0.0
            target_leading_offset_x = p.body_length_m / 2 * math.cos(target_yaw)
            target_x_at_encounter = impact_x - target_leading_offset_x
            gvt_reference_edge_x = target_x_at_encounter + target_leading_offset_x
            vut_reference_x = rear_axle_x
        initial_x = target_x_at_encounter - target_speed * encounter_t
        return TargetEncounter(
            encounter_t, initial_x, target_y, impact_x,
            gvt_reference_edge_x, vut_reference_x,
            gvt_reference_edge_x - vut_reference_x)

    def _target_samples(self, run: SimulationRun, motion: TargetMotion,
                        encounter: TargetEncounter | NominalEncounter | None, *,
                        parameters: ElkParameters | None = None,
                        ) -> tuple[TargetVehicleSample, ...]:
        if encounter is None:
            return ()
        speed = (self._signed_target_speed(motion) if parameters is None else
                 parameters.gvt_speed_kph / 3.6 * (
                     -1 if motion == TargetMotion.ONCOMING_72 else 1))
        yaw_deg = 180.0 if speed < 0 else 0.0
        return tuple(TargetVehicleSample(sample.time_s,
                                        encounter.initial_x_m + speed * sample.time_s,
                                        encounter.path_y_m, yaw_deg, abs(speed))
                     for sample in run.samples)

    def _signed_target_speed(self, motion: TargetMotion) -> float:
        if motion == TargetMotion.ONCOMING_72:
            return -self.config.speed_mps
        if motion == TargetMotion.OVERTAKING_72:
            return self.config.speed_mps
        if motion == TargetMotion.OVERTAKING_80:
            return 80.0 / 3.6
        raise ValueError("Target motion has no moving GVT")

    @staticmethod
    def _intervention_time(run: SimulationRun,
                           lane_support: LaneSupportCommand | None) -> float | None:
        if lane_support is None:
            return None
        for sample in run.samples:
            state = VehicleState(
                sample.time_s, sample.front_x_m, sample.front_y_m,
                math.radians(sample.yaw_deg), sample.body_lateral_speed_mps,
                math.radians(sample.yaw_rate_deg_s),
                math.radians(sample.steering_wheel_deg))
            if lane_support(state) is not None:
                return sample.time_s
        return None

    def _traffic_telemetry(
            self, samples: tuple[SimulationSample, ...],
            targets: tuple[TargetVehicleSample, ...],
            target_vehicle: VehicleParams | None = None) -> tuple[SimulationSample, ...]:
        from .collision import lateral_separation_m, time_to_collision_s
        return tuple(replace(
            sample,
            ttc_s=time_to_collision_s(sample, target, self.vehicle,
                                      self.config.speed_mps, target_vehicle),
            lateral_separation_m=lateral_separation_m(
                sample, target, self.vehicle, target_vehicle),
        ) for sample, target in zip(samples, targets))

    @staticmethod
    def _protocol_traffic_end(
            samples: tuple[SimulationSample, ...],
            intervention_time_s: float | None) -> tuple[int, str] | None:
        if intervention_time_s is None:
            for index, sample in enumerate(samples):
                if sample.ttc_s is not None and sample.ttc_s <= 0.8:
                    return index, "protocol_failure_ttc"
                if (sample.lateral_separation_m is not None
                        and sample.lateral_separation_m < 0.3):
                    return index, "protocol_failure_separation"
            return None
        seen_collision_course = False
        for index, sample in enumerate(samples):
            if sample.time_s < intervention_time_s:
                continue
            if sample.ttc_s is not None:
                seen_collision_course = True
            elif seen_collision_course:
                return index, "elk_collision_avoided"
        return None
