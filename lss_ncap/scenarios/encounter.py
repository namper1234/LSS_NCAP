"""Controller-independent ELK encounter geometry from the scheduled path."""
from __future__ import annotations

import math
from dataclasses import dataclass

from ..config import RoadConfig, VehicleParams
from .base import ReferencePath
from .elk import ElkParameters, ElkScenario


@dataclass(frozen=True, slots=True)
class NominalEncounter:
    time_s: float
    initial_x_m: float
    path_y_m: float
    vut_front_x_m: float
    vut_front_y_m: float
    vut_yaw_rad: float
    target_x_m: float
    gvt_reference_edge_x_m: float
    vut_reference_x_m: float
    overlap_m: float

    @property
    def longitudinal_error_m(self) -> float:
        return self.gvt_reference_edge_x_m - self.vut_reference_x_m


def synchronize_encounter(path: ReferencePath, scenario: ElkScenario,
                          speed_mps: float, vehicle: VehicleParams,
                          road: RoadConfig, parameters: ElkParameters
                          ) -> NominalEncounter:
    """Use §7.2.4 front-edge/oncoming and front-edge/rear-axle/overtaking rules."""
    from ..collision import clip_longitudinal, vehicle_rectangle

    overtaking = scenario == ElkScenario.OVERTAKING
    overlap = 0.0 if overtaking else vehicle.body_width_m * parameters.overlap_percent / 100
    if overlap > parameters.gvt_width_m:
        raise ValueError("Requested overlap exceeds the GVT width")
    marker = path.line_offset_m + vehicle.body_width_m / 2
    target_y = marker - road.marking_width_m / 2 + parameters.gvt_marker_distance_m
    required_edge = target_y - parameters.gvt_width_m / 2 + overlap

    def departure_edge(station: float) -> float:
        x, y, yaw, _, _ = path.at(station)
        if overtaking:
            centre_x = x - vehicle.l_front_m * math.cos(yaw)
            centre_y = y - vehicle.l_front_m * math.sin(yaw)
            rear_axle_x = x - vehicle.wheelbase_m * math.cos(yaw)
            footprint = vehicle_rectangle(
                centre_x, centre_y, math.degrees(yaw),
                vehicle.body_length_m, vehicle.body_width_m)
            contact_section = clip_longitudinal(
                footprint, rear_axle_x - parameters.gvt_length_m, rear_axle_x)
            if not contact_section:
                raise ValueError("GVT leading edge at VUT rear axle has no overlapping body span")
            return max(path.side * point[1] for point in contact_section)
        centre_y = path.side * (y - vehicle.l_front_m * math.sin(yaw))
        return (centre_y + vehicle.body_length_m / 2 * abs(math.sin(yaw))
                + vehicle.body_width_m / 2 * abs(math.cos(yaw)))

    if required_edge <= departure_edge(0):
        raise ValueError("GVT offset/overlap places the nominal encounter before departure")
    # The tangent extends beyond d2; d2 defines marker placement, not a path stop.
    arc_end_edge = departure_edge(path.s1)
    if required_edge > arc_end_edge:
        lateral_slope = abs(math.sin(math.radians(path.psi_deg)))
        station = path.s1 + (required_edge - arc_end_edge) / lateral_slope
    else:
        low, high = path.s0, path.s1
        for _ in range(60):
            middle = (low + high) / 2
            if departure_edge(middle) >= required_edge:
                high = middle
            else:
                low = middle
        station = (low + high) / 2
    x, y, yaw, _, _ = path.at(station)
    centre_x = x - vehicle.l_front_m * math.cos(yaw)
    projected_half_length = (vehicle.body_length_m / 2 * abs(math.cos(yaw))
                             + vehicle.body_width_m / 2 * abs(math.sin(yaw)))
    if scenario == ElkScenario.ONCOMING:
        reference_x = centre_x + projected_half_length
        target_x = reference_x + parameters.gvt_length_m / 2
        signed_speed = -parameters.gvt_speed_kph / 3.6
        target_front = target_x - parameters.gvt_length_m / 2
    elif scenario == ElkScenario.OVERTAKING:
        reference_x = x - vehicle.wheelbase_m * math.cos(yaw)
        target_x = reference_x - parameters.gvt_length_m / 2
        signed_speed = parameters.gvt_speed_kph / 3.6
        if signed_speed < speed_mps * math.cos(yaw):
            raise ValueError("Overtaking requires GVT speed at least the VUT departure longitudinal speed")
        target_front = target_x + parameters.gvt_length_m / 2
        # The nominal event is first physical contact, not a requested lateral overlap.
        centre_y = y - vehicle.l_front_m * math.sin(yaw)
        overlap = max(0.0, path.side * centre_y
                      + vehicle.body_length_m / 2 * abs(math.sin(yaw))
                      + vehicle.body_width_m / 2 * abs(math.cos(yaw))
                      - (target_y - parameters.gvt_width_m / 2))
    else:
        raise ValueError("A nominal traffic encounter requires oncoming or overtaking")
    time_s = station / speed_mps
    initial_x = target_x - signed_speed * time_s
    if scenario == ElkScenario.OVERTAKING:
        if initial_x >= -vehicle.l_front_m:
            raise ValueError("Configured overtaking GVT does not start behind the VUT; increase GVT speed or approach time")
    return NominalEncounter(
        time_s, initial_x, path.side * target_y, x, y, yaw, target_x,
        target_front, reference_x, overlap)
