"""Command-line entry point for the viewer and headless CSV export."""
from __future__ import annotations

import argparse
import logging
from dataclasses import replace
from pathlib import Path
from typing import TYPE_CHECKING

from .config import NCAP_PATH_TABLES, SIMULATION, SimulationConfig
from .export import export_run, export_swa_profile
from .scenarios.elk import ElkParameters, ElkScenario, LaneChangeIntent, TargetMotion
from .simulation import SimulationEngine, SimulationRun
from .targets import CUSTOM_TARGET_TYPE, GVT_TYPES_BY_KEY, target_type

if TYPE_CHECKING:
    from .ui.app import SimulationApp

LOGGER = logging.getLogger(__name__)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Euro NCAP LKA/ELK simulation and viewer")
    parser.add_argument("--vlat", type=float, default=0.4,
                        help="NCAP target lateral speed in m/s (default 0.4)")
    parser.add_argument("--ncap-profile", choices=tuple(NCAP_PATH_TABLES),
                        default=None,
                        help="legacy headless LKA table: standard or alternative_dim")
    parser.add_argument("--d2", type=float, default=None,
                        help="tangent lateral distance in metres (default 1)")
    parser.add_argument("--radius", type=float, default=None,
                        help="reference arc radius in metres (default 1200)")
    parser.add_argument("--side", choices=("left", "right"), default=None,
                        help="departure side (scenario default when omitted)")
    parser.add_argument("--mode", choices=("lka", "elk"), default="lka")
    parser.add_argument("--elk-scenario", choices=tuple(value.value for value in ElkScenario),
                        default=ElkScenario.ROAD_EDGE.value)
    parser.add_argument("--intent", choices=tuple(value.value for value in LaneChangeIntent),
                        default=LaneChangeIntent.UNINTENTIONAL.value)
    parser.add_argument("--vut-speed", type=float, default=None, help="VUT speed in km/h")
    parser.add_argument("--target-speed", type=float, default=72,
                        help="ELK GVT speed in km/h")
    parser.add_argument("--gvt-marker-distance", type=float, default=1.5,
                        help="GVT centre distance from the marker inner edge in metres")
    parser.add_argument("--overlap", type=float, default=None,
                        help="oncoming nominal overlap as percent of VUT width (default 10)")
    parser.add_argument("--gvt-type", choices=tuple(GVT_TYPES_BY_KEY), default=None,
                        help="named target; motorcycle length defaults to wheelbase + diameter/2")
    parser.add_argument("--gvt-length", type=float, default=None)
    parser.add_argument("--gvt-width", type=float, default=None)
    parser.add_argument("--gvt-wheelbase", type=float, default=None,
                        help="GVT wheelbase in metres; catalogue values are fixed")
    parser.add_argument("--elk-protocol-tables", action="store_true",
                        help="headless legacy ELK table/synchronization compatibility")
    parser.add_argument("--headless", action="store_true", help="Generate CSV without a GUI")
    parser.add_argument("--output", type=Path, default=Path("lka_ncap_profile.csv"))
    parser.add_argument("--swa-only", action="store_true",
                        help="export time and steering-wheel command for external replay")
    parser.add_argument("--log-level", choices=("DEBUG", "INFO", "WARNING", "ERROR"),
                        default="WARNING")
    return parser


def _elk_parameters(args: argparse.Namespace,
                    config: SimulationConfig) -> ElkParameters | None:
    """Resolve CLI target dimensions without involving the viewer."""
    if args.elk_protocol_tables:
        return None
    defaults = ElkParameters()
    target_length = (defaults.gvt_length_m if args.gvt_length is None
                     else args.gvt_length)
    target_width = (defaults.gvt_width_m if args.gvt_width is None
                    else args.gvt_width)
    if args.gvt_type is not None:
        target = target_type(args.gvt_type)
        target_length, target_width = target.dimensions(
            target_length if target.is_custom else args.gvt_length,
            width_m=target_width if target.is_custom else args.gvt_width)
    return ElkParameters(
        radius_m=config.path_radius_m if args.radius is None else args.radius,
        d2_m=config.lka_d2_m if args.d2 is None else args.d2,
        gvt_speed_kph=args.target_speed,
        gvt_marker_distance_m=args.gvt_marker_distance,
        overlap_percent=defaults.overlap_percent if args.overlap is None else args.overlap,
        gvt_length_m=target_length, gvt_width_m=target_width,
        gvt_type_key=args.gvt_type, gvt_wheelbase_m=args.gvt_wheelbase)


def _validate_arguments(parser: argparse.ArgumentParser,
                        args: argparse.Namespace) -> None:
    if args.mode != "lka" and args.ncap_profile is not None:
        parser.error("--ncap-profile applies only to LKA")
    if args.mode == "lka" and (
        args.target_speed != 72 or args.gvt_marker_distance != 1.5 or args.overlap is not None
        or args.gvt_length is not None or args.gvt_width is not None
        or args.gvt_type is not None or args.gvt_wheelbase is not None
    ):
        parser.error("GVT speed, geometry, and overlap apply only to ELK")
    if args.overlap is not None and args.elk_scenario != "oncoming":
        parser.error("--overlap applies only to the oncoming scenario")
    if args.elk_protocol_tables:
        if not args.headless or args.mode != "elk":
            parser.error("--elk-protocol-tables requires --headless --mode elk")
        if (args.d2 is not None or args.radius is not None or args.vut_speed is not None
                or args.overlap not in (None, 10) or args.gvt_marker_distance != 1.5
                or args.target_speed not in (72, 80)
                or args.gvt_length is not None or args.gvt_width is not None
                or args.gvt_type is not None or args.gvt_wheelbase is not None):
            parser.error("custom geometry/speeds cannot override legacy ELK protocol tables")
        if args.elk_scenario != "overtaking" and args.target_speed != 72:
            parser.error("legacy GVT speed selection applies only to overtaking")
    if args.ncap_profile is not None:
        if not args.headless:
            parser.error("--ncap-profile is headless-only; the viewer uses editable LKA geometry")
        if args.d2 is not None or args.radius is not None:
            parser.error("omit --ncap-profile when setting --d2 or --radius")


def _print_run_summary(run: SimulationRun, output: Path) -> None:
    max_error = max(abs(sample.lateral_error_m) for sample in run.samples)
    path = run.path
    print(f"Wrote {len(run.samples)} rows to {output}; "
          f"dt={SIMULATION.sample_period_s:.3f} s; "
          f"max front-axle path error={max_error:.4f} m; "
          f"NCAP d1+d2={path.d1_table + path.d2:.3f} m")
    print(f"Pure Pursuit: look-ahead={run.pursuit_controller.lookahead_m:g} m; "
          f"reference arc={run.reference_arc_steering_deg:+.3f} deg "
          f"for {run.reference_arc_duration_s:.3f} s")
    if run.target_samples:
        target = run.target_samples[0]
        print(f"Initial VUT front axle=(0.000, 0.000) m; "
              f"GVT=({target.x_m:.3f}, {target.y_m:.3f}) m; "
              f"GVT speed={target.speed_mps * 3.6:.1f} km/h")
    if run.nominal_sync_time_s is not None:
        reference_x = (run.nominal_encounter.vut_reference_x_m
                       if run.nominal_encounter is not None
                       else run.nominal_vut_rear_axle_x_m)
        print(
            f"No-intervention encounter: t={run.nominal_sync_time_s:.6f} s; "
            f"GVT front={run.nominal_gvt_front_edge_x_m:.6f} m; "
            f"VUT reference={reference_x:.6f} m; "
            f"dx={run.nominal_sync_error_m:+.9f} m; "
            f"first polygon contact={run.predicted_collision_time_s} s")


def _configure_elk_viewer(app: SimulationApp, args: argparse.Namespace) -> None:
    """Apply ELK CLI overrides to the viewer's editable controls."""
    from .scenarios.elk import ELK_SCENARIOS

    app.gvt_speed_kph_var.set(f"{args.target_speed:g}")
    app.gvt_marker_distance_var.set(f"{args.gvt_marker_distance:g}")
    if args.overlap is not None:
        app.overlap_var.set(f"{args.overlap:g}")
    if args.gvt_type is not None:
        app.gvt_type_var.set(target_type(args.gvt_type).name)
    elif (args.gvt_length is not None or args.gvt_width is not None
          or args.gvt_wheelbase is not None):
        app.gvt_type_var.set(CUSTOM_TARGET_TYPE.name)
    if args.gvt_length is not None:
        app.gvt_length_var.set(f"{args.gvt_length:g}")
    if args.gvt_width is not None:
        app.gvt_width_var.set(f"{args.gvt_width:g}")
    if args.gvt_wheelbase is not None:
        app.gvt_wheelbase_var.set(f"{args.gvt_wheelbase:g}")
    elif app._selected_target_type().is_custom:
        app.gvt_wheelbase_var.set("")
    app.mode_var.set("ELK")
    app.elk_scenario_var.set(ELK_SCENARIOS[ElkScenario(args.elk_scenario)].label)
    app.intent_var.set("Intentional" if args.intent == "intentional" else "Unintentional")
    app._scenario_selection_changed()


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    _validate_arguments(parser, args)
    logging.basicConfig(level=getattr(logging, args.log_level),
                        format="%(levelname)s %(name)s: %(message)s")
    side = None if args.side is None else (1 if args.side == "left" else -1)
    try:
        config = (SIMULATION if args.vut_speed is None else
                  replace(SIMULATION, speed_mps=args.vut_speed / 3.6))
        parameters = _elk_parameters(args, config)
        if args.headless:
            engine = SimulationEngine(config)
            if args.mode == "lka":
                run = engine.run_lka(args.vlat, 1 if side is None else side,
                                     profile=args.ncap_profile,
                                     d2_m=args.d2, radius_m=args.radius)
            else:
                target_motion = None
                if args.elk_protocol_tables and args.elk_scenario == ElkScenario.OVERTAKING.value:
                    target_motion = (TargetMotion.OVERTAKING_80 if args.target_speed == 80
                                     else TargetMotion.OVERTAKING_72)
                run = engine.run_elk(args.elk_scenario, args.vlat, side,
                                     intent=args.intent, target_motion=target_motion,
                                     parameters=parameters)
            if args.swa_only:
                export_swa_profile(args.output, run)
            else:
                export_run(args.output, run)
            _print_run_summary(run, args.output)
        else:
            from .ui.app import SimulationApp
            app = SimulationApp(args.vlat, 1 if side is None else side,
                                engine=SimulationEngine(config),
                                d2_m=args.d2, radius_m=args.radius)
            if args.mode == "elk":
                _configure_elk_viewer(app, args)
            app.run()
    except Exception as error:
        LOGGER.exception("Simulation failed")
        parser.exit(2, f"Error: {error}\n")
    return 0
