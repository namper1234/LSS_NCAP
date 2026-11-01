"""Command-line entry point for the viewer and headless CSV export."""
from __future__ import annotations

import argparse
import logging
from pathlib import Path

from .config import NCAP_PATH, SIMULATION
from .export import export_run, export_swa_profile
from .scenarios.elk import ElkScenario, LaneChangeIntent, TargetMotion
from .simulation import SimulationEngine

LOGGER = logging.getLogger(__name__)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Euro NCAP LKA/ELK simulation and viewer")
    parser.add_argument("--vlat", type=float, choices=sorted((*NCAP_PATH, 0.7)), default=0.4,
                        help="NCAP target lateral speed in m/s (default 0.4)")
    parser.add_argument("--side", choices=("left", "right"), default=None,
                        help="departure side (scenario default when omitted)")
    parser.add_argument("--mode", choices=("lka", "elk"), default="lka")
    parser.add_argument("--elk-scenario", choices=tuple(value.value for value in ElkScenario),
                        default=ElkScenario.ROAD_EDGE.value)
    parser.add_argument("--intent", choices=tuple(value.value for value in LaneChangeIntent),
                        default=LaneChangeIntent.UNINTENTIONAL.value)
    parser.add_argument("--target-speed", type=int, choices=(72, 80), default=72,
                        help="GVT speed for ELK overtaking")
    parser.add_argument("--headless", action="store_true", help="Generate CSV without a GUI")
    parser.add_argument("--output", type=Path, default=Path("lka_ncap_profile.csv"))
    parser.add_argument("--swa-only", action="store_true",
                        help="export time and steering-wheel command for external replay")
    parser.add_argument("--log-level", choices=("DEBUG", "INFO", "WARNING", "ERROR"),
                        default="WARNING")
    args = parser.parse_args(argv)
    logging.basicConfig(level=getattr(logging, args.log_level),
                        format="%(levelname)s %(name)s: %(message)s")
    side = None if args.side is None else (1 if args.side == "left" else -1)
    try:
        if args.headless:
            engine = SimulationEngine()
            if args.mode == "lka":
                run = engine.run_lka(args.vlat, 1 if side is None else side)
            else:
                target_motion = None
                if args.elk_scenario == ElkScenario.OVERTAKING.value:
                    target_motion = (TargetMotion.OVERTAKING_80 if args.target_speed == 80
                                     else TargetMotion.OVERTAKING_72)
                run = engine.run_elk(args.elk_scenario, args.vlat, side,
                                     intent=args.intent, target_motion=target_motion)
            if args.swa_only:
                export_swa_profile(args.output, run)
            else:
                export_run(args.output, run)
            max_error = max(abs(sample.lateral_error_m) for sample in run.samples)
            path = run.path
            print(f"Wrote {len(run.samples)} rows to {args.output}; "
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
        else:
            from .ui.app import SimulationApp
            SimulationApp(args.vlat, 1 if side is None else side).run()
    except Exception as error:
        LOGGER.exception("Simulation failed")
        parser.exit(2, f"Error: {error}\n")
    return 0
