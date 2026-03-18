from __future__ import annotations

import argparse

from multi_vehicle_scheduler.scheduler_backend import SceneScheduler

from .app import SimulationViewerApp
from .scenario_loader import load_scenario


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Multi-vehicle scheduler pyglet viewer."
    )
    parser.add_argument(
        "--scenario", type=str, default=None, help="Path to scenario json."
    )
    parser.add_argument(
        "--dt", type=float, default=1 / 30, help="Fixed simulation step in seconds."
    )
    parser.add_argument(
        "--horizon-steps",
        type=int,
        default=10,
        help="Scheduler trajectory horizon steps.",
    )
    return parser


def main() -> None:
    args = _build_parser().parse_args()

    scenario, world_state = load_scenario(args.scenario)
    scheduler = SceneScheduler(horizon_steps=args.horizon_steps)

    app = SimulationViewerApp(
        scenario=scenario,
        initial_world=world_state,
        scheduler=scheduler,
        fixed_dt=args.dt,
    )
    app.run()


if __name__ == "__main__":
    main()
