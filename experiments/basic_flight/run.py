"""Minimal takeoff-hover-land flight: climb a set height, hover, then land.

The simplest autonomous flight in the repo — use it to confirm the drone flies
before running more complex experiments. Requires the Flow deck: without it the
drone cannot estimate height and will not fly stably.

SAFETY: flies autonomously in a small area. Ctrl+C lands.

Run (from the repo root, after ``pip install -e .``)::

    python -m scripts.connect_check
    python -m experiments.basic_flight.run
    python -m experiments.basic_flight.run --height 0.5 --hover 5
"""
from __future__ import annotations

import argparse

from drl.cli import add_experiment_args
from drl.motion import VelocityFlight
from drl.session import ExperimentSession
from drl.telemetry import make_battery_config, make_state_config


def main() -> int:
    parser = argparse.ArgumentParser(description="drl: basic takeoff-hover-land")
    add_experiment_args(parser)
    parser.add_argument("--height", type=float, default=0.5, help="hover height (m)")
    parser.add_argument("--hover", type=float, default=4.0, help="hover duration (s)")
    parser.add_argument("--climb-rate", type=float, default=0.3, help="climb/descent speed (m/s)")
    args = parser.parse_args()

    with ExperimentSession(
        "basic flight",
        port=args.port,
        uri=args.uri,
        open_browser=not args.no_browser,
        connect_drone=True,
        arm=True,
    ) as sess:
        sess.link.require_decks("flow2")

        sess.hub.add_config(make_state_config())
        sess.hub.add_config(make_battery_config())
        sess.hub.attach_dashboard(sess.server, auto=["battery", "state"])

        with sess.hub:
            print(f"Taking off to {args.height:.2f} m ({args.height * 3.28084:.1f} ft)...")
            with VelocityFlight(sess.link.scf, default_height=args.height,
                                takeoff_velocity=args.climb_rate):
                print(f"Hovering for {args.hover:.1f} s. Ctrl+C to land early.")
                sess.stop.wait(args.hover)
            print("Landed.")

    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
