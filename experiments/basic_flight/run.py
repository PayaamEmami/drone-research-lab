"""Minimal takeoff-hover-land flight: climb a set height, hover, then land.

The simplest autonomous flight in the repo — use it to confirm the drone flies
before running more complex experiments. Requires the Flow deck: without it the
drone cannot estimate height and will not fly stably.

SAFETY: flies autonomously in a small area. Use ``--dry-run`` to check the link
without arming or spinning motors. Ctrl+C lands.

Run (from the repo root, after ``pip install -e .``)::

    python -m scripts.connect_check
    python -m experiments.basic_flight.run --dry-run
    python -m experiments.basic_flight.run
    python -m experiments.basic_flight.run --height 0.5 --hover 5
"""
from __future__ import annotations

import argparse
import time

from experiments.common import install_stop_handler, publish_battery, state_payload
from drl.config import ServerConfig
from drl.connection import connect
from drl.dashboard import DashboardServer, Frame
from drl.motion import VelocityFlight
from drl.telemetry import TelemetryHub, make_battery_config, make_state_config


def main() -> int:
    parser = argparse.ArgumentParser(description="drl: basic takeoff-hover-land")
    parser.add_argument("--uri", default=None, help="Crazyflie radio URI override")
    parser.add_argument("--port", type=int, default=8000, help="dashboard port")
    parser.add_argument("--height", type=float, default=0.5, help="hover height (m)")
    parser.add_argument("--hover", type=float, default=4.0, help="hover duration (s)")
    parser.add_argument("--climb-rate", type=float, default=0.3, help="climb/descent speed (m/s)")
    parser.add_argument("--dry-run", action="store_true",
                        help="connect and stream telemetry only; do NOT arm or fly")
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()

    stop = install_stop_handler()
    server = DashboardServer(ServerConfig(port=args.port)).start(open_browser=not args.no_browser)
    server.publish(Frame("meta", {"experiment": "basic flight"
                                  + (" [dry-run]" if args.dry_run else "")}))

    fly = not args.dry_run
    with connect(args.uri, arm=fly, reset_estimator_on_connect=fly) as link:
        if fly and not link.decks().get("flow2", False):
            raise RuntimeError(
                "Flow deck not detected. Basic flight needs it for altitude hold. "
                "Mount the Flow deck under the drone and retry."
            )

        hub = TelemetryHub(link.scf)
        hub.add_config(make_state_config())
        hub.add_config(make_battery_config())

        def on_sample(block: str, ts: int, sample) -> None:  # noqa: ANN001
            if block == "battery":
                publish_battery(server, sample)
            elif block == "state":
                server.publish(Frame("state", state_payload(sample)))

        hub.subscribe(on_sample)

        with hub:
            if args.dry_run:
                print("Dry run: connected, motors will NOT spin. Streaming for 10 s (Ctrl+C to stop).")
                stop.wait(10.0)
            else:
                print(f"Taking off to {args.height:.2f} m ({args.height * 3.28084:.1f} ft)...")
                with VelocityFlight(link.scf, default_height=args.height,
                                    takeoff_velocity=args.climb_rate):
                    print(f"Hovering for {args.hover:.1f} s. Ctrl+C to land early.")
                    stop.wait(args.hover)
                print("Landed.")

    server.stop()
    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
