"""Live proximity sensing: watch the platform's range beams react to its surroundings.

The platform sits still (no flight) while its six time-of-flight beams - the five
Multi-ranger beams (front/back/left/right/up) and the Flow deck's downward range
finder - report how close objects are in each direction. Move a hand, a wall, or
any object near a beam and the dashboard's Proximity HUD and raw-values table
update in real time.

This is the simplest sensing experiment in the repo: it only reads the
environment and displays it, with no filtering, control, or mapping on top.

Run (from the repo root, after ``pip install -e .``)::

    python -m experiments.proximity_sensing.run
    python -m experiments.proximity_sensing.run --no-record --port 8000
"""
from __future__ import annotations

import argparse
import time

from drl.cli import add_experiment_args
from drl.sensors import Sensors
from drl.session import ExperimentSession
from drl.telemetry import (
    make_battery_config,
    make_flow_config,
    make_multiranger_config,
)

_RANGE_BEAMS = ("front", "back", "left", "right", "up", "down")


def main() -> int:
    parser = argparse.ArgumentParser(description="drl: proximity sensing")
    add_experiment_args(parser, record=True)
    parser.add_argument("--rate-ms", type=int, default=50, help="sensor log period")
    args = parser.parse_args()

    fieldnames = ["elapsed_s"] + list(_RANGE_BEAMS)

    with ExperimentSession(
        "proximity sensing",
        port=args.port,
        uri=args.uri,
        open_browser=not args.no_browser,
        connect_drone=True,
        arm=False,
        record=None if args.no_record else "proximity_sensing",
        record_fieldnames=fieldnames,
        demo=args.demo,
        demo_frames=("battery", "ranger"),
        demo_rate_hz=args.demo_rate,
    ) as sess:
        if args.demo:
            print("Previewing proximity sensing with synthetic data. Ctrl+C to stop.")
            sess.run_demo()
            print("Done.")
            return 0

        decks = sess.link.decks()
        if not decks.get("multiranger", False):
            print("WARNING: Multi-ranger deck not detected; beams will read out of range.")

        sess.hub.add_config(make_multiranger_config(args.rate_ms))
        sess.hub.add_config(make_flow_config(args.rate_ms))
        sess.hub.add_config(make_battery_config())
        sess.hub.attach_dashboard(sess.server, auto=["battery", "ranger"])

        def on_sample(block: str, ts: int, sample) -> None:
            if block not in ("multiranger", "flow") or sess.recorder is None:
                return
            ranges = Sensors.from_hub(sess.hub).as_dict()
            sess.recorder.write({beam: ranges.get(beam) for beam in _RANGE_BEAMS})

        sess.hub.subscribe(on_sample)

        with sess.hub:
            print("Sensing proximity. Move objects near the beams. Ctrl+C to stop.")
            while not sess.stop.is_set():
                time.sleep(0.1)

    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
