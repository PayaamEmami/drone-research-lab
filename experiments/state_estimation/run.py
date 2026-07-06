"""Live state estimation: Kalman filtering across every onboard sensor.

This experiment reads *all* of the platform's sensors at once - the five
Multi-ranger beams, the downward range finder, the fused position/attitude
estimate, and the battery - and runs a Kalman filter over them so the dashboard
can show the raw signal next to its filtered estimate in real time.

Two layers of estimation are demonstrated:

1. Breadth: a per-channel :class:`~experiments.state_estimation.filters.ScalarKalman`
   smooths each scalar stream independently (denoising the whole sensor suite).
2. Depth: a :class:`~experiments.state_estimation.filters.HeightFusionKalman`
   fuses the vertical accelerometer with the downward range finder into a single
   height/velocity estimate.

No flight is involved - the platform can sit on a desk while you move objects
near the sensors and watch the raw vs. filtered traces respond.

Status: scaffold. The filter math and the dashboard "estimate" renderer are not
implemented yet; see the TODOs below and in ``filters.py``.

Run (from the repo root, after ``pip install -e .``)::

    python -m experiments.state_estimation.run
    python -m experiments.state_estimation.run --no-record --port 8000
"""
from __future__ import annotations

import argparse
import time

from experiments.common import install_stop_handler, state_payload
from experiments.state_estimation.filters import HeightFusionKalman, ScalarKalman
from drl.config import ServerConfig
from drl.connection import connect
from drl.dashboard import DashboardServer, Frame
from drl.recording import CsvRecorder
from drl.sensors.ranger import RangerReading
from drl.telemetry import (
    TelemetryHub,
    make_battery_config,
    make_ranger_config,
    make_state_config,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="drl: state estimation (Kalman filtering)")
    parser.add_argument("--uri", default=None, help="Crazyflie radio URI override")
    parser.add_argument("--port", type=int, default=8000, help="dashboard port")
    parser.add_argument("--rate-ms", type=int, default=50, help="sensor log period")
    parser.add_argument("--no-record", action="store_true", help="disable CSV recording")
    parser.add_argument("--no-browser", action="store_true", help="don't auto-open browser")
    args = parser.parse_args()

    stop = install_stop_handler()
    server = DashboardServer(ServerConfig(port=args.port)).start(open_browser=not args.no_browser)
    server.publish(Frame("meta", {"experiment": "state estimation (Kalman filtering)"}))
    recorder = None if args.no_record else CsvRecorder("state_estimation")

    # One scalar filter per smoothed channel, plus the height-fusion filter.
    # TODO(state_estimation): tune per-channel q/r; wire these into on_sample so
    # each incoming sample runs predict(dt) + update(z) and we publish an
    # "estimate" frame carrying {channel: {raw, filtered}} for the dashboard.
    range_filters = {beam: ScalarKalman() for beam in
                     ("front", "back", "left", "right", "up", "down")}
    height_filter = HeightFusionKalman()

    with connect(args.uri) as link:
        decks = link.decks()
        if not decks.get("multiranger", False):
            print("WARNING: Multi-ranger deck not detected; beams will read out of range.")

        hub = TelemetryHub(link.scf)
        hub.add_config(make_ranger_config(args.rate_ms))
        hub.add_config(make_state_config(args.rate_ms))
        hub.add_config(make_battery_config())

        def on_sample(block: str, ts: int, sample) -> None:
            if block == "ranger":
                reading = RangerReading.from_sample(sample)
                server.publish(Frame("ranger", reading.as_dict()))
                # TODO(state_estimation): predict+update each range_filters[beam]
                # and publish the raw vs. filtered pair.
            elif block == "state":
                server.publish(Frame("state", state_payload(sample)))
            # TODO(state_estimation): feed the height_filter from accel + down range.

        hub.subscribe(on_sample)

        with hub:
            print("Streaming sensors. Move objects near the beams. Ctrl+C to stop.")
            while not stop.is_set():
                time.sleep(0.1)

    if recorder is not None:
        recorder.close()
    server.stop()
    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
