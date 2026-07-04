"""Live 5-beam proximity HUD.

Streams the Multi-ranger's five beams (front/back/left/right/up) plus the
down-facing z-range to the web dashboard, where they are drawn as a radial HUD
and a rolling time-series chart. No flight involved: the drone can sit on your
desk while you wave objects at the sensors and watch the data react live.

Run (from the repo root, after ``pip install -e .``)::

    python -m experiments.proximity_hud.run
    python -m experiments.proximity_hud.run --no-record --port 8000

Then open the printed URL in a browser. Ctrl+C to stop.
"""
from __future__ import annotations

import argparse
import time

from experiments.common import install_stop_handler, state_payload
from drl.config import ServerConfig
from drl.connection import connect
from drl.recording import CsvRecorder
from drl.sensors.ranger import RangerReading
from drl.dashboard import DashboardServer, Frame
from drl.telemetry import TelemetryHub, make_ranger_config, make_state_config


def main() -> int:
    parser = argparse.ArgumentParser(description="drl: live proximity HUD")
    parser.add_argument("--uri", default=None, help="Crazyflie radio URI override")
    parser.add_argument("--port", type=int, default=8000, help="dashboard port")
    parser.add_argument("--rate-ms", type=int, default=50, help="sensor log period")
    parser.add_argument("--no-record", action="store_true", help="disable CSV recording")
    parser.add_argument("--no-browser", action="store_true", help="don't auto-open browser")
    args = parser.parse_args()

    stop = install_stop_handler()
    server = DashboardServer(ServerConfig(port=args.port)).start(open_browser=not args.no_browser)
    server.publish(Frame("meta", {"experiment": "proximity HUD"}))

    recorder = None if args.no_record else CsvRecorder("proximity")

    with connect(args.uri) as link:
        # The HUD does not require a deck to *run*, but warn if the ranger is absent.
        decks = link.decks()
        if not decks.get("multiranger", False):
            print("WARNING: Multi-ranger deck not detected; beams will read out of range.")

        hub = TelemetryHub(link.scf)
        hub.add_config(make_ranger_config(args.rate_ms))
        hub.add_config(make_state_config(args.rate_ms))

        def on_sample(block: str, ts: int, sample) -> None:
            if block == "ranger":
                reading = RangerReading.from_sample(sample)
                server.publish(Frame("ranger", reading.as_dict()))
                if recorder is not None:
                    recorder.write({"block": "ranger", **reading.as_dict()})
            elif block == "state":
                server.publish(Frame("state", state_payload(sample)))

        hub.subscribe(on_sample)

        with hub:
            print("Streaming proximity data. Wave something near the sensors. Ctrl+C to stop.")
            while not stop.is_set():
                time.sleep(0.1)

    if recorder is not None:
        recorder.close()
    server.stop()
    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
