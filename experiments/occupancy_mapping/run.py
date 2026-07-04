"""Live Multi-ranger occupancy mapping.

Builds a 2D occupancy grid of the room from the Multi-ranger beams while using
the Crazyflie's onboard state estimate for pose. The map updates live on the
dashboard, with the drone pose and the most recent beam hits overlaid.

Flight patterns (``--pattern``):

- ``spin``   - take off, rotate slowly in place to sweep the room, land.
- ``square`` - take off, fly a small square, land.
- ``hover``  - take off and just hover (you nudge it / it drifts); land on stop.
- ``no-fly`` - do NOT take off; map while you carry the drone by hand. Useful for
  validating the pipeline on a desk (move it over a textured floor so the flow
  deck tracks). Combine with the dashboard to watch the map form.

SAFETY: flying patterns take off autonomously. Read the safety notes in
experiments/reactive_flight/README.md (same rules apply). Ctrl+C lands.

Run (from the repo root, after ``pip install -e .``)::

    python -m experiments.occupancy_mapping.run --pattern no-fly      # safe desk test
    python -m experiments.occupancy_mapping.run --pattern spin
    python -m experiments.occupancy_mapping.run --pattern square --side 0.8
"""
from __future__ import annotations

import argparse
import logging
import threading
import time

from cflib.positioning.motion_commander import MotionCommander

from experiments.common import install_stop_handler, state_payload, yaw_radians
from experiments.occupancy_mapping.mapper import MapConfig, OccupancyGrid
from drl.config import ServerConfig
from drl.connection import connect
from drl.sensors.ranger import RangerReading
from drl.dashboard import DashboardServer, Frame
from drl.telemetry import TelemetryHub, make_ranger_config, make_state_config


def fly_pattern(mc: MotionCommander, pattern: str, args, stop) -> None:
    """Execute the chosen flight pattern (blocking) unless stopped."""
    time.sleep(1.0)
    if pattern == "spin":
        # Several slow partial turns so we can bail out between segments.
        for _ in range(4):
            if stop.is_set():
                return
            mc.turn_left(90, rate=args.yaw_rate)
            time.sleep(0.5)
    elif pattern == "square":
        for _ in range(4):
            if stop.is_set():
                return
            mc.forward(args.side, velocity=0.2)
            mc.turn_left(90, rate=args.yaw_rate)
            time.sleep(0.3)
    else:  # hover
        while not stop.is_set():
            time.sleep(0.1)


def main() -> int:
    parser = argparse.ArgumentParser(description="drl: occupancy mapping")
    parser.add_argument("--pattern", choices=["spin", "square", "hover", "no-fly"], default="spin")
    parser.add_argument("--uri", default=None)
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--height", type=float, default=0.4)
    parser.add_argument("--side", type=float, default=0.8, help="square: side length (m)")
    parser.add_argument("--yaw-rate", type=float, default=45.0, help="spin/turn rate (deg/s)")
    parser.add_argument("--size", type=float, default=8.0, help="map side length (m)")
    parser.add_argument("--res", type=float, default=0.05, help="map resolution (m/cell)")
    parser.add_argument("--map-hz", type=float, default=5.0, help="map broadcast rate")
    parser.add_argument("--save", default=None, help="path to save final grid as .npz")
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()

    fly = args.pattern != "no-fly"
    stop = install_stop_handler()
    server = DashboardServer(ServerConfig(port=args.port)).start(open_browser=not args.no_browser)
    server.publish(Frame("meta", {"experiment": f"occupancy mapping ({args.pattern})"}))

    grid = OccupancyGrid(MapConfig(size_m=args.size, resolution_m=args.res))

    with connect(args.uri, arm=fly, reset_estimator_on_connect=fly) as link:
        if fly:
            link.require_decks("flow2", "multiranger")

        hub = TelemetryHub(link.scf)
        hub.add_config(make_state_config(50))
        hub.add_config(make_ranger_config(50))

        def on_sample(block: str, ts: int, sample) -> None:
            if block == "ranger":
                state = hub.latest("state") or {}
                reading = RangerReading.from_sample(sample)
                grid.integrate(
                    x=state.get("stateEstimate.x", 0.0) or 0.0,
                    y=state.get("stateEstimate.y", 0.0) or 0.0,
                    yaw_rad=yaw_radians(state),
                    ranges=reading.as_dict(),
                )
                server.publish(Frame("ranger", reading.as_dict()))
            elif block == "state":
                server.publish(Frame("state", state_payload(sample)))

        hub.subscribe(on_sample)

        # Background thread broadcasts the (heavier) map payload at a fixed rate.
        def map_publisher() -> None:
            period = 1.0 / args.map_hz
            while not stop.is_set():
                try:
                    s = hub.latest("state") or {}
                    pose = (
                        s.get("stateEstimate.x", 0.0) or 0.0,
                        s.get("stateEstimate.y", 0.0) or 0.0,
                        yaw_radians(s),
                    )
                    server.publish(Frame("map", grid.to_payload(pose)))
                except Exception:  # noqa: BLE001 - keep publisher alive
                    logging.exception("map publisher failed")
                time.sleep(period)

        with hub:
            time.sleep(0.5)
            server.publish(Frame("map", grid.to_payload()))
            pub = threading.Thread(target=map_publisher, daemon=True, name="map-publisher")
            pub.start()

            if fly:
                print(f"Mapping with pattern '{args.pattern}'. Ctrl+C to land.")
                with MotionCommander(link.scf, default_height=args.height) as mc:
                    fly_pattern(mc, args.pattern, args, stop)
            else:
                print("NO-FLY mapping: carry the drone over a textured floor. Ctrl+C to stop.")
                while not stop.is_set():
                    time.sleep(0.1)

            stop.set()
            pub.join(timeout=2.0)

    # Final map + optional save.
    server.publish(Frame("map", grid.to_payload()))
    if args.save:
        grid.save_npz(args.save)
        print(f"Saved grid to {args.save}")
    time.sleep(0.3)
    server.stop()
    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
