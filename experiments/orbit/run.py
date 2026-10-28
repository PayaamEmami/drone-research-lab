"""Circle an object the front range beam can see.

The platform yaws until the front beam is the closest return inside the
acquisition window, then orbits: forward/back holds the range, a sideways
velocity walks the circle, and the yaw rate keeps the nose on the object.
A side beam inside ``--too-close``, or the front return disappearing, drops
the orbit and the search starts again.

SAFETY: flies autonomously in a small circle. Give the object a clear radius
and keep walls outside ``--too-close``. Ctrl+C lands.

Run (from the repo root, after ``pip install -e .``)::

    python -m experiments.orbit.run
    python -m experiments.orbit.run --radius 0.5 --direction cw
"""
from __future__ import annotations

import argparse
import time

from experiments.orbit.control import OrbitParams, OrbitTracker
from drl.cli import add_experiment_args
from drl.dashboard import Frame
from drl.motion import VelocityFlight
from drl.sensors import Sensors
from drl.session import ExperimentSession
from drl.telemetry import (
    make_battery_config,
    make_flow_config,
    make_multiranger_config,
    make_state_config,
)
from drl.timing import monotonic_elapsed

_BEAMS = ("front", "back", "left", "right", "up", "down")
_FIELDS = [
    "elapsed_s", "t", "x", "y", "z", "yaw",
    *_BEAMS,
    "body_vx", "body_vy", "vx", "vy", "vz", "yaw_rate", "label", "mode",
]


def params_from_args(args: argparse.Namespace) -> OrbitParams:
    direction = 1 if args.direction == "ccw" else -1
    return OrbitParams(
        radius=args.radius,
        tangential_speed=args.speed,
        direction=direction,
        object_radius=args.object_radius,
        max_speed=args.max_speed,
        too_close=args.too_close,
        acquire_max=args.acquire_max,
        lose_range=args.lose_range,
        target_height=args.height,
    )


def _check_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if args.climb_rate <= 0:
        parser.error("--climb-rate must be positive")
    if args.rate_hz <= 0:
        parser.error("--rate-hz must be positive")
    if args.max_speed <= 0 or args.speed <= 0:
        parser.error("--max-speed and --speed must be positive")
    if args.radius <= args.too_close:
        parser.error("--radius must be greater than --too-close")
    if args.acquire_max <= args.too_close:
        parser.error("--acquire-max must be greater than --too-close")
    if args.lose_range <= args.acquire_max:
        parser.error("--lose-range must be greater than --acquire-max")
    if args.object_radius < 0:
        parser.error("--object-radius must be non-negative")


def main() -> int:
    parser = argparse.ArgumentParser(description="drl: orbit")
    add_experiment_args(parser, record=True)
    parser.add_argument("--duration", type=float, default=30.0,
                        help="flight duration in seconds (0 runs until Ctrl+C)")
    parser.add_argument("--rate-hz", type=float, default=20.0, help="control loop rate")
    parser.add_argument("--height", type=float, default=0.4, help="hover height (m)")
    parser.add_argument("--climb-rate", type=float, default=0.3, help="climb/descent speed (m/s)")
    parser.add_argument("--radius", type=float, default=0.45, help="desired front range while orbiting (m)")
    parser.add_argument("--speed", type=float, default=0.15, help="tangential speed on the circle (m/s)")
    parser.add_argument("--direction", choices=("ccw", "cw"), default="ccw",
                        help="orbit direction")
    parser.add_argument("--object-radius", type=float, default=0.20,
                        help="guess of the object's radius, used to pace yaw (m)")
    parser.add_argument("--acquire-max", type=float, default=0.80,
                        help="farthest front range that counts as a target (m)")
    parser.add_argument("--lose-range", type=float, default=1.20,
                        help="front range that breaks the orbit (m)")
    parser.add_argument("--too-close", type=float, default=0.15, help="hard reverse / abort distance (m)")
    parser.add_argument("--max-speed", type=float, default=0.25, help="horizontal speed clamp (m/s)")
    args = parser.parse_args()
    _check_args(parser, args)
    params = params_from_args(args)

    demo_simulator = None
    if args.demo:
        from experiments.orbit.demo import simulate as demo_simulator

    with ExperimentSession(
        "orbit",
        port=args.port,
        uri=args.uri,
        open_browser=not args.no_browser,
        connect_drone=True,
        arm=True,
        record=None if args.no_record else "orbit",
        record_fieldnames=_FIELDS,
        demo=args.demo,
        demo_rate_hz=args.demo_rate,
        demo_simulator=demo_simulator,
    ) as sess:
        if args.demo:
            print("Previewing orbit with synthetic data. Ctrl+C to stop.")
            sess.run_demo()
            print("Done.")
            return 0

        sess.link.require_decks("flow2", "multiranger")

        rate_ms = max(10, int(1000 / args.rate_hz))
        sess.hub.add_config(make_state_config(rate_ms))
        sess.hub.add_config(make_multiranger_config(rate_ms))
        sess.hub.add_config(make_flow_config(rate_ms))
        sess.hub.add_config(make_battery_config())
        sess.hub.attach_dashboard(sess.server, auto=["battery", "state", "ranger"])

        tracker = OrbitTracker()

        def fly(flight: VelocityFlight) -> None:
            period = 1.0 / args.rate_hz
            started = time.monotonic()
            last = None
            last_label = None
            while not sess.stop.is_set():
                t, dt = monotonic_elapsed(started, last)
                last = started + t
                if args.duration > 0 and t >= args.duration:
                    break
                pose = sess.hub.pose()
                if pose is None:
                    flight.hover()
                    if sess.stop.wait(period):
                        break
                    continue
                ranges = Sensors.from_hub(sess.hub).as_dict()
                cmd = tracker.step(ranges, pose[3], pose[2], dt, params)
                flight.send_velocity(cmd.vx, cmd.vy, cmd.vz, cmd.yaw_rate)
                sess.server.publish(Frame("cmd", {
                    "vx": cmd.body_vx, "vy": cmd.body_vy, "label": cmd.label,
                }))
                if cmd.label != last_label:
                    front = ranges.get("front")
                    front_s = "--" if front is None else f"{front:.2f} m"
                    print(f"{cmd.label} (front {front_s})")
                    last_label = cmd.label
                if sess.recorder is not None:
                    sess.recorder.write({
                        "t": round(t, 3),
                        "x": pose[0], "y": pose[1], "z": pose[2], "yaw": pose[3],
                        **{beam: ranges.get(beam) for beam in _BEAMS},
                        "body_vx": cmd.body_vx, "body_vy": cmd.body_vy,
                        "vx": cmd.vx, "vy": cmd.vy, "vz": cmd.vz,
                        "yaw_rate": cmd.yaw_rate, "label": cmd.label,
                        "mode": tracker.mode,
                    })
                if sess.stop.wait(period):
                    break

        with sess.hub:
            way = "counterclockwise" if params.direction > 0 else "clockwise"
            print(
                f"Orbiting {way} at {params.radius:.2f} m. "
                "Put an object in front of the drone. Ctrl+C to land."
            )
            flight = VelocityFlight(
                sess.link.scf,
                default_height=args.height,
                takeoff_velocity=args.climb_rate,
            )
            try:
                flight.take_off(args.height)
                fly(flight)
            finally:
                pose = sess.hub.pose()
                flight.land(from_height=None if pose is None else pose[2])

    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
