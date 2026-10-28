"""Chase a hand held in front of the drone.

The platform holds the front Multi-ranger beam near a standoff distance: it
moves forward when the hand backs away and reverses when the hand comes
closer. A hand that slides to one side shortens that side beam, and the drone
yaws toward it. Nothing in front, or a return beyond ``--lost-range``, stops
the chase and the drone yaws in place to look for the hand again.

SAFETY: flies autonomously. Keep the area clear beyond the lost-range, and
stay in front of the drone rather than beside a wall. Ctrl+C lands.

Run (from the repo root, after ``pip install -e .``)::

    python -m experiments.hand_chase.run
    python -m experiments.hand_chase.run --standoff 0.4 --duration 20
"""
from __future__ import annotations

import argparse
import time

from experiments.hand_chase.control import ChaseParams, HandChase
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
    "body_vx", "body_vy", "vx", "vy", "vz", "yaw_rate", "label",
]


def params_from_args(args: argparse.Namespace) -> ChaseParams:
    return ChaseParams(
        standoff=args.standoff,
        max_speed=args.max_speed,
        too_close=args.too_close,
        lost_range=args.lost_range,
        target_height=args.height,
    )


def _check_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if args.climb_rate <= 0:
        parser.error("--climb-rate must be positive")
    if args.standoff <= args.too_close:
        parser.error("--standoff must be greater than --too-close")
    if args.lost_range <= args.standoff:
        parser.error("--lost-range must be greater than --standoff")
    if args.max_speed <= 0:
        parser.error("--max-speed must be positive")
    if args.rate_hz <= 0:
        parser.error("--rate-hz must be positive")


def main() -> int:
    parser = argparse.ArgumentParser(description="drl: hand chase")
    add_experiment_args(parser, record=True)
    parser.add_argument("--duration", type=float, default=30.0,
                        help="flight duration in seconds (0 runs until Ctrl+C)")
    parser.add_argument("--rate-hz", type=float, default=20.0, help="control loop rate")
    parser.add_argument("--height", type=float, default=0.4, help="hover height (m)")
    parser.add_argument("--climb-rate", type=float, default=0.3, help="climb/descent speed (m/s)")
    parser.add_argument("--standoff", type=float, default=0.30, help="desired front range (m)")
    parser.add_argument("--lost-range", type=float, default=1.2,
                        help="front range at which the chase gives up (m)")
    parser.add_argument("--too-close", type=float, default=0.12, help="hard reverse distance (m)")
    parser.add_argument("--max-speed", type=float, default=0.25, help="horizontal speed clamp (m/s)")
    args = parser.parse_args()
    _check_args(parser, args)
    params = params_from_args(args)

    demo_simulator = None
    if args.demo:
        from experiments.hand_chase.demo import simulate as demo_simulator

    with ExperimentSession(
        "hand chase",
        port=args.port,
        uri=args.uri,
        open_browser=not args.no_browser,
        connect_drone=True,
        arm=True,
        record=None if args.no_record else "hand_chase",
        record_fieldnames=_FIELDS,
        demo=args.demo,
        demo_rate_hz=args.demo_rate,
        demo_simulator=demo_simulator,
    ) as sess:
        if args.demo:
            print("Previewing hand chase with synthetic data. Ctrl+C to stop.")
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

        chase = HandChase()

        def fly(flight: VelocityFlight) -> None:
            period = 1.0 / args.rate_hz
            started = time.monotonic()
            last = None
            last_label = None
            while not sess.stop.is_set():
                t, _dt = monotonic_elapsed(started, last)
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
                cmd = chase.step(ranges, pose[3], pose[2], params)
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
                    })
                if sess.stop.wait(period):
                    break

        with sess.hub:
            print(
                f"Chasing at {params.standoff:.2f} m standoff. "
                "Hold a hand in front of the drone. Ctrl+C to land."
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
