"""Reactive flight: push-away and hand-follow using the Multi-ranger deck.

Two behaviors (``--mode``):

- ``push``   - the drone hovers and gently shies away from anything that gets
  too close on the front/back/left/right beams. Wave a hand at it and it backs
  off. Great "it's alive" demo.
- ``follow`` - the drone keeps a fixed standoff distance from whatever is in
  front of it: move your hand toward it and it retreats, pull away and it
  follows.

SAFETY FIRST. Read experiments/reactive_flight/README.md before flying.
Quick recap:
- Always test with ``--dry-run`` first: it streams the *commanded* velocity to
  the dashboard WITHOUT taking off, so you can verify the control law on a desk.
- Fly in an open area, low ceiling clear.
- Kill switches: Ctrl+C lands immediately; covering the UP sensor (hand over the
  top) also triggers an immediate landing.

Run (from the repo root, after ``pip install -e .``)::

    python -m experiments.reactive_flight.run --mode push --dry-run
    python -m experiments.reactive_flight.run --mode push
    python -m experiments.reactive_flight.run --mode follow --target 0.4
"""
from __future__ import annotations

import argparse
import time

from cflib.positioning.motion_commander import MotionCommander

from experiments.common import install_stop_handler, state_payload
from drl.config import ServerConfig
from drl.connection import connect
from drl.recording import CsvRecorder
from drl.sensors.ranger import RangerReading, RangerStream
from drl.dashboard import DashboardServer, Frame
from drl.telemetry import TelemetryHub, make_state_config


def _clamp(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


def push_command(r: RangerReading, safe_m: float, gain: float, vmax: float) -> tuple[float, float]:
    """Velocity (vx fwd, vy left) that pushes away from near obstacles."""
    vx = vy = 0.0
    if r.front is not None and r.front < safe_m:
        vx -= gain * (safe_m - r.front)
    if r.back is not None and r.back < safe_m:
        vx += gain * (safe_m - r.back)
    if r.left is not None and r.left < safe_m:
        vy -= gain * (safe_m - r.left)
    if r.right is not None and r.right < safe_m:
        vy += gain * (safe_m - r.right)
    return _clamp(vx, vmax), _clamp(vy, vmax)


def follow_command(
    r: RangerReading, target_m: float, track_max_m: float, gain: float, vmax: float
) -> tuple[float, float]:
    """Velocity that holds a fixed standoff distance from a front object."""
    if r.front is None or r.front > track_max_m:
        return 0.0, 0.0  # no target in range: hover
    err = r.front - target_m  # >0 means object is farther than target -> follow it
    return _clamp(gain * err, vmax), 0.0


def main() -> int:
    parser = argparse.ArgumentParser(description="drl: reactive flight")
    parser.add_argument("--mode", choices=["push", "follow"], default="push")
    parser.add_argument("--uri", default=None)
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--height", type=float, default=0.4, help="hover height (m)")
    parser.add_argument("--max-speed", type=float, default=0.3, help="velocity clamp (m/s)")
    parser.add_argument("--safe", type=float, default=0.5, help="push: react distance (m)")
    parser.add_argument("--target", type=float, default=0.4, help="follow: standoff distance (m)")
    parser.add_argument("--track-max", type=float, default=1.2, help="follow: max tracking distance (m)")
    parser.add_argument("--gain", type=float, default=1.2, help="proportional gain")
    parser.add_argument("--land-gesture", type=float, default=0.25,
                        help="land if UP beam is closer than this (m); 0 disables")
    parser.add_argument("--rate-hz", type=float, default=20.0, help="control loop rate")
    parser.add_argument("--dry-run", action="store_true",
                        help="do NOT take off; only stream sensors + computed command")
    parser.add_argument("--no-record", action="store_true")
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()

    stop = install_stop_handler()
    server = DashboardServer(ServerConfig(port=args.port)).start(open_browser=not args.no_browser)
    server.publish(Frame("meta", {"experiment": f"reactive flight ({args.mode})"
                                  + (" [dry-run]" if args.dry_run else "")}))
    recorder = None if args.no_record else CsvRecorder(f"reactive_{args.mode}")
    dt = 1.0 / args.rate_hz

    def compute(r: RangerReading) -> tuple[float, float]:
        if args.mode == "push":
            return push_command(r, args.safe, args.gain, args.max_speed)
        return follow_command(r, args.target, args.track_max, args.gain, args.max_speed)

    def publish_sensors(r: RangerReading, vx: float, vy: float, label: str) -> None:
        server.publish(Frame("ranger", r.as_dict()))
        server.publish(Frame("cmd", {"vx": vx, "vy": vy, "label": label}))
        if recorder is not None:
            recorder.write({"mode": args.mode, "label": label, "vx": vx, "vy": vy, **r.as_dict()})

    with connect(args.uri, arm=not args.dry_run,
                 reset_estimator_on_connect=not args.dry_run) as link:
        if not args.dry_run:
            link.require_decks("flow2", "multiranger")

        hub = TelemetryHub(link.scf)
        hub.add_config(make_state_config(50))
        hub.subscribe(lambda b, ts, s: server.publish(Frame("state", state_payload(s)))
                      if b == "state" else None)

        with hub, RangerStream(link.scf, rate_ms=50) as ranger:
            time.sleep(0.5)  # let the first samples arrive

            if args.dry_run:
                print("DRY RUN: computing commands without flying. Ctrl+C to stop.")
                while not stop.is_set():
                    r = ranger.reading()
                    vx, vy = compute(r)
                    publish_sensors(r, vx, vy, "dry-run")
                    time.sleep(dt)
            else:
                print(f"Taking off to {args.height} m. Cover the UP sensor or Ctrl+C to land.")
                with MotionCommander(link.scf, default_height=args.height) as mc:
                    time.sleep(1.0)
                    while not stop.is_set():
                        r = ranger.reading()
                        # Physical kill switch: hand over the top -> land.
                        if args.land_gesture and r.up is not None and r.up < args.land_gesture:
                            print("UP-sensor land gesture detected. Landing.")
                            break
                        vx, vy = compute(r)
                        mc.start_linear_motion(vx, vy, 0.0)
                        publish_sensors(r, vx, vy, args.mode)
                        time.sleep(dt)
                    # MotionCommander context lands automatically on exit.

    if recorder is not None:
        recorder.close()
    server.stop()
    print("Landed and stopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
