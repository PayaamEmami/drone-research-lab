"""Minimal takeoff-hover-land flight to verify the platform can fly.

Climb straight up to a target height, hold briefly, then descend. This is the
simplest autonomous flight in the repo — use it to isolate radio, arming,
estimator, deck, and motor issues before running more complex experiments.

SAFETY: flies autonomously in a small area. Use ``--dry-run`` to validate the
connection and telemetry without arming or spinning motors. Ctrl+C lands.

Run (from the repo root, after ``pip install -e .``)::

    python -m scripts.connect_check
    python -m experiments.basic_flight.run --dry-run
    python -m experiments.basic_flight.run
    python -m experiments.basic_flight.run --height 0.6 --hover 5
"""
from __future__ import annotations

import argparse
import time

from experiments.common import install_stop_handler, publish_battery, state_payload
from drl.config import ServerConfig, get_uri
from drl.connection import connect
from drl.dashboard import DashboardServer, Frame
from drl.motion import VelocityFlight
from drl.recording import CsvRecorder
from drl.telemetry import TelemetryHub, make_battery_config, make_state_config

_MIN_BATTERY_V = 3.7


def _battery_v(link) -> float | None:  # noqa: ANN001
    try:
        return float(link.cf.param.get_value("pm.vbat"))
    except (KeyError, AttributeError, TypeError, ValueError):
        return None


def _print_preflight(link, uri: str, *, dry_run: bool) -> None:  # noqa: ANN001
    print("\n--- preflight ---")
    print(f"  uri      {uri}")
    decks = link.decks()
    if decks:
        for name, present in sorted(decks.items()):
            mark = "ok" if present else "MISSING"
            print(f"  deck {name:<12} {mark}")
    else:
        print("  decks    (none reported)")

    vbat = _battery_v(link)
    if vbat is not None:
        print(f"  battery  {vbat:.2f} V", end="")
        if vbat < _MIN_BATTERY_V:
            print(f"  WARNING: below {_MIN_BATTERY_V:.1f} V — charge before flying")
        else:
            print()
    else:
        print("  battery  unavailable")

    if not dry_run and not decks.get("flow2", False):
        print(
            "  WARNING: flow deck not detected. Vertical flight may work, but "
            "position hold and landing accuracy will be poor without optical flow."
        )
    print("---\n")


def _current_z(hub: TelemetryHub) -> float | None:
    latest = hub.latest("state")
    if latest is None:
        return None
    value = latest.get("stateEstimate.z")
    return float(value) if value is not None else None


def _publish_state(server: DashboardServer, hub: TelemetryHub) -> None:
    latest = hub.latest("state")
    if latest is None:
        return
    server.publish(Frame("state", state_payload(latest)))


def _hover_loop(
    hub: TelemetryHub,
    server: DashboardServer,
    recorder: CsvRecorder | None,
    *,
    duration_s: float,
    stop,
    label: str,
) -> None:
    t0 = time.monotonic()
    last_print = 0.0
    while not stop.is_set() and (time.monotonic() - t0) < duration_s:
        z = _current_z(hub)
        _publish_state(server, hub)
        if z is not None:
            server.publish(Frame("cmd", {"vx": 0.0, "vy": 0.0, "label": label}))
            if recorder is not None:
                recorder.write({"phase": label, "z": z, "elapsed": round(time.monotonic() - t0, 3)})
            now = time.monotonic()
            if now - last_print >= 0.5:
                print(f"  z = {z:.2f} m ({z * 3.28084:.1f} ft)")
                last_print = now
        time.sleep(0.05)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="drl: basic takeoff-hover-land (flight smoke test)",
    )
    parser.add_argument("--uri", default=None, help="Crazyflie radio URI override")
    parser.add_argument("--port", type=int, default=8000, help="dashboard port")
    parser.add_argument(
        "--height",
        type=float,
        default=0.6,
        help="climb height in meters (default 0.6 m, about 2 ft)",
    )
    parser.add_argument("--hover", type=float, default=3.0, help="hover duration at top (s)")
    parser.add_argument(
        "--climb-rate",
        type=float,
        default=0.25,
        help="vertical speed during climb/descent (m/s)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="connect and stream telemetry only; do NOT arm or fly",
    )
    parser.add_argument(
        "--skip-estimator-reset",
        action="store_true",
        help="skip Kalman reset (use if estimator settle times out on the desk)",
    )
    parser.add_argument("--no-record", action="store_true")
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()

    if args.height <= 0:
        parser.error("--height must be positive")
    if args.hover < 0:
        parser.error("--hover must be non-negative")

    stop = install_stop_handler()
    server = DashboardServer(ServerConfig(port=args.port)).start(open_browser=not args.no_browser)
    label = "basic flight" + (" [dry-run]" if args.dry_run else "")
    server.publish(Frame("meta", {"experiment": label}))
    recorder = None if args.no_record else CsvRecorder("basic_flight", fieldnames=["phase", "z", "elapsed"])

    fly = not args.dry_run
    reset_estimator = fly and not args.skip_estimator_reset

    uri = args.uri or get_uri()
    print(f"Starting {label}. Target height: {args.height:.2f} m ({args.height * 3.28084:.1f} ft).")

    with connect(
        uri,
        arm=fly,
        reset_estimator_on_connect=reset_estimator,
    ) as link:
        _print_preflight(link, uri, dry_run=args.dry_run)

        hub = TelemetryHub(link.scf)
        hub.add_config(make_state_config(50))
        hub.add_config(make_battery_config())

        def on_sample(block: str, ts: int, sample) -> None:  # noqa: ANN001
            if block == "battery":
                publish_battery(server, sample)

        hub.subscribe(on_sample)

        with hub:
            if args.dry_run:
                print("Dry run: connected. Streaming state for 10 s — motors will NOT spin.")
                print("If you see z changing while the drone sits still, the estimator is alive.")
                _hover_loop(
                    hub,
                    server,
                    recorder,
                    duration_s=10.0,
                    stop=stop,
                    label="dry-run",
                )
            else:
                if fly:
                    print("Armed. Taking off...")
                flight = VelocityFlight(
                    link.scf,
                    default_height=args.height,
                    takeoff_velocity=args.climb_rate,
                )
                try:
                    climb_time = args.height / args.climb_rate if args.climb_rate > 0 else 0.0
                    print(
                        f"Climbing at {args.climb_rate:.2f} m/s for ~{climb_time:.1f} s "
                        f"to {args.height:.2f} m..."
                    )
                    flight.take_off(args.height)
                    print(f"Hovering for {args.hover:.1f} s. Ctrl+C to land early.")
                    _hover_loop(
                        hub,
                        server,
                        recorder,
                        duration_s=args.hover,
                        stop=stop,
                        label="hover",
                    )
                finally:
                    print("Landing...")
                    flight.land()
                    z = _current_z(hub)
                    if z is not None:
                        print(f"  landed (z estimate = {z:.2f} m)")
                    print("Motors stopped.")

    if recorder is not None:
        recorder.close()
        print(f"Recording saved to {recorder.path}")
    server.stop()
    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
