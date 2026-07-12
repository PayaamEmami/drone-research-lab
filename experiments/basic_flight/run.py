"""Minimal takeoff-hover-land flight to verify the platform can fly.

Uses the Crazyflie firmware position controller: ramp altitude up once, hold at
a fixed (x, y, z), then ramp down and stop. Simpler and calmer than an outer
velocity PID loop, which can bounce when the z estimate hits zero on the floor.

SAFETY: flies autonomously in a small area. Use ``--dry-run`` to validate the
connection and telemetry without arming or spinning motors. Ctrl+C lands.

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
from drl.config import ServerConfig, get_uri
from drl.connection import connect
from drl.dashboard import DashboardServer, Frame
from drl.motion import PositionFlight
from drl.recording import CsvRecorder
from drl.telemetry import TelemetryHub, make_battery_config, make_state_config

_MIN_BATTERY_V = 3.7
_CONTROL_HZ = 20.0


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


def _current_pose(hub: TelemetryHub) -> tuple[float, float, float, float] | None:
    latest = hub.latest("state")
    if latest is None:
        return None
    try:
        return (
            float(latest.get("stateEstimate.x", 0.0)),
            float(latest.get("stateEstimate.y", 0.0)),
            float(latest.get("stateEstimate.z", 0.0)),
            float(latest.get("stabilizer.yaw", 0.0)),
        )
    except (TypeError, ValueError):
        return None


def _wait_for_pose(hub: TelemetryHub, stop, *, timeout_s: float = 5.0) -> tuple[float, float, float, float] | None:
    deadline = time.monotonic() + timeout_s
    while not stop.is_set() and time.monotonic() < deadline:
        pose = _current_pose(hub)
        if pose is not None:
            return pose
        time.sleep(0.05)
    return None


def _publish(server: DashboardServer, hub: TelemetryHub, *, label: str, z_cmd: float) -> None:
    latest = hub.latest("state")
    if latest is not None:
        server.publish(Frame("state", state_payload(latest)))
    server.publish(Frame("cmd", {"vx": 0.0, "vy": 0.0, "vz": z_cmd, "label": label}))


def _record(
    recorder: CsvRecorder | None,
    *,
    phase: str,
    z: float,
    z_cmd: float,
    elapsed: float,
) -> None:
    if recorder is None:
        return
    recorder.write({
        "phase": phase,
        "z": z,
        "z_cmd": z_cmd,
        "elapsed": round(elapsed, 3),
    })


def _ramp_z(
    flight: PositionFlight,
    hub: TelemetryHub,
    server: DashboardServer,
    recorder: CsvRecorder | None,
    *,
    z_end: float,
    rate: float,
    phase: str,
    stop,
) -> None:
    """Linearly ramp the z setpoint; never commands upward during descent."""
    x, y, z_start, yaw = flight.position
    travel = z_end - z_start
    if abs(travel) < 1e-3:
        return

    period = 1.0 / _CONTROL_HZ
    duration = abs(travel) / rate
    steps = max(1, int(duration * _CONTROL_HZ))
    phase_t0 = time.monotonic()
    last_print = 0.0

    print(f"  {phase}: {z_start:.2f} m -> {z_end:.2f} m over ~{duration:.1f} s")
    for step in range(1, steps + 1):
        if stop.is_set():
            return
        alpha = step / steps
        z_cmd = z_start + travel * alpha
        flight.set_position(z=z_cmd)

        pose = _current_pose(hub)
        z_est = pose[2] if pose is not None else z_cmd
        elapsed = time.monotonic() - phase_t0
        _publish(server, hub, label=phase, z_cmd=z_cmd)
        _record(recorder, phase=phase, z=z_est, z_cmd=z_cmd, elapsed=elapsed)

        now = time.monotonic()
        if now - last_print >= 0.5:
            print(f"  {phase}: z_cmd = {z_cmd:.2f} m, z_est = {z_est:.2f} m")
            last_print = now
        time.sleep(period)


def _hold(
    flight: PositionFlight,
    hub: TelemetryHub,
    server: DashboardServer,
    recorder: CsvRecorder | None,
    *,
    duration_s: float,
    phase: str,
    stop,
) -> None:
    period = 1.0 / _CONTROL_HZ
    phase_t0 = time.monotonic()
    last_print = 0.0
    _, _, z_cmd, _ = flight.position

    while not stop.is_set() and (time.monotonic() - phase_t0) < duration_s:
        pose = _current_pose(hub)
        z_est = pose[2] if pose is not None else z_cmd
        elapsed = time.monotonic() - phase_t0
        _publish(server, hub, label=phase, z_cmd=z_cmd)
        _record(recorder, phase=phase, z=z_est, z_cmd=z_cmd, elapsed=elapsed)

        now = time.monotonic()
        if now - last_print >= 0.5:
            print(f"  {phase}: z_cmd = {z_cmd:.2f} m, z_est = {z_est:.2f} m")
            last_print = now
        time.sleep(period)

    if pose := _current_pose(hub):
        print(f"  {phase} complete at z_est = {pose[2]:.2f} m")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="drl: basic takeoff-hover-land (flight smoke test)",
    )
    parser.add_argument("--uri", default=None, help="Crazyflie radio URI override")
    parser.add_argument("--port", type=int, default=8000, help="dashboard port")
    parser.add_argument(
        "--height",
        type=float,
        default=0.5,
        help="hover height in meters (default 0.5 m, about 1.6 ft)",
    )
    parser.add_argument("--hover", type=float, default=4.0, help="hover duration at top (s)")
    parser.add_argument(
        "--ramp-rate",
        type=float,
        default=0.06,
        help="vertical ramp speed for climb and descent (m/s)",
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
    if args.ramp_rate <= 0:
        parser.error("--ramp-rate must be positive")

    stop = install_stop_handler()
    server = DashboardServer(ServerConfig(port=args.port)).start(open_browser=not args.no_browser)
    label = "basic flight" + (" [dry-run]" if args.dry_run else "")
    server.publish(Frame("meta", {"experiment": label}))
    recorder = None if args.no_record else CsvRecorder(
        "basic_flight",
        fieldnames=["phase", "z", "z_cmd", "elapsed"],
    )

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
        hub.add_config(make_state_config(int(1000 / _CONTROL_HZ)))
        hub.add_config(make_battery_config())

        def on_sample(block: str, ts: int, sample) -> None:  # noqa: ANN001
            if block == "battery":
                publish_battery(server, sample)

        hub.subscribe(on_sample)

        with hub:
            if args.dry_run:
                print("Dry run: connected. Streaming state for 10 s — motors will NOT spin.")
                t0 = time.monotonic()
                last_print = 0.0
                while not stop.is_set() and (time.monotonic() - t0) < 10.0:
                    pose = _current_pose(hub)
                    _publish(server, hub, label="dry-run", z_cmd=args.height)
                    if pose is not None:
                        _record(
                            recorder,
                            phase="dry-run",
                            z=pose[2],
                            z_cmd=args.height,
                            elapsed=time.monotonic() - t0,
                        )
                        if time.monotonic() - last_print >= 0.5:
                            print(f"  z = {pose[2]:.2f} m ({pose[2] * 3.28084:.1f} ft)")
                            last_print = time.monotonic()
                    time.sleep(1.0 / _CONTROL_HZ)
            else:
                flight = PositionFlight(link.scf)
                try:
                    print("Armed. Waiting for position estimate...")
                    pose = _wait_for_pose(hub, stop)
                    if pose is None:
                        raise RuntimeError("No position estimate received — check telemetry link.")
                    x0, y0, z0, yaw0 = pose
                    print(f"  home pose: x={x0:.2f}, y={y0:.2f}, z={z0:.2f}, yaw={yaw0:.0f} deg")

                    flight.start(x0, y0, z0, yaw0)
                    time.sleep(0.5)

                    print("Climbing...")
                    _ramp_z(
                        flight,
                        hub,
                        server,
                        recorder,
                        z_end=args.height,
                        rate=args.ramp_rate,
                        phase="climb",
                        stop=stop,
                    )

                    print(f"Holding for {args.hover:.1f} s. Ctrl+C to land early.")
                    _hold(
                        flight,
                        hub,
                        server,
                        recorder,
                        duration_s=args.hover,
                        phase="hover",
                        stop=stop,
                    )

                    print("Descending...")
                    _ramp_z(
                        flight,
                        hub,
                        server,
                        recorder,
                        z_end=0.0,
                        rate=args.ramp_rate,
                        phase="land",
                        stop=stop,
                    )
                    time.sleep(0.3)
                finally:
                    print("Stopping motors...")
                    flight.stop()
                    if pose := _current_pose(hub):
                        print(f"  final z estimate = {pose[2]:.2f} m")
                    print("Motors stopped.")

    if recorder is not None:
        recorder.close()
        print(f"Recording saved to {recorder.path}")
    server.stop()
    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
