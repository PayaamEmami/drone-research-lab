"""Preview the live dashboard with synthetic telemetry — no Crazyflie required.

Use this to explore the UI, capture screenshots, or rehearse a demo before
powering on hardware. Each experiment mode publishes the same frame types as the
real runner (``meta``, ``ranger``, ``state``, ``estimate``, ``traj``, ``cmd``,
``map``, ``cloud``, ``battery``).

Usage (from the repo root, after ``pip install -e .``)::

    python -m scripts.dashboard_demo
    python -m scripts.dashboard_demo --experiment trajectory
    python -m scripts.dashboard_demo --experiment slam --port 8000
"""
from __future__ import annotations

import argparse
import math
import time
from typing import Dict, Optional, Tuple

from experiments.common import install_stop_handler
from experiments.slam.mapper import MapConfig, OccupancyGrid
from experiments.slam.pointcloud import PointCloud
from experiments.slam.run import SlamState
from experiments.slam.scan_match import MatchConfig
from experiments.state_estimation.filters import HeightFusionKalman, ScalarKalman
from experiments.trajectory_tracking.controller import PID, TrajectoryController
from experiments.trajectory_tracking.trajectory import SpiralParams, spiral
from drl.config import ServerConfig
from drl.dashboard import DashboardServer, Frame

Pose = Tuple[float, float, float]
_BEAM_BEARINGS = {
    "front": 0.0,
    "left": math.pi / 2,
    "back": math.pi,
    "right": -math.pi / 2,
}
_RANGE_BEAMS = ("front", "back", "left", "right", "up", "down")
_ATTITUDE = ("roll", "pitch", "yaw")

_META = {
    "state_estimation": "state estimation (Kalman filtering) [demo]",
    "trajectory": "trajectory tracking (spiral) [demo]",
    "slam": "SLAM (autonomous exploration) [demo]",
}


def _simulate_ranges(
    x: float,
    y: float,
    yaw: float,
    *,
    room: tuple[float, float, float, float] = (-2.0, 2.0, -2.0, 2.0),
    max_range: float = 3.5,
) -> Dict[str, Optional[float]]:
    xmin, xmax, ymin, ymax = room
    ranges: Dict[str, Optional[float]] = {}
    for beam, bearing in _BEAM_BEARINGS.items():
        angle = yaw + bearing
        dx, dy = math.cos(angle), math.sin(angle)
        best = math.inf
        if dx > 1e-9:
            best = min(best, (xmax - x) / dx)
        elif dx < -1e-9:
            best = min(best, (xmin - x) / dx)
        if dy > 1e-9:
            best = min(best, (ymax - y) / dy)
        elif dy < -1e-9:
            best = min(best, (ymin - y) / dy)
        ranges[beam] = best if best <= max_range else None
    ranges["up"] = 1.8
    ranges["down"] = 0.02
    return ranges


def _sleep(period: float, stop) -> None:  # noqa: ANN001
    if stop.wait(period):
        return


def _publish_battery(server: DashboardServer, t: float) -> None:
    vbat = 4.05 - 0.15 * (1.0 - math.cos(t / 90.0))
    server.publish(Frame("battery", {"vbat": round(vbat, 2)}))


def run_state_estimation(server: DashboardServer, stop, rate_hz: float) -> None:
    period = 1.0 / rate_hz
    range_filters = {beam: ScalarKalman(q=1.0, r=0.05) for beam in _RANGE_BEAMS}
    attitude_filters = {angle: ScalarKalman(q=5.0, r=0.5) for angle in _ATTITUDE}
    height_filter = HeightFusionKalman()
    estimate: dict = {}
    t0 = time.monotonic()

    while not stop.is_set():
        t = time.monotonic() - t0
        raw_ranges = {
            "front": 0.22 + 0.08 * math.sin(t * 0.9) + 0.02 * math.sin(t * 7.3),
            "back": 0.31 + 0.05 * math.cos(t * 0.7),
            "left": 0.18 + 0.06 * math.sin(t * 1.1 + 0.4),
            "right": None if math.sin(t * 0.35) > 0.85 else 0.24 + 0.04 * math.cos(t * 1.4),
            "up": 1.9 + 0.05 * math.sin(t * 0.5),
            "down": 0.01 + 0.004 * abs(math.sin(t * 2.0)),
        }
        server.publish(Frame("ranger", raw_ranges))

        state = {
            "x": 0.1 * math.sin(t * 0.2),
            "y": -0.1 * math.cos(t * 0.17),
            "z": 0.002 + 0.001 * math.sin(t * 0.8),
            "roll": 0.6 * math.sin(t * 0.6),
            "pitch": -0.4 * math.cos(t * 0.5),
            "yaw": 12.0 * math.sin(t * 0.25),
        }
        server.publish(Frame("state", state))

        for beam in _RANGE_BEAMS:
            kf = range_filters[beam]
            kf.predict(period)
            kf.update(raw_ranges.get(beam))
            estimate[beam] = {"raw": raw_ranges.get(beam), "filtered": kf.value}

        height_filter.predict(period, 0.02 * math.sin(t * 1.7))
        height_filter.update(raw_ranges.get("down"))
        estimate["height"] = {"raw": raw_ranges.get("down"), "filtered": height_filter.height}

        for angle in _ATTITUDE:
            kf = attitude_filters[angle]
            kf.predict(period)
            kf.update(state[angle])
            estimate[angle] = {"raw": state[angle], "filtered": kf.value}

        server.publish(Frame("estimate", dict(estimate)))
        _publish_battery(server, t)
        _sleep(period, stop)


def run_trajectory(server: DashboardServer, stop, rate_hz: float) -> None:
    period = 1.0 / rate_hz
    params = SpiralParams()
    controller = TrajectoryController(
        x=PID(kp=1.0, out_limit=0.3),
        y=PID(kp=1.0, out_limit=0.3),
        z=PID(kp=1.0, out_limit=0.3),
    )
    est = [0.0, 0.0, 0.0]
    t0 = time.monotonic()
    last = None

    while not stop.is_set():
        now = time.monotonic()
        t = now - t0
        dt = period if last is None else now - last
        last = now

        ref = spiral(t, params)
        est[0] += 0.35 * (ref[0] - est[0])
        est[1] += 0.35 * (ref[1] - est[1])
        est[2] += 0.35 * (ref[2] - est[2])
        vx, vy, vz = controller.step(ref, tuple(est), dt)

        server.publish(Frame("state", {
            "x": est[0], "y": est[1], "z": est[2],
            "roll": 2.0 * math.sin(t * 0.4),
            "pitch": 1.5 * math.cos(t * 0.35),
            "yaw": math.degrees(math.atan2(ref[1], ref[0])),
        }))
        server.publish(Frame("traj", {
            "reference": {"x": ref[0], "y": ref[1], "z": ref[2]},
            "estimate": {"x": est[0], "y": est[1], "z": est[2]},
            "command": {"vx": vx, "vy": vy, "vz": vz},
        }))
        server.publish(Frame("cmd", {"vx": vx, "vy": vy, "label": "spiral [demo]"}))
        server.publish(Frame("ranger", _simulate_ranges(est[0], est[1], math.radians(t * 20))))
        _publish_battery(server, t)
        _sleep(period, stop)


def run_slam(server: DashboardServer, stop, rate_hz: float) -> None:
    period = 1.0 / rate_hz
    grid = OccupancyGrid(MapConfig(size_m=6.0))
    cloud = PointCloud()
    state = SlamState(grid, cloud, MatchConfig())
    t0 = time.monotonic()
    step = 0

    while not stop.is_set():
        t = time.monotonic() - t0
        radius = 0.8 + 0.15 * math.sin(t * 0.1)
        ekf: Pose = (
            radius * math.cos(0.35 * t),
            radius * math.sin(0.35 * t),
            0.35 * t + 0.05 * math.sin(t * 0.2),
        )
        drift = 0.03 * math.sin(t * 0.15)
        ekf_noisy: Pose = (ekf[0] + drift, ekf[1] - drift * 0.5, ekf[2])
        ranges = _simulate_ranges(ekf_noisy[0], ekf_noisy[1], ekf_noisy[2])
        state.step(ekf_noisy, 0.35, ranges)

        server.publish(Frame("ranger", ranges))
        server.publish(Frame("state", {
            "x": ekf_noisy[0],
            "y": ekf_noisy[1],
            "z": 0.35,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": math.degrees(ekf_noisy[2]),
        }))
        if step % 3 == 0:
            payload = state.map_payload()
            if payload is not None:
                server.publish(Frame("map", payload))
            server.publish(Frame("cloud", cloud.to_payload()))
        _publish_battery(server, t)
        step += 1
        _sleep(period, stop)


_RUNNERS = {
    "state_estimation": run_state_estimation,
    "trajectory": run_trajectory,
    "slam": run_slam,
}


def main() -> int:
    parser = argparse.ArgumentParser(description="drl: dashboard demo (no hardware)")
    parser.add_argument(
        "--experiment",
        choices=sorted(_RUNNERS),
        default="state_estimation",
        help="which experiment layout and frames to simulate",
    )
    parser.add_argument("--port", type=int, default=8000, help="dashboard port")
    parser.add_argument("--rate-hz", type=float, default=20.0, help="simulation update rate")
    parser.add_argument("--no-browser", action="store_true", help="don't auto-open browser")
    args = parser.parse_args()

    stop = install_stop_handler()
    server = DashboardServer(ServerConfig(port=args.port)).start(open_browser=not args.no_browser)
    server.publish(Frame("meta", {"experiment": _META[args.experiment]}))

    print(f"Dashboard demo: {args.experiment}. Synthetic data only — no radio link. Ctrl+C to stop.")
    try:
        _RUNNERS[args.experiment](server, stop, args.rate_hz)
    finally:
        server.stop()
    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
