"""Autonomous exploration + SLAM.

The platform explores a space on its own and builds two views of it live:

- a 2-D occupancy grid (with the trajectory trail and latest beam hits), and
- a 3-D point cloud of range hits that can be exported for an external viewer.

Localization is solved jointly with mapping: the onboard state estimate is used
as odometry and corrected each step by matching the current Multi-ranger scan
against the map so far (scan matching), so the map stays consistent as pose
drift accumulates. Exploration picks where to go from the map itself (frontier
detection + A* planning).

Modes (``--mode``):

- ``explore`` - take off and autonomously explore via frontiers (with periodic
  yaw sweeps to densify the sparse scan). Lands on completion or Ctrl+C.
- ``no-fly``  - do NOT take off; carry the platform by hand to validate the SLAM
  pipeline on a desk/floor.
- ``replay``  - no hardware; re-run SLAM over a recorded CSV (``--replay PATH``)
  to compare corrected vs. raw trajectory and re-render the map offline.

SAFETY: ``explore`` flies autonomously. Fly only in a small, bounded, obstacle-
padded area. Ctrl+C lands.

Run (from the repo root, after ``pip install -e .``)::

    python -m experiments.slam.run --mode no-fly
    python -m experiments.slam.run --mode explore
    python -m experiments.slam.run --mode replay --replay data/slam_xxx.csv
"""
from __future__ import annotations

import argparse
import csv
import math
import time
from typing import Dict, List, Optional, Tuple

from experiments.common import MapPublisher, install_stop_handler, publish_battery, yaw_radians
from experiments.slam.explorer import ExploreConfig, Explorer
from experiments.slam.mapper import MapConfig, OccupancyGrid
from experiments.slam.pointcloud import PointCloud
from experiments.slam.scan_match import MatchConfig, match_scan
from drl.config import ServerConfig
from drl.connection import connect
from drl.dashboard import DashboardServer, Frame
from drl.motion import VelocityFlight
from drl.recording import CsvRecorder
from drl.sensors.ranger import RangerReading
from drl.telemetry import (
    TelemetryHub,
    make_battery_config,
    make_ranger_config,
    make_state_config,
)

Pose = Tuple[float, float, float]

RECORD_FIELDS = [
    "elapsed_s",
    "ekf_x", "ekf_y", "ekf_yaw", "z",
    "x", "y", "yaw",
    "front", "back", "left", "right", "up", "down",
]


class SlamState:
    """Runs the shared SLAM step and keeps the artifacts the dashboard shows."""

    def __init__(self, grid: OccupancyGrid, cloud: PointCloud, match_cfg: MatchConfig):
        self.grid = grid
        self.cloud = cloud
        self.match_cfg = match_cfg
        self._last_ekf: Optional[Pose] = None
        self._corrected: Optional[Pose] = None
        self.trail: List[Tuple[float, float]] = []
        self.raw_trail: List[Tuple[float, float]] = []

    def step(self, ekf: Pose, z: float, ranges: Dict[str, Optional[float]]) -> Pose:
        if self._last_ekf is None or self._corrected is None:
            corrected = ekf
        else:
            # Predict from the odometry delta, then correct against the map.
            predicted = (
                self._corrected[0] + (ekf[0] - self._last_ekf[0]),
                self._corrected[1] + (ekf[1] - self._last_ekf[1]),
                self._corrected[2] + (ekf[2] - self._last_ekf[2]),
            )
            corrected = match_scan(self.grid, predicted, ranges, self.match_cfg)

        self.grid.integrate(corrected[0], corrected[1], corrected[2], ranges)
        self.cloud.add_scan(corrected[0], corrected[1], z, corrected[2], ranges)
        self.trail.append((corrected[0], corrected[1]))
        self.raw_trail.append((ekf[0], ekf[1]))
        self._last_ekf = ekf
        self._corrected = corrected
        return corrected

    def map_payload(self) -> Optional[dict]:
        if self._corrected is None:
            return None
        payload = self.grid.to_payload(self._corrected)
        payload["trail"] = [[round(x, 3), round(y, 3)] for x, y in self.trail[-500:]]
        payload["pose_raw"] = {"x": self._last_ekf[0], "y": self._last_ekf[1]} \
            if self._last_ekf else None
        return payload


def _sample_to_inputs(state_sample, ranger_sample) -> Tuple[Pose, float, Dict[str, Optional[float]]]:
    """Extract ``(ekf_pose, z, ranges)`` from raw telemetry samples."""
    ekf = (
        state_sample.get("stateEstimate.x", 0.0),
        state_sample.get("stateEstimate.y", 0.0),
        yaw_radians(state_sample),
    )
    z = state_sample.get("stateEstimate.z", 0.0)
    ranges = RangerReading.from_sample(ranger_sample).as_dict()
    return ekf, z, ranges


def _publish_frames(server: DashboardServer, state: SlamState) -> None:
    payload = state.map_payload()
    if payload is not None:
        server.publish(Frame("map", payload))
    server.publish(Frame("cloud", state.cloud.to_payload()))


def run_replay(args, server, state: SlamState, stop) -> None:
    with open(args.replay, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for i, row in enumerate(reader):
            if stop.is_set():
                break
            ekf = (float(row["ekf_x"]), float(row["ekf_y"]), float(row["ekf_yaw"]))
            z = float(row.get("z", 0.0) or 0.0)
            ranges = {beam: (float(row[beam]) if row.get(beam) not in (None, "") else None)
                      for beam in ("front", "back", "left", "right", "up", "down")}
            state.step(ekf, z, ranges)
            if i % 5 == 0:
                _publish_frames(server, state)
                time.sleep(1.0 / args.map_hz)
    _publish_frames(server, state)
    print(f"Replay complete: {len(state.trail)} scans integrated.")


def _go_to(flight: VelocityFlight, hub: TelemetryHub, target: Tuple[float, float],
           stop, *, tol: float = 0.15, max_speed: float = 0.2, timeout: float = 15.0) -> None:
    """Simple world-frame P-controller flight to a horizontal waypoint."""
    t0 = time.monotonic()
    while not stop.is_set() and (time.monotonic() - t0) < timeout:
        latest = hub.latest("state")
        if latest is None:
            time.sleep(0.05)
            continue
        px, py = latest.get("stateEstimate.x", 0.0), latest.get("stateEstimate.y", 0.0)
        ex, ey = target[0] - px, target[1] - py
        if math.hypot(ex, ey) < tol:
            break
        scale = min(1.0, max_speed / (math.hypot(ex, ey) + 1e-6))
        flight.send_velocity(ex * scale, ey * scale, 0.0)
        time.sleep(0.05)
    flight.hover()


def _yaw_sweep(flight: VelocityFlight, stop, *, rate_deg_s: float = 72.0) -> None:
    """Rotate roughly one full turn in place to densify the sparse scan."""
    duration = 360.0 / rate_deg_s
    t0 = time.monotonic()
    while not stop.is_set() and (time.monotonic() - t0) < duration:
        flight.send_velocity(0.0, 0.0, 0.0, yaw_rate=rate_deg_s)
        time.sleep(0.05)
    flight.hover()


def run_live(args, server, state: SlamState, stop, recorder) -> None:
    fly = args.mode == "explore"
    with connect(args.uri, arm=fly, reset_estimator_on_connect=fly) as link:
        if fly:
            link.require_decks("flow2", "multiranger")
        elif not link.decks().get("multiranger", False):
            print("WARNING: Multi-ranger deck not detected; map will stay empty.")

        hub = TelemetryHub(link.scf)
        hub.add_config(make_state_config(args.rate_ms))
        hub.add_config(make_ranger_config(args.rate_ms))
        hub.add_config(make_battery_config())

        def on_sample(block: str, ts: int, sample) -> None:
            if block == "battery":
                publish_battery(server, sample)
                return
            if block != "ranger":
                return
            latest_state = hub.latest("state")
            if latest_state is None:
                return
            ekf, z, ranges = _sample_to_inputs(latest_state, sample)
            corrected = state.step(ekf, z, ranges)
            if recorder is not None:
                recorder.write({
                    "ekf_x": ekf[0], "ekf_y": ekf[1], "ekf_yaw": ekf[2], "z": z,
                    "x": corrected[0], "y": corrected[1], "yaw": corrected[2],
                    **{beam: ranges.get(beam) for beam in
                       ("front", "back", "left", "right", "up", "down")},
                })

        hub.subscribe(on_sample)

        publisher = MapPublisher(server.publish,
                                 lambda: _map_frame(state), hz=args.map_hz)
        with hub, publisher:
            if fly:
                _explore_loop(args, hub, state, stop, link)
            else:
                print("Hand-carry the platform to map the space. Ctrl+C to stop.")
                while not stop.is_set():
                    server.publish(Frame("cloud", state.cloud.to_payload()))
                    time.sleep(0.5)


def _map_frame(state: SlamState) -> Optional[Frame]:
    payload = state.map_payload()
    return Frame("map", payload) if payload is not None else None


def _explore_loop(args, hub, state: SlamState, stop, link) -> None:
    explorer = Explorer(state.grid, ExploreConfig())
    print("Autonomous exploration. Ctrl+C to land.")
    with VelocityFlight(link.scf, default_height=args.height) as flight:
        # Initial in-place sweep so the first frontier search has structure.
        _yaw_sweep(flight, stop)
        while not stop.is_set():
            latest = hub.latest("state")
            pose = (latest.get("stateEstimate.x", 0.0),
                    latest.get("stateEstimate.y", 0.0),
                    yaw_radians(latest)) if latest else (0.0, 0.0, 0.0)
            waypoints = explorer.next_goal(pose)
            if not waypoints:
                print("No frontiers left; exploration complete.")
                break
            for wp in waypoints:
                if stop.is_set():
                    break
                _go_to(flight, hub, wp, stop)
            _yaw_sweep(flight, stop)


def main() -> int:
    parser = argparse.ArgumentParser(description="drl: SLAM + exploration")
    parser.add_argument("--mode", choices=["explore", "no-fly", "replay"], default="no-fly")
    parser.add_argument("--uri", default=None, help="Crazyflie radio URI override")
    parser.add_argument("--port", type=int, default=8000, help="dashboard port")
    parser.add_argument("--height", type=float, default=0.4, help="explore: hover height (m)")
    parser.add_argument("--size", type=float, default=8.0, help="map side length (m)")
    parser.add_argument("--res", type=float, default=0.05, help="map resolution (m/cell)")
    parser.add_argument("--rate-ms", type=int, default=100, help="telemetry log period")
    parser.add_argument("--map-hz", type=float, default=5.0, help="map broadcast rate")
    parser.add_argument("--replay", default=None, help="replay mode: path to a recorded CSV")
    parser.add_argument("--save-map", default=None, help="path to save final grid as .npz")
    parser.add_argument("--save-cloud", default=None, help="path to save point cloud as .ply")
    parser.add_argument("--no-record", action="store_true")
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()

    grid = OccupancyGrid(MapConfig(size_m=args.size, resolution_m=args.res))
    cloud = PointCloud()
    state = SlamState(grid, cloud, MatchConfig())

    stop = install_stop_handler()
    server = DashboardServer(ServerConfig(port=args.port)).start(open_browser=not args.no_browser)
    server.publish(Frame("meta", {"experiment": f"SLAM ({args.mode})"}))
    recorder = None if (args.no_record or args.mode == "replay") \
        else CsvRecorder("slam", fieldnames=RECORD_FIELDS)

    if args.mode == "replay":
        if not args.replay:
            parser.error("--mode replay requires --replay PATH")
        run_replay(args, server, state, stop)
    else:
        run_live(args, server, state, stop, recorder)

    if recorder is not None:
        recorder.close()
    if args.save_map:
        grid.save_npz(args.save_map)
    if args.save_cloud:
        cloud.save_ply(args.save_cloud)
    server.stop()
    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
