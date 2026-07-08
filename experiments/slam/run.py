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

Status: scaffold. The SLAM loop, scan matcher, explorer, and point cloud are not
implemented yet.

Run (from the repo root, after ``pip install -e .``)::

    python -m experiments.slam.run --mode no-fly
    python -m experiments.slam.run --mode explore
    python -m experiments.slam.run --mode replay --replay data/slam_xxx.csv
"""
from __future__ import annotations

import argparse

from experiments.common import install_stop_handler
from experiments.slam.explorer import ExploreConfig, Explorer
from experiments.slam.mapper import MapConfig, OccupancyGrid
from experiments.slam.pointcloud import PointCloud
from experiments.slam.scan_match import MatchConfig
from drl.config import ServerConfig
from drl.connection import connect
from drl.dashboard import DashboardServer, Frame
from drl.recording import CsvRecorder


def main() -> int:
    parser = argparse.ArgumentParser(description="drl: SLAM + exploration")
    parser.add_argument("--mode", choices=["explore", "no-fly", "replay"], default="no-fly")
    parser.add_argument("--uri", default=None, help="Crazyflie radio URI override")
    parser.add_argument("--port", type=int, default=8000, help="dashboard port")
    parser.add_argument("--height", type=float, default=0.4, help="explore: hover height (m)")
    parser.add_argument("--size", type=float, default=8.0, help="map side length (m)")
    parser.add_argument("--res", type=float, default=0.05, help="map resolution (m/cell)")
    parser.add_argument("--map-hz", type=float, default=5.0, help="map broadcast rate")
    parser.add_argument("--replay", default=None, help="replay mode: path to a recorded CSV")
    parser.add_argument("--save-map", default=None, help="path to save final grid as .npz")
    parser.add_argument("--save-cloud", default=None, help="path to save point cloud as .ply")
    parser.add_argument("--no-record", action="store_true")
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()

    grid = OccupancyGrid(MapConfig(size_m=args.size, resolution_m=args.res))
    cloud = PointCloud()
    match_cfg = MatchConfig()
    explorer = Explorer(grid, ExploreConfig())

    stop = install_stop_handler()
    server = DashboardServer(ServerConfig(port=args.port)).start(open_browser=not args.no_browser)
    server.publish(Frame("meta", {"experiment": f"SLAM ({args.mode})"}))
    recorder = None if (args.no_record or args.mode == "replay") else CsvRecorder("slam")

    if args.mode == "replay":
        if not args.replay:
            parser.error("--mode replay requires --replay PATH")
        # TODO(slam): read the CSV; for each row run the SLAM step
        #   (predict from odometry delta -> scan_match -> grid.integrate +
        #   cloud.add_scan) and publish map/cloud/traj frames. No hardware.
        raise NotImplementedError

    fly = args.mode == "explore"
    with connect(args.uri, arm=fly, reset_estimator_on_connect=fly) as link:
        if fly:
            link.require_decks("flow2", "multiranger")

        # TODO(slam): SLAM + exploration loop.
        #   - subscribe to state + ranger telemetry
        #   - each scan: predicted = last_corrected + (ekf_now - ekf_prev);
        #     corrected = match_scan(grid, predicted, ranges, match_cfg);
        #     grid.integrate(*corrected, ranges); cloud.add_scan(...)
        #   - publish "map" (grid.to_payload(pose)), a trajectory trail, and a
        #     "cloud" frame; publish both raw-EKF and corrected pose to show drift
        #   - explore: follow explorer.next_goal(pose) waypoints with yaw sweeps
        #   - no-fly: just integrate while the platform is carried by hand
        raise NotImplementedError

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
