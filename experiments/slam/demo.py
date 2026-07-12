"""High-fidelity SLAM preview for ``--demo`` (no hardware).

Drives the real occupancy grid, point cloud, and scan-matching pipeline with a
simulated pose looping around a room, so the previewed map and cloud build up
just like a live run. Passed to :class:`~drl.session.ExperimentSession` as its
``demo_simulator``.
"""
from __future__ import annotations

import math
import time
from typing import Tuple

from drl.dashboard import DashboardServer, Frame, simulate_ranges
from drl.dashboard.demo import battery_payload
from experiments.slam.mapper import MapConfig, OccupancyGrid
from experiments.slam.pointcloud import PointCloud
from experiments.slam.run import SlamState
from experiments.slam.scan_match import MatchConfig

Pose = Tuple[float, float, float]


def simulate(server: DashboardServer, stop, rate_hz: float) -> None:
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
        ranges = simulate_ranges(ekf_noisy[0], ekf_noisy[1], ekf_noisy[2])
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
        server.publish(Frame("battery", battery_payload(t)))
        step += 1
        if stop.wait(period):
            break
