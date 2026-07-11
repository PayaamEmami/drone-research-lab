"""Tests for ``experiments.slam.run``.

Offline checks for the shared SLAM state machine and CSV replay path.
Uses :func:`tests.conftest.simulate_ranges` in a fixed room.
"""
from __future__ import annotations

import csv
from threading import Event
from types import SimpleNamespace

from experiments.slam.mapper import MapConfig, OccupancyGrid
from experiments.slam.pointcloud import PointCloud
from experiments.slam.run import RECORD_FIELDS, SlamState, run_replay
from experiments.slam.scan_match import MatchConfig
from tests.conftest import simulate_ranges

ROOM = (-2.0, 2.0, -2.0, 2.0)


class _FakeServer:
    """Capture published frames instead of opening a websocket."""

    def __init__(self):
        self.frames = []

    def publish(self, frame):
        self.frames.append(frame)


def _new_state():
    """Fresh SLAM state backed by an empty grid and point cloud."""
    grid = OccupancyGrid(MapConfig(size_m=8.0, resolution_m=0.1))
    return SlamState(grid, PointCloud(), MatchConfig())


# ---------------------------------------------------------------------------
# SlamState
# ---------------------------------------------------------------------------


def test_slam_step_builds_map_and_trail():
    state = _new_state()
    for _ in range(10):
        ranges = simulate_ranges(0.0, 0.0, 0.0, room=ROOM)
        state.step((0.0, 0.0, 0.0), 0.4, ranges)
    assert len(state.trail) == 10
    assert len(state.cloud) > 0
    payload = state.map_payload()
    assert payload is not None
    assert "trail" in payload and "pose_raw" in payload


def test_slam_map_payload_none_before_first_step():
    assert _new_state().map_payload() is None


# ---------------------------------------------------------------------------
# Replay
# ---------------------------------------------------------------------------


def test_run_replay_reads_csv(tmp_path):
    path = tmp_path / "slam_test.csv"
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=RECORD_FIELDS)
        writer.writeheader()
        for i in range(8):
            ekf = (0.01 * i, 0.0, 0.0)
            ranges = simulate_ranges(ekf[0], ekf[1], ekf[2], room=ROOM)
            writer.writerow({
                "elapsed_s": 0.1 * i,
                "ekf_x": ekf[0], "ekf_y": ekf[1], "ekf_yaw": ekf[2], "z": 0.4,
                "x": ekf[0], "y": ekf[1], "yaw": ekf[2],
                **{beam: ranges.get(beam) for beam in
                   ("front", "back", "left", "right", "up", "down")},
            })

    state = _new_state()
    server = _FakeServer()
    args = SimpleNamespace(replay=str(path), map_hz=1000.0)
    run_replay(args, server, state, Event())

    assert len(state.trail) == 8
    assert any(f.type == "map" for f in server.frames)
    assert any(f.type == "cloud" for f in server.frames)
