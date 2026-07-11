"""Tests for ``experiments.slam.mapper`` and ``experiments.slam.scan_match``.

Offline checks for occupancy-grid integration, scan scoring, and pose
correction. Uses :func:`tests.conftest.simulate_ranges` in a fixed room.
"""
from __future__ import annotations

from experiments.slam.mapper import MapConfig, OccupancyGrid
from experiments.slam.scan_match import MatchConfig, match_scan
from tests.conftest import simulate_ranges

ROOM = (-2.0, 2.0, -2.0, 2.0)


def _build_map(pose=(0.0, 0.0, 0.0), passes=6):
    """Integrate repeated scans at ``pose`` until walls show in the grid."""
    grid = OccupancyGrid(MapConfig(size_m=8.0, resolution_m=0.05))
    for _ in range(passes):
        ranges = simulate_ranges(*pose, room=ROOM)
        grid.integrate(pose[0], pose[1], pose[2], ranges)
    return grid


# ---------------------------------------------------------------------------
# OccupancyGrid
# ---------------------------------------------------------------------------


def test_integrate_marks_free_and_occupied():
    grid = _build_map()
    prob = grid.probability()
    cx, cy = grid._world_to_cell(0.0, 0.0)
    assert prob[cy, cx] < 0.5
    wx, wy = grid._world_to_cell(2.0, 0.0)
    assert prob[wy, wx] > 0.5


def test_score_scan_peaks_at_true_pose():
    grid = _build_map(pose=(0.0, 0.0, 0.0))
    true_ranges = simulate_ranges(0.0, 0.0, 0.0, room=ROOM)
    score_true = grid.score_scan(0.0, 0.0, 0.0, true_ranges)
    score_off = grid.score_scan(0.3, 0.3, 0.0, true_ranges)
    assert score_true > score_off


def test_score_scan_no_beams_is_zero():
    grid = _build_map()
    empty = {"front": None, "back": None, "left": None, "right": None,
             "up": None, "down": None}
    assert grid.score_scan(0.0, 0.0, 0.0, empty) == 0.0


# ---------------------------------------------------------------------------
# Scan matching
# ---------------------------------------------------------------------------


def test_match_scan_recovers_drift():
    grid = _build_map(pose=(0.0, 0.0, 0.0))
    true_pose = (0.0, 0.0, 0.0)
    ranges = simulate_ranges(*true_pose, room=ROOM)
    drifted = (0.06, -0.04, 0.0)
    corrected = match_scan(grid, drifted, ranges, MatchConfig())
    err_before = abs(drifted[0]) + abs(drifted[1])
    err_after = abs(corrected[0]) + abs(corrected[1])
    assert err_after <= err_before


def test_match_scan_bails_without_enough_beams():
    grid = _build_map()
    sparse = {"front": 1.0, "back": None, "left": None, "right": None,
              "up": None, "down": None}
    predicted = (0.1, 0.1, 0.0)
    assert match_scan(grid, predicted, sparse, MatchConfig(min_beams=2)) == predicted
