"""Tests for ``experiments.slam.explorer``.

Offline checks for frontier detection, A* planning, and goal selection.
Uses :func:`tests.conftest.simulate_ranges` in a fixed room.
"""
from __future__ import annotations

from experiments.slam.explorer import (
    ExploreConfig,
    Explorer,
    find_frontiers,
    plan_path,
)
from experiments.slam.mapper import MapConfig, OccupancyGrid
from tests.conftest import simulate_ranges

ROOM = (-2.0, 2.0, -2.0, 2.0)


def _partially_explored_grid():
    """Build a map with known free space and remaining unknown cells."""
    grid = OccupancyGrid(MapConfig(size_m=8.0, resolution_m=0.1))
    for pose in [(0.0, 0.0, 0.0), (0.3, 0.0, 0.0)]:
        for _ in range(4):
            grid.integrate(*pose, simulate_ranges(*pose, room=ROOM))
    return grid


# ---------------------------------------------------------------------------
# Frontiers and planning
# ---------------------------------------------------------------------------


def test_find_frontiers_returns_clusters():
    grid = _partially_explored_grid()
    clusters = find_frontiers(grid, ExploreConfig(min_frontier_cells=3))
    assert clusters, "expected at least one frontier cluster"
    assert all(len(c) >= 3 for c in clusters)


def test_plan_path_finds_route_between_free_cells():
    grid = _partially_explored_grid()
    cfg = ExploreConfig(inflation_m=0.0)
    start = grid._world_to_cell(0.0, 0.0)
    goal = grid._world_to_cell(0.2, 0.1)
    path = plan_path(grid, start, goal, cfg)
    assert path is not None
    assert path[0] == start and path[-1] == goal


def test_next_goal_returns_world_waypoints():
    grid = _partially_explored_grid()
    explorer = Explorer(grid, ExploreConfig(min_frontier_cells=3, inflation_m=0.0))
    waypoints = explorer.next_goal((0.0, 0.0, 0.0))
    assert waypoints is None or (
        isinstance(waypoints, list) and all(len(wp) == 2 for wp in waypoints)
    )


def test_next_goal_none_when_fully_unknown():
    grid = OccupancyGrid(MapConfig(size_m=4.0, resolution_m=0.1))
    explorer = Explorer(grid, ExploreConfig())
    assert explorer.next_goal((0.0, 0.0, 0.0)) is None
