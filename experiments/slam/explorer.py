"""Frontier-based autonomous exploration.

Exploration drives the platform to reveal unknown space without a human picking
waypoints. The idea:

1. Find *frontiers* - free cells that border unknown cells - on the occupancy
   grid. Cluster adjacent frontier cells and rank the clusters (by size /
   proximity).
2. Plan a collision-free path from the current pose to the chosen frontier with
   A* over the known-free cells, using an inflated obstacle map for safety.
3. Follow the path (the runner adds periodic yaw sweeps to densify the sparse
   Multi-ranger scan), then repeat until no meaningful frontiers remain.

This module is pure grid logic (no hardware), so the frontier detection and the
planner can be unit-tested on synthetic maps.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

from experiments.slam.mapper import OccupancyGrid

Cell = Tuple[int, int]        # (gx, gy)
WorldPoint = Tuple[float, float]


@dataclass
class ExploreConfig:
    """Tunables for frontier selection and planning safety.

    :param free_threshold: P(occupied) below this counts as free.
    :param occ_threshold: P(occupied) above this counts as an obstacle.
    :param inflation_m: obstacle inflation radius for planning clearance (m).
    :param min_frontier_cells: ignore frontier clusters smaller than this.
    """

    free_threshold: float = 0.35
    occ_threshold: float = 0.65
    inflation_m: float = 0.2
    min_frontier_cells: int = 6


def find_frontiers(grid: OccupancyGrid, config: ExploreConfig) -> List[List[Cell]]:
    """Return clusters of frontier cells (free cells adjacent to unknown)."""
    # TODO(slam): classify cells free/occupied/unknown from probability();
    # collect free cells with an unknown 4-neighbor; cluster them; drop clusters
    # smaller than config.min_frontier_cells.
    raise NotImplementedError


def plan_path(grid: OccupancyGrid, start: Cell, goal: Cell,
              config: ExploreConfig) -> Optional[List[Cell]]:
    """A* path over known-free cells (obstacles inflated), or None if unreachable."""
    # TODO(slam): A* on the inflated free-space grid; 8-connected moves.
    raise NotImplementedError


class Explorer:
    """Stateful planner: pick the next frontier goal and hand back waypoints."""

    def __init__(self, grid: OccupancyGrid, config: Optional[ExploreConfig] = None):
        self.grid = grid
        self.config = config or ExploreConfig()

    def next_goal(self, pose: Tuple[float, float, float]) -> Optional[List[WorldPoint]]:
        """Choose a frontier and return world-space waypoints to it, or None if done."""
        # TODO(slam): find frontiers, pick the best cluster, plan a path
        # from the current cell, and convert the cell path to world waypoints.
        raise NotImplementedError
