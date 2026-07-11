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

import heapq
import math
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from experiments.slam.mapper import OccupancyGrid

Cell = Tuple[int, int]        # (gx, gy)
WorldPoint = Tuple[float, float]

# 8-connected neighborhood offsets.
_NEIGHBORS8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
_NEIGHBORS4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]


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


def _classify(grid: OccupancyGrid, config: ExploreConfig):
    """Return boolean masks ``(free, occupied, unknown)`` over the grid."""
    prob = grid.probability()
    observed = grid._observed
    free = observed & (prob < config.free_threshold)
    occupied = observed & (prob > config.occ_threshold)
    unknown = ~observed
    return free, occupied, unknown


def find_frontiers(grid: OccupancyGrid, config: ExploreConfig) -> List[List[Cell]]:
    """Return clusters of frontier cells (free cells adjacent to unknown)."""
    free, _occupied, unknown = _classify(grid, config)
    h, w = free.shape

    frontier = np.zeros_like(free)
    ys, xs = np.nonzero(free)
    for gy, gx in zip(ys, xs):
        for dx, dy in _NEIGHBORS4:
            nx, ny = gx + dx, gy + dy
            if 0 <= nx < w and 0 <= ny < h and unknown[ny, nx]:
                frontier[gy, gx] = True
                break

    # Cluster adjacent frontier cells (8-connected) with a flood fill.
    clusters: List[List[Cell]] = []
    visited = np.zeros_like(frontier)
    fys, fxs = np.nonzero(frontier)
    for sy, sx in zip(fys, fxs):
        if visited[sy, sx]:
            continue
        cluster: List[Cell] = []
        queue = deque([(sx, sy)])
        visited[sy, sx] = True
        while queue:
            cx, cy = queue.popleft()
            cluster.append((cx, cy))
            for dx, dy in _NEIGHBORS8:
                nx, ny = cx + dx, cy + dy
                if (0 <= nx < w and 0 <= ny < h
                        and frontier[ny, nx] and not visited[ny, nx]):
                    visited[ny, nx] = True
                    queue.append((nx, ny))
        if len(cluster) >= config.min_frontier_cells:
            clusters.append(cluster)
    return clusters


def _blocked_mask(grid: OccupancyGrid, config: ExploreConfig) -> np.ndarray:
    """Cells that A* must avoid: occupied cells inflated by ``inflation_m``."""
    free, occupied, unknown = _classify(grid, config)
    blocked = occupied | unknown
    inflate = int(round(config.inflation_m / grid.cfg.resolution_m))
    if inflate <= 0:
        return blocked
    h, w = occupied.shape
    inflated = blocked.copy()
    oys, oxs = np.nonzero(occupied)
    for gy, gx in zip(oys, oxs):
        x0, x1 = max(0, gx - inflate), min(w, gx + inflate + 1)
        y0, y1 = max(0, gy - inflate), min(h, gy + inflate + 1)
        inflated[y0:y1, x0:x1] = True
    return inflated


def plan_path(grid: OccupancyGrid, start: Cell, goal: Cell,
              config: ExploreConfig) -> Optional[List[Cell]]:
    """A* path over known-free cells (obstacles inflated), or None if unreachable."""
    blocked = _blocked_mask(grid, config)
    h, w = blocked.shape
    sx, sy = start
    gx, gy = goal
    if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h):
        return None
    # The goal is a frontier (free) cell, so it must not be treated as blocked.
    if blocked[sy, sx] or blocked[gy, gx]:
        return None

    def heuristic(x, y):
        return math.hypot(x - gx, y - gy)

    open_heap = [(heuristic(sx, sy), 0.0, (sx, sy))]
    came_from: dict = {}
    g_score = {(sx, sy): 0.0}
    while open_heap:
        _f, g, (cx, cy) = heapq.heappop(open_heap)
        if (cx, cy) == (gx, gy):
            path = [(cx, cy)]
            while (cx, cy) in came_from:
                cx, cy = came_from[(cx, cy)]
                path.append((cx, cy))
            path.reverse()
            return path
        for dx, dy in _NEIGHBORS8:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < w and 0 <= ny < h) or blocked[ny, nx]:
                continue
            step = math.hypot(dx, dy)
            tentative = g + step
            if tentative < g_score.get((nx, ny), math.inf):
                came_from[(nx, ny)] = (cx, cy)
                g_score[(nx, ny)] = tentative
                heapq.heappush(open_heap, (tentative + heuristic(nx, ny), tentative, (nx, ny)))
    return None


class Explorer:
    """Stateful planner: pick the next frontier goal and hand back waypoints."""

    def __init__(self, grid: OccupancyGrid, config: Optional[ExploreConfig] = None):
        self.grid = grid
        self.config = config or ExploreConfig()

    def _cell_to_world(self, cell: Cell) -> WorldPoint:
        gx, gy = cell
        res = self.grid.cfg.resolution_m
        return (self.grid.origin_x + (gx + 0.5) * res,
                self.grid.origin_y + (gy + 0.5) * res)

    def next_goal(self, pose: Tuple[float, float, float]) -> Optional[List[WorldPoint]]:
        """Choose a frontier and return world-space waypoints to it, or None if done."""
        clusters = find_frontiers(self.grid, self.config)
        if not clusters:
            return None

        start = self.grid._world_to_cell(pose[0], pose[1])

        def cluster_key(cluster: List[Cell]):
            cxs = sum(c[0] for c in cluster) / len(cluster)
            cys = sum(c[1] for c in cluster) / len(cluster)
            dist = math.hypot(cxs - start[0], cys - start[1])
            # Prefer large clusters, then nearer ones.
            return (-len(cluster), dist)

        # Try clusters best-first; return waypoints for the first reachable goal.
        for cluster in sorted(clusters, key=cluster_key):
            goal = min(cluster, key=lambda c: math.hypot(c[0] - start[0], c[1] - start[1]))
            path = plan_path(self.grid, start, goal, self.config)
            if path:
                # Decimate the dense cell path into sparser world waypoints.
                stride = max(1, len(path) // 12)
                waypoints = [self._cell_to_world(path[i]) for i in range(0, len(path), stride)]
                if self._cell_to_world(path[-1]) != waypoints[-1]:
                    waypoints.append(self._cell_to_world(path[-1]))
                return waypoints
        return None
