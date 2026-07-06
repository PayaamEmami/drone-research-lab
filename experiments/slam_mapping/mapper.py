"""A 2D log-odds occupancy grid built from Multi-ranger scans.

The grid is the mapping substrate for the SLAM experiment. For each pose and
each horizontal beam with a valid return, it ray-casts from the platform to the
hit point: cells along the ray are evidence of *free* space (log-odds
decremented) and the hit cell is evidence of an *obstacle* (log-odds
incremented). Beams with no return cast a free ray out to a clamp distance.

The grid is world-aligned and centered on the starting position. In addition to
integrating scans, it can *score* how well a hypothetical pose's scan agrees
with the map built so far (:meth:`score_scan`); the scan matcher uses that score
to correct pose drift.
"""
from __future__ import annotations

import base64
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

# Body-frame bearing (radians) of each horizontal beam: X forward, +yaw CCW.
_BEAM_BEARINGS = {
    "front": 0.0,
    "left": math.pi / 2,
    "back": math.pi,
    "right": -math.pi / 2,
}


def _safe_float(value: object, default: float = 0.0) -> float:
    """Return a JSON-safe float (no NaN/Inf) for websocket payloads."""
    try:
        number = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


@dataclass
class MapConfig:
    size_m: float = 8.0       # square map side length in meters
    resolution_m: float = 0.05  # meters per cell
    max_range_m: float = 3.5  # clamp beams to this for free-space carving
    l_occ: float = 0.85       # log-odds added for a hit
    l_free: float = 0.4       # log-odds subtracted along a free ray
    l_clamp: float = 6.0      # saturation limit on |log-odds|


class OccupancyGrid:
    def __init__(self, config: Optional[MapConfig] = None):
        self.cfg = config or MapConfig()
        self.width = int(round(self.cfg.size_m / self.cfg.resolution_m))
        self.height = self.width
        # World coordinate of grid cell (0, 0); map is centered on the origin.
        self.origin_x = -self.cfg.size_m / 2.0
        self.origin_y = -self.cfg.size_m / 2.0
        self._logodds = np.zeros((self.height, self.width), dtype=np.float32)
        self._observed = np.zeros((self.height, self.width), dtype=bool)
        self._last_points: List[Tuple[float, float]] = []

    # ------------------------------------------------------------------ helpers
    def _world_to_cell(self, wx: float, wy: float) -> Tuple[int, int]:
        gx = int((wx - self.origin_x) / self.cfg.resolution_m)
        gy = int((wy - self.origin_y) / self.cfg.resolution_m)
        return gx, gy

    def _in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    @staticmethod
    def _bresenham(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        """Integer line cells from (x0,y0) to (x1,y1), inclusive of both ends."""
        cells: List[Tuple[int, int]] = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy
        return cells

    def _apply(self, gx: int, gy: int, delta: float) -> None:
        if self._in_bounds(gx, gy):
            v = self._logodds[gy, gx] + delta
            self._logodds[gy, gx] = max(-self.cfg.l_clamp, min(self.cfg.l_clamp, v))
            self._observed[gy, gx] = True

    # -------------------------------------------------------------------- update
    def integrate(self, x: float, y: float, yaw_rad: float, ranges: Dict[str, Optional[float]]) -> None:
        """Fuse one scan taken at world pose (x, y, yaw) into the grid."""
        ox, oy = self._world_to_cell(x, y)
        points: List[Tuple[float, float]] = []

        for beam, bearing in _BEAM_BEARINGS.items():
            d = ranges.get(beam)
            hit = d is not None
            r = d if hit else self.cfg.max_range_m
            r = min(r, self.cfg.max_range_m)

            angle = yaw_rad + bearing
            ex = x + r * math.cos(angle)
            ey = y + r * math.sin(angle)
            egx, egy = self._world_to_cell(ex, ey)

            ray = self._bresenham(ox, oy, egx, egy)
            # All but the last cell are free space.
            for cx, cy in ray[:-1]:
                self._apply(cx, cy, -self.cfg.l_free)
            # The endpoint is an obstacle only if this beam actually hit something.
            if hit and r < self.cfg.max_range_m + 1e-6:
                self._apply(egx, egy, self.cfg.l_occ)
                points.append((ex, ey))

        self._last_points = points

    # -------------------------------------------------------------- scan matching
    def score_scan(self, x: float, y: float, yaw_rad: float,
                   ranges: Dict[str, Optional[float]]) -> float:
        """Score how well a scan taken at pose (x, y, yaw) matches the map.

        Read-only: computes each beam's hit endpoint and sums the current
        log-odds at those cells, rewarding poses whose beams land on cells the
        map already believes are occupied. The scan matcher searches poses to
        maximize this score.
        """
        # TODO(slam_mapping): sum self._logodds at each observed hit-endpoint cell
        # (reuse the beam-bearing math from integrate()).
        raise NotImplementedError

    # ------------------------------------------------------------------- export
    def probability(self) -> np.ndarray:
        """Occupancy probability in [0, 1] (0.5 where unobserved)."""
        return 1.0 - 1.0 / (1.0 + np.exp(self._logodds))

    def to_payload(self, pose: Optional[Tuple[float, float, float]] = None) -> dict:
        """Serialize to a dashboard 'map' payload.

        Cells are int8: -1 unknown, else occupancy percent (0..100), row-major.
        The grid is sent as base64-packed bytes to keep websocket frames small.
        """
        prob = self.probability()
        occ = np.full(self._logodds.shape, -1, dtype=np.int8)
        obs = self._observed
        occ[obs] = np.clip((prob[obs] * 100.0), 0, 100).astype(np.int8)

        payload = {
            "res": self.cfg.resolution_m,
            "width": self.width,
            "height": self.height,
            "origin": {"x": self.origin_x, "y": self.origin_y},
            "data_b64": base64.standard_b64encode(occ.tobytes()).decode("ascii"),
            "points": [
                [round(_safe_float(px), 3), round(_safe_float(py), 3)]
                for px, py in self._last_points
            ],
        }
        if pose is not None:
            payload["pose"] = {
                "x": _safe_float(pose[0]),
                "y": _safe_float(pose[1]),
                "yaw": _safe_float(pose[2]),
            }
        return payload

    def save_npz(self, path: str) -> None:
        """Persist the raw log-odds grid for offline analysis / replotting."""
        np.savez(
            path,
            logodds=self._logodds,
            observed=self._observed,
            resolution_m=self.cfg.resolution_m,
            origin=np.array([self.origin_x, self.origin_y]),
        )
