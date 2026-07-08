"""Accumulate range hits into a 3-D point cloud.

While the occupancy grid is a top-down 2-D summary, the point cloud keeps every
beam hit at its measured height ``(x, y, z)`` - horizontal beams at the
platform's current altitude, and the up beam at ceiling height. Accumulated over
a flight, these points form a sparse 3-D shell of the room that can be rendered
interactively or exported for an external viewer.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Tuple

Point3 = Tuple[float, float, float]


class PointCloud:
    """A growing list of 3-D points with export helpers."""

    def __init__(self) -> None:
        self._points: List[Point3] = []

    def add_scan(self, x: float, y: float, z: float, yaw_rad: float,
                 ranges: Dict[str, Optional[float]]) -> None:
        """Project this scan's beam hits to world 3-D points and store them."""
        # TODO(slam): for each horizontal beam hit, add a point at the
        # platform's z; for the up beam, add a point at z + up_range.
        raise NotImplementedError

    def to_payload(self) -> dict:
        """Serialize (a decimated view of) the cloud for the dashboard."""
        # TODO(slam): return {"points": [[x, y, z], ...]} (downsampled).
        raise NotImplementedError

    def save_ply(self, path: str) -> None:
        """Write the cloud as an ASCII .ply file openable in any 3-D viewer."""
        # TODO(slam): write a minimal ASCII PLY header + one line per point.
        raise NotImplementedError
