"""Accumulate range hits into a 3-D point cloud.

While the occupancy grid is a top-down 2-D summary, the point cloud keeps every
beam hit at its measured height ``(x, y, z)`` - horizontal beams at the
platform's current altitude, and the up beam at ceiling height. Accumulated over
a flight, these points form a sparse 3-D shell of the room that can be rendered
interactively or exported for an external viewer.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

Point3 = Tuple[float, float, float]

# Body-frame bearing (radians) of each horizontal beam: X forward, +yaw CCW.
_BEAM_BEARINGS = {
    "front": 0.0,
    "left": math.pi / 2,
    "back": math.pi,
    "right": -math.pi / 2,
}

# Beyond this the beam is treated as no return (matches the mapper clamp).
_MAX_RANGE_M = 3.5


class PointCloud:
    """A growing list of 3-D points with export helpers."""

    def __init__(self) -> None:
        self._points: List[Point3] = []

    def __len__(self) -> int:
        return len(self._points)

    def add_scan(self, x: float, y: float, z: float, yaw_rad: float,
                 ranges: Dict[str, Optional[float]]) -> None:
        """Project this scan's beam hits to world 3-D points and store them."""
        for beam, bearing in _BEAM_BEARINGS.items():
            d = ranges.get(beam)
            if d is None or d >= _MAX_RANGE_M:
                continue
            angle = yaw_rad + bearing
            self._points.append((x + d * math.cos(angle), y + d * math.sin(angle), z))
        up = ranges.get("up")
        if up is not None and up < _MAX_RANGE_M:
            self._points.append((x, y, z + up))

    def to_payload(self, max_points: int = 2000) -> dict:
        """Serialize (a decimated view of) the cloud for the dashboard."""
        n = len(self._points)
        stride = max(1, n // max_points) if max_points > 0 else 1
        pts = [[round(px, 3), round(py, 3), round(pz, 3)]
               for (px, py, pz) in self._points[::stride]]
        return {"points": pts}

    def save_ply(self, path: str) -> None:
        """Write the cloud as an ASCII .ply file openable in any 3-D viewer."""
        with open(path, "w", encoding="ascii") as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(self._points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("end_header\n")
            for px, py, pz in self._points:
                f.write(f"{px:.4f} {py:.4f} {pz:.4f}\n")
