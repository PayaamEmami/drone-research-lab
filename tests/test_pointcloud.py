"""Tests for ``experiments.slam.pointcloud``.

Offline checks for scan projection, payload downsampling, and PLY export.
No Crazyflie or radio link.
"""
from __future__ import annotations

import math

from experiments.slam.pointcloud import PointCloud


# ---------------------------------------------------------------------------
# Scan projection
# ---------------------------------------------------------------------------


def test_add_scan_projects_front_beam():
    cloud = PointCloud()
    cloud.add_scan(0.0, 0.0, 0.5, 0.0, {"front": 1.0, "back": None,
                                        "left": None, "right": None,
                                        "up": None, "down": None})
    assert len(cloud) == 1
    x, y, z = cloud._points[0]
    assert math.isclose(x, 1.0, abs_tol=1e-6)
    assert math.isclose(y, 0.0, abs_tol=1e-6)
    assert math.isclose(z, 0.5, abs_tol=1e-6)


def test_up_beam_adds_ceiling_point():
    cloud = PointCloud()
    cloud.add_scan(0.0, 0.0, 0.4, 0.0, {"front": None, "back": None,
                                        "left": None, "right": None,
                                        "up": 1.5, "down": None})
    assert len(cloud) == 1
    _x, _y, z = cloud._points[0]
    assert math.isclose(z, 0.4 + 1.5, abs_tol=1e-6)


# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------


def test_to_payload_downsamples():
    cloud = PointCloud()
    for i in range(5000):
        cloud.add_scan(float(i), 0.0, 0.0, 0.0, {"front": 1.0, "back": None,
                                                 "left": None, "right": None,
                                                 "up": None, "down": None})
    payload = cloud.to_payload(max_points=1000)
    assert "points" in payload
    assert len(payload["points"]) <= 1000
    assert all(len(p) == 3 for p in payload["points"])


def test_save_ply_writes_valid_header(tmp_path):
    cloud = PointCloud()
    cloud.add_scan(0.0, 0.0, 0.0, 0.0, {"front": 1.0, "back": 1.0,
                                        "left": None, "right": None,
                                        "up": None, "down": None})
    out = tmp_path / "cloud.ply"
    cloud.save_ply(str(out))
    text = out.read_text(encoding="ascii").splitlines()
    assert text[0] == "ply"
    assert "element vertex 2" in text
    assert text[-1].count(" ") == 2
