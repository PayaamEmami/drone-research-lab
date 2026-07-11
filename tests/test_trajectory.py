"""Tests for ``experiments.trajectory_tracking.trajectory``.

Offline checks for the expanding spiral reference generator: start pose,
monotonic growth, caps, and angular advance. No Crazyflie or radio link.
"""
from __future__ import annotations

import math

from experiments.trajectory_tracking.trajectory import SpiralParams, spiral


# ---------------------------------------------------------------------------
# Spiral geometry
# ---------------------------------------------------------------------------


def test_spiral_starts_at_base_radius_and_height():
    p = SpiralParams()
    x, y, z = spiral(0.0, p)
    assert math.isclose(math.hypot(x - p.center[0], y - p.center[1]), p.base_radius, rel_tol=1e-9)
    assert math.isclose(z, p.base_height, rel_tol=1e-9)


def test_spiral_radius_and_height_are_capped():
    p = SpiralParams()
    x, y, z = spiral(10_000.0, p)
    radius = math.hypot(x - p.center[0], y - p.center[1])
    assert radius <= p.max_radius + 1e-9
    assert math.isclose(radius, p.max_radius, rel_tol=1e-6)
    assert math.isclose(z, p.max_height, rel_tol=1e-9)


def test_spiral_radius_and_height_grow_monotonically():
    p = SpiralParams()
    prev_r = prev_z = -math.inf
    for t in [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]:
        x, y, z = spiral(t, p)
        r = math.hypot(x - p.center[0], y - p.center[1])
        assert r >= prev_r - 1e-9
        assert z >= prev_z - 1e-9
        prev_r, prev_z = r, z


def test_spiral_angle_advances():
    p = SpiralParams(radius_growth=0.0)
    quarter = (math.pi / 2) / p.angular_rate
    x0, y0, _ = spiral(0.0, p)
    x1, y1, _ = spiral(quarter, p)
    theta0 = math.atan2(y0 - p.center[1], x0 - p.center[0])
    theta1 = math.atan2(y1 - p.center[1], x1 - p.center[0])
    assert math.isclose((theta1 - theta0) % (2 * math.pi), math.pi / 2, abs_tol=1e-6)
