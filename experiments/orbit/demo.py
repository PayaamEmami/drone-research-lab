"""Orbit preview for ``--demo`` (no hardware).

A cylinder stands in front of a simulated drone. Beams are room walls with the
cylinder ray-cast over them, and the real :class:`~experiments.orbit.control.OrbitTracker`
flies the point mass. Passed to :class:`~drl.session.ExperimentSession` as
``demo_simulator``.
"""
from __future__ import annotations

import math

from drl.dashboard import DashboardServer, Frame, simulate_ranges
from drl.dashboard.demo import battery_payload
from experiments.orbit.control import OrbitParams, OrbitTracker

_BEARINGS = {
    "front": 0.0,
    "left": math.pi / 2,
    "back": math.pi,
    "right": -math.pi / 2,
}

# Surface sits about 0.63 m in front of a drone that starts at the origin,
# inside the default acquisition window and outside the orbit radius.
CYLINDER_CENTER = (0.85, 0.0)
CYLINDER_RADIUS = 0.22
ROOM = (-3.0, 3.0, -3.0, 3.0)


def _ray_circle(
    x: float,
    y: float,
    angle: float,
    center: tuple[float, float],
    radius: float,
) -> float | None:
    """Distance along a ray to a circle, or None. Inside the circle reads as a hit."""
    fx, fy = x - center[0], y - center[1]
    if fx * fx + fy * fy <= radius * radius:
        return 0.02
    dx, dy = math.cos(angle), math.sin(angle)
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - radius * radius
    disc = b * b - 4.0 * c
    if disc < 0.0:
        return None
    root = math.sqrt(disc)
    hits = [t for t in ((-b - root) / 2.0, (-b + root) / 2.0) if t > 0.02]
    return min(hits) if hits else None


def cylinder_ranges(
    x: float,
    y: float,
    yaw: float,
    z: float,
    center: tuple[float, float] = CYLINDER_CENTER,
    radius: float = CYLINDER_RADIUS,
) -> dict:
    """Wall ranges with a vertical cylinder overlaid on any beam that hits it."""
    ranges = simulate_ranges(x, y, yaw, room=ROOM, max_range=4.0)
    for name, bearing in _BEARINGS.items():
        hit = _ray_circle(x, y, yaw + bearing, center, radius)
        if hit is None:
            continue
        wall = ranges.get(name)
        ranges[name] = hit if wall is None else min(hit, wall)
    ranges["up"] = max(0.05, 2.0 - z)
    ranges["down"] = max(0.02, z)
    return ranges


def simulate(server: DashboardServer, stop, rate_hz: float) -> None:
    period = 1.0 / rate_hz
    params = OrbitParams(object_radius=CYLINDER_RADIUS)
    tracker = OrbitTracker()
    x, y, yaw = 0.0, 0.0, 0.0
    z = 0.15
    t = 0.0

    while not stop.is_set():
        ranges = cylinder_ranges(x, y, yaw, z)
        cmd = tracker.step(ranges, yaw, z, period, params)
        x += cmd.vx * period
        y += cmd.vy * period
        z += cmd.vz * period
        yaw += math.radians(cmd.yaw_rate) * period
        t += period

        server.publish(Frame("state", {
            "x": x, "y": y, "z": z,
            "roll": 1.5 * math.sin(t * 0.8),
            "pitch": 1.0 * math.cos(t * 0.7),
            "yaw": math.degrees(yaw),
        }))
        server.publish(Frame("ranger", ranges))
        server.publish(Frame("cmd", {
            "vx": cmd.body_vx, "vy": cmd.body_vy, "label": cmd.label,
        }))
        server.publish(Frame("battery", battery_payload(t)))
        if stop.wait(period):
            break
