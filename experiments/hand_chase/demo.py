"""Hand-chase preview for ``--demo`` (no hardware).

A point-sized hand moves in front of a simulated drone. Range beams are the
room walls, with a cone hit overlaid when the hand lies in that beam, and the
real :class:`~experiments.hand_chase.control.HandChase` law flies the point
mass. Passed to :class:`~drl.session.ExperimentSession` as ``demo_simulator``.
"""
from __future__ import annotations

import math

from drl.dashboard import DashboardServer, Frame, simulate_ranges
from drl.dashboard.demo import battery_payload
from experiments.hand_chase.control import ChaseParams, HandChase

# Matches the bearings inside ``drl.dashboard.demo.simulate_ranges``.
_BEARINGS = {
    "front": 0.0,
    "left": math.pi / 2,
    "back": math.pi,
    "right": -math.pi / 2,
}

HAND_RADIUS = 0.08
BEAM_HALF_ANGLE = math.radians(15.0)
ROOM = (-2.5, 2.5, -2.5, 2.5)


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def hand_motion(t: float) -> tuple[float, float]:
    """World position of the preview hand: a slow sway in front of the origin."""
    return (0.50 + 0.10 * math.sin(t * 0.35), 0.22 * math.sin(t * 0.22))


def hand_ranges(
    x: float,
    y: float,
    yaw: float,
    z: float,
    hand: tuple[float, float],
    *,
    hand_radius: float = HAND_RADIUS,
    half_angle: float = BEAM_HALF_ANGLE,
) -> dict:
    """Wall ranges for ``(x, y, yaw)``, replaced when the hand sits in a beam."""
    ranges = simulate_ranges(x, y, yaw, room=ROOM, max_range=4.0)
    hx, hy = hand
    for name, bearing in _BEARINGS.items():
        dx, dy = hx - x, hy - y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            ranges[name] = hand_radius
            continue
        da = abs(_wrap(math.atan2(dy, dx) - (yaw + bearing)))
        if da <= half_angle + math.atan2(hand_radius, dist):
            hit = max(0.02, dist - hand_radius)
            wall = ranges.get(name)
            ranges[name] = hit if wall is None else min(hit, wall)
    ranges["up"] = max(0.05, 2.0 - z)
    ranges["down"] = max(0.02, z)
    return ranges


def simulate(server: DashboardServer, stop, rate_hz: float) -> None:
    period = 1.0 / rate_hz
    params = ChaseParams()
    chase = HandChase()
    x, y, yaw = 0.0, 0.0, 0.0
    z = 0.15
    t = 0.0

    while not stop.is_set():
        hand = hand_motion(t)
        ranges = hand_ranges(x, y, yaw, z, hand)
        cmd = chase.step(ranges, yaw, z, params)
        x += cmd.vx * period
        y += cmd.vy * period
        z += cmd.vz * period
        yaw += math.radians(cmd.yaw_rate) * period
        t += period

        server.publish(Frame("state", {
            "x": x, "y": y, "z": z,
            "roll": 1.5 * math.sin(t * 0.6),
            "pitch": -1.0 * math.cos(t * 0.5),
            "yaw": math.degrees(yaw),
        }))
        server.publish(Frame("ranger", ranges))
        server.publish(Frame("cmd", {
            "vx": cmd.body_vx, "vy": cmd.body_vy, "label": cmd.label,
        }))
        server.publish(Frame("battery", battery_payload(t)))
        if stop.wait(period):
            break
