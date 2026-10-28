"""Reactive hand chase: hold a standoff on the front beam and yaw toward the hand.

The drone has no camera. A hand is whatever the Multi-ranger sees in front of
it. The controller holds that beam near a fixed distance (close the gap when
the hand backs away, reverse when it comes closer) and yaws toward whichever
side beam is shorter, so a hand that slides off the nose is reacquired.

Commands are computed in the body frame (x forward, y left) and rotated into
the world frame for :class:`drl.motion.VelocityFlight`.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional

from drl.motion import body_to_world

Ranges = Dict[str, Optional[float]]


@dataclass(frozen=True)
class ChaseParams:
    """Tunables for :meth:`HandChase.step`.

    Distances are meters. Yaw rates are degrees per second.
    """

    standoff: float = 0.30
    kp_range: float = 0.8
    max_speed: float = 0.25
    too_close: float = 0.12
    lost_range: float = 1.2
    yaw_gain: float = 80.0
    max_yaw_rate: float = 45.0
    search_yaw_rate: float = 25.0
    side_deadband: float = 0.04
    kp_z: float = 1.2
    max_climb: float = 0.2
    target_height: float = 0.4


@dataclass(frozen=True)
class FlightCommand:
    """One control tick, in both frames.

    ``vx`` / ``vy`` / ``vz`` are world-frame m/s. ``yaw_rate`` is deg/s.
    ``body_vx`` / ``body_vy`` are the pre-rotation commands (forward / left)
    for the dashboard arrow, which draws body frame.
    """

    vx: float
    vy: float
    vz: float
    yaw_rate: float
    body_vx: float
    body_vy: float
    label: str


def _clamp(value: float, limit: float) -> float:
    limit = abs(limit)
    return max(-limit, min(limit, value))


def _limit_horizontal(vx: float, vy: float, limit: float) -> tuple[float, float]:
    limit = abs(limit)
    mag = math.hypot(vx, vy)
    if mag > limit and mag > 0.0:
        scale = limit / mag
        return vx * scale, vy * scale
    return vx, vy


class HandChase:
    """Stateless range law plus a memory of which way to search.

    ``search_sign`` remembers the last side imbalance so that, once the hand
    leaves every beam, the drone keeps yawing toward the side it disappeared
    on instead of always spinning left.
    """

    def __init__(self) -> None:
        self.search_sign = 1.0

    def step(self, ranges: Ranges, yaw_rad: float, z: float, params: ChaseParams) -> FlightCommand:
        """Return the velocity command for one control tick."""
        front = ranges.get("front")
        left = ranges.get("left")
        right = ranges.get("right")

        front_lost = front is None or front >= params.lost_range
        body_vx = 0.0
        label = "search" if front_lost else "hold"

        if front is not None and front < params.too_close:
            body_vx = -params.max_speed
            label = "retreat"
        elif not front_lost and front is not None:
            body_vx = _clamp(params.kp_range * (front - params.standoff), params.max_speed)
            if body_vx > 0.02:
                label = "approach"
            elif body_vx < -0.02:
                label = "back off"
            else:
                label = "hold"

        dodge_left = left is not None and left < params.too_close
        dodge_right = right is not None and right < params.too_close
        body_vy = 0.0
        if dodge_left:
            body_vy -= params.max_speed
        if dodge_right:
            body_vy += params.max_speed
        if dodge_left or dodge_right:
            label = "dodge"

        yaw_rate = self._yaw_rate(left, right, params, front_lost, dodge_left or dodge_right)
        if label == "hold" and abs(yaw_rate) > 1.0:
            label = "turn"

        body_vx, body_vy = _limit_horizontal(body_vx, body_vy, params.max_speed)
        vx, vy = body_to_world(body_vx, body_vy, yaw_rad)
        vz = _clamp(params.kp_z * (params.target_height - z), params.max_climb)
        return FlightCommand(vx, vy, vz, yaw_rate, body_vx, body_vy, label)

    def _yaw_rate(
        self,
        left: Optional[float],
        right: Optional[float],
        params: ChaseParams,
        front_lost: bool,
        dodging: bool,
    ) -> float:
        """Yaw toward the nearer usable side beam, or search when nothing is ahead.

        A beam inside ``too_close`` is treated as far, so the nose turns away
        from a wall it is about to hit. Strafing away is handled separately.
        """
        if left is None and right is None:
            side: Optional[float] = None
        else:
            far = params.lost_range
            l = far if left is None or left < params.too_close else left
            r = far if right is None or right < params.too_close else right
            diff = r - l
            if abs(diff) < params.side_deadband:
                side = 0.0
            else:
                side = _clamp(params.yaw_gain * diff, params.max_yaw_rate)
                if abs(side) > 1.0:
                    self.search_sign = math.copysign(1.0, side)

        if front_lost and not dodging and (side is None or abs(side) < 1.0):
            return self.search_sign * params.search_yaw_rate
        return 0.0 if side is None else side
