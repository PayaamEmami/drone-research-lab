"""Orbit a nearby object using only the Multi-ranger's front beam.

The drone yaws in place until the front beam is the closest horizontal return
inside an acquisition window, then circles that return: forward/back holds the
range, a sideways velocity walks the circle, and an opposing yaw rate keeps
the nose pointed at the object. Losing the return, or a side beam getting too
close, drops back to the search.

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
class OrbitParams:
    """Tunables for :meth:`OrbitTracker.step`.

    ``direction`` is +1 for counterclockwise (strafe right and yaw left) and
    -1 for clockwise (strafe left and yaw right). The yaw rate is opposite
    the strafe so an object that starts in front stays in front.
    ``object_radius`` is only the yaw feedforward's guess of how big the
    object is; the range setpoint is ``radius``.
    """

    radius: float = 0.45
    tangential_speed: float = 0.15
    direction: int = 1
    object_radius: float = 0.20
    kp_radial: float = 0.8
    max_speed: float = 0.25
    too_close: float = 0.15
    acquire_max: float = 0.80
    lose_range: float = 1.20
    acquire_hold_s: float = 0.30
    lose_hold_s: float = 0.15
    search_yaw_rate: float = 30.0
    max_yaw_rate: float = 60.0
    min_radius: float = 0.25
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


def _tangential(error: float, params: OrbitParams) -> float:
    """Full orbit speed near the setpoint; mostly close range when far from it."""
    band = 0.10
    if abs(error) <= band:
        return params.tangential_speed
    scale = 1.0 - (abs(error) - band) / max(params.radius, 0.05)
    return params.tangential_speed * max(0.2, scale)


def _front_is_target(
    front: Optional[float],
    left: Optional[float],
    right: Optional[float],
    params: OrbitParams,
) -> bool:
    """True when the front beam, not a side beam, is looking at a candidate."""
    if front is None or front > params.acquire_max or front < params.too_close:
        return False
    for side in (left, right):
        if side is not None and side + 0.02 < front:
            return False
    return True


class OrbitTracker:
    """Acquire / orbit state machine over range beams."""

    def __init__(self) -> None:
        self.mode = "acquire"
        self.hold_s = 0.0
        self.miss_s = 0.0

    def step(
        self,
        ranges: Ranges,
        yaw_rad: float,
        z: float,
        dt: float,
        params: OrbitParams,
    ) -> FlightCommand:
        """Return the velocity command for one control tick."""
        front = ranges.get("front")
        left = ranges.get("left")
        right = ranges.get("right")
        vz = _clamp(params.kp_z * (params.target_height - z), params.max_climb)
        capped = min(max(dt, 0.0), 0.2)

        side_too_close = (
            (left is not None and left < params.too_close)
            or (right is not None and right < params.too_close)
        )
        if side_too_close:
            body_vy = 0.0
            if left is not None and left < params.too_close:
                body_vy -= params.max_speed
            if right is not None and right < params.too_close:
                body_vy += params.max_speed
            self.mode = "acquire"
            self.hold_s = 0.0
            self.miss_s = 0.0
            return self._pack(0.0, body_vy, vz, 0.0, yaw_rad, "dodge", params)

        if self.mode == "orbit":
            lost = front is None or front > params.lose_range
            if lost:
                self.miss_s += capped
                if self.miss_s >= params.lose_hold_s:
                    self.mode = "acquire"
                    self.hold_s = 0.0
                    self.miss_s = 0.0
                    return self._pack(0.0, 0.0, vz, 0.0, yaw_rad, "lost", params)
                return self._pack(0.0, 0.0, vz, 0.0, yaw_rad, "hold", params)
            self.miss_s = 0.0
            return self._orbit(front, vz, yaw_rad, params)

        if _front_is_target(front, left, right, params):
            self.hold_s += capped
            if self.hold_s >= params.acquire_hold_s:
                self.mode = "orbit"
                self.hold_s = 0.0
                self.miss_s = 0.0
                return self._orbit(front, vz, yaw_rad, params)
            return self._pack(0.0, 0.0, vz, 0.0, yaw_rad, "acquire", params)

        self.hold_s = 0.0
        yaw_rate = params.direction * params.search_yaw_rate
        return self._pack(0.0, 0.0, vz, yaw_rate, yaw_rad, "acquire", params)

    def _orbit(
        self,
        front: float,
        vz: float,
        yaw_rad: float,
        params: OrbitParams,
    ) -> FlightCommand:
        if front < params.too_close:
            return self._pack(-params.max_speed, 0.0, vz, 0.0, yaw_rad, "retreat", params)
        error = front - params.radius
        body_vx = _clamp(params.kp_radial * error, params.max_speed)
        speed = _tangential(error, params)
        # Strafe and yaw have opposite signs: sliding left swings an object
        # that was ahead off to the right, so the nose has to yaw right to
        # keep it on the front beam.
        body_vy = -params.direction * speed
        omega = params.direction * speed / max(front + params.object_radius, params.min_radius)
        yaw_rate = _clamp(math.degrees(omega), params.max_yaw_rate)
        return self._pack(body_vx, body_vy, vz, yaw_rate, yaw_rad, "orbit", params)

    def _pack(
        self,
        body_vx: float,
        body_vy: float,
        vz: float,
        yaw_rate: float,
        yaw_rad: float,
        label: str,
        params: OrbitParams,
    ) -> FlightCommand:
        body_vx, body_vy = _limit_horizontal(body_vx, body_vy, params.max_speed)
        vx, vy = body_to_world(body_vx, body_vy, yaw_rad)
        return FlightCommand(vx, vy, vz, yaw_rate, body_vx, body_vy, label)
