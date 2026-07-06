"""Parametric reference trajectories for the tracking controller.

A trajectory is a function of time that returns a desired setpoint
``(x, y, z)`` in meters relative to the takeoff point. The controller's job is
to drive the platform to follow this moving setpoint.

The default trajectory is an *expanding, ascending spiral*: the platform circles
while both the circle radius and the altitude grow with time, so the motion
spans all three axes simultaneously. Radius and height are capped so the path
stays inside a bounded volume.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

Point = Tuple[float, float, float]


@dataclass
class SpiralParams:
    """Geometry of an expanding, ascending spiral.

    :param base_radius: starting circle radius (m).
    :param radius_growth: radius increase per second (m/s).
    :param max_radius: radius cap (m).
    :param climb_rate: altitude increase per second (m/s).
    :param base_height: starting altitude (m).
    :param max_height: altitude cap (m).
    :param angular_rate: how fast the platform sweeps around the circle (rad/s).
    :param center: (x, y) center of the spiral in world coordinates (m).
    """

    base_radius: float = 0.3
    radius_growth: float = 0.05
    max_radius: float = 0.8
    climb_rate: float = 0.05
    base_height: float = 0.4
    max_height: float = 1.0
    angular_rate: float = 0.6
    center: Tuple[float, float] = (0.0, 0.0)


def spiral(t: float, p: SpiralParams) -> Point:
    """Return the desired ``(x, y, z)`` setpoint at time ``t`` seconds."""
    # TODO(trajectory_tracking): implement the expanding/ascending spiral.
    #   radius = min(base_radius + radius_growth * t, max_radius)
    #   z      = min(base_height + climb_rate * t, max_height)
    #   theta  = angular_rate * t
    #   x = center_x + radius * cos(theta); y = center_y + radius * sin(theta)
    raise NotImplementedError
