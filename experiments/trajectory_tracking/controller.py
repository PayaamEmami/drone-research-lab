"""PID position controllers for trajectory tracking.

The platform's firmware runs the inner attitude/rate loops; this module owns the
*outer* position loops. Each axis has its own PID that turns a position error
(reference minus estimate) into a velocity command, and the three velocity
commands are handed to the flight controller as a body/world velocity setpoint.

- :class:`PID` - a single-axis proportional-integral-derivative controller with
  output clamping and integral anti-windup.
- :class:`TrajectoryController` - bundles three :class:`PID` instances (x, y, z)
  and produces a velocity setpoint from a desired position and a current
  estimate.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple


@dataclass
class PID:
    """Single-axis PID controller with clamping and anti-windup.

    :param kp: proportional gain.
    :param ki: integral gain.
    :param kd: derivative gain.
    :param out_limit: symmetric clamp on the output command.
    """

    kp: float
    ki: float
    kd: float
    out_limit: float = 0.5
    _integral: float = field(default=0.0, init=False)
    _prev_error: float | None = field(default=None, init=False)

    def reset(self) -> None:
        """Clear the integral and derivative history."""
        self._integral = 0.0
        self._prev_error = None

    def step(self, error: float, dt: float) -> float:
        """Return the control output for the current error over timestep ``dt``.

        Clamps the output to ``out_limit`` and freezes the integral term while
        the (unclamped) output is saturated so it cannot wind up.
        """
        derivative = 0.0
        if self._prev_error is not None and dt > 0.0:
            derivative = (error - self._prev_error) / dt
        self._prev_error = error

        candidate_integral = self._integral + error * dt
        unclamped = self.kp * error + self.ki * candidate_integral + self.kd * derivative
        output = max(-self.out_limit, min(self.out_limit, unclamped))

        # Anti-windup: only accumulate the integral when not saturated, or when
        # integrating would pull the output back out of saturation.
        saturated = unclamped != output
        if not saturated or (error * unclamped < 0.0):
            self._integral = candidate_integral

        return output


@dataclass
class TrajectoryController:
    """Three-axis position controller producing a velocity setpoint."""

    x: PID
    y: PID
    z: PID

    def reset(self) -> None:
        self.x.reset()
        self.y.reset()
        self.z.reset()

    def step(
        self,
        reference: Tuple[float, float, float],
        estimate: Tuple[float, float, float],
        dt: float,
    ) -> Tuple[float, float, float]:
        """Return ``(vx, vy, vz)`` to move ``estimate`` toward ``reference``."""
        return (
            self.x.step(reference[0] - estimate[0], dt),
            self.y.step(reference[1] - estimate[1], dt),
            self.z.step(reference[2] - estimate[2], dt),
        )
