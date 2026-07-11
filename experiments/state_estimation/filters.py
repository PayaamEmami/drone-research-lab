"""Kalman filters for onboard sensor streams.

This module holds the estimators used by the state-estimation experiment. They
turn the platform's noisy raw telemetry into smooth, usable state estimates and
demonstrate two distinct flavors of the Kalman filter:

- :class:`ScalarKalman` - a 1-D constant-velocity Kalman filter that smooths a
  single noisy channel (e.g. one range beam or one attitude angle) and also
  exposes a derivative (rate) estimate. One instance is used per channel to
  denoise every scalar telemetry stream independently.
- :class:`HeightFusionKalman` - a small *sensor-fusion* filter that estimates
  height and vertical velocity by combining two different sensors: the vertical
  accelerometer drives the prediction step, and the downward range finder
  corrects it in the update step. This is the "fuse multiple sensors" case, as
  opposed to per-channel smoothing.

Both filters are pure Python/NumPy and carry no hardware dependency, so they can
be developed and unit-tested offline against synthetic signals.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class ScalarKalman:
    """1-D constant-velocity Kalman filter for a single noisy sensor channel.

    The state is ``[value, rate]``: the filtered signal and its time derivative.
    Feed it measurements with :meth:`update` and advance time with
    :meth:`predict`; read the smoothed signal from :attr:`value`.

    :param q: process-noise intensity (larger => trust the motion model less,
        track measurements more aggressively).
    :param r: measurement variance of the sensor (larger => trust each reading
        less, smooth more heavily).
    """

    q: float = 1.0
    r: float = 0.05
    _x: np.ndarray = field(default_factory=lambda: np.zeros(2), init=False)
    _P: np.ndarray = field(default_factory=lambda: np.eye(2) * 1e3, init=False)
    _initialized: bool = field(default=False, init=False)

    def predict(self, dt: float) -> None:
        """Advance the state estimate by ``dt`` seconds (constant-velocity model)."""
        if not self._initialized or dt <= 0.0:
            return
        F = np.array([[1.0, dt], [0.0, 1.0]])
        # Continuous white-noise-acceleration process covariance, scaled by q.
        Q = self.q * np.array([
            [dt ** 3 / 3.0, dt ** 2 / 2.0],
            [dt ** 2 / 2.0, dt],
        ])
        self._x = F @ self._x
        self._P = F @ self._P @ F.T + Q

    def update(self, z: Optional[float]) -> None:
        """Fuse one measurement ``z``.

        Passing ``None`` (an out-of-range beam) should leave the estimate to
        coast on the last prediction rather than injecting a bad measurement.
        """
        if z is None:
            return
        if not self._initialized:
            self._x = np.array([float(z), 0.0])
            self._P = np.eye(2) * 1.0
            self._initialized = True
            return
        H = np.array([[1.0, 0.0]])
        y = float(z) - (H @ self._x)[0]
        S = (H @ self._P @ H.T)[0, 0] + self.r
        K = (self._P @ H.T).flatten() / S
        self._x = self._x + K * y
        self._P = (np.eye(2) - np.outer(K, H)) @ self._P

    @property
    def value(self) -> float:
        """The current filtered signal estimate."""
        return float(self._x[0])

    @property
    def rate(self) -> float:
        """The current estimated rate of change of the signal."""
        return float(self._x[1])


@dataclass
class HeightFusionKalman:
    """Fuse vertical accelerometer + downward range into height and velocity.

    State is ``[height, vertical_velocity]``. The accelerometer reading acts as a
    control input in the predict step (integrating acceleration into velocity and
    height); the downward range finder provides a direct height measurement in
    the update step. This is a compact example of multi-sensor fusion.

    :param q: process-noise intensity for the constant-acceleration prediction.
    :param r: measurement variance of the downward range finder (meters^2).
    """

    q: float = 0.5
    r: float = 0.02
    _x: np.ndarray = field(default_factory=lambda: np.zeros(2), init=False)
    _P: np.ndarray = field(default_factory=lambda: np.eye(2) * 1e3, init=False)
    _initialized: bool = field(default=False, init=False)

    def predict(self, dt: float, accel_z: float) -> None:
        """Advance height/velocity using measured vertical acceleration as input.

        ``accel_z`` is the world-frame vertical acceleration in m/s^2 (already
        gravity-compensated by the caller): it acts as a control input driving
        the constant-acceleration prediction.
        """
        if not self._initialized or dt <= 0.0:
            return
        F = np.array([[1.0, dt], [0.0, 1.0]])
        B = np.array([0.5 * dt ** 2, dt])
        Q = self.q * np.array([
            [dt ** 3 / 3.0, dt ** 2 / 2.0],
            [dt ** 2 / 2.0, dt],
        ])
        self._x = F @ self._x + B * float(accel_z)
        self._P = F @ self._P @ F.T + Q

    def update(self, z_range: Optional[float]) -> None:
        """Correct the estimate with a downward range measurement (meters)."""
        if z_range is None:
            return
        if not self._initialized:
            self._x = np.array([float(z_range), 0.0])
            self._P = np.eye(2) * 1.0
            self._initialized = True
            return
        H = np.array([[1.0, 0.0]])
        y = float(z_range) - (H @ self._x)[0]
        S = (H @ self._P @ H.T)[0, 0] + self.r
        K = (self._P @ H.T).flatten() / S
        self._x = self._x + K * y
        self._P = (np.eye(2) - np.outer(K, H)) @ self._P

    @property
    def height(self) -> float:
        return float(self._x[0])

    @property
    def velocity(self) -> float:
        return float(self._x[1])
