"""World-frame velocity flight on top of cflib's commander.

This module centralizes takeoff, landing, and velocity-setpoint streaming for
flight experiments. A background thread keeps resending the latest setpoint so
the firmware watchdog does not time out between control ticks.

Setpoints are sent in the world frame
(:meth:`cflib.crazyflie.commander.Commander.send_velocity_world_setpoint`) so
they align with position errors computed from ``stateEstimate.{x,y,z}``.

Typical use::

    from drl.motion import VelocityFlight

    with VelocityFlight(link.scf, default_height=0.4) as flight:
        flight.send_velocity(vx, vy, vz)
"""
from __future__ import annotations

import logging
import time
from threading import Event, Lock, Thread
from typing import Tuple

from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

logger = logging.getLogger(__name__)

# World-frame velocity command: (vx, vy, vz) in m/s and yaw rate in deg/s.
Setpoint = Tuple[float, float, float, float]


class VelocityFlight:
    """Takeoff, stream world-frame velocity setpoints, and land.

    Use as a context manager so landing always runs, even on Ctrl+C::

        with VelocityFlight(link.scf, default_height=0.4) as flight:
            while flying:
                flight.send_velocity(vx, vy, vz)

    :param scf: a connected :class:`SyncCrazyflie` (already armed).
    :param default_height: hover height reached during takeoff (m).
    :param takeoff_velocity: vertical speed used to climb on takeoff (m/s).
    :param keepalive_hz: rate at which the latest setpoint is resent so the
        firmware setpoint watchdog does not expire.
    """

    def __init__(
        self,
        scf: SyncCrazyflie,
        *,
        default_height: float = 0.4,
        takeoff_velocity: float = 0.3,
        keepalive_hz: float = 20.0,
    ):
        self._cf = scf.cf if isinstance(scf, SyncCrazyflie) else scf
        self.default_height = default_height
        self.takeoff_velocity = takeoff_velocity
        self._period = 1.0 / keepalive_hz
        self._setpoint: Setpoint = (0.0, 0.0, 0.0, 0.0)
        self._lock = Lock()
        self._stop = Event()
        self._thread: Thread | None = None
        self._flying = False

    def _send(self, sp: Setpoint) -> None:
        self._cf.commander.send_velocity_world_setpoint(*sp)

    def _keepalive(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                sp = self._setpoint
            self._send(sp)
            self._stop.wait(self._period)

    def take_off(self, height: float | None = None) -> None:
        """Climb straight up to ``height`` (or the default) and start hovering."""
        if self._flying:
            return
        if not self._cf.is_connected():
            raise RuntimeError("Crazyflie is not connected")
        target = self.default_height if height is None else height
        self._flying = True
        self._stop.clear()
        self._thread = Thread(target=self._keepalive, name="velocity-flight", daemon=True)
        self._thread.start()

        climb_time = target / self.takeoff_velocity if self.takeoff_velocity > 0 else 0.0
        self.send_velocity(0.0, 0.0, self.takeoff_velocity)
        time.sleep(climb_time)
        self.send_velocity(0.0, 0.0, 0.0)

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0) -> None:
        """Set the world-frame velocity setpoint (m/s) and yaw rate (deg/s)."""
        sp: Setpoint = (vx, vy, vz, yaw_rate)
        with self._lock:
            self._setpoint = sp
        if self._flying:
            self._send(sp)

    def hover(self) -> None:
        """Hold position (zero velocity)."""
        self.send_velocity(0.0, 0.0, 0.0)

    def land(self, velocity: float | None = None) -> None:
        """Descend and stop the motors. Safe to call more than once."""
        if not self._flying:
            return
        descent = velocity if velocity is not None else self.takeoff_velocity
        descent_time = self.default_height / descent if descent > 0 else 0.0
        self.send_velocity(0.0, 0.0, -abs(descent))
        time.sleep(descent_time)
        self.stop()

    def stop(self) -> None:
        """Cut setpoints immediately (emergency halt) and stop the motors."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._flying:
            try:
                self._cf.commander.send_stop_setpoint()
                self._cf.commander.send_notify_setpoint_stop()
            except Exception:  # noqa: BLE001 - teardown must not raise
                logger.debug("Sending stop setpoint failed", exc_info=True)
        self._flying = False

    def __enter__(self) -> "VelocityFlight":
        self.take_off()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.land()
