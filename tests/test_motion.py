"""Tests for ``drl.motion.VelocityFlight`` landing safety guards."""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

from drl.motion import VelocityFlight


def _flight(**kwargs) -> VelocityFlight:
    commander = SimpleNamespace(
        send_velocity_world_setpoint=MagicMock(),
        send_stop_setpoint=MagicMock(),
        send_notify_setpoint_stop=MagicMock(),
    )
    cf = SimpleNamespace(is_connected=lambda: True, commander=commander)
    return VelocityFlight(cf, **kwargs)


def test_land_uses_positive_descent_when_velocity_is_zero(monkeypatch):
    sleeps: list[float] = []
    monkeypatch.setattr("drl.motion.time.sleep", sleeps.append)

    flight = _flight(default_height=0.4, takeoff_velocity=0.0)
    flight._flying = True
    flight.land(velocity=0.0, from_height=0.4)

    assert sleeps == [0.4 / 0.3]
    flight._cf.commander.send_velocity_world_setpoint.assert_any_call(0.0, 0.0, -0.3, 0.0)
    assert flight._flying is False


def test_land_uses_abs_takeoff_velocity_when_descent_non_positive(monkeypatch):
    sleeps: list[float] = []
    monkeypatch.setattr("drl.motion.time.sleep", sleeps.append)

    flight = _flight(default_height=0.4, takeoff_velocity=0.5)
    flight._flying = True
    flight.land(velocity=-0.0, from_height=0.5)

    assert sleeps == [0.5 / 0.5]
    flight._cf.commander.send_velocity_world_setpoint.assert_any_call(0.0, 0.0, -0.5, 0.0)
