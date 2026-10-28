"""Tests for ``drl.motion.VelocityFlight`` landing safety guards."""
from __future__ import annotations

import math
from types import SimpleNamespace
from unittest.mock import MagicMock

from drl.motion import VelocityFlight, body_to_world


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


def _close(a: float, b: float) -> bool:
    return math.isclose(a, b, rel_tol=0.0, abs_tol=1e-9)


def test_body_to_world_is_identity_at_zero_yaw():
    vx, vy = body_to_world(0.2, 0.1, 0.0)
    assert _close(vx, 0.2)
    assert _close(vy, 0.1)


def test_body_to_world_forward_faces_world_y_at_90_degrees():
    vx, vy = body_to_world(0.3, 0.0, math.pi / 2)
    assert _close(vx, 0.0)
    assert _close(vy, 0.3)


def test_body_to_world_strafe_left_at_90_degrees_is_world_negative_x():
    vx, vy = body_to_world(0.0, 0.4, math.pi / 2)
    assert _close(vx, -0.4)
    assert _close(vy, 0.0)
