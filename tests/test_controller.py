"""Tests for ``experiments.trajectory_tracking.controller``.

Offline checks for single-axis PID behavior and the three-axis
``TrajectoryController`` wrapper. No Crazyflie or radio link.
"""
from __future__ import annotations

from experiments.trajectory_tracking.controller import PID, TrajectoryController


# ---------------------------------------------------------------------------
# PID
# ---------------------------------------------------------------------------


def test_pid_zero_error_zero_output():
    pid = PID(kp=1.0, ki=0.5, kd=0.1)
    assert pid.step(0.0, 0.02) == 0.0


def test_pid_proportional_sign_and_clamp():
    pid = PID(kp=10.0, ki=0.0, kd=0.0, out_limit=0.3)
    assert pid.step(5.0, 0.02) == 0.3
    assert pid.step(-5.0, 0.02) == -0.3


def test_pid_drives_error_to_zero():
    # Integrator plant: position += velocity * dt; PID should null tracking error.
    pid = PID(kp=1.2, ki=0.4, kd=0.05, out_limit=0.5)
    x, target, dt = 0.0, 1.0, 0.05
    for _ in range(500):
        v = pid.step(target - x, dt)
        x += v * dt
    assert abs(target - x) < 0.02


def test_pid_anti_windup_bounds_integral():
    # Saturate in one direction, then reverse error; output should flip quickly.
    pid = PID(kp=1.0, ki=5.0, kd=0.0, out_limit=0.5)
    for _ in range(200):
        out = pid.step(10.0, 0.02)
        assert -0.5 <= out <= 0.5
    recovered = False
    for _ in range(20):
        out = pid.step(-10.0, 0.02)
        if out <= -0.4:
            recovered = True
            break
    assert recovered


def test_pid_reset_clears_state():
    pid = PID(kp=0.0, ki=1.0, kd=0.0, out_limit=10.0)
    pid.step(1.0, 0.1)
    pid.step(1.0, 0.1)
    assert pid._integral != 0.0
    pid.reset()
    assert pid._integral == 0.0
    assert pid._prev_error is None


# ---------------------------------------------------------------------------
# TrajectoryController
# ---------------------------------------------------------------------------


def test_trajectory_controller_three_axes():
    ctrl = TrajectoryController(
        x=PID(kp=1.0, ki=0.0, kd=0.0, out_limit=1.0),
        y=PID(kp=1.0, ki=0.0, kd=0.0, out_limit=1.0),
        z=PID(kp=1.0, ki=0.0, kd=0.0, out_limit=1.0),
    )
    vx, vy, vz = ctrl.step(reference=(0.5, -0.5, 0.2), estimate=(0.0, 0.0, 0.0), dt=0.02)
    assert vx > 0 and vy < 0 and vz > 0
