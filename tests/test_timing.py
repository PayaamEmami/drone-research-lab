"""Tests for monotonic timing helpers."""
from __future__ import annotations

import time

from drl.timing import monotonic_elapsed
from drl.telemetry import position_from_sample, pose_from_sample, yaw_radians


def test_monotonic_elapsed_first_tick_uses_elapsed_as_dt():
    t0 = time.monotonic()
    elapsed, dt = monotonic_elapsed(t0)
    assert elapsed >= 0.0
    assert dt == elapsed


def test_monotonic_elapsed_step_dt():
    t0 = time.monotonic()
    time.sleep(0.01)
    elapsed1, _ = monotonic_elapsed(t0)
    last = t0 + elapsed1
    time.sleep(0.01)
    elapsed2, dt = monotonic_elapsed(t0, last)
    assert elapsed2 > elapsed1
    assert dt > 0.0


def test_yaw_radians_converts_degrees():
    assert abs(yaw_radians({"stabilizer.yaw": 180.0}) - 3.14159265) < 1e-5


def test_position_from_sample_defaults_to_zero():
    assert position_from_sample({}) == (0.0, 0.0, 0.0)


def test_pose_from_sample_includes_yaw():
    sample = {
        "stateEstimate.x": 1.0,
        "stateEstimate.y": 2.0,
        "stateEstimate.z": 0.3,
        "stabilizer.yaw": 90.0,
    }
    x, y, z, yaw = pose_from_sample(sample)
    assert (x, y, z) == (1.0, 2.0, 0.3)
    assert abs(yaw - 1.5707963) < 1e-5
