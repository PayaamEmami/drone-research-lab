"""Tests for ``experiments.orbit.control`` and its cylinder preview."""
from __future__ import annotations

import math

from experiments.orbit.control import OrbitParams, OrbitTracker
from experiments.orbit.demo import CYLINDER_CENTER, CYLINDER_RADIUS, cylinder_ranges


def _close(a: float, b: float, tol: float = 1e-6) -> bool:
    return math.isclose(a, b, rel_tol=0.0, abs_tol=tol)


def _target(**overrides: float) -> dict:
    ranges = {"front": 0.50, "left": 1.5, "right": 1.5, "back": 2.0}
    ranges.update(overrides)
    return ranges


def test_acquire_holds_heading_while_the_target_is_already_in_front():
    tracker = OrbitTracker()
    params = OrbitParams()
    cmd = tracker.step(_target(), 0.0, params.target_height, 0.05, params)
    assert tracker.mode == "acquire"
    assert cmd.label == "acquire"
    assert _close(cmd.yaw_rate, 0.0)
    assert _close(cmd.body_vx, 0.0)


def test_acquire_yaws_when_nothing_is_in_front():
    tracker = OrbitTracker()
    params = OrbitParams()
    cmd = tracker.step(
        {"front": None, "left": None, "right": None},
        0.0,
        params.target_height,
        0.05,
        params,
    )
    assert cmd.label == "acquire"
    assert cmd.yaw_rate > 0.0


def test_a_closer_side_beam_blocks_acquisition():
    tracker = OrbitTracker()
    params = OrbitParams(acquire_hold_s=0.0)
    for _ in range(5):
        cmd = tracker.step(
            _target(front=0.50, left=0.30),
            0.0,
            params.target_height,
            0.05,
            params,
        )
    assert tracker.mode == "acquire"
    assert cmd.label == "acquire"


def test_locks_after_the_front_beam_stays_on_target():
    tracker = OrbitTracker()
    params = OrbitParams(acquire_hold_s=0.30)
    ranges = _target(front=params.radius)
    for _ in range(5):
        tracker.step(ranges, 0.0, params.target_height, 0.05, params)
        assert tracker.mode == "acquire"
    cmd = tracker.step(ranges, 0.0, params.target_height, 0.05, params)
    assert tracker.mode == "orbit"
    assert cmd.label == "orbit"


def test_orbit_holds_range_with_a_tangential_velocity():
    tracker = OrbitTracker()
    params = OrbitParams(acquire_hold_s=0.0)
    cmd = tracker.step(_target(front=params.radius), 0.0, params.target_height, 0.05, params)
    assert cmd.label == "orbit"
    assert _close(cmd.body_vx, 0.0)
    assert _close(cmd.body_vy, -params.tangential_speed)
    assert cmd.yaw_rate > 0.0


def test_clockwise_flips_strafe_and_yaw():
    tracker = OrbitTracker()
    params = OrbitParams(acquire_hold_s=0.0, direction=-1)
    cmd = tracker.step(_target(front=params.radius), 0.0, params.target_height, 0.05, params)
    assert cmd.body_vy > 0.0
    assert cmd.yaw_rate < 0.0


def test_range_beyond_acquire_max_stays_in_orbit_until_lose_range():
    tracker = OrbitTracker()
    params = OrbitParams(acquire_hold_s=0.0)
    tracker.step(_target(front=params.radius), 0.0, params.target_height, 0.05, params)
    assert tracker.mode == "orbit"
    cmd = tracker.step(_target(front=0.95), 0.0, params.target_height, 0.05, params)
    assert tracker.mode == "orbit"
    assert cmd.label == "orbit"
    assert cmd.body_vx > 0.0


def test_a_brief_miss_does_not_drop_the_lock():
    tracker = OrbitTracker()
    params = OrbitParams(acquire_hold_s=0.0, lose_hold_s=0.15)
    tracker.step(_target(front=params.radius), 0.0, params.target_height, 0.05, params)
    cmd = tracker.step(
        {"front": None, "left": 1.5, "right": 1.5},
        0.0,
        params.target_height,
        0.05,
        params,
    )
    assert tracker.mode == "orbit"
    assert cmd.label == "hold"


def test_a_sustained_miss_returns_to_acquire():
    tracker = OrbitTracker()
    params = OrbitParams(acquire_hold_s=0.0, lose_hold_s=0.15)
    tracker.step(_target(front=params.radius), 0.0, params.target_height, 0.05, params)
    cmd = tracker.step(
        {"front": None, "left": 1.5, "right": 1.5},
        0.0,
        params.target_height,
        0.20,
        params,
    )
    assert tracker.mode == "acquire"
    assert cmd.label == "lost"


def test_too_close_on_the_front_beam_retreats_without_strafing():
    tracker = OrbitTracker()
    params = OrbitParams(acquire_hold_s=0.0)
    tracker.step(_target(front=params.radius), 0.0, params.target_height, 0.05, params)
    cmd = tracker.step(_target(front=0.05), 0.0, params.target_height, 0.05, params)
    assert tracker.mode == "orbit"
    assert cmd.label == "retreat"
    assert cmd.body_vx < 0.0
    assert _close(cmd.body_vy, 0.0)


def test_a_close_side_beam_aborts_to_acquire_and_strafes_away():
    tracker = OrbitTracker()
    params = OrbitParams(acquire_hold_s=0.0)
    tracker.step(_target(front=params.radius), 0.0, params.target_height, 0.05, params)
    cmd = tracker.step(_target(left=0.05), 0.0, params.target_height, 0.05, params)
    assert tracker.mode == "acquire"
    assert cmd.label == "dodge"
    assert cmd.body_vy < 0.0


def test_cylinder_in_front_is_seen_by_the_front_beam():
    ranges = cylinder_ranges(0.0, 0.0, 0.0, 0.4)
    expected = CYLINDER_CENTER[0] - CYLINDER_RADIUS
    assert ranges["front"] is not None
    assert abs(ranges["front"] - expected) < 0.02
    assert ranges["left"] > 1.5


def test_closed_loop_holds_range_around_the_cylinder():
    tracker = OrbitTracker()
    params = OrbitParams(object_radius=CYLINDER_RADIUS)
    x = y = yaw = 0.0
    z = params.target_height
    dt = 0.05
    saw_orbit = False
    path = 0.0
    fronts: list[float] = []
    for _ in range(400):
        ranges = cylinder_ranges(x, y, yaw, z)
        cmd = tracker.step(ranges, yaw, z, dt, params)
        if tracker.mode == "orbit" and cmd.label == "orbit" and ranges["front"] is not None:
            saw_orbit = True
            fronts.append(ranges["front"])
        nx = x + cmd.vx * dt
        ny = y + cmd.vy * dt
        path += math.hypot(nx - x, ny - y)
        x, y = nx, ny
        z += cmd.vz * dt
        yaw += math.radians(cmd.yaw_rate) * dt
        dist = math.hypot(x - CYLINDER_CENTER[0], y - CYLINDER_CENTER[1])
        assert dist > CYLINDER_RADIUS + 0.02
    assert saw_orbit
    assert path > 0.5
    assert abs(x) < 3.0 and abs(y) < 3.0
    tail = fronts[-40:]
    assert tail
    mean = sum(tail) / len(tail)
    assert abs(mean - params.radius) < 0.12
