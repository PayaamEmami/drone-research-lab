"""Tests for ``experiments.hand_chase.control`` and its range preview."""
from __future__ import annotations

import math

from experiments.hand_chase.control import ChaseParams, HandChase
from experiments.hand_chase.demo import hand_ranges


def _close(a: float, b: float, tol: float = 1e-6) -> bool:
    return math.isclose(a, b, rel_tol=0.0, abs_tol=tol)


def test_hold_at_standoff_commands_nothing():
    cmd = HandChase().step(
        {"front": 0.30, "left": 1.0, "right": 1.0},
        yaw_rad=0.0,
        z=0.4,
        params=ChaseParams(),
    )
    assert cmd.label == "hold"
    assert _close(cmd.body_vx, 0.0)
    assert _close(cmd.body_vy, 0.0)
    assert _close(cmd.yaw_rate, 0.0)
    assert _close(cmd.vz, 0.0)


def test_approaches_when_the_hand_is_farther_than_standoff():
    cmd = HandChase().step(
        {"front": 0.60, "left": 1.0, "right": 1.0},
        yaw_rad=0.0,
        z=0.4,
        params=ChaseParams(),
    )
    assert cmd.label == "approach"
    assert cmd.body_vx > 0.0
    assert cmd.body_vx <= 0.25


def test_backs_off_when_the_hand_is_inside_standoff():
    cmd = HandChase().step(
        {"front": 0.20, "left": 1.0, "right": 1.0},
        yaw_rad=0.0,
        z=0.4,
        params=ChaseParams(),
    )
    assert cmd.label == "back off"
    assert cmd.body_vx < 0.0


def test_retreats_at_full_speed_when_too_close():
    params = ChaseParams()
    cmd = HandChase().step(
        {"front": 0.05, "left": 1.0, "right": 1.0},
        yaw_rad=0.0,
        z=0.4,
        params=params,
    )
    assert cmd.label == "retreat"
    assert _close(cmd.body_vx, -params.max_speed)


def test_lost_front_beam_searches_instead_of_driving():
    cmd = HandChase().step(
        {"front": None, "left": None, "right": None},
        yaw_rad=0.0,
        z=0.4,
        params=ChaseParams(),
    )
    assert cmd.label == "search"
    assert _close(cmd.body_vx, 0.0)
    assert cmd.yaw_rate > 0.0


def test_yaws_toward_the_closer_side_and_remembers_it():
    chase = HandChase()
    params = ChaseParams()
    turned = chase.step(
        {"front": 0.30, "left": 0.80, "right": 0.40},
        yaw_rad=0.0,
        z=0.4,
        params=params,
    )
    assert turned.yaw_rate < 0.0
    assert turned.label == "turn"

    searching = chase.step(
        {"front": None, "left": None, "right": None},
        yaw_rad=0.0,
        z=0.4,
        params=params,
    )
    assert searching.label == "search"
    assert searching.yaw_rate < 0.0


def test_strafes_away_from_a_wall_on_the_left():
    cmd = HandChase().step(
        {"front": 0.30, "left": 0.05, "right": 0.80},
        yaw_rad=0.0,
        z=0.4,
        params=ChaseParams(),
    )
    assert cmd.label == "dodge"
    assert cmd.body_vy < 0.0
    assert cmd.yaw_rate <= 0.0


def test_forward_at_90_degrees_yaw_is_world_positive_y():
    cmd = HandChase().step(
        {"front": 0.60, "left": 1.0, "right": 1.0},
        yaw_rad=math.pi / 2,
        z=0.4,
        params=ChaseParams(),
    )
    assert _close(cmd.vx, 0.0, tol=1e-9)
    assert cmd.vy > 0.0


def test_climbs_when_below_the_target_height():
    cmd = HandChase().step(
        {"front": 0.30, "left": 1.0, "right": 1.0},
        yaw_rad=0.0,
        z=0.1,
        params=ChaseParams(),
    )
    assert cmd.vz > 0.0


def test_hand_on_the_axis_is_seen_by_the_front_beam_only():
    ranges = hand_ranges(0.0, 0.0, 0.0, 0.4, (0.60, 0.0))
    assert ranges["front"] is not None and ranges["front"] < 0.7
    assert ranges["left"] is not None and ranges["left"] > 1.5
    assert ranges["right"] is not None and ranges["right"] > 1.5


def test_closed_loop_settles_on_a_fixed_hand():
    chase = HandChase()
    params = ChaseParams()
    x = y = yaw = 0.0
    z = 0.2
    hand = (0.70, 0.0)
    dt = 0.05
    for _ in range(200):
        ranges = hand_ranges(x, y, yaw, z, hand)
        cmd = chase.step(ranges, yaw, z, params)
        x += cmd.vx * dt
        y += cmd.vy * dt
        z += cmd.vz * dt
        yaw += math.radians(cmd.yaw_rate) * dt
    ranges = hand_ranges(x, y, yaw, z, hand)
    assert ranges["front"] is not None
    assert abs(ranges["front"] - params.standoff) < 0.08
    assert abs(z - params.target_height) < 0.05
    assert abs(y) < 0.15
