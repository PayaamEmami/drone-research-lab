"""Correlative scan matching for pose-drift correction.

The onboard state estimate is an excellent *short-term* motion source but drifts
over a longer flight. Scan matching treats that estimate as odometry and, at
each step, searches a small window of pose corrections ``(dx, dy, dyaw)`` around
the odometry prediction for the one whose Multi-ranger beams best agree with the
map built so far (scored by :meth:`OccupancyGrid.score_scan`). The corrected
pose is then used to integrate the scan, keeping the map globally consistent.

A coarse-to-fine search keeps the per-step cost low enough for the live loop.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from experiments.slam.mapper import OccupancyGrid

_HORIZONTAL_BEAMS = ("front", "back", "left", "right")

Pose = Tuple[float, float, float]  # (x, y, yaw_rad)


@dataclass
class MatchConfig:
    """Search window and resolution for the scan matcher.

    :param win_xy: half-width of the translational search window (m).
    :param step_xy: coarse translational step (m).
    :param win_yaw: half-width of the yaw search window (rad).
    :param step_yaw: coarse yaw step (rad).
    :param refine_iters: number of coarse-to-fine refinement passes.
    :param min_beams: minimum valid beams required to attempt a match.
    """

    win_xy: float = 0.10
    step_xy: float = 0.02
    win_yaw: float = 0.09
    step_yaw: float = 0.018
    refine_iters: int = 2
    min_beams: int = 2


def match_scan(
    grid: OccupancyGrid,
    predicted_pose: Pose,
    ranges: Dict[str, Optional[float]],
    config: Optional[MatchConfig] = None,
) -> Pose:
    """Return the best-scoring pose near ``predicted_pose`` for this scan.

    Falls back to ``predicted_pose`` when there is too little map structure or
    too few valid beams to match against.
    """
    cfg = config or MatchConfig()

    valid = sum(
        1 for beam in _HORIZONTAL_BEAMS
        if ranges.get(beam) is not None and ranges[beam] < grid.cfg.max_range_m
    )
    if valid < cfg.min_beams:
        return predicted_pose

    best_pose = predicted_pose
    best_score = grid.score_scan(*predicted_pose, ranges)

    win_xy, step_xy = cfg.win_xy, cfg.step_xy
    win_yaw, step_yaw = cfg.win_yaw, cfg.step_yaw

    for _ in range(cfg.refine_iters):
        cx, cy, cyaw = best_pose
        xs = _offsets(cx, win_xy, step_xy)
        ys = _offsets(cy, win_xy, step_xy)
        yaws = _offsets(cyaw, win_yaw, step_yaw)
        for x in xs:
            for y in ys:
                for yaw in yaws:
                    score = grid.score_scan(x, y, yaw, ranges)
                    if score > best_score:
                        best_score = score
                        best_pose = (x, y, yaw)
        # Refine: shrink the window and step for the next, finer pass.
        win_xy, step_xy = step_xy, step_xy / 2.0
        win_yaw, step_yaw = step_yaw, step_yaw / 2.0

    return best_pose


def _offsets(center: float, window: float, step: float):
    """Candidate values from center-window to center+window inclusive."""
    if step <= 0:
        return [center]
    n = int(round(window / step))
    return [center + i * step for i in range(-n, n + 1)]
