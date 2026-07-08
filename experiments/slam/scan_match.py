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
    # TODO(slam): coarse-to-fine search over (dx, dy, dyaw) maximizing
    # grid.score_scan(...); shrink the step and window each refine pass. Bail out
    # to predicted_pose when fewer than config.min_beams beams have returns.
    raise NotImplementedError
