"""Shared fixtures and synthetic helpers for the offline test suite.

All tests in this package run without hardware. This module supplies:

- a seeded RNG for reproducible noisy signals;
- :func:`noisy_signal` for filter tests;
- :func:`simulate_ranges` for Multi-ranger returns inside a rectangular room.

Beam bearings match ``experiments.slam.mapper``.
"""
from __future__ import annotations

import math
from typing import Dict, Optional

import numpy as np
import pytest


@pytest.fixture
def rng() -> np.random.Generator:
    """Fixed-seed RNG shared by filter and noise tests."""
    return np.random.default_rng(1234)


def noisy_signal(
    true_values: np.ndarray,
    noise_std: float,
    generator: np.random.Generator,
) -> np.ndarray:
    """Return ``true_values`` with additive Gaussian noise."""
    return true_values + generator.normal(0.0, noise_std, size=true_values.shape)


BEAM_BEARINGS = {
    "front": 0.0,
    "left": math.pi / 2,
    "back": math.pi,
    "right": -math.pi / 2,
}


def simulate_ranges(
    x: float,
    y: float,
    yaw: float,
    *,
    room: tuple = (-2.0, 2.0, -2.0, 2.0),
    max_range: float = 3.5,
) -> Dict[str, Optional[float]]:
    """Ray-cast the four horizontal beams against axis-aligned walls.

    :param room: ``(xmin, xmax, ymin, ymax)`` in meters.
    :returns: per-beam range in meters, or ``None`` beyond ``max_range``.
    """
    xmin, xmax, ymin, ymax = room
    ranges: Dict[str, Optional[float]] = {}
    for beam, bearing in BEAM_BEARINGS.items():
        angle = yaw + bearing
        dx, dy = math.cos(angle), math.sin(angle)
        best = math.inf
        if dx > 1e-9:
            best = min(best, (xmax - x) / dx)
        elif dx < -1e-9:
            best = min(best, (xmin - x) / dx)
        if dy > 1e-9:
            best = min(best, (ymax - y) / dy)
        elif dy < -1e-9:
            best = min(best, (ymin - y) / dy)
        ranges[beam] = best if best <= max_range else None
    ranges["up"] = None
    ranges["down"] = None
    return ranges
