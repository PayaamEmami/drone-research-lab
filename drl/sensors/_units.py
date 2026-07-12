"""Shared unit conversion for deck range sensors."""
from __future__ import annotations

from typing import Optional

from drl.config import RANGER_MAX_MM


def mm_to_m(value: Optional[float]) -> Optional[float]:
    """Convert a raw millimeter reading to meters, or None if out of range."""
    if value is None:
        return None
    if value >= RANGER_MAX_MM:
        return None
    return value / 1000.0
