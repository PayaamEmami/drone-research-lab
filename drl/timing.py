"""Monotonic clock helpers for control loops."""
from __future__ import annotations

import time
from typing import Optional, Tuple


def monotonic_elapsed(t0: float, last: Optional[float] = None) -> Tuple[float, float]:
    """Return ``(elapsed, dt)`` seconds using a monotonic clock.

    ``elapsed`` is measured from ``t0`` (a prior :func:`time.monotonic` value);
    ``dt`` is the step since ``last`` (falls back to ``elapsed`` on the first
    tick when ``last`` is None).
    """
    now = time.monotonic()
    elapsed = now - t0
    dt = elapsed if last is None else now - last
    return elapsed, dt
