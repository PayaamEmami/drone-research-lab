"""Synthetic dashboard data for previewing the UI without hardware.

This module is experiment-agnostic. It provides generic per-frame mock
generators and a small harness loop so any experiment can open its dashboard
with plausible dummy data via ``--demo`` - no radio or Crazyflie required.

Experiments that want a higher-fidelity preview (for example a real occupancy
map built by the SLAM pipeline) pass a custom ``simulator`` callable instead of
relying on the generic generators. Keeping the harness here means the core owns
the reusable plumbing while any experiment-specific simulation stays with the
experiment that needs it.
"""
from __future__ import annotations

import logging
import math
import random
import time
from typing import Callable, Dict, Optional, Sequence, Tuple

from drl.dashboard.app import DashboardServer, Frame

logger = logging.getLogger(__name__)

# A demo simulator takes over the whole preview loop: (server, stop, rate_hz).
Simulator = Callable[[DashboardServer, object, float], None]

_BEAM_BEARINGS = {
    "front": 0.0,
    "left": math.pi / 2,
    "back": math.pi,
    "right": -math.pi / 2,
}


def battery_payload(t: float) -> Dict[str, float]:
    """A slowly sagging battery voltage for the top-bar readout."""
    return {"vbat": round(4.05 - 0.15 * (1.0 - math.cos(t / 90.0)), 2)}


def simulate_ranges(
    x: float,
    y: float,
    yaw: float,
    *,
    room: Tuple[float, float, float, float] = (-2.0, 2.0, -2.0, 2.0),
    max_range: float = 3.5,
) -> Dict[str, Optional[float]]:
    """Ray-cast the four horizontal beams against a rectangular room.

    Shared by experiment demos that move a simulated pose through a space
    (trajectory tracking, SLAM) so their range beams stay geometrically
    consistent with the pose.
    """
    xmin, xmax, ymin, ymax = room
    ranges: Dict[str, Optional[float]] = {}
    for beam, bearing in _BEAM_BEARINGS.items():
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
    ranges["up"] = 1.8
    ranges["down"] = 0.02
    return ranges


class DemoSource:
    """Generate internally consistent synthetic frame payloads over time."""

    GENERIC_FRAMES = ("ranger", "state", "battery", "cmd", "estimate")

    def __init__(self) -> None:
        self._t0 = time.monotonic()
        self._filtered: Dict[str, float] = {}

    def elapsed(self) -> float:
        return time.monotonic() - self._t0

    def ranger(self, t: float) -> Dict[str, Optional[float]]:
        # An object swept toward and away from the front beam; steady elsewhere,
        # with the down beam resting on a desk and an occasional lost right beam.
        front = 0.35 + 0.3 * (0.5 - 0.5 * math.cos(t * 0.6))
        return {
            "front": round(front, 3),
            "back": round(0.9 + 0.05 * math.sin(t * 0.4), 3),
            "left": round(0.5 + 0.08 * math.sin(t * 0.9 + 0.7), 3),
            "right": None if math.sin(t * 0.3) > 0.9 else round(0.6 + 0.05 * math.cos(t * 1.1), 3),
            "up": round(1.8 + 0.03 * math.sin(t * 0.5), 3),
            "down": round(0.02 + 0.003 * abs(math.sin(t * 2.0)), 3),
        }

    def state(self, t: float) -> Dict[str, float]:
        return {
            "x": round(0.1 * math.sin(t * 0.2), 3),
            "y": round(-0.1 * math.cos(t * 0.17), 3),
            "z": round(0.5 + 0.05 * math.sin(t * 0.3), 3),
            "roll": round(2.0 * math.sin(t * 0.6), 2),
            "pitch": round(-1.5 * math.cos(t * 0.5), 2),
            "yaw": round(15.0 * math.sin(t * 0.25), 2),
        }

    def battery(self, t: float) -> Dict[str, float]:
        return battery_payload(t)

    def cmd(self, t: float) -> Dict[str, object]:
        return {
            "vx": round(0.2 * math.sin(t * 0.5), 3),
            "vy": round(0.2 * math.cos(t * 0.4), 3),
            "label": "demo",
        }

    def estimate(self, t: float) -> Dict[str, Dict[str, Optional[float]]]:
        raw_sources: Dict[str, Optional[float]] = dict(self.ranger(t))
        state = self.state(t)
        raw_sources["height"] = state["z"]
        for angle in ("roll", "pitch", "yaw"):
            raw_sources[angle] = state[angle]

        out: Dict[str, Dict[str, Optional[float]]] = {}
        for channel, value in raw_sources.items():
            if value is None:
                out[channel] = {"raw": None, "filtered": self._filtered.get(channel)}
                continue
            # Inject noise into the raw signal so the filtered trace has
            # something to smooth (angles are noisier than ranges).
            sigma = 0.5 if channel in ("roll", "pitch", "yaw") else 0.01
            noisy = value + random.gauss(0.0, sigma)
            prev = self._filtered.get(channel, value)
            filtered = prev + 0.2 * (noisy - prev)
            self._filtered[channel] = filtered
            out[channel] = {"raw": round(noisy, 4), "filtered": round(filtered, 4)}
        return out


def run_demo(
    server: DashboardServer,
    stop,
    *,
    frames: Sequence[str] = ("battery", "state"),
    rate_hz: float = 20.0,
    simulator: Optional[Simulator] = None,
) -> None:
    """Stream synthetic frames until ``stop`` is set.

    If ``simulator`` is provided it takes over the whole loop (used by
    experiments that need higher-fidelity mock data). Otherwise the generic
    per-frame generators emit the requested ``frames``.
    """
    if simulator is not None:
        simulator(server, stop, rate_hz)
        return

    source = DemoSource()
    known = set(DemoSource.GENERIC_FRAMES)
    selected = [f for f in frames if f in known]
    unknown = [f for f in frames if f not in known]
    if unknown:
        logger.warning("No generic demo generator for frame(s): %s", ", ".join(unknown))

    period = 1.0 / rate_hz
    while not stop.is_set():
        t = source.elapsed()
        for name in selected:
            server.publish(Frame(name, getattr(source, name)(t)))
        if stop.wait(period):
            break
