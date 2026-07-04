"""Shared helpers for the experiment runner scripts.

Experiments live outside the installable package so they read like standalone
demos. This module centralizes the few things they all need: a clean Ctrl+C
handler, and mapping telemetry samples into dashboard frame payloads.

Run experiments as modules from the repo root (e.g.
``python -m experiments.reactive_flight.run``) so ``import drl`` and
``import experiments.common`` resolve without any path manipulation. Install the
core in editable mode first with ``pip install -e .``.
"""
from __future__ import annotations

import math
import signal
from threading import Event
from typing import Dict, Optional


def install_stop_handler() -> Event:
    """Return an Event that is set on Ctrl+C / SIGTERM for clean shutdown."""
    stop = Event()

    def _handler(signum, frame):  # noqa: ANN001
        print("\nStopping (signal received)...")
        stop.set()

    signal.signal(signal.SIGINT, _handler)
    try:
        signal.signal(signal.SIGTERM, _handler)
    except (ValueError, AttributeError):
        pass  # SIGTERM may be unavailable on some platforms
    return stop


def state_payload(sample: Dict[str, float]) -> Dict[str, Optional[float]]:
    """Map a raw ``state`` telemetry sample to a dashboard payload (degrees)."""
    return {
        "x": sample.get("stateEstimate.x"),
        "y": sample.get("stateEstimate.y"),
        "z": sample.get("stateEstimate.z"),
        "roll": sample.get("stabilizer.roll"),
        "pitch": sample.get("stabilizer.pitch"),
        "yaw": sample.get("stabilizer.yaw"),
    }


def yaw_radians(sample: Dict[str, float]) -> float:
    """Extract yaw from a state sample and convert degrees -> radians."""
    return math.radians(sample.get("stabilizer.yaw", 0.0) or 0.0)
