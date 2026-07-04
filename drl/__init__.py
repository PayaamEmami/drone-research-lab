"""Drone Research Lab: a reusable Crazyflie flight and sensing research platform.

The package is split into a small reusable core and a web dashboard:

- ``drl.config``      - shared configuration (URI, log rates, defaults).
- ``drl.connection``  - connect/arm helpers around cflib's SyncCrazyflie.
- ``drl.telemetry``   - LogConfig builders + a subscription hub with ring buffers.
- ``drl.sensors``     - normalized sensor adapters (e.g. the Multi-ranger deck).
- ``drl.recording``   - stream arbitrary telemetry dicts to CSV for offline analysis.
- ``drl.dashboard``   - FastAPI + websocket server and browser UI that streams frames.
- ``drl.viz``         - matplotlib helpers for static, publishable figures.

Experiments live outside the package, under ``experiments/``, and build on this core.
"""
from __future__ import annotations

__version__ = "0.1.0"

__all__ = ["__version__"]
