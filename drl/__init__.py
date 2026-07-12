"""Drone Research Lab: a reusable Crazyflie flight and sensing research platform.

The package is split into a small reusable core and a web dashboard:

- ``drl.config``      - shared configuration (URI, log rates, defaults).
- ``drl.connection``  - connect/arm helpers around cflib's SyncCrazyflie.
- ``drl.telemetry``   - LogConfig builders + a subscription hub with ring buffers.
- ``drl.sensors``     - per-deck adapters (Multi-ranger, Flow) plus a combined ``Sensors`` snapshot.
- ``drl.motion``      - takeoff, landing, and world-frame velocity setpoints.
- ``drl.recording``   - stream arbitrary telemetry dicts to CSV for offline analysis.
- ``drl.dashboard``   - FastAPI + websocket server, frame payloads, and publishers.
- ``drl.plots``       - matplotlib helpers for static, publishable figures.
- ``drl.signals``     - Ctrl+C / SIGTERM shutdown coordination.
- ``drl.timing``      - monotonic clock helpers for control loops.
- ``drl.cli``         - shared argparse flags for experiment runners.
- ``drl.session``     - :class:`~drl.session.ExperimentSession` lifecycle helper.

Experiments live outside the package, under ``experiments/``, and build on this core.
"""
from __future__ import annotations

__version__ = "0.1.0"

__all__ = ["__version__"]
