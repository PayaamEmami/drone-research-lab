"""Shared helpers for the experiment runner scripts.

Experiments live outside the installable package so they read like standalone
demos. This module centralizes helpers used across multiple experiments:

- :func:`install_stop_handler` - clean Ctrl+C / SIGTERM shutdown.
- :func:`state_payload`, :func:`yaw_radians` - map telemetry to dashboard frames.
- :func:`monotonic_elapsed` - monotonic timing for control loops.
- :class:`MapPublisher` - rate-limited background publishing of heavy frames.
- :func:`publish_battery` - publish a dashboard ``battery`` frame from a log sample.

Run experiments as modules from the repo root (e.g.
``python -m experiments.state_estimation.run``) so ``import drl`` and
``import experiments.common`` resolve without any path manipulation. Install the
core in editable mode first with ``pip install -e .``.
"""
from __future__ import annotations

import math
import signal
import time
from threading import Event, Lock, Thread
from typing import Callable, Dict, Optional, Tuple, TYPE_CHECKING

if TYPE_CHECKING:
    from drl.dashboard import DashboardServer


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


def battery_payload(sample: Dict[str, float]) -> Dict[str, Optional[float]]:
    """Map a ``battery`` telemetry sample to a dashboard payload (volts)."""
    raw = sample.get("pm.vbat")
    if raw is None:
        return {"vbat": None}
    try:
        return {"vbat": float(raw)}
    except (TypeError, ValueError):
        return {"vbat": None}


def publish_battery(server: "DashboardServer", sample: Dict[str, float]) -> None:
    """Publish ``battery`` frame for the top-bar readout, if a voltage is present."""
    payload = battery_payload(sample)
    if payload["vbat"] is None:
        return
    from drl.dashboard import Frame

    server.publish(Frame("battery", payload))


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


class MapPublisher:
    """Rate-limited background publisher for heavy dashboard frames.

    The SLAM occupancy map is expensive to serialize, so publishing it directly
    from a telemetry callback would stall sensor handling. This thread snapshots
    the latest payload and broadcasts it at a fixed rate instead.

    :param publish: callback that emits one frame (e.g. ``server.publish``).
    :param build: callable returning the current frame payload object, or None
        to skip this cycle.
    :param hz: broadcast rate.
    """

    def __init__(self, publish: Callable[[object], None],
                 build: Callable[[], Optional[object]], hz: float = 5.0):
        self._publish = publish
        self._build = build
        self._period = 1.0 / hz if hz > 0 else 0.2
        self._stop = Event()
        self._lock = Lock()
        self._thread: Optional[Thread] = None

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                with self._lock:
                    frame = self._build()
                if frame is not None:
                    self._publish(frame)
            except Exception:  # noqa: BLE001 - a bad build must not kill the thread
                pass
            self._stop.wait(self._period)

    def start(self) -> "MapPublisher":
        self._thread = Thread(target=self._run, name="map-publisher", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def __enter__(self) -> "MapPublisher":
        return self.start()

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.stop()
