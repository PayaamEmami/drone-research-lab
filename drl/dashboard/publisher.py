"""Rate-limited background publishing for heavy dashboard frames."""
from __future__ import annotations

from threading import Event, Lock, Thread
from typing import Callable, Optional


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

    def __init__(
        self,
        publish: Callable[[object], None],
        build: Callable[[], Optional[object]],
        hz: float = 5.0,
    ):
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
