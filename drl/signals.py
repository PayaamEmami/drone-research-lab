"""OS signal handling for graceful experiment shutdown."""
from __future__ import annotations

import signal
from threading import Event


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
