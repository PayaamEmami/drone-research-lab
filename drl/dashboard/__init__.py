"""FastAPI + websocket dashboard server and its browser UI assets."""
from __future__ import annotations

from drl.dashboard.app import DashboardServer, Frame
from drl.dashboard.frames import (
    battery_payload,
    publish_battery,
    publish_state,
    state_payload,
)
from drl.dashboard.publisher import MapPublisher

__all__ = [
    "DashboardServer",
    "Frame",
    "MapPublisher",
    "battery_payload",
    "publish_battery",
    "publish_state",
    "state_payload",
]
