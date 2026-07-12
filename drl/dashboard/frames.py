"""Map raw telemetry samples to dashboard frame payloads."""
from __future__ import annotations

from typing import TYPE_CHECKING, Dict, Optional

if TYPE_CHECKING:
    from drl.dashboard import DashboardServer

Sample = Dict[str, float]


def state_payload(sample: Sample) -> Dict[str, Optional[float]]:
    """Map a raw ``state`` telemetry sample to a dashboard payload (degrees)."""
    return {
        "x": sample.get("stateEstimate.x"),
        "y": sample.get("stateEstimate.y"),
        "z": sample.get("stateEstimate.z"),
        "roll": sample.get("stabilizer.roll"),
        "pitch": sample.get("stabilizer.pitch"),
        "yaw": sample.get("stabilizer.yaw"),
    }


def battery_payload(sample: Sample) -> Dict[str, Optional[float]]:
    """Map a ``battery`` telemetry sample to a dashboard payload (volts)."""
    raw = sample.get("pm.vbat")
    if raw is None:
        return {"vbat": None}
    try:
        return {"vbat": float(raw)}
    except (TypeError, ValueError):
        return {"vbat": None}


def publish_battery(server: "DashboardServer", sample: Sample) -> None:
    """Publish a ``battery`` frame for the top-bar readout, if a voltage is present."""
    payload = battery_payload(sample)
    if payload["vbat"] is None:
        return
    from drl.dashboard import Frame

    server.publish(Frame("battery", payload))


def publish_state(server: "DashboardServer", sample: Sample) -> None:
    """Publish a ``state`` frame from a raw state telemetry sample."""
    from drl.dashboard import Frame

    server.publish(Frame("state", state_payload(sample)))
