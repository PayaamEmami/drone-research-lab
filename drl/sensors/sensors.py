"""Combined sensor snapshot for consumers that need every deck at once."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from drl.sensors.flow import FlowReading, FlowStream
from drl.sensors.multiranger import MultirangerReading, MultirangerStream
from drl.telemetry import TelemetryHub

Sample = Dict[str, float]


@dataclass(frozen=True)
class Sensors:
    """A snapshot across the Multi-ranger and Flow decks."""

    multiranger: MultirangerReading
    flow: FlowReading

    @classmethod
    def from_samples(
        cls,
        multiranger: Optional[Sample] = None,
        flow: Optional[Sample] = None,
    ) -> "Sensors":
        """Build from raw telemetry samples (mm) for one or both decks."""
        return cls(
            multiranger=MultirangerReading.from_sample(multiranger or {}),
            flow=FlowReading.from_sample(flow or {}),
        )

    @classmethod
    def from_hub(cls, hub: TelemetryHub) -> "Sensors":
        """Build from the latest samples already buffered on a :class:`TelemetryHub`."""
        return cls.from_samples(hub.latest("multiranger"), hub.latest("flow"))

    def as_dict(self) -> Dict[str, Optional[float]]:
        """Six-direction ranges in meters, for dashboard frames and SLAM."""
        ranges = self.multiranger.as_dict()
        ranges["down"] = self.flow.down
        return ranges


class SensorsStream:
    """Poll every deck sensor through one interface."""

    def __init__(self, scf: SyncCrazyflie, rate_ms: int = 50):
        self._multiranger = MultirangerStream(scf, rate_ms=rate_ms)
        self._flow = FlowStream(scf, rate_ms=rate_ms)

    def reading(self) -> Sensors:
        return Sensors(self._multiranger.reading(), self._flow.reading())

    def start(self) -> "SensorsStream":
        self._multiranger.start()
        self._flow.start()
        return self

    def stop(self) -> None:
        self._flow.stop()
        self._multiranger.stop()

    def __enter__(self) -> "SensorsStream":
        return self.start()

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.stop()
