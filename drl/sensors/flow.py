"""Normalized access to the Flow deck 2.0 downward range sensor."""
from __future__ import annotations

from dataclasses import dataclass
from threading import Lock
from typing import Dict, Optional

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from drl.sensors._units import mm_to_m

_DOWN_RAW = "range.zrange"


@dataclass(frozen=True)
class FlowReading:
    """Downward range from the Flow deck, in meters (None = out of range)."""

    down: Optional[float] = None

    @classmethod
    def from_sample(cls, sample: Dict[str, float]) -> "FlowReading":
        """Build from a telemetry sample containing raw ``range.zrange`` (mm)."""
        if _DOWN_RAW not in sample:
            return cls()
        return cls(down=mm_to_m(sample[_DOWN_RAW]))

    def as_dict(self) -> Dict[str, Optional[float]]:
        return {"down": self.down}


class FlowStream:
    """Context manager that polls the Flow deck downward range sensor."""

    def __init__(self, scf: SyncCrazyflie, rate_ms: int = 50):
        self._scf = scf
        self._lock = Lock()
        self._down: Optional[float] = None
        self._log_config = LogConfig("flow_down", rate_ms)
        self._log_config.add_variable(_DOWN_RAW, "uint16_t")
        self._log_config.data_received_cb.add_callback(self._on_data)

    def _on_data(self, timestamp: int, data: dict, logconf: LogConfig) -> None:
        with self._lock:
            self._down = mm_to_m(data.get(_DOWN_RAW))

    def reading(self) -> FlowReading:
        with self._lock:
            return FlowReading(down=self._down)

    def start(self) -> "FlowStream":
        self._scf.cf.log.add_config(self._log_config)
        self._log_config.start()
        return self

    def stop(self) -> None:
        try:
            self._log_config.stop()
        except Exception:  # noqa: BLE001
            pass

    def __enter__(self) -> "FlowStream":
        return self.start()

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.stop()
