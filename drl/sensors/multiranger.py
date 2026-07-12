"""Normalized access to the Multi-ranger deck (five TOF beams)."""
from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Dict, Optional

from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.multiranger import Multiranger

from drl.sensors._units import mm_to_m

_RAW_TO_FIELD = {
    "range.front": "front",
    "range.back": "back",
    "range.left": "left",
    "range.right": "right",
    "range.up": "up",
}


@dataclass(frozen=True)
class MultirangerReading:
    """A snapshot of the five Multi-ranger beams, in meters (None = out of range)."""

    front: Optional[float] = None
    back: Optional[float] = None
    left: Optional[float] = None
    right: Optional[float] = None
    up: Optional[float] = None

    @classmethod
    def from_sample(cls, sample: Dict[str, float]) -> "MultirangerReading":
        """Build from a telemetry sample containing raw ``range.*`` values (mm)."""
        kwargs: Dict[str, Optional[float]] = {}
        for raw, field in _RAW_TO_FIELD.items():
            if raw in sample:
                kwargs[field] = mm_to_m(sample[raw])
        return cls(**kwargs)

    def as_dict(self) -> Dict[str, Optional[float]]:
        """Return a plain dict (meters / None), suitable for JSON serialization."""
        return asdict(self)


class MultirangerStream:
    """Context manager wrapping cflib's ``Multiranger`` with normalized readings."""

    def __init__(self, scf: SyncCrazyflie, rate_ms: int = 50):
        self._multiranger = Multiranger(scf, rate_ms=rate_ms, zranger=False)

    def reading(self) -> MultirangerReading:
        m = self._multiranger
        return MultirangerReading(
            front=m.front,
            back=m.back,
            left=m.left,
            right=m.right,
            up=m.up,
        )

    def start(self) -> "MultirangerStream":
        self._multiranger.start()
        return self

    def stop(self) -> None:
        self._multiranger.stop()

    def __enter__(self) -> "MultirangerStream":
        return self.start()

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.stop()
