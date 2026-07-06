"""Normalized access to the Multi-ranger deck.

The Crazyflie reports each range as an unsigned integer in millimeters, using a
large sentinel value (>= 8000 mm, see :data:`drl.config.RANGER_MAX_MM`) to
mean "no return / out of range". This module converts those raw values into
meters as floats, with ``None`` for out-of-range beams, so experiments and the
dashboard speak a single, clean unit.

Two entry points:

- :class:`RangerReading` - an immutable snapshot of the six directions, with a
  :meth:`RangerReading.from_sample` builder for telemetry-hub samples.
- :class:`RangerStream` - a context manager around cflib's ``Multiranger`` for
  experiments that want a simple poll interface (e.g. alongside a
  MotionCommander during flight).
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Dict, Optional

from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.multiranger import Multiranger

from drl.config import RANGER_MAX_MM

# Map raw Crazyflie log variable names to RangerReading field names.
_RAW_TO_FIELD = {
    "range.front": "front",
    "range.back": "back",
    "range.left": "left",
    "range.right": "right",
    "range.up": "up",
    "range.zrange": "down",
}


def _mm_to_m(value: Optional[float]) -> Optional[float]:
    """Convert a raw millimeter reading to meters, or None if out of range."""
    if value is None:
        return None
    if value >= RANGER_MAX_MM:
        return None
    return value / 1000.0


@dataclass(frozen=True)
class RangerReading:
    """A snapshot of the six range directions, in meters (None = out of range)."""

    front: Optional[float] = None
    back: Optional[float] = None
    left: Optional[float] = None
    right: Optional[float] = None
    up: Optional[float] = None
    down: Optional[float] = None

    @classmethod
    def from_sample(cls, sample: Dict[str, float]) -> "RangerReading":
        """Build from a telemetry sample containing raw ``range.*`` values (mm)."""
        kwargs: Dict[str, Optional[float]] = {}
        for raw, field in _RAW_TO_FIELD.items():
            if raw in sample:
                kwargs[field] = _mm_to_m(sample[raw])
        return cls(**kwargs)

    def as_dict(self) -> Dict[str, Optional[float]]:
        """Return a plain dict (meters / None), suitable for JSON serialization."""
        return asdict(self)


class RangerStream:
    """Context manager wrapping cflib's Multiranger with normalized readings.

    Example::

        with connect() as link, RangerStream(link.scf) as ranger:
            reading = ranger.reading()
            if reading.front is not None and reading.front < 0.3:
                ...
    """

    def __init__(self, scf: SyncCrazyflie, rate_ms: int = 50):
        self._multiranger = Multiranger(scf, rate_ms=rate_ms, zranger=True)

    def reading(self) -> RangerReading:
        m = self._multiranger
        return RangerReading(
            front=m.front,
            back=m.back,
            left=m.left,
            right=m.right,
            up=m.up,
            down=m.down,
        )

    def start(self) -> "RangerStream":
        self._multiranger.start()
        return self

    def stop(self) -> None:
        self._multiranger.stop()

    def __enter__(self) -> "RangerStream":
        return self.start()

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.stop()
