"""Telemetry logging on top of cflib's LogConfig.

This provides:

- :func:`make_state_config`, :func:`make_ranger_config`, :func:`make_battery_config`,
  :func:`make_accel_config` - small builders for the LogConfig blocks experiments
  commonly need.
- :class:`TelemetryHub` - registers one or more LogConfigs, fans incoming data
  out to subscriber callbacks, and keeps a short ring buffer of the latest
  samples per block for polling-style consumers.

The Crazyflie logging framework streams a fixed set of variables at a fixed
rate; see the cflib example at
``crazyflie-lib-python/examples/logging/basiclog.py`` for the underlying pattern.
"""
from __future__ import annotations

import logging
import time
from collections import deque
from threading import Lock
from typing import Callable, Deque, Dict, List, Optional

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from drl.config import TelemetryConfig

logger = logging.getLogger(__name__)

# A telemetry sample: the variable name -> value mapping for one block, plus the
# Crazyflie timestamp (ms) and a host wall-clock time (s).
Sample = Dict[str, float]
SubscriberCallback = Callable[[str, int, Sample], None]


def make_state_config(rate_ms: int = 50) -> LogConfig:
    """Position + attitude estimate from the onboard Kalman filter."""
    cfg = LogConfig(name="state", period_in_ms=rate_ms)
    cfg.add_variable("stateEstimate.x", "float")
    cfg.add_variable("stateEstimate.y", "float")
    cfg.add_variable("stateEstimate.z", "float")
    cfg.add_variable("stabilizer.roll", "float")
    cfg.add_variable("stabilizer.pitch", "float")
    cfg.add_variable("stabilizer.yaw", "float")
    return cfg


def make_ranger_config(rate_ms: int = 50) -> LogConfig:
    """The five Multi-ranger beams plus the down-facing z-range (mm)."""
    cfg = LogConfig(name="ranger", period_in_ms=rate_ms)
    cfg.add_variable("range.front", "uint16_t")
    cfg.add_variable("range.back", "uint16_t")
    cfg.add_variable("range.left", "uint16_t")
    cfg.add_variable("range.right", "uint16_t")
    cfg.add_variable("range.up", "uint16_t")
    cfg.add_variable("range.zrange", "uint16_t")
    return cfg


def make_battery_config(rate_ms: int = 500) -> LogConfig:
    """Battery voltage, sampled slowly."""
    cfg = LogConfig(name="battery", period_in_ms=rate_ms)
    cfg.add_variable("pm.vbat", "FP16")
    return cfg


def make_accel_config(rate_ms: int = 50) -> LogConfig:
    """Raw IMU accelerometer in units of g (``acc.z`` is ~1.0 at rest on a level surface)."""
    cfg = LogConfig(name="accel", period_in_ms=rate_ms)
    cfg.add_variable("acc.x", "float")
    cfg.add_variable("acc.y", "float")
    cfg.add_variable("acc.z", "float")
    return cfg


class TelemetryHub:
    """Manage a set of LogConfigs and fan their data out to subscribers.

    Subscribers receive ``(block_name, timestamp_ms, sample_dict)``. The most
    recent sample for each block is also retained and accessible via
    :meth:`latest`, and a bounded history is kept for offline-style polling.
    """

    def __init__(self, scf: SyncCrazyflie, history: int = 256):
        self._scf = scf
        self._configs: Dict[str, LogConfig] = {}
        self._subscribers: List[SubscriberCallback] = []
        self._latest: Dict[str, Sample] = {}
        self._history: Dict[str, Deque[Sample]] = {}
        self._history_len = history
        self._lock = Lock()
        self._started = False

    def add_config(self, config: LogConfig) -> "TelemetryHub":
        """Register a LogConfig with the Crazyflie and wire up its callbacks."""
        name = config.name
        self._scf.cf.log.add_config(config)
        config.data_received_cb.add_callback(self._on_data)
        config.error_cb.add_callback(self._on_error)
        self._configs[name] = config
        self._history[name] = deque(maxlen=self._history_len)
        return self

    def subscribe(self, callback: SubscriberCallback) -> None:
        """Register a callback invoked for every incoming sample (any block)."""
        self._subscribers.append(callback)

    def _on_data(self, timestamp: int, data: dict, logconf: LogConfig) -> None:
        sample: Sample = dict(data)
        sample["_host_t"] = time.time()
        name = logconf.name
        with self._lock:
            self._latest[name] = sample
            self._history[name].append(sample)
        for cb in self._subscribers:
            try:
                cb(name, timestamp, sample)
            except Exception:  # noqa: BLE001 - a bad subscriber must not kill logging
                logger.exception("Telemetry subscriber raised")

    def _on_error(self, logconf: LogConfig, msg: str) -> None:
        logger.error("Logging error on %s: %s", logconf.name, msg)

    def latest(self, block: str) -> Optional[Sample]:
        """Return the most recent sample for a block, or None if none yet."""
        with self._lock:
            sample = self._latest.get(block)
            return dict(sample) if sample is not None else None

    def start(self) -> "TelemetryHub":
        for config in self._configs.values():
            config.start()
        self._started = True
        return self

    def stop(self) -> None:
        if not self._started:
            return
        for config in self._configs.values():
            try:
                config.stop()
            except Exception:  # noqa: BLE001
                logger.debug("Stopping log config failed", exc_info=True)
        self._started = False

    def __enter__(self) -> "TelemetryHub":
        return self.start()

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.stop()
