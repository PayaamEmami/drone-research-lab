"""Shared configuration for Drone Research Lab.

Values can be overridden with environment variables so experiments stay portable
across machines and radios without editing code.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field

# Default radio URI. Matches the cflib examples; override with DRL_URI or
# the standard CFLIB_URI environment variable.
DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E7"

# The Multi-ranger / Flow deck report "no return" as a large value. cflib treats
# >= 8000 mm as out of range; we expose this so the sensor model and UI agree.
RANGER_MAX_MM = 8000
RANGER_MAX_M = RANGER_MAX_MM / 1000.0


def get_uri() -> str:
    """Resolve the Crazyflie URI from the environment, falling back to the default."""
    return (
        os.environ.get("DRL_URI")
        or os.environ.get("CFLIB_URI")
        or DEFAULT_URI
    )


@dataclass
class ServerConfig:
    """Configuration for the dashboard web server."""

    host: str = field(default_factory=lambda: os.environ.get("DRL_HOST", "127.0.0.1"))
    port: int = field(default_factory=lambda: int(os.environ.get("DRL_PORT", "8000")))


@dataclass
class TelemetryConfig:
    """Default logging rates (milliseconds) for telemetry streams."""

    state_rate_ms: int = 50
    ranger_rate_ms: int = 50
    accel_rate_ms: int = 50
    battery_rate_ms: int = 500
