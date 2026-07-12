"""Shared argparse flags for experiment runner scripts."""
from __future__ import annotations

import argparse


def add_experiment_args(
    parser: argparse.ArgumentParser,
    *,
    record: bool = False,
) -> None:
    """Add CLI flags shared across experiment runners."""
    parser.add_argument("--uri", default=None, help="Crazyflie radio URI override")
    parser.add_argument("--port", type=int, default=8000, help="dashboard port")
    parser.add_argument("--no-browser", action="store_true", help="don't auto-open browser")
    parser.add_argument(
        "--demo",
        action="store_true",
        help="preview the dashboard with synthetic data; no hardware or radio needed",
    )
    parser.add_argument(
        "--demo-rate",
        type=float,
        default=20.0,
        help="demo data update rate in Hz (used with --demo)",
    )
    if record:
        parser.add_argument("--no-record", action="store_true", help="disable CSV recording")
