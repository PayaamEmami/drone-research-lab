"""Offline plotting helpers for recorded Drone Research Lab data.

These are convenience functions for turning recordings into publishable figures.
``matplotlib`` (and ``numpy``) are required; install the ``analysis`` extra::

    pip install -e ".[analysis]"

All functions import matplotlib lazily so the core package has no hard
dependency on it.
"""
from __future__ import annotations

import csv
from pathlib import Path
from typing import Iterable, Optional

import numpy as np

_DEFAULT_RANGE_COLS = ("front", "back", "left", "right")


def plot_ranges_csv(
    csv_path: str | Path,
    columns: Iterable[str] = _DEFAULT_RANGE_COLS,
    x_col: str = "elapsed_s",
    out_path: Optional[str | Path] = None,
):
    """Plot range columns from a proximity/reactive CSV against time.

    Returns the matplotlib Figure. If ``out_path`` is given, also saves it.
    """
    import matplotlib.pyplot as plt

    rows = list(csv.DictReader(open(csv_path, newline="", encoding="utf-8")))
    if not rows:
        raise ValueError(f"No rows in {csv_path}")

    def col(name: str) -> list[Optional[float]]:
        out: list[Optional[float]] = []
        for r in rows:
            v = r.get(name, "")
            out.append(float(v) if v not in ("", "None", None) else np.nan)
        return out

    x = col(x_col)
    fig, ax = plt.subplots(figsize=(9, 4))
    for c in columns:
        ax.plot(x, col(c), label=c, linewidth=1.5)
    ax.set_xlabel(x_col)
    ax.set_ylabel("distance (m)")
    ax.set_title(f"Multi-ranger distances - {Path(csv_path).name}")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    if out_path:
        fig.savefig(out_path, dpi=150)
    return fig


def plot_occupancy_npz(npz_path: str | Path, out_path: Optional[str | Path] = None):
    """Render an occupancy grid saved by ``OccupancyGrid.save_npz`` to a figure."""
    import matplotlib.pyplot as plt

    data = np.load(npz_path)
    logodds = data["logodds"]
    observed = data["observed"]
    prob = 1.0 - 1.0 / (1.0 + np.exp(logodds))
    display = np.where(observed, prob, np.nan)

    fig, ax = plt.subplots(figsize=(6, 6))
    # origin="lower" so +y points up, matching the live dashboard.
    im = ax.imshow(display, origin="lower", cmap="magma", vmin=0.0, vmax=1.0)
    ax.set_title(f"Occupancy grid - {Path(npz_path).name}")
    fig.colorbar(im, ax=ax, label="P(occupied)")
    fig.tight_layout()
    if out_path:
        fig.savefig(out_path, dpi=150)
    return fig
