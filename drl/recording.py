"""Record arbitrary telemetry samples to timestamped CSV files.

The recorder is schema-flexible: it discovers columns from the first row it sees
and writes a header then. Rows with extra keys are accommodated; re-writing is
*not* attempted (CSV has a fixed header), so pass a consistent set of keys per
recorder, or supply ``fieldnames`` up front.

Files land in ``data/`` (gitignored) named ``<prefix>_<YYYYmmdd_HHMMSS>.csv``.

Example::

    with CsvRecorder("proximity") as rec:
        rec.write({"t": 0.0, "front": 0.42, "left": None})
"""
from __future__ import annotations

import csv
import logging
import time
from datetime import datetime
from pathlib import Path
from threading import Lock
from typing import Dict, Iterable, Optional

logger = logging.getLogger(__name__)

DEFAULT_DATA_DIR = Path("data")


class CsvRecorder:
    """Append-only CSV recorder with a discovered or supplied header."""

    def __init__(
        self,
        prefix: str,
        *,
        data_dir: Path | str = DEFAULT_DATA_DIR,
        fieldnames: Optional[Iterable[str]] = None,
    ):
        self._data_dir = Path(data_dir)
        self._data_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = self._data_dir / f"{prefix}_{stamp}.csv"
        self._fieldnames: Optional[list[str]] = list(fieldnames) if fieldnames else None
        self._file = None
        self._writer: Optional[csv.DictWriter] = None
        self._lock = Lock()
        self._t0 = time.time()
        self._open()

    def _open(self) -> None:
        self._file = open(self.path, "w", newline="", encoding="utf-8")
        if self._fieldnames is not None:
            self._init_writer(self._fieldnames)
        logger.info("Recording to %s", self.path)

    def _init_writer(self, fieldnames: list[str]) -> None:
        self._fieldnames = fieldnames
        self._writer = csv.DictWriter(self._file, fieldnames=fieldnames)
        self._writer.writeheader()

    def write(self, row: Dict[str, object], *, add_elapsed: bool = True) -> None:
        """Write one row. Header is created from the first row if not supplied.

        :param add_elapsed: If True, inject an ``elapsed_s`` column measured from
            recorder creation (handy for plotting without parsing timestamps).
        """
        out = dict(row)
        if add_elapsed and "elapsed_s" not in out:
            out["elapsed_s"] = round(time.time() - self._t0, 4)
        with self._lock:
            if self._writer is None:
                self._init_writer(list(out.keys()))
            # Ignore keys not in the established header to keep CSV well-formed.
            filtered = {k: out.get(k) for k in self._fieldnames}
            self._writer.writerow(filtered)

    def close(self) -> None:
        with self._lock:
            if self._file is not None:
                self._file.flush()
                self._file.close()
                self._file = None
                self._writer = None

    def __enter__(self) -> "CsvRecorder":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()
