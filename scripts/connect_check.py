"""Sanity-check the radio link and report attached decks + a battery reading.

Run this first whenever you sit down to work. It confirms the Crazyradio, the
URI, and the decks are all healthy before you try an experiment.

Usage (from the repo root, after ``pip install -e .``)::

    python -m scripts.connect_check
    DRL_URI=radio://0/80/2M/E7E7E7E7E7 python -m scripts.connect_check
"""
from __future__ import annotations

import logging

from drl.config import get_uri
from drl.connection import (
    connect,
    crazyradio2_bootloader_message,
    crazyradio_usb_status,
)

logging.basicConfig(level=logging.INFO, format="%(message)s")


def main() -> int:
    uri = get_uri()
    radio_state = crazyradio_usb_status()
    if radio_state == "bootloader":
        print("Crazyradio 2.0 is in bootloader mode — radio firmware is not installed yet.")
        print(crazyradio2_bootloader_message())
        return 1

    print(f"Connecting to {uri} ...")
    try:
        with connect(uri) as link:
            print("Connected.")

            decks = link.decks()
            if decks:
                for name, present in sorted(decks.items()):
                    mark = "ok" if present else "MISSING"
                    print(f"  deck {name:<12} {mark}")
            else:
                print("  (no known decks reported yet)")

            try:
                vbat = link.cf.param.get_value("pm.vbat")
                print(f"  battery: {float(vbat):.2f} V")
            except (KeyError, AttributeError, TypeError, ValueError):
                print("  battery: unavailable")

        print("Link closed cleanly. You're good to go.")
        return 0
    except Exception as exc:  # noqa: BLE001 - top-level CLI guard
        print(f"Connection failed: {exc}")
        message = str(exc)
        if "No backend available" in message:
            print(
                "Tips: Python could not load a USB library. On Windows, reinstall the "
                "venv and retry. If this persists, check that Zadig bound the Crazyradio "
                "to WinUSB or libusbK."
            )
        elif "Cannot find a Crazyradio Dongle" in message:
            if crazyradio_usb_status() == "bootloader":
                print(crazyradio2_bootloader_message())
            else:
                print(
                    "Tips: the USB library loaded, but no Crazyradio was detected at "
                    "1915:7777. For Crazyradio 2.0, flash radio firmware first (see "
                    "README). Then in Zadig use Options → List All Devices, select the "
                    "1915:7777 entry, and install WinUSB or libusbK."
                )
        else:
            print("Tips: is the Crazyradio plugged in? Is the drone on? Is the URI correct?")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
