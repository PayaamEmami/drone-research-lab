"""Connection helpers around cflib's SyncCrazyflie.

This module centralizes the boilerplate every experiment needs: initializing the
CRTP drivers, opening the radio link, arming the platform, optionally resetting
the Kalman estimator, and detecting which expansion decks are attached.

Typical use::

    from drl.connection import connect

    with connect(arm=True) as link:
        print(link.decks())
        # link.cf is the underlying cflib Crazyflie instance
        # link.scf is the SyncCrazyflie wrapper
"""
from __future__ import annotations

import logging
import os
import time
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Iterator, Optional

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.reset_estimator import reset_estimator

from drl.config import get_uri

logger = logging.getLogger(__name__)

# Parameter names that report the presence of the decks this project cares about.
# Values are "1" (present) / "0" (absent) once params have been fetched.
DECK_PARAMS = {
    "flow2": "deck.bcFlow2",
    "multiranger": "deck.bcMultiranger",
}

_DRIVERS_INITIALIZED = False
_USB_BACKEND_PATCHED = False

# Crazyradio 2.0 UF2 bootloader (not the radio firmware cflib expects).
CR2_BOOTLOADER_VID = 0x35F0
CR2_BOOTLOADER_PID = 0xBAD2
CRADIO_VID = 0x1915
CRADIO_PID = 0x7777
CR2_FIRMWARE_URL = (
    "https://github.com/bitcraze/crazyradio2-firmware/releases/latest"
)


def _usb_backend():
    import libusb_package

    return libusb_package.get_libusb1_backend()


def crazyradio_usb_status() -> str:
    """Return ``bootloader``, ``ready``, or ``missing`` for the Crazyradio USB state."""
    import usb.core

    backend = _usb_backend()
    if usb.core.find(
        idVendor=CR2_BOOTLOADER_VID,
        idProduct=CR2_BOOTLOADER_PID,
        backend=backend,
    ):
        return "bootloader"
    if usb.core.find(idVendor=CRADIO_VID, idProduct=CRADIO_PID, backend=backend):
        return "ready"
    return "missing"


def crazyradio2_bootloader_message() -> str:
    """Explain what to do when a Crazyradio 2.0 is stuck in UF2 bootloader mode."""
    return (
        "Detected a Crazyradio 2.0 in UF2 bootloader mode (USB 35f0:bad2), not radio "
        "firmware (1915:7777). cflib cannot use the dongle until firmware is installed.\n"
        "\n"
        "1. Unplug the dongle. If you previously ran Zadig on this device, open Device "
        "Manager, find Crazyradio 2.0, and uninstall the libusb/WinUSB driver so Windows "
        "can mount the UF2 drive again.\n"
        "2. Hold the dongle button, plug into USB, and release. A drive named "
        "Crazyradio2 should appear in File Explorer.\n"
        "3. Download the latest crazyradio2-*.uf2 from:\n"
        f"   {CR2_FIRMWARE_URL}\n"
        "4. Drag the .uf2 file onto the Crazyradio2 drive. The dongle reboots when done.\n"
        "5. Plug in normally (no button). In Zadig, bind the device that shows USB ID "
        "1915:7777 to WinUSB or libusbK — not the 35f0:bad2 bootloader entry.\n"
        "6. Run connect_check again."
    )


def _patch_windows_usb_backend() -> None:
    """Fall back to libusb1 on Windows when cflib's libusb0 backend is missing.

    cflib 0.1.x selects ``usb.backend.libusb0`` on Windows, but modern installs
  only ship libusb-1.0 via ``libusb-package``. Without this patch, connection
    attempts fail with ``usb.core.NoBackendError`` before the dongle is scanned.
    """
    global _USB_BACKEND_PATCHED
    if _USB_BACKEND_PATCHED or os.name != "nt":
        return

    import usb.backend.libusb0 as libusb0

    if libusb0.get_backend() is not None:
        _USB_BACKEND_PATCHED = True
        return

    import libusb_package
    import usb.core
    from cflib.drivers import cfusb, crazyradio

    backend = libusb_package.get_libusb1_backend()
    logger.debug("Using libusb1 USB backend on Windows (libusb0 unavailable)")

    def _find_crazyradio_devices(serial=None):
        ret = []
        devices = usb.core.find(
            idVendor=0x1915,
            idProduct=0x7777,
            find_all=True,
            backend=backend,
        )
        if devices:
            for device in devices:
                if serial is not None and serial == device.serial_number:
                    return device
                ret.append(device)
        return ret

    def _find_cfusb_devices():
        ret = []
        devices = usb.core.find(
            idVendor=cfusb.USB_VID,
            idProduct=cfusb.USB_PID,
            find_all=True,
            backend=backend,
        )
        if devices:
            for device in devices:
                if device.manufacturer == "Bitcraze AB":
                    ret.append(device)
        return ret

    crazyradio._find_devices = _find_crazyradio_devices
    cfusb._find_devices = _find_cfusb_devices
    _USB_BACKEND_PATCHED = True


def init_drivers() -> None:
    """Initialize the low-level CRTP drivers exactly once per process."""
    global _DRIVERS_INITIALIZED
    if not _DRIVERS_INITIALIZED:
        _patch_windows_usb_backend()
        cflib.crtp.init_drivers()
        _DRIVERS_INITIALIZED = True


@dataclass
class Link:
    """A live connection to a Crazyflie plus convenience accessors."""

    scf: SyncCrazyflie

    @property
    def cf(self) -> Crazyflie:
        return self.scf.cf

    def decks(self) -> dict[str, bool]:
        """Return which known decks are attached, e.g. {"flow2": True, ...}.

        Returns an empty dict if parameters have not been populated yet.
        """
        result: dict[str, bool] = {}
        for name, param in DECK_PARAMS.items():
            try:
                value = self.cf.param.get_value(param)
            except (KeyError, AttributeError):
                continue
            if value is not None:
                result[name] = str(value) != "0"
        return result

    def require_decks(self, *names: str) -> None:
        """Raise RuntimeError if any of the named decks is not attached."""
        decks = self.decks()
        missing = [n for n in names if not decks.get(n, False)]
        if missing:
            raise RuntimeError(
                f"Required deck(s) not detected: {', '.join(missing)}. "
                f"Detected: {decks or 'none yet'}"
            )

    def arm(self, armed: bool = True, settle_s: float = 1.0) -> None:
        """Send an arming request to the platform (required before flight)."""
        self.cf.supervisor.send_arming_request(armed)
        if settle_s:
            time.sleep(settle_s)

    def reset_estimator(self) -> None:
        """Reset the Kalman estimator and wait for it to converge."""
        reset_estimator(self.scf)


@contextmanager
def connect(
    uri: Optional[str] = None,
    *,
    arm: bool = False,
    reset_estimator_on_connect: bool = False,
    rw_cache: str = "./cache",
) -> Iterator[Link]:
    """Open a connection to a Crazyflie as a context manager.

    :param uri: Radio URI; defaults to :func:`drl.config.get_uri`.
    :param arm: If True, arm the platform after connecting (needed to fly).
    :param reset_estimator_on_connect: If True, reset the Kalman estimator and
        wait for convergence before yielding. Recommended before autonomous flight.
    :param rw_cache: Directory used by cflib to cache the parameter/log TOCs.
    """
    init_drivers()
    resolved = uri or get_uri()
    logger.info("Connecting to %s", resolved)

    with SyncCrazyflie(resolved, cf=Crazyflie(rw_cache=rw_cache)) as scf:
        link = Link(scf=scf)
        if reset_estimator_on_connect:
            link.reset_estimator()
        if arm:
            link.arm(True)
        try:
            yield link
        finally:
            if arm:
                # Best-effort disarm; ignore errors during teardown.
                try:
                    link.arm(False, settle_s=0.0)
                except Exception:  # noqa: BLE001 - teardown must not raise
                    logger.debug("Disarm during teardown failed", exc_info=True)
