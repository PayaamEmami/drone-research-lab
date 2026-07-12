"""Lifecycle helper for one connected experiment run."""
from __future__ import annotations

from contextlib import ExitStack
from dataclasses import dataclass
from threading import Event
from typing import Callable, Iterable, Optional, Sequence

from drl.config import ServerConfig
from drl.connection import Link, connect
from drl.dashboard import DashboardServer, Frame
from drl.recording import CsvRecorder
from drl.signals import install_stop_handler
from drl.telemetry import TelemetryHub


@dataclass
class ExperimentSession:
    """One live experiment session: stop event, dashboard, optional link and hub.

    Use as a context manager to guarantee dashboard and recorder teardown::

        with ExperimentSession("basic flight", port=8000, connect=True, arm=True) as sess:
            sess.hub.add_config(make_state_config())
            sess.hub.attach_dashboard(sess.server, auto=["battery", "state"])
            with sess.hub:
                ...
    """

    stop: Event
    server: DashboardServer
    link: Optional[Link] = None
    hub: Optional[TelemetryHub] = None
    recorder: Optional[CsvRecorder] = None

    def __init__(
        self,
        name: str,
        *,
        port: int = 8000,
        host: str = "127.0.0.1",
        uri: Optional[str] = None,
        open_browser: bool = True,
        connect_drone: bool = True,
        arm: bool = False,
        reset_estimator_on_connect: Optional[bool] = None,
        record: Optional[str] = None,
        record_fieldnames: Optional[Iterable[str]] = None,
        demo: bool = False,
        demo_frames: Sequence[str] = ("battery", "state"),
        demo_rate_hz: float = 20.0,
        demo_simulator: Optional[Callable] = None,
    ):
        self._name = name
        self._port = port
        self._host = host
        self._uri = uri
        self._open_browser = open_browser
        self._connect_drone = connect_drone
        self._arm = arm
        self._reset_estimator = reset_estimator_on_connect
        self._record_prefix = record
        self._record_fieldnames = record_fieldnames
        self._demo_frames = demo_frames
        self._demo_rate_hz = demo_rate_hz
        self._demo_simulator = demo_simulator
        self._stack: Optional[ExitStack] = None

        self.demo = demo
        self.stop = Event()
        self.server = DashboardServer(ServerConfig(host=host, port=port))
        self.link = None
        self.hub = None
        self.recorder = None

    def __enter__(self) -> "ExperimentSession":
        self._stack = ExitStack()
        self.stop = install_stop_handler()

        self.server = DashboardServer(ServerConfig(host=self._host, port=self._port))
        self.server.start(open_browser=self._open_browser)
        label = f"{self._name} [demo]" if self.demo else self._name
        self.server.publish(Frame("meta", {"experiment": label}))

        # Demo mode previews the dashboard with synthetic data: no radio link,
        # no telemetry hub, and no recording.
        if self.demo:
            return self

        if self._connect_drone:
            reset = self._arm if self._reset_estimator is None else self._reset_estimator
            self.link = self._stack.enter_context(
                connect(
                    self._uri,
                    arm=self._arm,
                    reset_estimator_on_connect=reset,
                )
            )
            self.hub = TelemetryHub(self.link.scf)

        if self._record_prefix is not None:
            self.recorder = CsvRecorder(
                self._record_prefix,
                fieldnames=self._record_fieldnames,
            )

        return self

    def run_demo(self) -> None:
        """Stream synthetic data to the dashboard until the stop event is set.

        Only valid in demo mode. Uses the experiment's ``demo_simulator`` when
        one was supplied, otherwise the generic per-frame generators for
        ``demo_frames``.
        """
        from drl.dashboard.demo import run_demo as _run_demo

        _run_demo(
            self.server,
            self.stop,
            frames=self._demo_frames,
            rate_hz=self._demo_rate_hz,
            simulator=self._demo_simulator,
        )

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        if self.recorder is not None:
            self.recorder.close()
            self.recorder = None
        if self._stack is not None:
            self._stack.close()
            self._stack = None
        self.server.stop()
        return None
