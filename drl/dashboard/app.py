"""FastAPI + websocket dashboard server.

Design: experiments run their own (often blocking) control loops on the main
thread using cflib. The dashboard runs uvicorn on a background thread with its
own asyncio event loop. Experiments call :meth:`DashboardServer.publish` from
any thread; the frame is marshaled onto the server loop and broadcast to every
connected browser as JSON.

Each frame is ``{"type": str, "ts": float, "payload": dict}``. The browser keys
off ``type`` to update the matching panel (``meta``, ``ranger``, ``state``,
``cmd``, ``map``, ``estimate``, ``traj``, ``cloud``, ``battery``). The latest frame of each
type is cached and replayed to newly-connected clients so a browser opened
mid-run is immediately populated.
"""
from __future__ import annotations

import asyncio
import logging
import threading
import time
import warnings
from contextlib import asynccontextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional, Set

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles

from drl.config import ServerConfig

logger = logging.getLogger(__name__)

STATIC_DIR = Path(__file__).parent / "static"


@dataclass
class Frame:
    """A single message broadcast to the dashboard."""

    type: str
    payload: Dict[str, Any] = field(default_factory=dict)
    ts: float = field(default_factory=time.time)

    def to_json(self) -> Dict[str, Any]:
        return {"type": self.type, "ts": self.ts, "payload": self.payload}


class DashboardServer:
    """A self-contained dashboard web server runnable from any experiment."""

    def __init__(self, config: Optional[ServerConfig] = None):
        self.config = config or ServerConfig()
        self._clients: Set[WebSocket] = set()
        self._latest: Dict[str, Dict[str, Any]] = {}
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._ready = threading.Event()
        self._server: Optional[uvicorn.Server] = None
        self._thread: Optional[threading.Thread] = None

        @asynccontextmanager
        async def _lifespan(app: FastAPI):  # noqa: ANN202
            # Capture the server's event loop so publish() can marshal onto it.
            self._loop = asyncio.get_running_loop()
            self._ready.set()
            yield

        self._app = FastAPI(title="drl dashboard", lifespan=_lifespan)
        self._configure_routes()

    def _configure_routes(self) -> None:
        app = self._app

        @app.websocket("/ws")
        async def _ws(ws: WebSocket) -> None:
            await ws.accept()
            self._clients.add(ws)
            # Replay the latest frame of each type to the new client.
            for frame in list(self._latest.values()):
                try:
                    await ws.send_json(frame)
                except Exception:  # noqa: BLE001
                    logger.debug("Websocket replay failed for %s", frame.get("type"), exc_info=True)
            try:
                while True:
                    # We don't expect inbound messages; this detects disconnect.
                    await ws.receive_text()
            except WebSocketDisconnect:
                pass
            except Exception:  # noqa: BLE001
                logger.debug("Websocket error", exc_info=True)
            finally:
                self._clients.discard(ws)

        # Serve the dashboard's static assets (index.html at "/").
        app.mount("/", StaticFiles(directory=str(STATIC_DIR), html=True), name="static")

    async def _broadcast(self, frame: Dict[str, Any]) -> None:
        dead: list[WebSocket] = []
        for ws in list(self._clients):
            try:
                await ws.send_json(frame)
            except Exception:  # noqa: BLE001
                dead.append(ws)
        for ws in dead:
            self._clients.discard(ws)

    def publish(self, frame: Frame | Dict[str, Any]) -> None:
        """Broadcast a frame to all clients. Safe to call from any thread."""
        data = frame.to_json() if isinstance(frame, Frame) else frame
        self._latest[data["type"]] = data
        if self._loop is None:
            return  # server not up yet; frame is still cached for replay
        asyncio.run_coroutine_threadsafe(self._broadcast(data), self._loop)

    def start(self, *, open_browser: bool = False, timeout_s: float = 10.0) -> "DashboardServer":
        """Start the server on a background thread and wait until it's ready."""
        # uvicorn still imports websockets.legacy on current releases; silence noise.
        warnings.filterwarnings(
            "ignore",
            category=DeprecationWarning,
            message=r".*websockets.*",
        )
        cfg = uvicorn.Config(
            self._app,
            host=self.config.host,
            port=self.config.port,
            log_level="warning",
        )
        self._server = uvicorn.Server(cfg)
        # uvicorn installs its own signal handlers only on the main thread.
        self._server.install_signal_handlers = lambda: None  # type: ignore[assignment]
        self._thread = threading.Thread(target=self._server.run, daemon=True, name="drl-dashboard")
        self._thread.start()
        if not self._ready.wait(timeout=timeout_s):
            raise RuntimeError("Dashboard server failed to start in time")

        url = self.url
        logger.info("Dashboard available at %s", url)
        print(f"\n  drl dashboard -> {url}\n")
        if open_browser:
            import webbrowser

            webbrowser.open(url)
        return self

    def stop(self) -> None:
        if self._server is not None:
            self._server.should_exit = True
        if self._thread is not None:
            self._thread.join(timeout=5.0)

    @property
    def url(self) -> str:
        host = "localhost" if self.config.host in ("0.0.0.0", "127.0.0.1") else self.config.host
        return f"http://{host}:{self.config.port}"

    def __enter__(self) -> "DashboardServer":
        return self.start()

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.stop()
