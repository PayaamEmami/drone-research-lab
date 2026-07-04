# AGENTS.md

This file orients AI coding agents and human readers to this repository. It explains what the project is, how it is organized, and the guidance to follow when working here.

## Overview

Drone Research Lab is a Python research platform for running flight and sensing experiments on a [Bitcraze Crazyflie](https://www.bitcraze.io/) nano-quadcopter. It pairs a small reusable core library with standalone experiment runners and a live browser dashboard (FastAPI + websockets) for telemetry and visualization.

## Stack

- Language: Python 3.10+
- Drone link: `cflib` (official Bitcraze Crazyflie library) over a Crazyradio 2.0
- Dashboard backend: FastAPI + uvicorn, served on a background thread
- Dashboard frontend: vanilla JS, HTML Canvas, and Chart.js under `drl/dashboard/static/`
- Numerics: NumPy; optional offline analysis via pandas + matplotlib (`analysis` extra)
- Packaging: setuptools via `pyproject.toml`; the core installs editable with `pip install -e .`

## Repository Layout

```
drl/                reusable core library
  config.py              defaults for URI, dashboard, and telemetry
  connection.py          open the radio link and detect decks
  telemetry.py           subscribe to and route log data
  sensors/ranger.py      multiranger distance readings
  recording.py           save telemetry to CSV
  dashboard/             live web dashboard
  viz/                   offline plots (optional analysis extra)
experiments/             runnable experiment scripts
  common.py              shared helpers
  proximity_hud/         live range HUD, no flight
  reactive_flight/       reactive hover experiments
  occupancy_mapping/     live 2D mapping
scripts/                 setup and diagnostic utilities
```

## Architecture

Drone Research Lab separates a small reusable core (the `drl` package) from experiments (standalone scripts under `experiments/`). The core handles everything that every experiment needs; experiments add only their own logic.

```mermaid
flowchart TB
  cf["Crazyflie"] -->|"CRTP radio"| cflib["cflib (pip dependency)"]
  cflib --> core["drl core<br/>connection / telemetry / sensors / recording"]
  exp["experiment script<br/>experiments/*/run.py"] -->|uses| core
  core --> rec["recording.py (CSV)"]
  core --> server["dashboard server (FastAPI + WS)"]
  server --> browser["browser UI (Canvas + Chart.js)"]
```

The core package (`drl/`) is installable and import-only. Experiments live under `experiments/` and read like standalone demos. The dashboard runs on a background thread; experiments call `server.publish(Frame(...))` from anywhere and the frame is broadcast to every connected browser as JSON.

### Core modules

| Module | Responsibility |
|--------|----------------|
| `drl.config` | URI + server + telemetry defaults, all environment-overridable. |
| `drl.connection` | `connect()` context manager around cflib's `SyncCrazyflie`: driver init, arm, estimator reset, deck detection. |
| `drl.telemetry` | `LogConfig` builders + `TelemetryHub`, which fans incoming samples out to subscribers and keeps the latest sample per block. |
| `drl.sensors.ranger` | Normalizes raw Multi-ranger millimeters into meters/`None` (`RangerReading`, `RangerStream`). |
| `drl.recording` | `CsvRecorder`: append any telemetry dict to a timestamped CSV. |
| `drl.dashboard` | `DashboardServer` (FastAPI + websocket, runs on a background thread, broadcasts frames) plus the browser UI under `static/`. |
| `drl.viz` | Optional matplotlib helpers for static figures (the `analysis` extra). |

### Data flow

```mermaid
flowchart LR
  cf["Crazyflie"] -->|CRTP| cflib
  cflib --> hub["TelemetryHub"]
  hub -->|samples| exp["experiment loop"]
  exp -->|"Frame(...)"| server["DashboardServer"]
  exp --> rec["CsvRecorder"]
  server -->|"JSON over /ws"| browser["dashboard"]
```

cflib delivers log data on its own threads. The experiment subscribes to the `TelemetryHub`, transforms samples into dashboard **frames**, and publishes them. `DashboardServer.publish()` is thread-safe: it marshals the frame onto the server's asyncio loop with `run_coroutine_threadsafe` and broadcasts to all connected websockets. The latest frame of each `type` is cached and replayed to clients that connect mid-run.

### Frame protocol

Every websocket message is:

```json
{ "type": "ranger", "ts": 1718600000.12, "payload": { ... } }
```

The browser UI (`drl/dashboard/static/js/`) switches on `type`:

| `type` | payload | rendered as |
|--------|---------|-------------|
| `meta` | `{ experiment }` | header label |
| `ranger` | `{ front, back, left, right, up, down }` meters or `null` | radial HUD + chart |
| `state` | `{ x, y, z, roll, pitch, yaw }` | state readout |
| `cmd` | `{ vx, vy, label }` body-frame m/s | command vector |
| `map` | `{ res, width, height, origin, data, pose, points }` | occupancy grid |

### Adding an experiment

1. Create `experiments/<name>/run.py` (and an empty `experiments/<name>/__init__.py` so it is an importable package).
2. Import the core directly. With the core installed via `pip install -e .` and the experiment run as a module (`python -m experiments.<name>.run`), both `drl` and `experiments.common` resolve normally:

   ```python
   from experiments.common import install_stop_handler
   from drl.connection import connect
   from drl.dashboard import DashboardServer, Frame
   from drl.telemetry import TelemetryHub, make_ranger_config
   ```

3. Start a `DashboardServer`, open a `connect()` link, register log configs on a `TelemetryHub`, and `server.publish(Frame(type, payload))` from your loop or from telemetry callbacks.
4. If you invent a new frame `type`, add a small renderer module under `drl/dashboard/static/js/` and register it in `main.js` (and, if needed, a panel in `index.html`).

### Architecture Rules

- The `drl/` core is reusable and import-only. It should not depend on any single experiment.
- Experiments live under `experiments/<name>/run.py` and read like standalone demos; they add only their own logic and import the core directly.
- Run experiments and scripts as modules from the repo root (`python -m experiments.<name>.run`) so `drl` and `experiments.common` resolve without path hacks.
- The dashboard runs on a background thread. Publish data with `server.publish(Frame(type, payload))`; `DashboardServer.publish()` is thread-safe and broadcasts JSON to all connected browsers.
- Every websocket message follows the frame protocol: `{ "type", "ts", "payload" }`. Known types are `meta`, `ranger`, `state`, `cmd`, and `map` (see the Architecture section in `README.md`).
- A new frame `type` needs a matching renderer under `drl/dashboard/static/js/`, registered in `main.js` (and a panel in `index.html` if needed).
- Heavy payloads (the occupancy `map`) are published from a dedicated rate-limited thread so they never block sensor callbacks.

## Threading Model

- Main thread: the experiment loop (often blocking, e.g. `MotionCommander` moves).
- cflib threads: deliver telemetry log callbacks; `TelemetryHub` fans them out to subscribers.
- Dashboard thread: the uvicorn event loop serving HTTP + websockets.

## Recordings & Analysis

- Experiments stream telemetry to timestamped CSVs under `data/` via `drl.recording.CsvRecorder`. The `data/` folder is gitignored, so recorded runs are never committed.
- The occupancy mapping experiment can also dump its raw log-odds grid with `--save map.npz` (NumPy `.npz`) for offline re-plotting without a live drone.
- Offline analysis and plotting depend on the optional `analysis` extra (`pip install -e ".[analysis]"`), which adds pandas + matplotlib. The `drl.viz` matplotlib helpers are gated behind this extra; keep them out of the import-only core path.

## Environment Notes

Configuration is environment-overridable so experiments stay portable across machines and radios (see `drl/config.py`):

- `DRL_URI` (or `CFLIB_URI`): Crazyflie radio URI, defaults to `radio://0/80/2M/E7E7E7E7E7`
- `DRL_HOST`: dashboard bind host, defaults to `127.0.0.1`
- `DRL_PORT`: dashboard port, defaults to `8000`

Hardware/driver notes:

- On Windows the Crazyradio may need the Zadig USB driver; on Linux see cflib's udev rules.
- Do not assume a Crazyflie or Crazyradio is connected when verifying a change. Prefer dry-run / no-fly paths and local logic checks unless the task explicitly requires live hardware.

## Common Commands

Install the core (editable), optionally with extras:

```bash
python -m venv .venv
# Windows:  .venv\Scripts\activate
# macOS/Linux: source .venv/bin/activate
pip install -e .                 # core only
pip install -e ".[analysis]"     # + pandas + matplotlib
pip install -e ".[dev]"          # + pytest
```

Run from the repo root as modules:

```bash
python -m scripts.connect_check                              # confirm link + decks
python -m experiments.proximity_hud.run                      # no flight, safe first run
python -m experiments.reactive_flight.run --mode push --dry-run
python -m experiments.occupancy_mapping.run --pattern no-fly
```

## Verification Commands

There is no CI and no committed test suite yet. The `dev` extra provides `pytest` for when tests are added.

Run the strongest checks available for the files you changed:

```bash
python -m py_compile <changed_files>   # syntax check without hardware
python -m pytest                       # once tests exist (dev extra)
```

For experiment logic, prefer the non-flying validation paths (`--dry-run`, `--pattern no-fly`) over live flight.

## Agent Done Criteria

Before finishing a code change:

1. Run the strongest local checks available for the touched files (at minimum `py_compile`; `pytest` if tests exist).
2. If you changed a dashboard frame `type`, update both the publisher and the matching JS renderer / `main.js` registration.
3. If you changed core behavior, threading, env vars, or the frame protocol, keep the Architecture section in `README.md` consistent.
4. Report which checks you ran and whether they passed.
5. If a check could not be run (e.g. no hardware, no radio), say exactly why and call out the remaining risk.
6. If your changes make this file inaccurate or incomplete, update `AGENTS.md` in the same task.

## Conventions

- Keep comments sparse and useful; explain intent, not mechanics.
- The core stays dependency-light and import-only; heavyweight or experiment-specific logic belongs under `experiments/`.
- Drone Research Lab is GPLv3 to match `cflib`; `cflib` is a separate PyPI runtime dependency and is never vendored or modified.
