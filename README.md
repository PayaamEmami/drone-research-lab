# Drone Research Lab

Drone Research Lab (DRL) is a research platform for running experiments on a nano-quadcopter, with a browser dashboard for real-time telemetry and visualization.

## Experiments

| Experiment | What it does |
|------------|--------------|
| [Proximity HUD](experiments/proximity_hud) | Shows the drone's five range sensors live as a radial gauge and a scrolling chart. Nothing flies, so it's the safe first test. |
| [Reactive flight](experiments/reactive_flight) | The drone hovers and reacts to nearby objects: it either backs away from obstacles (push-away) or keeps a fixed distance from a hand you move toward it (hand-follow). |
| [Occupancy mapping](experiments/occupancy_mapping) | The drone scans the room as it moves and builds a live top-down map of which areas are open and which are blocked. |

## Hardware

- [Crazyflie 2.1 Brushless](https://www.bitcraze.io/products/crazyflie-2-1-brushless/): the 32g brushless nano-quadcopter this platform flies on.
- [Flow deck 2.0](https://www.bitcraze.io/products/flow-deck-v2/): downward optical-flow and time-of-flight sensor for position and altitude hold.
- [Multi-ranger deck](https://www.bitcraze.io/products/multi-ranger-deck/): five time-of-flight range sensors (front/back/left/right/up) for proximity sensing and mapping.
- [Crazyradio 2.0](https://www.bitcraze.io/products/crazyradio-2-0/): USB dongle providing the 2.4GHz radio link between the host computer and the drone.

## Getting started

Requires Python 3.10+ and a Crazyradio. Install the `drl` core package (editable):

```bash
python -m venv .venv
# Windows:  .venv\Scripts\activate
# macOS/Linux: source .venv/bin/activate
pip install -e .            # installs the drl core package (editable)
# or, with offline analysis extras (pandas + matplotlib):
pip install -e ".[analysis]"
```

This puts the `drl` core on the Python path so `import drl` works from anywhere. Run experiments and scripts as modules from the repo root (see below) so they resolve `drl` and `experiments.common` without any path hacks.

Next, bring up the hardware:

1. Plug the Crazyradio 2.0 into a USB port. On first use, follow Bitcraze's [Crazyradio 2.0 getting started guide](https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyradio-2-0/) to flash firmware and install the Windows USB driver with [Zadig](https://www.bitcraze.io/documentation/repository/crazyradio-firmware/master/building/usbwindows/). On Linux, see cflib's udev rules.
2. Attach a charged battery to the Crazyflie and mount the decks (Flow deck 2.0 underneath, Multi-ranger on top).
3. Power on the Crazyflie on a flat, level surface and leave it still. The connection resets the state estimator on connect and assumes the drone is stationary and level.
4. Set the radio URI if it differs from the default (`radio://0/80/2M/E7E7E7E7E7`):

```bash
export DRL_URI=radio://0/80/2M/E7E7E7E7E7   # or set DRL_URI=... on Windows
```

Then run everything as modules from the repo root, starting with the link check:

```bash
# 1) Confirm the link and decks (run this first, before anything flies):
python -m scripts.connect_check

# 2) Live proximity HUD (no flight; safe first run):
python -m experiments.proximity_hud.run

# 3) Reactive flight: validate on the desk first, then fly:
python -m experiments.reactive_flight.run --mode push --dry-run
python -m experiments.reactive_flight.run --mode push

# 4) Occupancy mapping: desk test, then a real scan:
python -m experiments.occupancy_mapping.run --pattern no-fly
python -m experiments.occupancy_mapping.run --pattern spin
```

Each command prints a dashboard URL (default <http://localhost:8000>) and opens it in the default browser. Press Ctrl+C to stop.

## License

Drone Research Lab is released under the [GNU General Public License v3.0](LICENSE).

## Acknowledgements

Built on top of Bitcraze's open-source `cflib` and Crazyflie ecosystem.
