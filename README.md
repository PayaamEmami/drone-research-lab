# Drone Research Lab

Drone Research Lab (DRL) is a research platform for running experiments on a nano-quadcopter, with a browser dashboard for real-time telemetry and visualization.

## Experiments

| Experiment | What it does |
|------------|--------------|
| [State estimation](experiments/state_estimation) | Reads every onboard sensor and runs Kalman filters over them, showing raw vs. filtered signals live. |
| [Basic flight](experiments/basic_flight) | Minimal takeoff-hover-land smoke test. Climb a few feet, hold briefly, then descend. |
| [Trajectory tracking](experiments/trajectory_tracking) | Flies a smooth parametric 3-D path (an expanding, ascending spiral) using PID position controllers, so the drone moves through all three axes at once. |
| [SLAM](experiments/slam) | The drone explores a space autonomously and builds a live 2-D occupancy map and a 3-D point cloud, correcting pose drift with scan matching. |

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

This puts the `drl` core on the Python path so `import drl` works from anywhere. Run experiments and scripts as modules from the repo root (see below).

Next, bring up the hardware:

1. Plug the Crazyradio 2.0 into a USB port. On first use, follow Bitcraze's [Crazyradio 2.0 getting started guide](https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyradio-2-0/) to flash firmware and install the Windows USB driver with [Zadig](https://www.bitcraze.io/documentation/repository/crazyradio-firmware/master/building/usbwindows/). On Linux, see cflib's udev rules.
2. Attach a charged battery to the Crazyflie and mount the decks (Flow deck 2.0 underneath, Multi-ranger on top).
3. Power on the Crazyflie on a flat, level surface and leave it still. The connection resets the state estimator on connect and assumes the drone is stationary and level.
4. Set the radio URI if it differs from the default (`radio://0/80/2M/E7E7E7E7E7`):

```bash
export DRL_URI=radio://0/80/2M/E7E7E7E7E7   # or set DRL_URI=... on Windows
```

Then run everything as modules from the repo root:

```bash
# Preview the dashboard with synthetic data (no drone required):
python -m scripts.dashboard_demo
python -m scripts.dashboard_demo --experiment trajectory
python -m scripts.dashboard_demo --experiment slam

# When hardware is ready, confirm the link and decks first:
python -m scripts.connect_check
```

Live experiment commands are documented in each experiment's README under `experiments/`. Each command prints a dashboard URL (default <http://localhost:8000>) and opens it in the default browser. Press Ctrl+C to stop.

## License

Drone Research Lab is released under the [GNU General Public License v3.0](LICENSE).

## Acknowledgements

Built on top of Bitcraze's open-source `cflib` and Crazyflie ecosystem.
