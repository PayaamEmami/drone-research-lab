# Occupancy mapping

Builds a live 2D occupancy grid of the room from the Multi-ranger beams, using the Crazyflie's onboard state estimate for pose. This is the mapping half of a SLAM pipeline (localization is taken from the estimator rather than solved jointly).

For each scan, every horizontal beam is ray-cast from the drone to its hit point with a log-odds sensor model: cells along the ray accumulate evidence of free space, and the hit cell accumulates evidence of an obstacle. The dashboard renders the grid (free / occupied / unknown), the drone pose, and the most recent beam hits.

See [mapper.py](mapper.py) for the grid, the Bresenham ray-casting, and the log-odds update.

## Usage

Run as a module from the repo root (after `pip install -e .`):

```bash
# Safe desk test (no takeoff):
python -m experiments.occupancy_mapping.run --pattern no-fly

# Real scans:
python -m experiments.occupancy_mapping.run --pattern spin
python -m experiments.occupancy_mapping.run --pattern square --side 0.8

# Save the raw grid for offline re-plotting:
python -m experiments.occupancy_mapping.run --pattern spin --save data/room.npz
```

Options:

- `--pattern`: flight pattern
  - `no-fly`: do not take off; carry the drone over a textured floor and map by hand (best for first tests).
  - `spin`: take off, rotate slowly in place to sweep the room, land.
  - `square`: take off, fly a small square; set edge length with `--side`.
  - `hover`: take off and hover until you stop it.
- `--side`: square edge length (m); used with `--pattern square`.
- `--size`: map side length (m).
- `--res`: cell size (m/cell).
- `--map-hz`: map broadcast rate (Hz).
- `--height`: takeoff height (m); used with flying patterns.
- `--yaw-rate`: spin rate (deg/s); used with `--pattern spin`.
- `--save`: write the raw log-odds grid to a `.npz` file for offline re-plotting.
- `--uri`, `--port`: radio URI and dashboard port.

## Notes

- Bigger rooms: increase `--size`. Finer detail: decrease `--res` (more cells = more data per frame; lower `--map-hz` if the browser lags).
- The map is centered on the drone's start position. Reset the estimator (done automatically before flight) so poses start near the origin.
- Drift in the onboard estimate shows up as smeared walls; that's expected for estimate-only mapping and is itself an interesting thing to study.
