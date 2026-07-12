# SLAM

The platform explores a space autonomously and builds a live 2-D occupancy map and a 3-D point cloud of it, correcting pose drift by matching its range scans against the map as it goes.

## Files

| File | Responsibility |
|------|----------------|
| `run.py` | CLI + modes (`explore` / `no-fly` / `replay`) and the SLAM loop. |
| `mapper.py` | `OccupancyGrid`: integrate scans, `score_scan` for matching, export. |
| `scan_match.py` | `match_scan`: correlative pose correction against the map. |
| `explorer.py` | Frontier detection + A* planning + `Explorer` goal selection. |
| `pointcloud.py` | `PointCloud`: accumulate 3-D hits, export `.ply` / dashboard payload. |

## Run

```bash
# Preview with synthetic data
python -m experiments.slam.run --demo

# Live (needs a Crazyflie + Flow + Multi-ranger decks; check with: python -m scripts.connect_check)
python -m experiments.slam.run --mode no-fly                        # desk / hand-carry
python -m experiments.slam.run --mode explore --save-cloud data/room.ply
python -m experiments.slam.run --mode replay --replay data/slam_xxx.csv
```

## Flags

| Flag | Default | Description |
|------|---------|-------------|
| `--mode` | `no-fly` | `explore` (autonomous flight), `no-fly` (hand-carry), or `replay` (offline) |
| `--replay` | none | Path to a recorded CSV (required for `--mode replay`) |
| `--height` | `0.4` | Hover height in meters (`explore` mode) |
| `--size` | `8.0` | Map side length in meters |
| `--res` | `0.05` | Map resolution in meters per cell |
| `--rate-ms` | `100` | Telemetry log period in milliseconds |
| `--map-hz` | `5.0` | Map broadcast rate in Hz |
| `--save-map` | none | Save the final occupancy grid to a `.npz` file |
| `--save-cloud` | none | Save the point cloud to a `.ply` file |
| `--no-record` | off | Disable CSV recording of the SLAM run |
| `--demo` | off | Preview the dashboard with synthetic data |
| `--demo-rate` | `20.0` | Demo update rate in Hz (with `--demo`) |
| `--uri` | `$DRL_URI` | Crazyflie radio URI override |
| `--port` | `8000` | Dashboard port |
| `--no-browser` | off | Don't auto-open the dashboard in a browser |
