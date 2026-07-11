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
python -m experiments.slam.run --mode no-fly                       # desk/hand-carry
python -m experiments.slam.run --mode explore --save-cloud data/room.ply
python -m experiments.slam.run --mode replay --replay data/slam_xxx.csv
```
