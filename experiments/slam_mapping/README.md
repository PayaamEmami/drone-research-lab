# SLAM mapping

The platform explores a space autonomously and builds a live 2-D occupancy map
and a 3-D point cloud of it, correcting pose drift by matching its range scans
against the map as it goes.

**Status:** scaffold. The occupancy grid data structure is implemented; the scan
matcher, frontier explorer, point cloud, and the SLAM loop are not.

## Concept

The Multi-ranger gives only five fixed beams, so a single scan is sparse. Two
ideas make useful maps out of it anyway:

- **Yaw-sweeping + accumulation.** Rotating the platform sweeps the four
  horizontal beams around the room; hits accumulate into a coherent map over
  time - a "poor man's rotating scanner."
- **Scan-matching SLAM.** The onboard state estimate is good short-term but
  drifts. Each step, it is treated as odometry and corrected by searching for
  the small pose offset that best aligns the current scan with the map so far.
  This is the localization half of SLAM; the occupancy grid is the mapping half.

Exploration is frontier-based: the platform finds the boundary between known and
unknown space, plans a safe path there with A*, flies it, and repeats.

## Files

| File | Responsibility |
|------|----------------|
| `run.py` | CLI + modes (`explore` / `no-fly` / `replay`) and the SLAM loop. |
| `mapper.py` | `OccupancyGrid`: integrate scans, `score_scan` for matching, export. |
| `scan_match.py` | `match_scan`: correlative pose correction against the map. |
| `explorer.py` | Frontier detection + A* planning + `Explorer` goal selection. |
| `pointcloud.py` | `PointCloud`: accumulate 3-D hits, export `.ply` / dashboard payload. |

## Planned dashboard frames

- `map` -> occupancy grid + pose + latest hits (already supported by the dashboard).
- `cloud` -> `{ points: [[x,y,z], ...] }` for the 3-D view (renderer is a TODO).
- trajectory trail + raw-vs-corrected pose overlay (renderer is a TODO).

## Run

```bash
python -m experiments.slam_mapping.run --mode no-fly                       # desk/hand-carry
python -m experiments.slam_mapping.run --mode explore --save-cloud data/room.ply
python -m experiments.slam_mapping.run --mode replay --replay data/slam_xxx.csv
```

## Verifying offline

`mapper.py`, `scan_match.py`, and `explorer.py` are pure grid logic. Validate the
scan matcher and planner on a synthetic room with simulated beams
(`python -m pytest`) before flying; `--mode replay` re-runs the full pipeline on
a recorded flight with no hardware.
