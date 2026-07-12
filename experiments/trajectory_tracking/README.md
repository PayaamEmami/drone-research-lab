# Trajectory tracking

Fly a smooth, time-parameterized 3-D path using outer-loop PID position control. The default path is an expanding, ascending spiral, so the platform moves through all three axes at once instead of just up/down or side/side.

## Files

| File | Responsibility |
|------|----------------|
| `run.py` | CLI, dashboard, connection, and the timed control loop. |
| `trajectory.py` | `SpiralParams` and `spiral(t)` reference generator. |
| `controller.py` | `PID` (single axis, anti-windup) and `TrajectoryController` (x/y/z). |

## Run

```bash
python -m experiments.trajectory_tracking.run
```
