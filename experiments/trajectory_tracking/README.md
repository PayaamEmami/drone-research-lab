# Trajectory tracking

Fly a smooth, time-parameterized 3-D path using outer-loop PID position control.
The default path is an expanding, ascending spiral, so the platform moves
through all three axes at once instead of just up/down or side/side.

**Status:** scaffold. The trajectory generator, PID controllers, and flight loop
are not implemented yet.

## Concept

The firmware handles the fast inner attitude/rate loops. This experiment owns
the *outer* position loops: for each axis, a PID controller turns the position
error (reference minus current estimate) into a velocity command, and the three
commands form a velocity setpoint for the flight controller. Because the
reference is a function of time, the controller is *tracking a moving target*
rather than holding a fixed point.

The controllers include output clamping and integral anti-windup so a saturated
velocity command cannot wind the integral term up uncontrollably.

## Files

| File | Responsibility |
|------|----------------|
| `run.py` | CLI, dashboard, connection, and the timed control loop (dry-run + flight). |
| `trajectory.py` | `SpiralParams` and `spiral(t)` reference generator. |
| `controller.py` | `PID` (single axis, anti-windup) and `TrajectoryController` (x/y/z). |

## Planned dashboard frame

`traj` -> `{ reference:{x,y,z}, estimate:{x,y,z}, command:{vx,vy,vz} }`, rendered
as reference-vs-actual paths. (Dashboard renderer is a TODO.)

## Safety

Always run `--dry-run` first: it streams the reference and computed command
without taking off, so the control law can be checked on a desk. Fly only in an
open, bounded area with clearance above for the climbing spiral. Ctrl+C lands.

## Run

```bash
python -m experiments.trajectory_tracking.run --dry-run
python -m experiments.trajectory_tracking.run
```
