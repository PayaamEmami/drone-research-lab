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
# Preview with synthetic data
python -m experiments.trajectory_tracking.run --demo

# Live (needs a Crazyflie + Flow deck; check with: python -m scripts.connect_check)
python -m experiments.trajectory_tracking.run
python -m experiments.trajectory_tracking.run --duration 45 --max-radius 1.0
```

## Flags

| Flag | Default | Description |
|------|---------|-------------|
| `--duration` | `30.0` | Flight duration in seconds |
| `--rate-hz` | `20.0` | Control loop rate in Hz |
| `--base-radius` | `0.3` | Starting spiral radius (m) |
| `--radius-growth` | `0.05` | Radius increase per revolution (m) |
| `--max-radius` | `0.8` | Maximum spiral radius (m) |
| `--base-height` | `0.4` | Starting height (m) |
| `--max-height` | `1.0` | Maximum height (m) |
| `--climb-rate` | `0.05` | Height increase per revolution (m) |
| `--angular-rate` | `0.6` | Angular speed around the spiral (rad/s) |
| `--kp-xy` / `--ki-xy` / `--kd-xy` | `1.0` / `0.0` / `0.0` | Horizontal PID gains |
| `--kp-z` / `--ki-z` / `--kd-z` | `1.0` / `0.0` / `0.0` | Vertical PID gains |
| `--max-speed` | `0.3` | Horizontal velocity clamp (m/s) |
| `--max-climb` | `0.3` | Vertical velocity clamp (m/s) |
| `--no-record` | off | Disable CSV recording of the reference vs. estimate |
| `--demo` | off | Preview the dashboard with synthetic data |
| `--demo-rate` | `20.0` | Demo update rate in Hz (with `--demo`) |
| `--uri` | `$DRL_URI` | Crazyflie radio URI override |
| `--port` | `8000` | Dashboard port |
| `--no-browser` | off | Don't auto-open the dashboard in a browser |
