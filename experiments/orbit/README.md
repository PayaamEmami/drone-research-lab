# Orbit

Find a nearby object with the front range beam and circle it. The drone yaws until that beam is the closest return inside the acquisition window, then holds a constant range while flying sideways. The yaw rate is paced so the nose stays on the object. A side beam inside `--too-close`, or the front return disappearing, aborts the circle and the search starts again.

`--object-radius` is only a guess used to pace that yaw. The distance the drone tries to hold is `--radius` (clearance to the surface).

Requires the Flow deck (height) and the Multi-ranger deck. Ctrl+C lands.

## Run

```bash
# Preview with synthetic data
python -m experiments.orbit.run --demo

# Live (needs a Crazyflie + Flow + Multi-ranger; check with: python -m scripts.connect_check)
python -m experiments.orbit.run
python -m experiments.orbit.run --radius 0.5 --direction cw --duration 45
```

## Flags

| Flag | Default | Description |
|------|---------|-------------|
| `--duration` | `30.0` | Flight duration in seconds (`0` runs until Ctrl+C) |
| `--rate-hz` | `20.0` | Control loop rate |
| `--height` | `0.4` | Hover height (m) |
| `--climb-rate` | `0.3` | Climb/descent speed (m/s) |
| `--radius` | `0.45` | Desired front range while orbiting (m) |
| `--speed` | `0.15` | Tangential speed on the circle (m/s) |
| `--direction` | `ccw` | `ccw` or `cw` |
| `--object-radius` | `0.20` | Guess of the object's radius, used to pace yaw (m) |
| `--acquire-max` | `0.80` | Farthest front range that counts as a target (m) |
| `--lose-range` | `1.20` | Front range that breaks the orbit (m) |
| `--too-close` | `0.15` | Hard reverse / abort distance (m) |
| `--max-speed` | `0.25` | Horizontal speed clamp (m/s) |
| `--no-record` | off | Disable CSV recording |
| `--demo` | off | Preview the dashboard with synthetic data |
| `--demo-rate` | `20.0` | Demo update rate in Hz (with `--demo`) |
| `--uri` | `$DRL_URI` | Crazyflie radio URI override |
| `--port` | `8000` | Dashboard port |
| `--no-browser` | off | Don't auto-open the dashboard in a browser |
