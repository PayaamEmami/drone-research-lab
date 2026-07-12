# Basic flight

Minimal takeoff-hover-land smoke test: climb straight up, hover for a few seconds, then land. Use it to confirm the platform flies before running more complex experiments.

## Run

```bash
# Preview with synthetic data
python -m experiments.basic_flight.run --demo

# Live (needs a Crazyflie + Flow deck; check with: python -m scripts.connect_check)
python -m experiments.basic_flight.run
python -m experiments.basic_flight.run --height 0.7 --hover 6
```

## Flags

| Flag | Default | Description |
|------|---------|-------------|
| `--height` | `0.5` | Hover height in meters |
| `--hover` | `4.0` | Hover duration in seconds |
| `--climb-rate` | `0.3` | Climb/descent speed in m/s |
| `--demo` | off | Preview the dashboard with synthetic data |
| `--demo-rate` | `20.0` | Demo update rate in Hz (with `--demo`) |
| `--uri` | `$DRL_URI` | Crazyflie radio URI override |
| `--port` | `8000` | Dashboard port |
| `--no-browser` | off | Don't auto-open the dashboard in a browser |
