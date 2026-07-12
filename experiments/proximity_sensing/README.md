# Proximity sensing

Watch the platform's six time-of-flight beams react to their surroundings in real time. The platform stays still (no flight) while the dashboard shows a Proximity HUD and a raw-values table; move objects near a beam and the readings respond.

## Run

```bash
# Preview with synthetic data
python -m experiments.proximity_sensing.run --demo

# Live (needs a Crazyflie + Multi-ranger deck; check with: python -m scripts.connect_check)
python -m experiments.proximity_sensing.run
python -m experiments.proximity_sensing.run --no-record --port 8001
```

## Flags

| Flag | Default | Description |
|------|---------|-------------|
| `--rate-ms` | `50` | Sensor log period in milliseconds |
| `--no-record` | off | Disable CSV recording of the six ranges |
| `--demo` | off | Preview the dashboard with synthetic data |
| `--demo-rate` | `20.0` | Demo update rate in Hz (with `--demo`) |
| `--uri` | `$DRL_URI` | Crazyflie radio URI override |
| `--port` | `8000` | Dashboard port |
| `--no-browser` | off | Don't auto-open the dashboard in a browser |
