# Basic flight

Minimal takeoff-hover-land smoke test. Uses the Crazyflie firmware position controller: ramp altitude up once, hold steady, then ramp down and land.

## Run

```bash
# 1. Verify radio + decks first
python -m scripts.connect_check

# 2. Connect and stream telemetry without flying
python -m experiments.basic_flight.run --dry-run

# 3. Fly (~1.6 ft up, hover 4 s, land)
python -m experiments.basic_flight.run

# Even slower climb/descent
python -m experiments.basic_flight.run --height 0.5 --ramp-rate 0.04 --hover 6
```

Defaults: `--height 0.5`, `--hover 4`, `--ramp-rate 0.06` m/s (about 8 s to climb).