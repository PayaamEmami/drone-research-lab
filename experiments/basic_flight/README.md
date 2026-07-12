# Basic flight

Minimal takeoff-hover-land smoke test: climb straight up, hover for a few seconds, then land.

## Run

```bash
# 1. Verify radio + decks first
python -m scripts.connect_check

# 2. Connect and stream telemetry without flying
python -m experiments.basic_flight.run --dry-run

# 3. Fly (~1.6 ft up, hover 4 s, land)
python -m experiments.basic_flight.run

# Higher / longer hover
python -m experiments.basic_flight.run --height 0.7 --hover 6
```

Defaults: `--height 0.5` m, `--hover 4` s, `--climb-rate 0.3` m/s.
