# Basic flight

Minimal takeoff-hover-land smoke test. Climb to a target height, hold a few seconds, then descend. Use this when more complex experiments fail and you need to isolate the problem.

## Run

```bash
# 1. Verify radio + decks first
python -m scripts.connect_check

# 2. Connect and stream telemetry without flying
python -m experiments.basic_flight.run --dry-run

# 3. Fly (~2 ft up, hover 3 s, land)
python -m experiments.basic_flight.run

# Taller / longer hover
python -m experiments.basic_flight.run --height 0.9 --hover 5
```
