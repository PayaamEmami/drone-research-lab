# Proximity HUD

A live heads-up display of the Multi-ranger deck. The five beams (front/back/left/right/up) and the down-facing z-range are streamed to the dashboard and drawn as:

- a radial HUD with one spoke per beam, length proportional to distance and color graded from green (far) to red (near), and
- a rolling time-series chart of the four horizontal beams.

This experiment does not fly. The drone can sit on your desk while you wave objects at the sensors and watch the data react in real time, which makes it the ideal first thing to run.

## Usage

Run as a module from the repo root (after `pip install -e .`):

```bash
python -m experiments.proximity_hud.run
python -m experiments.proximity_hud.run --no-record --port 8000
```

Open the printed URL and press Ctrl+C to stop.

Options: `--rate-ms` (sensor log period), `--no-record` (disable CSV logging), `--no-browser`, `--uri`, `--port`.

## Notes

- Beams report distance in meters; anything past ~4 m (the sensor ceiling) reads as "out of range" and is drawn faint at the HUD edge.
- Recordings land in `data/proximity_<timestamp>.csv`.
