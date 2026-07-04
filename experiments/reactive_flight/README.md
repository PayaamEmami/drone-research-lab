# Reactive flight

The drone takes off, hovers, and reacts to the Multi-ranger beams in real time:

- `--mode push`: shies away from anything that gets within `--safe` meters on the front/back/left/right beams (wave a hand, it backs off).
- `--mode follow`: holds a fixed standoff distance (`--target`) from whatever is in front of it.

The commanded body-frame velocity vector is streamed live to the dashboard (the Command panel), alongside the proximity HUD and state estimate.

## Usage

Run as a module from the repo root (after `pip install -e .`):

```bash
# 1) Validate the control law safely (no takeoff):
python -m experiments.reactive_flight.run --mode push --dry-run

# 2) Fly it:
python -m experiments.reactive_flight.run --mode push
python -m experiments.reactive_flight.run --mode follow --target 0.4
```

Options: `--max-speed` (velocity clamp), `--gain` (how aggressively it reacts), `--safe` (push react distance), `--track-max` (follow give-up distance), `--rate-hz` (control loop rate).

## Notes

- Each loop iteration polls the normalized `RangerReading`, computes a body-frame velocity, and sends it with `MotionCommander.start_linear_motion(vx, vy, 0)` (positive X forward, positive Y left).
- Push mode sums a proportional "repulsion" from each near beam; follow mode is a proportional controller on the front-beam distance error. Both clamp to `--max-speed`.
