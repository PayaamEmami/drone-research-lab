# State estimation

Read every onboard sensor at once and run a Kalman filter over each stream, so
the raw signal and its filtered estimate can be compared live on the dashboard.
Nothing flies: the platform sits still while you disturb the sensors by hand.

**Status:** scaffold. The filter math (`filters.py`) and the dashboard renderer
for the `estimate` frame are not implemented yet.

## Concept

Time-of-flight and IMU signals are noisy. A Kalman filter maintains a running
estimate of a quantity (and its rate of change), balancing a motion model
against each new measurement according to how much noise each is assumed to
have. This experiment applies that idea two ways:

- **Per-channel smoothing.** A 1-D constant-velocity Kalman filter
  (`ScalarKalman`) runs independently on each range beam, the downward range
  finder, and the attitude angles - denoising the whole sensor suite.
- **Sensor fusion.** A single filter (`HeightFusionKalman`) estimates height and
  vertical velocity by combining *two* sensors: the vertical accelerometer
  (prediction) and the downward range finder (correction).

## Files

| File | Responsibility |
|------|----------------|
| `run.py` | Connect, stream all sensors, apply the filters, publish raw vs. filtered frames. |
| `filters.py` | `ScalarKalman` (1-D smoother) and `HeightFusionKalman` (accel + range fusion). |

## Planned dashboard frame

`estimate` -> `{ channel: { raw: float|null, filtered: float } }`, rendered as
overlaid raw/filtered traces per channel. (Dashboard renderer is a TODO.)

## Run

```bash
python -m experiments.state_estimation.run
python -m experiments.state_estimation.run --no-record --port 8000
```

## Verifying offline

The filters are pure NumPy and hardware-free; validate them on synthetic noisy
signals with `python -m pytest` before running against the live platform.
