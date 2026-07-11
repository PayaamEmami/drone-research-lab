# State estimation

Read every onboard sensor at once and run a Kalman filter over each stream, so the raw signal and its filtered estimate can be compared live on the dashboard.

## Files

| File | Responsibility |
|------|----------------|
| `run.py` | Connect, stream all sensors, apply the filters, publish raw vs. filtered frames. |
| `filters.py` | `ScalarKalman` (1-D smoother) and `HeightFusionKalman` (accel + range fusion). |

## Run

```bash
python -m experiments.state_estimation.run
python -m experiments.state_estimation.run --no-record --port 8000
```