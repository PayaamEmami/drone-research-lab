# State estimation

Read every onboard sensor at once and run a Kalman filter over each stream, so the raw signal and its filtered estimate can be compared live on the dashboard.

## Files

| File | Responsibility |
|------|----------------|
| `run.py` | Connect, stream all sensors, apply the filters, publish raw vs. filtered frames. |
| `filters.py` | `ScalarKalman` (1-D smoother) and `HeightFusionKalman` (accel + range fusion). |

## Run

```bash
# Preview with synthetic data
python -m experiments.state_estimation.run --demo

# Live (needs a Crazyflie; check with: python -m scripts.connect_check)
python -m experiments.state_estimation.run
python -m experiments.state_estimation.run --no-record --port 8001
```

## Flags

| Flag | Default | Description |
|------|---------|-------------|
| `--rate-ms` | `50` | Sensor log period in milliseconds |
| `--no-record` | off | Disable CSV recording of raw/filtered channels |
| `--demo` | off | Preview the dashboard with synthetic data |
| `--demo-rate` | `20.0` | Demo update rate in Hz (with `--demo`) |
| `--uri` | `$DRL_URI` | Crazyflie radio URI override |
| `--port` | `8000` | Dashboard port |
| `--no-browser` | off | Don't auto-open the dashboard in a browser |
