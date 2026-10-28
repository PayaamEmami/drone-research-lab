# Hand chase

Hover and keep a hand at a fixed distance in front of the Multi-ranger. The drone moves forward when the hand backs away, reverses when the hand comes closer, and yaws toward whichever side beam sees the hand. Beyond `--lost-range` it stops translating and yaws in place until the hand shows up again.

Requires the Flow deck (height) and the Multi-ranger deck. Ctrl+C lands.

## Run

```bash
# Preview with synthetic data
python -m experiments.hand_chase.run --demo

# Live (needs a Crazyflie + Flow + Multi-ranger; check with: python -m scripts.connect_check)
python -m experiments.hand_chase.run
python -m experiments.hand_chase.run --standoff 0.4 --duration 20
```

## Flags

| Flag | Default | Description |
|------|---------|-------------|
| `--duration` | `30.0` | Flight duration in seconds (`0` runs until Ctrl+C) |
| `--rate-hz` | `20.0` | Control loop rate |
| `--height` | `0.4` | Hover height (m) |
| `--climb-rate` | `0.3` | Climb/descent speed (m/s) |
| `--standoff` | `0.30` | Desired front range (m) |
| `--lost-range` | `1.2` | Front range at which the chase gives up (m) |
| `--too-close` | `0.12` | Hard reverse distance (m) |
| `--max-speed` | `0.25` | Horizontal speed clamp (m/s) |
| `--no-record` | off | Disable CSV recording |
| `--demo` | off | Preview the dashboard with synthetic data |
| `--demo-rate` | `20.0` | Demo update rate in Hz (with `--demo`) |
| `--uri` | `$DRL_URI` | Crazyflie radio URI override |
| `--port` | `8000` | Dashboard port |
| `--no-browser` | off | Don't auto-open the dashboard in a browser |
