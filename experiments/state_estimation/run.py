"""Live state estimation: Kalman filtering across every onboard sensor.

This experiment reads *all* of the platform's sensors at once - the five
Multi-ranger beams, the downward range finder, the fused position/attitude
estimate, and the battery - and runs a Kalman filter over them so the dashboard
can show the raw signal next to its filtered estimate in real time.

Two layers of estimation are demonstrated:

1. Breadth: a per-channel :class:`~experiments.state_estimation.filters.ScalarKalman`
   smooths each scalar stream independently (denoising the whole sensor suite).
2. Depth: a :class:`~experiments.state_estimation.filters.HeightFusionKalman`
   fuses the vertical accelerometer with the downward range finder into a single
   height/velocity estimate.

No flight is involved - the platform can sit on a desk while you move objects
near the sensors and watch the raw vs. filtered traces respond.

Run (from the repo root, after ``pip install -e .``)::

    python -m experiments.state_estimation.run
    python -m experiments.state_estimation.run --no-record --port 8000
"""
from __future__ import annotations

import argparse
import time

from experiments.state_estimation.filters import HeightFusionKalman, ScalarKalman
from drl.cli import add_experiment_args
from drl.dashboard import Frame
from drl.sensors import Sensors
from drl.session import ExperimentSession
from drl.telemetry import (
    make_accel_config,
    make_battery_config,
    make_flow_config,
    make_multiranger_config,
    make_state_config,
)

# Standard gravity (m/s^2); acc.z reports ~1.0 g at rest, so subtracting 1 g
# yields the net world-frame vertical acceleration (assumes the platform is
# level, which holds for this desk experiment).
_GRAVITY = 9.80665

_RANGE_BEAMS = ("front", "back", "left", "right", "up", "down")
_ATTITUDE = ("roll", "pitch", "yaw")


def main() -> int:
    parser = argparse.ArgumentParser(description="drl: state estimation (Kalman filtering)")
    add_experiment_args(parser, record=True)
    parser.add_argument("--rate-ms", type=int, default=50, help="sensor log period")
    args = parser.parse_args()

    channels = list(_RANGE_BEAMS) + ["height"] + list(_ATTITUDE)
    fieldnames = ["elapsed_s", "vbat"]
    for channel in channels:
        fieldnames += [f"{channel}_raw", f"{channel}_filtered"]

    with ExperimentSession(
        "state estimation (Kalman filtering)",
        port=args.port,
        uri=args.uri,
        open_browser=not args.no_browser,
        connect_drone=True,
        arm=False,
        record=None if args.no_record else "state_estimation",
        record_fieldnames=fieldnames,
        demo=args.demo,
        demo_frames=("battery", "state", "ranger", "estimate"),
        demo_rate_hz=args.demo_rate,
    ) as sess:
        if args.demo:
            print("Previewing state estimation with synthetic data. Ctrl+C to stop.")
            sess.run_demo()
            print("Done.")
            return 0

        decks = sess.link.decks()
        if not decks.get("multiranger", False):
            print("WARNING: Multi-ranger deck not detected; beams will read out of range.")

        range_filters = {beam: ScalarKalman(q=1.0, r=0.05) for beam in _RANGE_BEAMS}
        attitude_filters = {angle: ScalarKalman(q=5.0, r=0.5) for angle in _ATTITUDE}
        height_filter = HeightFusionKalman()

        estimate: dict = {}
        last_t: dict = {}
        last_vbat = {"value": None}

        def _dt(block: str, sample) -> float:
            now = sample.get("_host_t", time.time())
            prev = last_t.get(block)
            last_t[block] = now
            return (now - prev) if prev is not None else 0.0

        sess.hub.add_config(make_multiranger_config(args.rate_ms))
        sess.hub.add_config(make_flow_config(args.rate_ms))
        sess.hub.add_config(make_state_config(args.rate_ms))
        sess.hub.add_config(make_accel_config(args.rate_ms))
        sess.hub.add_config(make_battery_config())
        sess.hub.attach_dashboard(sess.server, auto=["battery", "state"])

        def on_sample(block: str, ts: int, sample) -> None:
            if block in ("multiranger", "flow"):
                sensors = Sensors.from_hub(sess.hub)
                raw = sensors.as_dict()
                sess.server.publish(Frame("ranger", raw))
                dt = _dt("sensors", sample)
                for beam in _RANGE_BEAMS:
                    kf = range_filters[beam]
                    kf.predict(dt)
                    kf.update(raw.get(beam))
                    estimate[beam] = {"raw": raw.get(beam), "filtered": kf.value}
                height_filter.update(raw.get("down"))
                estimate["height"] = {"raw": raw.get("down"), "filtered": height_filter.height}
                sess.server.publish(Frame("estimate", dict(estimate)))
                _record(sample)
            elif block == "state":
                from drl.dashboard.frames import state_payload

                payload = state_payload(sample)
                dt = _dt("state", sample)
                for angle in _ATTITUDE:
                    kf = attitude_filters[angle]
                    kf.predict(dt)
                    kf.update(payload.get(angle))
                    estimate[angle] = {"raw": payload.get(angle), "filtered": kf.value}
            elif block == "accel":
                dt = _dt("accel", sample)
                accel_z = (sample.get("acc.z", 1.0) - 1.0) * _GRAVITY
                height_filter.predict(dt, accel_z)
            elif block == "battery":
                last_vbat["value"] = sample.get("pm.vbat")

        def _record(sample) -> None:
            if sess.recorder is None:
                return
            row: dict = {"vbat": last_vbat["value"]}
            for channel, pair in estimate.items():
                row[f"{channel}_raw"] = pair["raw"]
                row[f"{channel}_filtered"] = round(pair["filtered"], 4)
            sess.recorder.write(row)

        sess.hub.subscribe(on_sample)

        with sess.hub:
            print("Streaming sensors. Move objects near the beams. Ctrl+C to stop.")
            while not sess.stop.is_set():
                time.sleep(0.1)

    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
