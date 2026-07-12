"""High-fidelity trajectory-tracking preview for ``--demo`` (no hardware).

Reuses the real spiral generator and PID controller so the previewed path and
command vector behave like the live experiment. Passed to
:class:`~drl.session.ExperimentSession` as its ``demo_simulator``.
"""
from __future__ import annotations

import math
import time

from drl.dashboard import DashboardServer, Frame, simulate_ranges
from drl.dashboard.demo import battery_payload
from experiments.trajectory_tracking.controller import PID, TrajectoryController
from experiments.trajectory_tracking.trajectory import SpiralParams, spiral


def simulate(server: DashboardServer, stop, rate_hz: float) -> None:
    period = 1.0 / rate_hz
    params = SpiralParams()
    controller = TrajectoryController(
        x=PID(kp=1.0, ki=0.0, kd=0.0, out_limit=0.3),
        y=PID(kp=1.0, ki=0.0, kd=0.0, out_limit=0.3),
        z=PID(kp=1.0, ki=0.0, kd=0.0, out_limit=0.3),
    )
    est = [0.0, 0.0, 0.0]
    t0 = time.monotonic()
    last = None

    while not stop.is_set():
        now = time.monotonic()
        t = now - t0
        dt = period if last is None else now - last
        last = now

        ref = spiral(t, params)
        est[0] += 0.35 * (ref[0] - est[0])
        est[1] += 0.35 * (ref[1] - est[1])
        est[2] += 0.35 * (ref[2] - est[2])
        vx, vy, vz = controller.step(ref, tuple(est), dt)

        server.publish(Frame("state", {
            "x": est[0], "y": est[1], "z": est[2],
            "roll": 2.0 * math.sin(t * 0.4),
            "pitch": 1.5 * math.cos(t * 0.35),
            "yaw": math.degrees(math.atan2(ref[1], ref[0])),
        }))
        server.publish(Frame("traj", {
            "reference": {"x": ref[0], "y": ref[1], "z": ref[2]},
            "estimate": {"x": est[0], "y": est[1], "z": est[2]},
            "command": {"vx": vx, "vy": vy, "vz": vz},
        }))
        server.publish(Frame("cmd", {"vx": vx, "vy": vy, "label": "spiral [demo]"}))
        server.publish(Frame("ranger", simulate_ranges(est[0], est[1], math.radians(t * 20))))
        server.publish(Frame("battery", battery_payload(t)))
        if stop.wait(period):
            break
