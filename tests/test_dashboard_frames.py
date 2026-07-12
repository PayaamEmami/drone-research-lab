"""Tests for dashboard frame payload helpers."""
from __future__ import annotations

from drl.dashboard.frames import battery_payload, publish_battery, state_payload


class _FakeServer:
    def __init__(self):
        self.frames = []

    def publish(self, frame):
        self.frames.append(frame)


def test_state_payload_maps_fields():
    sample = {
        "stateEstimate.x": 1.0,
        "stateEstimate.y": 2.0,
        "stateEstimate.z": 0.5,
        "stabilizer.roll": 3.0,
        "stabilizer.pitch": -2.0,
        "stabilizer.yaw": 90.0,
    }
    payload = state_payload(sample)
    assert payload == {
        "x": 1.0,
        "y": 2.0,
        "z": 0.5,
        "roll": 3.0,
        "pitch": -2.0,
        "yaw": 90.0,
    }


def test_battery_payload_parses_voltage():
    assert battery_payload({"pm.vbat": 3.9}) == {"vbat": 3.9}


def test_battery_payload_missing_returns_none():
    assert battery_payload({}) == {"vbat": None}


def test_publish_battery_skips_missing_voltage():
    server = _FakeServer()
    publish_battery(server, {})
    assert server.frames == []


def test_publish_battery_emits_frame():
    server = _FakeServer()
    publish_battery(server, {"pm.vbat": 4.1})
    assert len(server.frames) == 1
    assert server.frames[0].type == "battery"
    assert server.frames[0].payload == {"vbat": 4.1}
