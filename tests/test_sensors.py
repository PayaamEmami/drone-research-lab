"""Tests for per-deck sensor adapters and the combined Sensors snapshot."""
from __future__ import annotations

from drl.sensors import Sensors
from drl.sensors.flow import FlowReading
from drl.sensors.multiranger import MultirangerReading


def test_multiranger_reading_converts_mm_to_meters():
    reading = MultirangerReading.from_sample({
        "range.front": 500,
        "range.back": 8000,
        "range.left": 1200,
    })
    assert reading.front == 0.5
    assert reading.back is None
    assert reading.left == 1.2
    assert reading.up is None


def test_flow_reading_converts_downward_range():
    reading = FlowReading.from_sample({"range.zrange": 250})
    assert reading.down == 0.25
    assert reading.as_dict() == {"down": 0.25}


def test_sensors_merges_deck_readings():
    sensors = Sensors.from_samples(
        {"range.front": 400, "range.up": 900},
        {"range.zrange": 300},
    )
    assert sensors.as_dict() == {
        "front": 0.4,
        "back": None,
        "left": None,
        "right": None,
        "up": 0.9,
        "down": 0.3,
    }
