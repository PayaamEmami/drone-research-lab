"""Tests for ``experiments.state_estimation.filters``.

Offline checks for per-channel smoothing and height fusion. Uses
:func:`tests.conftest.noisy_signal` and the shared ``rng`` fixture.
"""
from __future__ import annotations

import numpy as np

from experiments.state_estimation.filters import HeightFusionKalman, ScalarKalman
from tests.conftest import noisy_signal


# ---------------------------------------------------------------------------
# ScalarKalman
# ---------------------------------------------------------------------------


def test_scalar_kalman_smooths_noise(rng):
    n = 400
    truth = np.full(n, 1.5)
    noisy = noisy_signal(truth, noise_std=0.2, generator=rng)

    kf = ScalarKalman(q=0.5, r=0.04)
    out = []
    for z in noisy:
        kf.predict(0.02)
        kf.update(float(z))
        out.append(kf.value)
    out = np.array(out)

    settle = slice(50, None)
    assert out[settle].std() < noisy[settle].std()
    assert abs(out[-1] - 1.5) < 0.1
    assert np.all(np.isfinite(out))


def test_scalar_kalman_tracks_ramp(rng):
    n = 300
    t = np.arange(n) * 0.02
    truth = 0.5 * t
    noisy = noisy_signal(truth, noise_std=0.05, generator=rng)

    kf = ScalarKalman(q=2.0, r=0.01)
    for z in noisy:
        kf.predict(0.02)
        kf.update(float(z))

    assert abs(kf.value - truth[-1]) < 0.15
    assert abs(kf.rate - 0.5) < 0.2


def test_scalar_kalman_coasts_on_none():
    kf = ScalarKalman()
    kf.update(2.0)
    kf.update(2.2)
    before = kf.value
    kf.predict(0.02)
    kf.update(None)
    assert np.isfinite(kf.value)
    assert abs(kf.value - before) < 0.5


# ---------------------------------------------------------------------------
# HeightFusionKalman
# ---------------------------------------------------------------------------


def test_height_fusion_converges_to_range(rng):
    kf = HeightFusionKalman(q=0.5, r=0.01)
    true_h = 0.8
    for _ in range(300):
        kf.predict(0.02, accel_z=0.0)
        z = true_h + rng.normal(0.0, 0.02)
        kf.update(z)

    assert abs(kf.height - true_h) < 0.05
    assert abs(kf.velocity) < 0.1
    assert np.isfinite(kf.height)


def test_height_fusion_accel_prediction():
    kf = HeightFusionKalman(q=0.01, r=0.01)
    kf.update(0.0)
    for _ in range(50):
        kf.predict(0.02, accel_z=1.0)
    assert kf.height > 0.0
    assert kf.velocity > 0.0
