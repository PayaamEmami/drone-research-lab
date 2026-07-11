from __future__ import annotations

import time
from types import SimpleNamespace

import pytest

from drl import connection


class _ParamStub:
    def __init__(self, ready_at: float) -> None:
        self._ready_at = ready_at

    @property
    def is_updated(self) -> bool:
        return time.monotonic() >= self._ready_at


def test_wait_for_params_returns_true_when_ready() -> None:
    cf = SimpleNamespace(param=_ParamStub(time.monotonic() + 0.05))
    assert connection.wait_for_params(cf, timeout_s=1.0) is True


def test_wait_for_params_times_out() -> None:
    cf = SimpleNamespace(param=_ParamStub(time.monotonic() + 60.0))
    assert connection.wait_for_params(cf, timeout_s=0.05) is False


def test_require_decks_waits_for_late_detection(monkeypatch) -> None:
    cf = SimpleNamespace(
        param=SimpleNamespace(
            get_value=lambda _name: "1",
        )
    )
    link = connection.Link(scf=SimpleNamespace(cf=cf))
    seen: list[dict[str, bool]] = []
    original_decks = link.decks

    def decks() -> dict[str, bool]:
        result = original_decks()
        seen.append(result)
        if len(seen) == 1:
            return {}
        return {"flow2": True}

    monkeypatch.setattr(link, "decks", decks)
    link.require_decks("flow2", timeout_s=1.0)
    assert seen[-1]["flow2"] is True


def test_reset_estimator_timeout(monkeypatch) -> None:
    cf = SimpleNamespace(
        param=SimpleNamespace(
            set_value=lambda *_args, **_kwargs: None,
        )
    )
    scf = SimpleNamespace(cf=cf)

    class _NeverConverges:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def __iter__(self):
            tick = 0
            while True:
                tick += 1
                yield (
                    0,
                    {
                        "kalman.varPX": float(tick),
                        "kalman.varPY": float(tick),
                        "kalman.varPZ": float(tick),
                    },
                )

    monkeypatch.setattr(connection, "SyncLogger", lambda *_args, **_kwargs: _NeverConverges())

    with pytest.raises(RuntimeError, match="did not settle"):
        connection._reset_estimator_with_timeout(scf, timeout_s=0.05)
