"""Tests for the real-time link (services/realtime).

PhysicsWorker is driven by a duck-typed fake engine (no MuJoCo): we assert it
steps autonomously, routes the latest input to the engine, publishes a fresh
frame each step, resets cleanly, and stops/joins its thread. Timing-tolerant by
polling rather than asserting exact step counts.
"""

from __future__ import annotations

import threading
import time

from services.navigation_engine import NavigationState
from services.realtime import PhysicsWorker


class FakeNavEngine:
    """Minimal NavigationEngine stand-in: counts steps, echoes last input."""

    def __init__(self) -> None:
        self.n = 0
        self.last_input = (0.0, 0.0)
        self.closed = False
        self._lock = threading.Lock()

    def reset(self) -> NavigationState:
        with self._lock:
            self.n = 0
        return self._state()

    def step(self, push: float, rotate: float) -> NavigationState:
        with self._lock:
            self.last_input = (push, rotate)
            self.n += 1
        return self._state()

    def get_render_bodies(self) -> list:
        return [{"pos": [0.0, 0.0, 0.0], "quat": [0.0, 0.0, 0.0, 1.0]}]

    def _state(self) -> NavigationState:
        return NavigationState(episode_length=self.n)

    def close(self) -> None:
        self.closed = True


def _wait_until(predicate, timeout=2.0, interval=0.01) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


def test_seed_publishes_initial_frame():
    worker = PhysicsWorker(FakeNavEngine())
    worker.seed(NavigationState(episode_length=0))
    frame = worker.snapshot()
    assert frame is not None
    assert frame.seq == 0
    assert frame.state.episode_length == 0
    assert len(frame.bodies) == 1


def test_autonomous_stepping_advances_seq():
    engine = FakeNavEngine()
    worker = PhysicsWorker(engine)
    worker.seed(engine.reset())
    worker.start()
    try:
        assert _wait_until(lambda: (worker.snapshot().seq if worker.snapshot() else 0) >= 3)
        frame = worker.snapshot()
        # episode_length tracks the engine's step count (frame published per step).
        assert frame.state.episode_length == frame.seq
    finally:
        worker.stop()


def test_set_input_reaches_engine():
    engine = FakeNavEngine()
    worker = PhysicsWorker(engine)
    worker.seed(engine.reset())
    worker.start()
    try:
        worker.set_input(1.0, -0.5)
        assert _wait_until(lambda: engine.last_input == (1.0, -0.5))
    finally:
        worker.stop()


def test_reset_publishes_zeroed_frame():
    engine = FakeNavEngine()
    worker = PhysicsWorker(engine)
    worker.seed(engine.reset())
    worker.start()
    try:
        assert _wait_until(lambda: (worker.snapshot().seq if worker.snapshot() else 0) >= 2)
        seq_before = worker.snapshot().seq
        state = worker.reset()
        assert state.episode_length == 0
        # reset publishes a new frame with a higher seq.
        assert worker.snapshot().seq > seq_before
    finally:
        worker.stop()


def test_stop_joins_thread():
    worker = PhysicsWorker(FakeNavEngine())
    worker.seed(NavigationState())
    worker.start()
    assert worker.is_running
    worker.stop()
    assert not worker.is_running
    # The worker thread does not close the engine (SessionManager owns lifecycle).
    assert worker._thread is None
