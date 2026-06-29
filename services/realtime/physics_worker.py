"""PhysicsWorker: step a navigation engine autonomously in a background thread.

Replaces the lock-step "one client control == one physics step" model for
real-time (batch_mode) sessions. The worker loops ``engine.step(latest_input)``
continuously; ``control`` messages only overwrite the latest input, they no
longer drive a step. A separate sender (on the event loop) reads the most recent
frame and pushes it to the client at a fixed rate.

Threading model:
- ``mj_step`` releases the GIL, so the worker thread runs concurrently with the
  asyncio event loop (heartbeats, receive, sender) -- the loop never blocks on
  physics, which is why this does not starve ping/pong.
- ``_engine_lock`` serializes *engine access* (a step vs. a reset) so the two
  never run concurrently on the same MuJoCo model. A reset waits at most one
  in-flight step.
- ``_data_lock`` is a short critical section guarding the latest input and the
  published frame, so the sender always reads a consistent snapshot.

The published :class:`Frame` carries a *copy* of the render bodies (extracted
under the engine lock by the worker thread), so the sender never touches live
``physics.data`` arrays the worker is mutating.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from services.navigation_engine import NavigationEngine, NavigationState


@dataclass
class Frame:
    """One published physics frame: the state, render bodies, and timing tags."""

    state: NavigationState
    bodies: list
    seq: int  # monotonically increasing physics frame number (for client interp)
    t_phys: float  # seconds since the worker started (physics wall clock)


class PhysicsWorker:
    """Steps a NavigationEngine continuously, publishing the latest frame.

    Not started on construction: call :meth:`seed` with the post-reset state so
    the sender has a frame immediately, then :meth:`start`.
    """

    def __init__(self, engine: NavigationEngine, max_step_rate: float = 30.0) -> None:
        """Create a worker over ``engine``.

        Args:
            engine: The session's navigation engine (already reset by the caller).
            max_step_rate: Upper bound on the step loop rate (Hz). Slow physics
                steps exceed the period and never sleep; cheap kinematic steps are
                throttled to this rate so they do not spin a CPU core. It also
                bounds how fast a held push advances a kinematic guidewire.
        """
        self._engine = engine
        self._min_step_period = 1.0 / max_step_rate if max_step_rate > 0 else 0.0

        self._engine_lock = threading.Lock()
        self._data_lock = threading.Lock()

        self._push = 0.0
        self._rotate = 0.0
        self._seq = 0
        self._frame: Frame | None = None

        self._running = False
        self._thread: threading.Thread | None = None
        self._t0 = time.monotonic()

    def seed(self, state: NavigationState) -> None:
        """Publish an initial frame (frame 0) from the post-reset state."""
        with self._engine_lock:
            bodies = self._engine.get_render_bodies()
        with self._data_lock:
            self._frame = Frame(state=state, bodies=bodies, seq=self._seq, t_phys=0.0)

    def set_input(self, push: float, rotate: float) -> None:
        """Atomically overwrite the latest control input (no step is triggered)."""
        with self._data_lock:
            self._push = float(push)
            self._rotate = float(rotate)

    def snapshot(self) -> Frame | None:
        """Return the most recently published frame (or None before the first)."""
        with self._data_lock:
            return self._frame

    def start(self) -> None:
        """Begin the autonomous step loop in a daemon thread."""
        if self._running:
            return
        self._running = True
        self._t0 = time.monotonic()
        self._thread = threading.Thread(
            target=self._run, name="physics-worker", daemon=True
        )
        self._thread.start()

    def _run(self) -> None:
        while self._running:
            t_start = time.monotonic()

            with self._data_lock:
                push, rotate = self._push, self._rotate

            with self._engine_lock:
                if not self._running:
                    break
                state = self._engine.step(push, rotate)
                bodies = self._engine.get_render_bodies()

            with self._data_lock:
                self._seq += 1
                self._frame = Frame(
                    state=state, bodies=bodies, seq=self._seq, t_phys=t_start - self._t0
                )

            # Cap the maximum loop rate; slow physics steps already exceed the
            # period (no sleep), cheap kinematic steps get throttled here.
            elapsed = time.monotonic() - t_start
            if self._min_step_period > elapsed:
                time.sleep(self._min_step_period - elapsed)

    def reset(self) -> NavigationState:
        """Reset the engine (serialized against stepping) and publish the frame."""
        with self._engine_lock:
            state = self._engine.reset()
            bodies = self._engine.get_render_bodies()
        with self._data_lock:
            self._seq += 1
            self._frame = Frame(
                state=state,
                bodies=bodies,
                seq=self._seq,
                t_phys=time.monotonic() - self._t0,
            )
        return state

    def stop(self) -> None:
        """Stop the step loop and join the thread.

        Does not close the engine -- the SessionManager owns the engine lifecycle
        and closes it when the session ends.
        """
        self._running = False
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=5.0)
        self._thread = None

    @property
    def is_running(self) -> bool:
        return self._running
