"""Wiring tests for ShapeIntent (autopilot) control in NavigationEngine.

These exercise the real NavigationEngine construction/step path with a fake
physics backend (no MuJoCo/Newton), verifying that engaging an intent makes
``step`` derive push/rotate from the ShapeIntentController while disengaging
restores manual control -- the doc/09 §五 seam.
"""

from __future__ import annotations

import numpy as np
import pytest

from services.physics.base import RawPose


class FakeEngine:
    """Minimal PhysicsEngine stub: records the (push, rotate) it is stepped with
    and advances a tip straight down +z proportional to accumulated push."""

    def __init__(self, **kwargs):
        self._z = 0.1
        self.calls: list[tuple[float, float]] = []

    def _pose(self) -> RawPose:
        return RawPose(
            tip_position=[0.0, 0.0, self._z],
            tip_direction=[0.0, 0.0, 1.0],
            target_position=[0.0, 0.0, 0.5],
        )

    def reset(self) -> RawPose:
        self._z = 0.1
        self.calls.clear()
        return self._pose()

    def step(self, push: float, rotate: float) -> RawPose:
        self.calls.append((push, rotate))
        self._z += max(push, 0.0) * 0.01
        return self._pose()

    def render_bodies(self):
        return []

    def close(self):
        pass

    @property
    def control_timestep(self) -> float:
        return 0.033


def _straight_path(n: int = 50, length: float = 0.5):
    z = np.linspace(0.0, length, n)
    return np.stack([np.zeros_like(z), np.zeros_like(z), z], axis=1).tolist()


def _make_engine(monkeypatch):
    import services.navigation_engine as ne

    monkeypatch.setattr(ne, "make_engine", lambda **kw: FakeEngine(**kw))
    engine = ne.NavigationEngine(phantom="low_tort", planned_path=_straight_path())
    engine.reset()
    return engine


def test_manual_control_passes_through(monkeypatch):
    engine = _make_engine(monkeypatch)
    engine.step(0.3, 0.1)
    assert engine._engine.calls[-1] == pytest.approx((0.3, 0.1))


def test_centerline_intent_drives_push(monkeypatch):
    engine = _make_engine(monkeypatch)
    result = engine.set_shape_intent(None, active=True)
    assert result == {"active": True, "mode": "centerline"}
    # Manual (0, 0) is overridden by the autopilot: aligned down the path -> push.
    engine.step(0.0, 0.0)
    push, rotate = engine._engine.calls[-1]
    assert push > 0.0
    assert abs(rotate) < 1e-6


def test_disengage_restores_manual(monkeypatch):
    engine = _make_engine(monkeypatch)
    engine.set_shape_intent(None, active=True)
    engine.step(0.0, 0.0)
    off = engine.set_shape_intent(None, active=False)
    assert off == {"active": False, "mode": "off"}
    engine.step(0.25, -0.2)
    assert engine._engine.calls[-1] == pytest.approx((0.25, -0.2))


def test_waypoint_intent_steers(monkeypatch):
    engine = _make_engine(monkeypatch)
    res = engine.set_shape_intent({"target_waypoint": [0.1, 0.0, 0.15]}, active=True)
    assert res["mode"] == "waypoint"
    engine.step(0.0, 0.0)
    _, rotate = engine._engine.calls[-1]
    assert abs(rotate) > 0.0


def test_guided_mode_rejects_intent(monkeypatch):
    import services.navigation_engine as ne
    from services.physics.kinematic_engine import KinematicEngine

    class FakeKinematic(KinematicEngine):
        """Kinematic backend for the isinstance() guided check, without the real
        (path-building) constructor."""

        def __init__(self, **kwargs):
            self._z = 0.1

        def reset(self):
            return FakeEngine._pose(self)

        def step(self, push, rotate):
            return FakeEngine._pose(self)

        def render_bodies(self):
            return []

        def close(self):
            pass

        @property
        def control_timestep(self):
            return 0.033

    # A guided session uses the kinematic backend; intent control is unavailable.
    monkeypatch.setattr(ne, "make_engine", lambda **kw: FakeKinematic())
    engine = ne.NavigationEngine(
        phantom="low_tort", planned_path=_straight_path(), guided=True
    )
    engine.reset()
    result = engine.set_shape_intent(None, active=True)
    assert result == {"active": False, "mode": "off"}
