"""Unit tests for the ShapeIntent control abstraction (no MuJoCo required).

Two things are verified: (1) with ``intent=None`` the ShapeIntentController is
frame-for-frame identical to the underlying PhysicsAutopilot (the doc/09 §五
regression guarantee -- wiring it in front of the autopilot changes nothing);
and (2) a supplied intent actually redirects the aim (waypoint / direction /
intensity) using the same validated control law.
"""

from __future__ import annotations

import numpy as np
import pytest

from services.physics_autopilot import PhysicsAutopilot
from services.shape_intent import ShapeIntent, ShapeIntentController


def _straight_path(n: int = 50, length: float = 0.5) -> np.ndarray:
    z = np.linspace(0.0, length, n)
    return np.stack([np.zeros_like(z), np.zeros_like(z), z], axis=1)


def test_none_intent_matches_raw_autopilot_frame_by_frame():
    path = _straight_path()
    ap = PhysicsAutopilot(path)
    ctl = ShapeIntentController(path)
    ap.reset()
    ctl.reset()
    rng = np.random.default_rng(0)
    for _ in range(60):
        # Same synthetic trajectory into both controllers.
        tip = [rng.uniform(-0.02, 0.02), rng.uniform(-0.02, 0.02), rng.uniform(0, 0.5)]
        d = rng.uniform(-1, 1, 3)
        f = rng.uniform(0, 30)
        a = ap.compute(tip, d, f)
        b = ctl.compute(None, tip, d, f)
        assert a == b


def test_waypoint_intent_aims_off_path():
    path = _straight_path()
    ctl = ShapeIntentController(path)
    ctl.reset()
    # Tip on the path aimed straight down it, but the intent points to a
    # waypoint off to +x -> steering must engage (rotate != 0).
    intent = ShapeIntent(target_waypoint=[0.1, 0.0, 0.1])
    _, rotate = ctl.compute(intent, [0, 0, 0.1], [0, 0, 1.0], 0.0)
    assert abs(rotate) > 0.0


def test_direction_intent_overrides_centerline():
    path = _straight_path()
    ctl = ShapeIntentController(path)
    ctl.reset()
    # Aligned to the path, but the intent wants a sideways heading -> the
    # controller sees a heading error and steers, unlike the None case.
    _, rotate_none = ShapeIntentController(path).compute(None, [0, 0, 0.1], [0, 0, 1.0], 0.0)
    intent = ShapeIntent(target_direction=[1.0, 0.0, 0.0])
    _, rotate_dir = ctl.compute(intent, [0, 0, 0.1], [0, 0, 1.0], 0.0)
    assert abs(rotate_none) < 1e-6
    assert abs(rotate_dir) > 0.0


def test_intensity_scales_push():
    path = _straight_path()
    full = ShapeIntentController(path)
    half = ShapeIntentController(path)
    full.reset()
    half.reset()
    # Aligned down the path (full push regime), differing only in intensity.
    p_full, _ = full.compute(ShapeIntent(intensity=1.0), [0, 0, 0.1], [0, 0, 1.0], 0.0)
    p_half, _ = half.compute(ShapeIntent(intensity=0.5), [0, 0, 0.1], [0, 0, 1.0], 0.0)
    assert p_half < p_full
    assert p_half >= full.config.min_push - 1e-9


def test_direction_takes_precedence_over_waypoint():
    path = _straight_path()
    ctl = ShapeIntentController(path)
    intent = ShapeIntent(target_direction=[0, 0, 1.0], target_waypoint=[1.0, 0, 0.1])
    # Direction is down the path -> aligned -> no steering, proving the
    # waypoint (off to +x) was ignored.
    _, rotate = ctl.compute(intent, [0, 0, 0.1], [0, 0, 1.0], 0.0)
    assert abs(rotate) < 1e-6


def test_intent_validates_shapes():
    with pytest.raises(ValueError):
        ShapeIntent(target_direction=[1.0, 0.0])
    with pytest.raises(ValueError):
        ShapeIntent(target_waypoint=[0.0, 0.0, 0.0, 1.0])
