"""Unit tests for the PhysicsAutopilot control law (no MuJoCo required).

These verify the controller's decision logic in isolation -- aiming, push
gating, force attenuation, and stall detection -- on a synthetic straight path,
so the steering controller can be validated independently of whether any given
phantom's physical guidewire model can actually traverse its lumen.
"""

from __future__ import annotations

import numpy as np

from services.physics_autopilot import AutopilotConfig, PhysicsAutopilot


def _straight_path(n: int = 50, length: float = 0.5) -> np.ndarray:
    """A straight path along +z from the origin."""
    z = np.linspace(0.0, length, n)
    return np.stack([np.zeros_like(z), np.zeros_like(z), z], axis=1)


def test_pushes_hard_when_aligned():
    ap = PhysicsAutopilot(_straight_path())
    # Tip on the path, pointing straight down it.
    push, rotate = ap.compute([0, 0, 0.1], [0, 0, 1.0], contact_force=0.0)
    assert push >= ap.config.base_push - 1e-6
    assert abs(rotate) < 1e-6  # already aligned -> no steering


def test_eases_push_when_misaligned():
    ap = PhysicsAutopilot(_straight_path())
    # Tip pointing sideways relative to the path tangent.
    push, rotate = ap.compute([0, 0, 0.1], [1.0, 0, 0.0], contact_force=0.0)
    assert push < ap.config.base_push  # alignment gate reduces push
    assert push >= ap.config.min_push
    assert abs(rotate) > 0.0  # heading error drives steering


def test_force_attenuation_cuts_push():
    ap = PhysicsAutopilot(_straight_path())
    cfg = ap.config
    # Aligned but jammed hard -> push must be cut toward the floor.
    push, _ = ap.compute([0, 0, 0.1], [0, 0, 1.0], contact_force=cfg.force_hard + 1)
    assert push <= cfg.min_push + 1e-6


def test_stall_triggers_sweep_and_retract():
    cfg = AutopilotConfig(stall_window=5, retract_after=4)
    ap = PhysicsAutopilot(_straight_path(), config=cfg)
    ap.reset()
    # Feed the same off-path, stuck position repeatedly: no arc-length gain.
    stuck = [0.02, 0.0, 0.1]
    pushes = [ap.compute(stuck, [1.0, 0, 0.0], 3.0)[0] for _ in range(cfg.stall_window + 1)]
    # Once stalled, the controller enters the sweep (gentle push, not base push).
    assert pushes[-1] <= cfg.sweep_push + 1e-6
    # Pushing long enough into the stall eventually attempts a retract (push < 0).
    later = [ap.compute(stuck, [1.0, 0, 0.0], 3.0)[0] for _ in range(cfg.retract_after + 2)]
    assert min(later) < 0.0


def test_progress_resets_stall():
    cfg = AutopilotConfig(stall_window=5)
    ap = PhysicsAutopilot(_straight_path(), config=cfg)
    ap.reset()
    # Advance steadily down the path: deepest arc length must keep increasing and
    # never enter the stall branch (push stays >= min_push, sign stays forward).
    for i in range(20):
        z = 0.1 + i * 0.01
        push, _ = ap.compute([0, 0, z], [0, 0, 1.0], 0.0)
        assert push > 0.0
    assert ap.deepest_arclen > 0.25


def test_rejects_bad_path():
    import pytest

    with pytest.raises(ValueError):
        PhysicsAutopilot([[0, 0, 0]])  # only one point
