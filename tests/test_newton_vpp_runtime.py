"""Opt-in real Newton regression for the endpoint_1 VPP interaction loop.

Run with ``CATHSIM_RUN_NEWTON_INTEGRATION=1`` on a machine with Newton/Warp and
the case_001 VPP assets.  The ordinary unit suite remains lightweight and skips
this GPU-backed test when either prerequisite is absent.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
ROUTES = ROOT / "data/vpp_assets/case_001/derived/routes.json"

pytestmark = pytest.mark.skipif(
    os.environ.get("CATHSIM_RUN_NEWTON_INTEGRATION") != "1" or not ROUTES.exists(),
    reason="set CATHSIM_RUN_NEWTON_INTEGRATION=1 and install case_001 assets",
)


def _tip(engine) -> np.ndarray:
    return np.asarray(engine.render_bodies()[-1]["pos"], dtype=np.float64)


def _root(engine) -> np.ndarray:
    return np.asarray(engine.render_bodies()[0]["pos"], dtype=np.float64)


def test_endpoint1_force_drive_is_contained_reversible_and_steerable(monkeypatch):
    pytest.importorskip("newton")
    pytest.importorskip("warp")

    from services.physics.base import PlannedPath
    from services.physics.newton_engine import NewtonEngine

    payload = json.loads(ROUTES.read_text(encoding="utf-8"))
    route = payload["routes"]["endpoints_1"]
    path = PlannedPath(route["waypoints"], radii=route["radius_m"])

    monkeypatch.setenv("CATHSIM_NEWTON_SUBSTEPS", "8")
    monkeypatch.setenv("CATHSIM_NEWTON_ITERS", "4")
    monkeypatch.setenv("CATHSIM_NEWTON_SETTLE_STEPS", "480")
    engine = NewtonEngine(path=path)

    try:
        engine.reset()
        initial_tip = _tip(engine)
        initial_root = _root(engine)
        assert engine.diagnostics()["max_breach_m"] <= 0.0

        for _ in range(6):
            engine.step(0.0, 0.0)
        assert np.linalg.norm(_tip(engine) - initial_tip) < 0.0001
        assert engine.diagnostics()["max_breach_m"] <= 0.0

        engine.reset()
        initial_tip = _tip(engine)
        initial_root = _root(engine)
        for _ in range(6):
            engine.step(1.0, 0.0)
        assert engine.diagnostics()["insert_s_m"] == pytest.approx(0.01)
        assert engine.diagnostics()["max_breach_m"] <= 0.0
        for _ in range(6):
            engine.step(-1.0, 0.0)
        assert engine.diagnostics()["insert_s_m"] == pytest.approx(0.0)
        assert np.linalg.norm(_root(engine) - initial_root) < 1.0e-6
        assert np.linalg.norm(_tip(engine) - initial_tip) < 0.0005

        tips = []
        for rotate in (-1.0, 1.0):
            engine.reset()
            for _ in range(6):
                engine.step(0.0, rotate)
            assert engine.diagnostics()["max_breach_m"] <= 0.0
            tips.append(_tip(engine))
        assert np.linalg.norm(tips[0] - tips[1]) > 0.0002
    finally:
        engine.close()
