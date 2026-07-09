"""Tests for the PhysicsEngine abstraction (services/physics).

The headline payoff of the abstraction is testability *without MuJoCo*: a
``FakeEngine`` returns scripted :class:`RawPose` values, and we assert that
``NavigationEngine`` derives path progress / deviation / curvature / safety /
risk correctly from them. These run in milliseconds and never compile an MJCF.

The file also covers the seam's other pieces directly: the ``make_engine``
factory's backend choice, ``PlannedPath`` arc-length geometry, and the
``KinematicEngine`` raw-pose contract.
"""

from __future__ import annotations

import numpy as np
import pytest

from services.navigation_engine import NavigationEngine
from services.physics import (
    KinematicEngine,
    MuJoCoEngine,
    PlannedPath,
    RawPose,
    make_engine,
)

# A simple 3 m straight path along +x with unit-spaced vertices.
STRAIGHT_PATH = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]]


class FakeEngine:
    """Scripted PhysicsEngine: returns the next queued RawPose on each call.

    Lets us drive NavigationEngine's derived-quantity layer with no physics. The
    last pose is repeated once the script is exhausted so over-stepping is safe.
    """

    def __init__(self, poses: list[RawPose], control_timestep: float = 0.1) -> None:
        self._poses = list(poses)
        self._i = 0
        self._control_timestep = control_timestep
        self.closed = False

    @property
    def control_timestep(self) -> float:
        return self._control_timestep

    def _next(self) -> RawPose:
        pose = self._poses[min(self._i, len(self._poses) - 1)]
        self._i += 1
        return pose

    def reset(self) -> RawPose:
        self._i = 0
        return self._next()

    def step(self, push: float, rotate: float) -> RawPose:
        return self._next()

    def render_bodies(self) -> list[dict[str, list[float]]]:
        return []

    def close(self) -> None:
        self.closed = True


def _nav_with_fake(poses: list[RawPose], **fake_kwargs) -> tuple[NavigationEngine, FakeEngine]:
    """Build a NavigationEngine over a straight path, then swap in a FakeEngine.

    Construction uses guided=False so the real backend is a *lazy* MuJoCoEngine
    (no MJCF compiled until reset, which we never call on it). We then replace
    the backend with the fake so reset/step exercise NavigationEngine's derived
    layer only.
    """
    nav = NavigationEngine(planned_path=STRAIGHT_PATH, guided=False)
    fake = FakeEngine(poses, **fake_kwargs)
    nav._engine = fake
    return nav, fake


# ---------------------------------------------------------------------------
# NavigationEngine derived layer, driven by a fake engine (no MuJoCo)
# ---------------------------------------------------------------------------


class TestDerivedStateWithFakeEngine:
    def test_progress_from_tip_projection_when_no_arclen(self):
        # arclen=None -> progress/deviation come from projecting the tip.
        nav, _ = _nav_with_fake([RawPose(tip_position=[2.0, 0.0, 0.0])])
        state = nav.reset()
        assert state.path_progress == pytest.approx(2.0 / 3.0)
        assert state.path_deviation == pytest.approx(0.0)

    def test_progress_from_arclen_is_exact_and_continuous(self):
        # arclen set (kinematic style) -> exact progress, zero deviation, even
        # when the tip sits between path vertices.
        nav, _ = _nav_with_fake([RawPose(tip_position=[1.5, 0.0, 0.0], arclen=1.5)])
        state = nav.reset()
        assert state.path_progress == pytest.approx(0.5)
        assert state.path_deviation == pytest.approx(0.0)

    def test_deviation_off_path(self):
        nav, _ = _nav_with_fake([RawPose(tip_position=[2.0, 0.5, 0.0])])
        state = nav.reset()
        assert state.path_deviation == pytest.approx(0.5)

    def test_safety_standby_on_reset(self):
        nav, _ = _nav_with_fake([RawPose(wall_distance=0.0001)])
        state = nav.reset()  # episode_length 0 -> STANDBY regardless of distance
        assert state.safety_status == "STANDBY"

    def test_safety_bands_after_step(self):
        poses = [
            RawPose(),  # reset
            RawPose(wall_distance=0.05),  # free space
            RawPose(wall_distance=0.0008),  # between DANGER and SAFE
            RawPose(wall_distance=0.0003),  # below DANGER
        ]
        nav, _ = _nav_with_fake(poses)
        nav.reset()
        assert nav.step(1.0, 0.0).safety_status == "SAFE_NAV"
        assert nav.step(1.0, 0.0).safety_status == "DANGER_WARNING"
        assert nav.step(1.0, 0.0).safety_status == "COLLISION_STOP"

    def test_risk_score_is_populated(self):
        nav, _ = _nav_with_fake([RawPose(), RawPose(wall_distance=0.0003, contact_force=5.0)])
        nav.reset()
        state = nav.step(1.0, 0.0)
        assert state.risk_score > 0.0

    def test_curvature_from_tip_history(self):
        # Right-angle path through the tip history -> positive Menger curvature.
        poses = [
            RawPose(tip_position=[0.0, 0.0, 0.0]),
            RawPose(tip_position=[1.0, 0.0, 0.0]),
            RawPose(tip_position=[1.0, 1.0, 0.0]),
        ]
        nav, _ = _nav_with_fake(poses)
        nav.reset()
        nav.step(1.0, 0.0)
        state = nav.step(1.0, 0.0)
        assert state.curvature > 0.0

    def test_velocity_uses_engine_control_timestep(self):
        # Tip moves 0.2 m between steps; control_timestep 0.1 s -> 2.0 m/s.
        poses = [
            RawPose(tip_position=[0.0, 0.0, 0.0]),
            RawPose(tip_position=[0.2, 0.0, 0.0]),
        ]
        nav, _ = _nav_with_fake(poses, control_timestep=0.1)
        nav.reset()
        state = nav.step(1.0, 0.0)
        assert state.velocity == pytest.approx(2.0)

    def test_episode_length_and_passthrough(self):
        nav, _ = _nav_with_fake([RawPose(), RawPose(reward=1.5, done=True)])
        nav.reset()
        state = nav.step(1.0, 0.0)
        assert state.episode_length == 1
        assert state.reward == pytest.approx(1.5)
        assert state.done is True

    def test_close_delegates_to_engine(self):
        nav, fake = _nav_with_fake([RawPose()])
        nav.close()
        assert fake.closed is True


# ---------------------------------------------------------------------------
# Factory backend selection
# ---------------------------------------------------------------------------


class TestMakeEngine:
    def test_guided_with_path_is_kinematic(self):
        engine = make_engine(guided=True, path=PlannedPath(STRAIGHT_PATH), phantom="x", target="y")
        assert isinstance(engine, KinematicEngine)

    def test_guided_without_path_falls_back_to_physics(self):
        engine = make_engine(guided=True, path=None, phantom="x", target="y")
        assert isinstance(engine, MuJoCoEngine)

    def test_not_guided_is_physics(self):
        engine = make_engine(guided=False, path=PlannedPath(STRAIGHT_PATH), phantom="x", target="y")
        assert isinstance(engine, MuJoCoEngine)

    def test_zero_length_path_falls_back_to_physics(self):
        degenerate = PlannedPath([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        engine = make_engine(guided=True, path=degenerate, phantom="x", target="y")
        assert isinstance(engine, MuJoCoEngine)

    def test_mujoco_mode_overrides_guided(self):
        engine = make_engine(
            guided=True,
            path=PlannedPath(STRAIGHT_PATH),
            phantom="x",
            target="y",
            engine_mode="mujoco",
        )
        assert isinstance(engine, MuJoCoEngine)

    def test_guided_mode_overrides_physics(self):
        engine = make_engine(
            guided=False,
            path=PlannedPath(STRAIGHT_PATH),
            phantom="x",
            target="y",
            engine_mode="guided",
        )
        assert isinstance(engine, KinematicEngine)

    def test_unknown_engine_mode_is_rejected(self):
        with pytest.raises(ValueError):
            make_engine(
                guided=False,
                path=PlannedPath(STRAIGHT_PATH),
                phantom="x",
                target="y",
                engine_mode="bogus",
            )


# ---------------------------------------------------------------------------
# PlannedPath geometry
# ---------------------------------------------------------------------------


class TestPlannedPath:
    def test_total_length(self):
        assert PlannedPath(STRAIGHT_PATH).total_len == pytest.approx(3.0)

    def test_point_at_arclen_interpolates(self):
        path = PlannedPath(STRAIGHT_PATH)
        assert path.point_at_arclen(1.5) == pytest.approx([1.5, 0.0, 0.0])

    def test_point_at_arclen_clamps(self):
        path = PlannedPath(STRAIGHT_PATH)
        assert path.point_at_arclen(-1.0) == pytest.approx([0.0, 0.0, 0.0])
        assert path.point_at_arclen(99.0) == pytest.approx([3.0, 0.0, 0.0])

    def test_tangent_is_unit(self):
        path = PlannedPath(STRAIGHT_PATH)
        assert path.tangent_at_arclen(1.5) == pytest.approx([1.0, 0.0, 0.0])

    def test_progress_deviation(self):
        path = PlannedPath(STRAIGHT_PATH)
        progress, deviation = path.progress_deviation([2.0, 0.0, 0.0])
        assert progress == pytest.approx(2.0 / 3.0)
        assert deviation == pytest.approx(0.0)

    def test_inner_wall_offset_zero_on_straight(self):
        path = PlannedPath(STRAIGHT_PATH)
        offset = path.inner_wall_offset(1.5, wall_lean=0.0025, gain=1.5)
        assert float(np.linalg.norm(offset)) == pytest.approx(0.0, abs=1e-9)

    def test_inner_wall_offset_capped_on_bend(self):
        path = PlannedPath([[0, 0, 0], [1, 0, 0], [1, 1, 0]])
        offset = path.inner_wall_offset(1.0, wall_lean=0.0025, gain=1.5)
        mag = float(np.linalg.norm(offset))
        assert mag > 0.0
        assert mag <= 0.0025 + 1e-9


# ---------------------------------------------------------------------------
# KinematicEngine raw-pose contract (no MuJoCo)
# ---------------------------------------------------------------------------


class TestKinematicEngine:
    def _engine(self, **kwargs) -> KinematicEngine:
        return KinematicEngine(path=PlannedPath(STRAIGHT_PATH), advance_per_step=0.1, **kwargs)

    def test_reset_at_entry_reports_arclen(self):
        raw = self._engine().reset()
        assert raw.tip_position == pytest.approx([0.0, 0.0, 0.0])
        assert raw.arclen == pytest.approx(0.0)
        assert raw.wall_distance == NavigationEngine.MAX_WALL_DISTANCE
        assert raw.done is False

    def test_push_advances_arclen_to_done(self):
        engine = self._engine()
        engine.reset()
        raw = None
        for _ in range(200):
            raw = engine.step(1.0, 0.0)
            if raw.done:
                break
        assert raw.done is True
        assert raw.arclen == pytest.approx(3.0)
        assert raw.tip_position == pytest.approx([3.0, 0.0, 0.0])

    def test_retract_decreases_arclen(self):
        engine = self._engine()
        engine.reset()
        for _ in range(10):
            engine.step(1.0, 0.0)
        advanced = engine.step(0.0, 0.0).arclen
        retracted = engine.step(-1.0, 0.0).arclen
        assert retracted < advanced

    def test_render_bodies_span_inserted_length(self):
        engine = self._engine()
        engine.reset()
        for _ in range(15):
            engine.step(1.0, 0.0)  # s = 1.5 along the 3 m path
        bodies = engine.render_bodies()
        assert len(bodies) > 1
        assert bodies[0]["pos"][0] == pytest.approx(0.0, abs=1e-3)
        assert bodies[-1]["pos"][0] == pytest.approx(1.5, abs=1e-2)
