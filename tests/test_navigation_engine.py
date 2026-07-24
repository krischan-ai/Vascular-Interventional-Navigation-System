"""Tests for NavigationEngine and Session API.

These tests verify the CathSim bridge integration. Tests involving
the actual MuJoCo environment are marked with pytest.mark.slow
and can be skipped with `pytest -m "not slow"`.
"""

import pytest
from fastapi.testclient import TestClient

from services.main import app


# ============================================================================
# Unit Tests (No MuJoCo required)
# ============================================================================


class TestNavigationStateDataclass:
    """Test NavigationState dataclass."""

    def test_default_values(self):
        from services.navigation_engine import NavigationState

        state = NavigationState()

        assert state.tip_position == [0.0, 0.0, 0.0]
        assert state.tip_direction == [0.0, 0.0, 1.0]
        assert state.velocity == 0.0
        assert state.contact_force == 0.0
        assert state.episode_length == 0
        assert state.reward == 0.0
        assert state.done is False

    def test_as_dict(self):
        from services.navigation_engine import NavigationState

        state = NavigationState(
            tip_position=[1.0, 2.0, 3.0],
            velocity=0.5,
            episode_length=10,
        )
        result = state.as_dict()

        assert result["tip_position"] == [1.0, 2.0, 3.0]
        assert result["velocity"] == 0.5
        assert result["episode_length"] == 10
        assert "done" in result
        assert "reward" in result
        assert result["remaining_distance"] == 0.0
        assert result["path_total_distance"] is None
        assert result["path_travelled_distance"] is None
        assert result["vessel_radius"] is None
        assert result["fidelity_mode"] == "physics"
        assert result["risk_regions"] == []


class TestSessionManagerUnit:
    """Unit tests for SessionManager without MuJoCo."""

    def test_max_sessions_config(self):
        from services.session_manager import SessionManager

        manager = SessionManager(max_sessions=5)
        assert manager.max_sessions == 5
        assert manager.active_session_count == 0

    def test_close_nonexistent_session(self):
        from services.session_manager import SessionManager

        manager = SessionManager()
        result = manager.close_session("nonexistent-id")
        assert result is False

    def test_get_nonexistent_session(self):
        from services.session_manager import SessionManager

        manager = SessionManager()
        with pytest.raises(KeyError):
            manager.get_session("nonexistent-id")

    def test_list_sessions_empty(self):
        from services.session_manager import SessionManager

        manager = SessionManager()
        sessions = manager.list_sessions()
        assert sessions == []

    def test_emergency_stop_latches_and_rejects_nonzero_control(self, monkeypatch):
        from services.navigation_engine import NavigationState
        import services.session_manager as session_manager_module

        class DummyEngine:
            def __init__(self, **_kwargs):
                self.intent_calls = []
                self.step_calls = []
                self.params = {"jtip_deg": 35.0}

            def reset(self):
                return NavigationState()

            def step(self, push, rotate):
                self.step_calls.append((push, rotate))
                return NavigationState(episode_length=len(self.step_calls))

            def set_shape_intent(self, intent, active=True):
                self.intent_calls.append((intent, active))
                return {"active": active, "mode": "off" if not active else "centerline"}

            def set_engine_params(self, params):
                self.params.update(params)
                return dict(self.params)

            def close(self):
                pass

        monkeypatch.setattr(session_manager_module, "NavigationEngine", DummyEngine)
        manager = session_manager_module.SessionManager()
        session_id, initial = manager.create_session()
        engine = manager.get_session(session_id)

        stopped = manager.emergency_stop(session_id)
        assert stopped["emergency_stop_latched"] is True
        assert engine.intent_calls[-1] == (None, False)

        with pytest.raises(session_manager_module.ControlRejectedError):
            manager.step(session_id, 0.4, 0.0)
        assert engine.step_calls == []
        assert manager.step(session_id, 0.0, 0.0) is initial
        assert engine.step_calls == []

        resumed = manager.resume(session_id)
        assert resumed["emergency_stop_latched"] is False
        state = manager.step(session_id, 0.4, 0.0)
        assert state.episode_length == 1
        assert engine.step_calls == [(0.4, 0.0)]

    def test_control_protections_apply_backend_effective_values(self, monkeypatch):
        from services.navigation_engine import NavigationState
        import services.session_manager as session_manager_module

        class DummyEngine:
            def __init__(self, **_kwargs):
                self.step_calls = []
                self.params = {"jtip_deg": 42.0}

            def reset(self):
                return NavigationState(path_progress=0.0, path_travelled_distance=0.0)

            def step(self, push, rotate):
                self.step_calls.append((push, rotate))
                return NavigationState(path_progress=0.1, path_travelled_distance=0.01)

            def set_shape_intent(self, _intent, active=True):
                return {"active": active}

            def set_engine_params(self, params):
                self.params.update(params)
                return dict(self.params)

            def close(self):
                pass

        monkeypatch.setattr(session_manager_module, "NavigationEngine", DummyEngine)
        manager = session_manager_module.SessionManager()
        session_id, _ = manager.create_session()
        engine = manager.get_session(session_id)

        manager.step(session_id, -0.3, 1.0)
        assert engine.step_calls[-1] == (0.0, 0.5)
        applied = manager.get_control_state(session_id)["last_applied_control"]
        assert set(applied["protection_reasons"]) == {
            "WITHDRAWAL_AT_ENTRY_BLOCKED",
            "TORQUE_COMMAND_LIMITED",
        }

        disabled = manager.update_control_config(
            session_id,
            jtip_assist_enabled=False,
            torque_limit_enabled=False,
        )
        assert disabled["protections"]["jtip_assist_enabled"] is False
        assert engine.params["jtip_deg"] == 0.0
        manager.update_control_config(session_id, jtip_assist_enabled=True)
        assert engine.params["jtip_deg"] == 42.0


class TestSchemas:
    """Test Pydantic schemas."""

    def test_step_request_validation(self):
        from services.schemas import StepRequest

        request = StepRequest(delta_push=0.5, delta_rotate=-0.3)
        assert request.delta_push == 0.5
        assert request.delta_rotate == -0.3

    def test_step_request_out_of_range(self):
        from services.schemas import StepRequest
        from pydantic import ValidationError

        with pytest.raises(ValidationError):
            StepRequest(delta_push=1.5, delta_rotate=0.0)

        with pytest.raises(ValidationError):
            StepRequest(delta_push=0.0, delta_rotate=-2.0)

    def test_session_start_request_defaults(self):
        from services.schemas import SessionStartRequest

        request = SessionStartRequest()
        assert request.phantom == "low_tort"
        assert request.target == "bca"
        assert request.use_pixels is False

    def test_state_response_contains_dashboard_fields(self):
        from services.main import _state_to_response
        from services.navigation_engine import NavigationState

        state = NavigationState(
            remaining_distance=0.12,
            path_total_distance=0.2,
            path_travelled_distance=0.08,
            vessel_radius=0.003,
            eta_seconds=2.5,
            fidelity_mode="guided",
            risk_score=0.4,
            risk_regions=[{"level": "warning"}],
        )
        response = _state_to_response(state)

        assert response.remaining_distance == 0.12
        assert response.path_total_distance == 0.2
        assert response.path_travelled_distance == 0.08
        assert response.vessel_radius == 0.003
        assert response.eta_seconds == 2.5
        assert response.fidelity_mode == "guided"
        assert response.risk_score == 0.4
        assert response.risk_regions == [{"level": "warning"}]


class TestNavigationStateExtended:
    """Tests for the extended NavigationState fields (Stage 7)."""

    def test_extended_defaults(self):
        from services.navigation_engine import NavigationState

        state = NavigationState()
        assert state.tip_quaternion == [0.0, 0.0, 0.0, 1.0]
        assert state.wall_distance == 0.0
        assert state.curvature == 0.0
        assert state.path_progress == 0.0
        assert state.path_deviation == 0.0
        assert state.safety_status == "STANDBY"

    def test_as_dict_contains_extended_fields(self):
        from services.navigation_engine import NavigationState

        result = NavigationState().as_dict()
        for key in (
            "tip_quaternion",
            "wall_distance",
            "curvature",
            "path_progress",
            "path_deviation",
            "remaining_distance",
            "path_total_distance",
            "path_travelled_distance",
            "vessel_radius",
            "eta_seconds",
            "latency_ms",
            "fidelity_mode",
            "risk_score",
            "risk_regions",
            "safety_status",
        ):
            assert key in result


class TestNavigationEngineHelpers:
    """Unit tests for pure NavigationEngine helpers (no MuJoCo required)."""

    def _engine(self, **kwargs):
        from services.navigation_engine import NavigationEngine

        return NavigationEngine(**kwargs)

    def test_no_planned_path_returns_zero(self):
        engine = self._engine()
        assert engine._compute_path_progress([1.0, 2.0, 3.0]) == (0.0, 0.0)

    def test_planned_path_progress_midpoint(self):
        engine = self._engine(planned_path=[[0, 0, 0], [0, 0, 1], [0, 0, 2]])
        progress, deviation = engine._compute_path_progress([0.0, 0.0, 1.0])
        assert progress == pytest.approx(0.5)
        assert deviation == pytest.approx(0.0)

    def test_planned_path_progress_endpoint_with_deviation(self):
        engine = self._engine(planned_path=[[0, 0, 0], [0, 0, 1], [0, 0, 2]])
        progress, deviation = engine._compute_path_progress([0.5, 0.0, 2.0])
        assert progress == pytest.approx(1.0)
        assert deviation == pytest.approx(0.5)

    def test_remaining_distance_and_radius_from_path(self):
        engine = self._engine(planned_path=[[0, 0, 0], [0, 0, 1], [0, 0, 2]])
        engine.set_planned_path(
            [[0, 0, 0], [0, 0, 1], [0, 0, 2]],
            radii=[0.003, 0.004, 0.005],
        )

        assert engine._compute_remaining_distance(0.25) == pytest.approx(1.5)
        assert engine._compute_vessel_radius(0.5) == pytest.approx(0.004)
        assert engine._compute_eta_seconds(1.5, 0.5) == pytest.approx(3.0)
        assert engine._compute_eta_seconds(1.5, 0.0) is None

    def test_risk_regions_empty_until_source_backed_data(self):
        engine = self._engine(planned_path=[[0, 0, 0], [0, 0, 1], [0, 0, 2]])
        assert engine._risk_regions(0.5, engine.WALL_DISTANCE_SAFE * 2.0) == []

    def test_set_planned_path_clear(self):
        engine = self._engine(planned_path=[[0, 0, 0], [0, 0, 1]])
        assert engine._path is not None
        assert engine._path.total_len > 0
        engine.set_planned_path(None)
        assert engine._compute_path_progress([0.0, 0.0, 0.5]) == (0.0, 0.0)

    def test_curvature_insufficient_history(self):
        engine = self._engine()
        engine._tip_history.extend([[0, 0, 0], [1, 0, 0]])
        assert engine._compute_curvature() == 0.0

    def test_curvature_collinear_is_zero(self):
        engine = self._engine()
        engine._tip_history.extend([[0, 0, 0], [1, 0, 0], [2, 0, 0]])
        assert engine._compute_curvature() == 0.0

    def test_curvature_right_angle_positive(self):
        engine = self._engine()
        engine._tip_history.extend([[0, 0, 0], [1, 0, 0], [1, 1, 0]])
        assert engine._compute_curvature() > 0.0

    def test_safety_status_bands(self):
        engine = self._engine()
        assert engine._compute_safety_status(0, 0.0) == "STANDBY"
        assert engine._compute_safety_status(5, 0.05) == "SAFE_NAV"
        assert engine._compute_safety_status(5, 0.0007) == "DANGER_WARNING"
        assert engine._compute_safety_status(5, 0.0001) == "COLLISION_STOP"

    def test_safety_status_guided_bands_unchanged(self):
        # Non-force backend (low_tort -> MuJoCo) keeps the clearance bands: a
        # sub-0.5mm wall distance is a collision because the wire rides centered.
        engine = self._engine()
        assert engine._is_force_physics() is False
        assert engine._compute_safety_status(5, 0.0001) == "COLLISION_STOP"

    def test_safety_status_force_mode_wall_hug_is_safe(self):
        # Force-drive: the wire legitimately hugs the wall, so a tiny wall_distance
        # with NO contact force is normal, not a collision. Regression for the
        # D5/ShapeIntent smoke's false COLLISION_STOP (doc/09 §9.5).
        from types import SimpleNamespace

        engine = self._engine()
        engine._engine = SimpleNamespace(is_force_drive=True, contact_ke=3.0e6, close=lambda: None)
        assert engine._is_force_physics() is True
        assert engine._compute_safety_status(5, 0.0001, 0.0) == "SAFE_NAV"

    def test_safety_status_force_mode_penetration_bands(self):
        # Force-drive safety keys on penetration = contact_force / contact_ke.
        from types import SimpleNamespace

        ke = 3.0e6
        engine = self._engine()
        engine._engine = SimpleNamespace(is_force_drive=True, contact_ke=ke, close=lambda: None)
        # 0.02mm penetration < BREACH_WARN (0.05mm) -> numerical noise, safe.
        assert engine._compute_safety_status(5, 0.0, 0.00002 * ke) == "SAFE_NAV"
        # 0.10mm penetration in [WARN, STOP) -> warning.
        assert engine._compute_safety_status(5, 0.0, 0.0001 * ke) == "DANGER_WARNING"
        # 0.50mm penetration >= BREACH_STOP (0.30mm) -> collision stop.
        assert engine._compute_safety_status(5, 0.0, 0.0005 * ke) == "COLLISION_STOP"

    def test_resolve_entry_none_without_path(self):
        engine = self._engine()
        point, direction = engine._resolve_entry()
        assert point is None
        assert direction is None

    def test_resolve_entry_from_planned_path(self):
        engine = self._engine(planned_path=[[1, 0, 0], [1, 0, 1], [1, 0, 2]])
        point, direction = engine._resolve_entry()
        assert list(point) == [1.0, 0.0, 0.0]
        # Direction is the first non-degenerate segment.
        assert list(direction) == [0.0, 0.0, 1.0]

    def test_resolve_entry_skips_degenerate_first_segment(self):
        # First two points coincide; direction must come from the next segment.
        engine = self._engine(planned_path=[[0, 0, 0], [0, 0, 0], [0, 1, 0]])
        point, direction = engine._resolve_entry()
        assert list(point) == [0.0, 0.0, 0.0]
        assert list(direction) == [0.0, 1.0, 0.0]

    def test_resolve_entry_explicit_overrides_path(self):
        engine = self._engine(
            planned_path=[[1, 0, 0], [1, 0, 1]],
            entry_point=[5, 5, 5],
            entry_direction=[1, 0, 0],
        )
        point, direction = engine._resolve_entry()
        assert list(point) == [5.0, 5.0, 5.0]
        assert list(direction) == [1.0, 0.0, 0.0]

    def test_entry_pose_empty_without_path(self):
        engine = self._engine()
        pose = engine.entry_pose
        assert pose["position"] == []
        assert pose["direction"] == []

    def test_entry_pose_from_planned_path_is_normalized(self):
        engine = self._engine(planned_path=[[1, 0, 0], [1, 0, 2], [1, 0, 4]])
        pose = engine.entry_pose
        assert pose["position"] == [1.0, 0.0, 0.0]
        # Direction is the unit feed direction into the vessel.
        assert pose["direction"] == pytest.approx([0.0, 0.0, 1.0])

    def test_aorta_tree_routes_are_selectable_without_mujoco(self):
        engine = self._engine(phantom="aorta_tree", guided=True)
        routes = engine.available_routes

        assert routes
        target = sorted(routes)[0]
        assert engine.select_route(target) is True
        assert engine._route_target == target
        assert engine.entry_pose["position"] == pytest.approx(engine.planned_path[0])
        assert engine.reset().target_position == pytest.approx(routes[target])

    def test_aorta_tree_constructor_route_target_uses_branch(self):
        first = self._engine(phantom="aorta_tree", guided=True)
        target = sorted(first.available_routes)[-1]

        engine = self._engine(phantom="aorta_tree", guided=True, route_target=target)

        assert engine.planned_path[-1] == pytest.approx(engine.available_routes[target])

    def test_vpp_routes_are_selectable_without_mujoco(self):
        engine = self._engine(
            phantom="case_001_vpp",
            guided=True,
            physics_engine="guided",
            route_target="endpoints_1",
        )

        assert "endpoints_1" in engine.available_routes
        assert "endpoints_24" in engine.available_routes
        assert engine.planned_path[-1] == pytest.approx(engine.available_routes["endpoints_1"])
        assert engine._path is not None
        assert engine._path.radii is not None

        assert engine.select_route("endpoints_2") is True
        assert engine.planned_path[-1] == pytest.approx(engine.available_routes["endpoints_2"])


class TestGuidedMode:
    """Kinematic centerline-follow mode (no MuJoCo required)."""

    # A simple 3 m straight path along +x.
    PATH = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]]

    def _engine(self, **kwargs):
        from services.navigation_engine import NavigationEngine

        kwargs.setdefault("planned_path", self.PATH)
        kwargs.setdefault("guided", True)
        return NavigationEngine(**kwargs)

    def test_guided_requires_path(self):
        from services.navigation_engine import NavigationEngine

        engine = NavigationEngine(guided=True)  # no planned path
        assert engine._is_guided() is False

    def test_reset_spawns_at_entry(self):
        engine = self._engine()
        state = engine.reset()
        assert state.tip_position == pytest.approx([0.0, 0.0, 0.0])
        assert state.path_progress == 0.0
        assert state.path_deviation == 0.0
        assert state.path_total_distance == pytest.approx(3.0)
        assert state.path_travelled_distance == pytest.approx(0.0)
        assert state.remaining_distance == pytest.approx(3.0)
        assert state.safety_status == "STANDBY"
        assert engine.entry_pose["position"] == pytest.approx([0.0, 0.0, 0.0])

    def test_push_advances_progress_to_target(self):
        engine = self._engine(advance_per_step=0.1)
        engine.reset()
        last = -1.0
        state = None
        for _ in range(200):
            state = engine.step(1.0, 0.0)
            assert state.path_progress >= last - 1e-9  # monotonic non-decreasing
            last = state.path_progress
            if state.done:
                break
        assert state.done is True
        assert state.path_progress == pytest.approx(1.0)
        assert state.path_total_distance == pytest.approx(3.0)
        assert state.path_travelled_distance == pytest.approx(3.0)
        assert state.remaining_distance == pytest.approx(0.0)
        # Reached the planned target (last path point).
        assert state.tip_position == pytest.approx([3.0, 0.0, 0.0])

    def test_retract_decreases_progress(self):
        engine = self._engine(advance_per_step=0.1)
        engine.reset()
        for _ in range(10):
            engine.step(1.0, 0.0)
        advanced = engine.step(0.0, 0.0).path_progress
        retracted = engine.step(-1.0, 0.0).path_progress
        assert retracted < advanced

    def test_render_bodies_span_full_inserted_length(self):
        engine = self._engine(advance_per_step=0.1)
        engine.reset()
        for _ in range(15):
            engine.step(1.0, 0.0)  # s = 1.5 along the 3 m straight path
        bodies = engine.get_render_bodies()
        assert len(bodies) > 1
        # The wire spans the full inserted length: first body at the entry (x=0),
        # last at the tip (x=1.5). On a straight path there is no inner-wall lean.
        for body in bodies:
            assert body["pos"][1] == pytest.approx(0.0, abs=1e-9)
        assert bodies[0]["pos"][0] == pytest.approx(0.0, abs=1e-3)
        assert bodies[-1]["pos"][0] == pytest.approx(1.5, abs=1e-2)

    def test_inner_wall_lean_offsets_on_bend(self):
        # An L-shaped path bends at the corner; the lean offset must be nonzero
        # there and capped at wall_lean.
        engine = self._engine(
            planned_path=[[0, 0, 0], [1, 0, 0], [1, 1, 0]],
            advance_per_step=0.1,
            wall_lean=0.0025,
        )
        engine.reset()
        import numpy as np

        offset = engine._inner_wall_offset(1.0)  # at the corner
        mag = float(np.linalg.norm(offset))
        assert mag > 0.0
        assert mag <= 0.0025 + 1e-9


# ============================================================================
# Integration Tests (Require MuJoCo)
# ============================================================================


@pytest.mark.slow
class TestNavigationEngineIntegration:
    """Integration tests for NavigationEngine with real MuJoCo environment."""

    def test_engine_initialization(self):
        from services.navigation_engine import NavigationEngine

        engine = NavigationEngine(phantom="low_tort", target="bca")
        assert engine.phantom == "low_tort"
        assert engine.target == "bca"
        assert not engine.is_initialized
        engine.close()

    def test_engine_reset(self):
        from services.navigation_engine import NavigationEngine

        engine = NavigationEngine(phantom="low_tort", target="bca")
        state = engine.reset()

        assert engine.is_initialized
        assert len(state.tip_position) == 3
        assert len(state.tip_direction) == 3
        assert state.episode_length == 0
        assert state.done is False

        engine.close()

    def test_engine_step(self):
        from services.navigation_engine import NavigationEngine

        engine = NavigationEngine(phantom="low_tort", target="bca")
        engine.reset()

        state = engine.step(delta_push=0.5, delta_rotate=0.0)

        assert state.episode_length == 1
        assert len(state.tip_position) == 3

        engine.close()

    def test_engine_multiple_steps(self):
        from services.navigation_engine import NavigationEngine

        engine = NavigationEngine(phantom="low_tort", target="bca")
        engine.reset()

        for i in range(10):
            state = engine.step(delta_push=0.3, delta_rotate=0.1)
            assert state.episode_length == i + 1

        engine.close()

    def test_engine_safety_status(self):
        from services.navigation_engine import NavigationEngine

        engine = NavigationEngine(phantom="low_tort", target="bca")
        state = engine.reset()

        status = engine.get_safety_status(state)
        assert status == "STANDBY"

        state = engine.step(delta_push=0.0, delta_rotate=0.0)
        status = engine.get_safety_status(state)
        assert status in ("SAFE_NAV", "DANGER_WARNING", "COLLISION_STOP")

        engine.close()


@pytest.mark.slow
class TestQuatFromVectors:
    """Math checks for the guidewire entry-alignment quaternion.

    Marked slow because importing cathsim.dm.env pulls in dm_control/MuJoCo.
    """

    def _rotate(self, quat, vec):
        import numpy as np

        w, x, y, z = quat
        # Rotate vec by quaternion [w, x, y, z].
        q = np.array([x, y, z])
        v = np.asarray(vec, dtype=float)
        return v + 2 * np.cross(q, np.cross(q, v) + w * v)

    def test_identity_for_parallel(self):
        import numpy as np

        from cathsim.dm.env import quat_from_vectors

        q = quat_from_vectors([0, 1, 0], [0, 2, 0])
        assert np.allclose(q, [1, 0, 0, 0])

    def test_rotates_feed_axis_onto_target(self):
        import numpy as np

        from cathsim.dm.env import GUIDEWIRE_FEED_AXIS, quat_from_vectors

        target = np.array([-0.956, -0.005, 0.292])
        target = target / np.linalg.norm(target)
        q = quat_from_vectors(GUIDEWIRE_FEED_AXIS, target)
        rotated = self._rotate(q, GUIDEWIRE_FEED_AXIS)
        assert np.allclose(rotated, target, atol=1e-6)

    def test_antiparallel_is_180_degrees(self):
        import numpy as np

        from cathsim.dm.env import quat_from_vectors

        q = quat_from_vectors([0, 1, 0], [0, -1, 0])
        rotated = self._rotate(q, [0, 1, 0])
        assert np.allclose(rotated, [0, -1, 0], atol=1e-6)


@pytest.mark.slow
class TestVPPPhantomIntegration:
    """End-to-end checks for the generated VPP MuJoCo phantom."""

    @pytest.fixture
    def vpp_mujoco_dir(self):
        from pathlib import Path

        case_dir = Path(__file__).resolve().parents[1] / "data" / "vpp_assets" / "case_001"
        mujoco_dir = case_dir / "mujoco"
        if not (mujoco_dir / "case_001_vpp.xml").is_file():
            pytest.skip("VPP MuJoCo phantom is not generated")
        return mujoco_dir

    def test_vpp_phantom_sites_load(self, vpp_mujoco_dir):
        from cathsim.dm.components.phantom import Phantom

        phantom = Phantom("case_001_vpp.xml", assets_dir=vpp_mujoco_dir)

        assert len(phantom.sites) == 25
        assert "endpoints_1" in phantom.sites
        assert phantom.phantom_visual.is_file()

    def test_vpp_mjcf_physics_compiles(self, vpp_mujoco_dir):
        from dm_control import mjcf

        root = mjcf.from_file(
            (vpp_mujoco_dir / "case_001_vpp.xml").as_posix(),
            False,
            vpp_mujoco_dir.as_posix(),
        )
        physics = mjcf.Physics.from_mjcf_model(root)

        assert physics.model.nmesh == 129
        assert physics.model.ngeom == 129

    def test_navigation_engine_vpp_phantom_reset_and_step(self, vpp_mujoco_dir):
        from services.navigation_engine import NavigationEngine

        engine = NavigationEngine(
            phantom="case_001_vpp",
            target="endpoints_1",
            assets_dir=str(vpp_mujoco_dir),
        )

        state = engine.reset()
        assert state.episode_length == 0
        assert len(state.tip_position) == 3
        assert state.target_position == [-0.97565564, -0.21722368, 0.25031761]

        state = engine.step(delta_push=0.1, delta_rotate=0.0)
        assert state.episode_length == 1
        assert len(state.tip_position) == 3
        assert state.contact_force >= 0.0

        engine.close()

    def test_vpp_phantom_extended_state_fields(self, vpp_mujoco_dir):
        from services.navigation_engine import NavigationEngine

        engine = NavigationEngine(
            phantom="case_001_vpp",
            target="endpoints_1",
            assets_dir=str(vpp_mujoco_dir),
        )

        state = engine.reset()
        # At reset the episode has not started -> STANDBY.
        assert state.safety_status == "STANDBY"
        assert len(state.tip_quaternion) == 4

        state = engine.step(delta_push=0.2, delta_rotate=0.0)
        assert state.safety_status in (
            "SAFE_NAV",
            "DANGER_WARNING",
            "COLLISION_STOP",
        )
        assert state.wall_distance >= 0.0
        assert state.curvature >= 0.0
        assert 0.0 <= state.path_progress <= 1.0
        assert state.path_deviation == 0.0  # no planned path provided

        engine.close()

    def test_vpp_phantom_path_progress_tracking(self, vpp_mujoco_dir):
        from services.navigation_engine import NavigationEngine

        engine = NavigationEngine(
            phantom="case_001_vpp",
            target="endpoints_1",
            assets_dir=str(vpp_mujoco_dir),
        )

        # Reset once to obtain a valid tip position, then build a trivial path
        # passing through it so progress/deviation are well-defined.
        state = engine.reset()
        tip = state.tip_position
        engine.set_planned_path([tip, [tip[0], tip[1], tip[2] + 0.05]])

        state = engine.reset()
        for _ in range(5):
            state = engine.step(delta_push=0.3, delta_rotate=0.0)

        assert 0.0 <= state.path_progress <= 1.0
        assert state.path_deviation >= 0.0

        engine.close()

    def test_vpp_guidewire_spawns_at_planned_path_entry(self, vpp_mujoco_dir):
        """The guidewire must spawn at the VPP vessel entry, not the world origin.

        Without entry alignment the guidewire spawns ~1m away from the offset VPP
        vessel and never enters it. With a planned path, the entry is derived from
        the path's first point, so the tip starts on the path (small deviation)
        and advances into the vessel under a forward push.
        """
        from services.navigation_engine import NavigationEngine

        # A short straight path near endpoints_24 (proximal, x~0) heading toward
        # the descending vessel (-x), in MuJoCo meters.
        entry = [0.0, -0.268, 0.291]
        planned_path = [
            entry,
            [-0.02, -0.268, 0.291],
            [-0.04, -0.270, 0.292],
        ]

        engine = NavigationEngine(
            phantom="case_001_vpp",
            target="endpoints_1",
            assets_dir=str(vpp_mujoco_dir),
            planned_path=planned_path,
            n_bodies=40,
            n_substeps=2,
        )

        state = engine.reset()
        # Tip starts on the planned path (within a few mm), proving the guidewire
        # spawned at the vessel entry rather than ~1m away at the origin.
        assert state.path_deviation < 0.02
        # The spawn is far from the world origin (the vessel is offset ~1m).
        assert abs(state.tip_position[1]) > 0.1

        for _ in range(20):
            state = engine.step(delta_push=0.6, delta_rotate=0.0)

        # Advancing forward registers vessel contact and keeps progress valid.
        assert state.episode_length == 20
        assert 0.0 <= state.path_progress <= 1.0

        engine.close()


@pytest.mark.slow
class TestSessionAPIIntegration:
    """Integration tests for Session REST API."""

    def test_session_start_endpoint(self):
        client = TestClient(app)

        response = client.post(
            "/api/v1/session/start",
            json={"phantom": "low_tort", "target": "bca"},
        )

        assert response.status_code == 200
        payload = response.json()
        assert "session_id" in payload
        assert payload["phantom"] == "low_tort"
        assert payload["target"] == "bca"
        assert "state" in payload
        assert "tip_position" in payload["state"]

        session_id = payload["session_id"]
        client.delete(f"/api/v1/session/{session_id}")

    def test_session_step_endpoint(self):
        client = TestClient(app)

        start_resp = client.post(
            "/api/v1/session/start",
            json={"phantom": "low_tort", "target": "bca"},
        )
        session_id = start_resp.json()["session_id"]

        step_resp = client.post(
            f"/api/v1/session/{session_id}/step",
            json={"delta_push": 0.5, "delta_rotate": 0.0},
        )

        assert step_resp.status_code == 200
        payload = step_resp.json()
        assert payload["session_id"] == session_id
        assert payload["step_count"] == 1
        assert "state" in payload

        client.delete(f"/api/v1/session/{session_id}")

    def test_session_reset_endpoint(self):
        client = TestClient(app)

        start_resp = client.post(
            "/api/v1/session/start",
            json={"phantom": "low_tort", "target": "bca"},
        )
        session_id = start_resp.json()["session_id"]

        client.post(
            f"/api/v1/session/{session_id}/step",
            json={"delta_push": 0.5, "delta_rotate": 0.0},
        )

        reset_resp = client.post(f"/api/v1/session/{session_id}/reset")

        assert reset_resp.status_code == 200
        payload = reset_resp.json()
        assert payload["session_id"] == session_id
        assert payload["episode_count"] == 2
        assert payload["state"]["episode_length"] == 0

        client.delete(f"/api/v1/session/{session_id}")

    def test_session_list_endpoint(self):
        client = TestClient(app)

        start_resp = client.post(
            "/api/v1/session/start",
            json={"phantom": "low_tort", "target": "bca"},
        )
        session_id = start_resp.json()["session_id"]

        list_resp = client.get("/api/v1/session")

        assert list_resp.status_code == 200
        sessions = list_resp.json()
        assert any(s["session_id"] == session_id for s in sessions)

        client.delete(f"/api/v1/session/{session_id}")

    def test_session_close_endpoint(self):
        client = TestClient(app)

        start_resp = client.post(
            "/api/v1/session/start",
            json={"phantom": "low_tort", "target": "bca"},
        )
        session_id = start_resp.json()["session_id"]

        close_resp = client.delete(f"/api/v1/session/{session_id}")

        assert close_resp.status_code == 200
        assert close_resp.json()["status"] == "closed"

        get_resp = client.get(f"/api/v1/session/{session_id}")
        assert get_resp.status_code == 404

    def test_session_not_found(self):
        client = TestClient(app)

        response = client.post(
            "/api/v1/session/nonexistent-id/step",
            json={"delta_push": 0.0, "delta_rotate": 0.0},
        )
        assert response.status_code == 404
