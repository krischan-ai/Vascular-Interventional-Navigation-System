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

    def test_set_planned_path_clear(self):
        engine = self._engine(planned_path=[[0, 0, 0], [0, 0, 1]])
        assert engine._path_total_len > 0
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
