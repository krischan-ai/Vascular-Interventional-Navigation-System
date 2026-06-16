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
