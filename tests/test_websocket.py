"""Tests for WebSocket real-time communication.

These tests verify the WebSocket protocol implementation. Tests involving
the actual MuJoCo environment are marked with pytest.mark.slow.
"""

import time

import pytest
from fastapi.testclient import TestClient

from services.main import app


def _recv(websocket):
    """Receive the next message, skipping server-initiated ping heartbeats.

    The handler emits a ping every PING_INTERVAL seconds; during slow MuJoCo
    steps this can interleave with state responses, so tests must ignore it.
    """
    while True:
        message = websocket.receive_json()
        if message.get("type") != "ping":
            return message


# ============================================================================
# Unit Tests (No MuJoCo required)
# ============================================================================


class TestWebSocketHandlerUnit:
    """Unit tests for WebSocketHandler without MuJoCo."""

    def test_message_types_enum(self):
        from services.websocket_handler import MessageType

        assert MessageType.CONTROL.value == "control"
        assert MessageType.STATE_UPDATE.value == "state_update"
        assert MessageType.PING.value == "ping"
        assert MessageType.PONG.value == "pong"

    def test_control_data_validation(self):
        from services.websocket_handler import ControlData
        from pydantic import ValidationError

        valid = ControlData(delta_push=0.5, delta_rotate=-0.3)
        assert valid.delta_push == 0.5
        assert valid.delta_rotate == -0.3

        with pytest.raises(ValidationError):
            ControlData(delta_push=1.5, delta_rotate=0.0)

        with pytest.raises(ValidationError):
            ControlData(delta_push=0.0, delta_rotate=-2.0)

    def test_session_start_data_defaults(self):
        from services.websocket_handler import SessionStartData

        data = SessionStartData()
        assert data.phantom == "low_tort"
        assert data.target == "bca"
        assert data.use_pixels is False

    def test_websocket_message_parsing(self):
        from services.websocket_handler import WebSocketMessage

        msg = WebSocketMessage(
            type="control",
            session_id="test-id",
            timestamp=1718534400000,
            data={"delta_push": 0.5, "delta_rotate": 0.0},
        )
        assert msg.type == "control"
        assert msg.session_id == "test-id"
        assert msg.data["delta_push"] == 0.5

    def test_connection_state_defaults(self):
        from services.websocket_handler import ConnectionState
        from unittest.mock import MagicMock

        mock_ws = MagicMock()
        state = ConnectionState(websocket=mock_ws)

        assert state.session_id is None
        assert state.is_alive is True
        assert state.control_rate_limiter == 0.0
        assert state.batch_mode is False


class TestWebSocketProtocolExtensions:
    """Unit tests for Stage-8 protocol additions (no MuJoCo required)."""

    def test_new_message_types(self):
        from services.websocket_handler import MessageType

        assert MessageType.PATH_REQUEST.value == "path_request"
        assert MessageType.PATH_RESPONSE.value == "path_response"
        assert MessageType.STATE_BATCH.value == "state_batch"

    def test_session_start_batch_mode_default(self):
        from services.websocket_handler import SessionStartData

        assert SessionStartData().batch_mode is False
        assert SessionStartData(batch_mode=True).batch_mode is True

    def test_path_request_data_validation(self):
        from services.websocket_handler import PathRequestData
        from pydantic import ValidationError

        req = PathRequestData(
            start_position=[1.0, 2.0, 3.0],
            end_position=[4.0, 5.0, 6.0],
        )
        assert req.case_id == "case_001"
        assert req.algorithm == "astar"
        assert req.smooth is False

        with pytest.raises(ValidationError):
            PathRequestData(start_position=[1.0, 2.0], end_position=[4.0, 5.0, 6.0])


class TestWebSocketPathRequest:
    """path_request does not require MuJoCo (graph + A* only)."""

    def test_websocket_path_request_returns_path(self):
        from services.websocket_handler import _get_path_planner

        planner = _get_path_planner("case_001")
        start = list(planner.nodes[0])
        end = list(planner.nodes[200])

        client = TestClient(app)
        with client.websocket_connect("/ws/session") as websocket:
            websocket.send_json({
                "type": "path_request",
                "data": {
                    "case_id": "case_001",
                    "start_position": start,
                    "end_position": end,
                    "smooth": False,
                },
            })

            response = _recv(websocket)
            assert response["type"] == "path_response"
            assert "waypoints" in response["data"]
            assert response["data"]["node_count"] >= 1
            assert response["data"]["length_mm"] >= 0.0

    def test_websocket_path_request_invalid_params(self):
        client = TestClient(app)
        with client.websocket_connect("/ws/session") as websocket:
            websocket.send_json({
                "type": "path_request",
                "data": {"start_position": [0.0, 0.0], "end_position": [1.0, 1.0, 1.0]},
            })

            response = _recv(websocket)
            assert response["type"] == "error"
            assert response["data"]["code"] == "INVALID_PARAMS"


# ============================================================================
# Integration Tests (Require MuJoCo)
# ============================================================================


@pytest.mark.slow
class TestWebSocketIntegration:
    """Integration tests for WebSocket with real MuJoCo environment."""

    def test_websocket_connect_disconnect(self):
        """Test basic WebSocket connection lifecycle."""
        client = TestClient(app)

        with client.websocket_connect("/ws/session") as websocket:
            pass

    def test_websocket_session_start(self):
        """Test starting a session via WebSocket."""
        client = TestClient(app)

        with client.websocket_connect("/ws/session") as websocket:
            websocket.send_json({
                "type": "session_start",
                "data": {
                    "phantom": "low_tort",
                    "target": "bca",
                }
            })

            response = _recv(websocket)
            assert response["type"] == "session_started"
            assert "session_id" in response
            assert response["data"]["phantom"] == "low_tort"
            assert "state" in response["data"]
            assert "tip_position" in response["data"]["state"]

    def test_websocket_control_without_session(self):
        """Test control command without active session returns error."""
        client = TestClient(app)

        with client.websocket_connect("/ws/session") as websocket:
            websocket.send_json({
                "type": "control",
                "data": {
                    "delta_push": 0.5,
                    "delta_rotate": 0.0,
                }
            })

            response = _recv(websocket)
            assert response["type"] == "error"
            assert response["data"]["code"] == "NO_SESSION"

    def test_websocket_session_control_step(self):
        """Test control command execution via WebSocket."""
        client = TestClient(app)

        with client.websocket_connect("/ws/session") as websocket:
            websocket.send_json({
                "type": "session_start",
                "data": {"phantom": "low_tort", "target": "bca"}
            })
            start_response = _recv(websocket)
            assert start_response["type"] == "session_started"

            websocket.send_json({
                "type": "control",
                "data": {"delta_push": 0.5, "delta_rotate": 0.0}
            })

            state_response = _recv(websocket)
            assert state_response["type"] == "state_update"
            assert "tip_position" in state_response["data"]
            assert state_response["data"]["episode_length"] == 1

    def test_websocket_session_reset(self):
        """Test reset command via WebSocket."""
        client = TestClient(app)

        with client.websocket_connect("/ws/session") as websocket:
            websocket.send_json({
                "type": "session_start",
                "data": {"phantom": "low_tort", "target": "bca"}
            })
            _recv(websocket)

            websocket.send_json({
                "type": "control",
                "data": {"delta_push": 0.5, "delta_rotate": 0.0}
            })
            _recv(websocket)

            websocket.send_json({
                "type": "reset",
                "data": {}
            })

            reset_response = _recv(websocket)
            assert reset_response["type"] == "state_update"
            assert reset_response["data"]["episode_length"] == 0
            assert reset_response["data"]["episode_count"] == 2

    def test_websocket_session_stop(self):
        """Test stopping a session via WebSocket."""
        client = TestClient(app)

        with client.websocket_connect("/ws/session") as websocket:
            websocket.send_json({
                "type": "session_start",
                "data": {"phantom": "low_tort", "target": "bca"}
            })
            start_response = _recv(websocket)
            session_id = start_response["session_id"]

            websocket.send_json({
                "type": "session_stop",
                "data": {}
            })

            stop_response = _recv(websocket)
            assert stop_response["type"] == "session_stopped"
            assert stop_response["session_id"] == session_id

    def test_websocket_pong_response(self):
        """Test pong message handling."""
        client = TestClient(app)

        with client.websocket_connect("/ws/session") as websocket:
            websocket.send_json({
                "type": "pong",
            })

    def test_websocket_invalid_message_type(self):
        """Test handling of unknown message types."""
        client = TestClient(app)

        with client.websocket_connect("/ws/session") as websocket:
            websocket.send_json({
                "type": "unknown_type",
                "data": {}
            })

            response = _recv(websocket)
            assert response["type"] == "error"
            assert "UNKNOWN_TYPE" in response["data"]["code"]

    def test_websocket_invalid_control_params(self):
        """Test handling of invalid control parameters."""
        client = TestClient(app)

        with client.websocket_connect("/ws/session") as websocket:
            websocket.send_json({
                "type": "session_start",
                "data": {"phantom": "low_tort", "target": "bca"}
            })
            _recv(websocket)

            websocket.send_json({
                "type": "control",
                "data": {"delta_push": 2.0, "delta_rotate": 0.0}
            })

            response = _recv(websocket)
            assert response["type"] == "error"
            assert response["data"]["code"] == "INVALID_CONTROL"

    def test_websocket_multiple_steps(self):
        """Test multiple control steps in sequence."""
        client = TestClient(app)

        with client.websocket_connect("/ws/session") as websocket:
            websocket.send_json({
                "type": "session_start",
                "data": {"phantom": "low_tort", "target": "bca"}
            })
            _recv(websocket)

            for i in range(5):
                websocket.send_json({
                    "type": "control",
                    "data": {"delta_push": 0.3, "delta_rotate": 0.1}
                })
                response = _recv(websocket)
                assert response["type"] == "state_update"
                assert response["data"]["episode_length"] == i + 1

    def test_websocket_state_update_has_extended_fields(self):
        """state_update should carry the Stage-7 extended fields."""
        client = TestClient(app)

        with client.websocket_connect("/ws/session") as websocket:
            websocket.send_json({
                "type": "session_start",
                "data": {"phantom": "low_tort", "target": "bca"}
            })
            _recv(websocket)

            websocket.send_json({
                "type": "control",
                "data": {"delta_push": 0.3, "delta_rotate": 0.0}
            })
            response = _recv(websocket)

            data = response["data"]
            for key in (
                "tip_quaternion",
                "wall_distance",
                "curvature",
                "path_progress",
                "path_deviation",
                "safety_status",
                "risk_score",
            ):
                assert key in data
            assert data["safety_status"] in (
                "SAFE_NAV",
                "DANGER_WARNING",
                "COLLISION_STOP",
            )
            assert 0.0 <= data["risk_score"] <= 1.0

    def test_websocket_batch_mode_state_batch(self):
        """With batch_mode the control response is a state_batch with render data."""
        client = TestClient(app)

        with client.websocket_connect("/ws/session") as websocket:
            websocket.send_json({
                "type": "session_start",
                "data": {
                    "phantom": "low_tort",
                    "target": "bca",
                    "batch_mode": True,
                },
            })
            start_response = _recv(websocket)
            assert start_response["type"] == "session_started"

            websocket.send_json({
                "type": "control",
                "data": {"delta_push": 0.3, "delta_rotate": 0.0}
            })
            response = _recv(websocket)

            assert response["type"] == "state_batch"
            data = response["data"]
            assert set(data) >= {"tip", "bodies", "path", "safety", "episode"}
            assert "position" in data["tip"]
            assert isinstance(data["bodies"], list)
            assert len(data["bodies"]) > 0
            assert "pos" in data["bodies"][0]
            assert "quat" in data["bodies"][0]
            assert data["safety"]["status"] in (
                "SAFE_NAV",
                "DANGER_WARNING",
                "COLLISION_STOP",
            )
            assert data["episode"]["length"] == 1


class TestSessionStartDataVPP:
    """Unit checks for the VPP session_start fields (no MuJoCo)."""

    def test_vpp_fields_default_to_none(self):
        from services.websocket_handler import SessionStartData

        params = SessionStartData()
        assert params.case_id == "case_001"
        assert params.start_position is None
        assert params.end_position is None
        assert params.planned_path is None
        assert params.smooth is True

    def test_resolve_vpp_assets_dir(self):
        from services.navigation_engine import resolve_vpp_assets_dir

        # Non-VPP phantom -> built-in assets (None).
        assert resolve_vpp_assets_dir("low_tort") is None
        # Unknown VPP case -> None (directory does not exist).
        assert resolve_vpp_assets_dir("nope_vpp") is None
        # Known VPP case -> its mujoco assets dir (when generated).
        resolved = resolve_vpp_assets_dir("case_001_vpp")
        if resolved is not None:
            assert resolved.replace("\\", "/").endswith("data/vpp_assets/case_001/mujoco")


def _recv_pong(websocket):
    """Receive the next non-ping message, answering ping heartbeats with pong.

    VPP sessions have a slow MuJoCo cold start; replying to pings keeps the
    connection alive past PONG_TIMEOUT during the wait.
    """
    while True:
        message = websocket.receive_json()
        if message.get("type") == "ping":
            websocket.send_json({"type": "pong", "data": {}})
            continue
        return message


@pytest.mark.slow
class TestWebSocketVPPNavigation:
    """End-to-end VPP navigation over WebSocket with server-side planning."""

    def _vpp_available(self):
        from pathlib import Path

        xml = (
            Path(__file__).resolve().parents[1]
            / "data" / "vpp_assets" / "case_001" / "mujoco" / "case_001_vpp.xml"
        )
        if not xml.is_file():
            pytest.skip("VPP MuJoCo phantom is not generated")

    def test_vpp_session_plans_path_and_aligns_entry(self):
        self._vpp_available()
        client = TestClient(app)

        with client.websocket_connect("/ws/session") as websocket:
            websocket.send_json({
                "type": "session_start",
                "data": {
                    "phantom": "case_001_vpp",
                    "target": "endpoints_1",
                    "batch_mode": True,
                    "n_bodies": 40,
                    "n_substeps": 2,
                    "case_id": "case_001",
                    # Endpoints-24 (proximal) -> Endpoints-1 (distal), LPS mm.
                    "start_position": [0.173, -268.24, 291.25],
                    "end_position": [-975.65, -217.22, 250.32],
                    "smooth": True,
                },
            })
            start = _recv_pong(websocket)
            assert start["type"] == "session_started"
            state = start["data"]["state"]
            # Server planned the path and spawned the guidewire at the vessel
            # entry: the tip starts on the path (within a few mm), not ~1m away.
            assert state["path_deviation"] < 0.02

            websocket.send_json({
                "type": "control",
                "data": {"delta_push": 0.6, "delta_rotate": 0.0},
            })
            batch = _recv_pong(websocket)
            assert batch["type"] == "state_batch"
            data = batch["data"]
            # The planned path streams to the client for visualization.
            assert len(data["path"]["waypoints"]) > 100
            assert len(data["bodies"]) > 0
            assert 0.0 <= data["path"]["progress"] <= 1.0
            # Entry (vascular access) and target markers are highlighted in the
            # client. The entry rides the first batch and sits at the path start.
            assert len(data["entry"]["position"]) == 3
            assert len(data["entry"]["direction"]) == 3
            assert data["entry"]["position"] == pytest.approx(
                data["path"]["waypoints"][0], abs=1e-6
            )
            assert len(data["target"]) == 3

            # Guided (centerline-follow) mode is auto-enabled for VPP routes:
            # pushing advances the guidewire along the planned path. Pace sends
            # above the 33ms control rate limit so none are dropped. (Full
            # traversal to the target is covered by TestGuidedMode.)
            start_progress = data["path"]["progress"]
            last_progress = start_progress
            for _ in range(8):
                time.sleep(0.04)
                websocket.send_json({
                    "type": "control",
                    "data": {"delta_push": 1.0, "delta_rotate": 0.0},
                })
                last_progress = _recv_pong(websocket)["data"]["path"]["progress"]
            assert last_progress > start_progress
