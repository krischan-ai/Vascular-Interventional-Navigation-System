"""Tests for WebSocket real-time communication.

These tests verify the WebSocket protocol implementation. Tests involving
the actual MuJoCo environment are marked with pytest.mark.slow.
"""

import pytest
from fastapi.testclient import TestClient

from services.main import app


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

            response = websocket.receive_json()
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

            response = websocket.receive_json()
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
            start_response = websocket.receive_json()
            assert start_response["type"] == "session_started"

            websocket.send_json({
                "type": "control",
                "data": {"delta_push": 0.5, "delta_rotate": 0.0}
            })

            state_response = websocket.receive_json()
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
            websocket.receive_json()

            websocket.send_json({
                "type": "control",
                "data": {"delta_push": 0.5, "delta_rotate": 0.0}
            })
            websocket.receive_json()

            websocket.send_json({
                "type": "reset",
                "data": {}
            })

            reset_response = websocket.receive_json()
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
            start_response = websocket.receive_json()
            session_id = start_response["session_id"]

            websocket.send_json({
                "type": "session_stop",
                "data": {}
            })

            stop_response = websocket.receive_json()
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

            response = websocket.receive_json()
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
            websocket.receive_json()

            websocket.send_json({
                "type": "control",
                "data": {"delta_push": 2.0, "delta_rotate": 0.0}
            })

            response = websocket.receive_json()
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
            websocket.receive_json()

            for i in range(5):
                websocket.send_json({
                    "type": "control",
                    "data": {"delta_push": 0.3, "delta_rotate": 0.1}
                })
                response = websocket.receive_json()
                assert response["type"] == "state_update"
                assert response["data"]["episode_length"] == i + 1
