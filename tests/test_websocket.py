"""Tests for WebSocket real-time communication.

These tests verify the WebSocket protocol implementation. Tests involving
the actual MuJoCo environment are marked with pytest.mark.slow.
"""

import time
from pathlib import Path

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
        assert valid.microcatheter_advance == 0.0

        support = ControlData(delta_push=0.0, delta_rotate=0.0, microcatheter_advance=0.6)
        assert support.microcatheter_advance == 0.6

        with pytest.raises(ValidationError):
            ControlData(delta_push=1.5, delta_rotate=0.0)

        with pytest.raises(ValidationError):
            ControlData(delta_push=0.0, delta_rotate=-2.0)

        with pytest.raises(ValidationError):
            ControlData(delta_push=0.0, delta_rotate=0.0, microcatheter_advance=-2.0)

    def test_device_config_maps_tip_shape_to_engine_params(self):
        from services.websocket_handler import DeviceConfigData, WebSocketHandler

        straight = DeviceConfigData(guidewire={"tip_shape": "straight"})
        mapped = WebSocketHandler._device_config_to_engine_params(straight)
        assert mapped["tip_shape"] == "straight"
        assert mapped["jtip_deg"] == pytest.approx(0.0)
        assert mapped["jtip_bodies"] == 0

        j_tip = DeviceConfigData(guidewire={
            "tip_shape": "j_tip",
            "tip_curve_angle_deg": 45.0,
            "tip_length_mm": 12.0,
            "soft_tip_length_mm": 15.0,
            "tip_stiffness": 0.25,
        })
        mapped = WebSocketHandler._device_config_to_engine_params(j_tip)
        assert mapped["tip_shape"] == "j_tip"
        assert mapped["jtip_deg"] == pytest.approx(45.0)
        assert mapped["jtip_bodies"] == 4
        assert mapped["soft_tip"] == 5
        assert mapped["tip_bend"] == pytest.approx(5.375)

    def test_session_start_data_defaults(self):
        from services.websocket_handler import SessionStartData

        data = SessionStartData()
        assert data.phantom == "low_tort"
        assert data.target == "bca"
        assert data.use_pixels is False
        assert data.physics_engine == "auto"

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

    def test_control_data_forwards_microcatheter_advance(self):
        import asyncio
        from unittest.mock import AsyncMock, MagicMock

        from services.navigation_engine import NavigationState
        from services.websocket_handler import WebSocketHandler

        handler = WebSocketHandler(MagicMock())
        handler._session_manager.step.return_value = NavigationState()
        send_mock = AsyncMock()
        handler._send_message = send_mock
        conn = MagicMock()
        conn.session_id = "session-1"
        conn.batch_mode = False
        conn.control_rate_limiter = 0.0

        asyncio.run(handler._handle_control(conn, {
            "delta_push": 0.1,
            "delta_rotate": 0.2,
            "microcatheter_advance": 0.7,
        }))

        handler._session_manager.step.assert_called_once_with(
            session_id="session-1",
            delta_push=0.1,
            delta_rotate=0.2,
            microcatheter_advance=0.7,
        )

    def test_device_config_applies_engine_params(self):
        import asyncio
        from unittest.mock import AsyncMock, MagicMock

        from services.websocket_handler import MessageType, WebSocketHandler

        engine = MagicMock()
        engine.set_engine_params.return_value = {
            "tip_shape": "j_tip",
            "jtip_deg": 45.0,
            "jtip_bodies": 4,
        }
        handler = WebSocketHandler(MagicMock())
        handler._session_manager.get_session.return_value = engine
        send_mock = AsyncMock()
        handler._send_message = send_mock
        conn = MagicMock()
        conn.session_id = "session-1"

        asyncio.run(handler._handle_device_config(conn, {
            "guidewire": {
                "tip_shape": "j_tip",
                "tip_curve_angle_deg": 45.0,
                "tip_length_mm": 12.0,
            }
        }))

        engine.set_engine_params.assert_called_once_with({
            "jtip_deg": 45.0,
            "jtip_bodies": 4,
        })
        _, args, kwargs = send_mock.mock_calls[0]
        assert args[1] == MessageType.DEVICE_CONFIG
        assert kwargs["data"]["effective"]["jtip_deg"] == pytest.approx(45.0)
        assert kwargs["data"]["guidewire"]["tip_shape"] == "j_tip"

    def test_connection_state_defaults(self):
        from services.websocket_handler import ConnectionState
        from unittest.mock import MagicMock

        mock_ws = MagicMock()
        state = ConnectionState(websocket=mock_ws)

        assert state.session_id is None
        assert state.is_alive is True
        assert state.control_rate_limiter == 0.0
        assert state.batch_mode is False

    def test_state_batch_contains_dashboard_fields(self):
        from services.navigation_engine import NavigationState
        from services.session_manager import SessionManager
        from services.websocket_handler import WebSocketHandler

        class DummyBackend:
            def diagnostics(self):
                return {"drive": "force", "slack_m": 0.002, "feed_budget_m": 0.010}

            def mechanics_state(self):
                return {
                    "source": "newton_engine.mechanics_state",
                    "source_fields": ["jtip_deg", "free_len", "max_slack"],
                    "guidewire": {
                        "shape_type": "j_tip",
                        "curve_angle_deg": 35.0,
                        "tip_length_m": 0.009,
                        "proximal_rotation_deg": 45.0,
                        "distal_tip_rotation_deg": 30.0,
                        "torsion_lag_deg": 15.0,
                        "tip_deflection_score": 0.57,
                    },
                    "support": {
                        "support_state": "modeled",
                        "effective_support_type": "proximal_sheath",
                        "effective_support_tip_m": 0.03,
                        "free_wire_length_m": 0.03,
                        "support_ratio": 0.5,
                        "max_slack_m": 0.012,
                    },
                    "risk": {
                        "slack_m": 0.002,
                        "feed_budget_m": 0.010,
                        "wall_slide_state": "WALL_SLIDE_OK",
                        "buckling_risk": "LOW",
                        "normal_poking_score": 0.04,
                        "tangential_slide_score": 0.88,
                    },
                }

        class DummyEngine:
            _engine = DummyBackend()
            planned_path = [[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
            entry_pose = {"position": [0.0, 0.0, 0.0], "direction": [0.0, 0.0, 1.0]}

            def get_render_bodies(self):
                return []

        state = NavigationState(
            path_progress=0.25,
            path_deviation=0.001,
            remaining_distance=0.75,
            vessel_radius=0.004,
            eta_seconds=3.0,
            risk_score=0.2,
            risk_assessment={
                "metrics": {
                    "wall_distance": {"level": "WARNING"},
                    "curvature": {"level": "SAFE"},
                }
            },
            contact_force=0.12,
            wall_contact_count=4,
            max_penetration=0.0002,
            contact_impulse=0.015,
            wall_distance=0.0012,
            safety_status="DANGER_WARNING",
            fidelity_mode="physics",
            flow_guidance={
                "workflow": {
                    "phase": "WALL_SLIDE",
                    "step_index": 5,
                    "source": "navigation_engine.flow_guidance",
                },
                "tip_shape": {"tip_shape_state": "unknown", "shape_type": None},
                "support": {"support_state": "unknown", "effective_support_type": None},
                "micro_advance": {
                    "micro_advance_state": "caution",
                    "cadence_state": "micro_push",
                    "hard_push_state": "clear",
                    "hard_push_score": 0.1,
                },
                "training_score": {
                    "overall": 82,
                    "components": {"wall_slide": 90},
                    "source": "navigation_engine.large_curvature_training_score",
                },
            },
        )
        batch = WebSocketHandler(SessionManager())._state_to_batch(state, DummyEngine())

        assert batch["fidelity_mode"] == "physics"
        assert batch["path"]["remaining_distance"] == 0.75
        assert batch["path"]["vessel_radius"] == 0.004
        assert batch["path"]["eta_seconds"] == 3.0
        assert batch["safety"]["risk_score"] == 0.2
        assert batch["safety"]["risk_regions"] == []
        assert batch["flow_guidance"]["workflow"]["phase"] == "WALL_SLIDE"
        assert batch["safety"]["flow_guidance"] == batch["flow_guidance"]
        assert batch["training_score"]["overall"] == 82
        assert batch["training_score"] == batch["flow_guidance"]["training_score"]
        assert batch["flow_guidance"]["micro_advance"]["cadence_state"] == "micro_push"
        assert batch["flow_guidance"]["micro_advance"]["hard_push_score"] == pytest.approx(0.1)
        assert batch["schema_version"] == "navigation_visual_v2"
        assert isinstance(batch["timestamp_ms"], int)
        mechanics = batch["safety"]["guidewire_mechanics"]
        assert mechanics["source"] == "navigation_engine.risk_assessor"
        assert mechanics["tip_force_n"] == 0.12
        assert mechanics["contact_count"] == 4
        assert mechanics["max_penetration_m"] == pytest.approx(0.0002)
        assert mechanics["contact_impulse"] == pytest.approx(0.015)
        assert mechanics["wall_distance_m"] == 0.0012
        assert mechanics["lateral_force_n"] is None
        assert mechanics["torque_nm"] is None
        assert mechanics["safety_level"] == "warning"
        assert mechanics["stop_required"] is False
        assert mechanics["reason_codes"] == ["WALL_DISTANCE_WARNING"]
        assert batch["engine"] == "DummyBackend"
        assert batch["diagnostics"]["drive"] == "force"
        assert batch["diagnostics"]["slack_m"] == 0.002
        assert batch["guidewire"]["shape_type"] == "j_tip"
        assert batch["guidewire"]["curve_angle_deg"] == pytest.approx(35.0)
        assert batch["guidewire"]["distal_tip_rotation_deg"] == pytest.approx(30.0)
        assert batch["guidewire"]["torsion_lag_deg"] == pytest.approx(15.0)
        assert batch["support"]["effective_support_type"] == "proximal_sheath"
        assert batch["support"]["free_wire_length_m"] == pytest.approx(0.03)
        assert batch["risk"]["slack_m"] == pytest.approx(0.002)
        assert batch["risk"]["normal_poking_score"] == pytest.approx(0.04)
        assert batch["risk"]["tangential_slide_score"] == pytest.approx(0.88)
        assert batch["risk"]["safety_status"] == "DANGER_WARNING"

    def test_state_batch_visual_level_uses_real_risk_level_before_stop(self):
        from services.navigation_engine import NavigationState
        from services.session_manager import SessionManager
        from services.websocket_handler import WebSocketHandler

        class DummyEngine:
            _engine = None
            planned_path = []
            entry_pose = {}

            def get_render_bodies(self):
                return []

        state = NavigationState(
            risk_score=0.92,
            risk_assessment={
                "risk_level": "CRITICAL",
                "metrics": {
                    "wall_distance": {"level": "CRITICAL"},
                },
            },
            safety_status="SAFE_NAV",
        )

        batch = WebSocketHandler(SessionManager())._state_to_batch(state, DummyEngine())
        mechanics = batch["safety"]["guidewire_mechanics"]

        assert mechanics["safety_level"] == "danger"
        assert mechanics["stop_required"] is False
        assert mechanics["reason_codes"] == ["WALL_DISTANCE_CRITICAL"]

    def test_state_batch_visual_level_uses_real_risk_score_bands(self):
        from services.navigation_engine import NavigationState
        from services.session_manager import SessionManager
        from services.websocket_handler import WebSocketHandler

        class DummyEngine:
            _engine = None
            planned_path = []
            entry_pose = {}

            def get_render_bodies(self):
                return []

        handler = WebSocketHandler(SessionManager())

        medium = NavigationState(
            risk_score=0.4,
            risk_assessment={"risk_level": "SAFE", "metrics": {}},
            wall_distance=0.002,
            safety_status="SAFE_NAV",
        )
        normal = NavigationState(
            risk_score=0.1,
            risk_assessment={"risk_level": "SAFE", "metrics": {}},
            wall_distance=0.002,
            safety_status="SAFE_NAV",
        )

        medium_batch = handler._state_to_batch(medium, DummyEngine())
        normal_batch = handler._state_to_batch(normal, DummyEngine())

        assert medium_batch["safety"]["guidewire_mechanics"]["safety_level"] == "warning"
        assert normal_batch["safety"]["guidewire_mechanics"]["safety_level"] == "safe"

    def test_state_batch_visual_level_recovers_when_risk_score_is_normal(self):
        from services.navigation_engine import NavigationState
        from services.session_manager import SessionManager
        from services.websocket_handler import WebSocketHandler

        class DummyEngine:
            _engine = None
            planned_path = []
            entry_pose = {}

            def get_render_bodies(self):
                return []

        state = NavigationState(
            risk_score=0.0,
            wall_distance=0.002,
            risk_assessment={
                "risk_level": "CRITICAL",
                "metrics": {
                    "wall_distance": {"level": "CRITICAL"},
                },
            },
            safety_status="SAFE_NAV",
        )

        batch = WebSocketHandler(SessionManager())._state_to_batch(state, DummyEngine())

        assert batch["safety"]["guidewire_mechanics"]["safety_level"] == "safe"

    def test_state_batch_visual_level_uses_real_wall_distance_bands(self):
        from services.navigation_engine import NavigationState
        from services.session_manager import SessionManager
        from services.websocket_handler import WebSocketHandler

        class DummyEngine:
            _engine = None
            planned_path = []
            entry_pose = {}

            def get_render_bodies(self):
                return []

        near_wall = NavigationState(
            risk_score=0.1,
            wall_distance=0.0005,
            risk_assessment={"risk_level": "SAFE", "metrics": {}},
            safety_status="SAFE_NAV",
        )

        batch = WebSocketHandler(SessionManager())._state_to_batch(near_wall, DummyEngine())

        assert batch["safety"]["guidewire_mechanics"]["safety_level"] == "danger"

    def test_session_start_batch_mode_embeds_initial_batch(self):
        import asyncio

        from services.navigation_engine import NavigationState
        from services.websocket_handler import WebSocketHandler
        from unittest.mock import AsyncMock, MagicMock

        handler = WebSocketHandler(MagicMock())
        engine = MagicMock()
        engine.available_routes = {}
        engine.planned_path = [[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
        engine.entry_pose = {"position": [0.0, 0.0, 0.0], "direction": [0.0, 0.0, 1.0]}
        engine.get_render_bodies.return_value = [
            {"pos": [0.0, 0.0, 0.0], "quat": [0.0, 0.0, 0.0, 1.0]},
            {"pos": [0.0, 0.0, 0.01], "quat": [0.0, 0.0, 0.0, 1.0]},
        ]
        handler._session_manager.create_session.return_value = ("session-1", NavigationState())
        handler._session_manager.get_session.return_value = engine
        conn = MagicMock()
        conn.session_id = None
        conn.batch_mode = False
        conn.path_sent = False

        send_mock = AsyncMock()
        handler._send_message = send_mock

        asyncio.run(handler._handle_session_start(conn, {"batch_mode": True}))

        assert conn.session_id == "session-1"
        assert conn.batch_mode is True
        assert send_mock.await_count == 1
        assert send_mock.await_args.args[1].value == "session_started"
        data = send_mock.await_args.kwargs["data"]
        assert data["initial_batch"]["bodies"] == engine.get_render_bodies.return_value
        assert data["initial_batch"]["path"]["waypoints"] == engine.planned_path
        assert conn.path_sent is True


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

    def test_vpp_graph_resolves_when_started_from_godot_client(self, monkeypatch):
        from services.websocket_handler import _get_path_planner

        _get_path_planner.cache_clear()
        monkeypatch.chdir(Path(__file__).resolve().parents[1] / "godot_client")

        planner = _get_path_planner("case_001")

        assert planner.nodes

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
        assert params.physics_engine == "auto"

    def test_vpp_mujoco_override_uses_real_physics_backend(self):
        from services.websocket_handler import SessionStartData, _resolve_session_backend

        params = SessionStartData(
            phantom="case_001_vpp",
            physics_engine="mujoco",
            start_position=[0.0, 0.0, 0.0],
            end_position=[1.0, 0.0, 0.0],
        )

        guided, engine_mode = _resolve_session_backend(
            params,
            planned_path=[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
        )

        assert guided is False
        assert engine_mode == "mujoco"

    def test_vpp_guided_override_is_explicit_only(self):
        from services.websocket_handler import SessionStartData, _resolve_session_backend

        params = SessionStartData(phantom="case_001_vpp", physics_engine="guided")
        guided, engine_mode = _resolve_session_backend(
            params,
            planned_path=[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
        )

        assert guided is True
        assert engine_mode == "guided"

    def test_aorta_tree_defaults_to_physics_backend(self):
        from services.websocket_handler import SessionStartData, _resolve_session_backend

        params = SessionStartData(phantom="aorta_tree")
        guided, engine_mode = _resolve_session_backend(params, planned_path=None)

        assert guided is False
        assert engine_mode == "auto"

    def test_vpp_auto_backend_defaults_to_guided(self, monkeypatch):
        from services.websocket_handler import SessionStartData, _resolve_session_backend

        monkeypatch.delenv("CATHSIM_PHYSICS_ENGINE", raising=False)
        params = SessionStartData(phantom="case_001_vpp")
        guided, physics_engine = _resolve_session_backend(
            params,
            planned_path=[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
        )

        assert guided is True
        assert physics_engine == "auto"

    def test_vpp_newton_backend_disables_guided(self):
        from services.websocket_handler import SessionStartData, _resolve_session_backend

        params = SessionStartData(
            phantom="case_001_vpp",
            guided=True,
            physics_engine="newton_demo",
        )
        guided, physics_engine = _resolve_session_backend(
            params,
            planned_path=[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
        )

        assert guided is False
        assert physics_engine == "newton_demo"

    def test_vpp_mujoco_backend_disables_guided(self):
        from services.websocket_handler import SessionStartData, _resolve_session_backend

        params = SessionStartData(
            phantom="case_001_vpp",
            guided=True,
            physics_engine="mujoco",
        )
        guided, physics_engine = _resolve_session_backend(
            params,
            planned_path=[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
        )

        assert guided is False
        assert physics_engine == "mujoco"

    def test_vpp_resolved_path_carries_radii_in_meters(self):
        import asyncio

        from services.session_manager import SessionManager
        from services.websocket_handler import SessionStartData, WebSocketHandler

        params = SessionStartData(
            phantom="case_001_vpp",
            start_position=[0.173, -268.24, 291.25],
            end_position=[-30.0, -268.6, 296.0],
            smooth=False,
        )

        path, radii = asyncio.run(
            WebSocketHandler(SessionManager())._resolve_session_path(params)
        )

        assert path is not None
        assert radii is not None
        assert len(path) == len(radii)
        assert 0.0001 < min(radii) < max(radii) < 0.02

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
            initial_batch = start["data"]["initial_batch"]
            assert len(initial_batch["path"]["waypoints"]) > 100
            assert len(initial_batch["bodies"]) > 0
            assert len(initial_batch["entry"]["position"]) == 3
            assert initial_batch["entry"]["position"] == pytest.approx(
                initial_batch["path"]["waypoints"][0], abs=1e-6
            )

            websocket.send_json({
                "type": "control",
                "data": {"delta_push": 0.6, "delta_rotate": 0.0},
            })
            batch = _recv_pong(websocket)
            assert batch["type"] == "state_batch"
            data = batch["data"]
            # The planned path already streamed in session_started.initial_batch;
            # follow-up control frames can omit waypoints to stay small.
            assert data["path"]["waypoints"] == []
            assert len(data["bodies"]) > 0
            assert 0.0 <= data["path"]["progress"] <= 1.0
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
