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
                return {
                    "drive": "force",
                    "slack_m": 0.002,
                    "max_slack_m": 0.012,
                    "feed_budget_m": 0.010,
                    "max_breach_m": -0.001,
                    "guidewire": {"profile_name": "soft_j_tip_training_wire", "tip_shape": "j_tip"},
                    "support": {"effective_support_type": "microcatheter", "free_wire_length_mm": 30.0},
                    "procedure": {
                        "name": "femoral_aorta_branch_navigation",
                        "display_name_zh": "股动脉入路主动脉分支导航",
                        "procedure_type": "aorta_branch_navigation",
                        "access_site": "common_femoral_artery",
                        "access_site_label": "股总动脉",
                        "access_route_label": "股动脉入路",
                        "needle_entry_label": "股骨头投影区，腹股沟韧带以下、股动脉分叉以上",
                        "guidewire_summary": "0.014 J-tip 标准训练导丝 / 180 cm",
                    },
                    "normal_poking_score": 0.2,
                    "tangential_slide_score": 0.6,
                    "wall_slide_state": "WALL_SLIDE_OK",
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
            wall_distance=0.0012,
            safety_status="DANGER_WARNING",
            fidelity_mode="physics",
        )
        batch = WebSocketHandler(SessionManager())._state_to_batch(state, DummyEngine())

        assert batch["fidelity_mode"] == "physics"
        assert batch["path"]["remaining_distance"] == 0.75
        assert batch["path"]["vessel_radius"] == 0.004
        assert batch["path"]["eta_seconds"] == 3.0
        assert batch["safety"]["risk_score"] == 0.2
        assert batch["safety"]["risk_regions"] == []
        assert batch["schema_version"] == "navigation_visual_v2"
        assert isinstance(batch["timestamp_ms"], int)
        mechanics = batch["safety"]["guidewire_mechanics"]
        assert mechanics["source"] == "navigation_engine.risk_assessor"
        assert mechanics["tip_force_n"] == 0.12
        assert mechanics["wall_distance_m"] == 0.0012
        assert mechanics["lateral_force_n"] is None
        assert mechanics["torque_nm"] is None
        assert mechanics["safety_level"] == "warning"
        assert mechanics["stop_required"] is False
        assert mechanics["reason_codes"] == ["WALL_DISTANCE_WARNING"]
        assert batch["engine"] == "DummyBackend"
        assert batch["diagnostics"]["drive"] == "force"
        assert batch["diagnostics"]["slack_m"] == 0.002
        assert batch["guidewire"]["tip_shape"] == "j_tip"
        assert batch["guidewire"]["tip_shape_label"] == "J\u5c16"
        assert batch["guidewire"]["torsion_lag_deg_label"] == "\u626d\u8f6c\u6ede\u540e"
        assert batch["support"]["effective_support_type"] == "microcatheter"
        assert batch["support"]["effective_support_type_label"] == "\u5fae\u5bfc\u7ba1"
        assert batch["guidewire"]["design_name"] == "standard_014_jtip"
        assert batch["guidewire"]["clinical_total_length_mm"] == pytest.approx(1800.0)
        assert batch["procedure"]["name"] == "femoral_aorta_branch_navigation"
        assert batch["procedure"]["access_site_label"] == "股总动脉"
        assert batch["procedure"]["guidewire_summary"] == "0.014 J-tip 标准训练导丝 / 180 cm"
        assert batch["risk"]["slack_mm"] == pytest.approx(2.0)
        assert batch["risk"]["pile_ratio"] == pytest.approx(0.002 / 0.012)
        assert batch["risk"]["breach_mm"] == pytest.approx(0.0)
        assert batch["risk"]["buckling_risk"] == "LOW"
        assert batch["risk"]["buckling_risk_text"] == "\u4f4e"
        assert batch["risk"]["normal_poking_score"] == pytest.approx(0.2)
        assert batch["risk"]["normal_poking_score_label"] == "\u9876\u58c1\u98ce\u9669"
        assert batch["risk"]["normal_poking_score_text"] == "\u4f4e"
        assert batch["risk"]["tangential_slide_score"] == pytest.approx(0.6)
        assert batch["risk"]["tangential_slide_score_label"] == "\u8d34\u58c1\u6ed1\u5165"
        assert batch["risk"]["tangential_slide_score_text"] == "\u4e2d"
        assert batch["risk"]["wall_slide_state"] == "WALL_SLIDE_OK"
        assert batch["risk"]["wall_slide_state_label"] == "\u8d34\u58c1\u72b6\u6001"
        assert batch["risk"]["wall_slide_state_text"] == "\u8d34\u58c1\u6ed1\u5165"
        assert batch["risk"]["display"]["normal_poking_score"] == {"name": "\u9876\u58c1\u98ce\u9669", "value": "\u4f4e"}

    def test_state_batch_fills_device_defaults_without_backend_diagnostics(self):
        from services.navigation_engine import NavigationState
        from services.session_manager import SessionManager
        from services.websocket_handler import WebSocketHandler

        class DummyEngine:
            _engine = None
            planned_path = []
            entry_pose = {}

            def get_render_bodies(self):
                return []

        state = NavigationState(safety_status="SAFE_NAV")
        batch = WebSocketHandler(SessionManager())._state_to_batch(state, DummyEngine())

        assert batch["guidewire"]["tip_shape"] == "j_tip"
        assert batch["guidewire"]["tip_shape_label"] == "J\u5c16"
        assert batch["guidewire"]["current_tip_segment"] == "pre_shaped_soft_tip"
        assert batch["guidewire"]["current_tip_segment_label"] == "\u9884\u5851\u5f62\u8f6f\u5934"
        assert batch["support"]["effective_support_type"] == "microcatheter"
        assert batch["support"]["effective_support_type_label"] == "\u5fae\u5bfc\u7ba1"
        assert batch["support"]["free_wire_length_mm"] == pytest.approx(30.0)
        assert batch["risk"]["buckling_risk"] == "LOW"
        assert batch["risk"]["buckling_risk_text"] == "\u4f4e"
        assert batch["risk"]["normal_poking_score"] is None
        assert batch["risk"]["normal_poking_score_text"] == "\u672a\u77e5"
        assert batch["risk"]["tangential_slide_score"] is None
        assert batch["risk"]["tangential_slide_score_text"] == "\u672a\u77e5"

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






def test_state_batch_default_procedure_context_is_stable():
    from services.navigation_engine import NavigationState
    from services.session_manager import SessionManager
    from services.websocket_handler import WebSocketHandler

    class DummyEngine:
        _engine = None
        planned_path = []
        entry_pose = {}

        def get_render_bodies(self):
            return []

    batch = WebSocketHandler(SessionManager())._state_to_batch(
        NavigationState(safety_status="SAFE_NAV"),
        DummyEngine(),
    )

    assert batch["procedure"]["name"] == "femoral_aorta_branch_navigation"
    assert batch["procedure"]["access_route_label"] == "股动脉入路"
    assert batch["procedure"]["access_site_label"] == "股总动脉"
    assert batch["procedure"]["needle_entry_label"] == "股骨头投影区，腹股沟韧带以下、股动脉分叉以上"
    assert batch["procedure"]["guidewire_summary"] == "0.014 J-tip 标准训练导丝 / 180 cm"
