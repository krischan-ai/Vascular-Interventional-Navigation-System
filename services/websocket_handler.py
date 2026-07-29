"""WebSocket handler for real-time CathSim control and state streaming.

This module implements the WebSocket protocol defined in doc/03-API与通信协议.md.
"""

from __future__ import annotations

import asyncio
import os
import time
import traceback
from dataclasses import dataclass, field
from enum import Enum
from functools import lru_cache
from pathlib import Path
from typing import Any, Callable, Literal

from fastapi import WebSocket, WebSocketDisconnect
from pydantic import BaseModel, Field, ValidationError

from services.devices import default_guidewire_design, default_procedure_design, default_support_system
from services.navigation_engine import NavigationEngine, NavigationState
from services.path_planner import PathPlanner
from services.session_manager import SessionManager
from services.vpp_assets import require_vpp_graph_path

_PROJECT_ROOT = Path(__file__).resolve().parents[1]

# Built-in sealed-lumen phantoms can run physical backends despite shipping a
# centerline/routes. Other centerline phantoms default to kinematic guidance
# unless an explicit engine override is active.
_PHYSICS_CAPABLE_BUILTIN: frozenset[str] = frozenset({"aorta_trunk", "aorta_tree"})
PhysicsEngineMode = Literal["auto", "guided", "mujoco", "physics", "newton", "newton_demo"]

_TIP_SHAPE_LABELS = {
    "j_tip": "J尖",
    "straight": "直头",
}
_GUIDEWIRE_SEGMENT_LABELS = {
    "atraumatic_tip": "无创头端",
    "atraumatic_cap": "无创头端",
    "pre_shaped_soft_tip": "预塑形软头",
    "distal_soft": "远端软段",
    "transition": "过渡段",
    "flexible_transition": "柔顺过渡段",
    "torque_response": "扭矩响应段",
    "main_support_shaft": "主支撑杆身",
    "proximal_control": "近端控制段",
    "proximal_shaft": "近端杆身",
}
_SUPPORT_TYPE_LABELS = {
    "introducer_sheath": "导入鞘",
    "guiding_catheter": "导引导管",
    "intermediate_catheter": "中间导管",
    "microcatheter": "微导管",
    "none": "无支撑",
}
_BUCKLING_RISK_LABELS = {
    "LOW": "低",
    "MEDIUM": "中",
    "HIGH": "高",
    "UNKNOWN": "未知",
}
_WALL_SLIDE_STATE_LABELS = {
    "WALL_SLIDE_OK": "贴壁滑入",
    "TIP_POKING_WARNING": "顶壁风险",
    "FREE_CENTERED": "居中未贴壁",
    "SAFE_NAV": "安全导航",
    "DANGER_WARNING": "风险预警",
    "COLLISION_STOP": "碰撞制动",
    "STANDBY": "待机",
    "UNKNOWN": "未知",
}


def _cn_label(labels: dict[str, str], value: Any, default: str = "未知") -> str:
    if value is None:
        return default
    text = str(value)
    return labels.get(text, text if text else default)


def _score_text(score: Any) -> str:
    if not isinstance(score, (int, float)):
        return "未知"
    score_float = float(score)
    if score_float >= 0.7:
        return "高"
    if score_float >= 0.35:
        return "中"
    return "低"

def _builtin_phantom_centerline(phantom: str) -> Path | None:
    """Path to a built-in phantom's shipped centerline.json, if it exists.

    Built-in phantoms (e.g. segment_part) may ship a precomputed entry->target
    centerline. Their vessels are long/thin/off-origin like VPP cases, so the
    physical guidewire cannot traverse them; presence of this file is the signal
    to default to kinematic centerline-follow.
    """
    centerline = (
        _PROJECT_ROOT
        / "src/cathsim/dm/components/phantom_assets/meshes"
        / phantom
        / "centerline.json"
    )
    return centerline if centerline.is_file() else None


def _resolve_session_backend(
    params: "SessionStartData",
    planned_path: list[list[float]] | None,
) -> tuple[bool, str]:
    """Resolve guided flag plus concrete backend override for session creation."""
    requested = params.physics_engine.lower()
    if requested == "guided":
        return True, "guided"
    if requested in {"mujoco", "physics", "newton", "newton_demo"}:
        return False, requested

    env_mode = os.environ.get("CATHSIM_PHYSICS_ENGINE", "").lower()
    if env_mode in {"newton", "newton_demo"}:
        return False, env_mode

    ships_centerline = _builtin_phantom_centerline(params.phantom) is not None
    force_guided_builtin = (
        ships_centerline and params.phantom not in _PHYSICS_CAPABLE_BUILTIN
    )
    guided = (
        params.guided
        or (planned_path is not None and params.phantom.endswith("_vpp"))
        or force_guided_builtin
    )
    return guided, "auto"


@lru_cache(maxsize=8)
def _get_path_planner(case_id: str) -> PathPlanner:
    """Load (and cache) a PathPlanner for the given case's VPP graph."""
    return PathPlanner(require_vpp_graph_path(case_id))


class MessageType(str, Enum):
    """WebSocket message types."""

    # Client -> Server
    CONTROL = "control"
    SESSION_START = "session_start"
    SESSION_STOP = "session_stop"
    PATH_REQUEST = "path_request"
    SELECT_ROUTE = "select_route"
    DEVICE_CONFIG = "device_config"
    ENGINE_PARAMS = "engine_params"
    SHAPE_INTENT = "shape_intent"
    RESET = "reset"
    PONG = "pong"

    # Server -> Client
    STATE_UPDATE = "state_update"
    STATE_BATCH = "state_batch"
    PATH_RESPONSE = "path_response"
    ERROR = "error"
    PING = "ping"
    SESSION_STARTED = "session_started"
    SESSION_STOPPED = "session_stopped"


class ControlData(BaseModel):
    """Control command data."""

    delta_push: float = Field(ge=-1.0, le=1.0)
    delta_rotate: float = Field(ge=-1.0, le=1.0)
    microcatheter_advance: float = Field(default=0.0, ge=-1.0, le=1.0)


class ShapeIntentData(BaseModel):
    """ShapeIntent (autopilot) command data (doc/09).

    ``active=False`` disengages and returns manual control. With ``active=True``
    and no target, the controller follows the planned centerline (plain
    autopilot). A ``target_waypoint`` (e.g. a UI click projected onto the vessel)
    or ``target_direction`` redirects the aim; ``intensity`` scales the push.
    """

    active: bool = True
    target_waypoint: list[float] | None = Field(default=None, min_length=3, max_length=3)
    target_direction: list[float] | None = Field(default=None, min_length=3, max_length=3)
    intensity: float = Field(default=1.0, ge=0.0, le=1.0)


class GuidewireConfigData(BaseModel):
    """Semantic guidewire/tip-shape configuration for clinical workflow P1."""

    tip_shape: Literal["straight", "angled", "j_tip", "j", "c_shape", "hook"] = "j_tip"
    tip_curve_angle_deg: float | None = Field(default=None, ge=0.0, le=120.0)
    tip_length_mm: float | None = Field(default=None, ge=0.0, le=60.0)
    soft_tip_length_mm: float | None = Field(default=None, ge=0.0, le=80.0)
    tip_stiffness: float | None = Field(default=None, ge=0.0, le=1.0)


class DeviceConfigData(BaseModel):
    """Device configuration message.

    The public protocol is semantic (guidewire.tip_shape), while Newton still
    consumes low-level deformation params. The handler maps between them.
    """

    guidewire: GuidewireConfigData


class SessionStartData(BaseModel):
    """Session start request data.

    For VPP navigation the client may either pass an explicit ``planned_path``
    (MuJoCo meters) or let the server plan it by providing ``start_position`` and
    ``end_position`` (LPS millimeters) plus ``case_id``. The first path point is
    used as the guidewire entry, aligning it with the offset VPP vessel.
    """

    phantom: str = "low_tort"
    target: str = "bca"
    use_pixels: bool = False
    batch_mode: bool = False
    n_bodies: int = 80
    n_substeps: int | None = None
    # VPP path/entry options.
    case_id: str = "case_001"
    start_position: list[float] | None = None
    end_position: list[float] | None = None
    smooth: bool = True
    # B-spline smoothing strength. 0.0 = interpolating spline (threads through
    # every A* node, so a jagged centerline stays jagged); > 0 lets the curve
    # deviate from the noisy nodes to actually de-jag it. Residual RMS deviation
    # is ~sqrt(smooth_factor) mm, so keep it modest to stay inside thin vessels.
    smooth_factor: float = 0.5
    planned_path: list[list[float]] | None = None
    # Kinematic centerline-follow mode: drive the guidewire along the planned
    # path so it reliably reaches the target on full-length VPP vessels that the
    # physical guidewire cannot traverse. Auto-enabled for VPP routes below.
    guided: bool = False
    # Backend override for VPP development. ``auto`` preserves the current
    # defaults, ``guided`` forces centerline-follow, ``mujoco``/``physics`` forces
    # the MuJoCo backend, and ``newton_demo`` selects the experimental native
    # VPP lumen-wall backend.
    physics_engine: PhysicsEngineMode = "auto"
    # Initial branch route id (e.g. "endpoint_24") for multi-branch phantoms that
    # ship routes.json (aorta_tree). None -> the shipped primary centerline.
    route_target: str | None = None


class SelectRouteData(BaseModel):
    """Runtime branch-target switch for a multi-branch phantom session."""

    target: str


class ResetData(BaseModel):
    """Reset request data."""

    randomize: bool = False


class PathRequestData(BaseModel):
    """Path planning request data (positions in LPS millimeters)."""

    case_id: str = "case_001"
    start_position: list[float] = Field(min_length=3, max_length=3)
    end_position: list[float] = Field(min_length=3, max_length=3)
    algorithm: str = "astar"
    smooth: bool = False
    # See SessionStartData.smooth_factor. Only applied when smooth is True.
    smooth_factor: float = 0.5


class WebSocketMessage(BaseModel):
    """Base WebSocket message structure."""

    type: str
    session_id: str | None = None
    timestamp: int | None = None
    data: dict[str, Any] = Field(default_factory=dict)


@dataclass
class ConnectionState:
    """State for a single WebSocket connection."""

    websocket: WebSocket
    session_id: str | None = None
    last_ping_time: float = field(default_factory=time.time)
    last_pong_time: float = field(default_factory=time.time)
    is_alive: bool = True
    control_rate_limiter: float = 0.0  # Last control timestamp
    batch_mode: bool = False  # Send state_batch (with render data) instead of state_update
    path_sent: bool = False  # The planned path is constant; send it once, then omit


class WebSocketHandler:
    """Handles WebSocket connections for real-time CathSim control.

    Features:
    - Session lifecycle management via WebSocket
    - Real-time control input processing
    - State streaming at configurable rates
    - Heartbeat ping/pong mechanism
    - Rate limiting for control commands
    """

    PING_INTERVAL = 5.0  # seconds
    PONG_TIMEOUT = 45.0  # seconds (generous margin for MuJoCo cold-start init)
    MIN_CONTROL_INTERVAL = 0.033  # ~30Hz max

    def __init__(self, session_manager: SessionManager):
        """Initialize WebSocket handler.

        Args:
            session_manager: SessionManager instance for session operations
        """
        self._session_manager = session_manager
        self._connections: dict[WebSocket, ConnectionState] = {}
        self._state_callbacks: dict[str, Callable] = {}

    async def handle_connection(self, websocket: WebSocket) -> None:
        """Handle a WebSocket connection lifecycle.

        Args:
            websocket: FastAPI WebSocket instance
        """
        await websocket.accept()

        conn_state = ConnectionState(websocket=websocket)
        self._connections[websocket] = conn_state

        ping_task = asyncio.create_task(self._ping_loop(conn_state))
        receive_task = asyncio.create_task(self._receive_loop(conn_state))

        try:
            done, pending = await asyncio.wait(
                [ping_task, receive_task],
                return_when=asyncio.FIRST_COMPLETED,
            )

            for task in pending:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass

        except WebSocketDisconnect:
            pass
        finally:
            await self._cleanup_connection(conn_state)

    async def _ping_loop(self, conn_state: ConnectionState) -> None:
        """Send periodic ping messages and check for pong timeout."""
        while conn_state.is_alive:
            await asyncio.sleep(self.PING_INTERVAL)

            if not conn_state.is_alive:
                break

            # Session startup can take a while on first Newton/Warp init. Until
            # the session exists, keep the socket open without enforcing pong
            # timeouts so a slow create_session does not get killed mid-start.
            if conn_state.session_id is None:
                continue

            elapsed = time.time() - conn_state.last_pong_time
            if elapsed > self.PONG_TIMEOUT:
                conn_state.is_alive = False
                print(
                    f"[WS] PONG_TIMEOUT: no pong for {elapsed:.1f}s "
                    f"(session={conn_state.session_id})"
                )
                await self._send_error(
                    conn_state, "PONG_TIMEOUT", "Connection timed out"
                )
                break

            await self._send_message(
                conn_state,
                MessageType.PING,
                session_id=conn_state.session_id,
            )

    async def _receive_loop(self, conn_state: ConnectionState) -> None:
        """Receive and process incoming WebSocket messages."""
        while conn_state.is_alive:
            try:
                raw_data = await conn_state.websocket.receive_json()
                await self._handle_message(conn_state, raw_data)
            except WebSocketDisconnect:
                conn_state.is_alive = False
                break
            except Exception as e:
                traceback.print_exc()
                await self._send_error(
                    conn_state, "PARSE_ERROR", f"{type(e).__name__}: {e}"
                )

    async def _run_blocking(self, func, *args, **kwargs):
        """Run a blocking call (MuJoCo step/reset, path planning) off the event
        loop so heartbeats keep flowing and the connection does not time out."""
        loop = asyncio.get_running_loop()
        if kwargs:
            from functools import partial

            return await loop.run_in_executor(None, partial(func, *args, **kwargs))
        return await loop.run_in_executor(None, func, *args)

    async def _handle_message(
        self, conn_state: ConnectionState, raw_data: dict
    ) -> None:
        """Route incoming message to appropriate handler."""
        try:
            message = WebSocketMessage(**raw_data)
        except ValidationError as e:
            await self._send_error(conn_state, "INVALID_MESSAGE", str(e))
            return

        msg_type = message.type

        if msg_type == MessageType.PONG:
            conn_state.last_pong_time = time.time()

        elif msg_type == MessageType.SESSION_START:
            await self._handle_session_start(conn_state, message.data)

        elif msg_type == MessageType.SESSION_STOP:
            await self._handle_session_stop(conn_state)

        elif msg_type == MessageType.CONTROL:
            await self._handle_control(conn_state, message.data)

        elif msg_type == MessageType.PATH_REQUEST:
            await self._handle_path_request(conn_state, message.data)

        elif msg_type == MessageType.SELECT_ROUTE:
            await self._handle_select_route(conn_state, message.data)

        elif msg_type == MessageType.DEVICE_CONFIG:
            await self._handle_device_config(conn_state, message.data)

        elif msg_type == MessageType.ENGINE_PARAMS:
            await self._handle_engine_params(conn_state, message.data)

        elif msg_type == MessageType.SHAPE_INTENT:
            await self._handle_shape_intent(conn_state, message.data)

        elif msg_type == MessageType.RESET:
            await self._handle_reset(conn_state, message.data)

        else:
            await self._send_error(
                conn_state, "UNKNOWN_TYPE", f"Unknown message type: {msg_type}"
            )

    async def _handle_session_start(
        self, conn_state: ConnectionState, data: dict
    ) -> None:
        """Handle session_start message."""
        if conn_state.session_id:
            await self._send_error(
                conn_state,
                "SESSION_EXISTS",
                "Session already active. Stop it first.",
            )
            return

        try:
            params = SessionStartData(**data)
        except ValidationError as e:
            await self._send_error(conn_state, "INVALID_PARAMS", str(e))
            return

        try:
            planned_path, planned_radii = await self._resolve_session_path(params)
        except (FileNotFoundError, ValueError) as e:
            await self._send_error(conn_state, "PATH_NOT_FOUND", str(e))
            return

        # Full-length VPP/centerline vessels still default to kinematic guidance,
        # but VPP development can now explicitly request MuJoCo or Newton so real
        # backend motion owns the session.
        guided, physics_engine = _resolve_session_backend(params, planned_path)

        try:
            session_id, state = await self._run_blocking(
                self._session_manager.create_session,
                phantom=params.phantom,
                target=params.target,
                use_pixels=params.use_pixels,
                n_bodies=params.n_bodies,
                n_substeps=params.n_substeps,
                planned_path=planned_path,
                planned_radii=planned_radii,
                guided=guided,
                physics_engine=physics_engine,
                route_target=params.route_target,
            )
            conn_state.session_id = session_id
            # Session creation can spend longer than PONG_TIMEOUT compiling the
            # first Newton/Warp kernels.  The ping loop intentionally suspends
            # timeout checks while session_id is None; restart its clock when the
            # session becomes active so it does not immediately reject a healthy
            # client using the stale pre-compilation timestamp.
            conn_state.last_pong_time = time.time()
            conn_state.batch_mode = params.batch_mode

            # Multi-branch phantoms expose their selectable branch tips so the
            # client can offer targets and switch via a select_route message.
            engine = self._session_manager.get_session(session_id)
            initial_batch = (
                self._state_to_batch(state, engine, include_path=True)
                if params.batch_mode
                else None
            )
            if initial_batch is not None:
                conn_state.path_sent = True
            await self._send_message(
                conn_state,
                MessageType.SESSION_STARTED,
                session_id=session_id,
                data={
                    "phantom": params.phantom,
                    "target": params.target,
                    "guided": guided,
                    "physics_engine": physics_engine,
                    "engine": type(engine._engine).__name__,
                    "fidelity_mode": state.fidelity_mode,
                    "state": self._state_to_dict(state),
                    "initial_batch": initial_batch,
                    "routes": engine.available_routes,
                },
            )

        except Exception as e:  # noqa: BLE001 - report any init failure to the client
            traceback.print_exc()
            await self._send_error(
                conn_state, "SESSION_ERROR", f"{type(e).__name__}: {e}"
            )

    async def _resolve_session_path(
        self, params: SessionStartData
    ) -> tuple[list[list[float]] | None, list[float] | None]:
        """Resolve the planned path (MuJoCo meters) for a navigation session.

        An explicit ``planned_path`` is used as-is. Otherwise, when both
        ``start_position`` and ``end_position`` (LPS millimeters) are given, the
        path is planned server-side and converted to meters. Returns None when no
        path is requested (e.g. plain low_tort sessions).
        """
        if params.planned_path is not None:
            return params.planned_path, None

        if params.start_position is None or params.end_position is None:
            return None, None

        planner = _get_path_planner(params.case_id)
        result = await self._run_blocking(
            planner.plan,
            params.start_position,
            params.end_position,
            smooth=params.smooth,
            smooth_factor=params.smooth_factor,
        )
        if result.smooth_waypoints is not None:
            waypoints = result.smooth_waypoints
            radii = result.smooth_radii
        else:
            waypoints = result.waypoints
            radii = result.radii
        # Graph/planner coordinates are LPS millimeters; the MuJoCo phantom frame
        # is meters (V-HACD applied a /1000 scale with no axis change).
        path_m = [[c / 1000.0 for c in point] for point in waypoints]
        radii_m = [r / 1000.0 for r in radii] if radii is not None else None
        return path_m, radii_m

    async def _handle_session_stop(self, conn_state: ConnectionState) -> None:
        """Handle session_stop message."""
        if not conn_state.session_id:
            await self._send_error(
                conn_state, "NO_SESSION", "No active session to stop"
            )
            return

        session_id = conn_state.session_id
        self._session_manager.close_session(session_id)
        conn_state.session_id = None

        await self._send_message(
            conn_state,
            MessageType.SESSION_STOPPED,
            session_id=session_id,
            data={"status": "closed"},
        )

    async def _handle_control(
        self, conn_state: ConnectionState, data: dict
    ) -> None:
        """Handle control command with rate limiting."""
        if not conn_state.session_id:
            await self._send_error(
                conn_state, "NO_SESSION", "No active session for control"
            )
            return

        now = time.time()
        elapsed = now - conn_state.control_rate_limiter
        if elapsed < self.MIN_CONTROL_INTERVAL:
            return

        conn_state.control_rate_limiter = now

        try:
            control = ControlData(**data)
        except ValidationError as e:
            await self._send_error(conn_state, "INVALID_CONTROL", str(e))
            return

        try:
            state = await self._run_blocking(
                self._session_manager.step,
                session_id=conn_state.session_id,
                delta_push=control.delta_push,
                delta_rotate=control.delta_rotate,
                microcatheter_advance=control.microcatheter_advance,
            )

            if conn_state.batch_mode:
                engine = self._session_manager.get_session(conn_state.session_id)
                include_path = not conn_state.path_sent
                await self._send_message(
                    conn_state,
                    MessageType.STATE_BATCH,
                    session_id=conn_state.session_id,
                    data=self._state_to_batch(state, engine, include_path=include_path),
                )
                conn_state.path_sent = True
            else:
                await self._send_message(
                    conn_state,
                    MessageType.STATE_UPDATE,
                    session_id=conn_state.session_id,
                    data=self._state_to_dict(state),
                )

        except KeyError:
            conn_state.session_id = None
            await self._send_error(
                conn_state, "SESSION_EXPIRED", "Session no longer exists"
            )
        except RuntimeError as e:
            await self._send_error(conn_state, "STEP_ERROR", str(e))

    async def _handle_path_request(
        self, conn_state: ConnectionState, data: dict
    ) -> None:
        """Handle a path planning request over the WebSocket.

        Does not require an active session: a client may request a route before
        starting a navigation session.
        """
        try:
            req = PathRequestData(**data)
        except ValidationError as e:
            await self._send_error(conn_state, "INVALID_PARAMS", str(e))
            return

        try:
            planner = _get_path_planner(req.case_id)
        except FileNotFoundError as e:
            await self._send_error(conn_state, "PATH_NOT_FOUND", str(e))
            return

        try:
            result = await self._run_blocking(
                planner.plan,
                req.start_position,
                req.end_position,
                algorithm=req.algorithm,
                smooth=req.smooth,
                smooth_factor=req.smooth_factor,
            )
        except ValueError as e:
            await self._send_error(conn_state, "PATH_NOT_FOUND", str(e))
            return

        await self._send_message(
            conn_state,
            MessageType.PATH_RESPONSE,
            session_id=conn_state.session_id,
            data=result.as_dict(),
        )

    async def _handle_select_route(
        self, conn_state: ConnectionState, data: dict
    ) -> None:
        """Switch the active session to a different branch route and re-arm it.

        For multi-branch phantoms (aorta_tree): the client picks a target endpoint
        id; the engine swaps its planned path to that branch route and resets, so
        the guidewire re-runs from the entry toward the chosen branch. The new path
        is re-sent (path_sent cleared) so the client redraws it.
        """
        if not conn_state.session_id:
            await self._send_error(conn_state, "NO_SESSION", "No active session")
            return
        try:
            req = SelectRouteData(**data)
        except ValidationError as e:
            await self._send_error(conn_state, "INVALID_PARAMS", str(e))
            return

        try:
            engine = self._session_manager.get_session(conn_state.session_id)
        except KeyError:
            conn_state.session_id = None
            await self._send_error(conn_state, "SESSION_EXPIRED", "Session no longer exists")
            return

        def _apply() -> NavigationState:
            if not engine.select_route(req.target):
                raise ValueError(f"unknown route target: {req.target}")
            return engine.reset()

        try:
            state = await self._run_blocking(_apply)
        except ValueError as e:
            await self._send_error(conn_state, "ROUTE_NOT_FOUND", str(e))
            return

        conn_state.path_sent = False  # re-send the new branch path to the client
        if conn_state.batch_mode:
            payload = self._state_to_batch(state, engine, include_path=True)
            conn_state.path_sent = True
            msg_type = MessageType.STATE_BATCH
        else:
            payload = self._state_to_dict(state)
            msg_type = MessageType.STATE_UPDATE

        await self._send_message(
            conn_state, msg_type, session_id=conn_state.session_id, data=payload
        )

    @staticmethod
    def _device_config_to_engine_params(config: DeviceConfigData) -> dict[str, Any]:
        guidewire = config.guidewire
        shape = "j_tip" if guidewire.tip_shape == "j" else guidewire.tip_shape
        default_angle = {
            "straight": 0.0,
            "angled": 25.0,
            "j_tip": 35.0,
            "c_shape": 55.0,
            "hook": 75.0,
        }.get(shape, 35.0)
        angle = default_angle if guidewire.tip_curve_angle_deg is None else float(guidewire.tip_curve_angle_deg)
        if shape == "straight":
            angle = 0.0

        params: dict[str, Any] = {
            "jtip_deg": angle,
        }
        if guidewire.tip_length_mm is not None:
            params["jtip_bodies"] = max(0, int(round(float(guidewire.tip_length_mm) / 3.0)))
        elif shape == "straight":
            params["jtip_bodies"] = 0
        elif shape in {"c_shape", "hook"}:
            params["jtip_bodies"] = 5
        else:
            params["jtip_bodies"] = 3

        if guidewire.soft_tip_length_mm is not None:
            params["soft_tip"] = max(0, int(round(float(guidewire.soft_tip_length_mm) / 3.0)))
        if guidewire.tip_stiffness is not None:
            # Public config uses 0=very soft, 1=shaft-like. Newton uses bend
            # stiffness, so map onto the existing soft-tip range.
            params["tip_bend"] = 0.5 + float(guidewire.tip_stiffness) * 19.5
        params["tip_shape"] = shape
        return params

    async def _handle_device_config(
        self, conn_state: ConnectionState, data: dict
    ) -> None:
        """Apply semantic guidewire device configuration to the live backend."""
        if not conn_state.session_id:
            await self._send_error(conn_state, "NO_SESSION", "No active session")
            return
        try:
            config = DeviceConfigData(**data)
        except ValidationError as e:
            await self._send_error(conn_state, "INVALID_PARAMS", str(e))
            return
        try:
            engine = self._session_manager.get_session(conn_state.session_id)
        except KeyError:
            conn_state.session_id = None
            await self._send_error(conn_state, "SESSION_EXPIRED", "Session no longer exists")
            return

        mapped = self._device_config_to_engine_params(config)
        engine_params = {key: value for key, value in mapped.items() if key != "tip_shape"}
        try:
            effective = await self._run_blocking(engine.set_engine_params, engine_params)
        except Exception as e:  # noqa: BLE001
            await self._send_error(conn_state, "DEVICE_CONFIG_ERROR", f"{type(e).__name__}: {e}")
            return

        await self._send_message(
            conn_state,
            MessageType.DEVICE_CONFIG,
            session_id=conn_state.session_id,
            data={
                "effective": effective or {},
                "guidewire": {
                    **config.guidewire.model_dump(),
                    "tip_shape": mapped["tip_shape"],
                },
            },
        )

    async def _handle_engine_params(
        self, conn_state: ConnectionState, data: dict
    ) -> None:
        """Live-tune backend guidewire deformation params (interactive panel).

        ``data`` is a flat dict of param -> value (bend / tip_bend / soft_tip /
        stretch / push_speed / rotate_speed / jtip_deg / ...). Applied to the live
        engine; the effective (clamped) state is echoed back so the client HUD can
        sync its sliders. No step is run -- the next control frame reflects it.
        """
        if not conn_state.session_id:
            await self._send_error(conn_state, "NO_SESSION", "No active session")
            return
        try:
            engine = self._session_manager.get_session(conn_state.session_id)
        except KeyError:
            conn_state.session_id = None
            await self._send_error(conn_state, "SESSION_EXPIRED", "Session no longer exists")
            return

        try:
            effective = await self._run_blocking(engine.set_engine_params, dict(data))
        except Exception as e:  # noqa: BLE001 - surface any tuning error to the client
            await self._send_error(conn_state, "PARAM_ERROR", f"{type(e).__name__}: {e}")
            return

        await self._send_message(
            conn_state,
            MessageType.ENGINE_PARAMS,
            session_id=conn_state.session_id,
            data={"effective": effective or {}},
        )

    async def _handle_shape_intent(
        self, conn_state: ConnectionState, data: dict
    ) -> None:
        """Engage/adjust ShapeIntent (autopilot) control (doc/09).

        Physics stays pure force-driven; this only steers where the tip aims.
        While active, the server's control steps derive push/rotate from the
        intent, so the client can keep ticking (or stop sending manual control).
        The resulting control state is echoed back for the client HUD.
        """
        if not conn_state.session_id:
            await self._send_error(conn_state, "NO_SESSION", "No active session")
            return
        try:
            req = ShapeIntentData(**data)
        except ValidationError as e:
            await self._send_error(conn_state, "INVALID_PARAMS", str(e))
            return

        try:
            engine = self._session_manager.get_session(conn_state.session_id)
        except KeyError:
            conn_state.session_id = None
            await self._send_error(conn_state, "SESSION_EXPIRED", "Session no longer exists")
            return

        intent = {
            "target_waypoint": req.target_waypoint,
            "target_direction": req.target_direction,
            "intensity": req.intensity,
        }
        try:
            result = await self._run_blocking(engine.set_shape_intent, intent, req.active)
        except Exception as e:  # noqa: BLE001 - surface any control error to the client
            await self._send_error(conn_state, "INTENT_ERROR", f"{type(e).__name__}: {e}")
            return

        await self._send_message(
            conn_state,
            MessageType.SHAPE_INTENT,
            session_id=conn_state.session_id,
            data=result,
        )

    async def _handle_reset(
        self, conn_state: ConnectionState, data: dict
    ) -> None:
        """Handle reset command."""
        if not conn_state.session_id:
            await self._send_error(
                conn_state, "NO_SESSION", "No active session to reset"
            )
            return

        try:
            state = await self._run_blocking(
                self._session_manager.reset_session, conn_state.session_id
            )
            info = self._session_manager.get_session_info(conn_state.session_id)

            if conn_state.batch_mode:
                engine = self._session_manager.get_session(conn_state.session_id)
                payload = {
                    **self._state_to_batch(state, engine, include_path=True),
                    "episode_count": info.episode_count,
                }
                conn_state.path_sent = True
                msg_type = MessageType.STATE_BATCH
            else:
                payload = {
                    **self._state_to_dict(state),
                    "episode_count": info.episode_count,
                }
                msg_type = MessageType.STATE_UPDATE

            await self._send_message(
                conn_state,
                msg_type,
                session_id=conn_state.session_id,
                data=payload,
            )

        except KeyError:
            conn_state.session_id = None
            await self._send_error(
                conn_state, "SESSION_EXPIRED", "Session no longer exists"
            )

    def _safety_level_from_status(
        self,
        status: str,
        assessment: dict[str, Any],
        risk_score: float,
        wall_distance: float,
    ) -> str:
        """Map source-backed safety/risk state to the visual safety vocabulary."""
        if status == "COLLISION_STOP":
            return "stop"
        if status == "STANDBY":
            return "stale"

        if risk_score >= 0.75:
            return "danger"
        if risk_score >= 0.35:
            return "warning"
        if 0.0 < wall_distance < 0.0008:
            return "danger"
        if 0.0 < wall_distance < 0.0015:
            return "warning"

        risk_level = (
            str(assessment.get("risk_level", "SAFE")).upper()
            if isinstance(assessment, dict)
            else "SAFE"
        )
        if risk_level in {"CRITICAL", "WARNING"} and risk_score > 0.0:
            return "warning"
        if status == "DANGER_WARNING" and risk_score > 0.0:
            return "warning"
        if status == "SAFE_NAV":
            return "safe"
        return "stale"

    def _reason_codes_from_assessment(self, assessment: dict[str, Any], status: str) -> list[str]:
        """Expose reason codes from real risk-assessment metrics only."""
        reason_codes: list[str] = []
        metrics = assessment.get("metrics", {}) if isinstance(assessment, dict) else {}
        if isinstance(metrics, dict):
            for name, metric in metrics.items():
                if not isinstance(metric, dict):
                    continue
                level = str(metric.get("level", "SAFE"))
                if level != "SAFE":
                    reason_codes.append(f"{str(name).upper()}_{level}")
        if status == "COLLISION_STOP" and not reason_codes:
            reason_codes.append("BACKEND_COLLISION_STOP")
        elif status == "DANGER_WARNING" and not reason_codes:
            reason_codes.append("BACKEND_DANGER_WARNING")
        elif status == "STANDBY":
            reason_codes.append("NOT_RUNNING")
        return reason_codes

    def _guidewire_mechanics(self, state: NavigationState, timestamp_ms: int) -> dict[str, Any]:
        """Build source-backed mechanics data without inventing unavailable forces."""
        safety_level = self._safety_level_from_status(
            state.safety_status,
            state.risk_assessment,
            state.risk_score,
            state.wall_distance,
        )
        return {
            "timestamp_ms": timestamp_ms,
            "tip_force_n": state.contact_force,
            "wall_distance_m": state.wall_distance,
            "lateral_force_n": None,
            "axial_force_n": None,
            "torque_nm": None,
            "contact_count": max(
                int(getattr(state, "wall_contact_count", 0)),
                1 if state.contact_force > 0.0 else 0,
            ),
            "max_penetration_m": float(getattr(state, "max_penetration", 0.0)),
            "contact_impulse": float(getattr(state, "contact_impulse", 0.0)),
            "risk_score": state.risk_score,
            "safety_level": safety_level,
            "stop_required": state.safety_status == "COLLISION_STOP",
            "reason_codes": self._reason_codes_from_assessment(
                state.risk_assessment, state.safety_status
            ),
            "source": "navigation_engine.risk_assessor",
            "source_fields": [
                "contact_force",
                "wall_contact_count",
                "max_penetration",
                "contact_impulse",
                "wall_distance",
                "curvature",
                "velocity",
                "path_deviation",
                "safety_status",
                "risk_score",
            ],
        }

    def _state_to_dict(self, state: NavigationState) -> dict:
        """Convert NavigationState to dictionary for WebSocket transmission.

        Emits the full state (including wall_distance, curvature, path_progress,
        path_deviation, safety_status and tip_quaternion) per the state_update
        schema in doc/03-API与通信协议.md §1.4.
        """
        return state.as_dict()

    def _guidewire_state_from_diagnostics(self, diagnostics: dict[str, Any]) -> dict[str, Any]:
        diag = diagnostics if isinstance(diagnostics, dict) else {}
        guidewire = diag.get("guidewire")
        design = default_guidewire_design(
            active_sim_length_mm=float(diag.get("rod_length_m", 0.06)) * 1000.0
        )
        profile = design.profile
        result = {
            **design.diagnostics(),
            "profile_name": profile.name,
            "tip_shape": profile.tip_shape.shape_type,
            "current_tip_segment": "pre_shaped_soft_tip",
            "torsion_lag_deg": None,
            "total_length_mm": profile.total_length_mm,
            "segment_count": len(profile.segments),
            "jtip_deg": profile.tip_shape.precurve_angle_deg,
            "jtip_bodies": None,
        }
        if isinstance(guidewire, dict) and guidewire:
            result.update(guidewire)
        result["tip_shape_label"] = _cn_label(_TIP_SHAPE_LABELS, result.get("tip_shape"))
        result["current_tip_segment_label"] = _cn_label(
            _GUIDEWIRE_SEGMENT_LABELS,
            result.get("current_tip_segment"),
        )
        result["torsion_lag_deg_label"] = "扭转滞后"
        return result


    def _procedure_state_from_diagnostics(
        self,
        diagnostics: dict[str, Any],
        guidewire: dict[str, Any],
    ) -> dict[str, Any]:
        diag = diagnostics if isinstance(diagnostics, dict) else {}
        procedure = diag.get("procedure")
        if isinstance(procedure, dict) and procedure:
            result = dict(procedure)
        else:
            design = default_procedure_design()
            summary = guidewire.get("summary_zh") if isinstance(guidewire, dict) else None
            result = design.diagnostics(summary if isinstance(summary, str) else None)
        if not result.get("guidewire_summary") and isinstance(guidewire, dict):
            result["guidewire_summary"] = guidewire.get("summary_zh", "unknown")
        return result

    def _guidewire_state_with_render_lengths(
        self,
        guidewire: dict[str, Any],
        bodies: list[dict[str, Any]],
    ) -> dict[str, Any]:
        result = dict(guidewire)
        intravascular_length_mm = self._body_polyline_length_mm(bodies)
        clinical_total = result.get("clinical_total_length_mm")
        external_tail_length_mm = None
        if isinstance(clinical_total, (int, float)):
            external_tail_length_mm = max(0.0, float(clinical_total) - intravascular_length_mm)
        result.update({
            "intravascular_length_mm": intravascular_length_mm,
            "external_tail_length_mm": external_tail_length_mm,
            "render_scope": "intravascular_only",
        })
        return result

    @staticmethod
    def _body_polyline_length_mm(bodies: list[dict[str, Any]]) -> float:
        points: list[list[float]] = []
        for body in bodies:
            if not isinstance(body, dict):
                continue
            pos = body.get("pos")
            if isinstance(pos, list) and len(pos) >= 3:
                try:
                    points.append([float(pos[0]), float(pos[1]), float(pos[2])])
                except (TypeError, ValueError):
                    continue
        total_m = 0.0
        for prev, cur in zip(points, points[1:]):
            dx = cur[0] - prev[0]
            dy = cur[1] - prev[1]
            dz = cur[2] - prev[2]
            total_m += (dx * dx + dy * dy + dz * dz) ** 0.5
        return total_m * 1000.0

    def _support_state_from_diagnostics(self, diagnostics: dict[str, Any]) -> dict[str, Any]:
        diag = diagnostics if isinstance(diagnostics, dict) else {}
        support = diag.get("support")
        if isinstance(support, dict) and support:
            result = dict(support)
        else:
            rod_length_m = float(diag.get("rod_length_m", 0.06))
            free_len_m = float(diag.get("free_len_m", min(0.03, rod_length_m)))
            system = default_support_system(
                guidewire_tip_arclen_mm=rod_length_m * 1000.0,
                free_wire_length_mm=free_len_m * 1000.0,
            )
            result = {
                "effective_support_type": system.effective_support_type(),
                "effective_support_tip_mm": system.effective_support_tip(),
                "support_ratio": system.support_ratio(rod_length_m * 1000.0),
                "free_wire_length_mm": system.free_wire_length_mm(rod_length_m * 1000.0),
            }
        result["effective_support_type_label"] = _cn_label(
            _SUPPORT_TYPE_LABELS,
            result.get("effective_support_type"),
        )
        return result

    def _segmented_risk_state(self, state: NavigationState, diagnostics: dict[str, Any]) -> dict[str, Any]:
        """Expose guidewire-specific risk fields backed by live diagnostics."""
        diag = diagnostics if isinstance(diagnostics, dict) else {}
        slack_m = diag.get("slack_m")
        max_slack_m = diag.get("max_slack_m")
        max_breach_m = diag.get("max_breach_m")
        normal_poking_score = diag.get("normal_poking_score")
        tangential_slide_score = diag.get("tangential_slide_score")

        pile_ratio = None
        if isinstance(slack_m, (int, float)) and isinstance(max_slack_m, (int, float)) and max_slack_m > 0:
            pile_ratio = max(0.0, float(slack_m) / float(max_slack_m))

        if pile_ratio is None:
            buckling_risk = "LOW" if state.safety_status == "SAFE_NAV" else "UNKNOWN"
        elif pile_ratio >= 0.85:
            buckling_risk = "HIGH"
        elif pile_ratio >= 0.5:
            buckling_risk = "MEDIUM"
        else:
            buckling_risk = "LOW"

        wall_slide_state = str(diag.get("wall_slide_state", "WALL_SLIDE_OK" if state.safety_status == "SAFE_NAV" else state.safety_status))
        breach_mm = None
        if isinstance(max_breach_m, (int, float)):
            breach_mm = max(0.0, float(max_breach_m) * 1000.0)

        normal_poking_value = normal_poking_score if isinstance(normal_poking_score, (int, float)) else None
        tangential_slide_value = tangential_slide_score if isinstance(tangential_slide_score, (int, float)) else None
        buckling_text = _cn_label(_BUCKLING_RISK_LABELS, buckling_risk)
        wall_slide_text = _cn_label(_WALL_SLIDE_STATE_LABELS, wall_slide_state)
        normal_poking_text = _score_text(normal_poking_value)
        tangential_slide_text = _score_text(tangential_slide_value)

        return {
            "slack_mm": float(slack_m) * 1000.0 if isinstance(slack_m, (int, float)) else None,
            "pile_ratio": pile_ratio,
            "contact_force": state.contact_force,
            "wall_contact_count": int(getattr(state, "wall_contact_count", 0)),
            "contact_impulse": float(getattr(state, "contact_impulse", 0.0)),
            "max_penetration_mm": float(getattr(state, "max_penetration", 0.0)) * 1000.0,
            "breach_mm": breach_mm,
            "wall_slide_state": wall_slide_state,
            "wall_slide_state_label": "贴壁状态",
            "wall_slide_state_text": wall_slide_text,
            "buckling_risk": buckling_risk,
            "buckling_risk_label": "屈曲风险",
            "buckling_risk_text": buckling_text,
            "normal_poking_score": normal_poking_value,
            "normal_poking_score_label": "顶壁风险",
            "normal_poking_score_text": normal_poking_text,
            "tangential_slide_score": tangential_slide_value,
            "tangential_slide_score_label": "贴壁滑入",
            "tangential_slide_score_text": tangential_slide_text,
            "display": {
                "wall_slide_state": {"name": "贴壁状态", "value": wall_slide_text},
                "buckling_risk": {"name": "屈曲风险", "value": buckling_text},
                "normal_poking_score": {"name": "顶壁风险", "value": normal_poking_text},
                "tangential_slide_score": {"name": "贴壁滑入", "value": tangential_slide_text},
            },
        }

    def _state_to_batch(
        self,
        state: NavigationState,
        engine: NavigationEngine,
        include_path: bool = True,
    ) -> dict:
        """Build the state_batch payload with guidewire render data.

        Follows the state_batch structure in doc/03-API与通信协议.md §1.4,
        adding per-segment body positions (for tube rendering), the planned path,
        and aggregated safety/episode blocks.

        The planned path is constant for a session and can be large (thousands of
        points for VPP routes). It is only included when ``include_path`` is True
        (first batch / reset); subsequent batches omit ``waypoints`` to keep each
        frame small. The client keeps its existing path drawing when waypoints is
        empty.
        """
        timestamp_ms = int(time.time() * 1000)
        backend = getattr(engine, "_engine", None)
        diagnostics = {}
        if backend is not None and hasattr(backend, "diagnostics"):
            diagnostics = backend.diagnostics()
        mechanics = {}
        if backend is not None and hasattr(backend, "mechanics_state"):
            mechanics = backend.mechanics_state()
        guidewire_state = mechanics.get("guidewire", {}) if isinstance(mechanics, dict) else {}
        support_state = mechanics.get("support", {}) if isinstance(mechanics, dict) else {}
        mechanics_risk = mechanics.get("risk", {}) if isinstance(mechanics, dict) else {}
        flow_guidance = state.flow_guidance if isinstance(state.flow_guidance, dict) else {}
        flow_tip_shape = flow_guidance.get("tip_shape", {})
        flow_support = flow_guidance.get("support", {})
        training_score = flow_guidance.get("training_score", {})
        guidewire = self._guidewire_state_from_diagnostics(diagnostics)
        bodies = engine.get_render_bodies()
        guidewire = self._guidewire_state_with_render_lengths(guidewire, bodies)
        support = self._support_state_from_diagnostics(diagnostics)
        procedure = self._procedure_state_from_diagnostics(diagnostics, guidewire)
        risk = self._segmented_risk_state(state, diagnostics)
        guidewire_payload = {**flow_tip_shape, **guidewire, **guidewire_state}
        support_payload = {**flow_support, **support, **support_state}
        risk_payload = {
            **risk,
            **mechanics_risk,
            "safety_status": state.safety_status,
            "risk_score": state.risk_score,
            "risk_regions": state.risk_regions,
        }
        return {
            "schema_version": "navigation_visual_v2",
            "timestamp_ms": timestamp_ms,
            "engine": type(backend).__name__ if backend is not None else "",
            "fidelity_mode": state.fidelity_mode,
            "diagnostics": diagnostics,
            "guidewire": guidewire_payload,
            "support": support_payload,
            "procedure": procedure,
            "risk": risk_payload,
            "tip": {
                "position": state.tip_position,
                "direction": state.tip_direction,
                "quaternion": state.tip_quaternion,
            },
            "bodies": bodies,
            "path": {
                "waypoints": engine.planned_path if include_path else [],
                "progress": state.path_progress,
                "deviation": state.path_deviation,
                "remaining_distance": state.remaining_distance,
                "vessel_radius": state.vessel_radius,
                "eta_seconds": state.eta_seconds,
            },
            # Vessel-entry (vascular access) and target markers, for highlighting
            # in the client. The entry is constant for a session, so it rides the
            # same first-batch/reset frame as the path; the target is small and
            # sent every frame.
            "entry": engine.entry_pose if include_path else {},
            "target": state.target_position,
            "safety": {
                "status": state.safety_status,
                "wall_distance": state.wall_distance,
                "curvature": state.curvature,
                "speed": state.velocity,
                "risk_score": state.risk_score,
                "risk_regions": state.risk_regions,
                "guidewire_mechanics": self._guidewire_mechanics(state, timestamp_ms),
                "flow_guidance": state.flow_guidance,
            },
            "flow_guidance": state.flow_guidance,
            "training_score": training_score,
            "episode": {
                "length": state.episode_length,
                "reward": state.reward,
                "done": state.done,
            },
        }

    async def _send_message(
        self,
        conn_state: ConnectionState,
        msg_type: MessageType,
        session_id: str | None = None,
        data: dict | None = None,
    ) -> None:
        """Send a WebSocket message."""
        message = {
            "type": msg_type.value,
            "session_id": session_id,
            "timestamp": int(time.time() * 1000),
            "data": data or {},
        }
        try:
            await conn_state.websocket.send_json(message)
        except Exception as e:  # noqa: BLE001 - log why the send failed before closing
            print(f"[WS] send failed ({msg_type.value}): {type(e).__name__}: {e}")
            conn_state.is_alive = False

    async def _send_error(
        self, conn_state: ConnectionState, code: str, message: str
    ) -> None:
        """Send an error message."""
        await self._send_message(
            conn_state,
            MessageType.ERROR,
            session_id=conn_state.session_id,
            data={"code": code, "message": message},
        )

    async def _cleanup_connection(self, conn_state: ConnectionState) -> None:
        """Clean up resources when connection closes."""
        print(f"[WS] cleanup: closing connection (session={conn_state.session_id})")
        if conn_state.session_id:
            self._session_manager.close_session(conn_state.session_id)

        self._connections.pop(conn_state.websocket, None)

        try:
            await conn_state.websocket.close()
        except Exception:
            pass

    @property
    def active_connections(self) -> int:
        """Number of active WebSocket connections."""
        return len(self._connections)


_websocket_handler: WebSocketHandler | None = None


def get_websocket_handler(session_manager: SessionManager) -> WebSocketHandler:
    """Get or create the global WebSocketHandler instance."""
    global _websocket_handler
    if _websocket_handler is None:
        _websocket_handler = WebSocketHandler(session_manager)
    return _websocket_handler
