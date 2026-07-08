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
from typing import Any, Callable

from fastapi import WebSocket, WebSocketDisconnect
from pydantic import BaseModel, Field, ValidationError

from services.navigation_engine import NavigationEngine, NavigationState
from services.path_planner import PathPlanner
from services.session_manager import SessionManager

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
_DATA_ROOT = _PROJECT_ROOT / "data" / "vpp_assets"

# Built-in sealed-lumen phantoms can run physical backends despite shipping a
# centerline. Other centerline phantoms default to kinematic guidance unless an
# explicit engine override (Newton demo) is active.
_PHYSICS_CAPABLE_BUILTIN: frozenset[str] = frozenset({"aorta_trunk"})


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


@lru_cache(maxsize=8)
def _get_path_planner(case_id: str) -> PathPlanner:
    """Load (and cache) a PathPlanner for the given case's VPP graph."""
    graph_path = _DATA_ROOT / case_id / "graph" / "graph.json"
    if not graph_path.is_file():
        raise FileNotFoundError(f"Graph not found for case: {case_id}")
    return PathPlanner(graph_path)


class MessageType(str, Enum):
    """WebSocket message types."""

    # Client -> Server
    CONTROL = "control"
    SESSION_START = "session_start"
    SESSION_STOP = "session_stop"
    PATH_REQUEST = "path_request"
    SELECT_ROUTE = "select_route"
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
            planned_path = await self._resolve_session_path(params)
        except (FileNotFoundError, ValueError) as e:
            await self._send_error(conn_state, "PATH_NOT_FOUND", str(e))
            return

        # Full-length vessels the physical guidewire cannot traverse default to
        # kinematic centerline-follow. Sealed built-ins and explicit Newton demo
        # runs must remain physical so the selected backend actually owns motion.
        ships_centerline = _builtin_phantom_centerline(params.phantom) is not None
        force_guided_builtin = (
            ships_centerline and params.phantom not in _PHYSICS_CAPABLE_BUILTIN
        )
        guided = (
            params.guided
            or (planned_path is not None and params.phantom.endswith("_vpp"))
            or force_guided_builtin
        )
        newton_demo = os.environ.get("CATHSIM_PHYSICS_ENGINE", "").lower() in {
            "newton",
            "newton_demo",
        }
        if newton_demo:
            guided = False

        try:
            session_id, state = await self._run_blocking(
                self._session_manager.create_session,
                phantom=params.phantom,
                target=params.target,
                use_pixels=params.use_pixels,
                n_bodies=params.n_bodies,
                n_substeps=params.n_substeps,
                planned_path=planned_path,
                guided=guided,
                route_target=params.route_target,
            )
            conn_state.session_id = session_id
            conn_state.batch_mode = params.batch_mode

            # Multi-branch phantoms expose their selectable branch tips so the
            # client can offer targets and switch via a select_route message.
            engine = self._session_manager.get_session(session_id)
            await self._send_message(
                conn_state,
                MessageType.SESSION_STARTED,
                session_id=session_id,
                data={
                    "phantom": params.phantom,
                    "target": params.target,
                    "state": self._state_to_dict(state),
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
    ) -> list[list[float]] | None:
        """Resolve the planned path (MuJoCo meters) for a navigation session.

        An explicit ``planned_path`` is used as-is. Otherwise, when both
        ``start_position`` and ``end_position`` (LPS millimeters) are given, the
        path is planned server-side and converted to meters. Returns None when no
        path is requested (e.g. plain low_tort sessions).
        """
        if params.planned_path is not None:
            return params.planned_path

        if params.start_position is None or params.end_position is None:
            return None

        planner = _get_path_planner(params.case_id)
        result = await self._run_blocking(
            planner.plan,
            params.start_position,
            params.end_position,
            smooth=params.smooth,
            smooth_factor=params.smooth_factor,
        )
        waypoints = result.smooth_waypoints or result.waypoints
        # Graph/planner coordinates are LPS millimeters; the MuJoCo phantom frame
        # is meters (V-HACD applied a /1000 scale with no axis change).
        return [[c / 1000.0 for c in point] for point in waypoints]

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

    def _state_to_dict(self, state: NavigationState) -> dict:
        """Convert NavigationState to dictionary for WebSocket transmission.

        Emits the full state (including wall_distance, curvature, path_progress,
        path_deviation, safety_status and tip_quaternion) per the state_update
        schema in doc/03-API与通信协议.md §1.4.
        """
        return state.as_dict()

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
        return {
            "engine": type(engine._engine).__name__ if getattr(engine, "_engine", None) is not None else "",
            "fidelity_mode": state.fidelity_mode,
            "tip": {
                "position": state.tip_position,
                "direction": state.tip_direction,
                "quaternion": state.tip_quaternion,
            },
            "bodies": engine.get_render_bodies(),
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
            },
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
