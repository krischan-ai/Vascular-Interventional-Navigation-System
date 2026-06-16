"""WebSocket handler for real-time CathSim control and state streaming.

This module implements the WebSocket protocol defined in doc/03-API与通信协议.md.
"""

from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable

from fastapi import WebSocket, WebSocketDisconnect
from pydantic import BaseModel, Field, ValidationError

from services.navigation_engine import NavigationState
from services.session_manager import SessionManager


class MessageType(str, Enum):
    """WebSocket message types."""

    # Client -> Server
    CONTROL = "control"
    SESSION_START = "session_start"
    SESSION_STOP = "session_stop"
    RESET = "reset"
    PONG = "pong"

    # Server -> Client
    STATE_UPDATE = "state_update"
    STATE_BATCH = "state_batch"
    ERROR = "error"
    PING = "ping"
    SESSION_STARTED = "session_started"
    SESSION_STOPPED = "session_stopped"


class ControlData(BaseModel):
    """Control command data."""

    delta_push: float = Field(ge=-1.0, le=1.0)
    delta_rotate: float = Field(ge=-1.0, le=1.0)


class SessionStartData(BaseModel):
    """Session start request data."""

    phantom: str = "low_tort"
    target: str = "bca"
    use_pixels: bool = False


class ResetData(BaseModel):
    """Reset request data."""

    randomize: bool = False


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
    PONG_TIMEOUT = 15.0  # seconds
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
                await self._send_error(
                    conn_state, "PARSE_ERROR", f"Invalid message: {e}"
                )

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
            session_id, state = self._session_manager.create_session(
                phantom=params.phantom,
                target=params.target,
                use_pixels=params.use_pixels,
            )
            conn_state.session_id = session_id

            engine = self._session_manager.get_session(session_id)

            await self._send_message(
                conn_state,
                MessageType.SESSION_STARTED,
                session_id=session_id,
                data={
                    "phantom": params.phantom,
                    "target": params.target,
                    "state": self._state_to_dict(state, engine),
                },
            )

        except RuntimeError as e:
            await self._send_error(conn_state, "SESSION_ERROR", str(e))

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
            state = self._session_manager.step(
                session_id=conn_state.session_id,
                delta_push=control.delta_push,
                delta_rotate=control.delta_rotate,
            )
            engine = self._session_manager.get_session(conn_state.session_id)

            await self._send_message(
                conn_state,
                MessageType.STATE_UPDATE,
                session_id=conn_state.session_id,
                data=self._state_to_dict(state, engine),
            )

        except KeyError:
            conn_state.session_id = None
            await self._send_error(
                conn_state, "SESSION_EXPIRED", "Session no longer exists"
            )
        except RuntimeError as e:
            await self._send_error(conn_state, "STEP_ERROR", str(e))

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
            state = self._session_manager.reset_session(conn_state.session_id)
            engine = self._session_manager.get_session(conn_state.session_id)
            info = self._session_manager.get_session_info(conn_state.session_id)

            await self._send_message(
                conn_state,
                MessageType.STATE_UPDATE,
                session_id=conn_state.session_id,
                data={
                    **self._state_to_dict(state, engine),
                    "episode_count": info.episode_count,
                },
            )

        except KeyError:
            conn_state.session_id = None
            await self._send_error(
                conn_state, "SESSION_EXPIRED", "Session no longer exists"
            )

    def _state_to_dict(self, state: NavigationState, engine) -> dict:
        """Convert NavigationState to dictionary for WebSocket transmission."""
        return {
            "tip_position": state.tip_position,
            "tip_direction": state.tip_direction,
            "velocity": state.velocity,
            "contact_force": state.contact_force,
            "episode_length": state.episode_length,
            "target_position": state.target_position,
            "reward": state.reward,
            "done": state.done,
            "safety_status": engine.get_safety_status(state),
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
        except Exception:
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
