"""Session Manager: Manages multiple CathSim navigation sessions.

This module provides session lifecycle management for concurrent
navigation simulations.
"""

from __future__ import annotations

import threading
import time
import uuid
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any

from services.navigation_engine import NavigationEngine, NavigationState


class ControlRejectedError(RuntimeError):
    """Raised when a session safety latch rejects a control command."""


@dataclass
class SessionInfo:
    """Metadata about a navigation session."""

    session_id: str
    phantom: str
    target: str
    created_at: datetime
    last_active: datetime
    episode_count: int = 0
    total_steps: int = 0
    emergency_stop_latched: bool = False
    emergency_stop_reason: str | None = None
    emergency_stop_at: datetime | None = None
    jtip_assist_enabled: bool = True
    jtip_assist_supported: bool = False
    jtip_angle_deg: float = 35.0
    torque_limit_enabled: bool = True
    torque_command_limit: float = 0.5
    withdrawal_protection_enabled: bool = True
    auto_stop_push_enabled: bool = True
    last_state: NavigationState | None = field(default=None, repr=False)
    last_applied_control: dict[str, Any] = field(default_factory=dict)
    rejected_control_count: int = 0

    def as_dict(self) -> dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "session_id": self.session_id,
            "phantom": self.phantom,
            "target": self.target,
            "created_at": self.created_at.isoformat(),
            "last_active": self.last_active.isoformat(),
            "episode_count": self.episode_count,
            "total_steps": self.total_steps,
            "control_state": {
                "emergency_stop_latched": self.emergency_stop_latched,
                "emergency_stop_reason": self.emergency_stop_reason,
                "emergency_stop_at": (
                    self.emergency_stop_at.isoformat()
                    if self.emergency_stop_at is not None
                    else None
                ),
                "protections": {
                    "jtip_assist_enabled": self.jtip_assist_enabled,
                    "jtip_assist_supported": self.jtip_assist_supported,
                    "torque_limit_enabled": self.torque_limit_enabled,
                    "torque_command_limit": self.torque_command_limit,
                    "withdrawal_protection_enabled": self.withdrawal_protection_enabled,
                    "auto_stop_push_enabled": self.auto_stop_push_enabled,
                },
                "last_applied_control": dict(self.last_applied_control),
                "rejected_control_count": self.rejected_control_count,
            },
        }


class SessionManager:
    """Manages multiple CathSim navigation sessions.

    This class provides thread-safe session management with:
    - Session creation and destruction
    - Session lookup and validation
    - Automatic cleanup of expired sessions

    Example:
        manager = SessionManager()
        session_id, state = manager.create_session(phantom="low_tort", target="bca")
        state = manager.step(session_id, delta_push=0.5, delta_rotate=0.1)
        manager.close_session(session_id)
    """

    DEFAULT_SESSION_TIMEOUT = 3600  # 1 hour

    def __init__(self, max_sessions: int = 10, session_timeout: int = DEFAULT_SESSION_TIMEOUT):
        """Initialize session manager.

        Args:
            max_sessions: Maximum number of concurrent sessions
            session_timeout: Session timeout in seconds (default 1 hour)
        """
        self._max_sessions = max_sessions
        self._session_timeout = session_timeout

        self._sessions: dict[str, NavigationEngine] = {}
        self._session_info: dict[str, SessionInfo] = {}
        self._lock = threading.RLock()

    def create_session(
        self,
        phantom: str = "low_tort",
        target: str = "bca",
        use_pixels: bool = False,
        assets_dir: str | None = None,
        planned_path=None,
        planned_radii=None,
        n_bodies: int = 80,
        n_substeps: int | None = None,
        guided: bool = False,
        physics_engine: str | None = None,
        route_target: str | None = None,
    ) -> tuple[str, NavigationState]:
        """Create a new navigation session.

        Args:
            phantom: Phantom model name
            target: Target site name
            use_pixels: Whether to include pixel observations
            assets_dir: Optional phantom assets directory for VPP phantoms
            planned_path: Optional planned path ([x, y, z] points in meters) for
                          path progress/deviation tracking

        Returns:
            Tuple of (session_id, initial_state)

        Raises:
            RuntimeError: If max sessions reached
        """
        with self._lock:
            self._cleanup_expired_sessions()

            if len(self._sessions) >= self._max_sessions:
                raise RuntimeError(
                    f"Maximum sessions ({self._max_sessions}) reached. "
                    "Close existing sessions first."
                )

            session_id = str(uuid.uuid4())

            engine = NavigationEngine(
                phantom=phantom,
                target=target,
                use_pixels=use_pixels,
                assets_dir=assets_dir,
                planned_path=planned_path,
                planned_radii=planned_radii,
                n_bodies=n_bodies,
                n_substeps=n_substeps,
                guided=guided,
                physics_engine=physics_engine,
                route_target=route_target,
            )

            initial_state = engine.reset()

            self._sessions[session_id] = engine
            deform_params = engine.set_engine_params({})
            jtip_supported = deform_params is not None and "jtip_deg" in deform_params
            jtip_angle = (
                float(deform_params["jtip_deg"])
                if jtip_supported and float(deform_params["jtip_deg"]) > 0.0
                else 35.0
            )
            self._session_info[session_id] = SessionInfo(
                session_id=session_id,
                phantom=phantom,
                target=target,
                created_at=datetime.now(),
                last_active=datetime.now(),
                episode_count=1,
                total_steps=0,
                jtip_assist_enabled=(
                    bool(float(deform_params["jtip_deg"]) > 0.0)
                    if jtip_supported
                    else False
                ),
                jtip_assist_supported=jtip_supported,
                jtip_angle_deg=jtip_angle,
                last_state=initial_state,
            )

            return session_id, initial_state

    def get_session(self, session_id: str) -> NavigationEngine:
        """Get a session by ID.

        Args:
            session_id: Session UUID

        Returns:
            NavigationEngine instance

        Raises:
            KeyError: If session not found
        """
        with self._lock:
            if session_id not in self._sessions:
                raise KeyError(f"Session not found: {session_id}")

            self._session_info[session_id].last_active = datetime.now()
            return self._sessions[session_id]

    def step(
        self,
        session_id: str,
        delta_push: float,
        delta_rotate: float,
        microcatheter_advance: float = 0.0,
    ) -> NavigationState:
        """Execute a step in the specified session.

        Args:
            session_id: Session UUID
            delta_push: Push force coefficient [-1.0, 1.0]
            delta_rotate: Rotation force coefficient [-1.0, 1.0]
            microcatheter_advance: Optional support advance coefficient [-1.0, 1.0]

        Returns:
            NavigationState after the step
        """
        with self._lock:
            if session_id not in self._sessions:
                raise KeyError(f"Session not found: {session_id}")
            info = self._session_info[session_id]
            if info.emergency_stop_latched:
                if (
                    abs(delta_push) > 1e-9
                    or abs(delta_rotate) > 1e-9
                    or abs(microcatheter_advance) > 1e-9
                ):
                    info.rejected_control_count += 1
                    raise ControlRejectedError(
                        "Emergency stop is latched; non-zero control was rejected"
                    )
                if info.last_state is None:
                    raise ControlRejectedError("Emergency stop is latched")
                info.last_applied_control = {
                    "requested_push": float(delta_push),
                    "requested_rotate": float(delta_rotate),
                    "requested_support": float(microcatheter_advance),
                    "applied_push": 0.0,
                    "applied_rotate": 0.0,
                    "applied_support": 0.0,
                    "protection_reasons": ["EMERGENCY_STOP_LATCHED"],
                }
                return info.last_state

            applied_push = float(delta_push)
            applied_rotate = float(delta_rotate)
            applied_support = float(microcatheter_advance)
            protection_reasons: list[str] = []
            if (
                info.torque_limit_enabled
                and abs(applied_rotate) > info.torque_command_limit
            ):
                applied_rotate = max(
                    -info.torque_command_limit,
                    min(info.torque_command_limit, applied_rotate),
                )
                protection_reasons.append("TORQUE_COMMAND_LIMITED")

            previous = info.last_state
            if info.withdrawal_protection_enabled and applied_push < 0.0 and previous:
                travelled = previous.path_travelled_distance
                at_entry = (
                    travelled is not None and travelled <= 1e-9
                ) or (travelled is None and previous.path_progress <= 0.0)
                if at_entry:
                    applied_push = 0.0
                    protection_reasons.append("WITHDRAWAL_AT_ENTRY_BLOCKED")
            if (
                info.auto_stop_push_enabled
                and applied_push > 0.0
                and previous is not None
                and previous.safety_status == "COLLISION_STOP"
            ):
                applied_push = 0.0
                protection_reasons.append("SAFETY_STOP_PUSH_BLOCKED")

            engine = self._sessions[session_id]
            state = engine.step(applied_push, applied_rotate, applied_support)
            info.total_steps += 1
            info.last_active = datetime.now()
            info.last_state = state
            info.last_applied_control = {
                "requested_push": float(delta_push),
                "requested_rotate": float(delta_rotate),
                "requested_support": float(microcatheter_advance),
                "applied_push": applied_push,
                "applied_rotate": applied_rotate,
                "applied_support": applied_support,
                "protection_reasons": protection_reasons,
            }
            return state

    def emergency_stop(self, session_id: str, reason: str = "operator") -> dict[str, Any]:
        """Latch control at zero and disengage any active ShapeIntent."""
        with self._lock:
            if session_id not in self._sessions:
                raise KeyError(f"Session not found: {session_id}")
            info = self._session_info[session_id]
            self._sessions[session_id].set_shape_intent(None, active=False)
            info.emergency_stop_latched = True
            info.emergency_stop_reason = reason
            info.emergency_stop_at = datetime.now()
            info.last_active = datetime.now()
            info.last_applied_control = {
                "requested_push": 0.0,
                "requested_rotate": 0.0,
                "requested_support": 0.0,
                "applied_push": 0.0,
                "applied_rotate": 0.0,
                "applied_support": 0.0,
                "protection_reasons": ["EMERGENCY_STOP_LATCHED"],
            }
            return self._control_state_locked(info)

    def resume(self, session_id: str) -> dict[str, Any]:
        """Release a latched stop without rearming automatic control."""
        with self._lock:
            if session_id not in self._sessions:
                raise KeyError(f"Session not found: {session_id}")
            info = self._session_info[session_id]
            info.emergency_stop_latched = False
            info.emergency_stop_reason = None
            info.emergency_stop_at = None
            info.last_active = datetime.now()
            info.last_applied_control = {
                "requested_push": 0.0,
                "requested_rotate": 0.0,
                "requested_support": 0.0,
                "applied_push": 0.0,
                "applied_rotate": 0.0,
                "applied_support": 0.0,
                "protection_reasons": [],
            }
            return self._control_state_locked(info)

    def is_emergency_stopped(self, session_id: str) -> bool:
        with self._lock:
            if session_id not in self._session_info:
                raise KeyError(f"Session not found: {session_id}")
            return self._session_info[session_id].emergency_stop_latched

    def get_control_state(self, session_id: str) -> dict[str, Any]:
        with self._lock:
            if session_id not in self._session_info:
                raise KeyError(f"Session not found: {session_id}")
            return self._control_state_locked(self._session_info[session_id])

    def update_control_config(
        self,
        session_id: str,
        *,
        jtip_assist_enabled: bool | None = None,
        torque_limit_enabled: bool | None = None,
        withdrawal_protection_enabled: bool | None = None,
        auto_stop_push_enabled: bool | None = None,
    ) -> dict[str, Any]:
        with self._lock:
            if session_id not in self._sessions:
                raise KeyError(f"Session not found: {session_id}")
            info = self._session_info[session_id]
            engine = self._sessions[session_id]
            if torque_limit_enabled is not None:
                info.torque_limit_enabled = bool(torque_limit_enabled)
            if withdrawal_protection_enabled is not None:
                info.withdrawal_protection_enabled = bool(
                    withdrawal_protection_enabled
                )
            if auto_stop_push_enabled is not None:
                info.auto_stop_push_enabled = bool(auto_stop_push_enabled)
            if jtip_assist_enabled is not None and info.jtip_assist_supported:
                effective = engine.set_engine_params({}) or {}
                current_angle = float(effective.get("jtip_deg", 0.0))
                if current_angle > 0.0:
                    info.jtip_angle_deg = current_angle
                target_angle = info.jtip_angle_deg if jtip_assist_enabled else 0.0
                applied = engine.set_engine_params({"jtip_deg": target_angle}) or {}
                info.jtip_assist_enabled = float(applied.get("jtip_deg", 0.0)) > 0.0
            info.last_active = datetime.now()
            return self._control_state_locked(info)

    @staticmethod
    def _control_state_locked(info: SessionInfo) -> dict[str, Any]:
        return {
            "emergency_stop_latched": info.emergency_stop_latched,
            "emergency_stop_reason": info.emergency_stop_reason,
            "emergency_stop_at": (
                info.emergency_stop_at.isoformat()
                if info.emergency_stop_at is not None
                else None
            ),
            "protections": {
                "jtip_assist_enabled": info.jtip_assist_enabled,
                "jtip_assist_supported": info.jtip_assist_supported,
                "torque_limit_enabled": info.torque_limit_enabled,
                "torque_command_limit": info.torque_command_limit,
                "withdrawal_protection_enabled": info.withdrawal_protection_enabled,
                "auto_stop_push_enabled": info.auto_stop_push_enabled,
            },
            "last_applied_control": dict(info.last_applied_control),
            "rejected_control_count": info.rejected_control_count,
        }

    def reset_session(self, session_id: str) -> NavigationState:
        """Reset a session's environment.

        Args:
            session_id: Session UUID

        Returns:
            NavigationState after reset
        """
        engine = self.get_session(session_id)
        state = engine.reset()

        with self._lock:
            info = self._session_info[session_id]
            info.episode_count += 1
            info.last_active = datetime.now()
            info.last_state = state

        return state

    def close_session(self, session_id: str) -> bool:
        """Close and cleanup a session.

        Args:
            session_id: Session UUID

        Returns:
            True if session was closed, False if not found
        """
        with self._lock:
            if session_id not in self._sessions:
                return False

            engine = self._sessions.pop(session_id)
            self._session_info.pop(session_id, None)

            engine.close()
            return True

    def get_session_info(self, session_id: str) -> SessionInfo:
        """Get session metadata.

        Args:
            session_id: Session UUID

        Returns:
            SessionInfo dataclass
        """
        with self._lock:
            if session_id not in self._session_info:
                raise KeyError(f"Session not found: {session_id}")
            return self._session_info[session_id]

    def list_sessions(self) -> list[SessionInfo]:
        """List all active sessions.

        Returns:
            List of SessionInfo for all sessions
        """
        with self._lock:
            return list(self._session_info.values())

    def _cleanup_expired_sessions(self) -> int:
        """Remove sessions that have exceeded timeout.

        Returns:
            Number of sessions cleaned up
        """
        now = datetime.now()
        expired = []

        for session_id, info in self._session_info.items():
            elapsed = (now - info.last_active).total_seconds()
            if elapsed > self._session_timeout:
                expired.append(session_id)

        for session_id in expired:
            engine = self._sessions.pop(session_id, None)
            self._session_info.pop(session_id, None)
            if engine:
                engine.close()

        return len(expired)

    def close_all(self) -> int:
        """Close all sessions.

        Returns:
            Number of sessions closed
        """
        with self._lock:
            count = len(self._sessions)
            for engine in self._sessions.values():
                engine.close()
            self._sessions.clear()
            self._session_info.clear()
            return count

    @property
    def active_session_count(self) -> int:
        """Number of active sessions."""
        with self._lock:
            return len(self._sessions)

    @property
    def max_sessions(self) -> int:
        """Maximum allowed sessions."""
        return self._max_sessions


# Global session manager instance
_session_manager: SessionManager | None = None
_manager_lock = threading.Lock()


def get_session_manager() -> SessionManager:
    """Get the global session manager instance.

    Returns:
        SessionManager singleton
    """
    global _session_manager
    with _manager_lock:
        if _session_manager is None:
            _session_manager = SessionManager()
        return _session_manager
