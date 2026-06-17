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
        n_bodies: int = 80,
        n_substeps: int | None = None,
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
                n_bodies=n_bodies,
                n_substeps=n_substeps,
            )

            initial_state = engine.reset()

            self._sessions[session_id] = engine
            self._session_info[session_id] = SessionInfo(
                session_id=session_id,
                phantom=phantom,
                target=target,
                created_at=datetime.now(),
                last_active=datetime.now(),
                episode_count=1,
                total_steps=0,
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
    ) -> NavigationState:
        """Execute a step in the specified session.

        Args:
            session_id: Session UUID
            delta_push: Push force coefficient [-1.0, 1.0]
            delta_rotate: Rotation force coefficient [-1.0, 1.0]

        Returns:
            NavigationState after the step
        """
        engine = self.get_session(session_id)
        state = engine.step(delta_push, delta_rotate)

        with self._lock:
            info = self._session_info[session_id]
            info.total_steps += 1
            info.last_active = datetime.now()

        return state

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
