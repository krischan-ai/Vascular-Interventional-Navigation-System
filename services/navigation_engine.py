"""Navigation Engine: Bridge between FastAPI and CathSim MuJoCo environment.

This module provides a high-level interface for controlling the CathSim
guidewire simulation through the NavigationEngine class.
"""

from __future__ import annotations

import sys
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal, Sequence

import numpy as np

# Make the in-repo `cathsim` package importable even when the server runs in a
# Python environment where it was not installed editable (e.g. uvicorn launched
# outside the project venv). cathsim lives under <project_root>/src.
_SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if _SRC_DIR.is_dir() and str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))


@dataclass
class NavigationState:
    """Normalized state representation from CathSim environment.

    Coordinates and distances are in MuJoCo units (meters). Curvature is in
    inverse meters (m^-1). The quaternion uses [x, y, z, w] order to match the
    Godot/WebSocket protocol convention.
    """

    tip_position: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    tip_direction: list[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])
    tip_quaternion: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])
    velocity: float = 0.0
    contact_force: float = 0.0
    wall_distance: float = 0.0
    curvature: float = 0.0
    episode_length: int = 0
    target_position: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    path_progress: float = 0.0
    path_deviation: float = 0.0
    joint_positions: list[float] = field(default_factory=list)
    joint_velocities: list[float] = field(default_factory=list)
    safety_status: str = "STANDBY"
    risk_score: float = 0.0
    reward: float = 0.0
    done: bool = False

    def as_dict(self) -> dict[str, Any]:
        """Convert state to dictionary for JSON serialization."""
        return {
            "tip_position": self.tip_position,
            "tip_direction": self.tip_direction,
            "tip_quaternion": self.tip_quaternion,
            "velocity": self.velocity,
            "contact_force": self.contact_force,
            "wall_distance": self.wall_distance,
            "curvature": self.curvature,
            "episode_length": self.episode_length,
            "target_position": self.target_position,
            "path_progress": self.path_progress,
            "path_deviation": self.path_deviation,
            "joint_positions": self.joint_positions,
            "joint_velocities": self.joint_velocities,
            "safety_status": self.safety_status,
            "risk_score": self.risk_score,
            "reward": self.reward,
            "done": self.done,
        }


SafetyStatus = Literal["STANDBY", "SAFE_NAV", "DANGER_WARNING", "COLLISION_STOP"]


class NavigationEngine:
    """High-level interface for CathSim guidewire navigation.

    This class wraps the CathSim dm_control environment and provides:
    - Simplified step/reset interface
    - Normalized state extraction
    - Safety status monitoring

    Example:
        engine = NavigationEngine(phantom="low_tort", target="bca")
        state = engine.reset()
        state = engine.step(delta_push=0.5, delta_rotate=0.1)

    For VPP phantoms:
        engine = NavigationEngine(
            phantom="case_001_vpp",
            target="endpoints_1",
            assets_dir="/path/to/vpp_assets/case_001/mujoco"
        )
    """

    VALID_PHANTOMS = ("low_tort", "phantom2", "phantom3", "phantom4")
    VALID_TARGETS = ("bca", "lcca")

    # Distance reported when the guidewire is not in contact with any wall (m).
    MAX_WALL_DISTANCE = 0.05
    # Safety thresholds on wall distance, in MuJoCo meters (1.0mm / 0.5mm).
    WALL_DISTANCE_SAFE = 0.001
    WALL_DISTANCE_DANGER = 0.0005
    # Number of recent tip samples kept for curvature estimation.
    TIP_HISTORY_LEN = 5

    def __init__(
        self,
        phantom: str = "low_tort",
        target: str = "bca",
        use_pixels: bool = False,
        image_size: int = 80,
        assets_dir: str = None,
        planned_path: Sequence[Sequence[float]] | None = None,
        n_bodies: int = 80,
        n_substeps: int | None = None,
    ):
        """Initialize the navigation engine.

        Args:
            phantom: Phantom model name (low_tort, phantom2, phantom3, phantom4)
                     or VPP case name (e.g., case_001_vpp)
            target: Target site name (bca, lcca) or VPP endpoint (e.g., endpoints_1)
            use_pixels: Whether to include pixel observations
            image_size: Image size for pixel observations
            assets_dir: Optional path to phantom assets directory for VPP phantoms
            planned_path: Optional planned path as a list of [x, y, z] points in
                          MuJoCo meters. When provided, path_progress and
                          path_deviation are computed each step.
            n_bodies: Number of guidewire segments. Fewer segments greatly reduce
                      per-step cost (fewer contacts/DOFs) for interactive use.
            n_substeps: Physics substeps per control step. Fewer is faster; None
                        uses the model default (3).
        """
        self.phantom = phantom
        self.target = target
        self.use_pixels = use_pixels
        self.image_size = image_size
        self.assets_dir = assets_dir
        self.n_bodies = n_bodies
        self.n_substeps = n_substeps

        self._env = None
        self._time_step = None
        self._episode_length = 0
        self._previous_tip_pos = None
        self._initialized = False

        self._tip_history: deque[list[float]] = deque(maxlen=self.TIP_HISTORY_LEN)
        # Cached (geom_id, body_id) pairs for guidewire render data; built once
        # since the model structure is fixed after initialization.
        self._render_geom_ids: list[tuple[int, int]] | None = None

        from services.risk_assessor import RiskAssessor

        self._risk_assessor = RiskAssessor()

        # Planned path state (populated by set_planned_path / _setup_path).
        self._path_points: np.ndarray | None = None
        self._path_cumlen: np.ndarray | None = None
        self._path_total_len: float = 0.0
        self._path_kdtree = None
        if planned_path is not None:
            self.set_planned_path(planned_path)

    def set_planned_path(self, planned_path: Sequence[Sequence[float]] | None) -> None:
        """Set or clear the planned path used for progress/deviation tracking.

        Args:
            planned_path: List of [x, y, z] points in MuJoCo meters, or None to
                          disable path tracking.
        """
        if planned_path is None or len(planned_path) < 2:
            self._path_points = None
            self._path_cumlen = None
            self._path_total_len = 0.0
            self._path_kdtree = None
            return

        points = np.asarray(planned_path, dtype=np.float64)
        segment_len = np.linalg.norm(np.diff(points, axis=0), axis=1)
        cumlen = np.concatenate([[0.0], np.cumsum(segment_len)])

        self._path_points = points
        self._path_cumlen = cumlen
        self._path_total_len = float(cumlen[-1])

        try:
            from scipy.spatial import cKDTree

            self._path_kdtree = cKDTree(points)
        except Exception:
            self._path_kdtree = None

    def _ensure_initialized(self) -> None:
        """Lazy initialization of CathSim environment."""
        if self._initialized:
            return

        from cathsim.dm import make_dm_env

        self._env = make_dm_env(
            phantom=self.phantom,
            target=self.target,
            use_pixels=self.use_pixels,
            image_size=self.image_size,
            visualize_sites=False,
            visualize_target=False,
            sample_target=False,
            assets_dir=self.assets_dir,
            n_bodies=self.n_bodies,
            n_substeps=self.n_substeps,
        )
        self._initialized = True

    def reset(self) -> NavigationState:
        """Reset the environment and return initial state.

        Returns:
            NavigationState with initial positions and zeroed dynamics
        """
        self._ensure_initialized()

        self._time_step = self._env.reset()
        self._episode_length = 0
        self._previous_tip_pos = None
        self._tip_history.clear()

        return self._extract_state()

    def step(self, delta_push: float, delta_rotate: float) -> NavigationState:
        """Execute one simulation step.

        Args:
            delta_push: Push force coefficient [-1.0, 1.0], positive = forward
            delta_rotate: Rotation force coefficient [-1.0, 1.0], positive = clockwise

        Returns:
            NavigationState after the step
        """
        if not self._initialized or self._time_step is None:
            raise RuntimeError("Engine not initialized. Call reset() first.")

        delta_push = float(np.clip(delta_push, -1.0, 1.0))
        delta_rotate = float(np.clip(delta_rotate, -1.0, 1.0))

        action = np.array([delta_push, delta_rotate], dtype=np.float64)
        self._time_step = self._env.step(action)
        self._episode_length += 1

        return self._extract_state()

    def _extract_state(self) -> NavigationState:
        """Extract normalized state from current time_step."""
        obs = self._time_step.observation
        physics = self._env.physics
        task = self._env.task

        tip_pos = task.get_head_pos(physics).tolist()
        self._tip_history.append(tip_pos)

        tip_direction = self._compute_tip_direction(physics)
        tip_quaternion = self._compute_tip_quaternion(physics)

        velocity = self._compute_velocity(tip_pos)

        contact_force = float(task.get_total_force(physics))
        wall_distance = self._compute_wall_distance(physics)
        curvature = self._compute_curvature()

        target_pos = task.target_pos
        if isinstance(target_pos, np.ndarray):
            target_pos = target_pos.tolist()

        path_progress, path_deviation = self._compute_path_progress(tip_pos)

        joint_pos = obs.get("joint_pos", np.array([])).tolist()
        joint_vel = obs.get("joint_vel", np.array([])).tolist()

        reward = self._time_step.reward if self._time_step.reward is not None else 0.0
        done = self._time_step.last()

        safety_status = self._compute_safety_status(self._episode_length, wall_distance)

        state = NavigationState(
            tip_position=tip_pos,
            tip_direction=tip_direction,
            tip_quaternion=tip_quaternion,
            velocity=float(velocity),
            contact_force=contact_force,
            wall_distance=float(wall_distance),
            curvature=float(curvature),
            episode_length=self._episode_length,
            target_position=target_pos,
            path_progress=float(path_progress),
            path_deviation=float(path_deviation),
            joint_positions=joint_pos,
            joint_velocities=joint_vel,
            safety_status=safety_status,
            reward=float(reward),
            done=done,
        )
        state.risk_score = self._risk_assessor.assess(state)["risk_score"]
        return state

    def _compute_tip_quaternion(self, physics) -> list[float]:
        """Get the tip body orientation as a quaternion in [x, y, z, w] order.

        MuJoCo stores quaternions as [w, x, y, z]; we reorder to [x, y, z, w] to
        match the Godot/WebSocket protocol convention.
        """
        try:
            body_id = int(physics.model.geom_bodyid[-1])
            w, x, y, z = (float(v) for v in physics.data.xquat[body_id])
            return [x, y, z, w]
        except Exception:
            return [0.0, 0.0, 0.0, 1.0]

    def _compute_wall_distance(self, physics) -> float:
        """Estimate the minimum gap between the guidewire and the vessel wall.

        This is a contact-based proxy: when MuJoCo reports active contacts the
        gap distance (clamped at 0 for penetration) is used; otherwise a large
        sentinel (MAX_WALL_DISTANCE) is returned. Distances are in meters.
        """
        ncon = int(physics.data.ncon)
        if ncon == 0:
            return self.MAX_WALL_DISTANCE

        dists = np.asarray(physics.data.contact.dist[:ncon], dtype=np.float64)
        min_gap = float(np.clip(dists, 0.0, None).min())
        return min(min_gap, self.MAX_WALL_DISTANCE)

    def _compute_curvature(self) -> float:
        """Estimate local tip curvature (m^-1) via Menger curvature.

        Uses the last three tip positions; returns 0 when insufficient history
        or when the points are (near-)collinear or coincident.
        """
        if len(self._tip_history) < 3:
            return 0.0

        p1 = np.asarray(self._tip_history[-3], dtype=np.float64)
        p2 = np.asarray(self._tip_history[-2], dtype=np.float64)
        p3 = np.asarray(self._tip_history[-1], dtype=np.float64)

        a = np.linalg.norm(p1 - p2)
        b = np.linalg.norm(p2 - p3)
        c = np.linalg.norm(p1 - p3)
        if a < 1e-9 or b < 1e-9 or c < 1e-9:
            return 0.0

        area = 0.5 * np.linalg.norm(np.cross(p2 - p1, p3 - p1))
        if area < 1e-12:
            return 0.0

        return 4.0 * area / (a * b * c)

    def _compute_path_progress(self, tip_pos: list[float]) -> tuple[float, float]:
        """Compute progress along and deviation from the planned path.

        Returns:
            (path_progress, path_deviation) where progress is in [0, 1] and
            deviation is the distance to the nearest path vertex (meters).
            Returns (0.0, 0.0) when no planned path is set.
        """
        if self._path_points is None or self._path_total_len <= 0.0:
            return 0.0, 0.0

        tip = np.asarray(tip_pos, dtype=np.float64)
        if self._path_kdtree is not None:
            deviation, idx = self._path_kdtree.query(tip)
            idx = int(idx)
        else:
            diffs = self._path_points - tip
            sq = np.einsum("ij,ij->i", diffs, diffs)
            idx = int(np.argmin(sq))
            deviation = float(np.sqrt(sq[idx]))

        progress = float(self._path_cumlen[idx] / self._path_total_len)
        return progress, float(deviation)

    def _compute_safety_status(self, episode_length: int, wall_distance: float) -> SafetyStatus:
        """Derive the safety status from episode state and wall distance."""
        if episode_length == 0:
            return "STANDBY"
        if wall_distance >= self.WALL_DISTANCE_SAFE:
            return "SAFE_NAV"
        if wall_distance >= self.WALL_DISTANCE_DANGER:
            return "DANGER_WARNING"
        return "COLLISION_STOP"

    def _compute_tip_direction(self, physics) -> list[float]:
        """Compute tip direction from the last two geom positions."""
        geom_xpos = physics.data.geom_xpos
        if geom_xpos.shape[0] < 2:
            return [0.0, 0.0, 1.0]

        tip_pos = geom_xpos[-1]
        prev_pos = geom_xpos[-2]
        direction = tip_pos - prev_pos
        norm = np.linalg.norm(direction)
        if norm > 1e-8:
            direction = direction / norm
        else:
            direction = np.array([0.0, 0.0, 1.0])
        return direction.tolist()

    def _compute_velocity(self, tip_pos: list[float]) -> float:
        """Compute tip velocity from position change."""
        if self._previous_tip_pos is None:
            self._previous_tip_pos = tip_pos
            return 0.0

        delta = np.array(tip_pos) - np.array(self._previous_tip_pos)
        velocity = np.linalg.norm(delta)
        self._previous_tip_pos = tip_pos

        control_timestep = getattr(self._env.task, "control_timestep", 0.02)
        velocity_per_second = velocity / control_timestep if control_timestep > 0 else 0.0

        return velocity_per_second

    def get_render_bodies(self) -> list[dict[str, list[float]]]:
        """Return per-segment guidewire render data for tube rendering.

        Each entry is ``{"pos": [x, y, z], "quat": [x, y, z, w]}`` for one
        guidewire geom, ordered from base to tip. Quaternions are reordered from
        MuJoCo's [w, x, y, z] to the protocol's [x, y, z, w]. Returns an empty
        list when the environment is not initialized.
        """
        if not self._initialized or self._env is None:
            return []

        physics = self._env.physics
        model = physics.model
        data = physics.data

        if self._render_geom_ids is None:
            ids: list[tuple[int, int]] = []
            for geom_id in range(model.ngeom):
                body_id = int(model.geom_bodyid[geom_id])
                name = model.id2name(body_id, "body") or ""
                if "guidewire" in name:
                    ids.append((geom_id, body_id))
            self._render_geom_ids = ids

        bodies: list[dict[str, list[float]]] = []
        for geom_id, body_id in self._render_geom_ids:
            pos = [float(v) for v in data.geom_xpos[geom_id]]
            w, x, y, z = (float(v) for v in data.xquat[body_id])
            bodies.append({"pos": pos, "quat": [x, y, z, w]})
        return bodies

    def get_safety_status(self, state: NavigationState) -> SafetyStatus:
        """Return the safety status carried by the given state.

        The status is computed during state extraction from the episode length
        and wall distance (see ``_compute_safety_status``). This accessor is
        kept for backward compatibility with existing callers.

        Args:
            state: Current navigation state

        Returns:
            Safety status string
        """
        return state.safety_status  # type: ignore[return-value]

    def close(self) -> None:
        """Clean up resources."""
        if self._env is not None:
            del self._env
            self._env = None
        self._initialized = False
        self._time_step = None

    def __del__(self):
        self.close()

    @property
    def is_initialized(self) -> bool:
        """Check if engine is initialized."""
        return self._initialized

    @property
    def episode_length(self) -> int:
        """Current episode length."""
        return self._episode_length

    @property
    def planned_path(self) -> list[list[float]]:
        """The active planned path as a list of [x, y, z] points (meters)."""
        if self._path_points is None:
            return []
        return self._path_points.tolist()
