"""Navigation Engine: orchestration layer over a pluggable physics backend.

``NavigationEngine`` is the high-level interface FastAPI / the session manager
talk to. It owns the *engine-agnostic* concerns -- planned-path setup, entry-pose
resolution, progress / deviation, curvature, safety status and risk -- and
delegates the actual stepping to a single :class:`~services.physics.base.PhysicsEngine`
chosen once at construction (kinematic centerline-follow vs. MuJoCo physics).

The former ``if self._is_guided()`` fork that ran two engines inside this one
object is gone: a backend is selected by :func:`~services.physics.factory.make_engine`
and this class never branches on engine type in the hot path. Derived navigation
quantities stay here so they are computed once, independent of which backend (or
a future GPU solver) produced the raw pose.
"""

from __future__ import annotations

import json
import sys
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal, Sequence

import numpy as np

from services.physics import (
    MAX_WALL_DISTANCE,
    PhysicsEngine,
    PlannedPath,
    make_engine,
)
from services.physics.kinematic_engine import KinematicEngine

# Make the in-repo `cathsim` package importable even when the server runs in a
# Python environment where it was not installed editable (e.g. uvicorn launched
# outside the project venv). cathsim lives under <project_root>/src.
_SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if _SRC_DIR.is_dir() and str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))

_VPP_DATA_ROOT = Path(__file__).resolve().parents[1] / "data" / "vpp_assets"

# Built-in phantoms whose vessel is offset from the world origin and therefore
# need a fixed guidewire entry pose when no planned path or explicit entry is
# given. The entry POINT is read from the phantom's <site name="entry"> landmark
# (single source of truth in the XML); only the inward feed DIRECTION at that
# wall location -- which a point site cannot carry -- lives here.
_ENTRY_DIRECTIONS: dict[str, tuple[float, float, float]] = {
    "segment_part": (-0.334, 0.921, -0.200),
}


def resolve_vpp_assets_dir(phantom: str) -> str | None:
    """Resolve the MuJoCo assets directory for a VPP phantom from its name.

    VPP phantoms are named ``<case_id>_vpp`` (e.g. ``case_001_vpp``) and their
    generated MuJoCo assets live under ``data/vpp_assets/<case_id>/mujoco``.
    Returns the directory path as a string when it exists, otherwise None.
    Non-VPP phantoms (e.g. ``low_tort``) return None so the built-in phantom
    assets are used.
    """
    if not phantom.endswith("_vpp"):
        return None
    case_id = phantom[: -len("_vpp")]
    mujoco_dir = _VPP_DATA_ROOT / case_id / "mujoco"
    return str(mujoco_dir) if mujoco_dir.is_dir() else None


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
    """High-level orchestration interface for CathSim guidewire navigation.

    Wraps a single :class:`PhysicsEngine` backend (kinematic or MuJoCo) and
    provides:
    - Simplified step/reset interface
    - Normalized state extraction (progress / deviation / curvature / safety)
    - Risk scoring

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

    VALID_PHANTOMS = ("low_tort", "phantom2", "phantom3", "phantom4", "segment_part", "aorta_trunk")
    VALID_TARGETS = ("bca", "lcca", "root")

    # Distance reported when the guidewire is not in contact with any wall (m).
    MAX_WALL_DISTANCE = MAX_WALL_DISTANCE
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
        insertion_max: float = 0.2,
        stiffness_scale: float = 1.0,
        prethread: bool = False,
        entry_point: Sequence[float] | None = None,
        entry_direction: Sequence[float] | None = None,
        guided: bool = False,
        advance_per_step: float = 0.01,
        wire_length: float = 0.12,
        wall_lean: float = 0.0025,
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
            insertion_max: Upper bound (meters) of the physical guidewire slider
                        travel. Default 0.2 matches original CathSim; raise it
                        for offset long vessels (segment_part ~0.58m, VPP ~1.1m)
                        so the tip can be fed deep enough toward distal targets.
                        Only affects physical (non-guided) mode.
            entry_point: Optional [x, y, z] in MuJoCo meters where the guidewire
                         spawns. Required for VPP phantoms whose vessels are
                         offset from the origin. When None and planned_path is
                         set, it is derived from the path's first point.
            entry_direction: Optional [x, y, z] feed direction at the entry. When
                             None and planned_path is set, it is derived from the
                             first path segment.
            guided: Kinematic centerline-follow mode. When True together with a
                    planned_path, MuJoCo physics is bypassed: the guidewire is
                    driven along the planned centerline by an insertion-depth
                    parameter, so ``delta_push`` advances/retracts the tip along
                    the route and it reliably reaches the target. Required for
                    full-length VPP vessels (path ~1.1m) that the physical
                    guidewire (~0.08m, 0.2m insertion cap) cannot traverse.
            advance_per_step: Arc-length advanced per unit push in guided mode
                              (meters). delta_push=1.0 advances this much.
            wire_length: Legacy trailing-length hint for guided rendering; the
                         render now spans the full inserted length (entry->tip).
            wall_lean: Max offset (meters) toward the inner side of curves in
                       guided render, so the wire hugs the inner vessel wall at
                       bends like a tensioned wire. 0 disables (centerline).
        """
        self.phantom = phantom
        self.target = target
        self.use_pixels = use_pixels
        self.image_size = image_size
        # Auto-resolve the assets directory for VPP phantoms so callers only need
        # to pass the phantom name (e.g. "case_001_vpp"); an explicit assets_dir
        # still wins for custom layouts.
        self.assets_dir = assets_dir or resolve_vpp_assets_dir(phantom)
        self.n_bodies = n_bodies
        self.n_substeps = n_substeps
        self.insertion_max = float(insertion_max)
        self.stiffness_scale = float(stiffness_scale)
        self.prethread = bool(prethread)
        self._entry_point = (
            np.asarray(entry_point, dtype=np.float64) if entry_point is not None else None
        )
        self._entry_direction = (
            np.asarray(entry_direction, dtype=np.float64)
            if entry_direction is not None
            else None
        )

        # Guided (kinematic centerline-follow) configuration.
        self._guided = bool(guided)
        self._advance_per_step = float(advance_per_step)
        self._wire_length = float(wire_length)
        self._wall_lean = float(wall_lean)

        self._episode_length = 0
        self._previous_tip_pos = None
        self._tip_history: deque[list[float]] = deque(maxlen=self.TIP_HISTORY_LEN)

        from services.risk_assessor import RiskAssessor

        self._risk_assessor = RiskAssessor()

        # Planned path (arc-length geometry) shared with the backing engine.
        # Set _path / _engine to None first so set_planned_path's engine
        # propagation is a no-op during this initial setup.
        self._path: PlannedPath | None = None
        self._engine: PhysicsEngine | None = None

        if planned_path is not None:
            self.set_planned_path(planned_path)
        else:
            default_path = self._default_centerline_points()
            if default_path is not None:
                self.set_planned_path(default_path)

        # Resolve the entry pose (needs the path) then construct the one backend.
        entry_pt, entry_dir = self._resolve_entry()
        self._engine = make_engine(
            guided=self._guided,
            path=self._path,
            phantom=self.phantom,
            target=self.target,
            use_pixels=self.use_pixels,
            image_size=self.image_size,
            assets_dir=self.assets_dir,
            n_bodies=self.n_bodies,
            n_substeps=self.n_substeps,
            insertion_max=self.insertion_max,
            stiffness_scale=self.stiffness_scale,
            prethread=self.prethread,
            entry_point=entry_pt,
            entry_direction=entry_dir,
            advance_per_step=self._advance_per_step,
            wall_lean=self._wall_lean,
        )

    def _default_centerline_points(self) -> list[list[float]] | None:
        """Load a built-in phantom's shipped B-spline centerline, if present.

        Built-in phantoms (assets_dir is None) may carry a precomputed centerline
        at ``meshes/<phantom>/centerline.json`` (entry -> target_root, smoothed
        with the same scipy B-spline as PathPlanner). VPP phantoms instead plan
        on their graph.json at runtime, so they are skipped here.
        """
        if self.assets_dir is not None:
            return None
        centerline = (
            _SRC_DIR
            / "cathsim/dm/components/phantom_assets/meshes"
            / self.phantom
            / "centerline.json"
        )
        if not centerline.is_file():
            return None
        try:
            data = json.loads(centerline.read_text(encoding="utf-8"))
            waypoints = data.get("waypoints")
        except Exception:
            return None
        if isinstance(waypoints, list) and len(waypoints) >= 2:
            return waypoints
        return None

    def _load_phantom_graph(self) -> dict | None:
        """Load a built-in phantom's centerline graph for A* planning.

        Built-in phantoms may ship a graph.json (adjacency map format compatible
        with PathPlanner). Returns None if graph not found or load fails.
        """
        if self.assets_dir is not None:
            return None
        graph_path = (
            _SRC_DIR
            / "cathsim/dm/components/phantom_assets/meshes"
            / self.phantom
            / "graph.json"
        )
        if not graph_path.is_file():
            return None
        try:
            return json.loads(graph_path.read_text(encoding="utf-8"))
        except Exception:
            return None

    def set_planned_path(self, planned_path: Sequence[Sequence[float]] | None) -> None:
        """Set or clear the planned path used for progress/deviation tracking.

        When the engine already exists (e.g. an interactive re-plan via
        :meth:`plan_to_target`), the new geometry is propagated to it so the next
        reset/step uses it. Note: switching a guided session from "no path" to a
        path after construction does not retroactively turn it kinematic -- the
        backend type is fixed at construction; pass the path up front for guided.

        Args:
            planned_path: List of [x, y, z] points in MuJoCo meters, or None to
                          disable path tracking.
        """
        if planned_path is None or len(planned_path) < 2:
            self._path = None
        else:
            self._path = PlannedPath(planned_path)

        if getattr(self, "_engine", None) is not None:
            # Both backends keep a `_path` reference; keep it in sync for re-plan.
            self._engine._path = self._path

    def _resolve_entry(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        """Resolve the guidewire entry pose, deriving it from the planned path.

        An explicit entry_point/entry_direction always wins. Otherwise, when a
        planned path is set, the entry is the first path point and the direction
        is the first non-degenerate path segment.
        """
        entry_point = self._entry_point
        entry_direction = self._entry_direction

        if entry_point is None and self._path is not None:
            entry_point = self._path.points[0]

        if entry_direction is None and self._path is not None:
            origin = self._path.points[0]
            for point in self._path.points[1:]:
                seg = point - origin
                if np.linalg.norm(seg) > 1e-9:
                    entry_direction = seg
                    break

        # Fall back to the phantom's reserved <site name="entry"> landmark for
        # built-in phantoms whose vessel is offset from the origin (segment_part).
        if entry_point is None:
            site_point = self._entry_site_point()
            if site_point is not None:
                entry_point = site_point
                if entry_direction is None:
                    direction = _ENTRY_DIRECTIONS.get(self.phantom)
                    if direction is not None:
                        entry_direction = np.asarray(direction, dtype=np.float64)

        return entry_point, entry_direction

    def _entry_site_point(self) -> np.ndarray | None:
        """Read the ``entry`` landmark position from the phantom XML, if present."""
        try:
            from cathsim.dm.components.phantom import Phantom

            phantom = Phantom(self.phantom + ".xml", assets_dir=self.assets_dir)
            entry = phantom.sites.get("entry")
        except Exception:
            return None
        return np.asarray(entry, dtype=np.float64) if entry is not None else None

    def plan_to_target(
        self, start_pos: Sequence[float], end_pos: Sequence[float]
    ) -> tuple[list[list[float]], float] | None:
        """Plan path using the phantom's centerline graph via A* algorithm.

        Only works for phantoms that ship a graph.json (e.g. segment_part).

        Args:
            start_pos: [x, y, z] in MuJoCo meters
            end_pos: [x, y, z] in MuJoCo meters

        Returns:
            (smooth_waypoints, total_length_m) or None if no graph available
        """
        graph = self._load_phantom_graph()
        if graph is None:
            return None

        try:
            from services.path_planner import PathPlanner

            planner = PathPlanner()
            planner._graph = graph
            planner._nodes = [
                tuple(map(float, key.split(","))) for key in graph.keys()
            ]

            result = planner.plan(
                start=start_pos,
                end=end_pos,
                algorithm="astar",
                smooth=True,
                smooth_factor=0.25e-6,
            )

            waypoints = result.smooth_waypoints or result.waypoints
            if waypoints:
                self.set_planned_path(waypoints)
                return (waypoints, result.smooth_length_mm / 1000.0)
        except Exception:
            return None

        return None

    def reset(self) -> NavigationState:
        """Reset the backing engine and return the initial state."""
        raw = self._engine.reset()
        self._episode_length = 0
        self._previous_tip_pos = None
        self._tip_history.clear()
        return self._assemble_state(raw)

    def step(self, delta_push: float, delta_rotate: float) -> NavigationState:
        """Execute one simulation step.

        Args:
            delta_push: Push force coefficient [-1.0, 1.0], positive = forward
            delta_rotate: Rotation force coefficient [-1.0, 1.0], positive = clockwise

        Returns:
            NavigationState after the step
        """
        delta_push = float(np.clip(delta_push, -1.0, 1.0))
        delta_rotate = float(np.clip(delta_rotate, -1.0, 1.0))

        raw = self._engine.step(delta_push, delta_rotate)
        self._episode_length += 1
        return self._assemble_state(raw)

    def _is_guided(self) -> bool:
        """Whether the active backend is the kinematic centerline-follow engine.

        Kept as a thin accessor (the runtime fork is gone -- the backend type is
        fixed at construction). Returns False before the engine is constructed.
        """
        return isinstance(self._engine, KinematicEngine)

    def _assemble_state(self, raw) -> NavigationState:
        """Assemble a NavigationState from an engine RawPose plus derived terms.

        Derived (engine-agnostic) quantities -- velocity, curvature, path
        progress/deviation, safety status, risk -- are computed here so they are
        identical regardless of which backend produced ``raw``. When the engine
        reports an exact inserted arc length (``raw.arclen``, kinematic mode),
        progress is derived continuously from it and deviation is zero; otherwise
        progress/deviation come from projecting the tip onto the planned path.
        """
        tip_pos = raw.tip_position
        self._tip_history.append(tip_pos)

        velocity = self._compute_velocity(tip_pos)
        curvature = self._compute_curvature()

        if raw.arclen is not None and self._path is not None and self._path.total_len > 0.0:
            path_progress = float(raw.arclen / self._path.total_len)
            path_deviation = 0.0
        else:
            path_progress, path_deviation = self._compute_path_progress(tip_pos)

        safety_status = self._compute_safety_status(self._episode_length, raw.wall_distance)

        state = NavigationState(
            tip_position=tip_pos,
            tip_direction=raw.tip_direction,
            tip_quaternion=raw.tip_quaternion,
            velocity=float(velocity),
            contact_force=float(raw.contact_force),
            wall_distance=float(raw.wall_distance),
            curvature=float(curvature),
            episode_length=self._episode_length,
            target_position=raw.target_position,
            path_progress=float(path_progress),
            path_deviation=float(path_deviation),
            joint_positions=raw.joint_positions,
            joint_velocities=raw.joint_velocities,
            safety_status=safety_status,
            reward=float(raw.reward),
            done=raw.done,
        )
        state.risk_score = self._risk_assessor.assess(state)["risk_score"]
        return state

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
        if self._path is None or self._path.total_len <= 0.0:
            return 0.0, 0.0
        return self._path.progress_deviation(tip_pos)

    def _compute_safety_status(self, episode_length: int, wall_distance: float) -> SafetyStatus:
        """Derive the safety status from episode state and wall distance."""
        if episode_length == 0:
            return "STANDBY"
        if wall_distance >= self.WALL_DISTANCE_SAFE:
            return "SAFE_NAV"
        if wall_distance >= self.WALL_DISTANCE_DANGER:
            return "DANGER_WARNING"
        return "COLLISION_STOP"

    def _compute_velocity(self, tip_pos: list[float]) -> float:
        """Compute tip velocity (m/s) from position change over a control step."""
        if self._previous_tip_pos is None:
            self._previous_tip_pos = tip_pos
            return 0.0

        delta = np.array(tip_pos) - np.array(self._previous_tip_pos)
        velocity = np.linalg.norm(delta)
        self._previous_tip_pos = tip_pos

        control_timestep = self._engine.control_timestep if self._engine is not None else 0.033
        return velocity / control_timestep if control_timestep > 0 else 0.0

    def _inner_wall_offset(self, s: float) -> np.ndarray:
        """Inner-wall lean offset at arc length ``s`` (kinematic backend only)."""
        return self._engine._inner_wall_offset(s)

    def get_render_bodies(self) -> list[dict[str, list[float]]]:
        """Return per-segment guidewire render data for tube rendering.

        Each entry is ``{"pos": [x, y, z], "quat": [x, y, z, w]}`` for one
        guidewire segment, ordered from base to tip. Quaternions are in the
        protocol's [x, y, z, w] order. Returns an empty list when the backend
        has no geometry yet.
        """
        return self._engine.render_bodies()

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
        """Clean up resources held by the backing engine."""
        if getattr(self, "_engine", None) is not None:
            self._engine.close()

    def __del__(self):
        self.close()

    @property
    def is_initialized(self) -> bool:
        """Whether the backend has built its (lazy) simulation resources."""
        return bool(getattr(self._engine, "is_initialized", False))

    @property
    def episode_length(self) -> int:
        """Current episode length."""
        return self._episode_length

    @property
    def planned_path(self) -> list[list[float]]:
        """The active planned path as a list of [x, y, z] points (meters)."""
        if self._path is None:
            return []
        return self._path.as_list()

    @property
    def entry_pose(self) -> dict[str, list[float]]:
        """Resolved guidewire entry (vascular access) pose for client highlighting.

        Returns ``{"position": [x, y, z], "direction": [x, y, z]}`` in MuJoCo
        meters, where ``direction`` is the unit feed direction into the vessel
        (the introduction direction). This is the exact pose the guidewire is
        spawned at on reset (see ``_resolve_entry``), so a marker drawn here sits
        at the real entry. Both lists are empty when no entry is configured (e.g.
        plain low_tort sessions that spawn near the world origin).
        """
        point, direction = self._resolve_entry()
        pose: dict[str, list[float]] = {"position": [], "direction": []}
        if point is not None:
            pose["position"] = [float(v) for v in point]
        if direction is not None:
            norm = float(np.linalg.norm(direction))
            if norm > 1e-9:
                pose["direction"] = [float(v / norm) for v in direction]
        return pose
