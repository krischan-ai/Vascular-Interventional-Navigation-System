"""Navigation Engine: Bridge between FastAPI and CathSim MuJoCo environment.

This module provides a high-level interface for controlling the CathSim
guidewire simulation through the NavigationEngine class.
"""

from __future__ import annotations

import json
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


def _rot_x(a: float) -> np.ndarray:
    """Rotation matrix about the x axis by angle ``a`` (radians)."""
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rot_y(b: float) -> np.ndarray:
    """Rotation matrix about the y axis by angle ``b`` (radians)."""
    c, s = np.cos(b), np.sin(b)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


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

    VALID_PHANTOMS = ("low_tort", "phantom2", "phantom3", "phantom4", "segment_part")
    VALID_TARGETS = ("bca", "lcca", "root")

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

        # Guided (kinematic centerline-follow) mode state.
        self._guided = bool(guided)
        self._advance_per_step = float(advance_per_step)
        self._wire_length = float(wire_length)
        self._wall_lean = float(wall_lean)
        self._wall_lean_gain = 1.5
        self._s = 0.0  # current insertion depth as arc length along the path (m)

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
        else:
            default_path = self._default_centerline_points()
            if default_path is not None:
                self.set_planned_path(default_path)

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

    def _resolve_entry(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        """Resolve the guidewire entry pose, deriving it from the planned path.

        An explicit entry_point/entry_direction always wins. Otherwise, when a
        planned path is set, the entry is the first path point and the direction
        is the first non-degenerate path segment.
        """
        entry_point = self._entry_point
        entry_direction = self._entry_direction

        if entry_point is None and self._path_points is not None:
            entry_point = self._path_points[0]

        if entry_direction is None and self._path_points is not None:
            origin = self._path_points[0]
            for point in self._path_points[1:]:
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
        except Exception as e:
            return None

        return None

    def _ensure_initialized(self) -> None:
        """Lazy initialization of CathSim environment."""
        if self._initialized:
            return

        from cathsim.dm import make_dm_env

        entry_point, entry_direction = self._resolve_entry()

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
            insertion_max=self.insertion_max,
            stiffness_scale=self.stiffness_scale,
            entry_point=entry_point,
            entry_direction=entry_direction,
        )
        self._initialized = True

    def reset(self) -> NavigationState:
        """Reset the environment and return initial state.

        Returns:
            NavigationState with initial positions and zeroed dynamics
        """
        if self._is_guided():
            self._s = 0.0
            self._episode_length = 0
            self._previous_tip_pos = None
            self._tip_history.clear()
            return self._guided_state()

        self._ensure_initialized()

        self._time_step = self._env.reset()
        if self.prethread and self._path_points is not None:
            self._thread_physics_along_path()
        self._episode_length = 0
        self._previous_tip_pos = None
        self._tip_history.clear()

        return self._extract_state()

    def _thread_physics_along_path(self) -> None:
        """Bend the physical guidewire onto the planned centerline at spawn.

        A straight wire pushed by the straight base slider jams against the
        vessel wall as soon as the lumen curves. Instead we set the guidewire's
        steering-hinge angles so the body chain lies along the planned centerline
        from the entry inward, so the wire spawns *inside* the lumen with a
        physically plausible shape. Physics then holds it there via wall contacts
        (the wire presses on the bends, as a real guidewire does).

        The chain is a sequence of 2-DOF universal joints (J0 about local x, J1
        about local y); each body's segment direction is its local +z. Marching
        base->tip, for body i we solve the two hinge angles so its +z matches the
        centerline tangent T at that arc length::

            t_local = parent_rot^T @ T
            J1 = asin(t_local.x);  J0 = atan2(-t_local.y, t_local.z)
            child_rot = parent_rot @ Rx(J0) @ Ry(J1)
        """
        physics = self._env.physics
        model = physics.model

        chain = self._guidewire_chain_bodies(model)
        if not chain:
            return

        # Base segment orientation is fixed by the spawn pose; start the march
        # from the first base body and walk the centerline from the entry.
        base_id = chain[0][0]
        parent_rot = np.asarray(physics.data.xmat[base_id], dtype=np.float64).reshape(3, 3)
        qpos = physics.data.qpos

        # The proximal base body spawns ~one wire-length back from the entry along
        # the straight feed axis, so anchoring the centerline shape at it would
        # translate the whole wire out of the lumen. Advance the slider so the
        # base sits at the entry; then the bends below place the wire inside the
        # vessel from the entry inward.
        feed_axis = parent_rot @ np.array([0.0, 0.0, 1.0])
        entry = self._path_points[0]
        base_pos = np.asarray(physics.data.xpos[base_id], dtype=np.float64)
        slide = float(np.dot(entry - base_pos, feed_axis))
        slide = float(np.clip(slide, 0.0, self.insertion_max))
        slider_adr = self._slider_qposadr(model)
        if slider_adr is not None:
            qpos[slider_adr] = slide

        s = 0.0
        for _body_id, seg_len, j0_adr, j1_adr in chain[1:]:
            s = min(s + seg_len, self._path_total_len)
            tangent = self._tangent_at_arclen(s)
            t_local = parent_rot.T @ tangent
            tx = float(np.clip(t_local[0], -1.0, 1.0))
            j1 = float(np.arcsin(tx))
            cos_j1 = float(np.sqrt(max(1.0 - tx * tx, 1e-12)))
            j0 = float(np.arctan2(-t_local[1] / cos_j1, t_local[2] / cos_j1))
            if j0_adr is not None and j1_adr is not None:
                qpos[j0_adr] = j0
                qpos[j1_adr] = j1
            parent_rot = parent_rot @ _rot_x(j0) @ _rot_y(j1)

        physics.forward()

    @staticmethod
    def _slider_qposadr(model) -> int | None:
        """qpos address of the guidewire insertion (slider) joint, if present."""
        for j in range(model.njnt):
            if (model.id2name(j, "joint") or "").endswith("slider"):
                return int(model.jnt_qposadr[j])
        return None

    @staticmethod
    def _guidewire_chain_bodies(model) -> list[tuple[int, float, int | None, int | None]]:
        """Ordered guidewire/tip chain as (body_id, seg_len, j0_qposadr, j1_qposadr).

        The first entry is the base body (its orientation is fixed by the spawn
        pose, so its joint addresses are None). Subsequent entries carry the two
        steering-hinge qpos addresses used to bend the chain onto the centerline.
        """
        chain: list[tuple[int, float, int | None, int | None]] = []
        for body_id in range(model.nbody):
            name = model.id2name(body_id, "body") or ""
            is_gw = "guidewire_body_" in name
            is_tip = "tip_body_" in name
            if not (is_gw or is_tip):
                continue
            seg_len = float(model.body_pos[body_id][2])
            joints = [j for j in range(model.njnt) if int(model.jnt_bodyid[j]) == body_id]
            hinge_adrs = [int(model.jnt_qposadr[j]) for j in joints if int(model.jnt_type[j]) == 3]
            j0 = hinge_adrs[0] if len(hinge_adrs) >= 1 else None
            j1 = hinge_adrs[1] if len(hinge_adrs) >= 2 else None
            # Base body_0 carries the slider/rotator (its hinge is the rotator,
            # not a steering pair) -- mark it as the fixed root of the march.
            if name.endswith("guidewire_body_0"):
                chain.append((body_id, seg_len, None, None))
            else:
                chain.append((body_id, seg_len, j0, j1))
        return chain

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

        if self._is_guided():
            self._s = float(
                np.clip(
                    self._s + delta_push * self._advance_per_step,
                    0.0,
                    self._path_total_len,
                )
            )
            self._episode_length += 1
            return self._guided_state()

        if not self._initialized or self._time_step is None:
            raise RuntimeError("Engine not initialized. Call reset() first.")

        action = np.array([delta_push, delta_rotate], dtype=np.float64)
        self._time_step = self._env.step(action)
        self._episode_length += 1

        return self._extract_state()

    def _is_guided(self) -> bool:
        """Whether kinematic centerline-follow mode is active.

        Requires both the guided flag and a valid planned path; otherwise the
        engine falls back to the physical MuJoCo simulation.
        """
        return self._guided and self._path_points is not None and self._path_total_len > 0.0

    def _point_at_arclen(self, s: float) -> np.ndarray:
        """Interpolate a point on the planned path at arc length ``s`` (meters)."""
        cum = self._path_cumlen
        pts = self._path_points
        s = float(np.clip(s, 0.0, self._path_total_len))
        idx = int(np.searchsorted(cum, s))
        if idx <= 0:
            return pts[0].copy()
        if idx >= len(pts):
            return pts[-1].copy()
        s0, s1 = cum[idx - 1], cum[idx]
        t = 0.0 if s1 <= s0 else (s - s0) / (s1 - s0)
        return pts[idx - 1] + t * (pts[idx] - pts[idx - 1])

    def _tangent_at_arclen(self, s: float) -> np.ndarray:
        """Unit tangent of the planned path at arc length ``s`` (meters)."""
        eps = max(self._path_total_len * 1e-3, 1e-4)
        p0 = self._point_at_arclen(s - eps)
        p1 = self._point_at_arclen(s + eps)
        d = p1 - p0
        n = float(np.linalg.norm(d))
        return d / n if n > 1e-9 else np.array([0.0, 0.0, 1.0])

    @staticmethod
    def _quat_from_direction(direction: np.ndarray) -> list[float]:
        """Quaternion [x, y, z, w] rotating +z onto ``direction`` (protocol order)."""
        a = np.array([0.0, 0.0, 1.0])
        b = np.asarray(direction, dtype=np.float64)
        nb = float(np.linalg.norm(b))
        if nb < 1e-9:
            return [0.0, 0.0, 0.0, 1.0]
        b = b / nb
        d = float(np.dot(a, b))
        if d > 1.0 - 1e-9:
            return [0.0, 0.0, 0.0, 1.0]
        if d < -1.0 + 1e-9:
            return [1.0, 0.0, 0.0, 0.0]  # 180 deg about x
        axis = np.cross(a, b)
        w = 1.0 + d
        q = np.array([axis[0], axis[1], axis[2], w])
        q = q / np.linalg.norm(q)
        return [float(v) for v in q]

    def _guided_state(self) -> NavigationState:
        """Synthesize a NavigationState from the centerline-follow parameter.

        The tip rides the planned path at arc length ``self._s``; progress is the
        normalized arc length and deviation is zero by construction. No physical
        contact is modeled (wall_distance is the free-space sentinel), so the
        guidewire reliably traverses the full route to the target.
        """
        tip_pos_arr = self._point_at_arclen(self._s)
        tip_pos = [float(v) for v in tip_pos_arr]
        self._tip_history.append(tip_pos)

        tip_dir = self._tangent_at_arclen(self._s)
        tip_direction = [float(v) for v in tip_dir]
        tip_quaternion = self._quat_from_direction(tip_dir)

        velocity = self._compute_velocity(tip_pos)
        curvature = self._compute_curvature()

        target_pos = [float(v) for v in self._path_points[-1]]
        progress = float(self._s / self._path_total_len) if self._path_total_len > 0 else 0.0

        safety_status = "STANDBY" if self._episode_length == 0 else "SAFE_NAV"

        state = NavigationState(
            tip_position=tip_pos,
            tip_direction=tip_direction,
            tip_quaternion=tip_quaternion,
            velocity=float(velocity),
            contact_force=0.0,
            wall_distance=self.MAX_WALL_DISTANCE,
            curvature=float(curvature),
            episode_length=self._episode_length,
            target_position=target_pos,
            path_progress=progress,
            path_deviation=0.0,
            joint_positions=[],
            joint_velocities=[],
            safety_status=safety_status,
            reward=0.0,
            done=progress >= 0.999,
        )
        state.risk_score = self._risk_assessor.assess(state)["risk_score"]
        return state

    def _guided_render_bodies(self) -> list[dict[str, list[float]]]:
        """Sample the full inserted guidewire (entry -> tip) along the centerline.

        Renders the entire inserted length so the wire visibly curves through
        every vessel bend (not just a short tip stub), and leans each point
        toward the inner side of curves (like a tensioned wire pressing on the
        inner wall) so the shape reads as a real bending guidewire.
        """
        s_tip = self._s
        if s_tip <= 1e-6:
            pos = self._point_at_arclen(0.0)
            quat = self._quat_from_direction(self._tangent_at_arclen(0.0))
            return [{"pos": [float(v) for v in pos], "quat": quat}]

        spacing = 0.004  # ~4mm between render segments
        n = int(np.clip(int(s_tip / spacing) + 1, 2, 256))
        bodies: list[dict[str, list[float]]] = []
        for i in range(n + 1):
            s = s_tip * i / n
            pos = self._point_at_arclen(s)
            if self._wall_lean > 0.0:
                pos = pos + self._inner_wall_offset(s)
            quat = self._quat_from_direction(self._tangent_at_arclen(s))
            bodies.append({"pos": [float(v) for v in pos], "quat": quat})
        return bodies

    def _inner_wall_offset(self, s: float) -> np.ndarray:
        """Offset toward the inner (concave) side of the local curve.

        Zero on straight sections; grows with curvature, capped at wall_lean.
        """
        delta = 0.01
        p_prev = self._point_at_arclen(s - delta)
        p = self._point_at_arclen(s)
        p_next = self._point_at_arclen(s + delta)
        inward = 0.5 * (p_prev + p_next) - p  # toward the center of curvature
        # Keep only the lateral component; a one-sided difference at the path
        # ends (where p_prev/p_next clamp) is purely tangential and must not
        # shift the wire along its own direction.
        tangent = self._tangent_at_arclen(s)
        inward = inward - tangent * float(np.dot(inward, tangent))
        n = float(np.linalg.norm(inward))
        if n < 1e-9:
            return np.zeros(3)
        mag = min(self._wall_lean, n * self._wall_lean_gain)
        return (inward / n) * mag

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

        # Guided mode has no MuJoCo env; assume a nominal ~30Hz control step.
        if self._env is not None:
            control_timestep = getattr(self._env.task, "control_timestep", 0.02)
        else:
            control_timestep = 0.033
        velocity_per_second = velocity / control_timestep if control_timestep > 0 else 0.0

        return velocity_per_second

    def get_render_bodies(self) -> list[dict[str, list[float]]]:
        """Return per-segment guidewire render data for tube rendering.

        Each entry is ``{"pos": [x, y, z], "quat": [x, y, z, w]}`` for one
        guidewire geom, ordered from base to tip. Quaternions are reordered from
        MuJoCo's [w, x, y, z] to the protocol's [x, y, z, w]. Returns an empty
        list when the environment is not initialized.
        """
        if self._is_guided():
            return self._guided_render_bodies()

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
