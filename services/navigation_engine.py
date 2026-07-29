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
from services.vpp_assets import resolve_vpp_mujoco_dir

# Make the in-repo `cathsim` package importable even when the server runs in a
# Python environment where it was not installed editable (e.g. uvicorn launched
# outside the project venv). cathsim lives under <project_root>/src.
_SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if _SRC_DIR.is_dir() and str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))

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
    return resolve_vpp_mujoco_dir(phantom)


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
    contact_count: int = 0
    tip_contact_count: int = 0
    rod_contact_force: float = 0.0
    rod_contact_count: int = 0
    wall_contact_count: int = 0
    max_penetration: float = 0.0
    wall_penetration: float = 0.0
    contact_impulse: float = 0.0
    wall_distance: float = 0.0
    curvature: float = 0.0
    episode_length: int = 0
    target_position: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    path_progress: float = 0.0
    path_deviation: float = 0.0
    remaining_distance: float = 0.0
    path_total_distance: float | None = None
    path_travelled_distance: float | None = None
    vessel_radius: float | None = None
    eta_seconds: float | None = None
    latency_ms: float | None = None
    fidelity_mode: str = "physics"
    risk_regions: list[dict[str, Any]] = field(default_factory=list)
    joint_positions: list[float] = field(default_factory=list)
    joint_velocities: list[float] = field(default_factory=list)
    safety_status: str = "STANDBY"
    risk_score: float = 0.0
    risk_assessment: dict[str, Any] = field(default_factory=dict)
    flow_guidance: dict[str, Any] = field(default_factory=dict)
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
            "contact_count": self.contact_count,
            "tip_contact_count": self.tip_contact_count,
            "rod_contact_force": self.rod_contact_force,
            "rod_contact_count": self.rod_contact_count,
            "wall_contact_count": self.wall_contact_count,
            "max_penetration": self.max_penetration,
            "wall_penetration": self.wall_penetration,
            "contact_impulse": self.contact_impulse,
            "wall_distance": self.wall_distance,
            "curvature": self.curvature,
            "episode_length": self.episode_length,
            "target_position": self.target_position,
            "path_progress": self.path_progress,
            "path_deviation": self.path_deviation,
            "remaining_distance": self.remaining_distance,
            "path_total_distance": self.path_total_distance,
            "path_travelled_distance": self.path_travelled_distance,
            "vessel_radius": self.vessel_radius,
            "eta_seconds": self.eta_seconds,
            "latency_ms": self.latency_ms,
            "fidelity_mode": self.fidelity_mode,
            "risk_regions": self.risk_regions,
            "joint_positions": self.joint_positions,
            "joint_velocities": self.joint_velocities,
            "safety_status": self.safety_status,
            "risk_score": self.risk_score,
            "risk_assessment": self.risk_assessment,
            "flow_guidance": self.flow_guidance,
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

    VALID_PHANTOMS = ("low_tort", "phantom2", "phantom3", "phantom4", "segment_part", "aorta_trunk", "aorta_tree")
    VALID_TARGETS = ("bca", "lcca", "root")

    # Distance reported when the guidewire is not in contact with any wall (m).
    MAX_WALL_DISTANCE = MAX_WALL_DISTANCE
    # Safety thresholds on wall distance, in MuJoCo meters (1.0mm / 0.5mm).
    # These assume a *centered* wire (guided/kinematic mode): clearance to the
    # wall is the risk. In force-drive physics the wire legitimately hugs the
    # wall, so proximity alone is not danger -- see BREACH_* below.
    WALL_DISTANCE_SAFE = 0.001
    WALL_DISTANCE_DANGER = 0.0005
    # Force-drive safety thresholds on independent wall penetration (meters).
    # Hugging the wall with non-zero physical force can still be normal;
    # only real breach counts. Sub-BREACH_WARN penetration is numerical noise.
    BREACH_WARN = 0.00005   # 0.05 mm: past here, warn
    BREACH_STOP = 0.0003    # 0.30 mm: sustained breach, collision stop
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
        planned_radii: Sequence[float] | None = None,
        n_bodies: int = 80,
        n_substeps: int | None = None,
        insertion_max: float = 0.2,
        stiffness_scale: float = 1.0,
        prethread: bool = False,
        entry_point: Sequence[float] | None = None,
        entry_direction: Sequence[float] | None = None,
        guided: bool = False,
        physics_engine: str | None = None,
        advance_per_step: float = 0.01,
        wire_length: float = 0.12,
        wall_lean: float = 0.0025,
        route_target: str | None = None,
        newton_params: dict[str, Any] | None = None,
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
            planned_radii: Optional lumen radii in meters, one per planned-path
                          point. Consumed by NewtonEngine to build the same
                          variable-radius wall used by aorta_tree routes.
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
            physics_engine: Optional backend override. ``auto``/None preserves
                    the legacy selection, ``guided`` forces kinematic
                    centerline-follow, ``mujoco`` forces MuJoCo physics, and
                    ``newton_demo`` selects the experimental VPP Newton wall.
            advance_per_step: Arc-length advanced per unit push in guided mode
                              (meters). delta_push=1.0 advances this much.
            wire_length: Legacy trailing-length hint for guided rendering; the
                         render now spans the full inserted length (entry->tip).
            wall_lean: Max offset (meters) toward the inner side of curves in
                       guided render, so the wire hugs the inner vessel wall at
                       bends like a tensioned wire. 0 disables (centerline).
            route_target: Optional branch endpoint id for multi-route phantoms
                          that ship routes.json (e.g. ``aorta_tree``).
            newton_params: Optional Newton backend calibration values, such as
                          ``rod_length``, ``free_len``, ``max_slack`` and
                          ``insertion_margin``. These replace ad-hoc process
                          environment variables for training runs.
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
        self._physics_engine = physics_engine
        self._advance_per_step = float(advance_per_step)
        self._wire_length = float(wire_length)
        self._wall_lean = float(wall_lean)
        self._route_target = route_target
        self._newton_params = dict(newton_params or {})

        self._episode_length = 0
        self._previous_tip_pos = None
        self._tip_history: deque[list[float]] = deque(maxlen=self.TIP_HISTORY_LEN)

        # Control abstraction (doc/09 ShapeIntent). When intent mode is engaged a
        # ShapeIntentController drives push/rotate from a high-level intent instead
        # of the caller's manual values. Lazily built from the planned path;
        # physics mode only (guided already follows the centerline kinematically).
        self._controller = None
        self._intent = None
        self._intent_active = False
        self._last_tip_pos: list[float] | None = None
        self._last_tip_dir: list[float] | None = None
        self._last_contact_force = 0.0
        self._last_path_progress = 0.0
        self._pending_control: dict[str, float] = {
            "delta_push": 0.0,
            "delta_rotate": 0.0,
            "microcatheter_advance": 0.0,
        }
        self._last_control_metrics: dict[str, Any] = {
            "push_command": 0.0,
            "rotate_command": 0.0,
            "support_command": 0.0,
            "progress_delta": 0.0,
            "progress_delta_m": 0.0,
        }

        from services.risk_assessor import RiskAssessor

        self._risk_assessor = RiskAssessor()

        # Planned path (arc-length geometry) shared with the backing engine.
        # Set _path / _engine to None first so set_planned_path's engine
        # propagation is a no-op during this initial setup.
        self._path: PlannedPath | None = None
        self._engine: PhysicsEngine | None = None
        self._routes: dict | None = self._load_phantom_routes()

        if planned_path is not None:
            self.set_planned_path(planned_path, radii=planned_radii)
        else:
            default_path = self._route_waypoints(route_target)
            default_radii = self._route_radii(route_target) if default_path is not None else None
            if default_path is None:
                default_path = self._default_centerline_points()
                default_radii = self._default_centerline_radii() if default_path is not None else None
            if default_path is not None:
                self.set_planned_path(default_path, radii=default_radii)

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
            engine_mode=self._physics_engine,
            newton_params=self._newton_params,
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

    def _default_centerline_radii(self) -> list[float] | None:
        """Load per-waypoint centerline radii from ``centerline.json`` when present."""
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
            radii = data.get("radius_m")
        except Exception:
            return None
        return radii if isinstance(radii, list) and len(radii) >= 2 else None

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

    def _load_phantom_routes(self) -> dict | None:
        """Load a built-in phantom's per-endpoint branch routes (routes.json).

        Multi-branch phantoms (e.g. aorta_tree) ship a routes.json mapping each
        leaf endpoint id to a dense, centered route from the vascular entry.
        Returns None when absent (single-route phantoms use centerline.json).
        """
        if self.assets_dir is not None:
            routes_path = Path(self.assets_dir).resolve().parent / "derived" / "routes.json"
        else:
            routes_path = (
                _SRC_DIR
                / "cathsim/dm/components/phantom_assets/meshes"
                / self.phantom
                / "routes.json"
            )
        if not routes_path.is_file():
            return None
        try:
            return json.loads(routes_path.read_text(encoding="utf-8"))
        except Exception:
            return None

    def _route_waypoints(self, target: str | None) -> list[list[float]] | None:
        """Waypoints of the branch route named ``target`` from routes.json."""
        if not self._routes or not target:
            return None
        route = self._routes.get("routes", {}).get(target)
        if not route:
            return None
        waypoints = route.get("waypoints")
        if isinstance(waypoints, list) and len(waypoints) >= 2:
            return waypoints
        return None

    def _route_radii(self, target: str | None) -> list[float] | None:
        """Per-waypoint lumen radii (m) of route ``target``, if it carries them.

        Multi-branch routes ship real VMTK inscribed radii (``radius_m``); the
        Newton backend uses them to build a variable-radius vessel wall.
        """
        if not self._routes or not target:
            return None
        route = self._routes.get("routes", {}).get(target)
        if not route:
            return None
        radii = route.get("radius_m")
        return radii if isinstance(radii, list) and len(radii) >= 2 else None

    @property
    def available_routes(self) -> dict[str, list[float]]:
        """Selectable branch targets as ``{endpoint_id: [x, y, z] target}`` (m)."""
        if not self._routes:
            return {}
        return {
            rid: list(route.get("target", []))
            for rid, route in self._routes.get("routes", {}).items()
        }

    def select_route(self, target: str) -> bool:
        """Switch the planned path to branch route ``target`` and reset progress."""
        waypoints = self._route_waypoints(target)
        if waypoints is None:
            return False
        self.set_planned_path(waypoints, radii=self._route_radii(target))
        self._route_target = target
        self._episode_length = 0
        self._previous_tip_pos = None
        self._tip_history.clear()
        if self._engine is not None and hasattr(self._engine, "_s"):
            self._engine._s = 0.0
        return True

    def set_engine_params(self, params: dict) -> dict | None:
        """Live-tune backend deformation params (interactive parameter panel).

        Forwards to the physics engine's ``set_deform_params`` when it exposes one
        (the Newton backend does); returns the effective param state, or ``None``
        for engines without live tuning. Engine-agnostic: no-op on kinematic/MuJoCo.
        """
        engine = getattr(self, "_engine", None)
        if engine is not None and hasattr(engine, "set_deform_params"):
            return engine.set_deform_params(**params)
        return None

    def set_shape_intent(self, intent: dict | None, active: bool = True) -> dict:
        """Engage/adjust ShapeIntent (autopilot) control of push/rotate.

        The physics stays pure force-driven (doc/09 §一): this only decides where
        the tip aims. While engaged, :meth:`step` ignores its manual push/rotate
        and instead has a :class:`~services.shape_intent.ShapeIntentController`
        resolve the current intent against the live tip pose.

        Args:
            intent: Intent fields (``target_waypoint`` / ``target_direction`` /
                    ``intensity``), or ``None`` for plain centerline autopilot.
            active: ``False`` disengages and hands control back to manual
                    push/rotate.

        Returns:
            ``{"active": bool, "mode": "centerline"|"waypoint"|"direction"|"off"}``
            describing the resulting control state.
        """
        if not active or not self._ensure_controller():
            self._intent_active = False
            self._intent = None
            return {"active": False, "mode": "off"}

        self._intent = self._build_intent(intent)
        self._intent_active = True
        if self._intent is None:
            mode = "centerline"
        elif self._intent.target_direction is not None:
            mode = "direction"
        elif self._intent.target_waypoint is not None:
            mode = "waypoint"
        else:
            mode = "centerline"
        return {"active": True, "mode": mode}

    def _build_intent(self, intent: dict | None):
        """Turn a raw intent dict into a :class:`ShapeIntent` (or None)."""
        if not intent:
            return None
        from services.shape_intent import ShapeIntent

        kwargs: dict[str, Any] = {}
        if intent.get("target_waypoint") is not None:
            kwargs["target_waypoint"] = intent["target_waypoint"]
        if intent.get("target_direction") is not None:
            kwargs["target_direction"] = intent["target_direction"]
        if intent.get("intensity") is not None:
            kwargs["intensity"] = intent["intensity"]
        if not kwargs:
            return None
        return ShapeIntent(**kwargs)

    def _ensure_controller(self) -> bool:
        """Lazily build the ShapeIntentController from the planned path.

        Returns False (intent control unavailable) in guided/kinematic mode or
        when there is no planned path to follow/bookkeep against.
        """
        if self._is_guided() or self._path is None:
            return False
        if self._controller is None:
            from services.shape_intent import ShapeIntentController

            self._controller = ShapeIntentController(self._path.points)
            self._controller.reset()
        return True

    def set_planned_path(
        self, planned_path: Sequence[Sequence[float]] | None, radii: Sequence[float] | None = None
    ) -> None:
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
            self._path = PlannedPath(planned_path, radii=radii)

        # The ShapeIntentController is bound to the path geometry; drop it so the
        # next intent request rebuilds it against the new route.
        self._controller = None

        if getattr(self, "_engine", None) is not None:
            # Backends keep path-derived state; let engines that cache geometry
            # rebuild themselves, otherwise keep the plain `_path` reference in sync.
            if hasattr(self._engine, "set_path"):
                self._engine.set_path(self._path)
            else:
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
        self._last_path_progress = 0.0
        self._pending_control = {
            "delta_push": 0.0,
            "delta_rotate": 0.0,
            "microcatheter_advance": 0.0,
        }
        self._last_control_metrics = {
            "push_command": 0.0,
            "rotate_command": 0.0,
            "support_command": 0.0,
            "progress_delta": 0.0,
            "progress_delta_m": 0.0,
        }
        self._tip_history.clear()
        if self._controller is not None:
            self._controller.reset()
        return self._assemble_state(raw)

    def step(
        self,
        delta_push: float,
        delta_rotate: float,
        microcatheter_advance: float = 0.0,
    ) -> NavigationState:
        """Execute one simulation step.

        Args:
            delta_push: Push force coefficient [-1.0, 1.0], positive = forward
            delta_rotate: Rotation force coefficient [-1.0, 1.0], positive = clockwise
            microcatheter_advance: Optional support advance coefficient [-1.0, 1.0].

        Returns:
            NavigationState after the step
        """
        # ShapeIntent (autopilot) mode: resolve the high-level intent against the
        # live tip pose into real push/rotate, overriding the caller's manual
        # values. The physics engine is still driven purely by (push, rotate).
        if self._intent_active and self._controller is not None and self._last_tip_pos is not None:
            delta_push, delta_rotate = self._controller.compute(
                self._intent,
                self._last_tip_pos,
                self._last_tip_dir,
                self._last_contact_force,
            )

        delta_push = float(np.clip(delta_push, -1.0, 1.0))
        delta_rotate = float(np.clip(delta_rotate, -1.0, 1.0))
        microcatheter_advance = float(np.clip(microcatheter_advance, -1.0, 1.0))
        self._pending_control = {
            "delta_push": delta_push,
            "delta_rotate": delta_rotate,
            "microcatheter_advance": microcatheter_advance,
        }
        if microcatheter_advance != 0.0 and hasattr(self._engine, "apply_support_control"):
            self._engine.apply_support_control(microcatheter_advance)

        raw = self._engine.step(delta_push, delta_rotate)
        self._episode_length += 1
        return self._assemble_state(raw)

    def _is_guided(self) -> bool:
        """Whether the active backend is the kinematic centerline-follow engine.

        Kept as a thin accessor (the runtime fork is gone -- the backend type is
        fixed at construction). Returns False before the engine is constructed.
        """
        return isinstance(self._engine, KinematicEngine)

    def _is_force_physics(self) -> bool:
        """Whether the active backend is a real force-drive physics engine.

        Force-drive lets the wire physically ride against the lumen wall, so
        safety/risk must key on penetration (contact_force), not clearance.
        False for kinematic/MuJoCo/anchor backends (they keep the wire centered).
        """
        return bool(getattr(self._engine, "is_force_drive", False))

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

        # Cache the raw pose the ShapeIntentController steers on next step.
        self._last_tip_pos = tip_pos
        self._last_tip_dir = raw.tip_direction
        self._last_contact_force = float(raw.contact_force)

        velocity = self._compute_velocity(tip_pos)
        curvature = self._compute_curvature()

        if raw.arclen is not None and self._path is not None and self._path.total_len > 0.0:
            path_progress = float(raw.arclen / self._path.total_len)
            path_point = self._path.point_at_arclen(float(raw.arclen))
            path_deviation = float(
                np.linalg.norm(np.asarray(tip_pos, dtype=np.float64) - path_point)
            )
        else:
            path_progress, path_deviation = self._compute_path_progress(tip_pos)
        path_progress = float(np.clip(path_progress, 0.0, 1.0))

        remaining_distance = self._compute_remaining_distance(path_progress)
        path_total_distance = (
            float(self._path.total_len)
            if self._path is not None and self._path.total_len > 0.0
            else None
        )
        path_travelled_distance = (
            float(path_progress * path_total_distance)
            if path_total_distance is not None
            else None
        )
        vessel_radius = self._compute_vessel_radius(path_progress)
        eta_seconds = self._compute_eta_seconds(remaining_distance, velocity)
        progress_delta = float(path_progress - self._last_path_progress)
        progress_delta_m = (
            progress_delta * self._path.total_len
            if self._path is not None and self._path.total_len > 0.0
            else 0.0
        )
        self._last_control_metrics = {
            "push_command": float(self._pending_control.get("delta_push", 0.0)),
            "rotate_command": float(self._pending_control.get("delta_rotate", 0.0)),
            "support_command": float(self._pending_control.get("microcatheter_advance", 0.0)),
            "progress_delta": progress_delta,
            "progress_delta_m": float(progress_delta_m),
        }

        force_mode = self._is_force_physics()
        wall_penetration = float(
            max(
                0.0,
                getattr(
                    raw,
                    "wall_penetration",
                    getattr(raw, "max_penetration", 0.0),
                ),
            )
        )
        safety_status = self._compute_safety_status(
            self._episode_length, raw.wall_distance, wall_penetration
        )

        state = NavigationState(
            tip_position=tip_pos,
            tip_direction=raw.tip_direction,
            tip_quaternion=raw.tip_quaternion,
            velocity=float(velocity),
            contact_force=float(raw.contact_force),
            contact_count=int(getattr(raw, "contact_count", 0)),
            tip_contact_count=int(
                getattr(raw, "tip_contact_count", getattr(raw, "contact_count", 0))
            ),
            rod_contact_force=float(getattr(raw, "rod_contact_force", 0.0)),
            rod_contact_count=int(getattr(raw, "rod_contact_count", 0)),
            wall_contact_count=int(getattr(raw, "wall_contact_count", 0)),
            max_penetration=float(getattr(raw, "max_penetration", 0.0)),
            wall_penetration=wall_penetration,
            contact_impulse=float(getattr(raw, "contact_impulse", 0.0)),
            wall_distance=float(raw.wall_distance),
            curvature=float(curvature),
            episode_length=self._episode_length,
            target_position=raw.target_position,
            path_progress=float(path_progress),
            path_deviation=float(path_deviation),
            remaining_distance=float(remaining_distance),
            path_total_distance=path_total_distance,
            path_travelled_distance=path_travelled_distance,
            vessel_radius=vessel_radius,
            eta_seconds=eta_seconds,
            fidelity_mode=self.fidelity_mode,
            risk_regions=self._risk_regions(path_progress, path_deviation),
            joint_positions=raw.joint_positions,
            joint_velocities=raw.joint_velocities,
            safety_status=safety_status,
            reward=float(raw.reward),
            done=raw.done,
        )
        risk_assessment = self._risk_assessor.assess(state, force_mode=force_mode)
        state.risk_score = risk_assessment["risk_score"]
        state.risk_assessment = risk_assessment
        state.flow_guidance = self._flow_guidance(state)
        self._last_path_progress = float(path_progress)
        return state

    def _engine_mechanics_state(self) -> dict[str, Any] | None:
        """Return backend guidewire mechanics when the active engine models them."""
        backend = getattr(self, "_engine", None)
        if backend is None or not hasattr(backend, "mechanics_state"):
            return None
        mechanics = backend.mechanics_state()
        return mechanics if isinstance(mechanics, dict) else None

    def _flow_guidance(self, state: NavigationState) -> dict[str, Any]:
        """Derive the large-curvature workflow block from source-backed fields.

        Conservative fallback is preserved for backends that do not model
        guidewire shape/support. Newton force mode can now supply those sources
        through ``mechanics_state``.
        """
        orientation_score = self._orientation_score(state)
        phase = self._workflow_phase(state, orientation_score)
        step_index = {
            "STANDBY": 0,
            "TIP_SHAPE": 1,
            "ORIENTATION": 2,
            "SUPPORT": 3,
            "MICRO_ADVANCE": 4,
            "WALL_SLIDE": 5,
            "STRATEGY_SWITCH": 6,
        }[phase]
        allowed, blocked, suggestion = self._workflow_actions(phase)
        reason_codes = self._flow_reason_codes(state, orientation_score)
        mechanics = self._engine_mechanics_state()
        mechanics_source = mechanics.get("source") if mechanics else None
        mechanics_fields = mechanics.get("source_fields", []) if mechanics else []
        guidewire = mechanics.get("guidewire", {}) if mechanics else {}
        support = mechanics.get("support", {}) if mechanics else {}
        mechanics_risk = mechanics.get("risk", {}) if mechanics else {}

        orientation_state = "unknown"
        if orientation_score is not None:
            orientation_state = "aligned" if orientation_score >= 0.75 else "needs_alignment"
        tip_facing_score = guidewire.get("tip_facing_score")
        if tip_facing_score is None:
            tip_facing_score = orientation_score

        tip_shape_block = {
            "tip_shape_state": "unknown",
            "shape_type": None,
            "curve_angle_deg": None,
            "tip_length_m": None,
            "tip_facing_score": tip_facing_score,
            "torsion_lag_deg": guidewire.get("torsion_lag_deg"),
            "source": "not_modeled",
            "source_fields": [],
        }
        if guidewire:
            tip_shape_block.update({
                "tip_shape_state": "modeled",
                "shape_type": guidewire.get("shape_type") or guidewire.get("tip_shape"),
                "curve_angle_deg": guidewire.get("curve_angle_deg"),
                "tip_length_m": guidewire.get("tip_length_m"),
                "source": mechanics_source,
                "source_fields": mechanics_fields,
            })

        support_block = {
            "support_state": "unknown",
            "effective_support_type": None,
            "effective_support_tip_m": None,
            "free_wire_length_m": None,
            "support_ratio": None,
            "max_slack_m": None,
            "source": "not_modeled",
            "source_fields": [],
        }
        if support:
            support_block.update({
                "support_state": support.get("support_state", "modeled"),
                "effective_support_type": support.get("effective_support_type"),
                "effective_support_tip_m": support.get("effective_support_tip_m"),
                "free_wire_length_m": support.get("free_wire_length_m"),
                "support_ratio": support.get("support_ratio"),
                "max_slack_m": support.get("max_slack_m"),
                "source": mechanics_source,
                "source_fields": mechanics_fields,
            })

        wall_slide_state = mechanics_risk.get("wall_slide_state") or self._wall_slide_state(state)
        buckling_risk = mechanics_risk.get("buckling_risk")

        strategy_state = self._strategy_switch_state(state)
        micro_advance_block = self._micro_advance_block(state, mechanics_risk)
        strategy_switch_block = self._strategy_switch_block(
            state=state,
            strategy_state=strategy_state,
            reason_codes=reason_codes,
            buckling_risk=buckling_risk,
            mechanics_risk=mechanics_risk,
            micro_advance=micro_advance_block,
        )
        training_score = self._large_curvature_training_score(
            state=state,
            orientation_score=orientation_score,
            phase=phase,
            tip_shape=tip_shape_block,
            support=support_block,
            micro_advance=micro_advance_block,
            wall_slide_state=wall_slide_state,
            strategy_state=strategy_state,
            buckling_risk=buckling_risk,
            mechanics_risk=mechanics_risk,
        )

        return {
            "workflow": {
                "phase": phase,
                "step_index": step_index,
                "step_label": self._workflow_label(phase),
                "allowed_actions": allowed,
                "blocked_actions": blocked,
                "suggestion": suggestion,
                "reason_codes": reason_codes,
                "source": "navigation_engine.flow_guidance",
                "source_fields": [
                    "episode_length",
                    "tip_direction",
                    "planned_path",
                    "path_progress",
                    "path_deviation",
                    "wall_distance",
                    "contact_force",
                    "risk_score",
                    "safety_status",
                ],
            },
            "tip_shape": tip_shape_block,
            "orientation": {
                "orientation_state": orientation_state,
                "orientation_score": orientation_score,
                "tip_facing_score": tip_facing_score,
                "proximal_rotation_deg": guidewire.get("proximal_rotation_deg"),
                "distal_tip_rotation_deg": guidewire.get("distal_tip_rotation_deg"),
                "torsion_lag_deg": guidewire.get("torsion_lag_deg"),
                "tip_deflection_score": guidewire.get("tip_deflection_score"),
                "source": "planned_path_tangent" if orientation_score is not None else "unknown",
                "source_fields": ["tip_direction", "planned_path", "path_progress"]
                if orientation_score is not None
                else [],
            },
            "support": support_block,
            "micro_advance": micro_advance_block,
            "wall_slide": {
                "wall_slide_state": wall_slide_state,
                "normal_poking_score": mechanics_risk.get("normal_poking_score"),
                "tangential_slide_score": mechanics_risk.get("tangential_slide_score"),
                "source": mechanics_source or "navigation_engine.risk_state",
                "source_fields": mechanics_fields or ["wall_distance", "contact_force", "safety_status"],
            },
            "strategy_switch": strategy_switch_block,
            "training_score": training_score,
        }

    @staticmethod
    def _large_curvature_training_score(
        *,
        state: NavigationState,
        orientation_score: float | None,
        phase: str,
        tip_shape: dict[str, Any],
        support: dict[str, Any],
        micro_advance: dict[str, Any],
        wall_slide_state: str,
        strategy_state: str,
        buckling_risk: str | None,
        mechanics_risk: dict[str, Any],
    ) -> dict[str, Any]:
        def clamp_score(value: float) -> int:
            return int(round(float(np.clip(value, 0.0, 100.0))))

        source_fields = [
            "tip_shape",
            "orientation_score",
            "support_ratio",
            "risk_score",
            "wall_distance",
            "contact_force",
            "safety_status",
        ]
        reasons: list[str] = []

        shape_state = str(tip_shape.get("tip_shape_state", "unknown"))
        shape_type = tip_shape.get("shape_type")
        curve_angle = tip_shape.get("curve_angle_deg")
        if shape_state == "modeled":
            curve = 0.0 if curve_angle is None else abs(float(curve_angle))
            if str(shape_type) in {"j_tip", "c_shape", "hook", "angled", "s_shape"} and curve > 1.0:
                tip_shape_score = 100
            else:
                tip_shape_score = 70
                reasons.append("TIP_SHAPE_LOW_CURVE")
        else:
            tip_shape_score = 50
            reasons.append("TIP_SHAPE_NOT_MODELED")

        if orientation_score is None:
            orientation_component = 50
            reasons.append("ORIENTATION_UNKNOWN")
        else:
            orientation_component = clamp_score(orientation_score * 100.0)
            if orientation_score < 0.75:
                reasons.append("TIP_NOT_ALIGNED_TO_PATH")

        support_ratio = support.get("support_ratio")
        free_wire = support.get("free_wire_length_m")
        max_slack = support.get("max_slack_m")
        if support.get("support_state") == "unknown":
            support_component = 50
            reasons.append("SUPPORT_NOT_MODELED")
        elif buckling_risk == "HIGH":
            support_component = 35
            reasons.append("BUCKLING_RISK_HIGH")
        elif support_ratio is not None:
            support_component = clamp_score(45.0 + float(support_ratio) * 55.0)
        elif free_wire is not None and max_slack is not None and float(max_slack) > 0.0:
            support_component = clamp_score(100.0 - (float(free_wire) / float(max_slack)) * 45.0)
        else:
            support_component = 70

        micro_state = str(micro_advance.get("micro_advance_state", NavigationEngine._micro_advance_state(state)))
        hard_push_score = float(micro_advance.get("hard_push_score", 0.0) or 0.0)
        cadence_state = str(micro_advance.get("cadence_state", "unknown"))
        micro_component = {"ready": 100, "caution": 65, "blocked": 20, "unknown": 50}.get(micro_state, 50)
        if cadence_state == "micro_push":
            micro_component = max(micro_component, 90)
        elif cadence_state == "pullback":
            micro_component = max(micro_component, 80)
        elif cadence_state == "hard_push":
            micro_component = min(micro_component, 35)
        if hard_push_score >= 0.7:
            micro_component = min(micro_component, 25)
            reasons.append("HARD_PUSH_DETECTED")
        elif hard_push_score >= 0.35:
            micro_component = min(micro_component, 60)
            reasons.append("HARD_PUSH_WARNING")
        if micro_state in {"caution", "blocked"}:
            reasons.append("MICRO_ADVANCE_RISK")

        normal_poking = mechanics_risk.get("normal_poking_score")
        tangential_slide = mechanics_risk.get("tangential_slide_score")
        if normal_poking is not None and float(normal_poking) >= 0.7:
            wall_component = 25
            reasons.append("TIP_POKING_WARNING")
        elif tangential_slide is not None:
            wall_component = clamp_score(float(tangential_slide) * 100.0)
        else:
            wall_component = {
                "WALL_SLIDE_OK": 100,
                "active": 80,
                "clear": 75,
                "CLEAR": 75,
                "WALL_CONTACT_MONITOR": 60,
                "TIP_POKING_WARNING": 25,
                "BREACH_STOP": 10,
                "unsafe": 10,
                "unknown": 50,
            }.get(str(wall_slide_state), 50)
        if str(wall_slide_state) in {"TIP_POKING_WARNING", "BREACH_STOP", "unsafe"}:
            reasons.append(str(wall_slide_state))

        strategy_component = {
            "not_required": 100,
            "consider": 75,
            "recommended": 55,
            "required": 35,
            "unknown": 50,
        }.get(strategy_state, 50)
        if strategy_state in {"consider", "recommended", "required"}:
            reasons.append("STRATEGY_SWITCH_" + strategy_state.upper())

        components = {
            "tip_shape": tip_shape_score,
            "orientation": orientation_component,
            "support": support_component,
            "micro_advance": micro_component,
            "wall_slide": wall_component,
            "strategy_switch": strategy_component,
        }
        weights = {
            "tip_shape": 0.12,
            "orientation": 0.18,
            "support": 0.18,
            "micro_advance": 0.18,
            "wall_slide": 0.18,
            "strategy_switch": 0.16,
        }
        overall = clamp_score(sum(components[key] * weights[key] for key in components))
        return {
            "overall": overall,
            "components": components,
            "deductions": sorted(set(reasons)),
            "phase": phase,
            "source": "navigation_engine.large_curvature_training_score",
            "source_fields": source_fields,
        }

    @staticmethod
    def _strategy_recommendations(reason_codes: list[str], buckling_risk: str | None) -> list[str]:
        actions: list[str] = []
        if "COLLISION_STOP" in reason_codes or "HIGH_RISK_SCORE" in reason_codes:
            actions.extend(["pause", "pullback", "reorient_tip"])
        if buckling_risk == "HIGH":
            actions.extend(["advance_support", "reduce_free_span", "micro_pullback"])
        if "TIP_NOT_ALIGNED_TO_PATH" in reason_codes:
            actions.append("rotate_to_target_sector")
        if "PATH_DEVIATION_HIGH" in reason_codes:
            actions.extend(["pullback_to_known_path", "select_alternate_branch"])
        if not actions:
            actions.append("continue_micro_advance")
        deduped: list[str] = []
        for action in actions:
            if action not in deduped:
                deduped.append(action)
        return deduped

    def _strategy_switch_block(
        self,
        *,
        state: NavigationState,
        strategy_state: str,
        reason_codes: list[str],
        buckling_risk: str | None,
        mechanics_risk: dict[str, Any],
        micro_advance: dict[str, Any],
    ) -> dict[str, Any]:
        actions = self._strategy_recommendations(reason_codes, buckling_risk)
        reasons: list[str] = []
        if "COLLISION_STOP" in reason_codes:
            reasons.append("collision_stop")
        if "HIGH_RISK_SCORE" in reason_codes:
            reasons.append("high_risk_score")
        if mechanics_risk.get("wall_slide_state") in {"BREACH_STOP", "TIP_POKING_WARNING"}:
            reasons.append(str(mechanics_risk.get("wall_slide_state")).lower())
        if buckling_risk == "HIGH":
            reasons.append("buckling_high")
        elif buckling_risk == "MEDIUM":
            reasons.append("buckling_medium")
        if "TIP_NOT_ALIGNED_TO_PATH" in reason_codes:
            reasons.append("tip_not_aligned")
        if "PATH_DEVIATION_HIGH" in reason_codes:
            reasons.append("path_deviation_high")
        if micro_advance.get("hard_push_state") in {"warning", "unsafe"}:
            reasons.append("hard_push")
        if "NOT_RUNNING" in reason_codes:
            reasons.append("not_running")
        if not reasons:
            reasons.append("none")

        severity = "info"
        if strategy_state == "required" or "collision_stop" in reasons:
            severity = "critical"
        elif strategy_state in {"recommended", "consider"} or any(
            item not in {"none", "not_running"} for item in reasons
        ):
            severity = "warning"

        primary_action = actions[0] if actions else "continue_micro_advance"
        panel_title = {
            "critical": "Strategy switch required",
            "warning": "Strategy switch recommended",
            "info": "Strategy stable",
        }[severity]
        return {
            "strategy_switch_state": strategy_state,
            "severity": severity,
            "panel_title": panel_title,
            "primary_failure_reason": reasons[0],
            "failure_reasons": reasons,
            "primary_action": primary_action,
            "recommended_actions": actions,
            "buckling_risk": buckling_risk,
            "slack_m": mechanics_risk.get("slack_m"),
            "feed_budget_m": mechanics_risk.get("feed_budget_m"),
            "hard_push_state": micro_advance.get("hard_push_state"),
            "hard_push_score": micro_advance.get("hard_push_score"),
            "source": "navigation_engine.strategy_switch",
            "source_fields": [
                "risk_score",
                "safety_status",
                "path_deviation",
                "buckling_risk",
                "wall_slide_state",
                "hard_push_state",
            ],
        }

    def _orientation_score(self, state: NavigationState) -> float | None:
        """Tip/path alignment in [0, 1], or None when no planned path exists."""
        if self._path is None or self._path.total_len <= 0.0:
            return None
        tip_dir = np.asarray(state.tip_direction, dtype=np.float64)
        tip_norm = float(np.linalg.norm(tip_dir))
        if tip_norm <= 1e-9:
            return None
        tangent = self._path.tangent_at_arclen(state.path_progress * self._path.total_len)
        score = float(np.dot(tip_dir / tip_norm, tangent))
        return float(np.clip(score, 0.0, 1.0))

    def _workflow_phase(self, state: NavigationState, orientation_score: float | None) -> str:
        if state.episode_length == 0 or state.safety_status == "STANDBY":
            return "STANDBY"
        if state.safety_status == "COLLISION_STOP" or state.risk_score >= 0.75:
            return "STRATEGY_SWITCH"
        if state.contact_force > 0.0 or (0.0 < state.wall_distance < 0.0015):
            return "WALL_SLIDE"
        if orientation_score is not None and orientation_score < 0.75:
            return "ORIENTATION"
        return "MICRO_ADVANCE"

    @staticmethod
    def _workflow_label(phase: str) -> str:
        return {
            "STANDBY": "Standby",
            "TIP_SHAPE": "Tip shape",
            "ORIENTATION": "Orientation",
            "SUPPORT": "Support",
            "MICRO_ADVANCE": "Micro advance",
            "WALL_SLIDE": "Wall slide",
            "STRATEGY_SWITCH": "Strategy switch",
        }[phase]

    @staticmethod
    def _workflow_actions(phase: str) -> tuple[list[str], list[str], str]:
        if phase == "STANDBY":
            return [], ["hard_push"], "Wait for a valid navigation state"
        if phase == "STRATEGY_SWITCH":
            return ["pause", "pullback", "rotate", "change_strategy"], ["hard_push"], "Pause push, pull back, or reorient"
        if phase == "WALL_SLIDE":
            return ["micro_push", "rotate", "pause", "pullback"], ["hard_push"], "Use micro-advance while monitoring wall distance and contact force"
        if phase == "ORIENTATION":
            return ["rotate", "pause", "micro_push"], ["hard_push"], "Align by rotation before micro-advance"
        return ["micro_push", "rotate", "pause"], ["hard_push"], "Advance in small steps and keep monitoring risk"

    def _micro_advance_block(self, state: NavigationState, mechanics_risk: dict[str, Any]) -> dict[str, Any]:
        base_state = self._micro_advance_state(state)
        metrics = dict(self._last_control_metrics)
        push = float(metrics.get("push_command", 0.0))
        rotate = float(metrics.get("rotate_command", 0.0))
        support_command = float(metrics.get("support_command", 0.0))
        progress_delta = float(metrics.get("progress_delta", 0.0))
        progress_delta_m = float(metrics.get("progress_delta_m", 0.0))

        normal_poking = float(mechanics_risk.get("normal_poking_score") or 0.0)
        tangential_slide = float(mechanics_risk.get("tangential_slide_score") or 0.0)
        slack = mechanics_risk.get("slack_m")
        feed_budget = mechanics_risk.get("feed_budget_m")
        buckling_risk = str(mechanics_risk.get("buckling_risk", ""))

        if push < -0.05:
            cadence_state = "pullback"
        elif abs(push) <= 0.05 and abs(rotate) <= 0.05:
            cadence_state = "pause"
        elif 0.0 < push <= 0.35:
            cadence_state = "micro_push"
        elif push > 0.35:
            cadence_state = "hard_push"
        else:
            cadence_state = "rotate_only"

        risk_pressure = 0.0
        if state.risk_score >= 0.75 or state.safety_status == "COLLISION_STOP":
            risk_pressure = max(risk_pressure, 1.0)
        elif state.risk_score >= 0.35:
            risk_pressure = max(risk_pressure, 0.5)
        if state.contact_force > 0.0:
            risk_pressure = max(risk_pressure, 0.65)
        if normal_poking >= 0.7:
            risk_pressure = max(risk_pressure, 1.0)
        elif normal_poking >= 0.35:
            risk_pressure = max(risk_pressure, 0.55)
        if buckling_risk == "HIGH":
            risk_pressure = max(risk_pressure, 1.0)
        elif buckling_risk == "MEDIUM":
            risk_pressure = max(risk_pressure, 0.55)
        if feed_budget is not None:
            try:
                if float(feed_budget) <= 0.001:
                    risk_pressure = max(risk_pressure, 0.8)
            except (TypeError, ValueError):
                pass

        push_pressure = max(0.0, push)
        stalled = bool(push > 0.2 and progress_delta_m < 0.00025 and state.episode_length > 1)
        if stalled:
            risk_pressure = max(risk_pressure, 0.6)
        hard_push_score = float(np.clip(push_pressure * (0.35 + 0.65 * risk_pressure), 0.0, 1.0))
        if push > 0.75 and risk_pressure >= 0.5:
            hard_push_state = "unsafe"
        elif push > 0.35 and risk_pressure >= 0.5:
            hard_push_state = "warning"
        elif push > 0.75:
            hard_push_state = "excessive"
        else:
            hard_push_state = "clear"

        if base_state == "blocked":
            recommendation = "pullback_or_reorient"
        elif hard_push_state in {"unsafe", "warning"}:
            recommendation = "reduce_push_to_micro_steps"
        elif cadence_state == "pullback":
            recommendation = "recover_space_then_reorient"
        elif cadence_state == "pause":
            recommendation = "pause_observe_or_rotate"
        else:
            recommendation = "continue_micro_advance"

        return {
            "micro_advance_state": base_state,
            "cadence_state": cadence_state,
            "hard_push_state": hard_push_state,
            "hard_push_score": hard_push_score,
            "push_command": push,
            "rotate_command": rotate,
            "support_command": support_command,
            "progress_delta": progress_delta,
            "progress_delta_m": progress_delta_m,
            "stalled": stalled,
            "normal_poking_score": normal_poking,
            "tangential_slide_score": tangential_slide,
            "slack_m": slack,
            "feed_budget_m": feed_budget,
            "recommendation": recommendation,
            "source": "navigation_engine.control_cadence",
            "source_fields": [
                "delta_push",
                "delta_rotate",
                "microcatheter_advance",
                "path_progress",
                "contact_force",
                "risk_score",
                "normal_poking_score",
                "tangential_slide_score",
                "slack_m",
                "feed_budget_m",
            ],
        }

    @staticmethod
    def _micro_advance_state(state: NavigationState) -> str:
        if state.safety_status == "COLLISION_STOP" or state.risk_score >= 0.75:
            return "blocked"
        if state.risk_score >= 0.35 or (0.0 < state.wall_distance < 0.0015):
            return "caution"
        if state.safety_status == "STANDBY":
            return "unknown"
        return "ready"

    @staticmethod
    def _wall_slide_state(state: NavigationState) -> str:
        if state.safety_status == "COLLISION_STOP":
            return "unsafe"
        if state.contact_force > 0.0 or (0.0 < state.wall_distance < 0.0015):
            return "active"
        if state.safety_status == "STANDBY":
            return "unknown"
        return "clear"

    @staticmethod
    def _strategy_switch_state(state: NavigationState) -> str:
        if state.safety_status == "COLLISION_STOP":
            return "required"
        if state.risk_score >= 0.75:
            return "recommended"
        if state.path_deviation >= 0.003:
            return "consider"
        if state.safety_status == "STANDBY":
            return "unknown"
        return "not_required"

    @staticmethod
    def _flow_reason_codes(state: NavigationState, orientation_score: float | None) -> list[str]:
        reasons: list[str] = []
        if state.safety_status == "COLLISION_STOP":
            reasons.append("COLLISION_STOP")
        if state.risk_score >= 0.75:
            reasons.append("HIGH_RISK_SCORE")
        elif state.risk_score >= 0.35:
            reasons.append("WARNING_RISK_SCORE")
        if 0.0 < state.wall_distance < 0.0008:
            reasons.append("WALL_DISTANCE_DANGER")
        elif 0.0 < state.wall_distance < 0.0015:
            reasons.append("WALL_DISTANCE_WARNING")
        if state.contact_force > 0.0:
            reasons.append("CONTACT_FORCE_PRESENT")
        if orientation_score is not None and orientation_score < 0.75:
            reasons.append("TIP_NOT_ALIGNED_TO_PATH")
        if state.path_deviation >= 0.003:
            reasons.append("PATH_DEVIATION_HIGH")
        if state.safety_status == "STANDBY":
            reasons.append("NOT_RUNNING")
        return reasons

    def _compute_remaining_distance(self, path_progress: float) -> float:
        """Remaining arc length to target in meters."""
        if self._path is None or self._path.total_len <= 0.0:
            return 0.0
        return max(0.0, (1.0 - float(path_progress)) * self._path.total_len)

    def _compute_vessel_radius(self, path_progress: float) -> float | None:
        """Local lumen radius in meters from VMTK/route data, when available."""
        if self._path is None or self._path.total_len <= 0.0:
            return None
        return self._path.radius_at_arclen(float(path_progress) * self._path.total_len)

    def _compute_eta_seconds(self, remaining_distance: float, velocity: float) -> float | None:
        """Estimated time to target from current speed; None when stationary."""
        if remaining_distance <= 0.0:
            return 0.0
        if velocity <= 1e-6:
            return None
        return float(remaining_distance / velocity)

    def _risk_regions(self, path_progress: float, path_deviation: float) -> list[dict[str, Any]]:
        """Protocol placeholder for backend-provided risk regions.

        Until the SDF/risk-map backend supplies source-backed spatial regions,
        expose a stable empty list. Path deviation still contributes to the
        aggregate risk score, but it is not a renderable no-go/risk volume.
        """
        return []

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

    def _compute_safety_status(
        self,
        episode_length: int,
        wall_distance: float,
        wall_penetration: float = 0.0,
    ) -> SafetyStatus:
        """Derive the safety status, mode-aware.

        Guided/kinematic (default): the wire rides the centerline, so clearance to
        the wall (``wall_distance``) is the risk -- the original mm bands apply.

        Force-drive physics uses the independent geometric penetration signal;
        physical contact force is deliberately not converted back into breach.
        """
        if episode_length == 0:
            return "STANDBY"

        if self._is_force_physics():
            penetration = max(0.0, float(wall_penetration))
            if penetration < self.BREACH_WARN:
                return "SAFE_NAV"
            if penetration < self.BREACH_STOP:
                return "DANGER_WARNING"
            return "COLLISION_STOP"

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

    @property
    def fidelity_mode(self) -> str:
        """Current simulation fidelity bucket exposed to API/HUD clients."""
        return "guided" if self._is_guided() else "physics"

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
