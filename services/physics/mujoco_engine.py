"""MuJoCoEngine: the physical guidewire simulation behind the PhysicsEngine seam.

Owns everything MuJoCo-specific that used to live inline in ``NavigationEngine``:
``make_dm_env`` construction (lazy, on first reset), the prethread inverse
kinematics that bends the spawned wire onto the planned centerline, the
insertion slider / steering-hinge joint layout, and contact-based wall-distance
extraction. None of this leaks through the :class:`~services.physics.base.PhysicsEngine`
contract -- the engine emits only :class:`RawPose`.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Sequence

import numpy as np

from services.physics.base import MAX_WALL_DISTANCE, PlannedPath, RawPose

# Make the in-repo `cathsim` package importable even when the server runs in a
# Python environment where it was not installed editable. cathsim lives under
# <project_root>/src.
_SRC_DIR = Path(__file__).resolve().parents[2] / "src"
if _SRC_DIR.is_dir() and str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))


def _rot_x(a: float) -> np.ndarray:
    """Rotation matrix about the x axis by angle ``a`` (radians)."""
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rot_y(b: float) -> np.ndarray:
    """Rotation matrix about the y axis by angle ``b`` (radians)."""
    c, s = np.cos(b), np.sin(b)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


class MuJoCoEngine:
    """Physical guidewire engine implementing the PhysicsEngine seam.

    Construction is cheap; the dm_control environment is built lazily on the
    first :meth:`reset` so callers can introspect configuration without paying
    the (~18-20s for aorta_trunk) MJCF compile until they actually simulate.
    """

    def __init__(
        self,
        phantom: str,
        target: str,
        *,
        use_pixels: bool = False,
        image_size: int = 80,
        assets_dir: str | None = None,
        n_bodies: int = 80,
        n_substeps: int | None = None,
        insertion_max: float = 0.2,
        stiffness_scale: float = 1.0,
        prethread: bool = False,
        path: PlannedPath | None = None,
        entry_point: Sequence[float] | None = None,
        entry_direction: Sequence[float] | None = None,
    ) -> None:
        self.phantom = phantom
        self.target = target
        self.use_pixels = use_pixels
        self.image_size = image_size
        self.assets_dir = assets_dir
        self.n_bodies = n_bodies
        self.n_substeps = n_substeps
        self.insertion_max = float(insertion_max)
        self.stiffness_scale = float(stiffness_scale)
        self.prethread = bool(prethread)
        self._path = path
        self._entry_point = (
            np.asarray(entry_point, dtype=np.float64) if entry_point is not None else None
        )
        self._entry_direction = (
            np.asarray(entry_direction, dtype=np.float64)
            if entry_direction is not None
            else None
        )

        self._env = None
        self._time_step = None
        self._initialized = False
        # Cached (geom_id, body_id) pairs for guidewire render data; built once
        # since the model structure is fixed after initialization.
        self._render_geom_ids: list[tuple[int, int]] | None = None

    @property
    def is_initialized(self) -> bool:
        return self._initialized

    @property
    def control_timestep(self) -> float:
        if self._env is not None:
            return float(getattr(self._env.task, "control_timestep", 0.02))
        return 0.02

    def _ensure_initialized(self) -> None:
        """Lazy initialization of the CathSim dm_control environment."""
        if self._initialized:
            return

        from cathsim.dm import make_dm_env

        entry_point = self._entry_point
        entry_direction = self._entry_direction

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

    def reset(self) -> RawPose:
        """Reset the environment (building it lazily) and return the first pose."""
        self._ensure_initialized()
        self._time_step = self._env.reset()
        if self.prethread and self._path is not None:
            self._thread_physics_along_path()
        return self._raw_pose()

    def step(self, push: float, rotate: float) -> RawPose:
        """Execute one MuJoCo control step."""
        if not self._initialized or self._time_step is None:
            raise RuntimeError("Engine not initialized. Call reset() first.")
        action = np.array([push, rotate], dtype=np.float64)
        self._time_step = self._env.step(action)
        return self._raw_pose()

    # -- raw state extraction --------------------------------------------------

    def _raw_pose(self) -> RawPose:
        """Read the current MuJoCo state into an engine-agnostic RawPose."""
        obs = self._time_step.observation
        physics = self._env.physics
        task = self._env.task

        target_pos = task.target_pos
        if isinstance(target_pos, np.ndarray):
            target_pos = target_pos.tolist()

        reward = self._time_step.reward if self._time_step.reward is not None else 0.0

        return RawPose(
            tip_position=task.get_head_pos(physics).tolist(),
            tip_direction=self._compute_tip_direction(physics),
            tip_quaternion=self._compute_tip_quaternion(physics),
            contact_force=float(task.get_total_force(physics)),
            wall_distance=self._compute_wall_distance(physics),
            target_position=target_pos,
            joint_positions=obs.get("joint_pos", np.array([])).tolist(),
            joint_velocities=obs.get("joint_vel", np.array([])).tolist(),
            reward=float(reward),
            done=self._time_step.last(),
            arclen=None,
        )

    @staticmethod
    def _compute_tip_direction(physics) -> list[float]:
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

    @staticmethod
    def _compute_tip_quaternion(physics) -> list[float]:
        """Tip body orientation as a quaternion in [x, y, z, w] order.

        MuJoCo stores quaternions as [w, x, y, z]; reorder to [x, y, z, w] to
        match the Godot/WebSocket protocol convention.
        """
        try:
            body_id = int(physics.model.geom_bodyid[-1])
            w, x, y, z = (float(v) for v in physics.data.xquat[body_id])
            return [x, y, z, w]
        except Exception:
            return [0.0, 0.0, 0.0, 1.0]

    @staticmethod
    def _compute_wall_distance(physics) -> float:
        """Estimate the minimum gap between the guidewire and the vessel wall.

        Contact-based proxy: when MuJoCo reports active contacts the gap distance
        (clamped at 0 for penetration) is used; otherwise the free-space sentinel
        is returned. Distances are in meters.
        """
        ncon = int(physics.data.ncon)
        if ncon == 0:
            return MAX_WALL_DISTANCE

        dists = np.asarray(physics.data.contact.dist[:ncon], dtype=np.float64)
        min_gap = float(np.clip(dists, 0.0, None).min())
        return min(min_gap, MAX_WALL_DISTANCE)

    # -- render ---------------------------------------------------------------

    def render_bodies(self) -> list[dict[str, list[float]]]:
        """Per-segment guidewire render data from the live physics geoms."""
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

    # -- prethread inverse kinematics -----------------------------------------

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
        entry = self._path.points[0]
        base_pos = np.asarray(physics.data.xpos[base_id], dtype=np.float64)
        slide = float(np.dot(entry - base_pos, feed_axis))
        slide = float(np.clip(slide, 0.0, self.insertion_max))
        slider_adr = self._slider_qposadr(model)
        if slider_adr is not None:
            qpos[slider_adr] = slide

        s = 0.0
        for _body_id, seg_len, j0_adr, j1_adr in chain[1:]:
            s = min(s + seg_len, self._path.total_len)
            tangent = self._path.tangent_at_arclen(s)
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

    def close(self) -> None:
        """Release the dm_control environment."""
        if self._env is not None:
            del self._env
            self._env = None
        self._initialized = False
        self._time_step = None
