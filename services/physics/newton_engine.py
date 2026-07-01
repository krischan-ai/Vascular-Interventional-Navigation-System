"""NewtonEngine demo backend for front-end physics validation.

This is intentionally a small, opt-in demo engine.  It reuses the D0/D1 Newton
recipe (``add_rod`` + ``SolverVBD`` + thick-wall SDF tube), builds the tube along
the active planned path, and implements the existing ``PhysicsEngine`` seam so
Godot can render real Newton guidewire bodies through the current WebSocket
protocol.
"""

from __future__ import annotations

import math
import os
from typing import Sequence

import numpy as np

from services.physics.base import MAX_WALL_DISTANCE, PlannedPath, RawPose, quat_from_direction


def _parallel_frames(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    tang = np.zeros_like(points)
    tang[:-1] = points[1:] - points[:-1]
    tang[-1] = tang[-2]
    tang /= np.linalg.norm(tang, axis=1, keepdims=True) + 1e-12

    ref = np.array([0.0, 1.0, 0.0]) if abs(tang[0, 1]) < 0.9 else np.array([1.0, 0.0, 0.0])
    nrm = np.zeros_like(points)
    nrm[0] = np.cross(tang[0], ref)
    nrm[0] /= np.linalg.norm(nrm[0])
    for i in range(1, len(points)):
        v = nrm[i - 1] - tang[i] * float(np.dot(nrm[i - 1], tang[i]))
        nv = float(np.linalg.norm(v))
        nrm[i] = v / nv if nv > 1e-9 else nrm[i - 1]
    return tang, nrm, np.cross(tang, nrm)


def _resample(points: np.ndarray, ds: float) -> np.ndarray:
    points = _dedupe_consecutive(points)
    seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    samples = np.arange(0.0, cum[-1], ds)
    if len(samples) == 0 or samples[-1] < cum[-1]:
        samples = np.append(samples, cum[-1])
    out = []
    for s in samples:
        idx = int(np.searchsorted(cum, s))
        if idx <= 0:
            out.append(points[0])
        elif idx >= len(points):
            out.append(points[-1])
        else:
            t = (s - cum[idx - 1]) / max(cum[idx] - cum[idx - 1], 1e-12)
            out.append(points[idx - 1] + t * (points[idx] - points[idx - 1]))
    return _dedupe_consecutive(np.asarray(out, dtype=np.float64))


def _dedupe_consecutive(points: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    """Drop consecutive duplicate points that break Newton cable frames."""
    pts = np.asarray(points, dtype=np.float64)
    if len(pts) <= 1:
        return pts
    keep = [pts[0]]
    for point in pts[1:]:
        if float(np.linalg.norm(point - keep[-1])) > eps:
            keep.append(point)
    return np.asarray(keep, dtype=np.float64)


def _point_at_s(points: np.ndarray, s: float) -> np.ndarray:
    seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    s = float(np.clip(s, 0.0, cum[-1]))
    idx = int(np.searchsorted(cum, s))
    if idx <= 0:
        return points[0].copy()
    if idx >= len(points):
        return points[-1].copy()
    t = (s - cum[idx - 1]) / max(cum[idx] - cum[idx - 1], 1e-12)
    return points[idx - 1] + t * (points[idx] - points[idx - 1])


def _sample_along(points: np.ndarray, length: float, seg_len: float) -> np.ndarray:
    seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
    total = float(np.sum(seg))
    length = float(np.clip(length, 0.0, total))
    samples = np.arange(0.0, length + 0.5 * seg_len, seg_len)
    if len(samples) == 0 or samples[-1] < length:
        samples = np.append(samples, length)
    samples = np.clip(samples, 0.0, length)
    return _dedupe_consecutive(np.asarray([_point_at_s(points, s) for s in samples]))


def _point_to_polyline_distance(point: np.ndarray, points: np.ndarray) -> float:
    a = points[:-1]
    b = points[1:]
    ab = b - a
    ap = point - a
    denom = np.einsum("ij,ij->i", ab, ab) + 1e-18
    t = np.clip(np.einsum("ij,ij->i", ap, ab) / denom, 0.0, 1.0)
    proj = a + t[:, None] * ab
    return float(np.sqrt(np.min(np.sum((point - proj) ** 2, axis=1))))


class NewtonEngine:
    """Opt-in Newton physics demo backend.

    Enable with ``CATHSIM_PHYSICS_ENGINE=newton_demo``.  The engine is aimed at a
    front-end proof of life: short guidewire segment, real centerline, SDF wall,
    current WebSocket rendering.  It is not the final optimized Newton backend.
    """

    def __init__(
        self,
        *,
        path: PlannedPath | None,
        n_substeps: int | None = None,
        entry_point: Sequence[float] | None = None,
        entry_direction: Sequence[float] | None = None,
        **_unused,
    ) -> None:
        if path is None or path.total_len <= 0.0:
            raise ValueError("Newton demo engine requires a planned path/centerline")

        self._path = path
        self._entry_point = entry_point
        self._entry_direction = entry_direction
        self._control_dt = float(os.environ.get("CATHSIM_NEWTON_CONTROL_DT", "0.0333333333"))
        self._substeps = int(os.environ.get("CATHSIM_NEWTON_SUBSTEPS", str(n_substeps or 20)))
        self._sim_dt = self._control_dt / self._substeps
        self._rod_length = float(os.environ.get("CATHSIM_NEWTON_ROD_LENGTH", "0.06"))
        self._rod_radius = float(os.environ.get("CATHSIM_NEWTON_ROD_RADIUS", "0.0004"))
        self._rod_seg_len = float(os.environ.get("CATHSIM_NEWTON_SEG_LEN", "0.003"))
        self._lumen_radius = float(os.environ.get("CATHSIM_NEWTON_LUMEN_RADIUS", "0.0025"))
        self._wall_thickness = float(os.environ.get("CATHSIM_NEWTON_WALL_THICKNESS", "0.01"))
        self._push_speed = float(os.environ.get("CATHSIM_NEWTON_PUSH_SPEED", "0.03"))
        self._bend = float(os.environ.get("CATHSIM_NEWTON_BEND", "1.0"))
        self._contact_ke = float(os.environ.get("CATHSIM_NEWTON_CONTACT_KE", "1000000"))

        self._centerline = _resample(self._path.points, 0.002)
        self._insert_s = 0.0
        self._initialized = False
        self._model = None
        self._solver = None
        self._s0 = None
        self._s1 = None
        self._control = None
        self._contacts = None
        self._rod_bodies: list[int] = []

    def set_path(self, path: PlannedPath | None) -> None:
        """Replace the planned path and force the Newton scene to be rebuilt.

        Newton builds collision SDF and rod bodies from the centerline during
        initialization. A route switch changes that geometry, so merely swapping
        ``_path`` is not enough; the cached model/state must be discarded.
        """
        if path is None or path.total_len <= 0.0:
            raise ValueError("Newton demo engine requires a planned path/centerline")
        self.close()
        self._path = path
        self._centerline = _resample(self._path.points, 0.002)
        self._insert_s = 0.0

    @property
    def is_initialized(self) -> bool:
        return self._initialized

    @property
    def control_timestep(self) -> float:
        return self._control_dt

    def _build_tube_mesh(self, newton):
        _, nrm, binm = _parallel_frames(self._centerline)
        angles = np.linspace(0.0, 2.0 * math.pi, 24, endpoint=False)
        verts = []
        for p, n, b in zip(self._centerline, nrm, binm):
            for radius in (self._lumen_radius, self._lumen_radius + self._wall_thickness):
                for a in angles:
                    d = math.cos(a) * n + math.sin(a) * b
                    verts.append(p + radius * d)

        k_around = len(angles)
        stride = 2 * k_around

        def vi(i: int, ring: int, k: int) -> int:
            return i * stride + ring * k_around + (k % k_around)

        tris: list[int] = []
        for i in range(len(self._centerline) - 1):
            for k in range(k_around):
                kn = (k + 1) % k_around
                tris += [vi(i, 0, k), vi(i + 1, 0, k), vi(i + 1, 0, kn)]
                tris += [vi(i, 0, k), vi(i + 1, 0, kn), vi(i, 0, kn)]
                tris += [vi(i, 1, k), vi(i, 1, kn), vi(i + 1, 1, kn)]
                tris += [vi(i, 1, k), vi(i + 1, 1, kn), vi(i + 1, 1, k)]
        for i, flip in ((0, False), (len(self._centerline) - 1, True)):
            for k in range(k_around):
                kn = (k + 1) % k_around
                a, b, c, d = vi(i, 0, k), vi(i, 0, kn), vi(i, 1, kn), vi(i, 1, k)
                tris += [a, c, b, a, d, c] if flip else [a, b, c, a, c, d]

        mesh = newton.Mesh(np.asarray(verts, dtype=np.float32), np.asarray(tris, dtype=np.int32), is_solid=True)
        mesh.build_sdf(target_voxel_size=0.001, narrow_band_range=(-self._wall_thickness, self._lumen_radius))
        return mesh

    def _ensure_initialized(self) -> None:
        if self._initialized:
            return

        import newton
        import warp as wp

        wp.init()
        builder = newton.ModelBuilder()
        builder.rigid_gap = 0.0
        builder.default_shape_cfg.density = 1800.0
        builder.default_shape_cfg.ke = self._contact_ke
        builder.default_shape_cfg.kd = 1.0
        builder.default_shape_cfg.mu = 0.2

        wall_cfg = newton.ModelBuilder.ShapeConfig(ke=self._contact_ke, kd=1.0, mu=0.2)
        builder.add_shape_mesh(body=-1, mesh=self._build_tube_mesh(newton), cfg=wall_cfg, label="vessel")

        rod_pts = _sample_along(self._centerline, self._rod_length, self._rod_seg_len)
        quats = newton.utils.create_parallel_transport_cable_quaternions([wp.vec3(*p) for p in rod_pts], twist_total=0.0)
        rod_bodies, _ = builder.add_rod(
            positions=[wp.vec3(*p) for p in rod_pts],
            quaternions=quats,
            radius=self._rod_radius,
            stretch_stiffness=1.0e5,
            stretch_damping=0.0,
            bend_stiffness=self._bend,
            bend_damping=1.0e-1,
            label="guidewire",
        )

        root = rod_bodies[0]
        builder.body_mass[root] = 0.0
        builder.body_inv_mass[root] = 0.0
        builder.body_inertia[root] = wp.mat33(0.0)
        builder.body_inv_inertia[root] = wp.mat33(0.0)

        builder.color(balance_colors=False)
        self._model = builder.finalize()
        self._model.set_gravity((0.0, 0.0, 0.0))
        self._solver = newton.solvers.SolverVBD(self._model, iterations=8)
        self._s0 = self._model.state()
        self._s1 = self._model.state()
        self._control = self._model.control()
        self._contacts = self._model.contacts()
        self._rod_bodies = list(rod_bodies)
        self._initialized = True

    def reset(self) -> RawPose:
        self._initialized = False
        self._insert_s = 0.0
        self._ensure_initialized()
        return self._raw_pose()

    def step(self, push: float, rotate: float) -> RawPose:
        self._ensure_initialized()
        import numpy as _np

        direction = 1.0 if push >= 0.0 else -1.0
        prev_insert_s = self._insert_s
        self._insert_s = float(
            _np.clip(
                self._insert_s + direction * abs(push) * self._push_speed * self._control_dt,
                0.0,
                self._path.total_len,
            )
        )
        root_body = self._rod_bodies[0]

        for sub in range(self._substeps):
            frac = (sub + 1) / self._substeps
            root_pos = _point_at_s(self._centerline, prev_insert_s + (self._insert_s - prev_insert_s) * frac)
            bq = self._s0.body_q.numpy()
            bq[root_body, :3] = root_pos
            self._s0.body_q.assign(bq)
            self._s0.clear_forces()
            self._model.collide(self._s0, self._contacts)
            self._solver.step(self._s0, self._s1, self._control, self._contacts, self._sim_dt)
            self._s0, self._s1 = self._s1, self._s0

        return self._raw_pose()

    def _raw_pose(self) -> RawPose:
        q = self._s0.body_q.numpy()
        xyz = q[self._rod_bodies, :3]
        tip = xyz[-1]
        prev = xyz[-2] if len(xyz) > 1 else xyz[-1] - np.array([0.0, 0.0, 1.0])
        direction = tip - prev
        norm = float(np.linalg.norm(direction))
        if norm <= 1e-9:
            direction = np.array([0.0, 0.0, 1.0])
        else:
            direction = direction / norm

        radial = max(_point_to_polyline_distance(p, self._centerline) + self._rod_radius for p in xyz)
        wall_distance = max(0.0, self._lumen_radius - radial)
        contact_force = max(0.0, radial - self._lumen_radius) * self._contact_ke
        target = self._path.points[-1].tolist()
        done = bool(self._insert_s >= self._path.total_len)
        return RawPose(
            tip_position=[float(v) for v in tip],
            tip_direction=[float(v) for v in direction],
            tip_quaternion=quat_from_direction(direction),
            contact_force=float(contact_force),
            wall_distance=float(min(wall_distance, MAX_WALL_DISTANCE)),
            target_position=[float(v) for v in target],
            joint_positions=[],
            joint_velocities=[],
            reward=0.0,
            done=done,
            arclen=None,
        )

    def render_bodies(self) -> list[dict[str, list[float]]]:
        if not self._initialized:
            return []
        q = self._s0.body_q.numpy()
        bodies = []
        for body in self._rod_bodies:
            pos = [float(v) for v in q[body, :3]]
            quat = [float(v) for v in q[body, 3:7]]
            bodies.append({"pos": pos, "quat": quat})
        return bodies

    def close(self) -> None:
        self._model = None
        self._solver = None
        self._s0 = None
        self._s1 = None
        self._control = None
        self._contacts = None
        self._rod_bodies = []
        self._initialized = False
