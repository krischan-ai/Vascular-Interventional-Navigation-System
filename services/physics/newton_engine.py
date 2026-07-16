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

from services.devices import CoaxialSupportSystem, GuidewireProfile
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


def _nearest_arclen(points: np.ndarray, cum: np.ndarray, xyz: np.ndarray) -> float:
    """Arc length of the nearest point on the polyline to ``xyz`` (scalar)."""
    a, b = points[:-1], points[1:]
    ab = b - a
    denom = np.einsum("ij,ij->i", ab, ab) + 1e-18
    ap = xyz - a
    t = np.clip(np.einsum("ij,ij->i", ap, ab) / denom, 0.0, 1.0)
    proj = a + t[:, None] * ab
    d2 = np.sum((xyz - proj) ** 2, axis=1)
    i = int(np.argmin(d2))
    return float(cum[i] + t[i] * (cum[i + 1] - cum[i]))


def _feed_budget(requested: float, slack: float, max_slack: float) -> float:
    """Forward-feed advance allowed under the prolapse guard (force mode).

    ``slack`` is how much fed wire has NOT translated into tip advance along the
    route (fed arclen - actual tip arclen). Once it reaches ``max_slack`` the
    budget is exhausted -- pushing harder would only pile wire into a buckle
    (缠绕), so the feed stops until the tip catches up or the wire is pulled
    back. Backward feed (requested<=0) and a disabled guard (max_slack<=0) pass
    through untouched.
    """
    if requested <= 0.0 or max_slack <= 0.0:
        return requested
    return min(requested, max(0.0, max_slack - slack))


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product of two [x, y, z, w] quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def _quat_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    """[x, y, z, w] quaternion of a rotation ``angle`` (rad) about unit-ish ``axis``."""
    n = float(np.linalg.norm(axis))
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    u = axis / n
    s = math.sin(angle / 2.0)
    return np.array([u[0] * s, u[1] * s, u[2] * s, math.cos(angle / 2.0)])


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
        # D3 (doc/08) validated config: variable-radius tube wall + graded soft-anchor
        # drive at 6 substeps x 2 VBD iterations ~= 60fps, contained on aorta_tree.
        self._substeps = int(os.environ.get("CATHSIM_NEWTON_SUBSTEPS", str(n_substeps or 6)))
        self._iterations = int(os.environ.get("CATHSIM_NEWTON_ITERS", "2"))
        self._sim_dt = self._control_dt / self._substeps
        self._rod_length = float(os.environ.get("CATHSIM_NEWTON_ROD_LENGTH", "0.06"))
        self._rod_radius = float(os.environ.get("CATHSIM_NEWTON_ROD_RADIUS", "0.0004"))
        self._rod_seg_len = float(os.environ.get("CATHSIM_NEWTON_SEG_LEN", "0.003"))
        # Fallback lumen radius when a route ships no per-point radii.
        self._lumen_radius = float(os.environ.get("CATHSIM_NEWTON_LUMEN_RADIUS", "0.0025"))
        self._min_radius = float(os.environ.get("CATHSIM_NEWTON_MIN_RADIUS", "0.0015"))
        self._wall_thickness = float(os.environ.get("CATHSIM_NEWTON_WALL_THICKNESS", "0.006"))
        self._sdf_voxel = float(os.environ.get("CATHSIM_NEWTON_SDF_VOXEL", "0.0005"))
        self._lumen_band = float(os.environ.get("CATHSIM_NEWTON_LUMEN_BAND", "0.015"))
        self._push_speed = float(os.environ.get("CATHSIM_NEWTON_PUSH_SPEED", "0.05"))
        # D4 drive model: "force" = real force transmission (proximal sheath fed,
        # distal free physics, push carried by the cable stretch constraint,
        # `rotate` twists the J-tip); "anchor" = D3 graded soft-anchor fallback.
        self._drive = os.environ.get("CATHSIM_NEWTON_DRIVE", "force").strip().lower()
        _force = self._drive == "force"
        # Conforming shaft (bend~20) hugs vessel curvature and stays contained;
        # a stiff shaft (bend~50, the D3 anchor default) chords through outer walls
        # on sharp bends. See spikes/D4_RESULTS.md.
        self._bend = float(os.environ.get("CATHSIM_NEWTON_BEND", "20.0" if _force else "50.0"))
        # Force drive needs a near-inextensible rod so the push transmits instead
        # of the stretch constraint being shocked into 80-240% strain (D4a finding).
        self._stretch = float(os.environ.get("CATHSIM_NEWTON_STRETCH", "3.0e7" if _force else "1.0e5"))
        self._contact_ke = float(os.environ.get("CATHSIM_NEWTON_CONTACT_KE", "3000000" if _force else "1000000"))
        # 软头硬身: distal `soft_tip` joints ramp from `bend` down to `tip_bend`.
        self._soft_tip = int(os.environ.get("CATHSIM_NEWTON_SOFT_TIP", "10" if _force else "0"))
        self._tip_bend = float(os.environ.get("CATHSIM_NEWTON_TIP_BEND", "2.0"))
        # Force drive: proximal bodies glued to the centered route (the sheath/
        # introducer + already-traversed wire) and fed forward; everything distal
        # is free physics. sheath_bodies=-1 (default) resolves automatically so
        # only the distal ``free_len`` of wire is free -- a long unsupported
        # column buckles into loops under blocked push (doc/08 §9.1 sheath
        # constraint / §28.9 抗屈曲). Explicit >0 forces that exact glue count
        # (sheath_bodies=1 reproduces the original root-only D4 behavior).
        self._sheath_bodies = int(os.environ.get("CATHSIM_NEWTON_SHEATH_BODIES", "-1"))
        # Distal wire length (m) left free beyond the sheath in auto mode.
        self._free_len = float(os.environ.get("CATHSIM_NEWTON_FREE_LEN", "0.03"))
        # Prolapse guard (force mode): stop feeding wire when the tip stops
        # advancing. slack = fed arclen - actual tip arclen along the route; once
        # it exceeds max_slack the forward feed budget is exhausted and holding
        # W no longer piles wire into a buckle. Pull-back always allowed (it is
        # the recovery move). <=0 disables the guard.
        self._max_slack = float(os.environ.get("CATHSIM_NEWTON_MAX_SLACK", "0.012"))
        # Torsion steering: rotate input integrated (rad/s at rotate=1) into a
        # twist applied to the root anchor frame about the local tangent (D4c).
        self._rotate_speed = float(os.environ.get("CATHSIM_NEWTON_ROTATE_SPEED", "3.0"))
        self._support_speed = float(os.environ.get("CATHSIM_NEWTON_SUPPORT_SPEED", "0.03"))
        # Pre-bent J-tip rest shape: the distal `jtip_bodies` segments curve off
        # the tangent by `jtip_deg` total, giving `rotate` real branch-selecting
        # authority at sharp bends (incidental curvature alone is too weak). 0 deg
        # = straight tip (behaves like the pre-J-tip build).
        self._jtip_deg = float(os.environ.get("CATHSIM_NEWTON_JTIP_DEG", "35.0" if _force else "0.0"))
        self._jtip_bodies = int(os.environ.get("CATHSIM_NEWTON_JTIP_BODIES", "3"))
        self._tip_shape = os.environ.get(
            "CATHSIM_NEWTON_TIP_SHAPE",
            "j_tip" if self._jtip_deg > 0.0 and self._jtip_bodies > 0 else "straight",
        ).strip().lower()
        self._settle_steps = int(os.environ.get("CATHSIM_NEWTON_SETTLE_STEPS", "20"))
        self._twist = 0.0
        # Graded soft-anchor (anchor mode): proximal glued, distal ramp of
        # `free_span` bodies kept soft (tip_alpha) so the tip deflects.
        self._free_span = int(os.environ.get("CATHSIM_NEWTON_FREE_SPAN", "6"))
        self._tip_alpha = float(os.environ.get("CATHSIM_NEWTON_TIP_ALPHA", "0.3"))

        self._centerline = _resample(self._path.points, 0.002)
        self._radii = self._centerline_radii()
        self._insert_s = 0.0
        self._initialized = False
        self._model = None
        self._solver = None
        self._s0 = None
        self._s1 = None
        self._control = None
        self._contacts = None
        self._rod_bodies: list[int] = []
        self._alpha = None
        self._base_arc = None
        self._max_s_ins = 0.0
        self._guidewire_profile = self._make_guidewire_profile()
        self._support_system = self._make_support_system()

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
        self._radii = self._centerline_radii()
        self._insert_s = 0.0

    def _centerline_radii(self) -> np.ndarray:
        """Per-node lumen radius along the resampled centerline.

        Uses the route's real per-point radii (VMTK inscribed radius) when the
        path carries them, interpolated by arc length onto ``self._centerline``;
        falls back to a constant lumen radius otherwise.
        """
        n = len(self._centerline)
        path_radii = getattr(self._path, "radii", None)
        if path_radii is None:
            return np.full(n, self._lumen_radius, dtype=np.float64)
        seg = np.linalg.norm(np.diff(self._centerline, axis=0), axis=1)
        cl_cum = np.concatenate([[0.0], np.cumsum(seg)])
        radii = np.interp(cl_cum, self._path.cumlen, np.asarray(path_radii, dtype=np.float64))
        return np.maximum(radii, self._min_radius)

    def _make_guidewire_profile(self) -> GuidewireProfile:
        return GuidewireProfile.default(
            total_length_mm=self._rod_length * 1000.0,
            tip_shape=self._tip_shape,
            radius_mm=self._rod_radius * 1000.0,
            contact_ke=self._contact_ke,
        )

    def _make_support_system(self) -> CoaxialSupportSystem:
        return CoaxialSupportSystem.default(
            total_length_mm=self._rod_length * 1000.0,
            free_wire_length_mm=self._free_len * 1000.0,
        )

    def _refresh_device_models(self) -> None:
        self._guidewire_profile = self._make_guidewire_profile()
        self._support_system = self._make_support_system()

    @property
    def is_initialized(self) -> bool:
        return self._initialized

    @property
    def control_timestep(self) -> float:
        return self._control_dt

    @property
    def is_force_drive(self) -> bool:
        """True when driving via real force transmission (D4), not the D3 anchor.

        In force mode the wire legitimately rides against the lumen wall, so the
        orchestrator must judge safety on penetration (``contact_force``) rather
        than on raw wall clearance (see ``NavigationEngine._compute_safety_status``).
        """
        return self._drive == "force"

    @property
    def contact_ke(self) -> float:
        """Wall contact stiffness (N/m); ``contact_force = penetration_m * ke``."""
        return self._contact_ke

    def _build_tube_mesh(self, newton):
        """Variable-radius thick-wall annulus tube swept along the centerline.

        The lumen (inner ring at the local radius) is the free/positive SDF region
        and the wall (out to +wall_thickness) is solid; a single lumen surface would
        instead make the lumen solid and expel the rod (doc/08 D2 finding). The
        ``lumen_band`` must cover the widest local radius or a wide lumen's tip
        tunnels the SDF's central no-band region.
        """
        _, nrm, binm = _parallel_frames(self._centerline)
        angles = np.linspace(0.0, 2.0 * math.pi, 24, endpoint=False)
        verts = []
        for p, r, n, b in zip(self._centerline, self._radii, nrm, binm):
            for radius in (r, r + self._wall_thickness):
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
        mesh.build_sdf(
            target_voxel_size=self._sdf_voxel,
            narrow_band_range=(-self._wall_thickness, self._lumen_band),
        )
        return mesh

    def _apply_jtip(self, rod_pts: np.ndarray) -> np.ndarray:
        """Curve the distal `jtip_bodies` segments into a pre-bent J-tip rest shape.

        The straight-sampled tip is rebuilt as an arc bending `jtip_deg` off the
        local tangent (in an arbitrary plane -- torsion rotates it at runtime).
        The baked curvature is held by bend stiffness, so `rotate` twisting the
        shaft reorients this J to select branches (D4c steering authority).
        """
        if self._jtip_deg <= 0.0 or self._jtip_bodies <= 0 or len(rod_pts) < self._jtip_bodies + 2:
            return rod_pts
        pts = np.asarray(rod_pts, dtype=np.float64).copy()
        n = len(pts)
        k = min(self._jtip_bodies, n - 2)
        i0 = n - 1 - k  # last shaft node = base of the J
        tang = pts[i0] - pts[i0 - 1]
        tn = float(np.linalg.norm(tang))
        tang = tang / tn if tn > 1e-9 else np.array([0.0, 0.0, 1.0])
        ref = np.array([0.0, 1.0, 0.0]) if abs(tang[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
        axis = np.cross(tang, ref)
        axis /= np.linalg.norm(axis) + 1e-12
        dphi = math.radians(self._jtip_deg) / k
        c, s = math.cos(dphi), math.sin(dphi)
        d = tang.copy()
        p = pts[i0].copy()
        for j in range(1, k + 1):
            # Rodrigues rotation of the heading about `axis` by dphi per segment.
            d = d * c + np.cross(axis, d) * s + axis * float(np.dot(axis, d)) * (1.0 - c)
            p = p + self._rod_seg_len * d
            pts[i0 + j] = p
        return pts

    def _apply_joint_stiffness(self) -> None:
        """Write stretch/bend/soft-tip into ``joint_target_ke`` (no rebuild).

        Cable-joint DOF layout is ``[stretch, bend]`` per joint: even indices are
        stretch, odd are bend. The distal ``soft_tip`` joints ramp from ``bend``
        down to ``tip_bend`` (软头硬身). Reusable so the live parameter panel can
        re-tune shaft/tip stiffness on the fly.
        """
        if self._model is None or not self._rod_bodies:
            return
        ke = self._model.joint_target_ke.numpy()
        nj = len(self._rod_bodies) - 1
        bend_profile = self._bend_profile(nj)
        for j in range(nj):
            ke[2 * j] = self._stretch
            ke[2 * j + 1] = bend_profile[j]
        self._model.joint_target_ke.assign(ke)

    def _bend_profile(self, joint_count: int) -> np.ndarray:
        """Per-joint bend stiffness from proximal shaft to distal soft tip.

        This is the D4-R soft-tip/hard-shaft calibration surface. The shaft uses
        ``bend``; the distal ``soft_tip`` joints ramp down to ``tip_bend`` so
        push is transmitted by stretch constraints while the tip remains able to
        conform and rebound at the wall. If the rod has fewer joints than the
        configured soft-tip span, the available joints still cover the full ramp
        and the last joint reaches ``tip_bend``.
        """
        count = max(0, int(joint_count))
        profile = np.asarray(
            self._guidewire_profile.joint_bend_profile(count, self._bend),
            dtype=np.float64,
        )
        tip_count = min(max(0, int(self._soft_tip)), count)
        if tip_count <= 0:
            return profile
        start = count - tip_count
        for offset in range(tip_count):
            frac = (offset + 1) / tip_count
            profile[start + offset] = min(
                profile[start + offset],
                self._bend * (1.0 - frac) + self._tip_bend * frac,
            )
        return profile

    # Deformation params that live-update (no scene rebuild) vs. those baked into
    # the model geometry at build time (rebuild required).
    _LIVE_PARAMS = (
        "bend", "tip_bend", "soft_tip", "stretch", "push_speed", "rotate_speed",
        "sheath_bodies", "free_len", "max_slack", "support_speed",
    )
    _REBUILD_PARAMS = ("jtip_deg", "jtip_bodies", "contact_ke", "wall_thickness", "rod_length", "tip_shape")
    # Live params that change the glue mask (recomputed in place, no rebuild).
    _GLUE_PARAMS = ("sheath_bodies", "free_len")

    def set_deform_params(self, **params) -> dict:
        """Live-tune guidewire deformation for the interactive parameter panel.

        Accepts any of: ``bend``, ``tip_bend``, ``soft_tip``, ``stretch``,
        ``push_speed``, ``rotate_speed`` (applied immediately via joint stiffness /
        drive gains), and ``jtip_deg``/``jtip_bodies``/``contact_ke`` (geometry,
        which force a scene rebuild on the next step). Returns the effective
        deform-param state so the client HUD can reflect clamps.
        """
        rebuild = False
        reglue = False
        for key, val in params.items():
            if key == "tip_shape":
                self._tip_shape = str(val).strip().lower()
                rebuild = True
                continue
            attr = f"_{key}"
            if not hasattr(self, attr):
                continue
            setattr(self, attr, type(getattr(self, attr))(val))
            if key in self._REBUILD_PARAMS:
                rebuild = True
            if key in self._GLUE_PARAMS:
                reglue = True
        if "tip_shape" not in params and ("jtip_deg" in params or "jtip_bodies" in params):
            self._tip_shape = "j_tip" if self._jtip_deg > 0.0 and self._jtip_bodies > 0 else "straight"
        self._refresh_device_models()
        if rebuild:
            # Geometry changed: drop the cached model so the next step rebuilds
            # it (and re-settles) with the new shape, preserving insertion depth.
            self.close()
        elif self._initialized:
            self._apply_joint_stiffness()
            if reglue:
                # Sheath length changed: refresh the glue mask in place.
                self._alpha = self._glue_alpha(len(self._rod_bodies))
        return self.deform_params

    @property
    def deform_params(self) -> dict:
        tip_shape = self._tip_shape if self._jtip_deg > 0.0 and self._jtip_bodies > 0 else "straight"
        return {
            "tip_shape": tip_shape,
            "profile_name": self._guidewire_profile.name,
            "bend": self._bend,
            "tip_bend": self._tip_bend,
            "soft_tip": self._soft_tip,
            "stretch": self._stretch,
            "push_speed": self._push_speed,
            "rotate_speed": self._rotate_speed,
            "jtip_deg": self._jtip_deg,
            "jtip_bodies": self._jtip_bodies,
            "contact_ke": self._contact_ke,
            "sheath_bodies": self._sheath_bodies,
            "free_len": self._free_len,
            "max_slack": self._max_slack,
            "support_speed": self._support_speed,
            "segments": [
                {
                    "name": segment.name,
                    "start_ratio": segment.start_ratio,
                    "end_ratio": segment.end_ratio,
                    "bend_stiffness": segment.bend_stiffness,
                    "torsion_stiffness": segment.torsion_stiffness,
                    "preferred_spacing_mm": segment.preferred_spacing_mm,
                }
                for segment in self._guidewire_profile.segments
            ],
            "bend_profile": self._bend_profile(max(0, int(round(self._rod_length / self._rod_seg_len)))).tolist(),
        }

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
        rod_pts = self._apply_jtip(rod_pts)
        quats = newton.utils.create_parallel_transport_cable_quaternions([wp.vec3(*p) for p in rod_pts], twist_total=0.0)
        rod_bodies, rod_joints = builder.add_rod(
            positions=[wp.vec3(*p) for p in rod_pts],
            quaternions=quats,
            radius=self._rod_radius,
            stretch_stiffness=self._stretch,
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
        self._rod_bodies = list(rod_bodies)
        self._apply_joint_stiffness()

        self._solver = newton.solvers.SolverVBD(self._model, iterations=self._iterations)
        self._s0 = self._model.state()
        self._s1 = self._model.state()
        self._control = self._model.control()
        self._contacts = self._model.contacts()

        nb = len(self._rod_bodies)
        self._base_arc = np.arange(nb) * self._rod_seg_len
        self._alpha = self._glue_alpha(nb)
        seg = np.linalg.norm(np.diff(self._centerline, axis=0), axis=1)
        self._cl_cum = np.concatenate([[0.0], np.cumsum(seg)])
        cl_seg = float(self._cl_cum[-1])
        self._max_s_ins = max(0.0, cl_seg - float(self._base_arc[-1]) - 1e-3)
        self._twist = 0.0
        self._initialized = True

        # Let the free rod relax into the lumen before driving (force mode), so
        # the first push does not fight an off-lumen initial pose.
        if self._drive == "force" and self._settle_steps > 0:
            self._settle(self._settle_steps)

    def reset(self) -> RawPose:
        self._initialized = False
        self._insert_s = 0.0
        self._twist = 0.0
        self._ensure_initialized()
        return self._raw_pose()

    def _tangent_at_s(self, s: float) -> np.ndarray:
        eps = max(self._rod_seg_len, 1e-4)
        d = _point_at_s(self._centerline, s + eps) - _point_at_s(self._centerline, s - eps)
        n = float(np.linalg.norm(d))
        return d / n if n > 1e-9 else np.array([0.0, 0.0, 1.0])

    def _sheath_count(self, nb: int) -> int:
        """Number of proximal bodies glued to the route (force mode).

        Explicit ``sheath_bodies>0`` wins; otherwise (auto, -1) support all but
        the distal ``free_len`` of wire, so the unsupported column stays short
        enough not to buckle under normal push. Always in [1, nb].
        """
        if self._sheath_bodies > 0:
            n = self._sheath_bodies
        else:
            free_bodies = int(round(self._free_len / self._rod_seg_len))
            n = nb - free_bodies
        return int(np.clip(n, 1, nb))

    def _glue_alpha(self, nb: int) -> np.ndarray:
        """Glue mask: which bodies are kinematically fed along the centered route.

        force (D4): the proximal sheath (see ``_sheath_count``) glued, rest free
        physics. anchor (D3): proximal glued + distal soft ramp.
        """
        alpha = np.zeros(nb)
        if self._drive == "force":
            alpha[: self._sheath_count(nb)] = 1.0
        else:
            alpha[:] = 1.0
            for i in range(self._free_span):
                j = nb - 1 - i
                if j >= 0:
                    frac = (i + 1) / self._free_span
                    alpha[j] = max(self._tip_alpha, 1.0 - frac * (1.0 - self._tip_alpha))
        return alpha

    def _apply_glue(self, s_ins: float) -> None:
        """Feed the glued (sheath/proximal) bodies along the centered route and, in
        force mode, apply the accumulated torsion to the driven root frame.

        Force mode glues only the proximal ``sheath_bodies`` and twists the root
        anchor about the local tangent (D4c steering); the free distal length and
        tip emerge from physics. Anchor mode glues per the graded soft-anchor mask.
        """
        rb = np.asarray(self._rod_bodies)
        glued = self._alpha > 0.0
        if not glued.any():
            return
        alpha = self._alpha[glued][:, None]
        target = np.asarray([_point_at_s(self._centerline, a + s_ins) for a in self._base_arc[glued]])
        bq = self._s0.body_q.numpy()
        bq[rb[glued], :3] = (1.0 - alpha) * bq[rb[glued], :3] + alpha * target
        if self._drive == "force":
            tangent = self._tangent_at_s(s_ins)
            base_q = np.asarray(quat_from_direction(tangent), dtype=np.float64)
            root_q = _quat_mul(_quat_axis_angle(tangent, self._twist), base_q)
            bq[rb[0], 3:7] = root_q
        self._s0.body_q.assign(bq)

    def _settle(self, steps: int) -> None:
        for _ in range(steps):
            self._s0.clear_forces()
            self._model.collide(self._s0, self._contacts)
            self._solver.step(self._s0, self._s1, self._control, self._contacts, self._sim_dt)
            self._s0, self._s1 = self._s1, self._s0
            self._apply_glue(self._insert_s)

    def _tip_arclen(self) -> float:
        """Actual tip arc length along the route from the current sim state."""
        tip = self._s0.body_q.numpy()[self._rod_bodies[-1], :3]
        return float(_nearest_arclen(self._centerline, self._cl_cum, tip))

    def _breach_stats(self, xyz: np.ndarray) -> tuple[float, float, float]:
        """Worst lumen breach plus wall-distance/contact-force for rod bodies."""
        d2 = np.sum((xyz[:, None, :] - self._centerline[None, :, :]) ** 2, axis=2)
        nearest = np.argmin(d2, axis=1)
        dist = np.sqrt(d2[np.arange(len(xyz)), nearest])
        breach = dist + self._rod_radius - self._radii[nearest]  # >0 => outside lumen
        worst = float(breach.max())
        wall_distance = max(0.0, -worst)
        contact_force = max(0.0, worst) * self._contact_ke
        return worst, wall_distance, contact_force

    def _wall_slide_metrics(self, xyz: np.ndarray) -> dict:
        """Classify near-wall motion as tangential slide vs. normal poking.

        Scores are source-backed by current rod geometry and the real route
        radius. ``normal_poking_score`` rises when the distal local wire tangent
        points outward into the nearest wall normal; ``tangential_slide_score``
        rises when the tangent is mostly parallel to the wall surface.
        """
        if len(xyz) == 0:
            return {
                "normal_poking_score": None,
                "tangential_slide_score": None,
                "wall_contact_body_index": None,
                "wall_contact_arclen_m": None,
                "wall_normal": None,
                "wall_tangent_alignment": None,
            }

        d2 = np.sum((xyz[:, None, :] - self._centerline[None, :, :]) ** 2, axis=2)
        nearest = np.argmin(d2, axis=1)
        dist = np.sqrt(d2[np.arange(len(xyz)), nearest])
        breach = dist + self._rod_radius - self._radii[nearest]
        contact_index = int(np.argmax(breach))
        center = self._centerline[int(nearest[contact_index])]
        radial = xyz[contact_index] - center
        radial_norm = float(np.linalg.norm(radial))
        normal = radial / radial_norm if radial_norm > 1e-9 else np.zeros(3, dtype=np.float64)

        if len(xyz) >= 2:
            if contact_index >= len(xyz) - 1:
                tangent = xyz[-1] - xyz[-2]
            elif contact_index <= 0:
                tangent = xyz[1] - xyz[0]
            else:
                tangent = xyz[contact_index + 1] - xyz[contact_index - 1]
        else:
            tangent = np.zeros(3, dtype=np.float64)
        tangent_norm = float(np.linalg.norm(tangent))
        if tangent_norm > 1e-9:
            tangent = tangent / tangent_norm
            outward = float(np.clip(np.dot(tangent, normal), -1.0, 1.0))
            normal_poking = max(0.0, outward)
            tangential_slide = float(np.sqrt(max(0.0, 1.0 - outward * outward)))
            tangent_alignment = abs(outward)
        else:
            normal_poking = 0.0
            tangential_slide = 0.0
            tangent_alignment = None

        seg = np.linalg.norm(np.diff(self._centerline, axis=0), axis=1)
        cl_cum = np.concatenate([[0.0], np.cumsum(seg)])
        arclen = float(cl_cum[int(nearest[contact_index])])
        near_wall = bool(float(breach[contact_index]) >= -0.0015)
        return {
            "normal_poking_score": float(normal_poking) if near_wall else 0.0,
            "tangential_slide_score": float(tangential_slide) if near_wall else 0.0,
            "wall_contact_body_index": contact_index,
            "wall_contact_arclen_m": arclen,
            "wall_normal": [float(v) for v in normal],
            "wall_tangent_alignment": None if tangent_alignment is None else float(tangent_alignment),
        }

    def _tip_orientation_metrics(self, xyz: np.ndarray) -> dict:
        """Distal tip orientation in the local vessel frame.

        ``proximal_rotation_deg`` is the commanded/root twist. The distal angle is
        derived from the final rod segment projected into the normal plane of the
        nearest centerline tangent. Their wrapped difference is the observable
        torsion lag used by the orientation HUD.
        """
        if len(xyz) < 2:
            return {
                "distal_tip_rotation_deg": None,
                "torsion_lag_deg": None,
                "tip_deflection_score": None,
            }

        tip = xyz[-1]
        d2 = np.sum((self._centerline - tip) ** 2, axis=1)
        idx = int(np.argmin(d2))
        if len(self._centerline) >= 2:
            if idx <= 0:
                tangent = self._centerline[1] - self._centerline[0]
            elif idx >= len(self._centerline) - 1:
                tangent = self._centerline[-1] - self._centerline[-2]
            else:
                tangent = self._centerline[idx + 1] - self._centerline[idx - 1]
        else:
            tangent = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        tangent = tangent / (np.linalg.norm(tangent) + 1e-12)

        radial = tip - self._centerline[idx]
        normal = radial - tangent * float(np.dot(radial, tangent))
        if float(np.linalg.norm(normal)) <= 1e-9:
            ref = np.array([0.0, 1.0, 0.0]) if abs(tangent[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
            normal = ref - tangent * float(np.dot(ref, tangent))
        normal = normal / (np.linalg.norm(normal) + 1e-12)
        binormal = np.cross(tangent, normal)
        binormal = binormal / (np.linalg.norm(binormal) + 1e-12)

        tip_axis = xyz[-1] - xyz[-2]
        tip_axis = tip_axis / (np.linalg.norm(tip_axis) + 1e-12)
        projected = tip_axis - tangent * float(np.dot(tip_axis, tangent))
        tip_deflection = float(np.linalg.norm(projected))
        if tip_deflection <= 1e-9:
            distal_deg = None
            lag_deg = None
        else:
            projected = projected / tip_deflection
            distal_deg = math.degrees(math.atan2(float(np.dot(projected, binormal)), float(np.dot(projected, normal))))
            proximal_deg = math.degrees(self._twist)
            lag_deg = ((proximal_deg - distal_deg + 180.0) % 360.0) - 180.0

        return {
            "distal_tip_rotation_deg": None if distal_deg is None else float(distal_deg),
            "torsion_lag_deg": None if lag_deg is None else float(lag_deg),
            "tip_deflection_score": float(np.clip(tip_deflection, 0.0, 1.0)),
        }

    def diagnostics(self) -> dict:
        """Human-facing Newton debug metrics streamed to the front end.

        These are not part of the physics contract; they explain why a VPP force
        session behaves a certain way (blocked by slack guard, penetrating the
        lumen wall, or simply still feeding through the sheath).
        """
        body_count = len(self._rod_bodies)
        out = {
            "initialized": self._initialized,
            "drive": self._drive,
            "substeps": self._substeps,
            "iterations": self._iterations,
            "rod_length_m": self._rod_length,
            "rod_seg_len_m": self._rod_seg_len,
            "rod_radius_m": self._rod_radius,
            "body_count": body_count,
            "insert_s_m": self._insert_s,
            "max_insert_s_m": self._max_s_ins,
            "free_len_m": self._free_len,
            "max_slack_m": self._max_slack,
            "sheath_bodies": self._sheath_bodies,
            "contact_ke": self._contact_ke,
        }
        if body_count > 0:
            out["sheath_bodies_resolved"] = self._sheath_count(body_count)
        if not self._initialized or self._base_arc is None or self._s0 is None:
            return out

        xyz = self._s0.body_q.numpy()[self._rod_bodies, :3]
        tip_arclen = self._tip_arclen()
        fed_arclen = self._insert_s + float(self._base_arc[-1])
        slack = fed_arclen - tip_arclen
        worst, wall_distance, contact_force = self._breach_stats(xyz)
        out.update({
            "tip_arclen_m": tip_arclen,
            "fed_arclen_m": fed_arclen,
            "slack_m": slack,
            "feed_budget_m": (
                max(0.0, self._max_slack - slack)
                if self._drive == "force" and self._max_slack > 0.0
                else float("inf")
            ),
            "max_breach_m": worst,
            "wall_distance_m": min(wall_distance, MAX_WALL_DISTANCE),
            "contact_force": contact_force,
        })
        return out

    def apply_support_control(self, microcatheter_advance: float) -> dict:
        """Advance/retract the effective proximal support point.

        Positive input means the microcatheter/support follows the wire forward,
        shortening the distal free span. Negative input retracts support and
        lengthens the free span. The value is a normalized coefficient in
        [-1, 1], converted to meters per control tick by ``support_speed``.
        """
        command = float(np.clip(microcatheter_advance, -1.0, 1.0))
        if abs(command) <= 1e-9:
            return self.mechanics_state()
        delta = command * self._support_speed * self._control_dt
        min_free = max(self._rod_seg_len * max(1, self._jtip_bodies + 1), self._rod_seg_len)
        max_free = max(min_free, self._rod_length)
        # Support advance moves the support tip distally, so the unsupported
        # free span shrinks; retracting support grows it again.
        self._free_len = float(np.clip(self._free_len - delta, min_free, max_free))
        self._support_system = self._make_support_system()
        if self._initialized and self._rod_bodies:
            self._alpha = self._glue_alpha(len(self._rod_bodies))
        return self.mechanics_state()

    def mechanics_state(self) -> dict:
        """Source-backed guidewire mechanics for clinical workflow guidance.

        Unlike ``diagnostics`` this is a stable, semantic payload intended for
        ``state_batch`` / HUD consumers. It deliberately exposes only values this
        engine actually models today: pre-bent J-tip configuration, proximal
        support/free-span state, slack budget, and wall breach/contact state.
        """
        body_count = len(self._rod_bodies)
        estimated_body_count = max(1, int(round(self._rod_length / self._rod_seg_len)) + 1)
        support_count = self._sheath_count(body_count or estimated_body_count)
        total_wire_length_m = (
            float(self._base_arc[-1])
            if self._base_arc is not None and len(self._base_arc) > 0
            else self._rod_length
        )
        free_wire_length_m = float(np.clip(self._free_len, 0.0, total_wire_length_m))
        supported_length_m = max(0.0, total_wire_length_m - free_wire_length_m)
        self._support_system = self._make_support_system()
        effective_tube = self._support_system.effective_support_tube()
        support_ratio = self._support_system.support_ratio(total_wire_length_m * 1000.0)

        tip_shape = self._tip_shape if self._jtip_deg > 0.0 and self._jtip_bodies > 0 else "straight"
        segment_names = self._guidewire_profile.body_segment_names(body_count or estimated_body_count)
        guidewire = {
            "profile_name": self._guidewire_profile.name,
            "tip_shape": tip_shape,
            "shape_type": tip_shape,
            "curve_angle_deg": float(self._jtip_deg if tip_shape != "straight" else 0.0),
            "tip_length_m": float(max(0, self._jtip_bodies) * self._rod_seg_len),
            "current_tip_segment": segment_names[-1] if segment_names else "unknown",
            "body_segments": segment_names,
            "segment_count": len(self._guidewire_profile.segments),
            "soft_tip_joint_count": int(self._soft_tip),
            "tip_bend_stiffness": float(self._tip_bend),
            "shaft_bend_stiffness": float(self._bend),
            "proximal_rotation_deg": float(math.degrees(self._twist)),
            "distal_tip_rotation_deg": None,
            "torsion_lag_deg": None,
            "tip_deflection_score": None,
            "tip_facing_score": None,
        }
        support = {
            "support_state": "modeled",
            "effective_support_type": (
                effective_tube.tube_type if effective_tube is not None else
                ("graded_anchor" if self._drive != "force" else "none")
            ),
            "sheath_bodies": int(self._sheath_bodies),
            "sheath_bodies_resolved": int(support_count),
            "effective_support_tip_m": float(self._insert_s + supported_length_m),
            "free_wire_length_m": float(free_wire_length_m),
            "support_ratio": float(np.clip(support_ratio, 0.0, 1.0)),
            "max_slack_m": float(self._max_slack),
            "coaxial_support": [
                {
                    "name": tube.name,
                    "tube_type": tube.tube_type,
                    "tip_arclen_m": tube.tip_arclen_mm / 1000.0,
                    "is_active": tube.is_active,
                }
                for tube in self._support_system.active_tubes()
            ],
        }
        risk = {
            "slack_m": None,
            "feed_budget_m": None,
            "max_breach_m": None,
            "contact_force": None,
            "wall_distance_m": None,
            "normal_poking_score": None,
            "tangential_slide_score": None,
            "wall_contact_body_index": None,
            "wall_contact_arclen_m": None,
            "wall_normal": None,
            "wall_tangent_alignment": None,
            "wall_slide_state": "unknown",
            "buckling_risk": "unknown",
        }

        if self._initialized and self._base_arc is not None and self._s0 is not None:
            xyz = self._s0.body_q.numpy()[self._rod_bodies, :3]
            orientation_metrics = self._tip_orientation_metrics(xyz)
            tip_arclen = self._tip_arclen()
            fed_arclen = self._insert_s + float(self._base_arc[-1])
            slack = fed_arclen - tip_arclen
            worst, wall_distance, contact_force = self._breach_stats(xyz)
            wall_metrics = self._wall_slide_metrics(xyz)
            normal_poking = float(wall_metrics.get("normal_poking_score") or 0.0)
            tangential_slide = float(wall_metrics.get("tangential_slide_score") or 0.0)
            near_wall = contact_force > 0.0 or wall_distance < 0.0015
            budget = (
                max(0.0, self._max_slack - slack)
                if self._drive == "force" and self._max_slack > 0.0
                else float("inf")
            )
            if worst >= 0.0003:
                wall_slide_state = "BREACH_STOP"
            elif near_wall and normal_poking >= 0.7:
                wall_slide_state = "TIP_POKING_WARNING"
            elif contact_force > 0.0 and slack > max(0.001, self._max_slack * 0.5):
                wall_slide_state = "TIP_POKING_WARNING"
            elif near_wall and tangential_slide >= 0.7:
                wall_slide_state = "WALL_SLIDE_OK"
            elif near_wall:
                wall_slide_state = "WALL_CONTACT_MONITOR"
            else:
                wall_slide_state = "CLEAR"

            if self._max_slack > 0.0 and slack >= self._max_slack:
                buckling_risk = "HIGH"
            elif self._max_slack > 0.0 and slack >= self._max_slack * 0.5:
                buckling_risk = "MEDIUM"
            else:
                buckling_risk = "LOW"

            risk.update({
                "slack_m": float(slack),
                "feed_budget_m": float(budget),
                "max_breach_m": float(worst),
                "contact_force": float(contact_force),
                "wall_distance_m": float(min(wall_distance, MAX_WALL_DISTANCE)),
                **wall_metrics,
                "wall_slide_state": wall_slide_state,
                "buckling_risk": buckling_risk,
            })
            guidewire.update(orientation_metrics)

        return {
            "source": "newton_engine.mechanics_state",
            "source_fields": [
                "jtip_deg",
                "jtip_bodies",
                "soft_tip",
                "tip_bend",
                "bend",
                "guidewire_profile",
                "coaxial_support_system",
                "sheath_bodies",
                "free_len",
                "max_slack",
                "insert_s",
                "rod_bodies",
                "contact_force",
                "wall_distance",
                "tip_tangent",
                "wall_normal",
                "distal_tip_rotation",
                "torsion_lag",
            ],
            "guidewire": guidewire,
            "support": support,
            "risk": risk,
        }

    def step(self, push: float, rotate: float) -> RawPose:
        self._ensure_initialized()

        direction = 1.0 if push >= 0.0 else -1.0
        prev_insert_s = self._insert_s
        advance = direction * abs(push) * self._push_speed * self._control_dt
        if self._drive == "force" and advance > 0.0:
            # Prolapse guard: budget the forward feed by how far the tip actually
            # is behind the fed wire. A blocked tip exhausts the budget instead of
            # buckling the free span into loops; pulling back restores it.
            slack = (prev_insert_s + float(self._base_arc[-1])) - self._tip_arclen()
            advance = _feed_budget(advance, slack, self._max_slack)
        self._insert_s = float(
            np.clip(prev_insert_s + advance, 0.0, self._max_s_ins)
        )
        # Integrate rotate into the root-frame twist that steers the J-tip (D4c).
        self._twist += float(rotate) * self._rotate_speed * self._control_dt

        for sub in range(self._substeps):
            frac = (sub + 1) / self._substeps
            s_ins = prev_insert_s + (self._insert_s - prev_insert_s) * frac
            self._s0.clear_forces()
            self._model.collide(self._s0, self._contacts)
            self._solver.step(self._s0, self._s1, self._control, self._contacts, self._sim_dt)
            self._s0, self._s1 = self._s1, self._s0
            self._apply_glue(s_ins)

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

        worst, wall_distance, contact_force = self._breach_stats(xyz)
        target = self._path.points[-1].tolist()
        # Anchor mode rides the route so inserted arc is exact; force mode lets the
        # tip physically lag, so report its actual projected arc along the route.
        if self._drive == "force":
            tip_arclen = float(min(_nearest_arclen(self._centerline, self._cl_cum, tip), self._path.total_len))
        else:
            tip_arclen = float(min(self._insert_s + self._base_arc[-1], self._path.total_len))
        done = bool(self._insert_s >= self._max_s_ins - 1e-9)
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
            arclen=tip_arclen,
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
