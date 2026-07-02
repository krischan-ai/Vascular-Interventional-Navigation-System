"""D2b gate: Newton rod in the REAL segment_part lumen via a thick-wall annulus.

Why an annulus (not the raw visual.stl surface): Newton's ``build_sdf(is_solid=True)``
treats the volume *enclosed* by a single closed surface as solid, so the bare
``visual.stl`` makes the LUMEN solid and expels the rod (confirmed: both windings
eject). A thin shell also lets a fast rod tunnel and get expelled by the far side
(see D1's WALL_T note). So we build a bounded **thick-wall solid**: the lumen is a
tunnel bored through solid wall (lumen = SDF>0 free space, wall = SDF<0 solid).

Wall construction (dependency-free): offset the watertight ``visual.stl`` outward
along vertex normals by ``wall_t`` to get the outer shell, then combine
[inner (reversed) + outer] into one closed solid whose interior is the wall.

Ground truth: containment is measured against a signed field from the binary
``Segmentation.seg.nrrd`` (positive inside lumen), independent of the collision mesh.
The driven centerline is re-centered by gradient-ascent on that field so the rod is
never driven into the wall.

Run on the A6000 (from ~/cathsim-warp):
    python spikes/d2_visual_stl_sdf.py
"""

from __future__ import annotations

import argparse
import gzip
import json
import time
from pathlib import Path

import numpy as np
import warp as wp

import newton

try:
    from scipy import ndimage
except Exception as exc:  # pragma: no cover
    raise RuntimeError("D2 requires scipy for the ground-truth signed field") from exc


ROD_RADIUS = 4.0e-4
ROD_SEG_LEN = 3.0e-3
DENSITY = 1.8e3


# --------------------------------------------------------------------------- assets
def load_stl(path: Path) -> tuple[np.ndarray, np.ndarray]:
    buf = path.read_bytes()
    n = int(np.frombuffer(buf, dtype="<u4", count=1, offset=80)[0])
    rec = np.frombuffer(
        buf, dtype=np.dtype([("norm", "<3f4"), ("v", "<3,3f4"), ("attr", "<u2")]), count=n, offset=84
    )
    raw = rec["v"].reshape(-1, 3).astype(np.float64)
    key = np.round(raw * 1e6).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    return (uniq / 1e6).astype(np.float64), inv.reshape(n, 3).astype(np.int32)


def load_centerline(path: Path) -> np.ndarray:
    return np.asarray(json.loads(path.read_text(encoding="utf-8"))["waypoints"], dtype=np.float64)


def read_nrrd_mask(path: Path, order: str) -> np.ndarray:
    header, payload = path.read_bytes().split(b"\n\n", 1)
    fields = {}
    for line in header.decode("latin1").splitlines():
        if ":" in line and not line.startswith("#"):
            k, v = line.split(":", 1)
            fields[k.strip()] = v.strip()
    sizes = tuple(int(v) for v in fields["sizes"].split())
    data = np.frombuffer(gzip.decompress(payload), dtype=np.uint8)
    return data.reshape(sizes, order=order) > 0


# --------------------------------------------------------------------------- ground truth
def world_m_to_ijk(points_m: np.ndarray) -> np.ndarray:
    mm = np.asarray(points_m) * 1000.0
    return np.column_stack([-mm[:, 0], -mm[:, 1], mm[:, 2]])


def build_signed_field_mm(mask: np.ndarray) -> np.ndarray:
    return ndimage.distance_transform_edt(mask) - ndimage.distance_transform_edt(~mask)


def sample_signed_mm(signed_mm: np.ndarray, points_m: np.ndarray) -> np.ndarray:
    return ndimage.map_coordinates(signed_mm, world_m_to_ijk(points_m).T, order=1, mode="constant", cval=-1e3)


def recenter_centerline(signed_mm, points_m, iters, step_mm):
    """Gradient-ascent each point toward the lumen center (max signed), perpendicular to the tangent."""
    gz, gy, gx = np.gradient(signed_mm)  # d/di, d/dj, d/dk (voxel = mm)
    pts = points_m.copy()
    for _ in range(iters):
        ijk = world_m_to_ijk(pts).T
        gi = ndimage.map_coordinates(gz, ijk, order=1, mode="nearest")
        gj = ndimage.map_coordinates(gy, ijk, order=1, mode="nearest")
        gk = ndimage.map_coordinates(gx, ijk, order=1, mode="nearest")
        # grad in world m: i=-x_mm, j=-y_mm, k=+z_mm -> dsigned/dworld (per mm)
        g_world = np.column_stack([-gi, -gj, gk])
        norm = np.linalg.norm(g_world, axis=1, keepdims=True) + 1e-12
        g_dir = g_world / norm
        tang = np.zeros_like(pts)
        tang[1:-1] = pts[2:] - pts[:-2]
        tang[0] = pts[1] - pts[0]
        tang[-1] = pts[-1] - pts[-2]
        tang /= np.linalg.norm(tang, axis=1, keepdims=True) + 1e-12
        g_perp = g_dir - tang * np.sum(g_dir * tang, axis=1, keepdims=True)  # stay on cross-section
        pts = pts + g_perp * (step_mm * 1e-3)
    return pts


# --------------------------------------------------------------------------- mesh / wall
def vertex_normals(verts, tris):
    fn = np.cross(verts[tris[:, 1]] - verts[tris[:, 0]], verts[tris[:, 2]] - verts[tris[:, 0]])
    vn = np.zeros_like(verts)
    for k in range(3):
        np.add.at(vn, tris[:, k], fn)
    return vn / (np.linalg.norm(vn, axis=1, keepdims=True) + 1e-12)


def build_annulus(verts, tris, signed_mm, wall_t):
    """Thick-wall solid: outer shell = verts offset outward (into wall) by wall_t."""
    vn = vertex_normals(verts, tris)
    # outward = direction of DECREASING signed field (out of lumen, into wall)
    probe = 0.3e-3
    s_plus = sample_signed_mm(signed_mm, verts + vn * probe)
    s_minus = sample_signed_mm(signed_mm, verts - vn * probe)
    outward = np.where((s_minus - s_plus)[:, None] >= 0, vn, -vn)  # sign so outward lowers signed
    outer = verts + outward * wall_t
    inner_tris = tris[:, [0, 2, 1]]            # reversed -> normals point into lumen (hole)
    outer_tris = tris + len(verts)             # original -> normals point outward
    all_verts = np.vstack([verts, outer]).astype(np.float32)
    all_tris = np.vstack([inner_tris, outer_tris]).astype(np.int32).reshape(-1)
    return all_verts, all_tris


# --------------------------------------------------------------------------- polyline helpers
def point_at_s(points, s):
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


def resample_polyline(points, ds):
    seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
    total = float(np.sum(seg))
    return np.asarray([point_at_s(points, s) for s in np.arange(0.0, total, ds)] + [points[-1]])


def sample_along(points, length, seg_len):
    total = float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))
    length = min(length, total)
    return np.asarray([point_at_s(points, s) for s in np.arange(0.0, length + 0.5 * seg_len, seg_len)])


# --------------------------------------------------------------------------- model
def build_model(all_verts, all_tris, rod_pts, args):
    b = newton.ModelBuilder()
    b.rigid_gap = 0.0
    b.default_shape_cfg.density = DENSITY
    b.default_shape_cfg.ke = args.contact_ke
    b.default_shape_cfg.kd = 1.0
    b.default_shape_cfg.mu = args.mu

    mesh = newton.Mesh(all_verts, all_tris, is_solid=True)
    mesh.build_sdf(target_voxel_size=args.sdf_voxel, narrow_band_range=(-args.wall_t, args.lumen_band))
    b.add_shape_mesh(body=-1, mesh=mesh, cfg=newton.ModelBuilder.ShapeConfig(ke=args.contact_ke, kd=1.0, mu=args.mu),
                     label="vessel_wall")

    rod_q = newton.utils.create_parallel_transport_cable_quaternions([wp.vec3(*p) for p in rod_pts], twist_total=0.0)
    rod_bodies, _ = b.add_rod(
        positions=[wp.vec3(*p) for p in rod_pts], quaternions=rod_q, radius=ROD_RADIUS,
        stretch_stiffness=1.0e5, stretch_damping=0.0, bend_stiffness=args.bend, bend_damping=1.0e-1, label="guidewire",
    )
    root = rod_bodies[0]
    b.body_mass[root] = 0.0
    b.body_inv_mass[root] = 0.0
    b.body_inertia[root] = wp.mat33(0.0)
    b.body_inv_inertia[root] = wp.mat33(0.0)
    b.color(balance_colors=False)
    model = b.finalize()
    model.set_gravity((0.0, 0.0, 0.0))
    solver = newton.solvers.SolverVBD(model, iterations=args.iterations)
    return model, solver, model.state(), model.state(), model.control(), model.contacts(), rod_bodies


def breach_mm(signed_mm, rod_xyz):
    margin = sample_signed_mm(signed_mm, rod_xyz)
    return float((ROD_RADIUS * 1e3 - margin).max())


# --------------------------------------------------------------------------- run
def run(args) -> bool:
    root = Path(args.root)
    verts, tris = load_stl(root / "src/cathsim/dm/components/phantom_assets/meshes/segment_part/visual.stl")
    cl_raw = load_centerline(root / "src/cathsim/dm/components/phantom_assets/meshes/segment_part/centerline.json")
    mask = read_nrrd_mask(root / "data/aorta_centerline/Segmentation.seg.nrrd", args.nrrd_order)
    signed_mm = build_signed_field_mm(mask)

    print(f"=== D2b annulus wall on {wp.get_device()} (newton {newton.__version__}) ===")
    cl = resample_polyline(cl_raw, args.centerline_ds)
    before = sample_signed_mm(signed_mm, cl)
    cl = recenter_centerline(signed_mm, cl, args.recenter_iters, args.recenter_step)
    after = sample_signed_mm(signed_mm, cl)
    print(f"centerline signed(mm) before: p50={np.median(before):.2f} min={before.min():.2f} inside={ (before>0).mean():.0%}")
    print(f"centerline signed(mm) after : p50={np.median(after):.2f} min={after.min():.2f} inside={ (after>0).mean():.0%}")

    all_verts, all_tris = build_annulus(verts, tris, signed_mm, args.wall_t)
    rod_pts = sample_along(cl, args.rod_length, ROD_SEG_LEN)
    model, solver, s0, s1, ctrl, contacts, rod_bodies = build_model(all_verts, all_tris, rod_pts, args)
    root_body = rod_bodies[0]
    fps = 60
    sim_dt = (1.0 / fps) / args.substeps

    # settle (no push) -> is the rod contained by the wall at all?
    settle = []
    for _ in range(args.settle_steps):
        s0.clear_forces(); model.collide(s0, contacts); solver.step(s0, s1, ctrl, contacts, sim_dt); s0, s1 = s1, s0
        settle.append(breach_mm(signed_mm, s0.body_q.numpy()[rod_bodies, :3]))
    print(f"settle: breach start={settle[0]:+.2f}mm end={settle[-1]:+.2f}mm max={max(settle):+.2f}mm")

    # drive: advance root along the (re-centered) centerline
    pre = s0.body_q.numpy()[rod_bodies, :3].copy()
    tip0 = pre[-1].copy()
    sub_disp = args.push_speed * sim_dt
    margins, n = [], 0
    t0 = time.perf_counter()
    for _f in range(args.frames):
        for _ in range(args.substeps):
            n += 1
            bq = s0.body_q.numpy()
            bq[root_body, :3] = point_at_s(cl, sub_disp * n)
            s0.body_q.assign(bq)
            s0.clear_forces(); model.collide(s0, contacts); solver.step(s0, s1, ctrl, contacts, sim_dt); s0, s1 = s1, s0
        margins.append(breach_mm(signed_mm, s0.body_q.numpy()[rod_bodies, :3]))
    wp.synchronize()
    elapsed = time.perf_counter() - t0

    q = s0.body_q.numpy()
    disp = np.linalg.norm(q[rod_bodies, :3] - pre, axis=1) * 1e3
    root_target = point_at_s(cl, sub_disp * n)
    print(f"DEBUG root: commanded_s={sub_disp*n*1e3:.1f}mm target={root_target} actual={q[root_body,:3]}")
    print(f"DEBUG body disp(mm): min={disp.min():.2f} mean={disp.mean():.2f} max={disp.max():.2f}  root_disp={disp[0]:.2f} tip_disp={disp[-1]:.2f}")
    finite = bool(np.isfinite(q).all())
    margins = np.asarray(margins)
    steady = float(margins[int(0.75 * len(margins)):].mean())
    worst = float(margins.max())
    tip_adv = float(np.linalg.norm(q[rod_bodies[-1], :3] - tip0))
    ok = finite and steady < 0.5 and tip_adv > args.min_tip_advance
    print(f"drive push={args.push_speed:.2f} ss={args.substeps} bend={args.bend} wall_t={args.wall_t*1e3:.0f}mm "
          f"tip_adv={tip_adv*1e3:.1f}mm steady={steady:+.3f}mm worst={worst:+.2f}mm finite={finite}")
    print(f"throughput={args.frames / elapsed:.1f} control-fps ({args.frames * args.substeps / elapsed:.0f} phys-steps/s)")
    print(f"D2b ANNULUS GATE: {'PASS' if ok else 'PARTIAL/FAIL'}")
    return ok


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--root", default=".")
    p.add_argument("--nrrd-order", choices=["C", "F"], default="F")
    p.add_argument("--centerline-ds", type=float, default=1.5e-3)
    p.add_argument("--recenter-iters", type=int, default=30)
    p.add_argument("--recenter-step", type=float, default=0.3)  # mm/iter
    p.add_argument("--wall-t", type=float, default=4.0e-3)
    p.add_argument("--sdf-voxel", type=float, default=4.0e-4)
    p.add_argument("--lumen-band", type=float, default=4.0e-3)
    p.add_argument("--rod-length", type=float, default=0.06)
    p.add_argument("--bend", type=float, default=1.0)
    p.add_argument("--contact-ke", type=float, default=1.0e6)
    p.add_argument("--mu", type=float, default=0.2)
    p.add_argument("--iterations", type=int, default=8)
    p.add_argument("--substeps", type=int, default=40)
    p.add_argument("--push-speed", type=float, default=0.05)
    p.add_argument("--frames", type=int, default=120)
    p.add_argument("--settle-steps", type=int, default=30)
    p.add_argument("--min-tip-advance", type=float, default=0.02)
    args = p.parse_args()
    wp.init()
    raise SystemExit(0 if run(args) else 1)


if __name__ == "__main__":
    main()
