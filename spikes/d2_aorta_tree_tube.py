"""D2c gate: Newton rod in an aorta_tree route via a REAL-radius thick-wall tube.

The aorta_tree surface mesh is coarse and open-ended (bad SDF source), but every
navigation route in ``aorta_tree/routes.json`` carries real per-waypoint VMTK
inscribed radii (``radius_m``) along an already-centered centerline. So the wall
here is a **variable-radius thick-wall annulus tube** built from those real radii
(watertight by construction, real geometry) -- the robust P2 realization for the
tree. This is what the original ``d2_segment_part_radius_tube.py`` attempted, but
with *correct* radii instead of a fragile off-center distance transform.

Containment is checked against the variable-radius centerline tube (radii are the
real VMTK measurements, so this is a faithful self-consistent check).

Run on the A6000 (from ~/cathsim-warp):
    python spikes/d2_aorta_tree_tube.py --route endpoint_0
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import warp as wp

import newton


ROD_RADIUS = 4.0e-4
ROD_SEG_LEN = 3.0e-3
DENSITY = 1.8e3


def load_route(root: Path, phantom: str, route: str):
    rj = json.loads((root / f"src/cathsim/dm/components/phantom_assets/meshes/{phantom}/routes.json").read_text())
    routes = rj["routes"]
    key = route if route in routes else list(routes)[0]
    r = routes[key]
    return key, np.asarray(r["waypoints"], dtype=np.float64), np.asarray(r["radius_m"], dtype=np.float64)


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


def resample_with_radii(points, radii, ds):
    seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(cum[-1])
    ss = np.arange(0.0, total, ds)
    ss = np.append(ss, total)
    out_p = np.asarray([point_at_s(points, s) for s in ss])
    out_r = np.interp(ss, cum, radii)
    return out_p, out_r, total


def parallel_frames(points):
    tang = np.zeros_like(points)
    tang[:-1] = points[1:] - points[:-1]
    tang[-1] = tang[-2]
    tang /= np.linalg.norm(tang, axis=1, keepdims=True) + 1e-12
    ref = np.array([0.0, 1.0, 0.0]) if abs(tang[0, 1]) < 0.9 else np.array([1.0, 0.0, 0.0])
    nrm = np.zeros_like(points)
    nrm[0] = np.cross(tang[0], ref); nrm[0] /= np.linalg.norm(nrm[0])
    for i in range(1, len(points)):
        v = nrm[i - 1] - tang[i] * float(np.dot(nrm[i - 1], tang[i]))
        nv = float(np.linalg.norm(v))
        nrm[i] = v / nv if nv > 1e-9 else nrm[i - 1]
    return tang, nrm, np.cross(tang, nrm)


def build_tube(points, radii, wall_t, k_around):
    """Thick-wall annulus tube: inner ring at radius r, outer ring at r+wall_t."""
    _, nrm, binm = parallel_frames(points)
    angles = np.linspace(0.0, 2.0 * math.pi, k_around, endpoint=False)
    verts = []
    for p, r, nn, bb in zip(points, radii, nrm, binm):
        for ring_r in (r, r + wall_t):
            for a in angles:
                verts.append(p + ring_r * (math.cos(a) * nn + math.sin(a) * bb))
    stride = 2 * k_around

    def vi(i, ring, k):
        return i * stride + ring * k_around + (k % k_around)

    tris = []
    for i in range(len(points) - 1):
        for k in range(k_around):
            kn = (k + 1) % k_around
            tris += [vi(i, 0, k), vi(i + 1, 0, k), vi(i + 1, 0, kn)]
            tris += [vi(i, 0, k), vi(i + 1, 0, kn), vi(i, 0, kn)]
            tris += [vi(i, 1, k), vi(i, 1, kn), vi(i + 1, 1, kn)]
            tris += [vi(i, 1, k), vi(i + 1, 1, kn), vi(i + 1, 1, k)]
    for i, flip in ((0, False), (len(points) - 1, True)):
        for k in range(k_around):
            kn = (k + 1) % k_around
            a, b, c, dd = vi(i, 0, k), vi(i, 0, kn), vi(i, 1, kn), vi(i, 1, k)
            tris += [a, c, b, a, dd, c] if flip else [a, b, c, a, c, dd]
    return np.asarray(verts, dtype=np.float32), np.asarray(tris, dtype=np.int32).reshape(-1)


def sample_along(points, length, seg_len):
    total = float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))
    length = min(length, total)
    return np.asarray([point_at_s(points, s) for s in np.arange(0.0, length + 0.5 * seg_len, seg_len)])


def radial_breach_mm(rod_xyz, centerline, radii):
    a, b = centerline[:-1], centerline[1:]
    ab = b - a
    denom = np.einsum("ij,ij->i", ab, ab) + 1e-18
    worst = -1e9
    for p in rod_xyz:
        ap = p - a
        t = np.clip(np.einsum("ij,ij->i", ap, ab) / denom, 0.0, 1.0)
        proj = a + t[:, None] * ab
        d2 = np.sum((p - proj) ** 2, axis=1)
        i = int(np.argmin(d2))
        local_r = (1.0 - t[i]) * radii[i] + t[i] * radii[i + 1]
        worst = max(worst, math.sqrt(float(d2[i])) + ROD_RADIUS - float(local_r))
    return worst * 1e3


def build_model(verts, tris, rod_pts, args):
    b = newton.ModelBuilder()
    b.rigid_gap = 0.0
    b.default_shape_cfg.density = DENSITY
    b.default_shape_cfg.ke = args.contact_ke; b.default_shape_cfg.kd = 1.0; b.default_shape_cfg.mu = args.mu
    mesh = newton.Mesh(verts, tris, is_solid=True)
    mesh.build_sdf(target_voxel_size=args.sdf_voxel, narrow_band_range=(-args.wall_t, args.lumen_band))
    b.add_shape_mesh(body=-1, mesh=mesh, cfg=newton.ModelBuilder.ShapeConfig(ke=args.contact_ke, kd=1.0, mu=args.mu),
                     label="vessel_wall")
    rod_q = newton.utils.create_parallel_transport_cable_quaternions([wp.vec3(*p) for p in rod_pts], twist_total=0.0)
    rod_bodies, _ = b.add_rod(positions=[wp.vec3(*p) for p in rod_pts], quaternions=rod_q, radius=ROD_RADIUS,
                              stretch_stiffness=1.0e5, stretch_damping=0.0, bend_stiffness=args.bend,
                              bend_damping=1.0e-1, label="guidewire")
    root = rod_bodies[0]
    b.body_mass[root] = 0.0; b.body_inv_mass[root] = 0.0
    b.body_inertia[root] = wp.mat33(0.0); b.body_inv_inertia[root] = wp.mat33(0.0)
    b.color(balance_colors=False)
    model = b.finalize(); model.set_gravity((0.0, 0.0, 0.0))
    solver = newton.solvers.SolverVBD(model, iterations=args.iterations)
    return model, solver, model.state(), model.state(), model.control(), model.contacts(), rod_bodies


def run(args) -> bool:
    root = Path(args.root)
    key, wp_pts, wp_r = load_route(root, args.phantom, args.route)
    cl, radii, total = resample_with_radii(wp_pts, wp_r, args.centerline_ds)
    print(f"=== D2c aorta_tree tube [{key}] on {wp.get_device()} (newton {newton.__version__}) ===")
    print(f"route len={total*1e3:.0f}mm nodes={len(cl)} radius(mm) min={radii.min()*1e3:.2f} "
          f"p50={np.median(radii)*1e3:.2f} max={radii.max()*1e3:.2f}")
    verts, tris = build_tube(cl, radii, args.wall_t, args.around)

    rod_len = min(args.rod_length, 0.8 * total)
    rod_pts = sample_along(cl, rod_len, ROD_SEG_LEN)
    model, solver, s0, s1, ctrl, contacts, rod_bodies = build_model(verts, tris, rod_pts, args)
    root_body = rod_bodies[0]
    fps = 60; sim_dt = (1.0 / fps) / args.substeps

    settle = []
    for _ in range(args.settle_steps):
        s0.clear_forces(); model.collide(s0, contacts); solver.step(s0, s1, ctrl, contacts, sim_dt); s0, s1 = s1, s0
        settle.append(radial_breach_mm(s0.body_q.numpy()[rod_bodies, :3], cl, radii))
    print(f"settle: breach start={settle[0]:+.2f}mm end={settle[-1]:+.2f}mm max={max(settle):+.2f}mm")

    pre = s0.body_q.numpy()[rod_bodies, :3].copy()
    sub_disp = args.push_speed * sim_dt
    nb = len(rod_bodies)
    base_arc = np.arange(nb) * ROD_SEG_LEN
    max_s_ins = max(0.0, total - base_arc[-1] - 1e-3)  # keep the distal tip on the route

    # Graded anchor: proximal (inserted) bodies glued to the centered route (alpha=1);
    # a distal ramp of `free_span` bodies keeps most of their physics (alpha -> tip_alpha),
    # but are softly pulled back so the soft tip deflects without whipping through the wall.
    alpha = np.ones(nb)
    for i in range(args.free_span):
        j = nb - 1 - i
        if j >= 0:
            frac = (i + 1) / args.free_span          # 0 near ramp start .. 1 at the tip
            alpha[j] = max(args.tip_alpha, 1.0 - frac * (1.0 - args.tip_alpha))
    rb_idx = np.asarray(rod_bodies)

    # precompute cumulative arclength for vectorized point sampling
    cl_seg = np.linalg.norm(np.diff(cl, axis=0), axis=1)
    cl_cum = np.concatenate([[0.0], np.cumsum(cl_seg)])

    def points_at(arcs):
        arcs = np.clip(arcs, 0.0, cl_cum[-1])
        return np.column_stack([np.interp(arcs, cl_cum, cl[:, d]) for d in range(3)])

    prof = {"collide": 0.0, "step": 0.0, "sync": 0.0}
    margins, n = [], 0
    t0 = time.perf_counter()
    for _f in range(args.frames):
        for _ in range(args.substeps):
            n += 1
            s_ins = min(sub_disp * n, max_s_ins)
            target = points_at(base_arc + s_ins)
            if args.profile: wp.synchronize(); ta = time.perf_counter()
            s0.clear_forces(); model.collide(s0, contacts)
            if args.profile: wp.synchronize(); tb = time.perf_counter(); prof["collide"] += tb - ta
            solver.step(s0, s1, ctrl, contacts, sim_dt); s0, s1 = s1, s0
            if args.profile: wp.synchronize(); tc = time.perf_counter(); prof["step"] += tc - tb
            bq = s0.body_q.numpy()
            bq[rb_idx, :3] = (1.0 - alpha)[:, None] * bq[rb_idx, :3] + alpha[:, None] * target
            s0.body_q.assign(bq)
            if args.profile: wp.synchronize(); prof["sync"] += time.perf_counter() - tc
        margins.append(radial_breach_mm(s0.body_q.numpy()[rod_bodies, :3], cl, radii))
    if args.profile:
        tot = sum(prof.values()) or 1.0
        print("profile per-phase ms/substep + %%: " + " ".join(
            f"{k}={prof[k]/n*1e3:.2f}ms({prof[k]/tot:.0%})" for k in prof))
    wp.synchronize()
    elapsed = time.perf_counter() - t0

    q = s0.body_q.numpy()
    finite = bool(np.isfinite(q).all())
    margins = np.asarray(margins)
    steady = float(margins[int(0.75 * len(margins)):].mean())
    tip_adv = float(np.linalg.norm(q[rod_bodies[-1], :3] - pre[-1])) * 1e3
    ok = finite and steady < 0.5 and max(settle) < 0.5
    print(f"drive push={args.push_speed:.2f} ss={args.substeps} bend={args.bend} wall_t={args.wall_t*1e3:.0f}mm "
          f"tip_adv={tip_adv:.1f}mm steady={steady:+.3f}mm worst={margins.max():+.2f}mm finite={finite}")
    print(f"throughput={args.frames / elapsed:.1f} control-fps ({args.frames * args.substeps / elapsed:.0f} phys-steps/s)")
    print(f"D2c AORTA_TREE-TUBE (containment): {'PASS' if ok else 'PARTIAL/FAIL'}  [tip advance = stage D]")
    return ok


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--root", default=".")
    p.add_argument("--phantom", default="aorta_tree")
    p.add_argument("--route", default="endpoint_0")
    p.add_argument("--centerline-ds", type=float, default=1.5e-3)
    p.add_argument("--around", type=int, default=24)
    p.add_argument("--wall-t", type=float, default=6.0e-3)
    p.add_argument("--sdf-voxel", type=float, default=5.0e-4)
    p.add_argument("--lumen-band", type=float, default=8.0e-3)
    p.add_argument("--rod-length", type=float, default=0.06)
    p.add_argument("--bend", type=float, default=50.0)
    p.add_argument("--contact-ke", type=float, default=1.0e6)
    p.add_argument("--mu", type=float, default=0.2)
    p.add_argument("--iterations", type=int, default=8)
    p.add_argument("--substeps", type=int, default=40)
    p.add_argument("--push-speed", type=float, default=0.05)
    p.add_argument("--free-span", type=int, default=6)   # # distal bodies on the soft ramp
    p.add_argument("--tip-alpha", type=float, default=0.3)  # tip pull-to-center weight (0=free,1=glued)
    p.add_argument("--frames", type=int, default=90)
    p.add_argument("--settle-steps", type=int, default=30)
    p.add_argument("--profile", action="store_true")
    args = p.parse_args()
    wp.init()
    raise SystemExit(0 if run(args) else 1)


if __name__ == "__main__":
    main()
