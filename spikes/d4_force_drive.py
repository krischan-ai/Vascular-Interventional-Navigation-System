"""D4 gate: real force-driven guidewire (replace D3 graded soft-anchor).

D3 drove the rod kinematically: every inserted body was glued to the centered
route each substep (``pos = (1-alpha)*phys + alpha*target``). That eliminated
穿管 but modelled no real push transmission -- the whole rod teleported forward,
so there was no tip lag, no buckling, no wall-hug/slide/rebound.

D4 drives with real force propagation. Only a short PROXIMAL portion (the
sheath/introducer + already-traversed wire) is glued to the centered route and
advanced at the push speed; everything distal (the working length + tip) is FREE
physics -- the cable stretch constraint transmits the push down the rod, and the
tip pose emerges from bend stiffness + wall contact. That yields real tip lag,
buckling, and wall hugging on bends.

Knobs that turn D3 -> D4:
  --sheath-bodies K   proximal bodies glued to route (K large -> D3 baseline,
                      K small -> pure force drive; the sheath/port support).
  --soft-tip M        distal M joints get a reduced bend-stiffness ramp
                      (软头硬身): stiff proximal shaft, soft distal tip so the
                      tip follows curves instead of jamming.

Metrics (the D4 acceptance criteria):
  tip_lag   arc gap between where the tip *would* be if rigidly fed and where it
            physically is -- >0 and BOUNDED is real transmission; growing =
            tip stuck / buckling.
  breach    radial containment vs the variable-radius lumen (must stay <0.5mm).
  strain    max per-segment stretch strain -- large spikes = compression buckling.
  tip_reach how far along the route the physical tip got (vs total length).

Run on the A6000 (from ~/cathsim-warp):
    python spikes/d4_force_drive.py --route endpoint_0 --sheath-bodies 3 --soft-tip 8
    python spikes/d4_force_drive.py --route endpoint_0 --drive anchor   # D3 baseline
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


# --- geometry helpers (shared with d2_aorta_tree_tube.py) ---------------------
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


def nearest_arc(points_cum, points, xyz):
    """Arc length of the nearest point on the polyline to xyz (scalar)."""
    a, b = points[:-1], points[1:]
    ab = b - a
    denom = np.einsum("ij,ij->i", ab, ab) + 1e-18
    ap = xyz - a
    t = np.clip(np.einsum("ij,ij->i", ap, ab) / denom, 0.0, 1.0)
    proj = a + t[:, None] * ab
    d2 = np.sum((xyz - proj) ** 2, axis=1)
    i = int(np.argmin(d2))
    return float(points_cum[i] + t[i] * (points_cum[i + 1] - points_cum[i]))


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


def max_strain(rod_xyz, rest_len):
    seg = np.linalg.norm(np.diff(rod_xyz, axis=0), axis=1)
    return float(np.max(np.abs(seg - rest_len) / rest_len))


# --- model -------------------------------------------------------------------
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
    rod_bodies, rod_joints = b.add_rod(positions=[wp.vec3(*p) for p in rod_pts], quaternions=rod_q, radius=ROD_RADIUS,
                                       stretch_stiffness=args.stretch, stretch_damping=0.0, bend_stiffness=args.bend,
                                       bend_damping=1.0e-1, label="guidewire")
    root = rod_bodies[0]
    b.body_mass[root] = 0.0; b.body_inv_mass[root] = 0.0
    b.body_inertia[root] = wp.mat33(0.0); b.body_inv_inertia[root] = wp.mat33(0.0)
    b.color(balance_colors=False)
    model = b.finalize(); model.set_gravity((0.0, 0.0, 0.0))

    # 软头硬身: per-joint bend stiffness. Cable joint DOF layout is
    # [stretch, bend] per joint, so bend ke lives at odd indices (2*j + 1).
    # Ramp the distal `soft_tip` joints from full bend down to `tip_bend`.
    if args.soft_tip > 0:
        ke = model.joint_target_ke.numpy()
        nj = len(rod_joints)
        for i in range(args.soft_tip):
            j = nj - 1 - i                      # joint index from the tip
            if j < 0:
                break
            frac = (i + 1) / args.soft_tip      # 0 near shaft .. 1 at tip
            bend = args.bend * (1.0 - frac) + args.tip_bend * frac
            ke[2 * j + 1] = bend
        model.joint_target_ke.assign(ke)

    solver = newton.solvers.SolverVBD(model, iterations=args.iterations)
    return model, solver, model.state(), model.state(), model.control(), model.contacts(), rod_bodies


def run(args) -> bool:
    root = Path(args.root)
    key, wp_pts, wp_r = load_route(root, args.phantom, args.route)
    cl, radii, total = resample_with_radii(wp_pts, wp_r, args.centerline_ds)
    cl_seg = np.linalg.norm(np.diff(cl, axis=0), axis=1)
    cl_cum = np.concatenate([[0.0], np.cumsum(cl_seg)])
    print(f"=== D4 force-drive [{key}] on {wp.get_device()} (newton {newton.__version__}) ===")
    print(f"route len={total*1e3:.0f}mm nodes={len(cl)} radius(mm) min={radii.min()*1e3:.2f} "
          f"p50={np.median(radii)*1e3:.2f} max={radii.max()*1e3:.2f}")
    print(f"drive={args.drive} sheath_bodies={args.sheath_bodies} soft_tip={args.soft_tip} "
          f"bend={args.bend} tip_bend={args.tip_bend}")
    verts, tris = build_tube(cl, radii, args.wall_t, args.around)

    rod_len = min(args.rod_length, 0.8 * total)
    rod_pts = sample_along(cl, rod_len, ROD_SEG_LEN)
    model, solver, s0, s1, ctrl, contacts, rod_bodies = build_model(verts, tris, rod_pts, args)
    rb_idx = np.asarray(rod_bodies)
    nb = len(rod_bodies)
    base_arc = np.arange(nb) * ROD_SEG_LEN
    rod_span = float(base_arc[-1])
    max_s_ins = max(0.0, total - rod_span - 1e-3)
    fps = 60; sim_dt = (1.0 / fps) / args.substeps

    def points_at(arcs):
        arcs = np.clip(arcs, 0.0, cl_cum[-1])
        return np.column_stack([np.interp(arcs, cl_cum, cl[:, d]) for d in range(3)])

    # --- glue mask (which bodies are kinematically driven along the route) ----
    # anchor (D3): proximal glued + distal soft ramp. force (D4): only the
    # proximal `sheath_bodies` glued, everything distal is free physics.
    alpha = np.zeros(nb)
    if args.drive == "anchor":
        alpha[:] = 1.0
        for i in range(args.free_span):
            j = nb - 1 - i
            if j >= 0:
                frac = (i + 1) / args.free_span
                alpha[j] = max(args.tip_alpha, 1.0 - frac * (1.0 - args.tip_alpha))
    else:  # force
        k = max(1, args.sheath_bodies)
        alpha[:k] = 1.0
    glued = alpha > 0.0

    # --- settle (physics only, glued bodies pinned to their start arc) --------
    settle = []
    for _ in range(args.settle_steps):
        s0.clear_forces(); model.collide(s0, contacts)
        solver.step(s0, s1, ctrl, contacts, sim_dt); s0, s1 = s1, s0
        if glued.any():
            bq = s0.body_q.numpy()
            tgt = points_at(base_arc[glued])
            bq[rb_idx[glued], :3] = (1.0 - alpha[glued])[:, None] * bq[rb_idx[glued], :3] + alpha[glued][:, None] * tgt
            s0.body_q.assign(bq)
        settle.append(radial_breach_mm(s0.body_q.numpy()[rod_bodies, :3], cl, radii))
    print(f"settle: breach start={settle[0]:+.2f}mm end={settle[-1]:+.2f}mm max={max(settle):+.2f}mm")

    # --- drive ---------------------------------------------------------------
    sub_disp = args.push_speed * sim_dt
    n = 0
    prof = {"collide": 0.0, "step": 0.0, "sync": 0.0}
    breach_log, lag_log, strain_log, tip_arc_log = [], [], [], []
    t0 = time.perf_counter()
    for _f in range(args.frames):
        for _ in range(args.substeps):
            n += 1
            s_ins = min(sub_disp * n, max_s_ins)
            if args.profile: wp.synchronize(); ta = time.perf_counter()
            s0.clear_forces(); model.collide(s0, contacts)
            if args.profile: wp.synchronize(); tb = time.perf_counter(); prof["collide"] += tb - ta
            solver.step(s0, s1, ctrl, contacts, sim_dt); s0, s1 = s1, s0
            if args.profile: wp.synchronize(); tc = time.perf_counter(); prof["step"] += tc - tb
            if glued.any():
                bq = s0.body_q.numpy()
                tgt = points_at(base_arc[glued] + s_ins)
                bq[rb_idx[glued], :3] = (1.0 - alpha[glued])[:, None] * bq[rb_idx[glued], :3] + alpha[glued][:, None] * tgt
                s0.body_q.assign(bq)
            if args.profile: wp.synchronize(); prof["sync"] += time.perf_counter() - tc
        xyz = s0.body_q.numpy()[rod_bodies, :3]
        tip_arc = nearest_arc(cl_cum, cl, xyz[-1])
        fed_arc = min(s_ins + rod_span, total)          # where a rigid feed would put the tip
        breach_log.append(radial_breach_mm(xyz, cl, radii))
        lag_log.append((fed_arc - tip_arc) * 1e3)       # mm
        strain_log.append(max_strain(xyz, ROD_SEG_LEN))
        tip_arc_log.append(tip_arc * 1e3)
    if args.profile:
        tot = sum(prof.values()) or 1.0
        print("profile per-phase ms/substep + %: " + " ".join(
            f"{k}={prof[k]/n*1e3:.2f}ms({prof[k]/tot:.0%})" for k in prof))
    wp.synchronize()
    elapsed = time.perf_counter() - t0

    q = s0.body_q.numpy()
    finite = bool(np.isfinite(q).all())
    breach_arr = np.asarray(breach_log); lag_arr = np.asarray(lag_log); strain_arr = np.asarray(strain_log)
    tail = slice(int(0.75 * len(breach_arr)), None)
    steady_breach = float(breach_arr[tail].mean())
    steady_lag = float(lag_arr[tail].mean())
    tip_reach = tip_arc_log[-1]
    # tip must keep progressing (not stuck): compare last quarter mean vs mid mean
    mid = tip_arc_log[len(tip_arc_log) // 2]
    progressing = tip_reach > mid + 1.0     # advanced >1mm in the back half
    contained = finite and max(settle) < 0.5 and steady_breach < 0.5 and breach_arr.max() < 1.0
    steady_strain = float(strain_arr[tail].mean())
    strain_p95 = float(np.percentile(strain_arr, 95))
    transmits = progressing and steady_lag < args.lag_budget_mm and steady_strain < args.strain_budget

    print(f"breach: steady={steady_breach:+.3f}mm worst={breach_arr.max():+.2f}mm")
    print(f"tip_lag: steady={steady_lag:+.2f}mm max={lag_arr.max():+.2f}mm  (budget {args.lag_budget_mm}mm)")
    print(f"strain: steady={steady_strain:.3f} p95={strain_p95:.3f} max={strain_arr.max():.3f}  (budget {args.strain_budget})")
    print(f"tip_reach={tip_reach:.1f}mm / route {total*1e3:.0f}mm  progressing={progressing} finite={finite}")
    print(f"throughput={args.frames / elapsed:.1f} control-fps ({args.frames * args.substeps / elapsed:.0f} phys-steps/s)")
    ok = contained and transmits
    verdict = "PASS" if ok else ("CONTAINED-BUT-STUCK" if contained else "FAIL")
    print(f"D4 FORCE-DRIVE [{key}]: {verdict}  (contained={contained} transmits={transmits})")
    return ok


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--root", default=".")
    p.add_argument("--phantom", default="aorta_tree")
    p.add_argument("--route", default="endpoint_0")
    p.add_argument("--drive", choices=["force", "anchor"], default="force")
    p.add_argument("--centerline-ds", type=float, default=1.5e-3)
    p.add_argument("--around", type=int, default=24)
    p.add_argument("--wall-t", type=float, default=6.0e-3)
    p.add_argument("--sdf-voxel", type=float, default=5.0e-4)
    p.add_argument("--lumen-band", type=float, default=8.0e-3)
    p.add_argument("--rod-length", type=float, default=0.06)
    p.add_argument("--stretch", type=float, default=1.0e5)
    p.add_argument("--bend", type=float, default=50.0)
    p.add_argument("--tip-bend", type=float, default=5.0)     # distal soft-tip bend stiffness
    p.add_argument("--soft-tip", type=int, default=0)         # # distal joints on the soft ramp
    p.add_argument("--contact-ke", type=float, default=1.0e6)
    p.add_argument("--mu", type=float, default=0.2)
    p.add_argument("--iterations", type=int, default=4)
    p.add_argument("--substeps", type=int, default=8)
    p.add_argument("--push-speed", type=float, default=0.05)
    p.add_argument("--sheath-bodies", type=int, default=3)    # force mode: proximal glued bodies
    p.add_argument("--free-span", type=int, default=6)        # anchor mode: distal soft ramp
    p.add_argument("--tip-alpha", type=float, default=0.3)    # anchor mode: tip pull weight
    p.add_argument("--lag-budget-mm", type=float, default=20.0)
    p.add_argument("--strain-budget", type=float, default=0.5)
    p.add_argument("--frames", type=int, default=120)
    p.add_argument("--settle-steps", type=int, default=30)
    p.add_argument("--profile", action="store_true")
    args = p.parse_args()
    wp.init()
    raise SystemExit(0 if run(args) else 1)


if __name__ == "__main__":
    main()
