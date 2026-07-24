"""D2 gate: Newton rod in a segment_part tube derived from true segmentation radii.

This is the first "true-cavity" spike after D1.  It uses the real
``Segmentation.seg.nrrd`` binary labelmap to estimate the lumen radius along the
real ``segment_part/centerline.json`` path, then builds a watertight thick-wall
tube with variable inner radius.  That keeps the robust D1 collision recipe
(SDF over a solid wall with a tunnel) while grounding the tube size and path in
the real asset.

Run on the A6000 server:
    python d2_segment_part_radius_tube.py
"""

from __future__ import annotations

import argparse
import gzip
import json
import math
import time
from pathlib import Path

import numpy as np
import warp as wp

import newton

try:
    from scipy import ndimage
except Exception as exc:  # pragma: no cover - clearer failure on the server
    raise RuntimeError("D2 requires scipy for distance_transform_edt") from exc


ROD_RADIUS = 4.0e-4
ROD_SEG_LEN = 3.0e-3
ROD_LENGTH = 0.18
DENSITY = 1.8e3


def read_nrrd_mask(path: Path, order: str) -> np.ndarray:
    """Read this project's gzip uint8 NRRD mask without adding a new dependency."""
    blob = path.read_bytes()
    header, payload = blob.split(b"\n\n", 1)
    text = header.decode("latin1")
    fields: dict[str, str] = {}
    for line in text.splitlines():
        if not line or line.startswith("#") or ":" not in line:
            continue
        k, v = line.split(":", 1)
        fields[k.strip()] = v.strip()

    if fields.get("type") != "unsigned char" or fields.get("encoding") != "gzip":
        raise ValueError(f"Unsupported NRRD encoding/type: {fields.get('type')} {fields.get('encoding')}")
    sizes = tuple(int(v) for v in fields["sizes"].split())
    data = np.frombuffer(gzip.decompress(payload), dtype=np.uint8)
    if data.size != math.prod(sizes):
        raise ValueError(f"NRRD payload size mismatch: {data.size} vs {sizes}")

    return data.reshape(sizes, order=order) > 0


def load_centerline(path: Path) -> np.ndarray:
    data = json.loads(path.read_text(encoding="utf-8"))
    return np.asarray(data["waypoints"], dtype=np.float64)


def world_m_to_ijk(points_m: np.ndarray) -> np.ndarray:
    """Map MuJoCo/world meters to NRRD voxel indices.

    Header directions are (-1,0,0) (0,-1,0) (0,0,1) in millimeters, origin 0.
    """
    mm = points_m * 1000.0
    return np.column_stack([-mm[:, 0], -mm[:, 1], mm[:, 2]])


def sample_mask_values(mask: np.ndarray, points_m: np.ndarray) -> np.ndarray:
    ijk = world_m_to_ijk(points_m).T
    return ndimage.map_coordinates(mask.astype(np.float32), ijk, order=0, mode="constant", cval=0.0)


def sample_lumen_radii(mask: np.ndarray, points_m: np.ndarray) -> np.ndarray:
    # Distance from lumen voxels to the nearest non-lumen voxel.  Units are mm
    # because the header space directions are 1 mm on each axis.
    dist_mm = ndimage.distance_transform_edt(mask, sampling=(1.0, 1.0, 1.0))
    radii_mm = ndimage.map_coordinates(dist_mm, world_m_to_ijk(points_m).T, order=1, mode="constant", cval=0.0)
    return radii_mm * 1.0e-3


def resample_polyline(points: np.ndarray, ds: float) -> np.ndarray:
    seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    if cum[-1] <= 0.0:
        return points.copy()
    samples = np.arange(0.0, cum[-1], ds)
    if samples[-1] < cum[-1]:
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
    return np.asarray(out)


def parallel_frames(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
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
    binm = np.cross(tang, nrm)
    return tang, nrm, binm


def build_variable_tube_mesh(points: np.ndarray, radii: np.ndarray, wall_t: float, k_around: int) -> newton.Mesh:
    _, nrm, binm = parallel_frames(points)
    verts = []
    angles = np.linspace(0.0, 2.0 * math.pi, k_around, endpoint=False)
    for p, r, n, b in zip(points, radii, nrm, binm):
        for ring_r in (r, r + wall_t):
            for a in angles:
                d = math.cos(a) * n + math.sin(a) * b
                verts.append(p + ring_r * d)
    stride = 2 * k_around

    def vi(i: int, ring: int, k: int) -> int:
        return i * stride + ring * k_around + (k % k_around)

    tris: list[int] = []
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
            a, b, c, d = vi(i, 0, k), vi(i, 0, kn), vi(i, 1, kn), vi(i, 1, k)
            tris += [a, c, b, a, d, c] if flip else [a, b, c, a, c, d]
    return newton.Mesh(np.asarray(verts, dtype=np.float32), np.asarray(tris, dtype=np.int32), is_solid=True)


def sample_along(points: np.ndarray, length: float, seg_len: float) -> np.ndarray:
    seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    out = []
    for s in np.arange(0.0, length + 0.5 * seg_len, seg_len):
        idx = int(np.searchsorted(cum, s))
        if idx <= 0:
            out.append(points[0])
        elif idx >= len(points):
            out.append(points[-1])
        else:
            t = (s - cum[idx - 1]) / max(cum[idx] - cum[idx - 1], 1e-12)
            out.append(points[idx - 1] + t * (points[idx] - points[idx - 1]))
    return np.asarray(out)


def point_at_s(points: np.ndarray, s: float) -> np.ndarray:
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


def radial_margin(rod_xyz: np.ndarray, centerline: np.ndarray, radii: np.ndarray) -> float:
    """Return max rod-surface breach against variable-radius centerline tube."""
    a = centerline[:-1]
    b = centerline[1:]
    ab = b - a
    denom = np.einsum("ij,ij->i", ab, ab) + 1e-18
    worst = -1e9
    for p in rod_xyz:
        ap = p - a
        t = np.clip(np.einsum("ij,ij->i", ap, ab) / denom, 0.0, 1.0)
        proj = a + t[:, None] * ab
        d2 = np.sum((p - proj) ** 2, axis=1)
        i = int(np.argmin(d2))
        local_radius = (1.0 - t[i]) * radii[i] + t[i] * radii[i + 1]
        breach = math.sqrt(float(d2[i])) + ROD_RADIUS - float(local_radius)
        worst = max(worst, breach)
    return worst


def build_model(centerline: np.ndarray, radii: np.ndarray, args):
    builder = newton.ModelBuilder()
    builder.rigid_gap = 0.0
    builder.default_shape_cfg.density = DENSITY
    builder.default_shape_cfg.ke = args.contact_ke
    builder.default_shape_cfg.kd = 1.0
    builder.default_shape_cfg.mu = 0.2

    mesh = build_variable_tube_mesh(centerline, radii, args.wall_thickness, args.around)
    mesh.build_sdf(target_voxel_size=args.sdf_voxel, narrow_band_range=(-args.wall_thickness, float(radii.max())))
    wall_cfg = newton.ModelBuilder.ShapeConfig(ke=args.contact_ke, kd=1.0, mu=0.2)
    builder.add_shape_mesh(body=-1, mesh=mesh, cfg=wall_cfg, label="segment_part_radius_tube")

    rod_pts = sample_along(centerline, args.rod_length, ROD_SEG_LEN)
    rod_q = newton.utils.create_parallel_transport_cable_quaternions([wp.vec3(*p) for p in rod_pts], twist_total=0.0)
    rod_bodies, _ = builder.add_rod(
        positions=[wp.vec3(*p) for p in rod_pts],
        quaternions=rod_q,
        radius=ROD_RADIUS,
        stretch_stiffness=1.0e5,
        stretch_damping=0.0,
        bend_stiffness=args.bend,
        bend_damping=1.0e-1,
        label="guidewire",
    )
    root = rod_bodies[0]
    builder.body_mass[root] = 0.0
    builder.body_inv_mass[root] = 0.0
    builder.body_inertia[root] = wp.mat33(0.0)
    builder.body_inv_inertia[root] = wp.mat33(0.0)

    builder.color(balance_colors=False)
    model = builder.finalize()
    model.set_gravity((0.0, 0.0, 0.0))
    solver = newton.solvers.SolverVBD(model, iterations=args.iterations)
    return model, solver, model.state(), model.state(), model.control(), model.contacts(), rod_bodies


def run(args) -> bool:
    root = Path(args.root)
    centerline_raw = load_centerline(
        root / "src/cathsim/dm/components/phantom_assets/meshes/segment_part/centerline.json"
    )
    cache_path = Path(args.radius_cache)
    if cache_path.exists() and not args.rebuild_radius_cache:
        cached = np.load(cache_path)
        radii_raw = cached["radii_raw"]
        inside_frac = float(cached["inside_frac"])
        mask_shape = tuple(int(v) for v in cached["mask_shape"])
    else:
        mask = read_nrrd_mask(root / "data/aorta_centerline/Segmentation.seg.nrrd", args.nrrd_order)
        values = sample_mask_values(mask, centerline_raw)
        inside_frac = float((values > 0.5).mean())
        radii_raw = sample_lumen_radii(mask, centerline_raw)
        mask_shape = mask.shape
        cache_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(
            cache_path,
            radii_raw=radii_raw,
            inside_frac=np.asarray(inside_frac),
            mask_shape=np.asarray(mask_shape),
        )

    centerline = resample_polyline(centerline_raw, args.centerline_ds)
    radii = np.interp(
        np.linspace(0.0, 1.0, len(centerline)),
        np.linspace(0.0, 1.0, len(centerline_raw)),
        radii_raw,
    )
    radii = np.clip(radii - args.radius_margin, args.min_radius, args.max_radius)

    print(f"=== D2 segment_part radius tube on {wp.get_device()} (newton {newton.__version__}) ===")
    print(
        f"mask={mask_shape} nrrd_order={args.nrrd_order} "
        f"centerline_raw={len(centerline_raw)} inside_frac={inside_frac:.2%}"
    )
    print(
        f"radii(mm): min={radii.min()*1e3:.2f} p50={np.median(radii)*1e3:.2f} "
        f"max={radii.max()*1e3:.2f}  tube_nodes={len(centerline)}"
    )

    model, solver, s0, s1, ctrl, contacts, rod_bodies = build_model(centerline, radii, args)
    fps = 60
    sim_dt = (1.0 / fps) / args.substeps
    root_body = rod_bodies[0]
    initial_root = s0.body_q.numpy()[root_body].copy()
    entry_tangent = centerline[1] - centerline[0]
    entry_tangent /= np.linalg.norm(entry_tangent)
    sub_disp = args.push_speed * sim_dt

    tip0 = s0.body_q.numpy()[rod_bodies[-1], :3].copy()
    margins = []
    n = 0
    t0 = time.perf_counter()
    for _frame in range(args.frames):
        for _ in range(args.substeps):
            n += 1
            bq = s0.body_q.numpy()
            if args.drive == "centerline":
                bq[root_body, :3] = point_at_s(centerline, sub_disp * n)
            else:
                bq[root_body, :3] = initial_root[:3] + entry_tangent * (sub_disp * n)
            s0.body_q.assign(bq)
            s0.clear_forces()
            model.collide(s0, contacts)
            solver.step(s0, s1, ctrl, contacts, sim_dt)
            s0, s1 = s1, s0
        rod_xyz = s0.body_q.numpy()[rod_bodies, :3]
        margins.append(radial_margin(rod_xyz, centerline, radii))

    q = s0.body_q.numpy()
    wp.synchronize()
    elapsed = time.perf_counter() - t0
    tip1 = q[rod_bodies[-1], :3]
    finite = bool(np.isfinite(q).all())
    margins = np.asarray(margins)
    tail = margins[int(0.75 * len(margins)) :]
    steady = float(tail.mean())
    worst = float(margins.max())
    breach_frac = float((margins > 0.5e-3).mean())
    tip_adv = float(np.linalg.norm(tip1 - tip0))
    ok = finite and steady < 0.5e-3 and tip_adv > args.min_tip_advance
    print(
        f"push={args.push_speed:.2f}m/s frames={args.frames} ss={args.substeps} "
        f"tip_adv={tip_adv*1e3:.1f}mm steady={steady*1e3:+.3f}mm "
        f"worst={worst*1e3:+.2f}mm breach_frac={breach_frac:.0%} finite={finite}"
    )
    print(f"throughput={args.frames / elapsed:.1f} control-fps ({args.frames * args.substeps / elapsed:.0f} phys-steps/s)")
    print(f"D2 RADIUS-TUBE GATE: {'PASS' if ok else 'PARTIAL/FAIL'}")
    return ok


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=".")
    parser.add_argument("--nrrd-order", choices=["C", "F"], default="F")
    parser.add_argument("--radius-cache", default="spikes/d2_segment_part_radii_cache.npz")
    parser.add_argument("--rebuild-radius-cache", action="store_true")
    parser.add_argument("--centerline-ds", type=float, default=2.0e-3)
    parser.add_argument("--around", type=int, default=24)
    parser.add_argument("--wall-thickness", type=float, default=1.0e-2)
    parser.add_argument("--sdf-voxel", type=float, default=1.0e-3)
    parser.add_argument("--radius-margin", type=float, default=5.0e-4)
    parser.add_argument("--min-radius", type=float, default=1.5e-3)
    parser.add_argument("--max-radius", type=float, default=8.0e-3)
    parser.add_argument("--rod-length", type=float, default=ROD_LENGTH)
    parser.add_argument("--bend", type=float, default=5.0)
    parser.add_argument("--contact-ke", type=float, default=1.0e6)
    parser.add_argument("--iterations", type=int, default=8)
    parser.add_argument("--substeps", type=int, default=40)
    parser.add_argument("--push-speed", type=float, default=0.05)
    parser.add_argument("--drive", choices=["centerline", "entry_tangent"], default="centerline")
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--min-tip-advance", type=float, default=0.02)
    args = parser.parse_args()

    wp.init()
    ok = run(args)
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
