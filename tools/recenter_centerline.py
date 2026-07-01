"""Re-center a phantom's render centerline onto the lumen medial axis.

A phantom's ``meshes/<phantom>/centerline.json`` guided path can hug one vessel
wall (it was routed through the lumen, not computed as the medial axis), so the
rendered wire and the Newton demo tube sit off-center.  This tool pushes each
interior waypoint toward the local lumen center, then lightly re-smooths.

Centering method (per waypoint, a few relaxation passes):
  cast a fan of rays in the plane perpendicular to the local tangent against the
  vessel surface (the Godot ``<phantom>.glb``, which shares the mujoco-meter
  frame); for each opposite ray pair the centered offset along that axis is half
  the difference of the two wall hits.  Averaging over axes moves the point to
  where opposite walls are equidistant = the local center.  Endpoints (entry /
  target_root) stay fixed; steps are clamped to the nearest wall so a point never
  crosses out of the lumen.

Vessel mesh and physics geometry are untouched; only the render/guided path moves.

Usage:
    python tools/recenter_centerline.py segment_part --report      # measure only
    python tools/recenter_centerline.py segment_part --write       # recenter + save (backs up .bak)
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np
import trimesh

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

_MESH_DIR = _ROOT / "src/cathsim/dm/components/phantom_assets/meshes"
_GLB_DIR = _ROOT / "godot_client/assets/models"

N_RAYS = 24


def _tangents(pts: np.ndarray) -> np.ndarray:
    t = np.zeros_like(pts)
    t[1:-1] = pts[2:] - pts[:-2]
    t[0] = pts[1] - pts[0]
    t[-1] = pts[-1] - pts[-2]
    t /= np.linalg.norm(t, axis=1, keepdims=True) + 1e-12
    return t


def _perp_basis(t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    ref = np.array([0.0, 1.0, 0.0]) if abs(t[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(t, ref)
    u /= np.linalg.norm(u) + 1e-12
    return u, np.cross(t, u)


def _ray_dists(mesh: trimesh.Trimesh, p: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    angs = np.linspace(0.0, 2.0 * math.pi, N_RAYS, endpoint=False)
    dirs = np.array([math.cos(a) * u + math.sin(a) * v for a in angs])
    locs, ridx, _ = mesh.ray.intersects_location(
        ray_origins=np.tile(p, (N_RAYS, 1)), ray_directions=dirs, multiple_hits=False)
    r = np.full(N_RAYS, np.nan)
    for loc, ri in zip(locs, ridx):
        d = float(np.linalg.norm(loc - p))
        if np.isnan(r[ri]) or d < r[ri]:
            r[ri] = d
    return dirs, r


def _local_mask(r: np.ndarray) -> np.ndarray:
    """Valid rays that hit the *local* lumen wall (reject escapes down branches/axis:
    distances far above the local median are not this cross-section's wall)."""
    val = ~np.isnan(r)
    if val.sum() == 0:
        return val
    med = np.median(r[val])
    return val & (r < 3.0 * med)


def _offset_ratio(mesh: trimesh.Trimesh, pts: np.ndarray) -> tuple[float, float]:
    """Mean offset/radius and mean nearest-wall distance (mm) over sample points."""
    ratios, nearest = [], []
    t = _tangents(pts)
    for i in np.linspace(3, len(pts) - 4, 30).astype(int):
        u, v = _perp_basis(t[i])
        dirs, r = _ray_dists(mesh, pts[i], u, v)
        m = _local_mask(r)
        if m.sum() < N_RAYS * 0.4:
            continue
        rr = r[m]
        off = np.linalg.norm((pts[i] + dirs[m] * rr[:, None]).mean(0) - pts[i])
        ratios.append(off / rr.mean())
        nearest.append(rr.min())
    return float(np.mean(ratios)), float(np.mean(nearest) * 1e3)


def _smooth_field(vecs: np.ndarray, window: int) -> np.ndarray:
    """Moving-average a per-point vector field along the path (keep ends)."""
    if window < 2:
        return vecs
    k = np.ones(window) / window
    out = vecs.copy()
    for c in range(vecs.shape[1]):
        out[:, c] = np.convolve(vecs[:, c], k, mode="same")
    return out


def _recenter_pass(mesh: trimesh.Trimesh, pts: np.ndarray, relax: float,
                   window: int = 15, abs_clamp: float = 1.5e-3) -> np.ndarray:
    """One pass: compute each interior point's center-seeking offset, low-pass the
    offset field along the path so neighbours move coherently (centered *and*
    smooth), then apply clamped to the nearest wall."""
    t = _tangents(pts)
    half = N_RAYS // 2
    deltas = np.zeros_like(pts)
    near = np.full(len(pts), np.inf)
    for i in range(len(pts)):  # include endpoints: the entry/target must center too
        u, v = _perp_basis(t[i])
        dirs, r = _ray_dists(mesh, pts[i], u, v)
        m = _local_mask(r)  # only local lumen walls, no branch escapes
        rec = np.zeros(3)
        cnt = 0
        for k in range(half):
            j = k + half
            if m[k] and m[j]:
                rec += 0.5 * (r[k] - r[j]) * dirs[k]
                cnt += 1
        if cnt:
            deltas[i] = rec / cnt
            near[i] = r[m].min()
    deltas = _smooth_field(deltas, window)
    out = pts.copy()
    for i in range(len(pts)):
        step = relax * deltas[i]
        sn = np.linalg.norm(step)
        cap = min(0.8 * near[i], abs_clamp)  # never past the wall, nor a big jump
        if sn > cap:
            step *= cap / sn
        out[i] = pts[i] + step
    return out


def _laplacian(pts: np.ndarray, lam: float = 0.3, iters: int = 2) -> np.ndarray:
    """Gentle Laplacian smoothing, endpoints fixed. Unconditionally stable (no spline
    overshoot, which on this branched path produced 160deg cusps)."""
    pts = pts.copy()
    for _ in range(iters):
        mid = 0.5 * (pts[:-2] + pts[2:])
        pts[1:-1] = pts[1:-1] + lam * (mid - pts[1:-1])
    return pts


def _despike(pts: np.ndarray, thresh_deg: float = 25.0, iters: int = 20) -> np.ndarray:
    """Relax only the cusp points (turn > thresh) onto their neighbour midpoint.
    Branch junctions are the only places recentering overshoots; this leaves the
    rest of the (now well-centered, smooth) path untouched."""
    pts = pts.copy()
    for _ in range(iters):
        moved = False
        for i in range(1, len(pts) - 1):
            v1 = pts[i] - pts[i - 1]
            v2 = pts[i + 1] - pts[i]
            n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
            if n1 > 1e-9 and n2 > 1e-9:
                ang = math.degrees(math.acos(np.clip(np.dot(v1, v2) / (n1 * n2), -1, 1)))
                if ang > thresh_deg:
                    pts[i] = 0.5 * (pts[i - 1] + pts[i + 1])
                    moved = True
        if not moved:
            break
    return pts


def _perp_bases(pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Vectorized orthonormal (u, v) spanning the plane perpendicular to the tangent."""
    t = _tangents(pts)
    ref = np.tile(np.array([0.0, 1.0, 0.0]), (len(pts), 1))
    ref[np.abs(t[:, 1]) >= 0.9] = np.array([1.0, 0.0, 0.0])
    u = np.cross(t, ref)
    u /= np.linalg.norm(u, axis=1, keepdims=True) + 1e-12
    v = np.cross(t, u)
    return u, v


def _dist(pq, P: np.ndarray) -> np.ndarray:
    """Unsigned nearest distance to the vessel surface (fast: skips the inside/outside
    test). Equivalent to |signed| here because points start inside the lumen and the
    ascent only moves them further from walls, so they never leave the lumen."""
    _, dist, _ = pq.on_surface(P)
    return dist


def _sdf_targets(pq, pts: np.ndarray,
                 micro: int = 14, eps: float = 4.0e-4, lr: float = 1.0) -> np.ndarray:
    """Gradient-ASCEND each point on the distance-to-wall field, confined to the plane
    perpendicular to the local tangent, onto the lumen medial ridge (local max of
    distance-to-wall). Robust where the ray-fan fails: at the open mouth the lateral
    walls still give a well-defined gradient toward center, and at branches ascent
    climbs to the *local* thickest point instead of escaping down a branch.

    All four finite-difference samples are batched into one proximity query per step."""
    u, v = _perp_bases(pts)
    q = pts.copy()
    N = len(pts)
    for _ in range(micro):
        samples = np.vstack([q + eps * u, q - eps * u, q + eps * v, q - eps * v])
        d = _dist(pq, samples)
        gu = (d[:N] - d[N:2 * N]) / (2 * eps)
        gv = (d[2 * N:3 * N] - d[3 * N:]) / (2 * eps)
        g = gu[:, None] * u + gv[:, None] * v       # in-plane gradient of distance-to-wall
        gn = np.linalg.norm(g, axis=1)
        dirn = g / (gn[:, None] + 1e-9)              # unit ascent direction
        active = gn > 1e-3                           # at the ridge the gradient vanishes -> stop
        q = q + (lr * eps) * dirn * active[:, None]
    return q


def _sdf_recenter_pass(pq, pts: np.ndarray, relax: float,
                       window: int = 15, abs_clamp: float = 2.5e-3) -> np.ndarray:
    """One SDF-based pass: ascend to medial targets, low-pass the displacement field
    along the path (coherent, no zigzag), then apply clamped."""
    targets = _sdf_targets(pq, pts)
    deltas = _smooth_field(targets - pts, window)
    out = pts.copy()
    for i in range(len(pts)):
        step = relax * deltas[i]
        sn = np.linalg.norm(step)
        if sn > abs_clamp:
            step *= abs_clamp / sn
        out[i] = pts[i] + step
    return out


def _wall_report(pq, pts: np.ndarray) -> str:
    """Nearest-wall stats via proximity query (works at the mouth too)."""
    d = _dist(pq, pts) * 1e3
    return (f"nearest_wall mean={d.mean():.2f}mm min={d.min():.2f}mm "
            f"first8_min={d[:8].min():.2f}mm n<0.5mm={(d < 0.5).sum()}")


def _extrapolate_ends(pq, pts: np.ndarray, min_wall: float = 1.5e-3, max_pts: int = 24) -> np.ndarray:
    """Fix the open-mouth points that gradient ascent cannot center (rays/gradient are
    ill-posed at an open end). Project the leading/trailing near-wall points laterally
    onto the medial line of the first/last *well-centered* point (axial position kept,
    so insertion depth is unchanged)."""
    pts = pts.copy()
    d = _dist(pq, pts)
    n = len(pts)
    k = next((i for i in range(min(max_pts, n)) if d[i] > min_wall), None)
    if k is not None and k > 0:
        tang = pts[k + 1] - pts[k]
        tang /= np.linalg.norm(tang) + 1e-12
        for i in range(k):
            pts[i] = pts[k] + np.dot(pts[i] - pts[k], tang) * tang  # drop lateral offset
    j = next((i for i in range(n - 1, max(n - 1 - max_pts, -1), -1) if d[i] > min_wall), None)
    if j is not None and j < n - 1:
        tang = pts[j] - pts[j - 1]
        tang /= np.linalg.norm(tang) + 1e-12
        for i in range(j + 1, n):
            pts[i] = pts[j] + np.dot(pts[i] - pts[j], tang) * tang
    return pts


def _metrics(pts: np.ndarray) -> dict:
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    length = float(seg.sum())
    angs = []
    for i in range(1, len(pts) - 1):
        v1 = pts[i] - pts[i - 1]
        v2 = pts[i + 1] - pts[i]
        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if n1 > 1e-9 and n2 > 1e-9:
            angs.append(math.degrees(math.acos(np.clip(np.dot(v1, v2) / (n1 * n2), -1, 1))))
    angs = np.array(angs) if angs else np.array([0.0])
    return {"length_m": length, "max_turn_deg": float(angs.max()),
            "p50_turn": float(np.percentile(angs, 50)), "p95_turn": float(np.percentile(angs, 95)),
            "n_over_30": int((angs > 30).sum())}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("phantom")
    ap.add_argument("--method", choices=["sdf", "ray"], default="sdf",
                    help="sdf = signed-distance gradient ascent (robust at mouth/branches); ray = ray-fan")
    ap.add_argument("--iters", type=int, default=3)
    ap.add_argument("--relax", type=float, default=0.6)
    ap.add_argument("--smoothing", type=float, default=2e-6)
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--report", action="store_true")
    args = ap.parse_args()

    cl_path = _MESH_DIR / args.phantom / "centerline.json"
    glb_path = _GLB_DIR / f"{args.phantom}.glb"
    data = json.loads(cl_path.read_text())
    pts0 = np.array(data["waypoints"], dtype=float)
    scene = trimesh.load(glb_path, process=False)
    mesh = trimesh.util.concatenate(scene.dump()) if isinstance(scene, trimesh.Scene) else scene

    pq = trimesh.proximity.ProximityQuery(mesh)  # built once, reused across passes
    target = pq if args.method == "sdf" else mesh
    pass_fn = _sdf_recenter_pass if args.method == "sdf" else _recenter_pass

    m0 = _metrics(pts0)
    print(f"[{args.phantom}] before ({args.method}): {_wall_report(pq, pts0)}")
    print(f"            turn p50={m0['p50_turn']:.1f} p95={m0['p95_turn']:.1f} "
          f"max={m0['max_turn_deg']:.1f}deg n>30deg={m0['n_over_30']}")

    pts = pts0.copy()
    for it in range(args.iters):
        pts = pass_fn(target, pts, args.relax)
        pts = _laplacian(pts, lam=0.3, iters=2)  # keep smooth each iter (no spline overshoot)
    pts = _despike(pts)                           # relax any branch-junction cusps
    pts = _laplacian(pts, lam=0.3, iters=3)
    if args.method == "sdf":
        pts = _extrapolate_ends(pq, pts)          # center the open-mouth points

    m1 = _metrics(pts)
    print(f"[{args.phantom}] after  ({args.method}): {_wall_report(pq, pts)}")
    print(f"            turn p50={m1['p50_turn']:.1f} p95={m1['p95_turn']:.1f} "
          f"max={m1['max_turn_deg']:.1f}deg n>30deg={m1['n_over_30']}  length={m1['length_m']:.3f}m")

    if args.write:
        bak = cl_path.with_suffix(".json.bak")
        if not bak.exists():  # never clobber the true original on a re-run
            shutil.copy(cl_path, bak)
        data["waypoints"] = [[float(c) for c in p] for p in pts]
        # entry / target_root follow the now-centered endpoints (kept lateral-only,
        # so the insertion depth is unchanged; only the off-wall bias is removed).
        data["entry"] = [float(c) for c in pts[0]]
        data["target_root"] = [float(c) for c in pts[-1]]
        data["length_m"] = m1["length_m"]
        data["smooth"] = {**data.get("smooth", {}),
                          "recentered": True, "recenter_iters": args.iters,
                          "recenter_smoothing": args.smoothing}
        cl_path.write_text(json.dumps(data, indent=2))
        print(f"  wrote {cl_path}  (backup {cl_path.with_suffix('.json.bak').name})")
    else:
        print("  (report only; pass --write to save)")


if __name__ == "__main__":
    main()
