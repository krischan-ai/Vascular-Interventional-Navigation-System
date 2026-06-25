#!/usr/bin/env python
"""Build a *sealed-lumen* MuJoCo phantom from a radius-bearing centerline.

Why this exists
---------------
The previous blocker for physical guidewire navigation was containment: a
phantom whose wall is the V-HACD convex decomposition of a thin tortuous vessel
*surface* cannot seal the lumen -- convex hulls bulge inward or leave gaps, so a
pre-threaded wire springs out (measured dev 164mm) or falls out under gravity
(234mm). V-HACD is the wrong tool for a hollow pipe.

This generator instead builds the wall *analytically* from a centerline that
carries a per-point radius (e.g. 3D Slicer / VMTK ``*.mrk.json`` with a
``Radius`` measurement). It sweeps an **annular wall** along the centerline and
slices it into a grid of ``axial segment x angular sector`` convex hexahedral
"bricks". Adjacent bricks share exact vertices, so their union is watertight and
truly seals the lumen, and every brick is convex -> native MuJoCo mesh geoms,
no V-HACD, deterministic.

Output (under ``src/cathsim/dm/components/phantom_assets/``):
  * ``meshes/<name>/hull_*.stl``  -- convex wall bricks (collision)
  * ``meshes/<name>/visual.stl``  -- inner lumen surface (transparent visual)
  * ``meshes/<name>/centerline.json`` -- entry/target_root/waypoints (+radius)
  * ``<name>.xml`` -- phantom MJCF (segment_part-style: geoms in worldbody)

Coordinates: Slicer markups are LPS millimetres; like the existing VPP pipeline
we map to MuJoCo metres by dividing by 1000 with no axis flip (verified: the
aorta data registers onto segment_part to ~5mm in this frame).

Usage:
    python -m tools.build_tube_phantom \
        --curve "data/aorta_centerline/Centerline curve (10).mrk.json" \
        --name aorta_trunk --seg 0.004 --sectors 8 --wall 0.003
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import trimesh

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PHANTOM_ASSETS = PROJECT_ROOT / "src" / "cathsim" / "dm" / "components" / "phantom_assets"


# --------------------------------------------------------------------------
# Centerline loading (3D Slicer markups .mrk.json with a Radius measurement)
# --------------------------------------------------------------------------
def load_slicer_curve(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Return (points_m, radius_m) from a Slicer curve markup.

    Points are LPS millimetres -> metres (/1000, no flip). Radius comes from the
    per-control-point ``Radius`` measurement (also mm -> m).
    """
    data = json.loads(path.read_text(encoding="utf-8"))
    markup = data["markups"][0]
    pts = np.array([cp["position"] for cp in markup["controlPoints"]], dtype=np.float64)
    radius = None
    for m in markup.get("measurements", []):
        if m.get("name") == "Radius" and m.get("controlPointValues"):
            radius = np.asarray(m["controlPointValues"], dtype=np.float64)
    if radius is None:
        raise ValueError(f"{path.name} has no per-point Radius measurement")
    if len(radius) != len(pts):
        raise ValueError("Radius count does not match control point count")
    return pts / 1000.0, radius / 1000.0


def resample_by_arclen(
    pts: np.ndarray, radius: np.ndarray, seg: float
) -> tuple[np.ndarray, np.ndarray]:
    """Resample the polyline at uniform arc-length ``seg`` (m), interpolating radius."""
    d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(d)])
    total = float(cum[-1])
    n = max(int(round(total / seg)) + 1, 2)
    s = np.linspace(0.0, total, n)
    out_pts = np.empty((n, 3))
    for k in range(3):
        out_pts[:, k] = np.interp(s, cum, pts[:, k])
    out_rad = np.interp(s, cum, radius)
    return out_pts, out_rad


# --------------------------------------------------------------------------
# Geometry: rotation-minimizing frames + annular wall bricks
# --------------------------------------------------------------------------
def tangents(pts: np.ndarray) -> np.ndarray:
    """Unit tangents via central differences."""
    t = np.gradient(pts, axis=0)
    norms = np.linalg.norm(t, axis=1, keepdims=True)
    norms[norms < 1e-12] = 1.0
    return t / norms


def rmf_normals(pts: np.ndarray, tan: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Rotation-minimizing frame (normal, binormal) via double-reflection.

    Avoids the twist a naive per-section frame would introduce, keeping the
    swept wall from wrinkling at bends.
    """
    n = len(pts)
    normal = np.zeros((n, 3))
    binormal = np.zeros((n, 3))
    # Seed normal: any unit vector perpendicular to the first tangent.
    ref = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(ref, tan[0])) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    normal[0] = np.cross(tan[0], ref)
    normal[0] /= np.linalg.norm(normal[0])
    binormal[0] = np.cross(tan[0], normal[0])
    for i in range(n - 1):
        # Double-reflection (Wang et al. 2008) to transport the frame.
        v1 = pts[i + 1] - pts[i]
        c1 = float(np.dot(v1, v1))
        if c1 < 1e-18:
            normal[i + 1] = normal[i]
            binormal[i + 1] = binormal[i]
            continue
        n_l = normal[i] - (2.0 / c1) * np.dot(v1, normal[i]) * v1
        t_l = tan[i] - (2.0 / c1) * np.dot(v1, tan[i]) * v1
        v2 = tan[i + 1] - t_l
        c2 = float(np.dot(v2, v2))
        n_next = n_l if c2 < 1e-18 else n_l - (2.0 / c2) * np.dot(v2, n_l) * v2
        n_next /= np.linalg.norm(n_next)
        normal[i + 1] = n_next
        binormal[i + 1] = np.cross(tan[i + 1], n_next)
    return normal, binormal


def ring(center, normal, binormal, radius, sectors) -> np.ndarray:
    """Return ``sectors`` points on a circle of ``radius`` in the (normal,binormal) plane."""
    ang = np.linspace(0.0, 2.0 * np.pi, sectors, endpoint=False)
    return center + radius * (np.outer(np.cos(ang), normal) + np.outer(np.sin(ang), binormal))


def build_wall_bricks(
    pts, rad, normal, binormal, sectors, wall
) -> list[trimesh.Trimesh]:
    """Slice the annular wall into convex hexahedral bricks (one per axial x sector cell).

    Each brick is the convex hull of 8 vertices (inner/outer x ring i/i+1 x
    sector k/k+1). Bricks share vertices with their neighbours, so the union is
    watertight and seals the lumen. ``wall`` is the radial wall thickness (m).
    """
    n = len(pts)
    inner = [ring(pts[i], normal[i], binormal[i], rad[i], sectors) for i in range(n)]
    outer = [ring(pts[i], normal[i], binormal[i], rad[i] + wall, sectors) for i in range(n)]
    bricks: list[trimesh.Trimesh] = []
    for i in range(n - 1):
        for k in range(sectors):
            k2 = (k + 1) % sectors
            verts = np.array([
                inner[i][k], inner[i][k2], inner[i + 1][k], inner[i + 1][k2],
                outer[i][k], outer[i][k2], outer[i + 1][k], outer[i + 1][k2],
            ])
            hull = trimesh.Trimesh(vertices=verts, process=True).convex_hull
            if hull.volume > 1e-12:
                bricks.append(hull)
    return bricks


def smooth_render_path(
    pts: np.ndarray, rad: np.ndarray, smooth_factor: float, num_points: int | None
) -> tuple[np.ndarray, np.ndarray]:
    """B-spline smooth the centerline for the render / planned path.

    Reuses the same scipy B-spline as ``services.path_planner.PathPlanner`` (the
    server's single source of smoothing truth) so the frontend render path is a
    smooth curve, not the raw 4mm resample polyline. The per-point radius is
    re-interpolated by arc length onto the denser smooth path so it stays aligned
    with ``waypoints``.

    ``smooth_factor=0`` -> interpolating spline: the curve passes through every
    resampled point, so it stays inside the wall bricks built from those points
    (safe for pre-threading). ``>0`` trades fidelity for smoothness.
    """
    from services.path_planner import PathPlanner

    if len(pts) < 4:
        return pts, rad
    result = PathPlanner().smooth_path(
        [tuple(p) for p in pts],
        smoothing_factor=smooth_factor,
        num_points=num_points,
    )
    s_pts = np.asarray(result["waypoints"], dtype=np.float64)
    # Re-interpolate radius by arc length onto the smooth path.
    src_cum = np.concatenate(
        [[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    )
    dst_cum = np.concatenate(
        [[0.0], np.cumsum(np.linalg.norm(np.diff(s_pts, axis=0), axis=1))]
    )
    if src_cum[-1] > 0 and dst_cum[-1] > 0:
        s_rad = np.interp(dst_cum / dst_cum[-1], src_cum / src_cum[-1], rad)
    else:
        s_rad = np.full(len(s_pts), float(np.median(rad)))
    return s_pts, s_rad


def build_inner_surface(pts, rad, normal, binormal, sectors) -> trimesh.Trimesh:
    """Inner lumen surface mesh (for transparent visual)."""
    n = len(pts)
    rings = [ring(pts[i], normal[i], binormal[i], rad[i], sectors) for i in range(n)]
    verts = np.vstack(rings)
    faces = []
    for i in range(n - 1):
        base, nxt = i * sectors, (i + 1) * sectors
        for k in range(sectors):
            k2 = (k + 1) % sectors
            faces.append([base + k, base + k2, nxt + k])
            faces.append([base + k2, nxt + k2, nxt + k])
    return trimesh.Trimesh(vertices=verts, faces=np.array(faces), process=True)


# --------------------------------------------------------------------------
# Asset writing
# --------------------------------------------------------------------------
def write_xml(name: str, n_hulls: int, entry: np.ndarray, target: np.ndarray) -> Path:
    lines = [
        f'<mujoco model="{name}">',
        '  <compiler meshdir="meshes" />',
        '  <default>',
        '    <geom type="mesh" />',
        '  </default>',
        '  <asset>',
        f'    <mesh name="visual" file="{name}/visual.stl" />',
    ]
    for i in range(n_hulls):
        lines.append(f'    <mesh name="hull_{i}" file="{name}/hull_{i}.stl" />')
    lines += ['  </asset>', '  <worldbody>',
              # Visual only: contype/conaffinity 0 so MuJoCo does NOT collide its
              # convex hull (which would be a solid blob filling the whole curved
              # tube and eject the wire). Collision is the sealed wall bricks only.
              '    <geom type="mesh" mesh="visual" rgba=".8 .5 .3 .3" group="0" '
              'contype="0" conaffinity="0" />']
    for i in range(n_hulls):
        lines.append(
            f'    <geom type="mesh" mesh="hull_{i}" rgba=".8 .5 .3 .0" '
            'group="1" condim="1" friction="1." />'
        )
    e = " ".join(f"{v:.6f}" for v in entry)
    t = " ".join(f"{v:.6f}" for v in target)
    lines += [
        f'    <!-- Guidewire entry landmark (proximal end of the aortic trunk centerline). -->',
        f'    <site name="entry" pos="{e}" />',
        f'    <!-- Navigation target (distal end of the trunk centerline). -->',
        f'    <site name="root" pos="{t}" />',
        '  </worldbody>',
        '</mujoco>',
        '',
    ]
    out = PHANTOM_ASSETS / f"{name}.xml"
    out.write_text("\n".join(lines), encoding="utf-8")
    return out


def write_centerline_json(
    name: str,
    mesh_dir: Path,
    pts: np.ndarray,
    rad: np.ndarray,
    smooth_factor: float,
) -> None:
    """Write centerline.json whose ``waypoints`` are the B-spline render path.

    ``waypoints`` (the planned + frontend render path) is the smoothed curve;
    ``radius_m`` is aligned to it. ``smooth`` / ``smooth_factor`` record how it
    was produced so the asset is self-describing.
    """
    length = float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())
    payload = {
        "phantom": name,
        "coordinate_system": "mujoco",
        "unit": "m",
        "source": "data/aorta_centerline (3D Slicer VMTK, LPS mm /1000)",
        "smooth": True,
        "smooth_factor": smooth_factor,
        "entry": [round(float(v), 6) for v in pts[0]],
        "target_root": [round(float(v), 6) for v in pts[-1]],
        "length_m": round(length, 5),
        "radius_m": [round(float(r), 6) for r in rad],
        "waypoints": [[round(float(v), 6) for v in p] for p in pts],
    }
    (mesh_dir / "centerline.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--curve", required=True, help="path to a Slicer curve .mrk.json")
    ap.add_argument("--name", required=True, help="phantom name (e.g. aorta_trunk)")
    ap.add_argument("--seg", type=float, default=0.004, help="axial resample step (m)")
    ap.add_argument("--sectors", type=int, default=8, help="angular sectors per ring")
    ap.add_argument("--wall", type=float, default=0.003, help="radial wall thickness (m)")
    ap.add_argument("--radius-scale", type=float, default=1.0, help="scale lumen radius")
    ap.add_argument("--radius-min", type=float, default=0.0, help="floor on lumen radius (m)")
    ap.add_argument("--reverse", action="store_true", help="swap entry/target ends")
    ap.add_argument(
        "--smooth-factor", type=float, default=0.0,
        help="B-spline smoothing for the render/planned path "
             "(0=interpolating: passes through points, stays in lumen; >0 smoother)",
    )
    ap.add_argument(
        "--render-points", type=int, default=None,
        help="number of points in the smoothed render path (default 2x rings)",
    )
    args = ap.parse_args()

    curve_path = Path(args.curve)
    if not curve_path.is_absolute():
        curve_path = PROJECT_ROOT / curve_path
    print(f"[load] {curve_path.name}")
    pts, rad = load_slicer_curve(curve_path)
    if args.reverse:
        pts, rad = pts[::-1].copy(), rad[::-1].copy()
    print(f"  control points={len(pts)} radius mm: "
          f"min={rad.min()*1000:.2f} med={np.median(rad)*1000:.2f} max={rad.max()*1000:.2f}")

    pts, rad = resample_by_arclen(pts, rad, args.seg)
    rad = np.maximum(rad * args.radius_scale, args.radius_min)
    print(f"[resample] {len(pts)} rings @ {args.seg*1000:.1f}mm "
          f"(total {np.linalg.norm(np.diff(pts,axis=0),axis=1).sum()*1000:.1f}mm)")

    tan = tangents(pts)
    normal, binormal = rmf_normals(pts, tan)

    print(f"[wall] sectors={args.sectors} wall={args.wall*1000:.1f}mm")
    bricks = build_wall_bricks(pts, rad, normal, binormal, args.sectors, args.wall)
    print(f"  built {len(bricks)} convex wall bricks")
    visual = build_inner_surface(pts, rad, normal, binormal, args.sectors)
    print(f"  visual watertight={visual.is_watertight}")

    # B-spline smooth the centerline for the render / planned path (frontend
    # renders engine.planned_path; this makes it a smooth curve, not the raw
    # 4mm resample). Walls + pre-thread still use the raw resampled points.
    s_pts, s_rad = smooth_render_path(pts, rad, args.smooth_factor, args.render_points)
    print(f"[render path] B-spline smooth: {len(pts)} -> {len(s_pts)} pts "
          f"(smooth_factor={args.smooth_factor})")

    mesh_dir = PHANTOM_ASSETS / "meshes" / args.name
    mesh_dir.mkdir(parents=True, exist_ok=True)
    for f in mesh_dir.glob("hull_*.stl"):
        f.unlink()
    visual.export(str(mesh_dir / "visual.stl"), file_type="stl")
    for i, b in enumerate(bricks):
        b.export(str(mesh_dir / f"hull_{i}.stl"), file_type="stl")
    write_centerline_json(args.name, mesh_dir, s_pts, s_rad, args.smooth_factor)
    xml = write_xml(args.name, len(bricks), s_pts[0], s_pts[-1])

    print(f"[done] phantom '{args.name}': {len(bricks)} hulls -> {xml}")
    print(f"  entry={np.round(s_pts[0],3)}  target={np.round(s_pts[-1],3)}")
    print(f"  NOTE: register '{args.name}' in NavigationEngine.VALID_PHANTOMS to use it.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
