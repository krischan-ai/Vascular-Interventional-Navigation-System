#!/usr/bin/env python
"""Regenerate segment_part's guided render centerline (entry -> root) on a FINE grid.

Why this exists
---------------
The committed ``centerline.json`` was built by voxelizing ``visual.stl`` at a
**1.5 mm** pitch, skeletonizing, taking the entry->root geodesic and B-spline
smoothing it (doc 05 §24.3).  But the lumen here is only ~0.87 mm in radius
(~1.74 mm diameter) -- i.e. about *one voxel wide*.  A skeleton snapped to a
1.5 mm grid therefore carries a half-voxel (+-0.75 mm) discretization error that
is as large as the lumen radius, so the centerline sits off-axis / pokes through
the wall, biased toward one side.  Heavier B-spline smoothing then averages this
off-center skeleton without re-centering it.

This tool rebuilds the path on a pitch fine enough that the discretization error
is small versus the lumen radius (default 0.5 mm -> +-0.25 mm << 0.87 mm), so the
raw geodesic already sits inside the lumen, and verifies the result against the
GLB lumen surface (signed distance) before writing.

Vessel mesh / physics are untouched -- only ``centerline.json`` (the render/guided
path) changes.

Usage:
    python tools/gen_segment_part_centerline.py --pitch 5e-4 --report
    python tools/gen_segment_part_centerline.py --pitch 5e-4 --write
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

import networkx as nx
import numpy as np
import trimesh
from skimage.morphology import skeletonize

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from services.path_planner import PathPlanner  # noqa: E402

_MESH_DIR = _ROOT / "src/cathsim/dm/components/phantom_assets/meshes"
_GLB_DIR = _ROOT / "godot_client/assets/models"

# entry / root landmarks (single source of truth = segment_part.xml <site>s)
ENTRY_M = np.array([-0.824, -0.223, 0.263])
ROOT_M = np.array([-1.028, -0.260, 0.138])

_OFFSETS = [
    (a, b, c)
    for a in (-1, 0, 1)
    for b in (-1, 0, 1)
    for c in (-1, 0, 1)
    if (a, b, c) != (0, 0, 0)
]


def skeleton_graph(volume: np.ndarray):
    """Skeletonize a filled volume and return (skeleton voxel indices, nx.Graph
    over them with 26-connectivity, edges weighted by voxel-offset length)."""
    skel = skeletonize(volume)
    pts = np.argwhere(skel)
    skel_set = set(map(tuple, pts))
    G = nx.Graph()
    G.add_nodes_from(skel_set)
    for p in skel_set:
        for off in _OFFSETS:
            q = (p[0] + off[0], p[1] + off[1], p[2] + off[2])
            if q in skel_set:
                G.add_edge(p, q, weight=float(np.linalg.norm(off)))
    return pts, G


def geodesic_voxels(G, pts, grid, start_m, goal_m) -> np.ndarray:
    """Shortest skeleton path (voxel coords) between the skeleton voxels nearest
    to start_m and goal_m (world meters)."""
    world = grid.indices_to_points(pts.astype(float))
    s = tuple(pts[int(np.argmin(np.linalg.norm(world - start_m, axis=1)))])
    g = tuple(pts[int(np.argmin(np.linalg.norm(world - goal_m, axis=1)))])
    path = nx.shortest_path(G, s, g, weight="weight")
    return np.array(path, dtype=float)


def downsample(world_pts: np.ndarray, min_dist: float) -> np.ndarray:
    keep = [world_pts[0]]
    for p in world_pts[1:]:
        if np.linalg.norm(p - keep[-1]) > min_dist:
            keep.append(p)
    keep[-1] = world_pts[-1]  # always keep the true end
    return np.array(keep)


def wall_report(pq, pts: np.ndarray, label: str) -> np.ndarray:
    sd = pq.signed_distance(pts) * 1e3  # +inside, mm
    print(
        f"  {label:<22} n={len(pts):4d}  signed mean={sd.mean():+.2f} "
        f"min={sd.min():+.2f} max={sd.max():+.2f} mm  outside={int((sd < 0).sum())} "
        f"({100 * (sd < 0).mean():.0f}%)"
    )
    return sd


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--pitch", type=float, default=5e-4, help="voxel pitch (m), default 0.5mm")
    ap.add_argument("--downsample", type=float, default=1e-3, help="min waypoint spacing (m)")
    ap.add_argument("--smoothing", type=float, default=2e-7,
                    help="B-spline smoothing_factor (s = factor * n_points); keep small so "
                         "the curve tracks the fine skeleton instead of averaging off-center")
    ap.add_argument("--num-points", type=int, default=332, help="output waypoint count")
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--report", action="store_true")
    args = ap.parse_args()

    visual = _MESH_DIR / "segment_part/visual.stl"
    glb = _GLB_DIR / "segment_part.glb"
    cl_path = _MESH_DIR / "segment_part/centerline.json"

    print(f"[load] {visual.name}  pitch={args.pitch * 1e3:.2f}mm")
    mesh = trimesh.load(visual, process=False)
    grid = mesh.voxelized(args.pitch).fill()
    volume = np.asarray(grid.matrix, dtype=bool)
    print(f"  volume voxels: {volume.shape} ({volume.size / 1e6:.0f}M cells, {int(volume.sum())} filled)")

    pts_vox, G = skeleton_graph(volume)
    print(f"  skeleton voxels: {len(pts_vox)}  graph edges: {G.number_of_edges()}")

    path_vox = geodesic_voxels(G, pts_vox, grid, ENTRY_M, ROOT_M)
    raw_world = grid.indices_to_points(path_vox)
    ds = downsample(raw_world, args.downsample)
    print(f"  geodesic: {len(path_vox)} voxels -> {len(ds)} downsampled")

    planner = PathPlanner.__new__(PathPlanner)
    res = planner.smooth_path(
        [list(p) for p in ds], smoothing_factor=args.smoothing, num_points=args.num_points
    )
    smooth = np.asarray(res["waypoints"], dtype=float)

    scene = trimesh.load(glb, process=False)
    glb_mesh = trimesh.util.concatenate(scene.dump()) if isinstance(scene, trimesh.Scene) else scene
    pq = trimesh.proximity.ProximityQuery(glb_mesh)

    print("[verify] signed distance to lumen wall (+ = inside):")
    wall_report(pq, raw_world, "raw geodesic")
    sd = wall_report(pq, smooth, "smoothed (new)")
    if cl_path.with_suffix(".json.bak").exists():
        old = np.array(json.loads(cl_path.with_suffix(".json.bak").read_text())["waypoints"], float)
        wall_report(pq, old, "old .bak (reference)")

    if args.write:
        bak = cl_path.with_suffix(".json.regen.bak")
        if cl_path.exists() and not bak.exists():
            shutil.copy(cl_path, bak)
        data = json.loads(cl_path.read_text()) if cl_path.exists() else {}
        data.update({
            "phantom": "segment_part",
            "coordinate_system": "mujoco",
            "unit": "m",
            "entry": [float(c) for c in smooth[0]],
            "target_root": [float(c) for c in smooth[-1]],
            "length_m": float(res["length_mm"]),
            "max_curvature_per_m": float(res["max_curvature"]),
            "smooth": {
                "method": "bspline_splprep",
                "degree": 3,
                "smoothing_factor": float(args.smoothing),
                "source": "fine-grid geodesic",
                "voxel_pitch_m": float(args.pitch),
            },
            "waypoints": [[float(c) for c in p] for p in smooth],
        })
        cl_path.write_text(json.dumps(data, indent=2))
        print(f"  wrote {cl_path}  (backup {bak.name})")
    else:
        print("  (report only; pass --write to save)")


if __name__ == "__main__":
    main()
