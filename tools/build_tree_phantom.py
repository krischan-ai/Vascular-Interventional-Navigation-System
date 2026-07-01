#!/usr/bin/env python
"""Build a multi-branch *sealed-lumen* phantom from a whole VMTK centerline tree.

This is the multi-curve sibling of ``build_tube_phantom.py``. Where that tool
sweeps ONE radius-bearing Slicer curve into a sealed tube, this one ingests a
*directory* of Slicer curves (a VMTK ExtractCenterline export -- trunk plus every
branch) and sweeps each into its own sealed annular wall, then merges them into a
single phantom. Because every branch's wall is swept around its own VMTK
centerline, that centerline is centered in its lumen by construction (unlike a
voxel-skeleton route, which staircases off-center and pierces the wall).

Outputs (under ``src/cathsim/dm/components/phantom_assets/``):
  * ``meshes/<name>/hull_*.stl``   -- convex wall bricks (collision), all branches
  * ``meshes/<name>/visual.stl``   -- merged inner lumen surface (transparent)
  * ``meshes/<name>/centerlines.json`` -- every branch's centered waypoints+radius
  * ``meshes/<name>/centerline.json``  -- the primary navigable route (longest
                                          root->leaf chain), for guided nav
  * ``<name>.xml`` -- phantom MJCF (segment_part-style: geoms in worldbody)

Coordinates: Slicer markups are LPS millimetres -> MuJoCo metres (/1000, no flip),
matching build_tube_phantom / the VPP pipeline.

Usage:
    python -m tools.build_tree_phantom \
        --curves-dir "E:/下载/主动脉中心线" --name aorta_tree \
        --seg 0.004 --sectors 8 --wall 0.003
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np
import trimesh

from tools.build_tube_phantom import (
    PHANTOM_ASSETS,
    build_inner_surface,
    build_wall_bricks,
    load_slicer_curve,
    resample_by_arclen,
    rmf_normals,
    tangents,
)


def load_all_curves(curves_dir: Path) -> list[tuple[int, np.ndarray, np.ndarray]]:
    """Load every ``Centerline curve (N).mrk.json`` as (idx, points_m, radius_m)."""
    out: list[tuple[int, np.ndarray, np.ndarray]] = []
    for f in curves_dir.glob("Centerline curve (*).mrk.json"):
        idx = int(re.search(r"\((\d+)\)", f.name).group(1))
        try:
            pts, rad = load_slicer_curve(f)
        except ValueError as exc:
            print(f"  skip {f.name}: {exc}")
            continue
        out.append((idx, pts, rad))
    out.sort(key=lambda t: t[0])
    return out


# An endpoint graph: node coords + adjacency (node -> [(other, curve_idx, flip)]).
Graph = tuple[list[np.ndarray], dict[int, list[tuple[int, int, bool]]]]


def build_endpoint_graph(
    curves: list[tuple[int, np.ndarray, np.ndarray]], join_tol: float = 0.004
) -> Graph:
    """Build the branch tree: cluster curve endpoints into nodes, curves as edges.

    Each curve is an edge between its two endpoint-nodes (clustered within
    ``join_tol`` metres). ``flip`` records whether traversing the edge from the
    first node runs the curve reversed. Returns (node_coords, adjacency).
    """
    nodes: list[np.ndarray] = []

    def node_id(p: np.ndarray) -> int:
        for i, q in enumerate(nodes):
            if np.linalg.norm(p - q) <= join_tol:
                return i
        nodes.append(p.copy())
        return len(nodes) - 1

    adj: dict[int, list[tuple[int, int, bool]]] = {}
    for ci, (_idx, pts, _rad) in enumerate(curves):
        a, b = node_id(pts[0]), node_id(pts[-1])
        adj.setdefault(a, []).append((b, ci, False))
        adj.setdefault(b, []).append((a, ci, True))
    return nodes, adj


def _edge_len(curves, ci: int) -> float:
    return float(np.linalg.norm(np.diff(curves[ci][1], axis=0), axis=1).sum())


def farthest(
    adj: dict[int, list[tuple[int, int, bool]]], curves, start: int
) -> tuple[int, float, dict[int, tuple[int, int, bool]]]:
    """Farthest node from ``start`` over the tree (visited-set DFS).

    The adjacency is bidirectional, so a plain relaxation would bounce back and
    forth across an edge forever; a visited set is safe because a VMTK centerline
    export is a tree (no cycles). Returns (node, dist, parent) where ``parent``
    maps every reachable node to the (prev_node, curve_idx, flip) it arrived by.
    """
    best_node, best_dist = start, 0.0
    parent: dict[int, tuple[int, int, bool]] = {}
    visited = {start}
    stack = [(start, 0.0)]
    while stack:
        u, du = stack.pop()
        if du > best_dist:
            best_node, best_dist = u, du
        for v, ci, flip in adj.get(u, []):
            if v in visited:
                continue
            visited.add(v)
            parent[v] = (u, ci, flip)
            stack.append((v, du + _edge_len(curves, ci)))
    return best_node, best_dist, parent


def _concat_chain(curves, chain: list[tuple[int, bool]], join_tol: float):
    """Concatenate dense branch geometry along an ordered (curve_idx, flip) chain."""
    pts_out: list[np.ndarray] = []
    rad_out: list[np.ndarray] = []
    for ci, flip in chain:
        _idx, pts, rad = curves[ci]
        if flip:
            pts, rad = pts[::-1], rad[::-1]
        if pts_out and np.linalg.norm(pts_out[-1][-1] - pts[0]) <= join_tol:
            pts, rad = pts[1:], rad[1:]  # drop duplicate junction point
        pts_out.append(pts)
        rad_out.append(rad)
    return np.vstack(pts_out), np.concatenate(rad_out)


def route_between(
    curves, graph: Graph, start_node: int, end_node: int, join_tol: float = 0.004
) -> tuple[np.ndarray, np.ndarray]:
    """Dense (points_m, radius_m) of the unique tree path start_node -> end_node.

    Walks the real VMTK branch geometry between the two endpoint-nodes (not a
    straight line between them), so the route stays centered in every branch.
    """
    _nodes, adj = graph
    _far, _d, parent = farthest(adj, curves, start_node)
    chain: list[tuple[int, bool]] = []
    v = end_node
    while v in parent:
        u, ci, flip = parent[v]
        chain.append((ci, flip))
        v = u
    chain.reverse()
    if not chain:
        raise ValueError(f"no path between nodes {start_node} and {end_node}")
    return _concat_chain(curves, chain, join_tol)


def leaf_nodes(graph: Graph) -> list[int]:
    """Endpoint-graph nodes of degree 1 (branch tips / vessel leaves)."""
    _nodes, adj = graph
    return [n for n, edges in adj.items() if len(edges) == 1]


def nearest_node(graph: Graph, point: np.ndarray) -> int:
    """Index of the endpoint-graph node closest to ``point`` (metres)."""
    nodes, _adj = graph
    return int(np.argmin([np.linalg.norm(n - point) for n in nodes]))


def generate_routes(
    curves, graph: Graph, entry_node: int, seg: float, smooth_mm: float
) -> dict[str, dict]:
    """Dense, centered, smoothed routes from ``entry_node`` to every other leaf.

    One route per branch tip (vessel leaf), so a client can pick any target
    endpoint and the guidewire follows the real branch centerline to it. Keyed by
    ``endpoint_<node>``; each carries target position, length, and waypoints.
    """
    routes: dict[str, dict] = {}
    for leaf in leaf_nodes(graph):
        if leaf == entry_node:
            continue
        pts, rad = route_between(curves, graph, entry_node, leaf)
        pts, rad = smooth_route(pts, rad, seg, smooth_mm)
        if len(pts) < 2:
            continue
        routes[f"endpoint_{leaf}"] = {
            "target": [round(float(v), 6) for v in pts[-1]],
            "length_m": round(float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum()), 5),
            "n_waypoints": len(pts),
            "waypoints": [[round(float(v), 6) for v in p] for p in pts],
            "radius_m": [round(float(r), 6) for r in rad],
        }
    return routes


def longest_route(
    curves: list[tuple[int, np.ndarray, np.ndarray]], join_tol: float = 0.004
) -> tuple[np.ndarray, np.ndarray]:
    """Longest root->leaf dense polyline (tree diameter via two farthest walks)."""
    graph = build_endpoint_graph(curves, join_tol)
    _nodes, adj = graph
    end_a, _, _ = farthest(adj, curves, next(iter(adj)))
    end_b, _, _ = farthest(adj, curves, end_a)
    return route_between(curves, graph, end_a, end_b, join_tol)


def smooth_route(
    pts: np.ndarray, rad: np.ndarray, seg: float, smooth_mm: float
) -> tuple[np.ndarray, np.ndarray]:
    """B-spline smooth a chained route, ironing junction kinks, then resample.

    Chaining branches head-to-tail leaves a sharp corner wherever two VMTK curves
    meet at a bifurcation. A cubic ``splprep`` with a bounded residual (``s`` sized
    so the spline may drift up to ~``smooth_mm`` per point) rounds those corners
    without pulling the line off the branch centers on the smooth spans. Radius is
    interpolated along the same arc-length parameter. ``smooth_mm <= 0`` disables
    smoothing and just resamples.
    """
    pts = np.asarray(pts, dtype=np.float64)
    if smooth_mm <= 0.0 or len(pts) < 4:
        return resample_by_arclen(pts, rad, seg)

    # Drop zero-length segments so the arc-length parameter is strictly increasing
    # (splprep requires a monotonic u).
    d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    keep = np.concatenate([[True], d > 1e-9])
    pts, rad = pts[keep], np.asarray(rad, float)[keep]

    from scipy import interpolate

    cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    total = float(cum[-1])
    u = cum / total
    n = len(pts)
    s = n * (smooth_mm / 1000.0) ** 2
    tck, _ = interpolate.splprep([pts[:, 0], pts[:, 1], pts[:, 2]], u=u, k=3, s=s)

    n_out = max(int(round(total / seg)) + 1, 2)
    u_new = np.linspace(0.0, 1.0, n_out)
    smooth_pts = np.asarray(interpolate.splev(u_new, tck)).T
    rad_out = np.interp(u_new, u, rad)
    return smooth_pts, rad_out


def write_tree_xml(name: str, n_hulls: int, entry, target) -> Path:
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
    lines += [
        '  </asset>',
        '  <worldbody>',
        '    <geom type="mesh" mesh="visual" rgba=".8 .5 .3 .3" group="0" '
        'contype="0" conaffinity="0" />',
    ]
    for i in range(n_hulls):
        lines.append(
            f'    <geom type="mesh" mesh="hull_{i}" rgba=".8 .5 .3 .0" '
            'group="1" condim="1" friction="1." />'
        )
    e = " ".join(f"{v:.6f}" for v in entry)
    t = " ".join(f"{v:.6f}" for v in target)
    lines += [
        f'    <site name="entry" pos="{e}" />',
        f'    <site name="root" pos="{t}" />',
        '  </worldbody>',
        '</mujoco>',
        '',
    ]
    out = PHANTOM_ASSETS / f"{name}.xml"
    out.write_text("\n".join(lines), encoding="utf-8")
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--curves-dir", required=True, help="dir of Slicer curve .mrk.json")
    ap.add_argument("--name", required=True, help="phantom name (e.g. aorta_tree)")
    ap.add_argument("--seg", type=float, default=0.004, help="axial resample step (m)")
    ap.add_argument("--sectors", type=int, default=8, help="angular sectors per ring")
    ap.add_argument("--wall", type=float, default=0.003, help="radial wall thickness (m)")
    ap.add_argument("--smooth-mm", type=float, default=0.5,
                    help="B-spline smoothing budget for the primary route (mm); "
                         "rounds junction kinks while keeping the line centered. "
                         "0 disables.")
    ap.add_argument("--entry", type=float, nargs=3,
                    default=[-1.020652, -0.231892, 0.257573],
                    metavar=("X", "Y", "Z"),
                    help="Vascular-access entry point (MuJoCo metres); routes to "
                         "every leaf start here. Default = aorta_trunk entry.")
    ap.add_argument("--no-bricks", action="store_true",
                    help="skip collision wall bricks (guided-mode-only phantoms)")
    args = ap.parse_args()

    curves_dir = Path(args.curves_dir)
    print(f"[load] {curves_dir}")
    curves = load_all_curves(curves_dir)
    print(f"  {len(curves)} curves with radius")

    mesh_dir = PHANTOM_ASSETS / "meshes" / args.name
    mesh_dir.mkdir(parents=True, exist_ok=True)
    for f in mesh_dir.glob("hull_*.stl"):
        f.unlink()

    all_bricks: list[trimesh.Trimesh] = []
    inner_meshes: list[trimesh.Trimesh] = []
    centerlines: dict[str, dict] = {}

    for idx, pts, rad in curves:
        rs_pts, rs_rad = resample_by_arclen(pts, rad, args.seg)
        if len(rs_pts) < 3:
            continue
        tan = tangents(rs_pts)
        normal, binormal = rmf_normals(rs_pts, tan)
        inner_meshes.append(
            build_inner_surface(rs_pts, rs_rad, normal, binormal, args.sectors)
        )
        if not args.no_bricks:
            all_bricks.extend(
                build_wall_bricks(rs_pts, rs_rad, normal, binormal, args.sectors, args.wall)
            )
        centerlines[f"branch_{idx}"] = {
            "waypoints": [[round(float(v), 6) for v in p] for p in rs_pts],
            "radius_m": [round(float(r), 6) for r in rs_rad],
            "length_m": round(float(np.linalg.norm(np.diff(rs_pts, axis=0), axis=1).sum()), 5),
        }
        print(f"  branch {idx}: {len(rs_pts)} rings, "
              f"{'no bricks' if args.no_bricks else str(len(all_bricks)) + ' bricks so far'}")

    # Merge inner lumen surfaces into one visual mesh.
    visual = trimesh.util.concatenate(inner_meshes)
    visual.export(str(mesh_dir / "visual.stl"), file_type="stl")
    print(f"[visual] merged {len(inner_meshes)} branch surfaces "
          f"-> {len(visual.vertices)} verts")

    for i, b in enumerate(all_bricks):
        b.export(str(mesh_dir / f"hull_{i}.stl"), file_type="stl")
    print(f"[bricks] wrote {len(all_bricks)} collision hulls")

    # Branch routing: build the endpoint tree, fix the vascular-access entry, and
    # route from it to every leaf along the real (dense, centered) branch
    # geometry -- B-spline smoothed to iron the bifurcation corners.
    graph = build_endpoint_graph(curves)
    entry_pt = np.asarray(args.entry, dtype=np.float64)
    entry_node = nearest_node(graph, entry_pt)
    _far, _d, _p = farthest(graph[1], curves, entry_node)  # farthest leaf from entry
    far_leaf = _far

    route_pts, route_rad = route_between(curves, graph, entry_node, far_leaf)
    route_pts, route_rad = smooth_route(route_pts, route_rad, args.seg, args.smooth_mm)
    route_len = float(np.linalg.norm(np.diff(route_pts, axis=0), axis=1).sum())
    (mesh_dir / "centerline.json").write_text(
        json.dumps(
            {
                "phantom": args.name,
                "coordinate_system": "mujoco",
                "unit": "m",
                "source": f"{curves_dir.name} (3D Slicer VMTK, LPS mm /1000), entry->farthest leaf",
                "entry": [round(float(v), 6) for v in route_pts[0]],
                "target_root": [round(float(v), 6) for v in route_pts[-1]],
                "length_m": round(route_len, 5),
                "radius_m": [round(float(r), 6) for r in route_rad],
                "waypoints": [[round(float(v), 6) for v in p] for p in route_pts],
            },
            ensure_ascii=False, indent=2,
        ),
        encoding="utf-8",
    )
    (mesh_dir / "centerlines.json").write_text(
        json.dumps(
            {"phantom": args.name, "coordinate_system": "mujoco", "unit": "m",
             "branches": centerlines,
             "summary": {"n_branches": len(centerlines),
                         "total_length_m": round(sum(c["length_m"] for c in centerlines.values()), 3)}},
            ensure_ascii=False, indent=1,
        ),
        encoding="utf-8",
    )

    # Per-endpoint branch routes for target selection (guided navigation).
    routes = generate_routes(curves, graph, entry_node, args.seg, args.smooth_mm)
    (mesh_dir / "routes.json").write_text(
        json.dumps(
            {"phantom": args.name, "coordinate_system": "mujoco", "unit": "m",
             "entry": [round(float(v), 6) for v in graph[0][entry_node]],
             "n_routes": len(routes),
             "routes": routes},
            ensure_ascii=False, indent=1,
        ),
        encoding="utf-8",
    )
    print(f"[route] primary centerline: {len(route_pts)} pts, {route_len*1e3:.0f}mm")
    print(f"[routes] {len(routes)} branch routes from entry node {entry_node} "
          f"@ {np.round(graph[0][entry_node], 3)}")

    xml = write_tree_xml(args.name, len(all_bricks), route_pts[0], route_pts[-1])
    print(f"[done] phantom '{args.name}': {len(all_bricks)} hulls -> {xml}")
    print(f"  entry={np.round(route_pts[0],3)}  target={np.round(route_pts[-1],3)}")
    print(f"  NEXT: register '{args.name}' in NavigationEngine.VALID_PHANTOMS,")
    print(f"        export GLB (tools/export_godot_assets.py --phantom {args.name}),")
    print(f"        add to Godot MODELS.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
