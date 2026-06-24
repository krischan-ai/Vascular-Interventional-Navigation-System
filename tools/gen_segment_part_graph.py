#!/usr/bin/env python
"""Generate segment_part centerline graph (graph.json) from STL skeleton.

Full pipeline:
  1. Voxelize + skeletonize (preserve topology)
  2. Extract junction/endpoint topology
  3. Segment into individual branches (DFS)
  4. Downsample + B-spline smooth each branch
  5. Export graph.json (adjacency map format)

Usage:
    python tools/gen_segment_part_graph.py
"""

import json
import numpy as np
import trimesh
import networkx as nx
from pathlib import Path
from skimage.morphology import skeletonize

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PHANTOM_MESH_DIR = (
    PROJECT_ROOT
    / "src/cathsim/dm/components/phantom_assets/meshes/segment_part"
)
VISUAL_STL = PHANTOM_MESH_DIR / "visual.stl"
GRAPH_OUTPUT = PHANTOM_MESH_DIR / "graph.json"
CENTERLINES_OUTPUT = PHANTOM_MESH_DIR / "centerlines.json"

# Tuning parameters
VOXEL_PITCH = 0.0015  # 1.5 mm
DOWNSAMPLE_STRIDE = 5  # Every N voxels
DOWNSAMPLE_MIN_DIST = 0.003  # 3 mm
SMOOTH_FACTOR = 0.25e-6  # B-spline smoothing
BRANCH_MIN_LENGTH = 10  # Minimum voxels to be a valid branch


def build_voxel_graph(skeleton_pts):
    """Build NetworkX graph from skeleton voxel points (26-connectivity)."""
    print("[1] Building voxel connectivity graph...")
    G = nx.Graph()
    skel_set = set(map(tuple, skeleton_pts))
    G.add_nodes_from(skel_set)

    # 26-connectivity (3x3x3 neighborhood)
    offsets = [
        (a, b, c)
        for a in (-1, 0, 1)
        for b in (-1, 0, 1)
        for c in (-1, 0, 1)
        if (a, b, c) != (0, 0, 0)
    ]

    for pt in skel_set:
        for off in offsets:
            neighbor = (pt[0] + off[0], pt[1] + off[1], pt[2] + off[2])
            if neighbor in skel_set:
                G.add_edge(pt, neighbor, weight=float(np.linalg.norm(off)))

    print(f"  Nodes: {G.number_of_nodes()}, Edges: {G.number_of_edges()}")
    return G


def identify_key_points(G):
    """Find junctions (degree >= 3) and endpoints (degree == 1)."""
    junctions = [v for v in G.nodes() if G.degree(v) >= 3]
    endpoints = [v for v in G.nodes() if G.degree(v) == 1]
    print(f"  Junctions: {len(junctions)}, Endpoints: {len(endpoints)}")
    return junctions, endpoints


def dfs_extract_branches(G, entry_voxel, visited=None):
    """Extract branches recursively via DFS from entry_voxel.

    Returns: list of branches, where each branch is a list of voxel coordinates.
    """
    if visited is None:
        visited = set()

    branches = []

    def dfs(current, parent=None):
        """DFS to extract a single branch from current node."""
        path = [current]
        visited.add(current)

        while True:
            neighbors = [
                n for n in G[current] if n != parent and n not in visited
            ]

            if not neighbors:
                # Dead end (endpoint or visited junction)
                break

            if len(neighbors) > 1:
                # Junction: recursively process each child
                for neighbor in neighbors:
                    child_branches = dfs(neighbor, current)
                    branches.extend(child_branches)
                break

            # Continue along the branch
            current = neighbors[0]
            path.append(current)
            visited.add(current)

        return [path] if len(path) > BRANCH_MIN_LENGTH else []

    main = dfs(entry_voxel)
    branches.extend(main)

    # Collect unvisited components (shouldn't happen if fully connected)
    unvisited = set(G.nodes()) - visited
    while unvisited:
        root = next(iter(unvisited))
        orphan = dfs(root)
        branches.extend(orphan)
        unvisited -= visited

    return branches


def downsample_branch(voxel_path, voxel_grid, stride=5, min_dist=0.003):
    """Downsample voxel path to world coordinates with stride or distance threshold."""
    world_pts = voxel_grid.indices_to_points(np.asarray(voxel_path, float))

    downsampled = [world_pts[0]]  # Always keep start
    for i in range(1, len(world_pts)):
        if i % stride == 0 or np.linalg.norm(world_pts[i] - downsampled[-1]) > min_dist:
            downsampled.append(world_pts[i])

    if len(downsampled) > 1:
        downsampled[-1] = world_pts[-1]  # Always keep end

    return np.array(downsampled)


def smooth_branch(waypoints_m, smooth_factor=0.25e-6):
    """B-spline smooth a branch using scipy splprep."""
    from scipy import interpolate

    points = np.asarray(waypoints_m)
    if len(points) < 4:
        return points, 0.0, 0.0

    n_points = len(points)
    cumulative_dist = np.zeros(n_points)
    for i in range(1, n_points):
        cumulative_dist[i] = cumulative_dist[i - 1] + np.linalg.norm(
            points[i] - points[i - 1]
        )

    if cumulative_dist[-1] < 1e-10:
        return points, 0.0, 0.0

    u = cumulative_dist / cumulative_dist[-1]
    k = min(3, n_points - 1)

    s = smooth_factor * n_points
    tck, _ = interpolate.splprep(
        [points[:, 0], points[:, 1], points[:, 2]], u=u, k=k, s=s
    )

    u_new = np.linspace(0, 1, n_points * 2)
    smooth_points = np.array(interpolate.splev(u_new, tck)).T

    # Compute length and max curvature
    length_m = float(
        np.sum(np.linalg.norm(np.diff(smooth_points, axis=0), axis=1))
    )

    d1 = np.array(interpolate.splev(u_new, tck, der=1)).T
    d2 = np.array(interpolate.splev(u_new, tck, der=2)).T
    curvatures = []
    for i in range(len(u_new)):
        v1 = d1[i]
        v2 = d2[i]
        cross = np.cross(v1, v2)
        norm_v1 = np.linalg.norm(v1)
        if norm_v1 > 1e-10:
            kappa = np.linalg.norm(cross) / (norm_v1**3)
            curvatures.append(kappa)

    max_curvature = float(max(curvatures)) if curvatures else 0.0

    return smooth_points, length_m, max_curvature


def build_graph_dict(centerlines, junctions, endpoints):
    """Build adjacency map (graph.json format) from branches and key points."""
    print("[5] Building graph.json...")

    graph_dict = {}
    all_key_pts = junctions + endpoints

    for branch_idx, (branch_id, metadata) in enumerate(centerlines.items()):
        wpts = metadata["waypoints"]
        start_pt = np.array(wpts[0])
        end_pt = np.array(wpts[-1])

        # Find nearest key point for start and end
        start_key = min(
            all_key_pts, key=lambda p: np.linalg.norm(p - start_pt)
        )
        end_key = min(
            all_key_pts, key=lambda p: np.linalg.norm(p - end_pt)
        )

        start_key_str = f"{start_key[0]:.6f},{start_key[1]:.6f},{start_key[2]:.6f}"
        end_key_str = f"{end_key[0]:.6f},{end_key[1]:.6f},{end_key[2]:.6f}"

        edge_weight = metadata["length_m"]

        # Add bidirectional edges
        if start_key_str not in graph_dict:
            graph_dict[start_key_str] = []
        graph_dict[start_key_str].append([end_key_str, edge_weight])

        if end_key_str not in graph_dict:
            graph_dict[end_key_str] = []
        graph_dict[end_key_str].append([start_key_str, edge_weight])

    print(f"  Nodes: {len(graph_dict)}")
    return graph_dict


def main():
    print("=" * 70)
    print("Segment_part Centerline Graph Generation (方案 A)")
    print("=" * 70)

    # Load visual mesh
    print(f"\n[Load] Loading {VISUAL_STL}...")
    mesh = trimesh.load(VISUAL_STL)
    print(f"  Vertices: {len(mesh.vertices):,}, Faces: {len(mesh.faces):,}")

    # Step 1-2: Voxelize and skeletonize
    print(f"\n[Step 1-2] Voxelizing and skeletonizing (pitch={VOXEL_PITCH} m)...")
    voxel_grid = mesh.voxelized(VOXEL_PITCH).fill()
    volume = np.asarray(voxel_grid.matrix, dtype=bool)
    skeleton = skeletonize(volume)
    skel_pts = np.argwhere(skeleton)
    print(f"  Skeleton voxels: {len(skel_pts)}")

    # Build voxel graph
    G = build_voxel_graph(skel_pts)

    # Identify junctions and endpoints (convert voxel → world)
    print("\n[Step 2] Identifying key points...")
    junctions_vox, endpoints_vox = identify_key_points(G)
    junctions_world = voxel_grid.indices_to_points(np.asarray(junctions_vox, float))
    endpoints_world = voxel_grid.indices_to_points(np.asarray(endpoints_vox, float))

    # Step 3: Branch segmentation via DFS
    print("\n[Step 3] Extracting branches via DFS...")
    entry_m = np.array([-0.824, -0.223, 0.263])
    entry_vox_idx = np.argmin(
        np.linalg.norm(voxel_grid.indices_to_points(skel_pts) - entry_m, axis=1)
    )
    entry_vox = tuple(skel_pts[entry_vox_idx])

    branches_vox = dfs_extract_branches(G, entry_vox)
    print(f"  Branches extracted: {len(branches_vox)}")

    # Step 4: Downsample + B-spline smooth
    print("\n[Step 4] Downsampling and B-spline smoothing...")
    centerlines = {}
    for i, branch_vox in enumerate(branches_vox):
        downsampled_m = downsample_branch(
            branch_vox, voxel_grid, stride=DOWNSAMPLE_STRIDE, min_dist=DOWNSAMPLE_MIN_DIST
        )

        if len(downsampled_m) < 4:
            continue

        smooth_pts, length_m, max_curv = smooth_branch(
            downsampled_m, smooth_factor=SMOOTH_FACTOR
        )

        centerlines[f"branch_{i}"] = {
            "waypoints": [[float(x) for x in p] for p in smooth_pts],
            "length_m": round(length_m, 5),
            "max_curvature": round(max_curv, 3),
            "n_waypoints": len(smooth_pts),
        }

        print(
            f"  Branch {i}: {len(downsampled_m)} voxels → {len(smooth_pts)} smooth waypoints, "
            f"len={length_m:.3f}m, curv={max_curv:.1f}/m"
        )

    # Step 5: Build graph.json
    graph_dict = build_graph_dict(
        centerlines, junctions_world.tolist(), endpoints_world.tolist()
    )

    # Save outputs
    print("\n[Output] Writing graph.json and centerlines.json...")

    GRAPH_OUTPUT.parent.mkdir(parents=True, exist_ok=True)

    with open(GRAPH_OUTPUT, "w", encoding="utf-8") as f:
        json.dump(graph_dict, f, indent=2, ensure_ascii=False)
    print(f"  Saved: {GRAPH_OUTPUT}")

    with open(CENTERLINES_OUTPUT, "w", encoding="utf-8") as f:
        json.dump(
            {
                "phantom": "segment_part",
                "coordinate_system": "mujoco",
                "unit": "m",
                "branches": centerlines,
                "summary": {
                    "n_branches": len(centerlines),
                    "total_length_m": round(
                        sum(c["length_m"] for c in centerlines.values()), 3
                    ),
                },
            },
            f,
            indent=1,
            ensure_ascii=False,
        )
    print(f"  Saved: {CENTERLINES_OUTPUT}")

    print("\n" + "=" * 70)
    print("Graph generation complete!")
    print("=" * 70)
    print(f"\nNext steps:")
    print(f"  1. Verify graph.json loads: python -c \"from services.graph_loader import GraphLoader; GraphLoader('{GRAPH_OUTPUT}')\"")
    print(f"  2. Test A* planning: NavigationEngine(phantom='segment_part').plan_to_target(...)")
    print(f"  3. Update doc/05-开发进度记录.md with results")


if __name__ == "__main__":
    main()
