#!/usr/bin/env python
"""Visualize centerlines vs mesh to verify they're in the vessel center.

Creates a cross-section cut showing:
  - Vessel mesh profile
  - Centerline position within the profile
  - Distance metrics
"""

import numpy as np
import trimesh
import json
from pathlib import Path
from scipy import ndimage

PROJECT_ROOT = Path(__file__).resolve().parents[1]
MESH = PROJECT_ROOT / "src/cathsim/dm/components/phantom_assets/meshes/segment_part/visual.stl"
CENTERLINE_OLD = PROJECT_ROOT / "src/cathsim/dm/components/phantom_assets/meshes/segment_part/centerline.json"
CENTERLINES_NEW = PROJECT_ROOT / "src/cathsim/dm/components/phantom_assets/meshes/segment_part/centerlines.json"

mesh = trimesh.load(MESH)

# Load centerlines
with open(CENTERLINE_OLD) as f:
    old_cl = np.array(json.load(f)["waypoints"])

with open(CENTERLINES_NEW) as f:
    data_new = json.load(f)
    new_branches = {k: np.array(v["waypoints"]) for k, v in data_new["branches"].items()}

print("Centerline Quality Analysis")
print("=" * 70)

# Metric 1: Distance to surface
print("\n[1] Distance from centerline to mesh surface")
print("-" * 70)

def analyze_centerline(name, points):
    """Check if centerline is inside vessel and measure offset from surface."""
    inside_count = 0
    distances = []

    for pt in points:
        # Check if inside mesh
        if mesh.contains([pt])[0]:
            inside_count += 1

        # Distance to surface
        cp, dist, fid = trimesh.proximity.closest_point(mesh, pt[None,:])
        distances.append(dist[0])

    distances = np.array(distances)
    inside_ratio = inside_count / len(points) * 100

    print(f"{name:30s}: {inside_ratio:5.1f}% inside, "
          f"offset μ={distances.mean()*1000:5.2f}mm, "
          f"σ={distances.std()*1000:5.2f}mm, "
          f"max={distances.max()*1000:5.2f}mm")

    return distances

print(f"\nOld centerline (阶段十九):")
old_dist = analyze_centerline("  centerline.json", old_cl)

print(f"\nNew centerlines (方案A - 样本):")
for branch_id in ["branch_0", "branch_2", "branch_11", "branch_56"]:
    if branch_id in new_branches:
        analyze_centerline(f"  {branch_id}", new_branches[branch_id])

# Metric 2: Vessel radius estimation
print("\n[2] Vessel lumen radius estimation")
print("-" * 70)

pitch = 0.0015
voxel_grid = mesh.voxelized(pitch).fill()
volume = np.asarray(voxel_grid.matrix, dtype=bool)
edt = ndimage.distance_transform_edt(volume)

# Sample radius at different centerline points
radii_old = []
for pt in old_cl[::10]:  # Every 10th point
    ijk = np.round(voxel_grid.points_to_indices(pt[None,:])[0]).astype(int)
    ijk = np.clip(ijk, 0, np.array(volume.shape) - 1)
    r = edt[tuple(ijk)] * pitch
    radii_old.append(r)

radii_old = np.array(radii_old)
print(f"  Vessel lumen radius (voxel estimate):")
print(f"    Old centerline: μ={radii_old.mean()*1000:.1f}mm, σ={radii_old.std()*1000:.1f}mm")

# Metric 3: Offset as fraction of radius
print("\n[3] Centerline offset as fraction of radius")
print("-" * 70)

offset_ratio = old_dist.mean() / radii_old.mean()
print(f"  Mean offset / mean radius = {old_dist.mean()*1000:.2f}mm / {radii_old.mean()*1000:.1f}mm")
print(f"                           = {offset_ratio:.1%}")
print(f"\n  OK if < 20%: centerline is well-centered")
print(f"  BAD if > 50%: centerline is off-center (needs adjustment)")
print(f"  → Result: {offset_ratio:.1%} → {'OK - CENTERED' if offset_ratio < 0.3 else 'PROBLEM - OFF-CENTER'}")

print("\n" + "=" * 70)
