#!/usr/bin/env python
"""V-HACD convex decomposition tool for blood vessel meshes.

Converts VTK blood vessel mesh to MuJoCo-compatible convex hull STL files
and generates the corresponding MuJoCo XML phantom definition.

Usage:
    python tools/vhacd_decompose.py [--case-id CASE_ID] [--max-hulls N]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np
import pyvista as pv
import trimesh

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATA_ROOT = PROJECT_ROOT / "data" / "vpp_assets"


def load_vtk_as_trimesh(vtk_path: Path) -> trimesh.Trimesh:
    """Load VTK mesh and convert to trimesh."""
    pv_mesh = pv.read(str(vtk_path))

    if not pv_mesh.is_all_triangles:
        pv_mesh = pv_mesh.triangulate()

    faces = pv_mesh.faces.reshape(-1, 4)[:, 1:4]

    mesh = trimesh.Trimesh(
        vertices=np.array(pv_mesh.points),
        faces=faces,
        process=True,
    )
    return mesh


def transform_lps_to_mujoco(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Transform mesh from LPS (mm) to MuJoCo (m) coordinate system.

    LPS: Left-Posterior-Superior (medical imaging)
    MuJoCo: Uses a different convention, but we primarily need mm->m conversion.

    For VPP data, we keep the coordinate axes but scale from mm to m.
    """
    vertices = mesh.vertices.copy()
    vertices = vertices / 1000.0

    return trimesh.Trimesh(
        vertices=vertices,
        faces=mesh.faces.copy(),
        process=False,
    )


def run_vhacd(
    mesh: trimesh.Trimesh,
    max_hulls: int = 256,
    resolution: int = 100000,
    max_vertices_per_hull: int = 64,
) -> list[trimesh.Trimesh]:
    """Run V-HACD convex decomposition on the mesh."""
    print(f"Running V-HACD decomposition (max_hulls={max_hulls})...")
    print(f"  Input mesh: {len(mesh.vertices)} vertices, {len(mesh.faces)} faces")

    try:
        hull_args = trimesh.decomposition.convex_decomposition(
            mesh,
            maxConvexHulls=max_hulls,
            resolution=resolution,
            maxNumVerticesPerCH=max_vertices_per_hull,
            minimumVolumePercentErrorAllowed=1.0,
            maxRecursionDepth=14,
            shrinkWrap=True,
            fillMode="flood",
        )
    except Exception as e:
        print(f"V-HACD failed: {e}")
        print("Falling back to simple convex hull...")
        hull_args = [{"vertices": mesh.convex_hull.vertices, "faces": mesh.convex_hull.faces}]

    hulls = []
    for i, args in enumerate(hull_args):
        hull = trimesh.Trimesh(**args, process=True)
        hulls.append(hull)

    print(f"  Generated {len(hulls)} convex hulls")
    return hulls


def export_stl_files(
    visual_mesh: trimesh.Trimesh,
    hulls: list[trimesh.Trimesh],
    output_dir: Path,
    case_id: str,
) -> dict[str, Any]:
    """Export visual and collision meshes as STL files."""
    mesh_dir = output_dir / case_id
    mesh_dir.mkdir(parents=True, exist_ok=True)

    visual_path = mesh_dir / "visual.stl"
    visual_mesh.export(str(visual_path), file_type="stl")
    print(f"  Exported visual mesh: {visual_path.name}")

    hull_files = []
    for i, hull in enumerate(hulls):
        hull_path = mesh_dir / f"hull_{i}.stl"
        hull.export(str(hull_path), file_type="stl")
        hull_files.append(f"hull_{i}")

    print(f"  Exported {len(hull_files)} collision hulls")

    return {
        "visual": "visual.stl",
        "hulls": [f"{name}.stl" for name in hull_files],
        "mesh_dir": str(mesh_dir.relative_to(output_dir.parent)),
    }


def load_targets(case_dir: Path) -> dict[str, list[float]]:
    """Load target positions from derived targets.json."""
    targets_path = case_dir / "derived" / "targets.json"
    if not targets_path.is_file():
        print(f"Warning: targets.json not found at {targets_path}")
        return {}

    with targets_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    targets = {}
    for key, value in data.get("endpoints", {}).items():
        position = value.get("position_lps") if isinstance(value, dict) else None
        if isinstance(position, list) and len(position) == 3:
            pos_m = [v / 1000.0 for v in position]
            targets[key.lower().replace("-", "_").replace(" ", "_")] = pos_m

    return targets


def generate_mujoco_xml(
    case_id: str,
    mesh_info: dict[str, Any],
    targets: dict[str, list[float]],
    output_path: Path,
) -> None:
    """Generate MuJoCo XML phantom definition."""
    lines = [
        f'<mujoco model="{case_id}_vpp">',
        f'  <compiler meshdir="meshes" />',
        '  <default>',
        '    <geom type="mesh" />',
        '  </default>',
        '  <asset>',
    ]

    for hull_file in mesh_info["hulls"]:
        hull_name = hull_file.replace(".stl", "")
        lines.append(f'    <mesh name="{hull_name}" file="{case_id}/{hull_file}" />')

    lines.append('  </asset>')
    lines.append('  <worldbody>')

    for name, pos in targets.items():
        pos_str = " ".join(f"{v:.8f}" for v in pos)
        lines.append(f'    <site name="{name}" pos="{pos_str}" />')

    lines.append('    <body name="phantom">')

    for hull_file in mesh_info["hulls"]:
        hull_name = hull_file.replace(".stl", "")
        lines.append(f'      <geom mesh="{hull_name}" />')

    lines.append('    </body>')
    lines.append('  </worldbody>')
    lines.append('</mujoco>')
    lines.append('')

    output_path.write_text('\n'.join(lines), encoding='utf-8')
    print(f"Generated MuJoCo XML: {output_path}")


def process_case(
    case_id: str,
    max_hulls: int = 256,
    resolution: int = 100000,
) -> None:
    """Process a single VPP case."""
    case_dir = DATA_ROOT / case_id
    if not case_dir.is_dir():
        print(f"Error: Case directory not found: {case_dir}")
        sys.exit(1)

    vtk_path = case_dir / "raw" / "blood_vessels.vtk"
    if not vtk_path.is_file():
        print(f"Error: VTK file not found: {vtk_path}")
        sys.exit(1)

    print(f"Processing case: {case_id}")
    print(f"  VTK file: {vtk_path}")

    print("\n[1/5] Loading VTK mesh...")
    mesh = load_vtk_as_trimesh(vtk_path)
    print(f"  Loaded mesh: {len(mesh.vertices)} vertices, {len(mesh.faces)} faces")
    print(f"  Bounds (mm): {mesh.bounds}")

    print("\n[2/5] Transforming coordinates (LPS mm -> MuJoCo m)...")
    mesh_mujoco = transform_lps_to_mujoco(mesh)
    print(f"  Bounds (m): {mesh_mujoco.bounds}")

    print("\n[3/5] Running V-HACD decomposition...")
    hulls = run_vhacd(mesh_mujoco, max_hulls=max_hulls, resolution=resolution)

    print("\n[4/5] Exporting STL files...")
    mesh_dir = case_dir / "mujoco" / "meshes"
    mesh_info = export_stl_files(mesh_mujoco, hulls, mesh_dir, case_id)

    print("\n[5/5] Generating MuJoCo XML...")
    targets = load_targets(case_dir)
    if not targets:
        print("  Using default endpoint positions from Endpoints.fcsv")
        endpoints_path = case_dir / "raw" / "Endpoints.fcsv"
        if endpoints_path.is_file():
            targets = parse_endpoints_fcsv(endpoints_path)

    xml_path = case_dir / "mujoco" / f"{case_id}_vpp.xml"
    generate_mujoco_xml(case_id, mesh_info, targets, xml_path)

    print("\n" + "=" * 60)
    print(f"V-HACD decomposition complete for {case_id}")
    print(f"  Convex hulls: {len(hulls)}")
    print(f"  Mesh directory: {mesh_dir / case_id}")
    print(f"  MuJoCo XML: {xml_path}")


def parse_endpoints_fcsv(fcsv_path: Path) -> dict[str, list[float]]:
    """Parse endpoints from FCSV file and convert to MuJoCo coordinates."""
    targets = {}

    with fcsv_path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line.startswith("#") or not line:
                continue

            parts = line.split(",")
            if len(parts) >= 12:
                try:
                    x = float(parts[1]) / 1000.0
                    y = float(parts[2]) / 1000.0
                    z = float(parts[3]) / 1000.0
                    label = parts[11].strip()

                    if label:
                        safe_name = label.lower().replace("-", "_").replace(" ", "_")
                        targets[safe_name] = [x, y, z]
                except (ValueError, IndexError):
                    continue

    return targets


def main() -> None:
    parser = argparse.ArgumentParser(
        description="V-HACD convex decomposition for blood vessel meshes"
    )
    parser.add_argument(
        "--case-id",
        default="case_001",
        help="Case ID to process (default: case_001)",
    )
    parser.add_argument(
        "--max-hulls",
        type=int,
        default=256,
        help="Maximum number of convex hulls (default: 256)",
    )
    parser.add_argument(
        "--resolution",
        type=int,
        default=100000,
        help="V-HACD voxel resolution (default: 100000)",
    )

    args = parser.parse_args()
    process_case(args.case_id, args.max_hulls, args.resolution)


if __name__ == "__main__":
    main()
