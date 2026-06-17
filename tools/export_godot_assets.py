#!/usr/bin/env python
"""Export Godot-ready GLB assets from CathSim/VPP meshes.

Godot imports glTF/GLB but cannot read VTK or STL scene meshes directly. This
tool converts the high-resolution vessel surface (``visual.stl``, already in the
MuJoCo/guidewire meter frame) into a decimated GLB so the Godot client can
overlay the guidewire (whose positions come from MuJoCo physics in the same
frame) without any coordinate conversion.

Usage:
    python tools/export_godot_assets.py [--case-id CASE_ID] [--max-faces N]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import trimesh

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATA_ROOT = PROJECT_ROOT / "data" / "vpp_assets"
GODOT_MODELS = PROJECT_ROOT / "godot_client" / "assets" / "models"
PHANTOM_MESH_ROOT = (
    PROJECT_ROOT / "src" / "cathsim" / "dm" / "components" / "phantom_assets" / "meshes"
)


def export_cathsim_phantom(phantom_name: str) -> None:
    """Export a CathSim phantom's visual mesh to GLB for the Godot client.

    The guidewire is simulated inside this phantom (at native scale, origin), so
    rendering it aligns the vessel with the streamed guidewire positions.
    """
    visual_stl = PHANTOM_MESH_ROOT / phantom_name / "visual.stl"
    if not visual_stl.is_file():
        raise FileNotFoundError(f"Phantom visual mesh not found: {visual_stl}")

    print(f"Loading phantom visual mesh: {visual_stl}")
    mesh = trimesh.load(visual_stl)
    print(f"  {len(mesh.vertices)} verts / {len(mesh.faces)} faces")

    GODOT_MODELS.mkdir(parents=True, exist_ok=True)
    out_path = GODOT_MODELS / f"{phantom_name}.glb"
    mesh.export(out_path, file_type="glb")
    size_mb = out_path.stat().st_size / (1024 * 1024)
    print(f"Exported GLB: {out_path} ({size_mb:.2f} MB)")


def decimate(mesh: trimesh.Trimesh, max_faces: int) -> trimesh.Trimesh:
    """Reduce face count to <= max_faces, preferring pyvista's quadric decimation."""
    if len(mesh.faces) <= max_faces:
        return mesh

    target_reduction = 1.0 - (max_faces / len(mesh.faces))
    try:
        import pyvista as pv

        faces = np.hstack(
            [np.full((len(mesh.faces), 1), 3, dtype=np.int64), mesh.faces]
        ).ravel()
        pv_mesh = pv.PolyData(mesh.vertices, faces)
        reduced = pv_mesh.decimate(target_reduction)
        reduced_faces = reduced.faces.reshape(-1, 4)[:, 1:4]
        return trimesh.Trimesh(
            vertices=np.asarray(reduced.points),
            faces=reduced_faces,
            process=True,
        )
    except Exception as exc:  # pragma: no cover - depends on optional backend
        print(f"  Decimation unavailable ({exc}); exporting full-resolution mesh")
        return mesh


def export_case(case_id: str, max_faces: int) -> None:
    case_dir = DATA_ROOT / case_id
    visual_stl = case_dir / "mujoco" / "meshes" / case_id / "visual.stl"
    if not visual_stl.is_file():
        raise FileNotFoundError(f"visual.stl not found: {visual_stl}")

    print(f"Loading vessel surface: {visual_stl}")
    mesh = trimesh.load(visual_stl)
    print(f"  Loaded {len(mesh.vertices)} verts / {len(mesh.faces)} faces")
    print(f"  Extents (m): {mesh.extents.tolist()}")

    mesh = decimate(mesh, max_faces)
    print(f"  After decimation: {len(mesh.faces)} faces")

    GODOT_MODELS.mkdir(parents=True, exist_ok=True)
    out_path = GODOT_MODELS / "blood_vessels.glb"
    mesh.export(out_path, file_type="glb")
    size_mb = out_path.stat().st_size / (1024 * 1024)
    print(f"Exported GLB: {out_path} ({size_mb:.2f} MB)")

    # Mirror into the case derived/ directory so manifest references resolve.
    derived = case_dir / "derived" / "blood_vessels.glb"
    derived.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(derived, file_type="glb")
    print(f"Exported GLB: {derived}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Export Godot GLB assets")
    parser.add_argument("--case-id", default="case_001")
    parser.add_argument(
        "--max-faces",
        type=int,
        default=120000,
        help="Decimate the vessel mesh down to at most this many faces",
    )
    parser.add_argument(
        "--phantom",
        default=None,
        help="Export a CathSim phantom visual mesh (e.g. low_tort) instead of the VPP case",
    )
    args = parser.parse_args()
    if args.phantom:
        export_cathsim_phantom(args.phantom)
    else:
        export_case(args.case_id, args.max_faces)


if __name__ == "__main__":
    main()
