#!/usr/bin/env python
"""Export Godot-ready GLB assets from CathSim/VPP meshes.

Godot imports glTF/GLB but cannot read VTK or STL scene meshes directly. This
tool converts the high-resolution vessel surface (``visual.stl``, already in the
MuJoCo/guidewire meter frame) into a decimated GLB so the Godot client can
overlay the guidewire (whose positions come from MuJoCo physics in the same
frame) without any coordinate conversion.

Usage:
    python tools/export_godot_assets.py [--case-id CASE_ID] [--quality visual_high]
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Literal

import numpy as np
import trimesh

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATA_ROOT = PROJECT_ROOT / "data" / "vpp_assets"
GODOT_MODELS = PROJECT_ROOT / "godot_client" / "assets" / "models"
PHANTOM_MESH_ROOT = (
    PROJECT_ROOT / "src" / "cathsim" / "dm" / "components" / "phantom_assets" / "meshes"
)

Quality = Literal["visual_high", "preview"]


def default_max_faces(quality: Quality) -> int:
    """Face budget for a quality preset."""
    return 300000 if quality == "visual_high" else 120000


def output_stem(base: str, quality: Quality) -> str:
    return f"{base}_{quality}"


def export_cathsim_phantom(
    phantom_name: str,
    quality: Quality,
    max_faces: int,
    no_decimate: bool,
    smooth_normals: bool,
    taubin_smooth_iter: int,
) -> None:
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
    mesh = prepare_visual_mesh(
        mesh,
        max_faces=max_faces,
        no_decimate=no_decimate,
        smooth_normals=smooth_normals,
        taubin_smooth_iter=taubin_smooth_iter,
    )

    GODOT_MODELS.mkdir(parents=True, exist_ok=True)
    out_path = GODOT_MODELS / f"{output_stem(phantom_name, quality)}.glb"
    mesh.export(out_path, file_type="glb")
    size_mb = out_path.stat().st_size / (1024 * 1024)
    print(f"Exported GLB: {out_path} ({size_mb:.2f} MB)")


def decimate(mesh: trimesh.Trimesh, max_faces: int) -> trimesh.Trimesh:
    """Reduce face count to <= max_faces, preferring pyvista's quadric decimation."""
    if len(mesh.faces) <= max_faces:
        return mesh

    # PyVista's quadric decimator can overshoot the requested face count by a
    # small amount, so aim a touch below the public budget.
    target_faces = max(3, int(max_faces * 0.995))
    target_reduction = 1.0 - (target_faces / len(mesh.faces))
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


def smooth_mesh_taubin(mesh: trimesh.Trimesh, iterations: int) -> trimesh.Trimesh:
    """Apply optional Taubin smoothing when trimesh exposes the backend."""
    if iterations <= 0:
        return mesh
    try:
        smoothed = mesh.copy()
        trimesh.smoothing.filter_taubin(smoothed, iterations=iterations)
        return smoothed
    except Exception as exc:  # pragma: no cover - depends on optional backend details
        print(f"  Taubin smoothing unavailable ({exc}); keeping unsmoothed mesh")
        return mesh


def prepare_visual_mesh(
    mesh: trimesh.Trimesh,
    max_faces: int,
    no_decimate: bool = False,
    smooth_normals: bool = False,
    taubin_smooth_iter: int = 0,
) -> trimesh.Trimesh:
    """Prepare a render mesh without touching collision/SDF assets."""
    mesh = smooth_mesh_taubin(mesh, taubin_smooth_iter)
    if no_decimate:
        print("  Decimation disabled (--no-decimate)")
    else:
        before = len(mesh.faces)
        mesh = decimate(mesh, max_faces)
        print(f"  After decimation: {len(mesh.faces)} faces (from {before})")

    if smooth_normals:
        mesh = mesh.copy()
        mesh.fix_normals()
        # Touch vertex_normals so trimesh computes smooth per-vertex normals before export.
        _ = mesh.vertex_normals
        print("  Smooth normals requested")
    return mesh


def export_case(
    case_id: str,
    quality: Quality,
    max_faces: int,
    no_decimate: bool,
    smooth_normals: bool,
    taubin_smooth_iter: int,
) -> None:
    case_dir = DATA_ROOT / case_id
    visual_stl = case_dir / "mujoco" / "meshes" / case_id / "visual.stl"
    if not visual_stl.is_file():
        raise FileNotFoundError(f"visual.stl not found: {visual_stl}")

    print(f"Loading vessel surface: {visual_stl}")
    mesh = trimesh.load(visual_stl)
    print(f"  Loaded {len(mesh.vertices)} verts / {len(mesh.faces)} faces")
    print(f"  Extents (m): {mesh.extents.tolist()}")

    mesh = prepare_visual_mesh(
        mesh,
        max_faces=max_faces,
        no_decimate=no_decimate,
        smooth_normals=smooth_normals,
        taubin_smooth_iter=taubin_smooth_iter,
    )

    GODOT_MODELS.mkdir(parents=True, exist_ok=True)
    filename = f"{output_stem('blood_vessels', quality)}.glb"
    out_path = GODOT_MODELS / filename
    mesh.export(out_path, file_type="glb")
    size_mb = out_path.stat().st_size / (1024 * 1024)
    print(f"Exported GLB: {out_path} ({size_mb:.2f} MB)")

    # Mirror into the case derived/ directory so manifest references resolve.
    derived = case_dir / "derived" / filename
    derived.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(derived, file_type="glb")
    print(f"Exported GLB: {derived}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Export Godot GLB assets")
    parser.add_argument("--case-id", default="case_001")
    parser.add_argument(
        "--quality",
        choices=("visual_high", "preview"),
        default="visual_high",
        help="Output quality preset and filename suffix",
    )
    parser.add_argument(
        "--max-faces",
        type=int,
        default=None,
        help="Override the quality preset's face budget",
    )
    parser.add_argument("--no-decimate", action="store_true", help="Export full face count")
    parser.add_argument(
        "--smooth-normals",
        action="store_true",
        help="Recompute smooth vertex normals before exporting",
    )
    parser.add_argument(
        "--taubin-smooth-iter",
        type=int,
        default=0,
        help="Apply N Taubin smoothing iterations before export",
    )
    parser.add_argument(
        "--phantom",
        default=None,
        help="Export a CathSim phantom visual mesh (e.g. low_tort) instead of the VPP case",
    )
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    max_faces = args.max_faces or default_max_faces(args.quality)
    if args.phantom:
        export_cathsim_phantom(
            args.phantom,
            args.quality,
            max_faces,
            args.no_decimate,
            args.smooth_normals,
            args.taubin_smooth_iter,
        )
    else:
        export_case(
            args.case_id,
            args.quality,
            max_faces,
            args.no_decimate,
            args.smooth_normals,
            args.taubin_smooth_iter,
        )


if __name__ == "__main__":
    main()
