#!/usr/bin/env python
"""Audit render mesh geometry for clinical vessel visual QA.

The audit is intentionally separate from physics/SDF assets. It answers whether
a render mesh is suitable for close clinical viewing: enough faces, smooth
adjacent normals, few hard seams, and no unexpected open/non-manifold edges.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
import trimesh


def _percentile(values: np.ndarray, pct: float) -> float:
    if values.size == 0:
        return 0.0
    return round(float(np.percentile(values, pct)), 4)


def audit_mesh(mesh: trimesh.Trimesh, sharp_angle_deg: float = 35.0) -> dict[str, Any]:
    """Return geometry QA metrics for a render mesh."""
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError("audit_mesh expects a trimesh.Trimesh")

    edge_counts = np.bincount(mesh.edges_unique_inverse)
    boundary_edges = int(np.count_nonzero(edge_counts == 1))
    nonmanifold_edges = int(np.count_nonzero(edge_counts > 2))

    angles_deg = np.degrees(mesh.face_adjacency_angles)
    sharp_edges = int(np.count_nonzero(angles_deg >= sharp_angle_deg))
    adjacency_count = int(angles_deg.size)
    sharp_ratio = float(sharp_edges / adjacency_count) if adjacency_count else 0.0

    return {
        "vertices": int(len(mesh.vertices)),
        "faces": int(len(mesh.faces)),
        "watertight": bool(mesh.is_watertight),
        "extents": [round(float(v), 6) for v in mesh.extents],
        "boundary_edges": boundary_edges,
        "nonmanifold_edges": nonmanifold_edges,
        "adjacent_edge_count": adjacency_count,
        "sharp_angle_deg": float(sharp_angle_deg),
        "sharp_edges": sharp_edges,
        "sharp_edge_ratio": round(sharp_ratio, 6),
        "adjacency_angle_p50_deg": _percentile(angles_deg, 50),
        "adjacency_angle_p95_deg": _percentile(angles_deg, 95),
        "adjacency_angle_p99_deg": _percentile(angles_deg, 99),
    }


def audit_file(path: Path, sharp_angle_deg: float = 35.0) -> dict[str, Any]:
    mesh = trimesh.load(path, force="mesh")
    report = audit_mesh(mesh, sharp_angle_deg=sharp_angle_deg)
    report["path"] = str(path)
    report["size_mb"] = round(path.stat().st_size / (1024 * 1024), 3)
    return report


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Audit visual vessel mesh geometry")
    parser.add_argument("mesh", type=Path, help="STL/GLB/OBJ mesh to audit")
    parser.add_argument(
        "--sharp-angle-deg",
        type=float,
        default=35.0,
        help="Adjacent face angle threshold counted as a hard visual edge",
    )
    parser.add_argument("--json", action="store_true", help="Emit machine-readable JSON")
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    report = audit_file(args.mesh, sharp_angle_deg=args.sharp_angle_deg)
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return
    for key, value in report.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
