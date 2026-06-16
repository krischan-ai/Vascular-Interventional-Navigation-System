from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REQUIRED_FILES = {
    "blood_vessels.vtk": "raw/blood_vessels.vtk",
    "centerline_curves.vtk": "raw/centerline_curves.vtk",
    "Endpoints.fcsv": "raw/Endpoints.fcsv",
    "crossPoints.fcsv": "raw/crossPoints.fcsv",
    "intervenPoints.fcsv": "raw/intervenPoints.fcsv",
    "graph.json": "graph/graph.json",
    "node_radii.json": "graph/node_radii.json",
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file_obj:
        for chunk in iter(lambda: file_obj.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def graph_stats(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as file_obj:
        graph = json.load(file_obj)

    if not isinstance(graph, dict):
        raise ValueError(f"Graph JSON must be an object: {path}")

    edge_count = 0
    weights: list[float] = []
    for key, neighbors in graph.items():
        if not isinstance(key, str):
            raise ValueError(f"Graph node key must be a string: {key!r}")
        if not isinstance(neighbors, list):
            raise ValueError(f"Neighbors for {key!r} must be a list")
        edge_count += len(neighbors)
        for edge in neighbors:
            if not isinstance(edge, list) or len(edge) < 2:
                raise ValueError(f"Invalid edge entry for {key!r}: {edge!r}")
            weights.append(float(edge[1]))

    return {
        "nodes": len(graph),
        "edges": edge_count,
        "min_weight": min(weights) if weights else None,
        "max_weight": max(weights) if weights else None,
    }


def fcsv_point_count(path: Path) -> int:
    count = 0
    with path.open("r", encoding="utf-8") as file_obj:
        for line in file_obj:
            line = line.strip()
            if line and not line.startswith("#"):
                count += 1
    return count


def migrate(source: Path, destination: Path, case_id: str) -> Path:
    if not source.is_dir():
        raise FileNotFoundError(f"VPP source data directory not found: {source}")

    destination.mkdir(parents=True, exist_ok=True)
    for relative_path in ["raw", "graph", "derived", "mujoco/meshes"]:
        (destination / relative_path).mkdir(parents=True, exist_ok=True)

    copied_files: dict[str, dict[str, Any]] = {}
    for source_name, destination_name in REQUIRED_FILES.items():
        src = source / source_name
        dst = destination / destination_name
        if not src.is_file():
            raise FileNotFoundError(f"Required VPP asset not found: {src}")
        shutil.copy2(src, dst)
        copied_files[destination_name] = {
            "source": str(src),
            "size_bytes": dst.stat().st_size,
            "sha256": sha256_file(dst),
        }

    graph_path = destination / "graph/graph.json"
    radii_path = destination / "graph/node_radii.json"
    endpoints_path = destination / "raw/Endpoints.fcsv"
    cross_points_path = destination / "raw/crossPoints.fcsv"
    interven_points_path = destination / "raw/intervenPoints.fcsv"

    with radii_path.open("r", encoding="utf-8") as file_obj:
        radii = json.load(file_obj)
    radii_count = len(radii.get("radii", {})) if isinstance(radii, dict) else 0

    manifest = {
        "case_id": case_id,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "source_directory": str(source),
        "coordinate_system": "LPS",
        "unit": "mm",
        "files": copied_files,
        "graph": graph_stats(graph_path),
        "radii": {
            "path": "graph/node_radii.json",
            "nodes": radii_count,
        },
        "markers": {
            "endpoints": fcsv_point_count(endpoints_path),
            "cross_points": fcsv_point_count(cross_points_path),
            "interven_points": fcsv_point_count(interven_points_path),
        },
        "derived": {
            "blood_vessels_glb": "derived/blood_vessels.glb",
            "centerline_curves_glb": "derived/centerline_curves.glb",
            "targets": "derived/targets.json",
        },
        "mujoco": {
            "xml": "mujoco/low_tort_vpp.xml",
            "mesh_dir": "mujoco/meshes",
        },
    }

    manifest_path = destination / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return manifest_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Migrate VPP data into cathsim.")
    parser.add_argument(
        "--source",
        type=Path,
        default=Path("..") / "vascular_path_planner_sim" / "data",
        help="Source VPP data directory.",
    )
    parser.add_argument(
        "--destination",
        type=Path,
        default=Path("data") / "vpp_assets" / "case_001",
        help="Destination case directory.",
    )
    parser.add_argument("--case-id", default="case_001")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    manifest_path = migrate(
        args.source.resolve(),
        args.destination.resolve(),
        args.case_id,
    )
    print(f"Migrated VPP assets: {manifest_path}")


if __name__ == "__main__":
    main()
