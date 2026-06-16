from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


REQUIRED_PATHS = [
    "manifest.json",
    "raw/blood_vessels.vtk",
    "raw/centerline_curves.vtk",
    "raw/Endpoints.fcsv",
    "raw/crossPoints.fcsv",
    "raw/intervenPoints.fcsv",
    "graph/graph.json",
    "graph/node_radii.json",
]


def parse_node_key(value: str) -> tuple[float, float, float]:
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 3:
        raise ValueError(f"Node key must contain three coordinates: {value!r}")
    return (float(parts[0]), float(parts[1]), float(parts[2]))


def validate_graph(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as file_obj:
        graph = json.load(file_obj)

    if not isinstance(graph, dict):
        raise ValueError("graph.json must be an object adjacency map")

    keys = set(graph.keys())
    edge_count = 0
    weights: list[float] = []
    for key, neighbors in graph.items():
        parse_node_key(key)
        if not isinstance(neighbors, list):
            raise ValueError(f"Neighbors for {key!r} must be a list")
        for edge in neighbors:
            if not isinstance(edge, list) or len(edge) < 2:
                raise ValueError(f"Invalid edge entry for {key!r}: {edge!r}")
            target_key = edge[0]
            if target_key not in keys:
                raise ValueError(f"Missing neighbor {target_key!r} referenced by {key!r}")
            weight = float(edge[1])
            if weight < 0:
                raise ValueError(f"Negative edge weight for {key!r}: {weight}")
            weights.append(weight)
            edge_count += 1

    return {
        "nodes": len(graph),
        "edges": edge_count,
        "min_weight": min(weights) if weights else None,
        "max_weight": max(weights) if weights else None,
    }


def validate_radii(path: Path, graph_path: Path) -> dict[str, Any]:
    with graph_path.open("r", encoding="utf-8") as file_obj:
        graph = json.load(file_obj)
    with path.open("r", encoding="utf-8") as file_obj:
        radii_data = json.load(file_obj)

    if not isinstance(radii_data, dict) or not isinstance(radii_data.get("radii"), dict):
        raise ValueError("node_radii.json must contain a radii object")

    graph_keys = set(graph.keys())
    radii_keys = set(radii_data["radii"].keys())
    missing = graph_keys - radii_keys
    extra = radii_keys - graph_keys

    return {
        "nodes": len(radii_keys),
        "missing_graph_nodes": len(missing),
        "extra_radii_nodes": len(extra),
    }


def count_fcsv_points(path: Path) -> int:
    count = 0
    with path.open("r", encoding="utf-8") as file_obj:
        for line in file_obj:
            line = line.strip()
            if line and not line.startswith("#"):
                count += 1
    return count


def validate_case(case_dir: Path) -> dict[str, Any]:
    if not case_dir.is_dir():
        raise FileNotFoundError(f"Case directory not found: {case_dir}")

    missing_paths = [
        relative_path
        for relative_path in REQUIRED_PATHS
        if not (case_dir / relative_path).is_file()
    ]
    if missing_paths:
        raise FileNotFoundError(f"Missing required files: {missing_paths}")

    graph_path = case_dir / "graph/graph.json"
    radii_path = case_dir / "graph/node_radii.json"

    result = {
        "case_dir": str(case_dir),
        "graph": validate_graph(graph_path),
        "radii": validate_radii(radii_path, graph_path),
        "markers": {
            "endpoints": count_fcsv_points(case_dir / "raw/Endpoints.fcsv"),
            "cross_points": count_fcsv_points(case_dir / "raw/crossPoints.fcsv"),
            "interven_points": count_fcsv_points(case_dir / "raw/intervenPoints.fcsv"),
        },
        "derived": {
            "targets_exists": (case_dir / "derived/targets.json").is_file(),
        },
    }
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate migrated VPP assets.")
    parser.add_argument(
        "case_dir",
        nargs="?",
        type=Path,
        default=Path("data") / "vpp_assets" / "case_001",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = validate_case(args.case_dir.resolve())
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
