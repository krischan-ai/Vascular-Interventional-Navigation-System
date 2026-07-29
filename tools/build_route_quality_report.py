from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from services.graph_connectivity import connected_components
from services.graph_loader import GraphLoader
from services.path_planner import PathPlanner


@dataclass(frozen=True)
class Endpoint:
    route_id: str
    label: str
    position_lps_mm: list[float]


def build_route_quality_report(
    case_dir: Path,
    *,
    entry: list[float] | None = None,
    smooth: bool = True,
    smooth_factor: float = 0.5,
    output_path: Path | None = None,
) -> Path:
    """Audit graph, radii, and endpoint route quality for a VPP case.

    Source graph and target coordinates are LPS millimeters. The report keeps
    millimeter units so it can be compared directly with graph assets.
    """
    case_dir = case_dir.resolve()
    graph_path = case_dir / "graph" / "graph.json"
    radii_path = case_dir / "graph" / "node_radii.json"
    targets_path = case_dir / "derived" / "targets.json"
    if not graph_path.is_file():
        raise FileNotFoundError(f"graph.json not found: {graph_path}")
    if not targets_path.is_file():
        raise FileNotFoundError(f"targets.json not found: {targets_path}")

    raw_graph = json.loads(graph_path.read_text(encoding="utf-8"))
    if not isinstance(raw_graph, dict):
        raise ValueError(f"graph.json must be an object adjacency map: {graph_path}")

    loader = GraphLoader(graph_path)
    components = connected_components(raw_graph)
    targets = json.loads(targets_path.read_text(encoding="utf-8"))
    endpoints = _endpoint_items(targets)
    if not endpoints:
        raise ValueError(f"No endpoints found in {targets_path}")

    entry_mm = entry or _default_entry(targets, endpoints)
    planner = PathPlanner(graph_path)
    route_reports = []
    unreachable_targets: list[str] = []
    for endpoint in endpoints:
        route_reports.append(
            _audit_route(
                planner,
                endpoint,
                entry_mm,
                smooth=smooth,
                smooth_factor=smooth_factor,
            )
        )
        if not route_reports[-1]["reachable"]:
            unreachable_targets.append(endpoint.route_id)

    report = {
        "schema_version": 1,
        "case_id": case_dir.name,
        "coordinate_system": "LPS",
        "unit": "mm",
        "source": {
            "case_dir": str(case_dir),
            "graph": str(graph_path),
            "node_radii": str(radii_path) if radii_path.is_file() else None,
            "targets": str(targets_path),
        },
        "graph": {
            "node_count": loader.stats.node_count,
            "edge_count": loader.stats.edge_count,
            "component_count": len(components),
            "component_sizes": [len(component) for component in components],
            "min_edge_weight_mm": loader.stats.min_weight,
            "max_edge_weight_mm": loader.stats.max_weight,
        },
        "radii": _audit_radii(radii_path, set(raw_graph.keys())),
        "targets": {
            "entry_lps_mm": entry_mm,
            "endpoint_count": len(endpoints),
            "reachable_count": len(endpoints) - len(unreachable_targets),
            "unreachable_count": len(unreachable_targets),
            "unreachable_targets": unreachable_targets,
        },
        "routes": route_reports,
        "warnings": _warnings(components, route_reports),
    }

    out_path = output_path or (case_dir / "derived" / "route_quality_report.json")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return out_path


def _audit_route(
    planner: PathPlanner,
    endpoint: Endpoint,
    entry_mm: list[float],
    *,
    smooth: bool,
    smooth_factor: float,
) -> dict[str, Any]:
    base = {
        "route_id": endpoint.route_id,
        "label": endpoint.label,
        "target_lps_mm": endpoint.position_lps_mm,
    }
    try:
        result = planner.plan(
            entry_mm,
            endpoint.position_lps_mm,
            smooth=smooth,
            smooth_factor=smooth_factor,
        )
    except Exception as exc:
        return {
            **base,
            "reachable": False,
            "error": f"{type(exc).__name__}: {exc}",
        }

    radii = result.smooth_radii if result.smooth_radii is not None else result.radii
    return {
        **base,
        "reachable": True,
        "node_count": result.node_count,
        "length_mm": result.length_mm,
        "smooth_length_mm": result.smooth_length_mm,
        "max_curvature": result.max_curvature,
        "has_radii": radii is not None,
        "min_radius_mm": min(radii) if radii else None,
        "max_radius_mm": max(radii) if radii else None,
    }


def _audit_radii(radii_path: Path, graph_keys: set[str]) -> dict[str, Any]:
    if not radii_path.is_file():
        return {
            "exists": False,
            "node_count": 0,
            "matched_node_count": 0,
            "missing_node_count": len(graph_keys),
            "missing_rate": 1.0 if graph_keys else 0.0,
            "min_radius_mm": None,
            "max_radius_mm": None,
        }
    raw = json.loads(radii_path.read_text(encoding="utf-8"))
    radii = raw.get("radii", {}) if isinstance(raw, dict) else {}
    if not isinstance(radii, dict):
        radii = {}
    matched = graph_keys.intersection(radii.keys())
    values = [float(radii[key]) for key in matched]
    missing_count = len(graph_keys) - len(matched)
    return {
        "exists": True,
        "node_count": len(radii),
        "matched_node_count": len(matched),
        "missing_node_count": missing_count,
        "missing_rate": missing_count / len(graph_keys) if graph_keys else 0.0,
        "min_radius_mm": min(values) if values else None,
        "max_radius_mm": max(values) if values else None,
    }


def _endpoint_items(targets: dict[str, Any]) -> list[Endpoint]:
    endpoints = targets.get("endpoints", {})
    items: list[Endpoint] = []
    if not isinstance(endpoints, dict):
        return items
    for label, data in endpoints.items():
        if not isinstance(data, dict):
            continue
        position = data.get("position_lps")
        if isinstance(position, list) and len(position) == 3:
            route_id = str(label).replace("Endpoints-", "endpoints_").replace(
                "Endpoint-",
                "endpoint_",
            )
            items.append(
                Endpoint(
                    route_id=route_id,
                    label=str(label),
                    position_lps_mm=[float(v) for v in position],
                )
            )
    return items


def _default_entry(targets: dict[str, Any], endpoints: list[Endpoint]) -> list[float]:
    preferred = targets.get("endpoints", {}).get("Endpoints-24")
    if isinstance(preferred, dict):
        position = preferred.get("position_lps")
        if isinstance(position, list) and len(position) == 3:
            return [float(v) for v in position]
    return endpoints[0].position_lps_mm


def _warnings(components: list[list[str]], routes: list[dict[str, Any]]) -> list[str]:
    warnings = []
    if len(components) > 1:
        warnings.append(f"graph has {len(components)} connected components")
    unreachable = [route["route_id"] for route in routes if not route["reachable"]]
    if unreachable:
        warnings.append(f"unreachable targets: {', '.join(unreachable)}")
    missing_radii = [route["route_id"] for route in routes if route.get("reachable") and not route.get("has_radii")]
    if missing_radii:
        warnings.append(f"routes missing complete radii: {', '.join(missing_radii)}")
    return warnings


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build route_quality_report.json for a VPP case.")
    parser.add_argument(
        "case_dir",
        nargs="?",
        type=Path,
        default=Path("data") / "vpp_assets" / "case_001",
    )
    parser.add_argument("--entry", nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument("--no-smooth", action="store_true")
    parser.add_argument("--smooth-factor", type=float, default=0.5)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    out = build_route_quality_report(
        args.case_dir,
        entry=args.entry,
        smooth=not args.no_smooth,
        smooth_factor=args.smooth_factor,
        output_path=args.output,
    )
    print(f"Wrote route quality report: {out}")


if __name__ == "__main__":
    main()
