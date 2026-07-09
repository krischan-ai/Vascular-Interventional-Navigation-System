from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from services.path_planner import PathPlanner


def _path_length(points: list[list[float]]) -> float:
    total = 0.0
    for a, b in zip(points, points[1:]):
        total += sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)) ** 0.5
    return total


def _endpoint_items(targets: dict) -> list[tuple[str, list[float]]]:
    endpoints = targets.get("endpoints", {})
    items = []
    for label, data in endpoints.items():
        position = data.get("position_lps")
        if isinstance(position, list) and len(position) == 3:
            route_id = label.replace("Endpoints-", "endpoints_").replace("Endpoint-", "endpoint_")
            items.append((route_id, [float(v) for v in position]))
    return items


def _default_entry(targets: dict, endpoints: list[tuple[str, list[float]]]) -> list[float]:
    preferred = targets.get("endpoints", {}).get("Endpoints-24")
    if isinstance(preferred, dict):
        position = preferred.get("position_lps")
        if isinstance(position, list) and len(position) == 3:
            return [float(v) for v in position]
    return endpoints[0][1]


def build_routes(
    case_dir: Path,
    *,
    entry: list[float] | None = None,
    smooth: bool = True,
    smooth_factor: float = 0.5,
) -> Path:
    """Build aorta_tree-style route cache for one VPP case.

    The source graph/targets use LPS millimeters. The generated ``routes.json``
    stores MuJoCo-frame meters, matching the built-in aorta_tree route schema.
    """
    graph_path = case_dir / "graph" / "graph.json"
    targets_path = case_dir / "derived" / "targets.json"
    if not graph_path.is_file():
        raise FileNotFoundError(f"graph.json not found: {graph_path}")
    if not targets_path.is_file():
        raise FileNotFoundError(f"targets.json not found: {targets_path}")

    targets = json.loads(targets_path.read_text(encoding="utf-8"))
    endpoints = _endpoint_items(targets)
    if not endpoints:
        raise ValueError(f"No endpoints found in {targets_path}")

    entry_mm = entry or _default_entry(targets, endpoints)
    planner = PathPlanner(graph_path)
    routes = {}
    for route_id, target_mm in endpoints:
        result = planner.plan(
            entry_mm,
            target_mm,
            smooth=smooth,
            smooth_factor=smooth_factor,
        )
        if result.smooth_waypoints is not None:
            waypoints_mm = result.smooth_waypoints
            radii_mm = result.smooth_radii
        else:
            waypoints_mm = result.waypoints
            radii_mm = result.radii

        waypoints_m = [[float(c) / 1000.0 for c in point] for point in waypoints_mm]
        radii_m = [float(r) / 1000.0 for r in radii_mm] if radii_mm is not None else None
        routes[route_id] = {
            "target": waypoints_m[-1],
            "length_m": round(_path_length(waypoints_m), 6),
            "n_waypoints": len(waypoints_m),
            "waypoints": waypoints_m,
        }
        if radii_m is not None:
            routes[route_id]["radius_m"] = radii_m

    output = {
        "phantom": f"{case_dir.name}_vpp",
        "case_id": case_dir.name,
        "coordinate_system": "mujoco",
        "unit": "m",
        "source_entry_lps_mm": entry_mm,
        "entry": routes[endpoints[0][0]]["waypoints"][0],
        "n_routes": len(routes),
        "routes": routes,
    }

    out_path = case_dir / "derived" / "routes.json"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(output, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return out_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build VPP routes.json from targets + graph.")
    parser.add_argument(
        "case_dir",
        nargs="?",
        type=Path,
        default=Path("data") / "vpp_assets" / "case_001",
    )
    parser.add_argument("--entry", nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument("--no-smooth", action="store_true")
    parser.add_argument("--smooth-factor", type=float, default=0.5)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    out = build_routes(
        args.case_dir.resolve(),
        entry=args.entry,
        smooth=not args.no_smooth,
        smooth_factor=args.smooth_factor,
    )
    print(f"Wrote VPP routes: {out}")


if __name__ == "__main__":
    main()
