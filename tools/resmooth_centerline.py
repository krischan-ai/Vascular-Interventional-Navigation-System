"""Re-smooth a built-in phantom's render centerline with a stronger B-spline.

A phantom's ``meshes/<phantom>/centerline.json`` carries the guided render path
(``waypoints``, entry -> target_root). Some were generated with a near-zero
smoothing factor that interpolates voxel-staircase noise, leaving sharp kinks
that read as a jagged wire in the frontend. This re-fits the existing waypoints
with a larger smoothing factor (same scipy B-spline as PathPlanner) so the guided
guidewire renders as a smooth curve, and records the parameters back into the
file's ``smooth`` block for reproducibility.

The vessel mesh and physics are untouched -- only the render/guided path changes.

Usage:
    python tools/resmooth_centerline.py segment_part --smoothing 5e-6
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from services.path_planner import PathPlanner  # noqa: E402

_MESH_DIR = _ROOT / "src/cathsim/dm/components/phantom_assets/meshes"


def _max_turn_deg(points: np.ndarray) -> float:
    """Largest turning angle (deg) between consecutive polyline segments."""
    worst = 0.0
    for i in range(1, len(points) - 1):
        v1 = points[i] - points[i - 1]
        v2 = points[i + 1] - points[i]
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        if n1 > 0 and n2 > 0:
            cos = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
            worst = max(worst, math.degrees(math.acos(cos)))
    return worst


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phantom", help="phantom name, e.g. segment_part")
    parser.add_argument(
        "--smoothing",
        type=float,
        default=5e-6,
        help="B-spline smoothing factor (0 = interpolating; larger = smoother)",
    )
    parser.add_argument(
        "--num-points",
        type=int,
        default=None,
        help="output point count (default: keep the input count)",
    )
    args = parser.parse_args()

    path = _MESH_DIR / args.phantom / "centerline.json"
    if not path.is_file():
        raise SystemExit(f"not found: {path}")

    data = json.loads(path.read_text(encoding="utf-8"))
    waypoints = data.get("waypoints")
    if not isinstance(waypoints, list) or len(waypoints) < 4:
        raise SystemExit("centerline.json has no usable 'waypoints'")

    src = np.asarray(waypoints, dtype=float)
    num_points = args.num_points or len(src)

    planner = PathPlanner.__new__(PathPlanner)  # smooth_path needs no graph state
    result = planner.smooth_path(
        [list(p) for p in src],
        smoothing_factor=args.smoothing,
        num_points=num_points,
    )
    smooth = np.asarray(result["waypoints"], dtype=float)

    # Deviation of the original waypoints from the new smooth curve (sanity check
    # that smoothing did not pull the path out of the lumen).
    from scipy.spatial import cKDTree

    dist_mm = cKDTree(smooth).query(src)[0] * 1000.0

    data["waypoints"] = [[float(x) for x in p] for p in smooth]
    data["length_m"] = float(result["length_mm"])  # smooth_path returns meters here
    data["max_curvature_per_m"] = float(result["max_curvature"])
    data["smooth"] = {
        "method": "bspline_splprep",
        "degree": 3,
        "smoothing_factor": float(args.smoothing),
        "resmoothed_from": len(src),
    }
    path.write_text(json.dumps(data, indent=2), encoding="utf-8")

    print(f"[resmooth] {args.phantom}: {len(src)} -> {len(smooth)} waypoints")
    print(
        f"  max turn {_max_turn_deg(src):.1f} deg -> {_max_turn_deg(smooth):.1f} deg"
        f"  | deviation from original: mean {dist_mm.mean():.2f}mm "
        f"max {dist_mm.max():.2f}mm"
    )
    print(f"  wrote {path}")


if __name__ == "__main__":
    main()
