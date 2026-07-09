"""Shared VPP asset path resolution.

The Godot client may be launched from the project root, from ``godot_client``,
or against a server started by an IDE. Resolve VPP assets from a small set of
stable candidates instead of relying on the process cwd alone.
"""

from __future__ import annotations

import os
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _as_vpp_root(path: Path) -> list[Path]:
    """Return plausible ``data/vpp_assets`` roots for a configured path."""
    roots = [path]
    if path.name != "vpp_assets":
        roots.append(path / "vpp_assets")
        roots.append(path / "data" / "vpp_assets")
    return roots


def vpp_data_root_candidates() -> list[Path]:
    """Candidate directories that may contain VPP case folders."""
    candidates: list[Path] = []

    env_root = os.environ.get("CATHSIM_VPP_DATA_ROOT")
    if env_root:
        candidates.extend(_as_vpp_root(Path(env_root).expanduser()))

    candidates.append(PROJECT_ROOT / "data" / "vpp_assets")

    cwd = Path.cwd()
    for base in [cwd, *list(cwd.parents)[:4]]:
        candidates.append(base / "data" / "vpp_assets")

    seen: set[str] = set()
    unique: list[Path] = []
    for candidate in candidates:
        try:
            key = str(candidate.resolve())
        except OSError:
            key = str(candidate.absolute())
        if key in seen:
            continue
        seen.add(key)
        unique.append(candidate)
    return unique


def vpp_case_dir(case_id: str) -> Path | None:
    """Return the first existing directory for ``case_id``."""
    for root in vpp_data_root_candidates():
        case_dir = root / case_id
        if case_dir.is_dir():
            return case_dir
    return None


def require_vpp_graph_path(case_id: str) -> Path:
    """Return ``graph/graph.json`` for a case, or raise a diagnostic error."""
    case_dir = vpp_case_dir(case_id)
    if case_dir is not None:
        graph_path = case_dir / "graph" / "graph.json"
        if graph_path.is_file():
            return graph_path

    tried = [str((root / case_id / "graph" / "graph.json").resolve()) for root in vpp_data_root_candidates()]
    raise FileNotFoundError(
        f"Graph not found for case: {case_id}. "
        f"cwd={Path.cwd().resolve()}; tried={tried}"
    )


def resolve_vpp_mujoco_dir(phantom: str) -> str | None:
    """Resolve ``data/vpp_assets/<case_id>/mujoco`` for ``<case_id>_vpp``."""
    if not phantom.endswith("_vpp"):
        return None
    case_id = phantom[: -len("_vpp")]
    case_dir = vpp_case_dir(case_id)
    if case_dir is None:
        return None
    mujoco_dir = case_dir / "mujoco"
    return str(mujoco_dir) if mujoco_dir.is_dir() else None
