from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException

from services.path_planner import PathPlanner
from services.schemas import (
    CaseManifestResponse,
    CaseSummary,
    HealthResponse,
    PlanPathRequest,
    PlanPathResponse,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATA_ROOT = PROJECT_ROOT / "data" / "vpp_assets"
VERSION = "0.1.0"

app = FastAPI(
    title="CathSim VPP Services",
    version=VERSION,
    description="Path planning and asset access services for CathSim/VPP integration.",
)


def list_case_ids() -> list[str]:
    if not DATA_ROOT.is_dir():
        return []
    return sorted(
        path.name
        for path in DATA_ROOT.iterdir()
        if path.is_dir() and (path / "manifest.json").is_file()
    )


def case_dir(case_id: str) -> Path:
    path = DATA_ROOT / case_id
    if not path.is_dir():
        raise HTTPException(status_code=404, detail=f"Case not found: {case_id}")
    return path


def load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise HTTPException(status_code=404, detail=f"File not found: {path.name}")
    with path.open("r", encoding="utf-8") as file_obj:
        data = json.load(file_obj)
    if not isinstance(data, dict):
        raise HTTPException(status_code=500, detail=f"Expected JSON object: {path.name}")
    return data


def load_manifest(case_id: str) -> dict[str, Any]:
    return load_json(case_dir(case_id) / "manifest.json")


@lru_cache(maxsize=8)
def get_path_planner(case_id: str) -> PathPlanner:
    graph_path = case_dir(case_id) / "graph" / "graph.json"
    if not graph_path.is_file():
        raise HTTPException(status_code=404, detail=f"Graph not found for case: {case_id}")
    return PathPlanner(graph_path)


def summarize_case(manifest: dict[str, Any]) -> CaseSummary:
    graph = manifest.get("graph", {})
    markers = manifest.get("markers", {})
    return CaseSummary(
        case_id=str(manifest.get("case_id", "")),
        coordinate_system=str(manifest.get("coordinate_system", "LPS")),
        unit=str(manifest.get("unit", "mm")),
        graph_nodes=int(graph.get("nodes", 0)),
        graph_edges=int(graph.get("edges", 0)),
        endpoints=int(markers.get("endpoints", 0)),
    )


@app.get("/api/v1/health", response_model=HealthResponse)
def health() -> HealthResponse:
    case_ids = list_case_ids()
    return HealthResponse(
        status="ok",
        version=VERSION,
        vpp_ready=bool(case_ids),
        cases=case_ids,
    )


@app.get("/api/v1/assets/cases", response_model=list[CaseSummary])
def list_cases() -> list[CaseSummary]:
    return [summarize_case(load_manifest(case_id)) for case_id in list_case_ids()]


@app.get("/api/v1/assets/cases/{case_id}", response_model=CaseManifestResponse)
def get_case(case_id: str) -> CaseManifestResponse:
    return CaseManifestResponse(case_id=case_id, manifest=load_manifest(case_id))


@app.post("/api/v1/path/plan", response_model=PlanPathResponse)
def plan_path(request: PlanPathRequest) -> PlanPathResponse:
    planner = get_path_planner(request.case_id)
    try:
        result = planner.plan(
            start=request.start,
            end=request.end,
            algorithm=request.algorithm,
        )
    except ValueError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc

    return PlanPathResponse(
        case_id=request.case_id,
        **result.as_dict(),
    )


def main() -> None:
    import uvicorn

    uvicorn.run("services.main:app", host="0.0.0.0", port=8000, reload=False)


if __name__ == "__main__":
    main()
