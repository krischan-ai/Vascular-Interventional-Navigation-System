from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, WebSocket

from services.path_planner import PathPlanner
from services.schemas import (
    CaseManifestResponse,
    CaseSummary,
    HealthResponse,
    NavigationStateResponse,
    PlanPathRequest,
    PlanPathResponse,
    ResetResponse,
    SessionInfoResponse,
    SessionStartRequest,
    SessionStartResponse,
    StepRequest,
    StepResponse,
)
from services.session_manager import get_session_manager


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


# ============================================================================
# Session Management Endpoints
# ============================================================================


def _state_to_response(state, engine) -> NavigationStateResponse:
    """Convert NavigationState to response model."""
    return NavigationStateResponse(
        tip_position=state.tip_position,
        tip_direction=state.tip_direction,
        velocity=state.velocity,
        contact_force=state.contact_force,
        episode_length=state.episode_length,
        target_position=state.target_position,
        reward=state.reward,
        done=state.done,
        safety_status=engine.get_safety_status(state),
    )


@app.post("/api/v1/session/start", response_model=SessionStartResponse)
def session_start(request: SessionStartRequest) -> SessionStartResponse:
    """Start a new CathSim navigation session."""
    manager = get_session_manager()
    try:
        session_id, state = manager.create_session(
            phantom=request.phantom,
            target=request.target,
            use_pixels=request.use_pixels,
        )
    except RuntimeError as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc

    engine = manager.get_session(session_id)
    return SessionStartResponse(
        session_id=session_id,
        phantom=request.phantom,
        target=request.target,
        state=_state_to_response(state, engine),
    )


@app.get("/api/v1/session", response_model=list[SessionInfoResponse])
def list_sessions() -> list[SessionInfoResponse]:
    """List all active sessions."""
    manager = get_session_manager()
    sessions = manager.list_sessions()
    return [
        SessionInfoResponse(
            session_id=info.session_id,
            phantom=info.phantom,
            target=info.target,
            created_at=info.created_at.isoformat(),
            last_active=info.last_active.isoformat(),
            episode_count=info.episode_count,
            total_steps=info.total_steps,
        )
        for info in sessions
    ]


@app.get("/api/v1/session/{session_id}", response_model=SessionInfoResponse)
def get_session_info(session_id: str) -> SessionInfoResponse:
    """Get session metadata."""
    manager = get_session_manager()
    try:
        info = manager.get_session_info(session_id)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc

    return SessionInfoResponse(
        session_id=info.session_id,
        phantom=info.phantom,
        target=info.target,
        created_at=info.created_at.isoformat(),
        last_active=info.last_active.isoformat(),
        episode_count=info.episode_count,
        total_steps=info.total_steps,
    )


@app.post("/api/v1/session/{session_id}/step", response_model=StepResponse)
def session_step(session_id: str, request: StepRequest) -> StepResponse:
    """Execute one simulation step in the specified session."""
    manager = get_session_manager()
    try:
        state = manager.step(
            session_id=session_id,
            delta_push=request.delta_push,
            delta_rotate=request.delta_rotate,
        )
        info = manager.get_session_info(session_id)
        engine = manager.get_session(session_id)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except RuntimeError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc

    return StepResponse(
        session_id=session_id,
        state=_state_to_response(state, engine),
        step_count=info.total_steps,
    )


@app.post("/api/v1/session/{session_id}/reset", response_model=ResetResponse)
def session_reset(session_id: str) -> ResetResponse:
    """Reset the environment in the specified session."""
    manager = get_session_manager()
    try:
        state = manager.reset_session(session_id)
        info = manager.get_session_info(session_id)
        engine = manager.get_session(session_id)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc

    return ResetResponse(
        session_id=session_id,
        state=_state_to_response(state, engine),
        episode_count=info.episode_count,
    )


@app.delete("/api/v1/session/{session_id}")
def session_close(session_id: str) -> dict:
    """Close and cleanup a session."""
    manager = get_session_manager()
    if not manager.close_session(session_id):
        raise HTTPException(status_code=404, detail=f"Session not found: {session_id}")

    return {"status": "closed", "session_id": session_id}


# ============================================================================
# WebSocket Endpoints
# ============================================================================


@app.websocket("/ws/session")
async def websocket_session(websocket: WebSocket) -> None:
    """WebSocket endpoint for real-time session control.

    Protocol:
    - Client sends: control, session_start, session_stop, reset, pong
    - Server sends: state_update, state_batch, error, ping, session_started
    """
    from services.websocket_handler import get_websocket_handler

    handler = get_websocket_handler(get_session_manager())
    await handler.handle_connection(websocket)


def main() -> None:
    import uvicorn

    uvicorn.run("services.main:app", host="0.0.0.0", port=8000, reload=False)


if __name__ == "__main__":
    main()
