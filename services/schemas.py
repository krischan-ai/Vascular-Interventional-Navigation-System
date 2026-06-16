from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field, field_validator


Vector3 = list[float]


class HealthResponse(BaseModel):
    status: Literal["ok"]
    version: str
    vpp_ready: bool
    cases: list[str]


class CaseSummary(BaseModel):
    case_id: str
    coordinate_system: str
    unit: str
    graph_nodes: int
    graph_edges: int
    endpoints: int


class CaseManifestResponse(BaseModel):
    case_id: str
    manifest: dict[str, Any]


class PlanPathRequest(BaseModel):
    case_id: str = "case_001"
    start: Vector3 = Field(..., description="Start position in LPS millimeters.")
    end: Vector3 = Field(..., description="End position in LPS millimeters.")
    algorithm: Literal["astar"] = "astar"
    smooth: bool = False

    @field_validator("start", "end")
    @classmethod
    def validate_vector3(cls, value: Vector3) -> Vector3:
        if len(value) != 3:
            raise ValueError("Position must contain exactly three coordinates")
        return [float(item) for item in value]


class PlanPathResponse(BaseModel):
    path_id: str
    case_id: str
    coordinate_system: str = "LPS"
    unit: str = "mm"
    waypoints: list[Vector3]
    smooth_waypoints: list[Vector3] | None = None
    length_mm: float
    smooth_length_mm: float | None = None
    max_curvature: float | None = None
    node_count: int
    compute_time_ms: float
