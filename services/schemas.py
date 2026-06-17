from __future__ import annotations

from datetime import datetime
from typing import Any, Literal

from pydantic import BaseModel, Field, field_validator


Vector3 = list[float]
SafetyStatus = Literal["STANDBY", "SAFE_NAV", "DANGER_WARNING", "COLLISION_STOP"]


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


# Session models


class SessionStartRequest(BaseModel):
    phantom: str = Field(
        default="low_tort",
        description="Phantom model: low_tort, phantom2, phantom3, phantom4",
    )
    target: str = Field(
        default="bca",
        description="Target site: bca, lcca",
    )
    use_pixels: bool = Field(
        default=False,
        description="Include pixel observations",
    )


class NavigationStateResponse(BaseModel):
    tip_position: Vector3 = Field(description="Guidewire tip position (MuJoCo coords)")
    tip_direction: Vector3 = Field(description="Tip direction unit vector")
    tip_quaternion: Vector3 = Field(
        default_factory=lambda: [0.0, 0.0, 0.0, 1.0],
        description="Tip orientation quaternion [x, y, z, w]",
    )
    velocity: float = Field(description="Tip velocity (m/s)")
    contact_force: float = Field(description="Contact force magnitude (N)")
    wall_distance: float = Field(default=0.0, description="Min wall distance (m)")
    curvature: float = Field(default=0.0, description="Local tip curvature (m^-1)")
    episode_length: int = Field(description="Steps in current episode")
    target_position: Vector3 = Field(description="Target position")
    path_progress: float = Field(default=0.0, description="Planned-path progress [0, 1]")
    path_deviation: float = Field(default=0.0, description="Deviation from path (m)")
    reward: float = Field(description="Reward from last step")
    done: bool = Field(description="Episode terminated")
    safety_status: SafetyStatus = Field(default="STANDBY", description="Safety status")


class SessionStartResponse(BaseModel):
    session_id: str
    phantom: str
    target: str
    state: NavigationStateResponse


class SessionInfoResponse(BaseModel):
    session_id: str
    phantom: str
    target: str
    created_at: str
    last_active: str
    episode_count: int
    total_steps: int


class StepRequest(BaseModel):
    delta_push: float = Field(
        ...,
        ge=-1.0,
        le=1.0,
        description="Push force coefficient [-1.0, 1.0], positive = forward",
    )
    delta_rotate: float = Field(
        ...,
        ge=-1.0,
        le=1.0,
        description="Rotation force coefficient [-1.0, 1.0], positive = clockwise",
    )


class StepResponse(BaseModel):
    session_id: str
    state: NavigationStateResponse
    step_count: int


class ResetResponse(BaseModel):
    session_id: str
    state: NavigationStateResponse
    episode_count: int
