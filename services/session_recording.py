from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

from services.navigation_engine import NavigationState


@dataclass(frozen=True)
class SessionFrame:
    t: float
    control: dict[str, float]
    state: dict[str, Any]


@dataclass(frozen=True)
class SessionScore:
    duration_s: float
    steps: int
    completed: bool
    completion: float
    mean_deviation: float
    max_deviation: float
    max_contact_force: float
    max_risk_score: float
    danger_events: int

    def as_dict(self) -> dict[str, Any]:
        return {
            "duration_s": self.duration_s,
            "steps": self.steps,
            "completed": self.completed,
            "completion": self.completion,
            "mean_deviation": self.mean_deviation,
            "max_deviation": self.max_deviation,
            "max_contact_force": self.max_contact_force,
            "max_risk_score": self.max_risk_score,
            "danger_events": self.danger_events,
        }


class SessionRecorder:
    """Append-only JSONL recorder for training/HCI review sessions."""

    def __init__(self, path: str | Path, metadata: dict[str, Any] | None = None) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.metadata = metadata or {}
        self._frames: list[SessionFrame] = []

    def record(
        self,
        *,
        t: float,
        control: dict[str, float],
        state: NavigationState,
    ) -> None:
        self._frames.append(
            SessionFrame(
                t=float(t),
                control={key: float(value) for key, value in control.items()},
                state=state.as_dict(),
            )
        )

    def save(self) -> Path:
        with self.path.open("w", encoding="utf-8") as file_obj:
            file_obj.write(json.dumps({"type": "metadata", "data": self.metadata}) + "\n")
            for frame in self._frames:
                file_obj.write(
                    json.dumps(
                        {
                            "type": "frame",
                            "t": frame.t,
                            "control": frame.control,
                            "state": frame.state,
                        },
                        separators=(",", ":"),
                    )
                    + "\n"
                )
        return self.path


def load_recording(path: str | Path) -> tuple[dict[str, Any], list[SessionFrame]]:
    metadata: dict[str, Any] = {}
    frames: list[SessionFrame] = []
    with Path(path).open("r", encoding="utf-8") as file_obj:
        for line in file_obj:
            if not line.strip():
                continue
            item = json.loads(line)
            if item.get("type") == "metadata":
                metadata = dict(item.get("data") or {})
            elif item.get("type") == "frame":
                frames.append(
                    SessionFrame(
                        t=float(item["t"]),
                        control=dict(item.get("control") or {}),
                        state=dict(item.get("state") or {}),
                    )
                )
    return metadata, frames


def score_frames(frames: Iterable[SessionFrame], success_progress: float = 0.98) -> SessionScore:
    seq = list(frames)
    if not seq:
        return SessionScore(0.0, 0, False, 0.0, 0.0, 0.0, 0.0, 0.0, 0)

    progress = [_float(frame.state.get("path_progress")) for frame in seq]
    deviations = [_float(frame.state.get("path_deviation")) for frame in seq]
    contacts = [_float(frame.state.get("contact_force")) for frame in seq]
    risks = [_float(frame.state.get("risk_score")) for frame in seq]
    statuses = [str(frame.state.get("safety_status", "")) for frame in seq]
    duration = max(0.0, seq[-1].t - seq[0].t)
    completion = max(progress) if progress else 0.0

    return SessionScore(
        duration_s=duration,
        steps=len(seq),
        completed=completion >= success_progress,
        completion=completion,
        mean_deviation=sum(deviations) / len(deviations),
        max_deviation=max(deviations),
        max_contact_force=max(contacts),
        max_risk_score=max(risks),
        danger_events=sum(1 for status in statuses if status in {"DANGER_WARNING", "COLLISION_STOP"}),
    )


def score_recording(path: str | Path, success_progress: float = 0.98) -> SessionScore:
    _, frames = load_recording(path)
    return score_frames(frames, success_progress=success_progress)


def _float(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0
