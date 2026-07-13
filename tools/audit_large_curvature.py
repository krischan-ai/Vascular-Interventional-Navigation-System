from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))


@dataclass
class ScenarioMetrics:
    route_target: str | None
    steps: int = 0
    final_progress: float = 0.0
    max_progress: float = 0.0
    max_path_deviation_m: float = 0.0
    min_wall_distance_m: float | None = None
    max_contact_force: float = 0.0
    max_risk_score: float = 0.0
    stop_count: int = 0
    warning_count: int = 0
    max_slack_m: float | None = None
    max_breach_m: float | None = None
    min_feed_budget_m: float | None = None
    final_safety_status: str = "STANDBY"
    final_phase: str | None = None
    elapsed_s: float = 0.0
    error: str | None = None
    diagnostics: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "route_target": self.route_target,
            "steps": self.steps,
            "final_progress": self.final_progress,
            "max_progress": self.max_progress,
            "max_path_deviation_m": self.max_path_deviation_m,
            "min_wall_distance_m": self.min_wall_distance_m,
            "max_contact_force": self.max_contact_force,
            "max_risk_score": self.max_risk_score,
            "stop_count": self.stop_count,
            "warning_count": self.warning_count,
            "max_slack_m": self.max_slack_m,
            "max_breach_m": self.max_breach_m,
            "min_feed_budget_m": self.min_feed_budget_m,
            "final_safety_status": self.final_safety_status,
            "final_phase": self.final_phase,
            "elapsed_s": self.elapsed_s,
            "error": self.error,
            "diagnostics": self.diagnostics,
        }


def update_metrics(metrics: ScenarioMetrics, state, diagnostics: dict[str, Any] | None = None) -> None:
    """Accumulate one simulated state into a scenario metrics object."""
    metrics.final_progress = float(state.path_progress)
    metrics.max_progress = max(metrics.max_progress, float(state.path_progress))
    metrics.max_path_deviation_m = max(metrics.max_path_deviation_m, float(state.path_deviation))
    wall_distance = float(state.wall_distance)
    metrics.min_wall_distance_m = (
        wall_distance
        if metrics.min_wall_distance_m is None
        else min(metrics.min_wall_distance_m, wall_distance)
    )
    metrics.max_contact_force = max(metrics.max_contact_force, float(state.contact_force))
    metrics.max_risk_score = max(metrics.max_risk_score, float(state.risk_score))
    metrics.final_safety_status = str(state.safety_status)
    if state.safety_status == "COLLISION_STOP":
        metrics.stop_count += 1
    elif state.safety_status == "DANGER_WARNING":
        metrics.warning_count += 1
    flow = getattr(state, "flow_guidance", {}) or {}
    workflow = flow.get("workflow", {}) if isinstance(flow, dict) else {}
    if isinstance(workflow, dict):
        metrics.final_phase = workflow.get("phase")

    if diagnostics:
        metrics.diagnostics = dict(diagnostics)
        _max_optional(metrics, "max_slack_m", diagnostics.get("slack_m"))
        _max_optional(metrics, "max_breach_m", diagnostics.get("max_breach_m"))
        _min_optional(metrics, "min_feed_budget_m", diagnostics.get("feed_budget_m"))


def verdict(metrics: ScenarioMetrics, *, min_progress: float, max_stop_count: int = 0) -> dict[str, Any]:
    """Return a simple pass/fail verdict for a large-curvature scenario."""
    failed_reasons: list[str] = []
    if metrics.error:
        failed_reasons.append("scenario_error")
    if metrics.max_progress < min_progress:
        failed_reasons.append("insufficient_progress")
    if metrics.stop_count > max_stop_count:
        failed_reasons.append("collision_stop")
    if metrics.max_breach_m is not None and metrics.max_breach_m > 0.0003:
        failed_reasons.append("breach_over_0.3mm")
    return {
        "passed": not failed_reasons,
        "failed_reasons": failed_reasons,
        "min_progress": min_progress,
        "max_stop_count": max_stop_count,
    }


def run_scenario(args: argparse.Namespace, route_target: str | None) -> ScenarioMetrics:
    from services.navigation_engine import NavigationEngine

    metrics = ScenarioMetrics(route_target=route_target)
    t0 = time.perf_counter()
    engine = None
    try:
        engine = NavigationEngine(
            phantom=args.phantom,
            target=args.target,
            guided=False,
            physics_engine=args.physics_engine,
            route_target=route_target,
            n_bodies=args.n_bodies,
            n_substeps=args.n_substeps,
            insertion_max=args.insertion_max,
            prethread=args.prethread,
        )
        if route_target is not None and route_target not in engine.available_routes:
            raise ValueError(
                f"unknown route target {route_target!r}; "
                f"available={sorted(engine.available_routes)[:20]}"
            )
        state = engine.reset()
        backend = getattr(engine, "_engine", None)
        update_metrics(metrics, state, _diagnostics(backend))

        autopilot = None
        if args.autopilot:
            from services.physics_autopilot import AutopilotConfig, PhysicsAutopilot

            autopilot = PhysicsAutopilot(
                engine.planned_path,
                config=AutopilotConfig(base_push=args.base_push, lookahead_m=args.lookahead),
            )

        for _ in range(args.steps):
            if autopilot is not None:
                push, rotate = autopilot.compute(
                    state.tip_position,
                    state.tip_direction,
                    state.contact_force,
                )
            else:
                push, rotate = args.push, args.rotate
            state = engine.step(push, rotate)
            metrics.steps += 1
            update_metrics(metrics, state, _diagnostics(backend))

    except Exception as exc:  # noqa: BLE001 - report scenario availability/failure
        metrics.error = f"{type(exc).__name__}: {exc}"
    finally:
        metrics.elapsed_s = time.perf_counter() - t0
        if engine is not None:
            engine.close()
    return metrics


def build_report(args: argparse.Namespace, scenario_metrics: list[ScenarioMetrics]) -> dict[str, Any]:
    scenarios = []
    for metrics in scenario_metrics:
        data = metrics.as_dict()
        data["verdict"] = verdict(metrics, min_progress=args.min_progress)
        scenarios.append(data)
    return {
        "schema_version": 1,
        "kind": "large_curvature_audit",
        "phantom": args.phantom,
        "target": args.target,
        "physics_engine": args.physics_engine,
        "autopilot": args.autopilot,
        "controls": {
            "steps": args.steps,
            "push": args.push,
            "rotate": args.rotate,
            "base_push": args.base_push,
            "lookahead_m": args.lookahead,
        },
        "thresholds": {
            "min_progress": args.min_progress,
            "max_stop_count": 0,
            "max_breach_m": 0.0003,
        },
        "scenarios": scenarios,
        "summary": {
            "scenario_count": len(scenarios),
            "passed_count": sum(1 for item in scenarios if item["verdict"]["passed"]),
            "failed_count": sum(1 for item in scenarios if not item["verdict"]["passed"]),
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run large-curvature Newton scenario audits.")
    parser.add_argument("--phantom", default="aorta_tree")
    parser.add_argument("--target", default="root")
    parser.add_argument("--physics-engine", default="newton_demo")
    parser.add_argument("--routes", nargs="*", default=["endpoint_18", "endpoint_21", "endpoint_15"])
    parser.add_argument("--steps", type=int, default=240)
    parser.add_argument("--push", type=float, default=0.7)
    parser.add_argument("--rotate", type=float, default=0.0)
    parser.add_argument("--autopilot", action="store_true")
    parser.add_argument("--base-push", type=float, default=0.35)
    parser.add_argument("--lookahead", type=float, default=0.025)
    parser.add_argument("--n-bodies", type=int, default=80)
    parser.add_argument("--n-substeps", type=int, default=None)
    parser.add_argument("--insertion-max", type=float, default=0.2)
    parser.add_argument("--prethread", action="store_true")
    parser.add_argument("--min-progress", type=float, default=0.25)
    parser.add_argument("--output", type=Path, default=Path("data") / "large_curvature_audit.json")
    return parser.parse_args()


def main() -> int:
    os.environ.setdefault("MUJOCO_GL", "glfw")
    args = parse_args()
    metrics = [run_scenario(args, route) for route in args.routes]
    report = build_report(args, metrics)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"Wrote large-curvature audit: {args.output}")
    print(json.dumps(report["summary"], ensure_ascii=False, indent=2))
    return 0 if report["summary"]["failed_count"] == 0 else 1


def _diagnostics(backend) -> dict[str, Any]:
    if backend is not None and hasattr(backend, "diagnostics"):
        return backend.diagnostics()
    return {}


def _max_optional(metrics: ScenarioMetrics, attr: str, value: Any) -> None:
    try:
        val = float(value)
    except (TypeError, ValueError):
        return
    current = getattr(metrics, attr)
    setattr(metrics, attr, val if current is None else max(current, val))


def _min_optional(metrics: ScenarioMetrics, attr: str, value: Any) -> None:
    try:
        val = float(value)
    except (TypeError, ValueError):
        return
    current = getattr(metrics, attr)
    setattr(metrics, attr, val if current is None else min(current, val))


if __name__ == "__main__":
    raise SystemExit(main())
